"""Discrete-time simulation engine (no rendering, no wall-clock dependence).

One call to :meth:`SimulationEngine.step` advances the world by one tick:

1. Task allocation for idle robots.
2. Coordination layer chooses robots that must yield (deadlock breaking).
3. Robots act one by one in the rotating turn order. Each robot receives a
   percept, its agent returns a :class:`Decision`, and the conflict manager
   validates/executes the action.
4. Physical invariants are verified and metrics are updated.

The engine is used identically by the Pygame interface and by the headless
experiment runner, so everything shown on screen is real simulation output.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

from agents import create_agent
from agents.base_agent import FREE, ROBOT, SHELF, WALL, BaseAgent, CandidateAction, Decision, Percept
from algorithms.pathfinder import Pathfinder
from config.settings import (
    DEADLOCK_THRESHOLD,
    YIELD_HOLD_TICKS,
    DECISION_HISTORY_LIMIT,
    MIN_STEP_LIMIT,
    STEP_LIMIT_PER_CELL,
    SimulationConfig,
)
from coordination.conflict_manager import ConflictManager, MoveOutcome
from coordination.task_allocator import TaskAllocator
from environment.grid import MOVE_ACTIONS, Action, Position, action_between, apply_move, manhattan
from environment.robot import Robot
from environment.scenario_generator import Scenario
from environment.warehouse import Warehouse


@dataclass
class TickEvent:
    """Something noteworthy that happened during a tick (drives UI effects)."""

    step: int
    kind: str          # assign | pickup | deliver | conflict | invalid | yield | wait
    robot_id: int
    position: Position
    message: str


@dataclass
class DecisionRecord:
    step: int
    robot_id: int
    percept: dict[str, str]
    candidates: list[CandidateAction]
    chosen: Action
    executed: Action
    reason: str
    outcome: str
    features: dict[str, dict[str, float]] = field(default_factory=dict)


def default_step_limit(scenario: Scenario) -> int:
    free = scenario.width * scenario.height - scenario.shelf_count
    return max(MIN_STEP_LIMIT, int(STEP_LIMIT_PER_CELL * free))


class SimulationEngine:
    def __init__(
        self,
        scenario: Scenario,
        config: SimulationConfig,
        max_steps: Optional[int] = None,
        record_history: bool = True,
    ) -> None:
        config.validate()
        self.scenario = scenario
        self.config = config
        self.max_steps = max_steps or default_step_limit(scenario)
        self.record_history = record_history
        self.reset()

    # ------------------------------------------------------------------ #
    def reset(self) -> None:
        cfg = self.config
        self.warehouse = Warehouse.from_scenario(self.scenario, cfg.num_robots)
        self.allocator = TaskAllocator(self.warehouse, cfg.allocation)
        self.conflicts = ConflictManager(self.warehouse, seed=cfg.seed)
        self.agents: dict[int, BaseAgent] = {}
        for robot in self.warehouse.robots:
            pf = Pathfinder(self.warehouse.grid, cfg.algorithm)
            self.agents[robot.robot_id] = create_agent(
                cfg.agent_type, robot.robot_id, self.warehouse.grid, pf,
                seed=cfg.seed, weights=cfg.utility_weights,
            )
        self.step_count = 0
        self.completion_step: Optional[int] = None
        self.done = False
        self.termination_reason = ""
        self.compute_time_ms = 0.0
        self.deadlocks_resolved = 0
        self.events: list[TickEvent] = []          # events of the last tick
        self.event_log: deque[TickEvent] = deque(maxlen=400)
        self.history: dict[int, deque[DecisionRecord]] = {
            r.robot_id: deque(maxlen=DECISION_HISTORY_LIMIT) for r in self.warehouse.robots
        }
        self.last_decision: dict[int, DecisionRecord] = {}
        self._blocked_on: dict[int, int] = {}
        self.last_tick_order: list[int] = []
        self.rejected_moves = 0
        # Instrumentation only (not counted as agent search effort): exact
        # distance-to-target fields used to measure robot conflicts uniformly.
        self._dist_fields: dict[Position, dict[Position, int]] = {}

    # ------------------------------------------------------------------ #
    # Percepts
    # ------------------------------------------------------------------ #
    def build_percept(self, robot: Robot) -> Percept:
        wh = self.warehouse
        neighbors: dict[Action, str] = {}
        neighbor_robot: dict[Action, int] = {}
        for action in MOVE_ACTIONS:
            cell = apply_move(robot.position, action)
            if not wh.grid.in_bounds(cell):
                neighbors[action] = WALL
            elif wh.grid.is_shelf(cell):
                neighbors[action] = SHELF
            else:
                other = wh.robot_at(cell)
                if other is not None:
                    neighbors[action] = ROBOT
                    neighbor_robot[action] = other.robot_id
                else:
                    neighbors[action] = FREE
        target = wh.parcel_target(robot)
        if robot.carrying is not None:
            kind = "station"
            label = wh.station_for(wh.parcels[robot.carrying]).label
        elif robot.assigned_parcel is not None:
            kind, label = "parcel", wh.parcels[robot.assigned_parcel].label
        else:
            kind, label = None, ""
        return Percept(
            robot_id=robot.robot_id,
            step=self.step_count,
            position=robot.position,
            state=robot.state,
            carrying=robot.carrying,
            target=target,
            target_kind=kind,
            target_label=label,
            heading=robot.direction,
            neighbors=neighbors,
            neighbor_robot=neighbor_robot,
            other_robots={r.robot_id: r.position for r in wh.robots if r.robot_id != robot.robot_id},
            blocked_streak=robot.blocked_streak,
        )

    # ------------------------------------------------------------------ #
    # Tick
    # ------------------------------------------------------------------ #
    def step(self) -> list[TickEvent]:
        if self.done:
            return []
        t0 = time.perf_counter()
        self.step_count += 1
        step = self.step_count
        wh = self.warehouse
        events: list[TickEvent] = []

        for rid, pid in self.allocator.allocate():
            self.agents[rid].reset_task_memory()
            robot = wh.robots[rid]
            events.append(TickEvent(step, "assign", rid, robot.position,
                                    f"R{rid} assigned P{pid}"))

        cm = self.conflicts
        cm.begin_tick()
        order = cm.turn_order(step)
        self.last_tick_order = order
        self._plan_yields(events)
        self._blocked_on = {}

        for robot in wh.iter_robots(order):
            robot.previous_position = robot.position
            if robot.is_yielding:
                self._step_yield(robot, events)
                continue
            percept = self.build_percept(robot)
            decision = self.agents[robot.robot_id].decide(percept)
            self._execute(robot, percept, decision, events)
            self._measure_conflict(robot, percept, decision, events)

        wh.check_invariants()
        self.compute_time_ms += (time.perf_counter() - t0) * 1000
        self.events = events
        self.event_log.extend(events)
        self._check_termination()
        return events

    # ------------------------------------------------------------------ #
    # Deadlock handling (coordination override)
    # ------------------------------------------------------------------ #
    def route_cells(self, robot: Robot) -> set[Position]:
        """Cells ``robot`` needs: its current plan plus one static shortest route."""
        cells = {robot.position, *robot.planned_path}
        target = self.warehouse.parcel_target(robot)
        if target is None:
            return cells
        dist = self.distance_field(target)
        cur = robot.position
        while cur in dist and dist[cur] > 0:
            nxt = next((n for n in self.warehouse.grid.neighbors(cur)
                        if dist.get(n) == dist[cur] - 1), None)
            if nxt is None:
                break
            cells.add(nxt)
            cur = nxt
        return cells

    def _plan_yields(self, events: list[TickEvent]) -> None:
        robots = {r.robot_id: r for r in self.warehouse.robots}
        for yid, bid in self.conflicts.choose_yields(DEADLOCK_THRESHOLD, self._blocked_on):
            yielder, beneficiary = robots[yid], robots[bid]
            path = self.conflicts.refuge_path(yielder, self.route_cells(beneficiary))
            if not path:
                # The yielder is trapped (e.g. dead end): swap roles if possible.
                alt = self.conflicts.refuge_path(beneficiary, self.route_cells(yielder))
                if alt:
                    yielder, beneficiary, path = beneficiary, yielder, alt
            if not path:
                action = self.conflicts.yield_move(yielder, self.route_cells(beneficiary))
                if action is None:
                    continue
                path = [apply_move(yielder.position, action)]
            yielder.yield_path = path
            yielder.yield_hold = YIELD_HOLD_TICKS
            yielder.yield_to = beneficiary.robot_id
            yielder.planned_path = [yielder.position, *path]
            self.deadlocks_resolved += 1
            self.conflicts.deadlocks_resolved += 1
            self.agents[yielder.robot_id].reset_task_memory()
            events.append(TickEvent(
                self.step_count, "yield", yielder.robot_id, path[-1],
                f"R{yielder.robot_id} yields to R{beneficiary.robot_id}: "
                f"retreating {len(path)} cell(s) to {path[-1]}"))

    def _step_yield(self, robot: Robot, events: list[TickEvent]) -> None:
        cm = self.conflicts
        src = robot.position
        to = robot.yield_to
        if robot.yield_path:
            nxt = robot.yield_path[0]
            action = action_between(src, nxt)
            check = cm.check_move(robot, action)
            if check.ok:
                self.warehouse.move_robot(robot, nxt)
                cm.commit(robot, src, nxt)
                robot.yield_path.pop(0)
                robot.direction = action
                executed, outcome = action, "retreating"
            else:
                cm.commit(robot, src, src)
                robot.yield_path = []  # refuge route blocked: hold here instead
                executed, outcome = Action.WAIT, "retreat blocked; holding"
            reason = (f"Coordination override: deadlock with R{to} detected. R{to} has "
                      f"right of way, so this robot retreats to a refuge cell "
                      f"({len(robot.yield_path)} cell(s) left).")
        else:
            cm.commit(robot, src, src)
            robot.yield_hold -= 1
            robot.waits += 1
            executed, outcome = Action.WAIT, "holding at refuge"
            reason = (f"Coordination override: holding at refuge cell so R{to} can pass "
                      f"({robot.yield_hold} tick(s) left).")
        robot.planned_path = [robot.position, *robot.yield_path]
        robot.current_action = executed
        robot.blocked_streak = 0
        if not robot.is_yielding:
            robot.yield_to = None
            self.agents[robot.robot_id].reset_task_memory()
        self._record(robot, {"Override": "coordination layer (yield)"},
                     [CandidateAction(executed, True, note="yield")],
                     executed, executed, reason, outcome, {})

    def _execute(self, robot: Robot, percept: Percept, decision: Decision,
                 events: list[TickEvent]) -> None:
        wh, cm, step = self.warehouse, self.conflicts, self.step_count
        src = robot.position
        action = decision.action
        executed = action
        outcome = "ok"

        if action.is_move:
            check = cm.check_move(robot, action)
            if check.ok:
                wh.move_robot(robot, check.destination)
                cm.commit(robot, src, check.destination)
                robot.direction = action
                robot.blocked_streak = 0
                outcome = "moved"
            elif check.outcome == MoveOutcome.INVALID:
                robot.invalid_moves += 1
                cm.commit(robot, src, src)
                executed = Action.WAIT
                outcome = "invalid move (shelf/wall)"
                events.append(TickEvent(step, "invalid", robot.robot_id, check.destination,
                                        f"R{robot.robot_id} tried to enter a shelf/wall"))
            else:
                self.rejected_moves += 1
                robot.blocked_streak += 1
                cm.record_conflict(check)
                cm.commit(robot, src, src)
                executed = Action.WAIT
                if check.other_robot is not None:
                    self._blocked_on[robot.robot_id] = check.other_robot
                outcome = f"{check.outcome.value} conflict with R{check.other_robot}: waited"
                events.append(TickEvent(step, "conflict", robot.robot_id, check.destination,
                                        f"R{robot.robot_id} blocked by R{check.other_robot}"))
        elif action == Action.PICKUP:
            cm.commit(robot, src, src)
            if wh.pick_up(robot, step):
                self.agents[robot.robot_id].reset_task_memory()
                outcome = "picked up"
                events.append(TickEvent(step, "pickup", robot.robot_id, src,
                                        f"R{robot.robot_id} picked up P{robot.carrying}"))
            else:
                robot.invalid_moves += 1
                outcome = "invalid pickup"
        elif action == Action.DELIVER:
            cm.commit(robot, src, src)
            parcel_id = robot.carrying
            if wh.deliver(robot, step):
                self.agents[robot.robot_id].reset_task_memory()
                outcome = "delivered"
                events.append(TickEvent(step, "deliver", robot.robot_id, src,
                                        f"R{robot.robot_id} delivered P{parcel_id}"))
            else:
                robot.invalid_moves += 1
                outcome = "invalid delivery"
        else:  # WAIT
            cm.commit(robot, src, src)
            robot.waits += 1
            blocker = self._detect_blocker(robot, percept, decision)
            if blocker is not None:
                robot.blocked_streak += 1
                self._blocked_on[robot.robot_id] = blocker
                outcome = f"waiting for R{blocker}"
                events.append(TickEvent(step, "wait", robot.robot_id, src, outcome))
            else:
                robot.blocked_streak = 0
                outcome = "waiting"

        if executed == Action.WAIT and action != Action.WAIT:
            robot.waits += 1
        robot.current_action = executed
        robot.planned_path = list(decision.plan)
        self._record(robot, percept.summary(), decision.candidates, action, executed,
                     decision.reason, outcome, decision.features)

    def distance_field(self, target: Position) -> dict[Position, int]:
        """Exact shortest-path distance from every cell to ``target`` (cached)."""
        field_ = self._dist_fields.get(target)
        if field_ is None:
            field_ = {target: 0}
            frontier = deque([target])
            grid = self.warehouse.grid
            while frontier:
                cur = frontier.popleft()
                for nxt in grid.neighbors(cur):
                    if nxt not in field_:
                        field_[nxt] = field_[cur] + 1
                        frontier.append(nxt)
            self._dist_fields[target] = field_
        return field_

    def _measure_conflict(self, robot: Robot, percept: Percept, decision: Decision,
                          events: list[TickEvent]) -> None:
        """Count a robot conflict: the robot had a navigation target, did not get
        closer to it this tick, and another robot occupied a neighbouring cell
        that lies on one of its shortest routes (or its move was rejected)."""
        if percept.target is None or decision.action in (Action.PICKUP, Action.DELIVER):
            return
        dist = self.distance_field(percept.target)
        d_before = dist.get(percept.position)
        d_after = dist.get(robot.position)
        if d_before is None or d_after is None or d_after < d_before:
            return
        rejected = decision.action.is_move and robot.position == percept.position             and percept.neighbors.get(decision.action) == ROBOT
        on_route = any(
            dist.get(apply_move(percept.position, a)) == d_before - 1
            for a in percept.neighbor_robot
        )
        if rejected or on_route:
            robot.conflicts += 1
            if not rejected:
                events.append(TickEvent(self.step_count, "conflict", robot.robot_id,
                                        robot.position,
                                        f"R{robot.robot_id} route obstructed by another robot"))

    def _detect_blocker(self, robot: Robot, percept: Percept, decision: Decision) -> Optional[int]:
        """Robot whose cell prevents progress when ``robot`` chose to wait."""
        if percept.target is None:
            return None
        if len(decision.plan) > 1:
            other = self.warehouse.robot_at(decision.plan[1])
            if other is not None and other.robot_id != robot.robot_id:
                return other.robot_id
        d = manhattan(robot.position, percept.target)
        for action, rid in percept.neighbor_robot.items():
            if manhattan(apply_move(robot.position, action), percept.target) < d:
                return rid
        return None

    def _record(self, robot: Robot, percept: dict[str, str], candidates: list[CandidateAction],
                chosen: Action, executed: Action, reason: str, outcome: str,
                features: dict[str, dict[str, float]]) -> None:
        rec = DecisionRecord(self.step_count, robot.robot_id, percept, candidates,
                             chosen, executed, reason, outcome, features)
        self.last_decision[robot.robot_id] = rec
        if self.record_history:
            self.history[robot.robot_id].append(rec)

    def _check_termination(self) -> None:
        wh = self.warehouse
        if wh.all_delivered():
            self.done = True
            self.completion_step = self.step_count
            self.termination_reason = "all parcels delivered"
        elif self.step_count >= self.max_steps:
            self.done = True
            self.termination_reason = f"step limit ({self.max_steps}) reached"
        elif not any(r.has_task for r in wh.robots):
            # Nothing in progress and nothing assignable -> remaining parcels unreachable.
            if all(self.allocator.choose(r, wh.waiting_parcels()) is None for r in wh.robots):
                self.done = True
                self.termination_reason = "remaining parcels unreachable"

    # ------------------------------------------------------------------ #
    def run(self) -> "SimulationEngine":
        while not self.done:
            self.step()
        return self

    # ------------------------------------------------------------------ #
    # Metrics
    # ------------------------------------------------------------------ #
    def search_totals(self) -> tuple[int, int, float]:
        searches = sum(a.pathfinder.stats.searches for a in self.agents.values())
        nodes = sum(a.pathfinder.stats.nodes_expanded for a in self.agents.values())
        ms = sum(a.pathfinder.stats.runtime_ms for a in self.agents.values())
        return searches, nodes, ms

    def metrics(self) -> dict[str, float | int | str]:
        wh = self.warehouse
        total = len(wh.parcels)
        delivered = wh.delivered_count()
        searches, nodes, search_ms = self.search_totals()
        return {
            "parcels_total": total,
            "parcels_delivered": delivered,
            "parcels_pending": total - delivered,
            "completion_pct": round(100.0 * delivered / total, 2) if total else 100.0,
            "total_distance": sum(r.distance_travelled for r in wh.robots),
            "steps": self.step_count,
            "completion_steps": self.completion_step if self.completion_step is not None else self.step_count,
            "completed": self.completion_step is not None,
            "invalid_moves": sum(r.invalid_moves for r in wh.robots),
            "conflicts": sum(r.conflicts for r in wh.robots),
            "rejected_moves": self.rejected_moves,
            "waits": sum(r.waits for r in wh.robots),
            "deadlocks_resolved": self.deadlocks_resolved,
            "searches": searches,
            "nodes_expanded": nodes,
            "search_time_ms": round(search_ms, 4),
            "compute_time_ms": round(self.compute_time_ms, 4),
            "termination": self.termination_reason,
        }
