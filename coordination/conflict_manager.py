"""Deterministic multi-robot conflict avoidance.

Strategy (documented limitations in ``docs/ARCHITECTURE.md``):

1. **Turn ordering.** Robots act one after another in a deterministic priority
   order that rotates every tick (tick t starts with robot ``t mod n``) so no
   robot is permanently disadvantaged.
2. **Validation.** Every proposed move is checked against the static map
   (invalid move), cells occupied *now* by robots that have not vacated them
   (vertex conflict), cells already reserved this tick (vertex conflict) and the
   reverse edge reserved this tick (swap conflict). A refused move becomes WAIT.
3. **Deadlock breaking (priority retreat).** When robots block each other for
   ``DEADLOCK_THRESHOLD`` consecutive ticks, or an idle robot sits on a working
   robot's route, the robot with the lower right of way *yields*: it follows a
   BFS path to the nearest refuge cell outside the other robot's route, holds
   there briefly, and then resumes its own agent. A random side-step is the
   fallback when no refuge exists.

This guarantees that two robots never share a cell and never swap through
each other. It does not compute globally optimal multi-agent plans.
"""

from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from coordination.reservations import ReservationTable
from environment.grid import MOVE_ACTIONS, Action, Position, apply_move
from environment.robot import Robot
from environment.warehouse import Warehouse


class MoveOutcome(str, Enum):
    OK = "ok"
    INVALID = "invalid"            # shelf / outside the grid
    VERTEX_CONFLICT = "vertex"     # cell occupied or reserved by another robot
    SWAP_CONFLICT = "swap"         # two robots exchanging cells


@dataclass
class MoveCheck:
    outcome: MoveOutcome
    destination: Position
    other_robot: Optional[int] = None

    @property
    def ok(self) -> bool:
        return self.outcome == MoveOutcome.OK


class ConflictManager:
    def __init__(self, warehouse: Warehouse, seed: int = 0) -> None:
        self.warehouse = warehouse
        self.reservations = ReservationTable()
        self.rng = random.Random(seed * 104729 + 17)
        self.total_conflicts = 0
        self.total_swaps_prevented = 0
        self.deadlocks_resolved = 0

    # ------------------------------------------------------------------ #
    def turn_order(self, step: int) -> list[int]:
        ids = sorted(r.robot_id for r in self.warehouse.robots)
        k = step % len(ids)
        return ids[k:] + ids[:k]

    def begin_tick(self) -> None:
        self.reservations.clear()

    def check_move(self, robot: Robot, action: Action) -> MoveCheck:
        """Validate ``action`` for ``robot`` given the current tick's reservations."""
        src = robot.position
        dst = apply_move(src, action)
        if not self.warehouse.grid.is_walkable(dst):
            return MoveCheck(MoveOutcome.INVALID, dst)
        swap = self.reservations.swap_conflict(src, dst, robot.robot_id)
        if swap is not None:
            return MoveCheck(MoveOutcome.SWAP_CONFLICT, dst, swap)
        holder = self.reservations.holder(dst)
        if holder is not None and holder != robot.robot_id:
            return MoveCheck(MoveOutcome.VERTEX_CONFLICT, dst, holder)
        occupant = self.warehouse.robot_at(dst)
        if occupant is not None and occupant.robot_id != robot.robot_id:
            return MoveCheck(MoveOutcome.VERTEX_CONFLICT, dst, occupant.robot_id)
        return MoveCheck(MoveOutcome.OK, dst)

    def commit(self, robot: Robot, src: Position, dst: Position) -> None:
        self.reservations.reserve(robot.robot_id, src, dst)

    def record_conflict(self, check: MoveCheck) -> None:
        self.total_conflicts += 1
        if check.outcome == MoveOutcome.SWAP_CONFLICT:
            self.total_swaps_prevented += 1

    # ------------------------------------------------------------------ #
    # Deadlock / blocking resolution
    # ------------------------------------------------------------------ #
    def choose_yields(self, threshold: int, blocked_on: dict[int, int]) -> list[tuple[int, int]]:
        """Decide which robots must retreat this tick.

        Returns ``(yielder_id, beneficiary_id)`` pairs. ``blocked_on`` maps a
        robot to the robot that blocked it during the previous tick.

        * An idle robot (no task) that blocks a working robot yields at once.
        * Two robots blocked for ``threshold`` ticks where the blocker is also
          stuck (mutual / chained deadlock): the robot with the lower right of
          way (:attr:`Robot.priority`) yields.
        * A robot blocked for ``3 * threshold`` ticks by a robot that is not
          itself stuck yields as a last resort.
        """
        robots = {r.robot_id: r for r in self.warehouse.robots}
        pairs: list[tuple[int, int]] = []
        busy: set[int] = {r.robot_id for r in robots.values() if r.is_yielding}
        for rid, bid in sorted(blocked_on.items()):
            r, b = robots.get(rid), robots.get(bid)
            if r is None or b is None or rid in busy or bid in busy:
                continue
            if not b.has_task:
                pair = (bid, rid)
            elif r.blocked_streak >= threshold and (
                blocked_on.get(bid) is not None or b.blocked_streak >= threshold
            ):
                pair = (rid, bid) if r.priority > b.priority else (bid, rid)
            elif r.blocked_streak >= 3 * threshold:
                pair = (rid, bid)
            else:
                continue
            pairs.append(pair)
            busy.update(pair)
        return pairs

    def refuge_path(self, yielder: Robot, route: set[Position]) -> list[Position]:
        """Shortest path (BFS) from ``yielder`` to the nearest free cell outside
        ``route`` - the cells the other robot needs. Other robots are obstacles.
        Returns the cells to traverse (excluding the start); empty if none exists
        or the yielder is already off the route."""
        if yielder.position not in route:
            return []
        grid = self.warehouse.grid
        occupied = self.warehouse.occupied_cells(exclude=yielder.robot_id)
        parent: dict[Position, Optional[Position]] = {yielder.position: None}
        queue = deque([yielder.position])
        while queue:
            cur = queue.popleft()
            if cur not in route:
                path = [cur]
                while parent[path[-1]] is not None:
                    path.append(parent[path[-1]])  # type: ignore[arg-type]
                path.reverse()
                return path[1:]
            for nxt in grid.neighbors(cur):
                if nxt not in parent and nxt not in occupied:
                    parent[nxt] = cur
                    queue.append(nxt)
        return []

    def yield_move(self, robot: Robot, avoid: set[Position]) -> Optional[Action]:
        """Random (seeded) free neighbouring move, preferring cells not in ``avoid``."""
        options: list[Action] = []
        fallback: list[Action] = []
        for action in MOVE_ACTIONS:
            check = self.check_move(robot, action)
            if not check.ok:
                continue
            (fallback if check.destination in avoid else options).append(action)
        pool = options or fallback
        return self.rng.choice(pool) if pool else None
