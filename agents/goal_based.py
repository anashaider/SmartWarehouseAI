"""Goal-Based agent: plans a full route to its current goal and follows it.

Goal formulation: the current goal is the assigned parcel cell until the
parcel is picked up, then the parcel's destination station.

Planning: the selected search algorithm (BFS or A*) computes a shortest path
on the static shelf map. When the next cell of the plan is occupied by another
robot, the agent first waits (the other robot is usually moving away); if it
is still blocked it replans while treating every other robot as an obstacle.
If no alternative route exists it keeps waiting and the coordination layer's
deadlock breaker eventually intervenes.
"""

from __future__ import annotations

from typing import Optional

from agents.base_agent import (
    FREE,
    ROBOT,
    BaseAgent,
    CandidateAction,
    Decision,
    Percept,
    build_neighbor_candidates,
)
from algorithms.common import SearchResult
from config.settings import ALGORITHM_LABELS, REPLAN_AFTER_BLOCKED
from environment.grid import Action, Position, action_between, manhattan


class GoalBasedAgent(BaseAgent):
    agent_type = "goal_based"
    display_name = "Goal-Based"
    uses_search = True

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.plan: list[Position] = []       # remaining cells, plan[0] = next cell
        self.plan_target: Optional[Position] = None
        self.last_search: Optional[SearchResult] = None

    def reset_task_memory(self) -> None:
        self.plan = []
        self.plan_target = None

    def _alg(self) -> str:
        return ALGORITHM_LABELS[self.pathfinder.algorithm]

    def _plan(self, percept: Percept, avoid_robots: bool) -> SearchResult:
        blocked = percept.other_robots.values() if avoid_robots else ()
        result = self.pathfinder.find_path(percept.position, percept.target, blocked)  # type: ignore[arg-type]
        self.last_search = result
        self.plan_target = percept.target
        self.plan = result.path[1:] if result.found else []
        return result

    def decide(self, percept: Percept) -> Decision:
        rule = self.task_rule(percept)
        if rule is not None:
            self.reset_task_memory()
            return rule

        # Consume the part of the plan that has been executed.
        if self.plan and self.plan[0] == percept.position:
            self.plan.pop(0)

        reason_prefix = ""
        searched: Optional[SearchResult] = None
        plan_valid = (
            self.plan
            and self.plan_target == percept.target
            and manhattan(self.plan[0], percept.position) == 1
        )
        if not plan_valid:
            searched = self._plan(percept, avoid_robots=False)
            if not searched.found:
                searched = self._plan(percept, avoid_robots=True)
            if not searched.found:
                return Decision(
                    Action.WAIT,
                    f"Goal {percept.target_label} {percept.target} is unreachable "
                    f"({self._alg()} expanded {searched.nodes_expanded} nodes). Waiting.",
                    build_neighbor_candidates(percept) + [CandidateAction(Action.WAIT, True)],
                    search=searched,
                )
            reason_prefix = (
                f"Goal {percept.target_label} selected. {self._alg()} computed a "
                f"shortest route of {searched.cost} moves "
                f"({searched.nodes_expanded} nodes expanded). "
            )

        candidates = build_neighbor_candidates(percept)
        action = action_between(percept.position, self.plan[0])
        for c in candidates:
            if c.action == action:
                c.note += " | next step of plan"

        if percept.neighbors[action] == ROBOT:
            blocker = percept.neighbor_robot.get(action)
            if percept.blocked_streak >= REPLAN_AFTER_BLOCKED:
                searched = self._plan(percept, avoid_robots=True)
                if searched.found and percept.neighbors[action_between(percept.position, self.plan[0])] == FREE:
                    action = action_between(percept.position, self.plan[0])
                    return Decision(
                        action,
                        f"Route blocked by R{blocker}. Replanned around robots with "
                        f"{self._alg()}: new route {searched.cost} moves "
                        f"({searched.nodes_expanded} nodes expanded); moving "
                        f"{action.name.lower()}.",
                        candidates,
                        plan=[percept.position] + self.plan,
                        search=searched,
                    )
                # No detour: keep the static plan and wait for the blocker.
                self._plan(percept, avoid_robots=False)
                candidates.append(CandidateAction(Action.WAIT, True, note="blocked"))
                return Decision(
                    Action.WAIT,
                    f"Route blocked by R{blocker} and no detour exists. Waiting.",
                    candidates,
                    plan=[percept.position] + self.plan,
                    search=searched,
                )
            candidates.append(CandidateAction(Action.WAIT, True, note="blocked"))
            return Decision(
                Action.WAIT,
                f"Next cell {self.plan[0]} is occupied by R{blocker}. Waiting one "
                "step for it to move before replanning.",
                candidates,
                plan=[percept.position] + self.plan,
                search=searched,
            )

        return Decision(
            action,
            reason_prefix
            + f"Following plan toward {percept.target_label}: moving "
            f"{action.name.lower()} ({len(self.plan)} moves remaining).",
            candidates,
            plan=[percept.position] + self.plan,
            search=searched,
        )
