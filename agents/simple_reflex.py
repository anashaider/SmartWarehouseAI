"""Simple Reflex agent: condition-action rules over the current percept only.

Rules (checked in order):

1. On the assigned parcel  -> PICKUP.
2. Carrying and on the destination station -> DELIVER.
3. No task -> WAIT.
4. Greedy rule: move along the axis with the larger remaining offset to the
   target (then the other axis) if that adjacent cell is free.
5. Blocked rule: otherwise move to a random free neighbour, avoiding turning
   straight back (heading is part of the robot's own percept). Randomisation
   is seeded, so runs are reproducible; it is the classic way a memoryless
   reflex agent escapes infinite loops (Russell & Norvig, Sec. 2.4.2).
6. Nothing free -> WAIT.

No search and no long-term plan is used, so this agent can get trapped behind
shelf rows; that limitation is visible in the experiments by design.
"""

from __future__ import annotations

import random

from agents.base_agent import (
    FREE,
    BaseAgent,
    CandidateAction,
    Decision,
    Percept,
    build_neighbor_candidates,
)
from environment.grid import Action

OPPOSITE = {
    Action.UP: Action.DOWN,
    Action.DOWN: Action.UP,
    Action.LEFT: Action.RIGHT,
    Action.RIGHT: Action.LEFT,
}


class SimpleReflexAgent(BaseAgent):
    agent_type = "simple_reflex"
    display_name = "Simple Reflex"
    uses_search = False

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.rng = random.Random(self.seed * 7919 + self.robot_id)

    @staticmethod
    def preferred_moves(percept: Percept) -> list[Action]:
        """Moves that reduce the offset to the target, larger offset first."""
        assert percept.target is not None
        dr = percept.target[0] - percept.position[0]
        dc = percept.target[1] - percept.position[1]
        vertical = Action.DOWN if dr > 0 else Action.UP if dr < 0 else None
        horizontal = Action.RIGHT if dc > 0 else Action.LEFT if dc < 0 else None
        ordered = [vertical, horizontal] if abs(dr) >= abs(dc) else [horizontal, vertical]
        return [a for a in ordered if a is not None]

    def decide(self, percept: Percept) -> Decision:
        rule = self.task_rule(percept)
        if rule is not None:
            return rule

        candidates = build_neighbor_candidates(percept)
        preferred = self.preferred_moves(percept)
        for c in candidates:
            if c.action in preferred:
                c.note += " | reduces distance to target"

        for action in preferred:
            if percept.neighbors[action] == FREE:
                return Decision(
                    action,
                    f"Rule 4 (greedy): target {percept.target_label} lies "
                    f"{action.name.lower()}; adjacent cell is free, so moving "
                    f"{action.name.lower()}.",
                    candidates,
                )

        blocked_by = ", ".join(
            f"{a.name.lower()}={percept.neighbors[a]}" for a in preferred
        )
        free = [a for a in candidates if a.valid and a.action not in preferred]
        non_reverse = [c.action for c in free if c.action != OPPOSITE.get(percept.heading)]
        options = non_reverse or [c.action for c in free]
        if options:
            choice = self.rng.choice(options)
            return Decision(
                choice,
                f"Rule 5 (blocked): preferred direction blocked ({blocked_by}); "
                f"choosing free move {choice.name.lower()} at random "
                f"(options: {', '.join(o.name.lower() for o in options)}).",
                candidates,
            )
        candidates.append(CandidateAction(Action.WAIT, True, note="no free neighbour"))
        return Decision(
            Action.WAIT,
            "Rule 6: every adjacent cell is blocked. Waiting.",
            candidates,
        )
