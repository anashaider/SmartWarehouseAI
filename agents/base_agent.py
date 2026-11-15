"""Agent interface, percepts and explainable decisions."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

from algorithms.common import SearchResult
from algorithms.pathfinder import Pathfinder
from environment.grid import Action, Grid, MOVE_ACTIONS, Position, apply_move
from environment.robot import RobotState

# Neighbour statuses that may appear in a percept.
FREE, SHELF, WALL, ROBOT = "free", "shelf", "wall", "robot"


@dataclass(frozen=True)
class Percept:
    """What a robot senses at the beginning of its turn (PEAS sensors)."""

    robot_id: int
    step: int
    position: Position
    state: RobotState
    carrying: Optional[int]
    target: Optional[Position]
    target_kind: Optional[str]          # "parcel" | "station" | None
    target_label: str
    heading: Action
    neighbors: dict[Action, str]        # status of the 4 adjacent cells
    neighbor_robot: dict[Action, int]   # robot id occupying an adjacent cell
    other_robots: dict[int, Position]
    blocked_streak: int

    def summary(self) -> dict[str, str]:
        """Human readable percept for the explainability panel."""
        near = ", ".join(f"R{i}@{p}" for i, p in sorted(self.other_robots.items())) or "none"
        return {
            "Position": str(self.position),
            "State": self.state.label,
            "Carrying": f"P{self.carrying}" if self.carrying is not None else "nothing",
            "Target": f"{self.target_label} {self.target}" if self.target else "none",
            "Neighbours": "  ".join(
                f"{a.name[0]}:{s}" for a, s in self.neighbors.items()
            ),
            "Other robots": near,
            "Blocked streak": str(self.blocked_streak),
        }


@dataclass
class CandidateAction:
    """One action considered by the agent (for the decision inspector)."""

    action: Action
    valid: bool
    score: Optional[float] = None
    note: str = ""


@dataclass
class Decision:
    action: Action
    reason: str
    candidates: list[CandidateAction] = field(default_factory=list)
    plan: list[Position] = field(default_factory=list)
    search: Optional[SearchResult] = None
    features: dict[str, dict[str, float]] = field(default_factory=dict)


def build_neighbor_candidates(percept: Percept) -> list[CandidateAction]:
    """Basic list of move candidates with validity from the percept."""
    out = []
    for action in MOVE_ACTIONS:
        status = percept.neighbors[action]
        out.append(CandidateAction(action, status == FREE, note=status))
    return out


class BaseAgent(ABC):
    """Common base for all agents. One agent instance controls one robot."""

    agent_type: str = "base"
    display_name: str = "Agent"
    uses_search: bool = False

    def __init__(self, robot_id: int, grid: Grid, pathfinder: Pathfinder, seed: int = 0) -> None:
        self.robot_id = robot_id
        self.grid = grid
        self.pathfinder = pathfinder
        self.seed = seed

    @abstractmethod
    def decide(self, percept: Percept) -> Decision:
        """Return the action to perform this tick together with an explanation."""

    def task_rule(self, percept: Percept) -> Optional[Decision]:
        """Rules shared by every agent: pick up / deliver when standing on the target."""
        if percept.state == RobotState.PICKING_UP:
            return Decision(
                Action.PICKUP,
                f"Parcel {percept.target_label} detected at current position. "
                "Executing pickup action.",
                [CandidateAction(Action.PICKUP, True, note="on assigned parcel")],
            )
        if percept.state == RobotState.DELIVERING:
            return Decision(
                Action.DELIVER,
                f"Carrying P{percept.carrying} and standing on station "
                f"{percept.target_label}. Executing delivery.",
                [CandidateAction(Action.DELIVER, True, note="on destination station")],
            )
        if percept.target is None:
            return Decision(
                Action.WAIT,
                "No task assigned (all parcels taken or delivered). Waiting.",
                [CandidateAction(Action.WAIT, True, note="idle")],
            )
        return None

    def reset_task_memory(self) -> None:
        """Hook called when the robot's task changes."""

    @staticmethod
    def next_cell(position: Position, action: Action) -> Position:
        return apply_move(position, action)
