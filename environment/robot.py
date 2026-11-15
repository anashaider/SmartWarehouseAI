"""Robot model and its finite-state machine.

State machine (enforced by :meth:`Robot.transition`)::

    IDLE -> NAVIGATING_TO_PARCEL -> PICKING_UP -> NAVIGATING_TO_DROPOFF
         -> DELIVERING -> IDLE

``NAVIGATING_TO_PARCEL -> IDLE`` is additionally allowed so that the task
allocator can cancel an assignment whose parcel turned out to be unreachable.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from environment.grid import Action, Position


class RobotState(str, Enum):
    IDLE = "IDLE"
    NAVIGATING_TO_PARCEL = "NAVIGATING_TO_PARCEL"
    PICKING_UP = "PICKING_UP"
    NAVIGATING_TO_DROPOFF = "NAVIGATING_TO_DROPOFF"
    DELIVERING = "DELIVERING"

    @property
    def label(self) -> str:
        return self.value.replace("_", " ").title()


ALLOWED_TRANSITIONS: dict[RobotState, frozenset[RobotState]] = {
    RobotState.IDLE: frozenset({RobotState.NAVIGATING_TO_PARCEL}),
    RobotState.NAVIGATING_TO_PARCEL: frozenset(
        {RobotState.PICKING_UP, RobotState.IDLE}
    ),
    RobotState.PICKING_UP: frozenset({RobotState.NAVIGATING_TO_DROPOFF}),
    RobotState.NAVIGATING_TO_DROPOFF: frozenset({RobotState.DELIVERING}),
    RobotState.DELIVERING: frozenset({RobotState.IDLE}),
}


class InvalidStateTransition(RuntimeError):
    """Raised for a transition not present in :data:`ALLOWED_TRANSITIONS`."""


@dataclass
class DeliveryRecord:
    parcel_id: int
    station_id: int
    picked_step: int
    delivered_step: int


@dataclass
class Robot:
    """A warehouse robot. Holds state only; behaviour lives in agents."""

    robot_id: int
    position: Position
    state: RobotState = RobotState.IDLE
    assigned_parcel: Optional[int] = None
    carrying: Optional[int] = None
    destination: Optional[Position] = None
    direction: Action = Action.DOWN
    current_action: Action = Action.WAIT
    distance_travelled: int = 0
    deliveries_completed: int = 0
    invalid_moves: int = 0
    conflicts: int = 0
    waits: int = 0
    blocked_streak: int = 0
    previous_position: Position = field(init=False)
    start_position: Position = field(init=False)
    delivery_history: list[DeliveryRecord] = field(default_factory=list)
    planned_path: list[Position] = field(default_factory=list)
    # Coordination-layer retreat ("yield") state.
    yield_path: list[Position] = field(default_factory=list)
    yield_hold: int = 0
    yield_to: Optional[int] = None

    def __post_init__(self) -> None:
        self.previous_position = self.position
        self.start_position = self.position

    @property
    def label(self) -> str:
        return f"R{self.robot_id}"

    @property
    def is_carrying(self) -> bool:
        return self.carrying is not None

    @property
    def has_task(self) -> bool:
        return self.assigned_parcel is not None

    @property
    def is_yielding(self) -> bool:
        return bool(self.yield_path) or self.yield_hold > 0

    @property
    def priority(self) -> tuple[int, int, int]:
        """Smaller tuple = higher right of way (carrying > has task > idle, then id)."""
        return (0 if self.is_carrying else 1, 0 if self.has_task else 1, self.robot_id)

    def can_transition(self, new_state: RobotState) -> bool:
        return new_state in ALLOWED_TRANSITIONS[self.state]

    def transition(self, new_state: RobotState) -> None:
        if not self.can_transition(new_state):
            raise InvalidStateTransition(
                f"{self.label}: {self.state.value} -> {new_state.value} is not allowed"
            )
        self.state = new_state
