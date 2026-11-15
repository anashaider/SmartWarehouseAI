"""Parcels and delivery stations."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from environment.grid import Position


class ParcelStatus(str, Enum):
    WAITING = "waiting"        # on the shelf-side pickup cell, unassigned
    ASSIGNED = "assigned"      # reserved for exactly one robot
    CARRIED = "carried"        # on board a robot
    DELIVERED = "delivered"    # final state, never available again


class InvalidParcelTransition(RuntimeError):
    """Raised when a parcel is moved through an illegal life-cycle step."""


@dataclass(frozen=True)
class DeliveryStation:
    """Drop-off station. Stations are walkable floor cells."""

    station_id: int
    position: Position

    @property
    def label(self) -> str:
        return f"D{self.station_id}"


@dataclass
class Parcel:
    """A parcel that must be carried from ``position`` to station ``destination_id``."""

    parcel_id: int
    position: Position
    destination_id: int
    status: ParcelStatus = ParcelStatus.WAITING
    assigned_robot: Optional[int] = None
    carried_by: Optional[int] = None
    picked_step: Optional[int] = None
    delivered_step: Optional[int] = None
    origin: Position = field(init=False)

    def __post_init__(self) -> None:
        self.origin = self.position

    @property
    def label(self) -> str:
        return f"P{self.parcel_id}"

    @property
    def is_available(self) -> bool:
        return self.status == ParcelStatus.WAITING

    # ---------------------------- life cycle --------------------------- #
    def assign(self, robot_id: int) -> None:
        if self.status != ParcelStatus.WAITING:
            raise InvalidParcelTransition(
                f"{self.label} cannot be assigned while {self.status.value}"
            )
        self.status = ParcelStatus.ASSIGNED
        self.assigned_robot = robot_id

    def unassign(self) -> None:
        if self.status != ParcelStatus.ASSIGNED:
            raise InvalidParcelTransition(
                f"{self.label} cannot be unassigned while {self.status.value}"
            )
        self.status = ParcelStatus.WAITING
        self.assigned_robot = None

    def pick_up(self, robot_id: int, step: int) -> None:
        if self.status != ParcelStatus.ASSIGNED or self.assigned_robot != robot_id:
            raise InvalidParcelTransition(
                f"Robot {robot_id} cannot pick up {self.label} "
                f"(status={self.status.value}, assigned={self.assigned_robot})"
            )
        self.status = ParcelStatus.CARRIED
        self.carried_by = robot_id
        self.picked_step = step

    def deliver(self, robot_id: int, station_pos: Position, step: int) -> None:
        if self.status != ParcelStatus.CARRIED or self.carried_by != robot_id:
            raise InvalidParcelTransition(
                f"Robot {robot_id} cannot deliver {self.label} "
                f"(status={self.status.value})"
            )
        self.status = ParcelStatus.DELIVERED
        self.position = station_pos
        self.carried_by = None
        self.delivered_step = step
