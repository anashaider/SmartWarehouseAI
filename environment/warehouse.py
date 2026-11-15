"""Runtime warehouse state: grid + stations + parcels + robots.

The warehouse enforces the physical invariants of the world:

* robots never stand on shelves or outside the grid,
* two robots never share a cell,
* a parcel is assigned to / carried by at most one robot,
* a delivered parcel never becomes available again.

Higher-level decisions (who moves first, what to do on a conflict) belong to
the coordination layer and the simulation engine.
"""

from __future__ import annotations

from typing import Iterable, Optional

from environment.grid import Grid, Position
from environment.parcel import DeliveryStation, Parcel, ParcelStatus
from environment.robot import DeliveryRecord, Robot, RobotState
from environment.scenario_generator import Scenario


class WarehouseInvariantError(RuntimeError):
    """Raised if an operation would break a physical invariant."""


class Warehouse:
    def __init__(
        self,
        grid: Grid,
        stations: list[DeliveryStation],
        parcels: list[Parcel],
        robots: list[Robot],
    ) -> None:
        self.grid = grid
        self.stations: dict[int, DeliveryStation] = {s.station_id: s for s in stations}
        self.parcels: dict[int, Parcel] = {p.parcel_id: p for p in parcels}
        self.robots: list[Robot] = robots
        self._check_initial_state()

    @classmethod
    def from_scenario(cls, scenario: Scenario, num_robots: int) -> "Warehouse":
        """Create fresh runtime state from a scenario (always identical for a scenario)."""
        if num_robots < 1 or num_robots > len(scenario.robot_starts):
            raise ValueError(
                f"Scenario provides {len(scenario.robot_starts)} start cells; "
                f"{num_robots} robots requested"
            )
        stations = [DeliveryStation(i, pos) for i, pos in enumerate(scenario.stations)]
        parcels = [
            Parcel(i, spec.position, spec.destination_id)
            for i, spec in enumerate(scenario.parcels)
        ]
        robots = [Robot(i, scenario.robot_starts[i]) for i in range(num_robots)]
        return cls(scenario.grid.copy(), stations, parcels, robots)

    def _check_initial_state(self) -> None:
        cells = [r.position for r in self.robots]
        if len(set(cells)) != len(cells):
            raise WarehouseInvariantError("Robots must start on distinct cells")
        for robot in self.robots:
            if not self.grid.is_walkable(robot.position):
                raise WarehouseInvariantError(f"{robot.label} starts inside an obstacle")
        for parcel in self.parcels.values():
            if not self.grid.is_walkable(parcel.position):
                raise WarehouseInvariantError(f"{parcel.label} placed inside an obstacle")
            if parcel.destination_id not in self.stations:
                raise WarehouseInvariantError(f"{parcel.label} has no valid destination")

    # ------------------------------------------------------------------ #
    # Queries
    # ------------------------------------------------------------------ #
    def robot_at(self, pos: Position) -> Optional[Robot]:
        for robot in self.robots:
            if robot.position == pos:
                return robot
        return None

    def occupied_cells(self, exclude: Optional[int] = None) -> set[Position]:
        return {r.position for r in self.robots if r.robot_id != exclude}

    def station_cells(self) -> set[Position]:
        return {s.position for s in self.stations.values()}

    def station_for(self, parcel: Parcel) -> DeliveryStation:
        return self.stations[parcel.destination_id]

    def parcel_target(self, robot: Robot) -> Optional[Position]:
        """Current navigation target for ``robot`` (parcel cell or station cell)."""
        if robot.carrying is not None:
            return self.station_for(self.parcels[robot.carrying]).position
        if robot.assigned_parcel is not None:
            return self.parcels[robot.assigned_parcel].position
        return None

    def waiting_parcels(self) -> list[Parcel]:
        return [p for p in self.parcels.values() if p.status == ParcelStatus.WAITING]

    def pending_parcels(self) -> list[Parcel]:
        return [p for p in self.parcels.values() if p.status != ParcelStatus.DELIVERED]

    def delivered_count(self) -> int:
        return sum(1 for p in self.parcels.values() if p.status == ParcelStatus.DELIVERED)

    def all_delivered(self) -> bool:
        return self.delivered_count() == len(self.parcels)

    # ------------------------------------------------------------------ #
    # Mutations
    # ------------------------------------------------------------------ #
    def assign(self, robot: Robot, parcel: Parcel) -> None:
        """Assign ``parcel`` to an idle ``robot`` (one parcel <-> one robot)."""
        if robot.state != RobotState.IDLE or robot.has_task:
            raise WarehouseInvariantError(f"{robot.label} already has a task")
        parcel.assign(robot.robot_id)  # raises if the parcel is not WAITING
        robot.assigned_parcel = parcel.parcel_id
        robot.transition(RobotState.NAVIGATING_TO_PARCEL)
        robot.destination = parcel.position
        self.update_arrival_state(robot)

    def unassign(self, robot: Robot) -> None:
        """Cancel a not-yet-picked-up assignment (e.g. unreachable parcel)."""
        if robot.assigned_parcel is None or robot.carrying is not None:
            raise WarehouseInvariantError(f"{robot.label} has nothing to unassign")
        self.parcels[robot.assigned_parcel].unassign()
        robot.assigned_parcel = None
        robot.destination = None
        robot.planned_path = []
        robot.transition(RobotState.IDLE)

    def move_robot(self, robot: Robot, new_pos: Position) -> None:
        """Move ``robot`` to an adjacent free cell (invariants are hard-checked)."""
        if abs(new_pos[0] - robot.position[0]) + abs(new_pos[1] - robot.position[1]) != 1:
            raise WarehouseInvariantError(f"{robot.label}: {new_pos} is not adjacent")
        if not self.grid.is_walkable(new_pos):
            raise WarehouseInvariantError(f"{robot.label}: {new_pos} is not walkable")
        other = self.robot_at(new_pos)
        if other is not None:
            raise WarehouseInvariantError(
                f"{robot.label} would collide with {other.label} at {new_pos}"
            )
        robot.position = new_pos
        robot.distance_travelled += 1
        self.update_arrival_state(robot)

    def update_arrival_state(self, robot: Robot) -> None:
        """Advance NAVIGATING_* -> PICKING_UP / DELIVERING when the target is reached."""
        target = self.parcel_target(robot)
        if target is None or robot.position != target:
            return
        if robot.state == RobotState.NAVIGATING_TO_PARCEL:
            robot.transition(RobotState.PICKING_UP)
        elif robot.state == RobotState.NAVIGATING_TO_DROPOFF:
            robot.transition(RobotState.DELIVERING)

    def can_pick_up(self, robot: Robot) -> bool:
        if robot.state != RobotState.PICKING_UP or robot.assigned_parcel is None:
            return False
        parcel = self.parcels[robot.assigned_parcel]
        return (
            parcel.status == ParcelStatus.ASSIGNED
            and parcel.assigned_robot == robot.robot_id
            and parcel.position == robot.position
        )

    def pick_up(self, robot: Robot, step: int) -> bool:
        """Execute PICKUP. Returns False (invalid action) when not possible."""
        if not self.can_pick_up(robot):
            return False
        parcel = self.parcels[robot.assigned_parcel]  # type: ignore[index]
        parcel.pick_up(robot.robot_id, step)
        robot.carrying = parcel.parcel_id
        robot.transition(RobotState.NAVIGATING_TO_DROPOFF)
        robot.destination = self.station_for(parcel).position
        robot.planned_path = []
        self.update_arrival_state(robot)
        return True

    def can_deliver(self, robot: Robot) -> bool:
        if robot.state != RobotState.DELIVERING or robot.carrying is None:
            return False
        parcel = self.parcels[robot.carrying]
        return self.station_for(parcel).position == robot.position

    def deliver(self, robot: Robot, step: int) -> bool:
        """Execute DELIVER. Returns False (invalid action) when not possible."""
        if not self.can_deliver(robot):
            return False
        parcel = self.parcels[robot.carrying]  # type: ignore[index]
        station = self.station_for(parcel)
        parcel.deliver(robot.robot_id, station.position, step)
        robot.delivery_history.append(
            DeliveryRecord(parcel.parcel_id, station.station_id, parcel.picked_step or step, step)
        )
        robot.deliveries_completed += 1
        robot.carrying = None
        robot.assigned_parcel = None
        robot.destination = None
        robot.planned_path = []
        robot.transition(RobotState.IDLE)
        return True

    def check_invariants(self) -> None:
        """Assert every physical invariant (used by the engine and the tests)."""
        cells = [r.position for r in self.robots]
        if len(set(cells)) != len(cells):
            raise WarehouseInvariantError(f"Robots share a cell: {cells}")
        for robot in self.robots:
            if not self.grid.is_walkable(robot.position):
                raise WarehouseInvariantError(f"{robot.label} is inside an obstacle")
        holders: dict[int, int] = {}
        for robot in self.robots:
            for pid in {robot.assigned_parcel, robot.carrying} - {None}:
                if pid in holders and holders[pid] != robot.robot_id:
                    raise WarehouseInvariantError(f"Parcel {pid} held by two robots")
                holders[pid] = robot.robot_id  # type: ignore[index]

    def iter_robots(self, order: Iterable[int]) -> list[Robot]:
        by_id = {r.robot_id: r for r in self.robots}
        return [by_id[i] for i in order]
