"""Task allocation: which robot carries which parcel.

Two policies are compared in the experiments:

* ``fixed_order`` (baseline): idle robots, in id order, take the waiting parcel
  with the lowest id.
* ``nearest``: idle robots, in id order, take the waiting parcel closest to them
  (Manhattan distance; ties -> lower parcel id).

Allocation uses Manhattan distance on purpose so that the search statistics
(nodes expanded, search time) only measure the agents' navigation searches.
A parcel can only be assigned when it is WAITING, so two robots can never be
assigned the same parcel; parcels unreachable from a robot's current
connected region are skipped.
"""

from __future__ import annotations

from environment.grid import manhattan
from environment.parcel import Parcel
from environment.robot import Robot, RobotState
from environment.warehouse import Warehouse


class TaskAllocator:
    def __init__(self, warehouse: Warehouse, policy: str = "nearest") -> None:
        if policy not in ("nearest", "fixed_order"):
            raise ValueError(f"Unknown allocation policy {policy!r}")
        self.warehouse = warehouse
        self.policy = policy
        # Static connectivity: cells reachable on the shelf map from each start.
        self._reach_cache: dict[tuple[int, int], frozenset] = {}

    def _reachable(self, robot: Robot) -> frozenset:
        key = robot.position
        for cells in self._reach_cache.values():
            if key in cells:
                return cells
        cells = frozenset(self.warehouse.grid.reachable_from(key))
        self._reach_cache[key] = cells
        return cells

    def choose(self, robot: Robot, candidates: list[Parcel]) -> Parcel | None:
        reach = self._reachable(robot)
        stations = self.warehouse.stations
        usable = [
            p for p in candidates
            if p.position in reach and stations[p.destination_id].position in reach
        ]
        if not usable:
            return None
        if self.policy == "fixed_order":
            return min(usable, key=lambda p: p.parcel_id)
        return min(usable, key=lambda p: (manhattan(robot.position, p.position), p.parcel_id))

    def allocate(self) -> list[tuple[int, int]]:
        """Assign waiting parcels to idle robots. Returns ``(robot_id, parcel_id)`` pairs."""
        assignments: list[tuple[int, int]] = []
        for robot in sorted(self.warehouse.robots, key=lambda r: r.robot_id):
            if robot.state != RobotState.IDLE or robot.has_task:
                continue
            parcel = self.choose(robot, self.warehouse.waiting_parcels())
            if parcel is None:
                continue
            self.warehouse.assign(robot, parcel)
            assignments.append((robot.robot_id, parcel.parcel_id))
        return assignments
