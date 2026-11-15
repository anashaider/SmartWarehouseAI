"""Deterministic multi-robot conflict avoidance.

Strategy (documented limitations in ``docs/ARCHITECTURE.md``):

1. **Turn ordering.** Robots act one after another in a deterministic priority
   order that rotates every tick (tick t starts with robot ``t mod n``) so no
   robot is permanently disadvantaged.
2. **Validation.** Every proposed move is checked against the static map
   (invalid move), cells occupied *now* by robots that have not vacated them
   (vertex conflict), cells already reserved this tick (vertex conflict) and the
   reverse edge reserved this tick (swap conflict). A refused move becomes WAIT.
3. **Deadlock breaking.** A robot blocked for ``DEADLOCK_THRESHOLD`` consecutive
   ticks, or an idle robot that is sitting on another robot's route, is told
   to *yield*: it side-steps to a free neighbour that is not reserved.

This guarantees that two robots never share a cell and never swap through
each other. It does not compute globally optimal multi-agent plans.
"""

from __future__ import annotations

import random
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
    def yield_candidates(self, threshold: int, blocked_on: dict[int, int]) -> list[int]:
        """Robots that should side-step this tick.

        * idle robots (no task) that are blocking a robot with a task;
        * the highest-id robot among those blocked for ``threshold`` ticks.
          Only one deadlocked robot yields per tick so pairs do not both move.
        """
        robots = {r.robot_id: r for r in self.warehouse.robots}
        out: list[int] = []
        for waiter, blocker in sorted(blocked_on.items()):
            b = robots.get(blocker)
            if b is not None and not b.has_task and blocker not in out:
                out.append(blocker)
        stuck = [r.robot_id for r in self.warehouse.robots
                 if r.blocked_streak >= threshold and r.robot_id not in out]
        if stuck:
            out.append(max(stuck))
        return out

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
