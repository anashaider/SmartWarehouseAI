"""Run watchdog: oscillation, repeated-state and stagnation detection.

The monitor is part of the *simulation environment*, not of any agent. It only
observes the world after each tick and never changes what an agent decides, so
it adds no memory or goal reasoning to the Simple Reflex agent and treats all
agents identically.

Signals
-------
* **Task progress** - a pickup, a delivery, or a robot reaching a cell that is
  closer (true shortest-path distance) to its current target than any cell it
  reached before on that same task.
* **Oscillation** - a robot with a target that, over the last
  ``window`` ticks, stayed within ``max_cells`` distinct cells without any
  progress of its own (e.g. bouncing between two cells next to a blocked goal).
* **Repeated state** - the whole world configuration (every robot's cell, load
  and task, plus the delivered count) recurring ``repeat_threshold`` times
  since the last progress.
* **Stagnation** - no task progress for ``stagnation_limit`` ticks. The engine
  then terminates the run honestly as INCOMPLETE.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Optional

from config.settings import (
    MIN_STAGNATION_LIMIT,
    OSCILLATION_MAX_CELLS,
    OSCILLATION_WINDOW,
    REPEATED_STATE_THRESHOLD,
    STAGNATION_PER_CELL,
)
from environment.grid import Position

if TYPE_CHECKING:
    from environment.scenario_generator import Scenario
    from environment.warehouse import Warehouse


def default_stagnation_limit(scenario: "Scenario") -> int:
    free = scenario.width * scenario.height - scenario.shelf_count
    return max(MIN_STAGNATION_LIMIT, int(STAGNATION_PER_CELL * free))


@dataclass
class WatchdogEvent:
    kind: str            # "oscillation" | "repeated_state"
    robot_id: int        # -1 for world-level events
    position: Position
    message: str


class ProgressMonitor:
    def __init__(
        self,
        stagnation_limit: int,
        window: int = OSCILLATION_WINDOW,
        max_cells: int = OSCILLATION_MAX_CELLS,
        repeat_threshold: int = REPEATED_STATE_THRESHOLD,
    ) -> None:
        self.stagnation_limit = stagnation_limit
        self.window = window
        self.max_cells = max_cells
        self.repeat_threshold = repeat_threshold
        self.reset()

    def reset(self) -> None:
        self.last_progress_step = 0
        self._last_step = 0
        self.oscillation_events = 0
        self.repeated_state_events = 0
        self.oscillating: set[int] = set()
        self._handled = -1
        self._best: dict[int, tuple[tuple, int]] = {}      # rid -> (task key, best dist)
        self._robot_progress: dict[int, int] = {}          # rid -> last progress step
        self._trail: dict[int, deque[Position]] = {}
        self._states: dict[tuple, int] = {}
        self._repeat_flagged = False

    # ------------------------------------------------------------------ #
    @property
    def ticks_without_progress(self) -> int:
        return self._last_step - self.last_progress_step

    @property
    def stagnated(self) -> bool:
        return self.stagnation_limit > 0 and self.ticks_without_progress >= self.stagnation_limit

    # ------------------------------------------------------------------ #
    def observe(
        self,
        step: int,
        warehouse: "Warehouse",
        distance: Callable[[Position, Position], Optional[int]],
    ) -> list[WatchdogEvent]:
        """Update the watchdog after tick ``step``. Returns new detections."""
        self._last_step = step
        events: list[WatchdogEvent] = []
        handled = warehouse.delivered_count() + sum(
            1 for p in warehouse.parcels.values() if p.picked_step is not None)
        progress = handled != self._handled
        self._handled = handled

        for robot in warehouse.robots:
            rid = robot.robot_id
            trail = self._trail.setdefault(rid, deque(maxlen=self.window))
            target = warehouse.parcel_target(robot)
            if target is None:
                trail.clear()
                self._best.pop(rid, None)
                self.oscillating.discard(rid)
                continue
            key = (robot.assigned_parcel, robot.carrying)
            d = distance(robot.position, target)
            prev = self._best.get(rid)
            if prev is None or prev[0] != key:
                self._best[rid] = (key, d if d is not None else 1 << 30)
                self._robot_progress[rid] = step
                trail.clear()
            elif d is not None and d < prev[1]:
                self._best[rid] = (key, d)
                self._robot_progress[rid] = step
                progress = True
            trail.append(robot.position)

            stuck = (len(trail) == self.window
                     and len(set(trail)) <= self.max_cells
                     and step - self._robot_progress.get(rid, step) >= self.window)
            if stuck and rid not in self.oscillating:
                self.oscillating.add(rid)
                self.oscillation_events += 1
                cells = ", ".join(str(c) for c in sorted(set(trail)))
                events.append(WatchdogEvent(
                    "oscillation", rid, robot.position,
                    f"R{rid} oscillating between {cells} for {self.window} ticks "
                    f"without getting closer to {target}"))
            elif not stuck:
                self.oscillating.discard(rid)

        if progress:
            self.last_progress_step = step
            self._states.clear()
            self._repeat_flagged = False
        else:
            sig = (warehouse.delivered_count(),) + tuple(
                (r.position, r.carrying, r.assigned_parcel) for r in warehouse.robots)
            count = self._states.get(sig, 0) + 1
            self._states[sig] = count
            if count >= self.repeat_threshold and not self._repeat_flagged:
                self._repeat_flagged = True
                self.repeated_state_events += 1
                events.append(WatchdogEvent(
                    "repeated_state", -1, warehouse.robots[0].position,
                    f"World state repeated {count}x with no task progress for "
                    f"{step - self.last_progress_step} ticks"))
        return events
