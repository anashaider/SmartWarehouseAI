"""Live warehouse intelligence derived from the real simulation (no GPU code).

:class:`FleetTracker` is attached to ``SimulationController.tick_listeners``
and samples the engine once per real tick (also when several ticks run in one
rendered frame). It never modifies the engine. It provides:

* a decaying **congestion heatmap**: robot occupancy per cell, with extra weight
  for robots that are waiting or yielding there and for conflict events;
* per-robot **utilisation**: share of ticks spent moving, handling a parcel
  (pickup/delivery) or yielding, versus waiting for another robot or idle;
* **fleet activity**: current counts per activity, throughput, mean delivery
  cycle time, per-station delivery progress and the current hotspot cell.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from environment.grid import Action, Position
from environment.parcel import ParcelStatus
from simulation.simulation_engine import SimulationEngine, TickEvent

ACTIVITIES = ("moving", "handling", "yielding", "waiting", "idle")
HEAT_DECAY = 0.96          # per tick (half-life ~17 ticks)
HEAT_OCCUPY = 1.0
HEAT_WAIT = 2.0
HEAT_CONFLICT = 3.0


def classify(robot) -> str:
    """Activity of a robot during the last tick, from its real executed action."""
    if robot.is_yielding:
        return "yielding"
    if robot.current_action.is_move:
        return "moving"
    if robot.current_action in (Action.PICKUP, Action.DELIVER):
        return "handling"
    if robot.has_task:
        return "waiting"
    return "idle"


@dataclass
class RobotUsage:
    counts: dict[str, int] = field(default_factory=lambda: {a: 0 for a in ACTIVITIES})
    current: str = "idle"

    @property
    def ticks(self) -> int:
        return sum(self.counts.values())

    @property
    def utilization(self) -> float:
        """Productive share: moving + handling + yielding (a retreat is useful work)."""
        t = self.ticks
        return (self.counts["moving"] + self.counts["handling"] + self.counts["yielding"]) / t if t else 0.0


class FleetTracker:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.heat: dict[Position, float] = {}
        self.conflict_cells: dict[Position, int] = {}
        self.usage: dict[int, RobotUsage] = {}
        self.delivery_steps: list[int] = []
        self.last_step = -1
        self.engine_id: Optional[int] = None
        self.version = 0          # increments every sampled tick (for cheap change detection)

    # ------------------------------------------------------------------ sampling
    def on_tick(self, engine: SimulationEngine, events: list[TickEvent]) -> None:
        if engine.step_count == self.last_step and self.engine_id == id(engine):
            return                                   # duplicate sample of the same tick
        if self.engine_id != id(engine) or engine.step_count < self.last_step:
            self.reset()                             # a new run (reset / new layout)
            self.engine_id = id(engine)
        self.last_step = engine.step_count
        for cell in list(self.heat):
            v = self.heat[cell] * HEAT_DECAY
            if v < 0.02:
                del self.heat[cell]
            else:
                self.heat[cell] = v
        for robot in engine.warehouse.robots:
            u = self.usage.setdefault(robot.robot_id, RobotUsage())
            act = classify(robot)
            u.current = act
            u.counts[act] += 1
            w = HEAT_OCCUPY + (HEAT_WAIT if act in ("waiting", "yielding") else 0.0)
            self.heat[robot.position] = self.heat.get(robot.position, 0.0) + w
        for ev in events:
            if ev.kind == "conflict":
                self.heat[ev.position] = self.heat.get(ev.position, 0.0) + HEAT_CONFLICT
                self.conflict_cells[ev.position] = self.conflict_cells.get(ev.position, 0) + 1
            elif ev.kind == "deliver":
                self.delivery_steps.append(ev.step)
        self.version += 1

    # ------------------------------------------------------------------ queries
    def max_heat(self) -> float:
        return max(self.heat.values(), default=0.0)

    def hotspot(self) -> Optional[tuple[Position, float]]:
        if not self.heat:
            return None
        cell = max(self.heat, key=lambda c: (self.heat[c], c))
        return cell, self.heat[cell]

    def activity_counts(self) -> dict[str, int]:
        out = {a: 0 for a in ACTIVITIES}
        for u in self.usage.values():
            out[u.current] += 1
        return out

    def utilization(self, robot_id: int) -> float:
        u = self.usage.get(robot_id)
        return u.utilization if u else 0.0

    def fleet_utilization(self) -> float:
        total = sum(u.ticks for u in self.usage.values())
        if not total:
            return 0.0
        busy = sum(u.utilization * u.ticks for u in self.usage.values())
        return busy / total

    def throughput(self, window: int = 100) -> float:
        """Deliveries in the last ``window`` ticks, scaled to deliveries / 100 ticks."""
        if self.last_step <= 0:
            return 0.0
        lo = self.last_step - window
        n = sum(1 for s in self.delivery_steps if s > lo)
        return 100.0 * n / min(window, self.last_step)

    @staticmethod
    def cycle_time(engine: SimulationEngine) -> Optional[float]:
        """Mean ticks from pickup to delivery over delivered parcels."""
        times = [p.delivered_step - p.picked_step for p in engine.warehouse.parcels.values()
                 if p.status == ParcelStatus.DELIVERED and p.picked_step is not None
                 and p.delivered_step is not None]
        return sum(times) / len(times) if times else None

    @staticmethod
    def station_progress(engine: SimulationEngine) -> dict[int, tuple[int, int]]:
        """station id -> (delivered, total parcels destined for it)."""
        out = {sid: [0, 0] for sid in engine.warehouse.stations}
        for p in engine.warehouse.parcels.values():
            out[p.destination_id][1] += 1
            if p.status == ParcelStatus.DELIVERED:
                out[p.destination_id][0] += 1
        return {k: (v[0], v[1]) for k, v in out.items()}
