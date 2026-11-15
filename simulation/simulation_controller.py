"""Controller between the interface and the simulation engine.

Responsibilities:

* build the scenario for the selected layout (predefined JSON or seeded random);
* own the :class:`SimulationEngine` and the run status (start/pause/resume/step/reset);
* advance the engine at the selected tick rate using a fixed-timestep
  accumulator, completely decoupled from the rendering frame rate;
* expose an interpolation factor so robots glide smoothly between the cells
  they actually occupied before and after the last tick.
"""

from __future__ import annotations

import time
from dataclasses import replace
from typing import Optional

from config.settings import (
    DEFAULT_SPEED_INDEX,
    SPEED_STEPS,
    SimulationConfig,
)
from environment.scenario_generator import (
    Scenario,
    ScenarioError,
    generate_random_scenario,
    load_predefined,
)
from simulation.simulation_engine import SimulationEngine, TickEvent
from simulation.state_manager import SimStatus, StateManager

#: Maximum simulation ticks processed per rendered frame (prevents freezing).
MAX_TICKS_PER_FRAME = 8


class SimulationController:
    def __init__(self, config: Optional[SimulationConfig] = None) -> None:
        self.config = config or SimulationConfig()
        self.state = StateManager()
        self.speed_index = DEFAULT_SPEED_INDEX
        self.random_size = "medium"
        self.scenario: Scenario
        self.engine: SimulationEngine
        self.message = ""
        self._accum = 0.0
        self.last_tick_time = time.perf_counter()
        self.new_events: list[TickEvent] = []
        #: Observers called after every tick with (engine, events). Used by the
        #: 3D layer's analytics overlays; they must not modify the engine.
        self.tick_listeners: list = []
        self.load()

    # ------------------------------------------------------------------ #
    @property
    def status(self) -> SimStatus:
        return self.state.status

    @property
    def ticks_per_second(self) -> float:
        return SPEED_STEPS[self.speed_index]

    def set_speed_index(self, index: int) -> None:
        self.speed_index = max(0, min(len(SPEED_STEPS) - 1, index))

    def build_scenario(self) -> Scenario:
        if self.config.layout == "random":
            return generate_random_scenario(self.config.seed, self.random_size)
        return load_predefined(self.config.layout)

    def load(self) -> None:
        """(Re)build scenario + engine from the current configuration."""
        try:
            self.config.validate()
            self.scenario = self.build_scenario()
        except (ScenarioError, ValueError) as exc:
            self.message = f"Could not load layout: {exc}. Using the small layout."
            self.config = replace(self.config, layout="small")
            self.scenario = load_predefined("small")
        else:
            self.message = self.scenario.metadata.get("warning", "")
        self.engine = SimulationEngine(self.scenario, self.config)
        self._reset_timing()
        self.state.set(SimStatus.READY)

    def apply_config(self, **changes) -> None:
        """Change configuration values and reset the simulation."""
        candidate = replace(self.config, **changes)
        try:
            candidate.validate()
        except ValueError as exc:
            self.message = str(exc)
            return
        self.config = candidate
        self.load()

    def _reset_timing(self) -> None:
        self._accum = 0.0
        self.last_tick_time = time.perf_counter() - 10.0  # no pending animation

    # ------------------------------------------------------------------ #
    # Commands
    # ------------------------------------------------------------------ #
    def start(self) -> None:
        if self.status == SimStatus.READY:
            self.state.set(SimStatus.RUNNING)
            self._accum = 1.0 / self.ticks_per_second  # first tick immediately

    def pause(self) -> None:
        if self.status == SimStatus.RUNNING:
            self.state.set(SimStatus.PAUSED)

    def resume(self) -> None:
        if self.status == SimStatus.PAUSED:
            self.state.set(SimStatus.RUNNING)

    def toggle(self) -> None:
        {SimStatus.READY: self.start, SimStatus.RUNNING: self.pause,
         SimStatus.PAUSED: self.resume}.get(self.status, lambda: None)()

    def reset(self) -> None:
        self.engine.reset()
        self._reset_timing()
        self.new_events = []
        self.state.set(SimStatus.READY)

    def step_once(self) -> None:
        """Advance exactly one tick (pauses a running simulation first)."""
        if self.status == SimStatus.FINISHED:
            return
        if self.status in (SimStatus.READY, SimStatus.RUNNING):
            self.state.set(SimStatus.PAUSED)
        self._tick()

    def _tick(self) -> None:
        events = self.engine.step()
        self.new_events.extend(events)
        for listener in self.tick_listeners:
            listener(self.engine, events)
        self.last_tick_time = time.perf_counter()
        if self.engine.done:
            self.state.set(SimStatus.FINISHED)

    # ------------------------------------------------------------------ #
    def update(self, dt: float) -> list[TickEvent]:
        """Advance the simulation according to elapsed real time ``dt`` (seconds)."""
        if self.status == SimStatus.RUNNING:
            interval = 1.0 / self.ticks_per_second
            self._accum += min(dt, 0.25)
            ticks = 0
            while self._accum >= interval and ticks < MAX_TICKS_PER_FRAME:
                self._accum -= interval
                self._tick()
                ticks += 1
                if self.status != SimStatus.RUNNING:
                    self._accum = 0.0
                    break
            if ticks == MAX_TICKS_PER_FRAME:
                self._accum = min(self._accum, interval)
        events, self.new_events = self.new_events, []
        return events

    def animation_progress(self) -> float:
        """0..1 progress of the movement animation of the last tick."""
        # Robots glide for most of the tick interval (continuous motion at normal
        # speeds), capped so very slow speeds still show crisp moves.
        duration = min(0.95 / self.ticks_per_second, 0.6)
        return max(0.0, min(1.0, (time.perf_counter() - self.last_tick_time) / duration))
