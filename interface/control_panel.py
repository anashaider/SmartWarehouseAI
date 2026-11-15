"""Left control panel: run commands, configuration selectors, speed, view toggles."""

from __future__ import annotations

from typing import Callable

import pygame

from config.settings import (
    AGENT_LABELS,
    AGENT_TYPES,
    ALGORITHM_LABELS,
    ALGORITHMS,
    ALLOCATION_LABELS,
    ALLOCATION_POLICIES,
    LAYOUT_LABELS,
    MAX_ROBOTS,
    MIN_ROBOTS,
    SPEED_STEPS,
)
from interface import theme as T
from interface.widgets import Button, Selector, Slider, Toggle, section_title
from simulation.simulation_controller import SimulationController
from simulation.state_manager import SimStatus

RANDOM_SEEDS = [str(s) for s in (7, 13, 21, 42, 64, 99, 123, 256, 512, 777, 1024, 2026)]


class ControlPanel:
    def __init__(self, rect: pygame.Rect, controller: SimulationController,
                 on_config_change: Callable[[], None], view_options: dict[str, bool],
                 on_view_change: Callable[[str, bool], None]) -> None:
        self.rect = pygame.Rect(rect)
        self.ctl = controller
        self.on_config_change = on_config_change
        self.view_options = view_options
        self.on_view_change = on_view_change
        self.widgets: list = []
        self.build()

    # ------------------------------------------------------------------ #
    def _change(self, **kwargs) -> None:
        self.ctl.apply_config(**kwargs)
        self.on_config_change()
        self.sync()

    def build(self) -> None:
        x, w = self.rect.x + 16, self.rect.width - 32
        y = self.rect.y + 14
        self.y_controls = y
        y += 28
        half = (w - 8) // 2
        third = (w - 16) // 3
        ctl = self.ctl
        self.btn_start = Button(pygame.Rect(x, y, half, 38), "Start", ctl.start, primary=True, icon="play")
        self.btn_pause = Button(pygame.Rect(x + half + 8, y, half, 38), "Pause", ctl.pause, icon="pause")
        y += 46
        self.btn_resume = Button(pygame.Rect(x, y, half, 38), "Resume", ctl.resume, icon="play")
        self.btn_step = Button(pygame.Rect(x + half + 8, y, half, 38), "Step", ctl.step_once, icon="step")
        y += 46
        self.btn_reset = Button(pygame.Rect(x, y, w, 34), "Reset Simulation", self._reset, icon="reset")
        y += 48
        self.y_config = y
        y += 28
        cfg = ctl.config
        sel_h = 52
        self.sel_agent = Selector(pygame.Rect(x, y, w, sel_h), "AI Agent",
                                  [(a, AGENT_LABELS[a]) for a in AGENT_TYPES], cfg.agent_type,
                                  lambda v: self._change(agent_type=v))
        y += sel_h + 8
        self.sel_alg = Selector(pygame.Rect(x, y, w, sel_h), "Pathfinding",
                                [(a, ALGORITHM_LABELS[a]) for a in ALGORITHMS], cfg.algorithm,
                                lambda v: self._change(algorithm=v))
        y += sel_h + 8
        self.sel_layout = Selector(pygame.Rect(x, y, w, sel_h), "Warehouse Layout",
                                   [(k, LAYOUT_LABELS[k]) for k in ("small", "medium", "large", "random")],
                                   cfg.layout, lambda v: self._change(layout=v))
        y += sel_h + 8
        self.sel_robots = Selector(pygame.Rect(x, y, half, sel_h), "Robots",
                                   [(str(n), f"{n} robots") for n in range(MIN_ROBOTS, MAX_ROBOTS + 1)],
                                   str(cfg.num_robots), lambda v: self._change(num_robots=int(v)))
        self.sel_seed = Selector(pygame.Rect(x + half + 8, y, half, sel_h), "Seed",
                                 [(s, s) for s in RANDOM_SEEDS], str(cfg.seed),
                                 lambda v: self._change(seed=int(v)))
        y += sel_h + 8
        self.sel_alloc = Selector(pygame.Rect(x, y, w, sel_h), "Task Allocation",
                                  [(a, ALLOCATION_LABELS[a]) for a in ALLOCATION_POLICIES], cfg.allocation,
                                  lambda v: self._change(allocation=v))
        y += sel_h + 10
        self.slider = Slider(pygame.Rect(x, y, w, 46), "Simulation Speed", SPEED_STEPS, ctl.speed_index,
                             ctl.set_speed_index, fmt=lambda v: f"{v:g} ticks/s")
        y += 54
        self.y_view = y
        y += 28
        self.tog_paths = Toggle(pygame.Rect(x, y, w, 26), "Planned paths & targets",
                                self.view_options["paths"], lambda v: self.on_view_change("paths", v))
        y += 30
        self.tog_compare = Toggle(pygame.Rect(x, y, w, 26), "BFS vs A* overlay (selected robot)",
                                  self.view_options["compare"], lambda v: self.on_view_change("compare", v))
        y += 30
        self.tog_res = Toggle(pygame.Rect(x, y, w, 26), "Cell reservations",
                              self.view_options["reservations"], lambda v: self.on_view_change("reservations", v))
        y += 34
        self.y_help = y
        self.widgets = [self.btn_start, self.btn_pause, self.btn_resume, self.btn_step, self.btn_reset,
                        self.sel_agent, self.sel_alg, self.sel_layout, self.sel_robots, self.sel_seed,
                        self.sel_alloc, self.slider, self.tog_paths, self.tog_compare, self.tog_res]
        self.sync()

    def _reset(self) -> None:
        self.ctl.reset()
        self.on_config_change()

    def set_rect(self, rect: pygame.Rect) -> None:
        self.rect = pygame.Rect(rect)
        self.build()

    def sync(self) -> None:
        """Enable/disable buttons according to the run status."""
        st = self.ctl.status
        self.btn_start.enabled = st == SimStatus.READY
        self.btn_pause.enabled = st == SimStatus.RUNNING
        self.btn_resume.enabled = st == SimStatus.PAUSED
        self.btn_step.enabled = st != SimStatus.FINISHED
        self.sel_seed.enabled = self.ctl.config.layout == "random"
        cfg = self.ctl.config
        self.sel_agent.set_value(cfg.agent_type)
        self.sel_alg.set_value(cfg.algorithm)
        self.sel_layout.set_value(cfg.layout)
        self.sel_robots.set_value(str(cfg.num_robots))
        self.sel_alloc.set_value(cfg.allocation)
        if str(cfg.seed) not in RANDOM_SEEDS:
            self.sel_seed.options.insert(0, (str(cfg.seed), str(cfg.seed)))
        self.sel_seed.set_value(str(cfg.seed))
        self.slider.index = self.ctl.speed_index
        self.sel_alg.label = ("Pathfinding (unused by reflex)" if cfg.agent_type == "simple_reflex"
                              else "Pathfinding")

    def handle(self, event: pygame.event.Event) -> bool:
        for w in self.widgets:
            if w.handle(event):
                self.sync()
                return True
        return False

    def draw(self, surface: pygame.Surface) -> None:
        self.sync()
        T.rounded_rect(surface, T.PANEL, self.rect, 12, 1, T.BORDER)
        x, w = self.rect.x + 16, self.rect.width - 32
        section_title(surface, "Simulation Control", x, self.y_controls, w)
        section_title(surface, "Configuration", x, self.y_config, w)
        section_title(surface, "View", x, self.y_view, w)
        for wd in self.widgets:
            wd.draw(surface)
        help_lines = ["Space start/pause · S step · R reset",
                      "Tab next robot · Click robot to inspect",
                      "C compare overlay · A analytics · F1 help"]
        y = self.y_help
        if y + 50 < self.rect.bottom:
            for line in help_lines:
                T.draw_text(surface, line, (x + 2, y), 11, T.TEXT_MUTED)
                y += 16
