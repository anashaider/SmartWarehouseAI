"""Main application window (Pygame).

The dashboard renders onto a fixed logical canvas (1440 x 900) that is scaled
to the actual window, so the layout is identical on every screen size and the
window can be resized freely. Simulation updates (fixed tick rate) are
separated from rendering (60 FPS) by :class:`SimulationController`.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Optional

import pygame

from algorithms.pathfinder import compare
from config.settings import (
    AGENT_LABELS,
    ALGORITHM_LABELS,
    LAYOUT_LABELS,
    TARGET_FPS,
    SimulationConfig,
)
from interface import theme as T
from interface.analytics_view import AnalyticsView
from interface.control_panel import ControlPanel
from interface.robot_panel import RobotPanel
from interface.warehouse_view import WarehouseView
from simulation.simulation_controller import SimulationController
from simulation.state_manager import SimStatus

LOGICAL_SIZE = (1440, 900)
TOP_H = 60
MARGIN = 12
LEFT_W = 280
RIGHT_W = 370
BOTTOM_H = 100


def _enable_dpi_awareness() -> None:
    if sys.platform.startswith("win"):
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)  # type: ignore[attr-defined]
        except (AttributeError, OSError):
            pass


class Dashboard:
    def __init__(self, config: Optional[SimulationConfig] = None, headless: bool = False) -> None:
        if not headless:
            _enable_dpi_awareness()
        pygame.init()
        pygame.display.set_caption("Smart Warehouse AI — Robot Simulator")
        self.headless = headless
        self.window = self._create_window()
        self.canvas = pygame.Surface(LOGICAL_SIZE)
        self._update_transform()
        self.clock = pygame.time.Clock()
        self.ctl = SimulationController(config)
        self.selected = 0
        self.screen_mode = "simulation"     # simulation | analytics
        self.show_help = False
        self.view_options = {"paths": True, "compare": False, "reservations": True}
        W, H = LOGICAL_SIZE
        body_h = H - TOP_H - 2 * MARGIN
        left = pygame.Rect(MARGIN, TOP_H + MARGIN, LEFT_W, body_h)
        right = pygame.Rect(W - MARGIN - RIGHT_W, TOP_H + MARGIN, RIGHT_W, body_h)
        center_w = right.x - left.right - 2 * MARGIN
        self.view = WarehouseView(pygame.Rect(left.right + MARGIN, TOP_H + MARGIN, center_w,
                                              body_h - BOTTOM_H - MARGIN))
        self.metrics_rect = pygame.Rect(self.view.rect.x, self.view.rect.bottom + MARGIN, center_w, BOTTOM_H)
        self.controls = ControlPanel(left, self.ctl, self._on_config_change, self.view_options, self._on_view)
        self.robot_panel = RobotPanel(right, self._select)
        self.analytics = AnalyticsView(pygame.Rect(MARGIN, TOP_H + MARGIN, W - 2 * MARGIN, body_h))
        self._nav_rects: dict[str, pygame.Rect] = {}
        self._cmp_key: Optional[tuple] = None
        self.running = True
        self._apply_view_options()

    # ------------------------------------------------------------------ #
    # Window / scaling
    # ------------------------------------------------------------------ #
    def _create_window(self) -> pygame.Surface:
        if self.headless:
            return pygame.display.set_mode(LOGICAL_SIZE)
        try:
            dw, dh = pygame.display.get_desktop_sizes()[0]
        except (pygame.error, IndexError):
            dw, dh = LOGICAL_SIZE
        scale = min(1.0, (dw * 0.94) / LOGICAL_SIZE[0], (dh * 0.88) / LOGICAL_SIZE[1])
        size = (int(LOGICAL_SIZE[0] * scale), int(LOGICAL_SIZE[1] * scale))
        return pygame.display.set_mode(size, pygame.RESIZABLE)

    def _update_transform(self) -> None:
        ww, wh = self.window.get_size()
        s = min(ww / LOGICAL_SIZE[0], wh / LOGICAL_SIZE[1])
        self._scaled_size = (int(LOGICAL_SIZE[0] * s), int(LOGICAL_SIZE[1] * s))
        self._offset = ((ww - self._scaled_size[0]) // 2, (wh - self._scaled_size[1]) // 2)
        T.set_mouse_transform((s, s), self._offset)

    def _map_event(self, event: pygame.event.Event) -> pygame.event.Event:
        if hasattr(event, "pos") and event.type in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEBUTTONUP,
                                                     pygame.MOUSEMOTION):
            attrs = dict(event.dict)
            attrs["pos"] = T.to_logical(event.pos)
            return pygame.event.Event(event.type, attrs)
        return event

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def _select(self, robot_id: int) -> None:
        self.selected = robot_id

    def _on_config_change(self) -> None:
        self.selected = min(self.selected, self.ctl.config.num_robots - 1)
        self.view.effects.clear()
        self._cmp_key = None
        self.robot_panel.inspect_step = None

    def _on_view(self, key: str, value: bool) -> None:
        self.view_options[key] = value
        self._apply_view_options()

    def _apply_view_options(self) -> None:
        self.view.show_paths = self.view_options["paths"]
        self.view.show_reservations = self.view_options["reservations"]
        self.controls.tog_compare.value = self.view_options["compare"]

    def _update_comparison(self) -> None:
        """Run BFS and A* from the selected robot to its target (cached per query)."""
        engine = self.ctl.engine
        robots = engine.warehouse.robots
        robot = robots[min(self.selected, len(robots) - 1)]
        target = engine.warehouse.parcel_target(robot)
        key = (id(engine), robot.robot_id, robot.position, target)
        if key == self._cmp_key:
            return
        self._cmp_key = key
        if target is None:
            self.robot_panel.comparison = None
        else:
            self.robot_panel.comparison = compare(engine.warehouse.grid, robot.position, target)

    # ------------------------------------------------------------------ #
    # Events
    # ------------------------------------------------------------------ #
    def handle_event(self, event: pygame.event.Event) -> None:
        if event.type == pygame.QUIT:
            self.running = False
            return
        if event.type == pygame.VIDEORESIZE and not self.headless:
            self.window = pygame.display.set_mode((max(800, event.w), max(500, event.h)), pygame.RESIZABLE)
            self._update_transform()
            return
        event = self._map_event(event)
        if event.type == pygame.KEYDOWN and self._handle_key(event):
            return
        if self.show_help:
            if event.type == pygame.MOUSEBUTTONUP:
                self.show_help = False
            return
        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            for mode, rect in self._nav_rects.items():
                if rect.collidepoint(event.pos):
                    self.screen_mode = mode
                    return
        if self.screen_mode == "analytics":
            self.analytics.handle(event)
            return
        if self.controls.handle(event):
            return
        if self.robot_panel.handle(event, self.ctl.engine):
            return
        if event.type == pygame.MOUSEBUTTONUP and event.button == 1 and self.view.rect.collidepoint(event.pos):
            rid = self.view.robot_at_point(event.pos, self.ctl.engine)
            if rid is not None:
                self._select(rid)
                self.robot_panel.inspect_step = None

    def _handle_key(self, event: pygame.event.Event) -> bool:
        key = event.key
        if key == pygame.K_F1:
            self.show_help = not self.show_help
            return True
        if key == pygame.K_ESCAPE:
            if self.show_help:
                self.show_help = False
            else:
                self.running = False
            return True
        if key == pygame.K_a:
            self.screen_mode = "analytics" if self.screen_mode == "simulation" else "simulation"
            return True
        if self.screen_mode != "simulation":
            return False
        if key == pygame.K_SPACE:
            self.ctl.toggle()
        elif key == pygame.K_s:
            self.ctl.step_once()
        elif key == pygame.K_r:
            self.ctl.reset()
            self._on_config_change()
        elif key == pygame.K_TAB:
            self.selected = (self.selected + 1) % len(self.ctl.engine.warehouse.robots)
            self.robot_panel.inspect_step = None
        elif key == pygame.K_c:
            self._on_view("compare", not self.view_options["compare"])
        elif key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            self.ctl.set_speed_index(self.ctl.speed_index + 1)
        elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            self.ctl.set_speed_index(self.ctl.speed_index - 1)
        elif pygame.K_1 <= key <= pygame.K_3:
            idx = key - pygame.K_1
            if idx < len(self.ctl.engine.warehouse.robots):
                self.selected = idx
        else:
            return False
        return True

    # ------------------------------------------------------------------ #
    # Drawing
    # ------------------------------------------------------------------ #
    def _draw_top_bar(self, s: pygame.Surface) -> None:
        W = LOGICAL_SIZE[0]
        pygame.draw.rect(s, T.PANEL, (0, 0, W, TOP_H))
        pygame.draw.line(s, T.BORDER, (0, TOP_H - 1), (W, TOP_H - 1))
        # logo: rounded square with a 3x3 grid and a robot dot
        logo = pygame.Rect(18, 12, 36, 36)
        T.rounded_rect(s, T.ACCENT, logo, 9)
        for i in range(1, 3):
            pygame.draw.line(s, T.ACCENT_DIM, (logo.x + 6, logo.y + i * 12), (logo.right - 6, logo.y + i * 12), 2)
            pygame.draw.line(s, T.ACCENT_DIM, (logo.x + i * 12, logo.y + 6), (logo.x + i * 12, logo.bottom - 6), 2)
        pygame.draw.circle(s, (255, 255, 255), (logo.x + 24, logo.y + 12), 5)
        T.draw_text(s, "SmartWarehouse AI", (66, 11), 19, T.TEXT, bold=True)
        T.draw_text(s, "Robot Simulator · AL-2002 AI Lab · FAST-NUCES Karachi", (67, 36), 11, T.TEXT_MUTED)
        # nav
        self._nav_rects = {}
        nx = 470
        for mode, label in (("simulation", "Simulation"), ("analytics", "Analytics")):
            r = pygame.Rect(nx, 14, 120, 32)
            active = self.screen_mode == mode
            if active:
                T.rounded_rect(s, T.CARD_HOVER, r, 8, 1, T.BORDER)
            T.draw_text(s, label, r.center, 14, T.TEXT if active else T.TEXT_SECONDARY, bold=True, anchor="center")
            if active:
                pygame.draw.line(s, T.ACCENT, (r.x + 22, r.bottom + 6), (r.right - 22, r.bottom + 6), 3)
            self._nav_rects[mode] = r
            nx += 128
        # status cluster
        cfg = self.ctl.config
        st = self.ctl.status.value
        items = [
            ("STATUS", st, T.STATUS_COLORS[st]),
            ("LAYOUT", LAYOUT_LABELS.get(cfg.layout, cfg.layout), T.TEXT),
            ("AGENT · SEARCH", f"{AGENT_LABELS[cfg.agent_type]} · {ALGORITHM_LABELS[cfg.algorithm]}", T.TEXT),
            ("SPEED", f"{self.ctl.ticks_per_second:g} ticks/s", T.TEXT),
            ("STEP", f"{self.ctl.engine.step_count} / {self.ctl.engine.max_steps}", T.TEXT),
        ]
        x = W - 18
        for label, value, colr in reversed(items):
            f = T.font(13, True)
            vw = max(f.size(value)[0], T.font(10, True).size(label)[0])
            if label == "STATUS":
                pill = pygame.Rect(0, 0, vw + 30, 24)
                pill.topright = (x, 26)
                T.rounded_rect(s, T.mix(colr, T.PANEL, 0.8), pill, 12, 1, colr)
                pygame.draw.circle(s, colr, (pill.x + 12, pill.centery), 4)
                T.draw_text(s, value, (pill.x + 22, pill.centery), 12, colr, bold=True, anchor="midleft")
                T.draw_text(s, label, (pill.x + 2, 9), 10, T.TEXT_MUTED, bold=True)
                x = pill.x - 24
            else:
                T.draw_text(s, label, (x, 9), 10, T.TEXT_MUTED, bold=True, anchor="topright")
                T.draw_text(s, value, (x, 27), 13, colr, bold=True, anchor="topright")
                x -= vw + 24

    def _draw_metrics(self, s: pygame.Surface) -> None:
        engine = self.ctl.engine
        m = engine.metrics()
        cards = [
            ("Total|deliveries", f"{m['parcels_delivered']}", T.GOOD),
            ("Pending|parcels", f"{m['parcels_pending']}", T.WARN if m["parcels_pending"] else T.GOOD),
            ("Total|distance", f"{m['total_distance']}", T.TEXT),
            ("Steps|simulated", f"{m['steps']}", T.TEXT),
            ("Invalid|moves", f"{m['invalid_moves']}", T.BAD if m["invalid_moves"] else T.TEXT),
            ("Robot|conflicts", f"{m['conflicts']}", T.WARN if m["conflicts"] else T.TEXT),
            ("Nodes|expanded", f"{m['nodes_expanded']:,}", T.TEXT),
        ]
        area = self.metrics_rect
        comp_w = 160
        gap = 8
        n = len(cards)
        cw = (area.width - comp_w - gap * n) / n
        for i, (label, value, colr) in enumerate(cards):
            r = pygame.Rect(int(area.x + i * (cw + gap)), area.y, int(cw), area.height)
            T.rounded_rect(s, T.PANEL, r, 12, 1, T.BORDER)
            for k, line in enumerate(label.upper().split("|")):
                T.draw_text(s, line, (r.x + 12, r.y + 12 + k * 13), 10, T.TEXT_MUTED, bold=True,
                            max_width=r.width - 16)
            size = 26 if len(value) <= 5 else 20
            T.draw_text(s, value, (r.x + 12, r.bottom - 14), size, colr, bold=True, anchor="bottomleft",
                        max_width=r.width - 16)
        r = pygame.Rect(area.right - comp_w, area.y, comp_w, area.height)
        T.rounded_rect(s, T.PANEL, r, 12, 1, T.BORDER)
        pct = m["completion_pct"]
        T.draw_text(s, "COMPLETION", (r.x + 14, r.y + 14), 10, T.TEXT_MUTED, bold=True)
        T.draw_text(s, f"{pct:.0f}%", (r.x + 14, r.y + 34), 26, T.GOOD if pct >= 100 else T.TEXT, bold=True)
        bar = pygame.Rect(r.x + 14, r.bottom - 22, r.width - 28, 8)
        T.rounded_rect(s, T.CARD, bar, 4)
        T.rounded_rect(s, T.GOOD, pygame.Rect(bar.x, bar.y, int(bar.width * pct / 100), bar.height), 4)
        if engine.done:
            T.draw_text(s, engine.termination_reason, (r.right - 14, r.y + 16), 10, T.TEXT_SECONDARY,
                        anchor="topright", max_width=r.width - 100)

    def _draw_banner(self, s: pygame.Surface) -> None:
        msg = self.ctl.message
        if not msg:
            return
        r = pygame.Rect(self.view.rect.x + 16, self.view.rect.bottom - 40, self.view.rect.width - 32, 28)
        T.alpha_rect(s, T.WARN, r, 60, 8)
        T.draw_text(s, msg, (r.x + 10, r.centery), 12, T.TEXT, anchor="midleft", max_width=r.width - 20)

    def _draw_finished(self, s: pygame.Surface) -> None:
        engine = self.ctl.engine
        if self.ctl.status != SimStatus.FINISHED:
            return
        m = engine.metrics()
        r = pygame.Rect(0, 0, 420, 118)
        r.midtop = (self.view.rect.centerx, self.view.rect.y + 36)
        T.alpha_rect(s, T.PANEL, r, 235, 12)
        pygame.draw.rect(s, T.GOOD if m["completed"] else T.WARN, r, 1, border_radius=12)
        title = "All parcels delivered" if m["completed"] else "Simulation stopped"
        T.draw_text(s, title, (r.centerx, r.y + 14), 18, T.TEXT, bold=True, anchor="midtop")
        T.draw_text(s, f"{m['parcels_delivered']}/{m['parcels_total']} parcels · {m['completion_steps']} steps · "
                    f"{m['total_distance']} cells travelled", (r.centerx, r.y + 46), 13, T.TEXT_SECONDARY, anchor="midtop")
        T.draw_text(s, f"{m['conflicts']} conflicts · {m['deadlocks_resolved']} deadlocks resolved · "
                    f"engine time {m['compute_time_ms']:.1f} ms", (r.centerx, r.y + 68), 12, T.TEXT_MUTED, anchor="midtop")
        T.draw_text(s, "Press R to reset or change a setting", (r.centerx, r.y + 92), 11, T.TEXT_MUTED, anchor="midtop")

    def _draw_help(self, s: pygame.Surface) -> None:
        T.alpha_rect(s, (0, 0, 0), s.get_rect(), 150)
        r = pygame.Rect(0, 0, 640, 470)
        r.center = s.get_rect().center
        T.rounded_rect(s, T.PANEL, r, 14, 1, T.BORDER)
        T.draw_text(s, "Help & Legend", (r.x + 24, r.y + 20), 20, T.TEXT, bold=True)
        lines = [
            ("Space", "Start / pause / resume"), ("S", "Advance exactly one tick"),
            ("R", "Reset the current scenario"), ("Tab / 1-3", "Select robot"),
            ("C", "Toggle BFS vs A* overlay"), ("+ / -", "Change speed"),
            ("A", "Switch Simulation / Analytics"), ("F1 / Esc", "Close help / quit"),
        ]
        y = r.y + 62
        for k, v in lines:
            T.draw_text(s, k, (r.x + 24, y), 13, T.ACCENT_HOVER, bold=True)
            T.draw_text(s, v, (r.x + 140, y), 13, T.TEXT_SECONDARY)
            y += 24
        y += 10
        legend = [
            (T.SHELF, "Shelf (obstacle)"), (T.STATION, "Delivery station Dn (green badge = delivered count)"),
            (T.PARCEL, "Parcel Pn (outlined in the colour of the robot assigned to it)"),
            (T.ROBOT_COLORS[0], "Robot; white notch = heading; small box = carrying"),
            (T.WARN, "Amber pause badge = waiting because another robot blocks the route"),
            (T.INFO, "Cyan ring = yielding (deadlock retreat by the coordination layer)"),
            (T.BAD, "Red cross = conflict / rejected move"),
        ]
        for col, text in legend:
            pygame.draw.rect(s, col, (r.x + 24, y + 3, 14, 14), border_radius=3)
            T.draw_text(s, text, (r.x + 48, y), 13, T.TEXT_SECONDARY)
            y += 24
        T.draw_text(s, "Click anywhere to close", (r.centerx, r.bottom - 26), 12, T.TEXT_MUTED, anchor="center")

    def render(self) -> None:
        s = self.canvas
        s.fill(T.BG)
        self._draw_top_bar(s)
        if self.screen_mode == "analytics":
            self.analytics.draw(s)
        else:
            engine = self.ctl.engine
            self.selected = min(self.selected, len(engine.warehouse.robots) - 1)
            need_cmp = self.view_options["compare"] or self.robot_panel.tab == 2
            if need_cmp:
                self._update_comparison()
            self.view.comparison = self.robot_panel.comparison if self.view_options["compare"] else None
            self.view.draw(s, engine, self.ctl.animation_progress(), self.selected)
            self._draw_banner(s)
            self._draw_finished(s)
            self._draw_metrics(s)
            self.controls.draw(s)
            self.robot_panel.draw(s, engine, self.selected)
        if self.show_help:
            self._draw_help(s)
        if self.window.get_size() == LOGICAL_SIZE:
            self.window.blit(s, (0, 0))
        else:
            self.window.fill(T.BG)
            self.window.blit(pygame.transform.smoothscale(s, self._scaled_size), self._offset)
        pygame.display.flip()

    # ------------------------------------------------------------------ #
    def frame(self, dt: float) -> None:
        events = self.ctl.update(dt)
        if events:
            self.view.add_events(events)
        self.analytics.update()
        self.render()

    def run(self) -> None:
        last = time.perf_counter()
        while self.running:
            for event in pygame.event.get():
                self.handle_event(event)
            now = time.perf_counter()
            dt, last = now - last, now
            self.frame(dt)
            self.clock.tick(TARGET_FPS)
        pygame.quit()

    def save_screenshot(self, path: Path) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        pygame.image.save(self.canvas, str(path))


def launch(config: Optional[SimulationConfig] = None) -> None:
    Dashboard(config).run()
