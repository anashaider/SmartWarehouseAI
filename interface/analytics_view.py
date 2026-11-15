"""Analytics screen: run experiments in-app, browse charts and summary tables.

Experiments are executed incrementally inside the main loop (a time budget per
frame) through :meth:`ExperimentRunner.iter_run`, so the window never freezes
and no background threads are used. Charts are the PNG files generated from
the recorded results.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from typing import Iterator, Optional

import pygame

from config.settings import CHARTS_DIR, RESULTS_DIR
from experiments.experiment_runner import ExperimentRunner, Progress
from experiments.metrics import agent_summary, path_optimality_check, search_summary
from experiments.result_exporter import ExperimentResults, export_results, load_results
from interface import theme as T
from interface.widgets import Button, section_title

CHART_FILES = [
    ("chart1_path_length", "1 · Path length (BFS vs A*)"),
    ("chart2_nodes_expanded", "2 · Nodes expanded"),
    ("chart3_agent_deliveries", "3 · Parcels delivered by agent"),
    ("chart4_completion_steps", "4 · Completion steps"),
    ("chart5_search_runtime", "5 · Search runtime"),
    ("chart6_conflicts", "6 · Robot conflicts"),
    ("chart7_scenario_success", "7 · Scenario-wise success"),
    ("chart8_allocation", "8 · Allocation baseline"),
    ("chart9_utilization", "9 · Robot utilization"),
]

FRAME_BUDGET_S = 0.030


class AnalyticsView:
    def __init__(self, rect: pygame.Rect) -> None:
        self.rect = pygame.Rect(rect)
        self.results: Optional[ExperimentResults] = None
        self.message = ""
        self.selected = 0
        self._image_cache: dict[tuple[str, tuple[int, int]], pygame.Surface] = {}
        self._iter: Optional[Iterator[Progress]] = None
        self._runner: Optional[ExperimentRunner] = None
        self.progress: Optional[Progress] = None
        self.phase = "idle"   # idle | running | charts
        self.chart_buttons: list[Button] = []
        self.reload()
        self.layout()

    # ------------------------------------------------------------------ #
    def set_rect(self, rect: pygame.Rect) -> None:
        self.rect = pygame.Rect(rect)
        self.layout()

    def layout(self) -> None:
        self.side = pygame.Rect(self.rect.x, self.rect.y, 300, self.rect.height)
        self.main = pygame.Rect(self.side.right + 12, self.rect.y, self.rect.width - 312, self.rect.height)
        x, w = self.side.x + 16, self.side.width - 32
        y = self.side.y + 42
        self.btn_run = Button(pygame.Rect(x, y, w, 40), "Run Experiments", self.start_run, primary=True, icon="play")
        y += 48
        half = (w - 8) // 2
        self.btn_reload = Button(pygame.Rect(x, y, half, 34), "Reload", self.reload, icon="reset")
        self.btn_folder = Button(pygame.Rect(x + half + 8, y, half, 34), "Open folder", self.open_folder)
        y += 84
        self.chart_y = y
        self.chart_buttons = []
        for i, (_, label) in enumerate(CHART_FILES):
            btn = Button(pygame.Rect(x, y + 28 + i * 40, w, 34), label, lambda i=i: self.select(i))
            self.chart_buttons.append(btn)
        self.info_y = y + 28 + len(CHART_FILES) * 40 + 10

    def select(self, i: int) -> None:
        self.selected = i

    def reload(self) -> None:
        self.results, self.message = load_results(RESULTS_DIR)
        self._image_cache.clear()

    def open_folder(self) -> None:
        RESULTS_DIR.mkdir(parents=True, exist_ok=True)
        try:
            if sys.platform.startswith("win"):
                os.startfile(str(RESULTS_DIR))  # type: ignore[attr-defined]
            else:
                self.message = f"Results folder: {RESULTS_DIR}"
        except OSError as exc:
            self.message = f"Could not open folder: {exc}"

    # ------------------------------------------------------------------ #
    @property
    def busy(self) -> bool:
        return self.phase != "idle"

    def start_run(self) -> None:
        if self.busy:
            return
        self._runner = ExperimentRunner()
        self._iter = self._runner.iter_run()
        self.progress = Progress(0, self._runner.total_units, "Preparing scenarios")
        self.phase = "running"
        self.message = "Running experiments…"

    def update(self) -> None:
        if self.phase == "running" and self._iter is not None:
            deadline = time.perf_counter() + FRAME_BUDGET_S
            try:
                while time.perf_counter() < deadline:
                    self.progress = next(self._iter)
            except StopIteration:
                self.phase = "charts"
                self.message = "Exporting results and generating charts…"
        elif self.phase == "charts":
            # Executed one frame after the message is shown.
            assert self._runner is not None
            try:
                results = self._runner.results()
                export_results(results, RESULTS_DIR)
                from experiments.charts import generate_all_charts
                generate_all_charts(results, CHARTS_DIR)
                self.reload()
                self.message = (f"Completed {len(results.runs)} simulation runs and "
                                f"{len(results.search)} searches in {self._runner.elapsed_s:.1f}s.")
            except Exception as exc:  # noqa: BLE001 - surface any export failure in the UI
                self.message = f"Export failed: {exc}"
            self.phase = "idle"
            self._iter = None

    # ------------------------------------------------------------------ #
    def handle(self, event: pygame.event.Event) -> bool:
        self.btn_run.enabled = not self.busy
        for b in [self.btn_run, self.btn_reload, self.btn_folder, *self.chart_buttons]:
            if b.handle(event):
                return True
        if event.type == pygame.KEYDOWN and event.key in (pygame.K_DOWN, pygame.K_RIGHT):
            self.selected = (self.selected + 1) % len(CHART_FILES)
            return True
        if event.type == pygame.KEYDOWN and event.key in (pygame.K_UP, pygame.K_LEFT):
            self.selected = (self.selected - 1) % len(CHART_FILES)
            return True
        return False

    def _chart_surface(self, name: str, box: pygame.Rect) -> Optional[pygame.Surface]:
        key = (name, box.size)
        if key in self._image_cache:
            return self._image_cache[key]
        path = Path(CHARTS_DIR) / f"{name}.png"
        if not path.exists():
            return None
        try:
            img = pygame.image.load(str(path)).convert()
        except (pygame.error, OSError):
            return None
        scale = min(box.width / img.get_width(), box.height / img.get_height())
        img = pygame.transform.smoothscale(img, (int(img.get_width() * scale), int(img.get_height() * scale)))
        self._image_cache[key] = img
        return img

    def draw(self, surface: pygame.Surface) -> None:
        # Side panel
        T.rounded_rect(surface, T.PANEL, self.side, 12, 1, T.BORDER)
        x, w = self.side.x + 16, self.side.width - 32
        section_title(surface, "Experiment Runner", x, self.side.y + 14, w)
        self.btn_run.enabled = not self.busy
        for b in (self.btn_run, self.btn_reload, self.btn_folder):
            b.draw(surface)
        py = self.btn_reload.rect.bottom + 12
        bar = pygame.Rect(x, py, w, 8)
        T.rounded_rect(surface, T.CARD, bar, 4)
        if self.progress:
            fill = pygame.Rect(bar.x, bar.y, int(bar.width * self.progress.fraction), bar.height)
            T.rounded_rect(surface, T.GOOD if self.phase == "idle" else T.ACCENT, fill, 4)
            T.draw_text(surface, f"{self.progress.done}/{self.progress.total}  {self.progress.label}",
                        (x, py + 14), 11, T.TEXT_SECONDARY, max_width=w)
        else:
            T.draw_text(surface, "24 seeded scenarios × 12 configurations", (x, py + 14), 11, T.TEXT_MUTED)
        section_title(surface, "Charts", x, self.chart_y, w)
        for i, b in enumerate(self.chart_buttons):
            b.active = i == self.selected
            b.draw(surface)
        y = self.info_y
        T.draw_wrapped(surface, self.message, pygame.Rect(x, y, w, 60), 12,
                       T.WARN if "fail" in self.message.lower() or "No saved" in self.message else T.TEXT_SECONDARY)
        y += 60
        if self.results is not None:
            meta = self.results.metadata
            for k, v in (("Scenarios", meta.get("num_scenarios", self.results.runs["scenario_id"].nunique())),
                         ("Base seed", meta.get("base_seed", "?")),
                         ("Simulation runs", len(self.results.runs)),
                         ("Search queries", len(self.results.search)),
                         ("Generated", str(meta.get("generated_at", "?")).replace("T", " "))):
                if y + 18 > self.side.bottom:
                    break
                T.draw_text(surface, str(k), (x, y), 12, T.TEXT_MUTED)
                T.draw_text(surface, str(v), (x + w, y), 12, T.TEXT, bold=True, anchor="topright")
                y += 18

        # Main panel: chart + tables
        T.rounded_rect(surface, T.PANEL, self.main, 12, 1, T.BORDER)
        table_h = 210
        chart_box = pygame.Rect(self.main.x + 16, self.main.y + 16, self.main.width - 32,
                                self.main.height - table_h - 32)
        name = CHART_FILES[self.selected][0]
        img = self._chart_surface(name, chart_box)
        if img is not None:
            surface.blit(img, img.get_rect(center=chart_box.center))
        else:
            T.draw_text(surface, "No chart available yet", chart_box.center, 20, T.TEXT_SECONDARY,
                        bold=True, anchor="center")
            T.draw_text(surface, "Click 'Run Experiments' (about a minute) or run: python main.py --experiments",
                        (chart_box.centerx, chart_box.centery + 30), 13, T.TEXT_MUTED, anchor="center")
        self._draw_tables(surface, pygame.Rect(self.main.x + 16, self.main.bottom - table_h - 8,
                                               self.main.width - 32, table_h))

    def _draw_tables(self, surface: pygame.Surface, area: pygame.Rect) -> None:
        if self.results is None or self.results.runs.empty:
            return
        half = (area.width - 16) // 2
        left = pygame.Rect(area.x, area.y, half + 60, area.height)
        right = pygame.Rect(left.right + 16, area.y, area.width - left.width - 16, area.height)
        agents = agent_summary(self.results.runs)
        self._table(surface, left, "Agent summary (mean per run)",
                    ["Agent", "Delivered", "Compl. %", "Steps", "Distance", "Conflicts", "Invalid", "Nodes"],
                    [[r.agent_label, f"{r.avg_delivered:.2f}", f"{r.avg_completion_pct:.1f}",
                      f"{r.avg_completion_steps:.0f}", f"{r.avg_distance:.0f}", f"{r.avg_conflicts:.1f}",
                      f"{r.avg_invalid_moves:.1f}", f"{r.avg_nodes_expanded:,.0f}"]
                     for r in agents.itertuples()])
        search = search_summary(self.results.search)
        opt = path_optimality_check(self.results.search)
        rows = [[r.size_class, r.algorithm_label, f"{r.avg_path_length:.2f}",
                 f"{r.avg_nodes_expanded:.1f}", f"{r.avg_runtime_ms:.3f}"] for r in search.itertuples()]
        self._table(surface, right, f"Search summary · equal path length in "
                    f"{opt['equal_length']}/{opt['queries_compared']} queries",
                    ["Size", "Alg.", "Path", "Nodes", "ms"], rows)

    @staticmethod
    def _table(surface: pygame.Surface, area: pygame.Rect, title: str, headers: list[str],
               rows: list[list[str]]) -> None:
        T.rounded_rect(surface, T.PANEL_ALT, area, 10)
        T.draw_text(surface, title, (area.x + 12, area.y + 8), 12, T.ACCENT_HOVER, bold=True,
                    max_width=area.width - 24)
        first = int(area.width * (0.26 if len(headers) > 5 else 0.22))
        rest = (area.width - 24 - first) / max(1, len(headers) - 1)
        xs = [area.x + 12] + [int(area.x + 12 + first + i * rest) for i in range(len(headers) - 1)]
        y = area.y + 32
        for i, h in enumerate(headers):
            T.draw_text(surface, h, (xs[i] if i == 0 else xs[i] + int(rest) - 8, y), 11, T.TEXT_MUTED,
                        bold=True, anchor="topleft" if i == 0 else "topright")
        y += 20
        pygame.draw.line(surface, T.DIVIDER, (area.x + 10, y - 3), (area.right - 10, y - 3))
        row_h = max(16, min(22, (area.bottom - y - 6) // max(1, len(rows))))
        for row in rows:
            if y + row_h > area.bottom:
                break
            for i, cell in enumerate(row):
                T.draw_text(surface, cell, (xs[i] if i == 0 else xs[i] + int(rest) - 8, y), 12,
                            T.TEXT if i == 0 else T.TEXT_SECONDARY, bold=i == 0,
                            anchor="topleft" if i == 0 else "topright", max_width=first - 4 if i == 0 else None)
            y += row_h
