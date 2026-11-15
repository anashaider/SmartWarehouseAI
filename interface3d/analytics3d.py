"""Analytics overlay for the 3D app.

Shows the recorded experiment charts (PNG files produced by
``experiments/charts.py``) and summary tables computed from the saved CSV
results. "Run Experiments" executes the real ExperimentRunner incrementally
(time budget per frame, no threads) and then exports CSV/JSON and charts.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from typing import Iterator, Optional

from direct.gui.OnscreenImage import OnscreenImage
from panda3d.core import Filename, SamplerState, TransparencyAttrib

from config.settings import CHARTS_DIR, RESULTS_DIR
from experiments.experiment_runner import ExperimentRunner, Progress
from experiments.metrics import agent_summary, path_optimality_check, search_summary
from experiments.result_exporter import ExperimentResults, export_results, load_results
from interface3d import hud as H

CHARTS = [
    ("chart1_path_length", "1  Path length: BFS vs A*"),
    ("chart2_nodes_expanded", "2  Nodes expanded"),
    ("chart3_agent_deliveries", "3  Parcels delivered by agent"),
    ("chart4_completion_steps", "4  Completion steps"),
    ("chart5_search_runtime", "5  Search runtime"),
    ("chart6_conflicts", "6  Robot conflicts"),
    ("chart7_scenario_success", "7  Scenario-wise success"),
    ("chart8_allocation", "8  Allocation baseline"),
    ("chart9_utilization", "9  Robot utilization"),
]
BUDGET_S = 0.025


class AnalyticsOverlay:
    def __init__(self, base, hud: H.Hud) -> None:
        self.base = base
        self.hud = hud
        self.visible = False
        self.selected = 0
        self.results: Optional[ExperimentResults] = None
        self.message = ""
        self.phase = "idle"   # idle | running | export
        self._iter: Optional[Iterator[Progress]] = None
        self._runner: Optional[ExperimentRunner] = None
        self.progress: Optional[Progress] = None
        self.root = None
        self.image: Optional[OnscreenImage] = None
        self.reload()

    def reload(self) -> None:
        self.results, self.message = load_results(RESULTS_DIR)

    # ------------------------------------------------------------------ build
    def build(self, width: int, height: int) -> None:
        if self.root is not None:
            self.root.removeNode()
        hud = self.hud
        self.root = self.base.pixel2d.attachNewNode("analytics")
        saved_root = hud.root
        hud.root = self.root        # reuse the HUD widget helpers on this layer
        top = H.TOP_H
        hud._frame(0, top, width, height - top, (0.03, 0.045, 0.08, 1.0))
        hud.blocking_rects.append((0, top, width, height - top))
        side_w = 290
        x0 = 20
        y = top + 20
        hud.text(None, "EXPERIMENT ANALYTICS", x0, y, 13, H.ACCENT_HOVER, bold=True)
        y += 30
        hud.button("an:run", "RUN EXPERIMENTS", x0, y, side_w - 20, 36, self.start_run, primary=True)
        y += 42
        half = (side_w - 28) / 2
        hud.button("an:reload", "RELOAD", x0, y, half, 30, self.cmd_reload)
        hud.button("an:folder", "OPEN FOLDER", x0 + half + 8, y, half, 30, self.open_folder)
        y += 40
        hud._frame(x0, y, side_w - 20, 6, H.CARD)
        self.bar = hud._frame(x0, y, 1, 6, H.ACCENT, dynamic=True)
        self.bar_w = side_w - 20
        hud.text("an:progress", "", x0, y + 12, 11, H.TEXT2, wrap=side_w - 20)
        y += 50
        for i, (_, label) in enumerate(CHARTS):
            hud.button(f"an:chart{i}", label, x0, y, side_w - 20, 30, lambda i=i: self.select(i), size=12)
            y += 34
        y += 6
        hud.text("an:msg", "", x0, y, 11, H.TEXT2, wrap=side_w - 20)
        hud.text("an:meta", "", x0, y + 52, 11, H.TEXT2, mono=True)
        hud.button("an:close", "CLOSE  (Esc)", width - 150, top + 16, 130, 30, self.hide)
        # KPI tiles (computed from the recorded results)
        kx = x0 + side_w + 10
        kw_total = width - kx - 170
        kpis = 5
        kw = (kw_total - (kpis - 1) * 10) / kpis
        for i in range(kpis):
            hud._frame(kx + i * (kw + 10), top + 14, kw, 64, H.PANEL)
            hud._frame(kx + i * (kw + 10), top + 14, 3, 64, H.ACCENT)
            hud.text(f"an:kpi{i}", "", kx + i * (kw + 10) + 14, top + 22, 20, H.TEXT, bold=True)
            hud.text(f"an:kpil{i}", "", kx + i * (kw + 10) + 14, top + 52, 10, H.TEXT2, wrap=kw - 20)
        # chart area
        self.chart_box = (kx, top + 120, width - kx - 20, height - top - 120 - 230)
        hud.text("an:title", "", self.chart_box[0], top + 92, 15, H.TEXT, bold=True)
        tx, ty = self.chart_box[0], height - 220
        hud._frame(tx, ty, self.chart_box[2], 200, H.PANEL)
        hud.text(None, "AGENT SUMMARY (mean per run)", tx + 14, ty + 10, 11, H.ACCENT_HOVER, bold=True)
        hud.text("an:agents", "", tx + 14, ty + 32, 12, H.TEXT, mono=True)
        hud.text("an:searchhead", "", tx + self.chart_box[2] * 0.56, ty + 10, 11, H.ACCENT_HOVER, bold=True)
        hud.text("an:search", "", tx + self.chart_box[2] * 0.56, ty + 32, 12, H.TEXT, mono=True)
        hud.finalize(self.root)
        hud.root = saved_root
        self.image = None
        self._image_key = None
        if not self.visible:
            self.root.hide()
        self.refresh()

    # ------------------------------------------------------------------ commands
    def show(self) -> None:
        self.visible = True
        if self.root is not None:
            self.root.show()
        self.refresh()

    def hide(self) -> None:
        self.visible = False
        if self.root is not None:
            self.root.hide()

    def toggle(self) -> None:
        (self.hide if self.visible else self.show)()

    def select(self, i: int) -> None:
        self.selected = i % len(CHARTS)
        self.refresh()

    def cmd_reload(self) -> None:
        self.reload()
        self._image_key = None
        self.refresh()

    def open_folder(self) -> None:
        RESULTS_DIR.mkdir(parents=True, exist_ok=True)
        if sys.platform.startswith("win"):
            try:
                os.startfile(str(RESULTS_DIR))  # type: ignore[attr-defined]
            except OSError as exc:
                self.message = f"Could not open folder: {exc}"
        else:
            self.message = f"Results folder: {RESULTS_DIR}"
        self.refresh()

    def start_run(self) -> None:
        if self.phase != "idle":
            return
        self._runner = ExperimentRunner()
        self._iter = self._runner.iter_run()
        self.progress = Progress(0, self._runner.total_units, "Preparing scenarios")
        self.phase = "running"
        self.message = "Running experiments (the simulation view stays responsive)."

    # ------------------------------------------------------------------ per frame
    def update(self) -> None:
        if self.phase == "running" and self._iter is not None:
            deadline = time.perf_counter() + BUDGET_S
            try:
                while time.perf_counter() < deadline:
                    self.progress = next(self._iter)
            except StopIteration:
                self.phase = "export"
                self.message = "Exporting CSV / JSON and generating charts..."
            self.refresh()
        elif self.phase == "export":
            assert self._runner is not None
            try:
                res = self._runner.results()
                export_results(res, RESULTS_DIR)
                from experiments.charts import generate_all_charts
                generate_all_charts(res, CHARTS_DIR)
                self.reload()
                self.message = (f"Completed {len(res.runs)} simulation runs and {len(res.search)} searches "
                                f"in {self._runner.elapsed_s:.1f} s. Results exported.")
            except Exception as exc:  # noqa: BLE001 - surface any failure to the user
                self.message = f"Export failed: {exc}"
            self.phase = "idle"
            self._iter = None
            self._image_key = None
            self.refresh()

    def refresh(self) -> None:
        if self.root is None:
            return
        hud = self.hud
        saved_root = hud.root
        hud.root = self.root
        hud.set_button("an:run", enabled=self.phase == "idle")
        for i in range(len(CHARTS)):
            hud.set_button(f"an:chart{i}", active=i == self.selected)
        hud.root = saved_root
        frac = self.progress.fraction if self.progress else 0.0
        self.bar["frameSize"] = (0, max(1.0, self.bar_w * frac), -6, 0)
        hud.set_text("an:progress", f"{self.progress.done}/{self.progress.total}  {self.progress.label}"
                     if self.progress else "24 seeded scenarios x 12 configurations")
        hud.set_text("an:msg", self.message, H.WARN if "fail" in self.message.lower() or "No saved" in self.message
                     else H.TEXT2)
        name, label = CHARTS[self.selected]
        hud.set_text("an:title", label[3:])
        self._show_chart(name)
        if self.results is None or self.results.runs.empty:
            hud.set_text("an:agents", "No recorded results yet.")
            hud.set_text("an:search", "")
            hud.set_text("an:meta", "")
            return
        meta = self.results.metadata
        hud.set_text("an:meta", f"scenarios   {meta.get('num_scenarios', '?')}\n"
                                f"base seed   {meta.get('base_seed', '?')}\n"
                                f"runs        {len(self.results.runs)}\n"
                                f"searches    {len(self.results.search)}\n"
                                f"generated   {str(meta.get('generated_at', '?')).replace('T', ' ')[:16]}")
        a = agent_summary(self.results.runs)
        lines = [f"{'Agent':<14}{'Deliv':>7}{'Compl%':>8}{'Steps':>8}{'Dist':>8}{'Confl':>7}"]
        for r in a.itertuples():
            lines.append(f"{r.agent_label:<14}{r.avg_delivered:>7.2f}{r.avg_completion_pct:>8.1f}"
                         f"{r.avg_completion_steps:>8.0f}{r.avg_distance:>8.0f}{r.avg_conflicts:>7.1f}")
        hud.set_text("an:agents", "\n".join(lines))
        self._set_kpis(a)
        s = search_summary(self.results.search)
        opt = path_optimality_check(self.results.search)
        hud.set_text("an:searchhead", f"SEARCH SUMMARY  ·  equal path length {opt['equal_length']}/{opt['queries_compared']}")
        lines = [f"{'Size':<8}{'Alg':<5}{'Path':>7}{'Nodes':>8}{'ms':>8}"]
        for r in s.itertuples():
            lines.append(f"{r.size_class:<8}{r.algorithm_label:<5}{r.avg_path_length:>7.2f}"
                         f"{r.avg_nodes_expanded:>8.1f}{r.avg_runtime_ms:>8.3f}")
        hud.set_text("an:search", "\n".join(lines))

    def _set_kpis(self, agents) -> None:
        """Headline numbers, all computed from the saved CSV results."""
        hud = self.hud
        runs, search = self.results.runs, self.results.search
        found = search[search["found"].astype(bool)]
        bfs_n = found[found["algorithm"] == "bfs"]["nodes_expanded"].mean()
        ast_n = found[found["algorithm"] == "astar"]["nodes_expanded"].mean()
        opt = path_optimality_check(search)
        plan = runs[runs["agent"] != "simple_reflex"]
        util = (runs["total_distance"] / (runs["robots"] * runs["steps"].clip(lower=1))).groupby(
            runs["agent_label"]).mean()
        tiles = [
            (f"{bfs_n / ast_n:.1f}x" if ast_n else "-", "fewer nodes expanded by A* than BFS (same queries)"),
            (f"{opt['equal_length']}/{opt['queries_compared']}", "queries where BFS and A* path lengths are equal"),
            (f"{plan['completion_pct'].mean():.0f}%", "delivery completion, Goal-Based + Utility-Based agents"),
            (f"{runs[runs['agent'] == 'simple_reflex']['completion_pct'].mean():.0f}%",
             "delivery completion, Simple Reflex agent"),
            (f"{100 * util.get('Goal-Based', 0):.0f}%",
             f"of robot-ticks spent moving, Goal-Based (Reflex {100 * util.get('Simple Reflex', 0):.0f}%: "
             "busy but mostly unproductive)"),
        ]
        for i, (value, label) in enumerate(tiles):
            hud.set_text(f"an:kpi{i}", value)
            hud.set_text(f"an:kpil{i}", label)

    def _show_chart(self, name: str) -> None:
        path = Path(CHARTS_DIR) / f"{name}.png"
        key = (name, path.exists() and path.stat().st_mtime, self.chart_box)
        if key == self._image_key:
            return
        self._image_key = key
        if self.image is not None:
            self.image.destroy()
            self.image = None
        if not path.exists():
            return
        tex = self.base.loader.loadTexture(Filename.fromOsSpecific(str(path)))
        tex.setMinfilter(SamplerState.FT_linear_mipmap_linear)
        tw, th = tex.getOrigFileXSize() or tex.getXSize(), tex.getOrigFileYSize() or tex.getYSize()
        bx, by, bw, bh = self.chart_box
        scale = min(bw / tw, bh / th)
        w, h = tw * scale, th * scale
        self.image = OnscreenImage(image=tex, parent=self.root, scale=(w / 2, 1, h / 2),
                                   pos=(bx + bw / 2, 0, -(by + bh / 2)))
        self.image.setTransparency(TransparencyAttrib.MAlpha)
        self.image.setBin("fixed", H.LAYER_BASE["analytics"] + 1)
