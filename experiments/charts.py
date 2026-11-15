"""Analytical charts generated from recorded experiment results (Matplotlib, Agg).

Every chart is computed from the DataFrames produced by the experiment runner;
no values are typed in by hand. Colours follow a validated colour-blind-safe
categorical order on the dark dashboard surface (blue, orange, aqua).
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

from config.settings import CHARTS_DIR  # noqa: E402
from experiments.result_exporter import ExperimentResults  # noqa: E402

SURFACE = "#151c2c"
TEXT = "#e6ebf5"
TEXT_MUTED = "#9aa6bf"
GRID = "#2a3450"
SERIES = ["#3987e5", "#d95926", "#199e70"]   # validated categorical slots 1-3 (dark)
SEQ = ["#0d366b", "#184f95", "#256abf", "#3987e5", "#6da7ec", "#9ec5f4", "#cde2fb"]

SIZE_ORDER = ["small", "medium", "large"]
AGENT_ORDER = ["Simple Reflex", "Goal-Based", "Utility-Based"]
ALG_ORDER = ["BFS", "A*"]

CHART_TITLES = {
    "chart1_path_length": "Path Length Comparison (BFS vs A*)",
    "chart2_nodes_expanded": "Search Efficiency: Nodes Expanded",
    "chart3_agent_deliveries": "Agent Performance: Parcels Delivered",
    "chart4_completion_steps": "Average Completion Steps",
    "chart5_search_runtime": "Search Execution Runtime",
    "chart6_conflicts": "Robot Conflict Comparison",
    "chart7_scenario_success": "Scenario-wise Delivery Success",
    "chart8_allocation": "Task Allocation Baseline Comparison",
}


def _style(ax: plt.Axes, title: str, ylabel: str, xlabel: str = "") -> None:
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=TEXT, fontsize=13, fontweight="bold", loc="left", pad=12)
    ax.set_ylabel(ylabel, color=TEXT_MUTED, fontsize=10)
    ax.set_xlabel(xlabel, color=TEXT_MUTED, fontsize=10)
    ax.tick_params(colors=TEXT_MUTED, labelsize=9, length=0)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(GRID)
    ax.yaxis.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def _figure() -> tuple[plt.Figure, plt.Axes]:
    fig, ax = plt.subplots(figsize=(8.0, 4.6), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    return fig, ax


def _legend(ax: plt.Axes, title: str | None = None) -> None:
    leg = ax.legend(frameon=False, fontsize=9, title=title, loc="upper left",
                    bbox_to_anchor=(1.0, 1.0))
    for text in leg.get_texts():
        text.set_color(TEXT)
    if leg.get_title():
        leg.get_title().set_color(TEXT_MUTED)
        leg.get_title().set_fontsize(9)


def _grouped_bars(ax: plt.Axes, table: pd.DataFrame, groups: list[str], series: list[str],
                  fmt: str = "{:.1f}", label_values: bool = True) -> None:
    """``table``: index = groups, columns = series."""
    n = len(series)
    width = 0.8 / n
    x = np.arange(len(groups))
    for i, name in enumerate(series):
        vals = [table.loc[g, name] if (g in table.index and name in table.columns) else np.nan
                for g in groups]
        bars = ax.bar(x + (i - (n - 1) / 2) * width, vals, width, label=name,
                      color=SERIES[i % len(SERIES)], edgecolor=SURFACE, linewidth=2)
        if label_values:
            for b, v in zip(bars, vals):
                if not np.isnan(v):
                    ax.annotate(fmt.format(v), (b.get_x() + b.get_width() / 2, b.get_height()),
                                xytext=(0, 3), textcoords="offset points", ha="center",
                                va="bottom", fontsize=8, color=TEXT_MUTED)
    ax.set_xticks(x, [g.title() if g.islower() else g for g in groups])


def _save(fig: plt.Figure, out_dir: Path, name: str, note: str | None = None) -> Path:
    if note:
        fig.text(0.01, 0.01, note, color=TEXT_MUTED, fontsize=8, ha="left", va="bottom")
    fig.tight_layout(rect=(0, 0.04 if note else 0, 1, 1))
    path = out_dir / f"{name}.png"
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)
    return path


def _by_size(df: pd.DataFrame, series_col: str, value: str, series_order: list[str]) -> pd.DataFrame:
    table = df.pivot_table(index="size_class", columns=series_col, values=value, aggfunc="mean")
    overall = df.groupby(series_col)[value].mean()
    table.loc["all"] = overall
    return table.reindex(columns=[s for s in series_order if s in table.columns])


def generate_all_charts(results: ExperimentResults, out_dir: Path = CHARTS_DIR) -> list[Path]:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    search = results.search[results.search["found"].astype(bool)].copy()
    runs = results.runs.copy()
    groups = SIZE_ORDER + ["all"]
    paths: list[Path] = []
    n_queries = search["query_id"].count() // max(1, search["algorithm"].nunique())
    n_scen = runs["scenario_id"].nunique()

    # 1. Path length --------------------------------------------------------
    fig, ax = _figure()
    _grouped_bars(ax, _by_size(search, "algorithm_label", "path_length", ALG_ORDER), groups, ALG_ORDER)
    _style(ax, CHART_TITLES["chart1_path_length"], "Average path length (moves)", "Warehouse size")
    _legend(ax, "Algorithm")
    paths.append(_save(fig, out_dir, "chart1_path_length",
                       f"{n_queries} identical start-goal queries per algorithm over {n_scen} seeded scenarios."))

    # 2. Nodes expanded -----------------------------------------------------
    fig, ax = _figure()
    _grouped_bars(ax, _by_size(search, "algorithm_label", "nodes_expanded", ALG_ORDER), groups, ALG_ORDER, "{:.0f}")
    _style(ax, CHART_TITLES["chart2_nodes_expanded"], "Average nodes expanded per search", "Warehouse size")
    _legend(ax, "Algorithm")
    paths.append(_save(fig, out_dir, "chart2_nodes_expanded",
                       "Same queries as Chart 1. Lower is more efficient."))

    # 3. Parcels delivered per agent ---------------------------------------
    fig, ax = _figure()
    table = _by_size(runs, "agent_label", "parcels_delivered", AGENT_ORDER)
    _grouped_bars(ax, table, groups, AGENT_ORDER)
    totals = runs.groupby("size_class")["parcels_total"].mean()
    for i, g in enumerate(groups):
        total = totals.get(g, runs["parcels_total"].mean() if g == "all" else np.nan)
        ax.hlines(total, i - 0.42, i + 0.42, colors=TEXT_MUTED, linestyles="dashed", linewidth=1)
    ax.plot([], [], color=TEXT_MUTED, linestyle="dashed", linewidth=1, label="Parcels available")
    _style(ax, CHART_TITLES["chart3_agent_deliveries"], "Average parcels delivered per run", "Warehouse size")
    _legend(ax, "Agent")
    paths.append(_save(fig, out_dir, "chart3_agent_deliveries",
                       "Averaged over both algorithms and both allocation policies."))

    # 4. Completion steps ---------------------------------------------------
    fig, ax = _figure()
    _grouped_bars(ax, _by_size(runs, "agent_label", "completion_steps", AGENT_ORDER), groups, AGENT_ORDER, "{:.0f}")
    _style(ax, CHART_TITLES["chart4_completion_steps"], "Average simulation steps (ticks)", "Warehouse size")
    _legend(ax, "Agent")
    paths.append(_save(fig, out_dir, "chart4_completion_steps",
                       "Incomplete runs are counted at the step limit; lower is better."))

    # 5. Search runtime -----------------------------------------------------
    fig, ax = _figure()
    _grouped_bars(ax, _by_size(search, "algorithm_label", "runtime_ms", ALG_ORDER), groups, ALG_ORDER, "{:.3f}")
    _style(ax, CHART_TITLES["chart5_search_runtime"], "Average runtime per search (ms)", "Warehouse size")
    _legend(ax, "Algorithm")
    paths.append(_save(fig, out_dir, "chart5_search_runtime",
                       "Best of repeated timings per query; excludes rendering. Hardware dependent."))

    # 6. Conflicts ----------------------------------------------------------
    fig, ax = _figure()
    _grouped_bars(ax, _by_size(runs, "agent_label", "conflicts", AGENT_ORDER), groups, AGENT_ORDER)
    _style(ax, CHART_TITLES["chart6_conflicts"], "Average robot conflicts per run", "Warehouse size")
    _legend(ax, "Agent")
    paths.append(_save(fig, out_dir, "chart6_conflicts",
                       "Conflict = a tick where another robot blocked the robot's shortest route. Collisions: always 0."))

    # 7. Scenario-wise success heatmap -------------------------------------
    pivot = runs.pivot_table(index="agent_label", columns="scenario_id",
                             values="completion_pct", aggfunc="mean").reindex(AGENT_ORDER)
    fig, ax = plt.subplots(figsize=(11.0, 3.6), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    cmap = LinearSegmentedColormap.from_list("seq", SEQ[::-1])
    im = ax.imshow(pivot.values, aspect="auto", cmap=cmap, vmin=0, vmax=100)
    ax.set_yticks(range(len(pivot.index)), pivot.index)
    ax.set_xticks(range(len(pivot.columns)), [c.split("-")[0] for c in pivot.columns], rotation=0)
    for (r, c), v in np.ndenumerate(pivot.values):
        ax.text(c, r, f"{v:.0f}", ha="center", va="center", fontsize=7,
                color="#0b1220" if v >= 55 else TEXT)
    _style(ax, CHART_TITLES["chart7_scenario_success"], "", "Scenario (S01-S24; sizes cycle small, medium, large)")
    ax.yaxis.grid(False)
    cbar = fig.colorbar(im, ax=ax, fraction=0.025, pad=0.01)
    cbar.set_label("Delivery completion (%)", color=TEXT_MUTED, fontsize=9)
    cbar.ax.tick_params(colors=TEXT_MUTED, labelsize=8)
    cbar.outline.set_visible(False)
    paths.append(_save(fig, out_dir, "chart7_scenario_success",
                       "Mean completion % per scenario over both algorithms and both allocation policies."))

    # 8. Allocation policy ---------------------------------------------------
    fig, ax = _figure()
    alloc = runs.pivot_table(index="agent_label", columns="allocation_label",
                             values="total_distance", aggfunc="mean").reindex(AGENT_ORDER)
    policies = [p for p in ("Fixed Order", "Nearest Parcel") if p in alloc.columns]
    _grouped_bars(ax, alloc, AGENT_ORDER, policies, "{:.0f}")
    _style(ax, CHART_TITLES["chart8_allocation"], "Average total distance per run (cells)", "Agent")
    _legend(ax, "Allocation")
    paths.append(_save(fig, out_dir, "chart8_allocation",
                       "Fixed-order baseline vs nearest-parcel allocation, all scenarios and algorithms."))
    return paths
