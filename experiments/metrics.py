"""Metric definitions and aggregation helpers for experiment results.

Simulation-run metrics (one row per scenario x agent x algorithm x allocation)
-------------------------------------------------------------------------------
parcels_delivered   parcels that reached their destination station
total_distance      sum of cells moved by all robots
completion_steps    tick at which the last parcel was delivered (= steps run if incomplete)
outcome             COMPLETED or TERMINATED; termination_kind = completed | step_limit |
                    stagnation | unreachable
oscillation_events  watchdog detections of a robot bouncing between <= 3 cells without progress
compute_time_ms     wall-clock time of the engine's step() calls (no rendering)
nodes_expanded      nodes expanded by every navigation search of every agent
invalid_moves       attempted moves into shelves/walls + illegal pickup/deliver actions
conflicts           ticks where a robot with a target made no progress while another
                    robot occupied a neighbouring cell on one of its shortest routes,
                    plus moves rejected by the conflict manager
completion_pct      100 * delivered / total

Search-benchmark metrics (one row per scenario x query x algorithm)
--------------------------------------------------------------------
path_length, nodes_expanded, nodes_generated, runtime_ms (min over repeats), found
"""

from __future__ import annotations

import pandas as pd

RUN_METRICS: tuple[str, ...] = (
    "parcels_delivered",
    "total_distance",
    "completion_steps",
    "compute_time_ms",
    "nodes_expanded",
    "invalid_moves",
    "conflicts",
    "completion_pct",
)

#: Columns that depend on wall-clock time and therefore differ between runs.
TIMING_COLUMNS: tuple[str, ...] = ("compute_time_ms", "search_time_ms", "runtime_ms")


def agent_summary(runs: pd.DataFrame) -> pd.DataFrame:
    """Mean metrics per agent (averaged over scenarios, algorithms and policies)."""
    g = runs.groupby("agent_label", sort=False)
    out = g.agg(
        runs=("scenario_id", "count"),
        avg_delivered=("parcels_delivered", "mean"),
        avg_completion_pct=("completion_pct", "mean"),
        fully_completed_pct=("completed", lambda s: 100.0 * s.mean()),
        avg_completion_steps=("completion_steps", "mean"),
        avg_distance=("total_distance", "mean"),
        avg_conflicts=("conflicts", "mean"),
        avg_invalid_moves=("invalid_moves", "mean"),
        avg_nodes_expanded=("nodes_expanded", "mean"),
        avg_compute_ms=("compute_time_ms", "mean"),
    )
    return out.round(2).reset_index()


def config_summary(runs: pd.DataFrame) -> pd.DataFrame:
    """Mean metrics for every agent x algorithm x allocation configuration."""
    g = runs.groupby(["agent_label", "algorithm_label", "allocation_label"], sort=False)
    out = g.agg(
        avg_delivered=("parcels_delivered", "mean"),
        avg_completion_pct=("completion_pct", "mean"),
        avg_completion_steps=("completion_steps", "mean"),
        avg_distance=("total_distance", "mean"),
        avg_conflicts=("conflicts", "mean"),
        avg_nodes_expanded=("nodes_expanded", "mean"),
        avg_compute_ms=("compute_time_ms", "mean"),
    )
    return out.round(2).reset_index()


def search_summary(search: pd.DataFrame) -> pd.DataFrame:
    """Mean search metrics per algorithm and warehouse size (found paths only)."""
    found = search[search["found"]]
    g = found.groupby(["size_class", "algorithm_label"], sort=False)
    out = g.agg(
        queries=("query_id", "count"),
        avg_path_length=("path_length", "mean"),
        avg_nodes_expanded=("nodes_expanded", "mean"),
        avg_runtime_ms=("runtime_ms", "mean"),
    )
    overall = found.groupby("algorithm_label", sort=False).agg(
        queries=("query_id", "count"),
        avg_path_length=("path_length", "mean"),
        avg_nodes_expanded=("nodes_expanded", "mean"),
        avg_runtime_ms=("runtime_ms", "mean"),
    )
    overall.index = pd.MultiIndex.from_tuples(
        [("all", a) for a in overall.index], names=["size_class", "algorithm_label"]
    )
    return pd.concat([out, overall]).round(4).reset_index()


def path_optimality_check(search: pd.DataFrame) -> dict[str, int]:
    """Compare BFS and A* path lengths query by query (both should be optimal)."""
    pivot = search.pivot_table(
        index=["scenario_id", "query_id"], columns="algorithm", values="path_length",
        aggfunc="first",
    ).dropna()
    equal = int((pivot["bfs"] == pivot["astar"]).sum()) if not pivot.empty else 0
    return {"queries_compared": int(len(pivot)), "equal_length": equal,
            "different_length": int(len(pivot)) - equal}
