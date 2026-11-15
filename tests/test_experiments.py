"""Experiment tests: reproducibility, exports, metric calculations, scenario count."""

from __future__ import annotations

import json

import pandas as pd
import pytest

from config.settings import EXPERIMENT_SCENARIOS
from environment.scenario_generator import build_experiment_scenarios
from experiments.experiment_runner import ExperimentRunner
from experiments.metrics import TIMING_COLUMNS, agent_summary, path_optimality_check, search_summary
from experiments.result_exporter import export_results, load_results


@pytest.fixture(scope="module")
def small_results():
    runner = ExperimentRunner(num_scenarios=3, base_seed=77, timing_repeats=1)
    return runner.run_all()


def test_at_least_twenty_unique_seeded_scenarios():
    assert EXPERIMENT_SCENARIOS >= 20
    scenarios = build_experiment_scenarios(EXPERIMENT_SCENARIOS, 2026)
    assert len(scenarios) == EXPERIMENT_SCENARIOS
    assert len({s.scenario_id for s in scenarios}) == EXPERIMENT_SCENARIOS
    assert len({s.seed for s in scenarios}) == EXPERIMENT_SCENARIOS
    assert {s.size_class for s in scenarios} == {"small", "medium", "large"}


def test_run_matrix_shape(small_results):
    runs = small_results.runs
    assert len(runs) == 3 * 3 * 2 * 2
    assert set(runs["agent"]) == {"simple_reflex", "goal_based", "utility_based"}
    assert set(runs["algorithm"]) == {"bfs", "astar"}
    assert set(runs["allocation"]) == {"nearest", "fixed_order"}


def _deterministic(df: pd.DataFrame) -> pd.DataFrame:
    return df.drop(columns=[c for c in TIMING_COLUMNS if c in df.columns]).reset_index(drop=True)


def test_fixed_seed_reproducibility(small_results):
    again = ExperimentRunner(num_scenarios=3, base_seed=77, timing_repeats=1).run_all()
    pd.testing.assert_frame_equal(_deterministic(small_results.runs), _deterministic(again.runs))
    pd.testing.assert_frame_equal(_deterministic(small_results.search), _deterministic(again.search))


def test_metric_consistency(small_results):
    runs = small_results.runs
    assert (runs["parcels_delivered"] <= runs["parcels_total"]).all()
    expected_pct = (100 * runs["parcels_delivered"] / runs["parcels_total"]).round(2)
    assert (runs["completion_pct"] == expected_pct).all()
    assert (runs["parcels_pending"] == runs["parcels_total"] - runs["parcels_delivered"]).all()
    completed = runs[runs["completed"]]
    assert (completed["completion_pct"] == 100).all()
    assert (completed["completion_steps"] <= completed["max_steps"]).all()
    reflex = runs[runs["agent"] == "simple_reflex"]
    assert (reflex["nodes_expanded"] == 0).all()
    planners = runs[runs["agent"] != "simple_reflex"]
    assert (planners["nodes_expanded"] > 0).all()
    assert (runs["compute_time_ms"] > 0).all()
    # Moving robots travel at least the distance implied by their deliveries.
    assert (runs.loc[runs["parcels_delivered"] > 0, "total_distance"] > 0).all()


def test_search_benchmark_bfs_and_astar_agree(small_results):
    check = path_optimality_check(small_results.search)
    assert check["queries_compared"] > 0 and check["different_length"] == 0
    summ = search_summary(small_results.search)
    overall = summ[summ["size_class"] == "all"].set_index("algorithm_label")
    assert overall.loc["A*", "avg_nodes_expanded"] <= overall.loc["BFS", "avg_nodes_expanded"]


def test_summary_aggregation(small_results):
    summ = agent_summary(small_results.runs)
    assert list(summ["agent_label"]) == ["Simple Reflex", "Goal-Based", "Utility-Based"]
    for row in summ.itertuples():
        sub = small_results.runs[small_results.runs["agent_label"] == row.agent_label]
        assert row.avg_delivered == pytest.approx(sub["parcels_delivered"].mean(), abs=0.01)


def test_csv_and_json_generation(small_results, tmp_path):
    paths = export_results(small_results, tmp_path)
    for key in ("runs_csv", "search_csv", "agent_summary_csv", "json"):
        assert paths[key].exists() and paths[key].stat().st_size > 0
    runs = pd.read_csv(paths["runs_csv"])
    assert len(runs) == len(small_results.runs)
    data = json.loads(paths["json"].read_text(encoding="utf-8"))
    assert {"metadata", "summary", "runs", "search_benchmark"} <= set(data)
    assert data["metadata"]["num_scenarios"] == 3 and data["metadata"]["base_seed"] == 77
    loaded, msg = load_results(tmp_path)
    assert loaded is not None and len(loaded.runs) == len(runs)


def test_load_results_handles_missing_and_corrupted(tmp_path):
    res, msg = load_results(tmp_path)
    assert res is None and "No saved" in msg
    (tmp_path / "simulation_runs.csv").write_text("garbage\n\x00\x00", encoding="utf-8")
    (tmp_path / "search_benchmark.csv").write_text("a,b\n1,2\n", encoding="utf-8")
    res, msg = load_results(tmp_path)
    assert res is None


def test_charts_generated_from_results(small_results, tmp_path):
    from experiments.charts import generate_all_charts
    paths = generate_all_charts(small_results, tmp_path)
    assert len(paths) >= 3 and all(p.exists() and p.stat().st_size > 10_000 for p in paths)


def test_iterative_runner_reports_progress():
    runner = ExperimentRunner(num_scenarios=1, base_seed=5, timing_repeats=1,
                              agents=("goal_based",), algorithms=("astar",), allocations=("nearest",))
    progress = list(runner.iter_run())
    assert progress[-1].done == progress[-1].total == runner.total_units == 2
    assert progress[-1].fraction == 1.0
