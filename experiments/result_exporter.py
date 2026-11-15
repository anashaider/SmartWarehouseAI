"""Saving and loading experiment results (CSV + JSON) with safe error handling."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import pandas as pd

from config.settings import RESULTS_DIR

RUNS_CSV = "simulation_runs.csv"
SEARCH_CSV = "search_benchmark.csv"
AGENT_SUMMARY_CSV = "summary_agents.csv"
CONFIG_SUMMARY_CSV = "summary_configurations.csv"
SEARCH_SUMMARY_CSV = "summary_search.csv"
RESULTS_JSON = "experiment_results.json"


@dataclass
class ExperimentResults:
    runs: pd.DataFrame
    search: pd.DataFrame
    metadata: dict[str, Any] = field(default_factory=dict)


def export_results(results: ExperimentResults, out_dir: Path = RESULTS_DIR) -> dict[str, Path]:
    """Write all CSV/JSON outputs; returns the created file paths."""
    from experiments.metrics import agent_summary, config_summary, search_summary

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = {
        "runs_csv": out_dir / RUNS_CSV,
        "search_csv": out_dir / SEARCH_CSV,
        "agent_summary_csv": out_dir / AGENT_SUMMARY_CSV,
        "config_summary_csv": out_dir / CONFIG_SUMMARY_CSV,
        "search_summary_csv": out_dir / SEARCH_SUMMARY_CSV,
        "json": out_dir / RESULTS_JSON,
    }
    results.runs.to_csv(paths["runs_csv"], index=False)
    results.search.to_csv(paths["search_csv"], index=False)
    agents = agent_summary(results.runs)
    configs = config_summary(results.runs)
    search = search_summary(results.search)
    agents.to_csv(paths["agent_summary_csv"], index=False)
    configs.to_csv(paths["config_summary_csv"], index=False)
    search.to_csv(paths["search_summary_csv"], index=False)

    payload = {
        "metadata": results.metadata,
        "summary": {
            "agents": agents.to_dict(orient="records"),
            "configurations": configs.to_dict(orient="records"),
            "search": search.to_dict(orient="records"),
        },
        "runs": results.runs.to_dict(orient="records"),
        "search_benchmark": results.search.to_dict(orient="records"),
    }
    tmp = paths["json"].with_suffix(".json.tmp")
    tmp.write_text(json.dumps(payload, indent=2, default=_json_default), encoding="utf-8")
    tmp.replace(paths["json"])  # atomic: never leaves a half-written results file
    return paths


def _json_default(obj: Any) -> Any:
    if hasattr(obj, "item"):  # numpy scalars
        return obj.item()
    return str(obj)


def load_results(out_dir: Path = RESULTS_DIR) -> tuple[Optional[ExperimentResults], str]:
    """Load saved results. Returns ``(results, message)``; ``results`` is ``None``
    when the files are missing or corrupted (the message explains why)."""
    out_dir = Path(out_dir)
    runs_path, search_path, json_path = out_dir / RUNS_CSV, out_dir / SEARCH_CSV, out_dir / RESULTS_JSON
    if not runs_path.exists() or not search_path.exists():
        return None, "No saved experiment results yet. Run the experiments first."
    try:
        runs = pd.read_csv(runs_path)
        search = pd.read_csv(search_path)
    except (OSError, ValueError, pd.errors.ParserError) as exc:
        return None, f"Saved results are unreadable ({exc}). Re-run the experiments."
    required_runs = {"scenario_id", "agent_label", "parcels_delivered", "completion_pct"}
    required_search = {"scenario_id", "algorithm", "path_length", "nodes_expanded", "found"}
    if not required_runs <= set(runs.columns) or not required_search <= set(search.columns):
        return None, "Saved results have an unexpected format. Re-run the experiments."
    metadata: dict[str, Any] = {}
    try:
        metadata = json.loads(json_path.read_text(encoding="utf-8")).get("metadata", {})
    except (OSError, ValueError, AttributeError):
        pass  # metadata is optional; CSVs are the source of truth
    return ExperimentResults(runs, search, metadata), f"Loaded {len(runs)} simulation runs."
