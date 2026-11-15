"""Reproducible experiment runner.

Two experiments are executed on the same set of seeded scenarios:

A. **Search benchmark** - BFS and A* answer identical point-to-point queries on
   the static shelf map (every robot start -> every parcel, and every parcel ->
   its destination station). This isolates the algorithms from agent behaviour.

B. **Simulation runs** - every agent x algorithm x allocation-policy
   configuration runs the full multi-robot delivery task on every scenario,
   starting from identical initial conditions (fresh :class:`Warehouse` built
   from the same :class:`Scenario`).

Runs are produced by a generator (:meth:`ExperimentRunner.iter_run`) so the
Pygame interface can execute them incrementally without freezing and without
background threads. Nothing is hard-coded: every number comes from executing
the code below.
"""

from __future__ import annotations

import argparse
import platform
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterator, Optional

import pandas as pd

from algorithms.pathfinder import SEARCH_FUNCTIONS
from config.settings import (
    AGENT_LABELS,
    AGENT_TYPES,
    ALGORITHM_LABELS,
    ALGORITHMS,
    ALLOCATION_LABELS,
    ALLOCATION_POLICIES,
    EXPERIMENT_BASE_SEED,
    EXPERIMENT_SCENARIOS,
    RESULTS_DIR,
    SEARCH_TIMING_REPEATS,
    SimulationConfig,
)
from environment.scenario_generator import Scenario, build_experiment_scenarios
from experiments.result_exporter import ExperimentResults, export_results
from simulation.simulation_engine import SimulationEngine

ROBOTS_BY_SIZE = {"small": 2, "medium": 3, "large": 3}


@dataclass(frozen=True)
class RunSpec:
    scenario_index: int
    agent_type: str
    algorithm: str
    allocation: str


@dataclass
class Progress:
    done: int
    total: int
    label: str

    @property
    def fraction(self) -> float:
        return self.done / self.total if self.total else 1.0


class ExperimentRunner:
    def __init__(
        self,
        num_scenarios: int = EXPERIMENT_SCENARIOS,
        base_seed: int = EXPERIMENT_BASE_SEED,
        agents: tuple[str, ...] = AGENT_TYPES,
        algorithms: tuple[str, ...] = ALGORITHMS,
        allocations: tuple[str, ...] = ALLOCATION_POLICIES,
        timing_repeats: int = SEARCH_TIMING_REPEATS,
    ) -> None:
        if num_scenarios < 1:
            raise ValueError("num_scenarios must be positive")
        self.num_scenarios = num_scenarios
        self.base_seed = base_seed
        self.agents = agents
        self.algorithms = algorithms
        self.allocations = allocations
        self.timing_repeats = max(1, timing_repeats)
        self.scenarios: list[Scenario] = build_experiment_scenarios(num_scenarios, base_seed)
        self.specs = [
            RunSpec(i, ag, alg, alloc)
            for i in range(len(self.scenarios))
            for ag in agents
            for alg in algorithms
            for alloc in allocations
        ]
        self.run_rows: list[dict] = []
        self.search_rows: list[dict] = []
        self.started_at: Optional[str] = None
        self.elapsed_s = 0.0

    # ------------------------------------------------------------------ #
    # Experiment A: search benchmark
    # ------------------------------------------------------------------ #
    @staticmethod
    def search_queries(scenario: Scenario) -> list[tuple[str, tuple[int, int], tuple[int, int]]]:
        queries = []
        for r, start in enumerate(scenario.robot_starts):
            for p, spec in enumerate(scenario.parcels):
                queries.append((f"R{r}->P{p}", start, spec.position))
        for p, spec in enumerate(scenario.parcels):
            queries.append((f"P{p}->D{spec.destination_id}", spec.position,
                            scenario.stations[spec.destination_id]))
        return queries

    def benchmark_scenario(self, index: int) -> list[dict]:
        scenario = self.scenarios[index]
        rows = []
        for query_id, start, goal in self.search_queries(scenario):
            for alg in self.algorithms:
                fn = SEARCH_FUNCTIONS[alg]
                best = None
                for _ in range(self.timing_repeats):
                    res = fn(scenario.grid, start, goal, ())
                    if best is None or res.runtime_ms < best.runtime_ms:
                        best = res
                assert best is not None
                rows.append({
                    "scenario_id": scenario.scenario_id,
                    "seed": scenario.seed,
                    "size_class": scenario.size_class,
                    "query_id": query_id,
                    "algorithm": alg,
                    "algorithm_label": ALGORITHM_LABELS[alg],
                    "start": str(start),
                    "goal": str(goal),
                    "found": best.found,
                    "path_length": best.cost if best.found else None,
                    "nodes_expanded": best.nodes_expanded,
                    "nodes_generated": best.nodes_generated,
                    "runtime_ms": round(best.runtime_ms, 5),
                })
        return rows

    # ------------------------------------------------------------------ #
    # Experiment B: full simulations
    # ------------------------------------------------------------------ #
    def execute(self, spec: RunSpec) -> dict:
        scenario = self.scenarios[spec.scenario_index]
        config = SimulationConfig(
            layout=scenario.size_class,
            agent_type=spec.agent_type,
            algorithm=spec.algorithm,
            allocation=spec.allocation,
            num_robots=ROBOTS_BY_SIZE.get(scenario.size_class, 3),
            seed=scenario.seed,
        )
        engine = SimulationEngine(scenario, config, record_history=False).run()
        row = {
            "scenario_id": scenario.scenario_id,
            "seed": scenario.seed,
            "size_class": scenario.size_class,
            "width": scenario.width,
            "height": scenario.height,
            "shelves": scenario.shelf_count,
            "stations": len(scenario.stations),
            "robots": config.num_robots,
            "agent": spec.agent_type,
            "agent_label": AGENT_LABELS[spec.agent_type],
            "algorithm": spec.algorithm,
            "algorithm_label": ALGORITHM_LABELS[spec.algorithm],
            "allocation": spec.allocation,
            "allocation_label": ALLOCATION_LABELS[spec.allocation],
            "max_steps": engine.max_steps,
        }
        row.update(engine.metrics())
        return row

    # ------------------------------------------------------------------ #
    @property
    def total_units(self) -> int:
        return len(self.scenarios) + len(self.specs)

    def iter_run(self) -> Iterator[Progress]:
        """Execute everything, yielding after each unit of work."""
        self.run_rows, self.search_rows = [], []
        self.started_at = datetime.now().isoformat(timespec="seconds")
        t0 = time.perf_counter()
        done = 0
        for i, sc in enumerate(self.scenarios):
            self.search_rows.extend(self.benchmark_scenario(i))
            done += 1
            yield Progress(done, self.total_units, f"Search benchmark {sc.scenario_id}")
        for spec in self.specs:
            self.run_rows.append(self.execute(spec))
            done += 1
            sc = self.scenarios[spec.scenario_index]
            yield Progress(done, self.total_units,
                           f"{sc.scenario_id} | {AGENT_LABELS[spec.agent_type]} | "
                           f"{ALGORITHM_LABELS[spec.algorithm]} | {ALLOCATION_LABELS[spec.allocation]}")
        self.elapsed_s = time.perf_counter() - t0

    def results(self) -> ExperimentResults:
        meta = {
            "project": "Smart Warehouse Robot Simulator",
            "generated_at": self.started_at,
            "elapsed_seconds": round(self.elapsed_s, 2),
            "base_seed": self.base_seed,
            "num_scenarios": len(self.scenarios),
            "agents": list(self.agents),
            "algorithms": list(self.algorithms),
            "allocations": list(self.allocations),
            "timing_repeats": self.timing_repeats,
            "robots_by_size": ROBOTS_BY_SIZE,
            "python": sys.version.split()[0],
            "platform": platform.platform(),
            "scenarios": [
                {"scenario_id": s.scenario_id, "seed": s.seed, "size_class": s.size_class,
                 "width": s.width, "height": s.height, "shelves": s.shelf_count,
                 "parcels": len(s.parcels), "stations": len(s.stations),
                 "robot_starts": [list(p) for p in s.robot_starts]}
                for s in self.scenarios
            ],
        }
        return ExperimentResults(pd.DataFrame(self.run_rows), pd.DataFrame(self.search_rows), meta)

    def run_all(self, verbose: bool = False) -> ExperimentResults:
        for progress in self.iter_run():
            if verbose and (progress.done % 25 == 0 or progress.done == progress.total):
                print(f"[{progress.done:4d}/{progress.total}] {progress.label}", flush=True)
        return self.results()


def run_and_export(out_dir: Path = RESULTS_DIR, num_scenarios: int = EXPERIMENT_SCENARIOS,
                   base_seed: int = EXPERIMENT_BASE_SEED, charts: bool = True,
                   verbose: bool = True) -> ExperimentResults:
    runner = ExperimentRunner(num_scenarios=num_scenarios, base_seed=base_seed)
    results = runner.run_all(verbose=verbose)
    paths = export_results(results, out_dir)
    if charts:
        from experiments.charts import generate_all_charts
        chart_paths = generate_all_charts(results, Path(out_dir) / "charts")
        paths.update({f"chart:{p.stem}": p for p in chart_paths})
    if verbose:
        print(f"Finished in {runner.elapsed_s:.1f}s")
        for key, p in paths.items():
            print(f"  {key:22s} {p}")
    return results


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run the reproducible warehouse experiments.")
    parser.add_argument("--scenarios", type=int, default=EXPERIMENT_SCENARIOS)
    parser.add_argument("--seed", type=int, default=EXPERIMENT_BASE_SEED)
    parser.add_argument("--out", type=Path, default=RESULTS_DIR)
    parser.add_argument("--no-charts", action="store_true")
    args = parser.parse_args(argv)
    run_and_export(args.out, args.scenarios, args.seed, charts=not args.no_charts)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
