"""Smart Warehouse Robot Simulator - entry point.

Examples
--------
python main.py                                   # launch the interactive dashboard
python main.py --layout large --agent utility_based --algorithm bfs --robots 2
python main.py --experiments                     # run the 24-scenario experiment suite
python main.py --headless --layout medium        # run one simulation without a window
python main.py --screenshot docs/screenshots/simulation.png --ticks 25
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config.settings import (  # noqa: E402
    AGENT_TYPES,
    ALGORITHMS,
    ALLOCATION_POLICIES,
    DEFAULT_ROBOTS,
    DEFAULT_SEED,
    EXPERIMENT_BASE_SEED,
    EXPERIMENT_SCENARIOS,
    MAX_ROBOTS,
    MIN_ROBOTS,
    RESULTS_DIR,
    SimulationConfig,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Smart Warehouse Robot Simulator (AL-2002 AI Lab)")
    p.add_argument("--layout", choices=["small", "medium", "large", "random"], default="medium")
    p.add_argument("--agent", choices=AGENT_TYPES, default="goal_based")
    p.add_argument("--algorithm", choices=ALGORITHMS, default="astar")
    p.add_argument("--allocation", choices=ALLOCATION_POLICIES, default="nearest")
    p.add_argument("--robots", type=int, choices=range(MIN_ROBOTS, MAX_ROBOTS + 1), default=DEFAULT_ROBOTS)
    p.add_argument("--seed", type=int, default=DEFAULT_SEED, help="seed for the random layout / agents")
    p.add_argument("--experiments", action="store_true", help="run the experiment suite and exit")
    p.add_argument("--scenarios", type=int, default=EXPERIMENT_SCENARIOS)
    p.add_argument("--base-seed", type=int, default=EXPERIMENT_BASE_SEED)
    p.add_argument("--headless", action="store_true", help="run one simulation without a window and print metrics")
    p.add_argument("--screenshot", type=Path, help="render the dashboard headlessly and save a PNG")
    p.add_argument("--ticks", type=int, default=20, help="ticks to simulate before --screenshot")
    p.add_argument("--view", choices=["simulation", "analytics"], default="simulation")
    p.add_argument("--select", type=int, default=0, help="robot selected in --screenshot")
    p.add_argument("--tab", type=int, default=0, help="right panel tab in --screenshot (0-3)")
    p.add_argument("--compare", action="store_true", help="enable BFS vs A* overlay in --screenshot")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    config = SimulationConfig(layout=args.layout, agent_type=args.agent, algorithm=args.algorithm,
                              allocation=args.allocation, num_robots=args.robots, seed=args.seed)

    if args.experiments:
        from experiments.experiment_runner import run_and_export
        run_and_export(RESULTS_DIR, args.scenarios, args.base_seed)
        return 0

    if args.headless:
        from simulation.simulation_controller import SimulationController
        ctl = SimulationController(config)
        engine = ctl.engine.run()
        print(f"Scenario: {ctl.scenario.name}")
        print(json.dumps(engine.metrics(), indent=2))
        return 0

    if args.screenshot:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        from interface.dashboard import Dashboard
        app = Dashboard(config, headless=True)
        app.screen_mode = args.view
        app.selected = args.select
        app.robot_panel.tab = args.tab
        if args.compare:
            app._on_view("compare", True)
        for _ in range(args.ticks):
            app.ctl.step_once()
            events = app.ctl.update(0.0)
        app.view.add_events(events if args.ticks else [])  # effects of the final tick only
        app.ctl.last_tick_time = 0.0  # show the settled (post-animation) state
        app.render()
        app.save_screenshot(args.screenshot)
        print(f"Saved {args.screenshot}")
        return 0

    from interface.dashboard import launch
    launch(config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
