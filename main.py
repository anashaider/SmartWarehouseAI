"""Smart Warehouse Robot Simulator - entry point.

Examples
--------
python main.py                                   # launch the 3D digital twin (falls back to 2D)
python main.py --2d                              # classic 2D Pygame dashboard
python main.py --quality high --present          # 3D, high graphics, start in Presentation Mode
python main.py --layout large --agent utility_based --algorithm bfs --robots 2
python main.py --experiments                     # run the 24-scenario experiment suite
python main.py --headless --layout medium        # run one simulation without a window
python main.py --screenshot docs/screenshots/simulation.png --ticks 25          # 2D, no window
python main.py --screenshot3d docs/screenshots/3d_overview.png --ticks 25        # 3D, offscreen
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
    p.add_argument("--max-steps", type=int, default=None,
                   help="hard tick limit for one run (default: derived from the layout size)")
    p.add_argument("--stagnation-limit", type=int, default=None,
                   help="ticks without task progress before a run is terminated as incomplete "
                        "(default: derived from the layout size; 0 disables the watchdog)")
    p.add_argument("--trace", action="store_true",
                   help="with --headless: print a per-tick log of positions, actions, percepts and targets")
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
    p.add_argument("--2d", dest="two_d", action="store_true", help="use the classic 2D Pygame dashboard")
    p.add_argument("--quality", choices=["low", "medium", "high"], default="medium", help="3D graphics quality")
    p.add_argument("--no-intro", action="store_true", help="skip the 3D opening flythrough")
    p.add_argument("--present", action="store_true", help="start the 3D app in Presentation Mode")
    p.add_argument("--screenshot3d", type=Path, help="render the 3D digital twin offscreen and save a PNG")
    p.add_argument("--camera", choices=["overview", "topdown", "tracking", "follow", "free", "focus"], default="overview",
                   help="camera mode for --screenshot3d")
    p.add_argument("--size", default="1600x900", help="window size for --screenshot3d, e.g. 1920x1080")
    return p.parse_args(argv)


def launch_3d(config: SimulationConfig, args: argparse.Namespace) -> bool:
    """Start the 3D digital twin. Returns False if 3D is unavailable (caller falls back to 2D)."""
    from interface3d import panda3d_available
    ok, reason = panda3d_available()
    if not ok:
        print(f"[3D] {reason}")
        print("[3D] Falling back to the 2D dashboard.")
        return False
    try:
        from interface3d.app3d import WarehouseApp3D
        app = WarehouseApp3D(config, quality=args.quality, intro=not args.no_intro, size=_window_size())
    except Exception as exc:  # noqa: BLE001 - any graphics failure -> 2D fallback
        print(f"[3D] Could not start the 3D renderer ({exc}).")
        print("[3D] Falling back to the 2D dashboard.")
        return False
    if args.present:
        app.cmd_presentation()
    app.run()
    return True


def _window_size() -> tuple[int, int]:
    """About 90 % of the primary screen, 16:9, between 1280x720 and 1920x1080."""
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)  # type: ignore[attr-defined]
        sw = ctypes.windll.user32.GetSystemMetrics(0)  # type: ignore[attr-defined]
        sh = ctypes.windll.user32.GetSystemMetrics(1)  # type: ignore[attr-defined]
    except (AttributeError, OSError):
        sw, sh = 1600, 900
    w = max(1280, min(1920, int(sw * 0.9)))
    h = max(720, min(1080, int(min(sh * 0.86, w * 9 / 16))))
    return (int(h * 16 / 9) if int(h * 16 / 9) <= sw else w, h)


def screenshot_3d(config: SimulationConfig, args: argparse.Namespace) -> int:
    from interface3d.app3d import WarehouseApp3D
    w, h = (int(v) for v in args.size.lower().split("x"))
    app = WarehouseApp3D(config, offscreen=True, size=(w, h), quality=args.quality, intro=False)
    app.cmd_select(args.select)
    if args.compare:
        app.cmd_toggle_compare()
    for _ in range(args.ticks):
        app.ctl.step_once()
        app.frame(1 / 60)
    app.cmd_camera(args.camera)
    if args.present:
        app.cmd_presentation()
        app.ctl.pause()
    if args.view == "analytics":
        app.cmd_analytics()
    for _ in range(150):          # let the camera settle (no simulation ticks while paused)
        app.frame(1 / 30)
    app.screenshot(args.screenshot3d)
    print(f"Saved {args.screenshot3d}")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    config = SimulationConfig(layout=args.layout, agent_type=args.agent, algorithm=args.algorithm,
                              allocation=args.allocation, num_robots=args.robots, seed=args.seed,
                              max_steps=args.max_steps, stagnation_limit=args.stagnation_limit)
    try:
        config.validate()
    except ValueError as exc:
        print(f"Invalid configuration: {exc}")
        return 2

    if args.experiments:
        from experiments.experiment_runner import run_and_export
        run_and_export(RESULTS_DIR, args.scenarios, args.base_seed)
        return 0

    if args.headless:
        from simulation.simulation_controller import SimulationController
        from simulation.run_trace import format_tick
        ctl = SimulationController(config)
        engine = ctl.engine
        print(f"Scenario: {ctl.scenario.name}")
        while not engine.done:
            engine.step()
            if args.trace:
                print(format_tick(engine))
        print(f"Outcome: {engine.outcome} ({engine.termination_reason})")
        print(json.dumps(engine.metrics(), indent=2))
        return 0

    if args.screenshot3d:
        return screenshot_3d(config, args)

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

    if not args.two_d and launch_3d(config, args):
        return 0
    from interface.dashboard import launch
    launch(config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
