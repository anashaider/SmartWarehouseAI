"""Regression tests: Simple Reflex looping on the last parcel.

Root cause (found by tracing small layout, 3 robots): after its last delivery
an idle robot stays parked on the drop-off station. The carrier, a Simple
Reflex agent, never bumps into it - rule 5 side-steps to a random free cell
instead - so no rejected move / WAIT was recorded, the coordination layer never
saw the obstruction, the idle robot was never told to yield and the carrier
bounced around the station until the hard step limit.

Fix: the engine registers an *idle* robot on a working robot's shortest route
as a blocker from the percept, so the documented "idle robot yields at once"
rule applies to every agent. A run watchdog (oscillation, repeated state,
stagnation) ends genuinely stuck runs early and honestly as TERMINATED.
"""

from __future__ import annotations

import pytest

from agents.simple_reflex import SimpleReflexAgent
from config.settings import SimulationConfig
from environment.scenario_generator import load_predefined
from simulation.progress_monitor import ProgressMonitor, default_stagnation_limit
from simulation.simulation_controller import SimulationController
from simulation.simulation_engine import SimulationEngine
from simulation.state_manager import SimStatus
from tests.conftest import make_scenario


def run(layout_or_scenario, agent="simple_reflex", robots=3, seed=42, **cfg) -> SimulationEngine:
    sc = load_predefined(layout_or_scenario) if isinstance(layout_or_scenario, str) else layout_or_scenario
    config = SimulationConfig(agent_type=agent, num_robots=robots, seed=seed, **cfg)
    return SimulationEngine(sc, config, record_history=False).run()


# --------------------------------------------------------------------------- #
# The exact reported issue
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("allocation", ["nearest", "fixed_order"])
def test_small_layout_last_parcel_no_longer_loops(allocation):
    """Previously: 5/6 delivered by tick 29, then looping until the 300-tick limit."""
    eng = run("small", allocation=allocation)
    m = eng.metrics()
    assert m["completed"] and m["parcels_delivered"] == 6
    assert eng.outcome == "COMPLETED" and eng.termination_kind == "completed"
    assert m["steps"] < 100                       # was 300 (step limit)
    assert m["deadlocks_resolved"] >= 1           # the idle robot was told to yield


def test_simple_reflex_idle_robot_on_station_yields():
    """Idle R1 starts on the only station; the reflex carrier side-steps instead
    of bumping into it. The coordination layer must still make R1 yield."""
    sc = make_scenario(["R.P..", ".....", "..D.."], parcels=[((0, 2), 0)],
                       robots=[(0, 0), (2, 2)])
    eng = run(sc, robots=2, max_steps=100)
    assert eng.metrics()["completed"]
    assert any(e.kind == "yield" and e.robot_id == 1 for e in eng.event_log)


def test_idle_obstruction_registered_without_rejected_move():
    sc = make_scenario(["R.P..", ".....", "..D.."], parcels=[((0, 2), 0)],
                       robots=[(0, 0), (2, 2)])
    eng = SimulationEngine(sc, SimulationConfig(agent_type="simple_reflex", num_robots=2), max_steps=100)
    while not eng.done:
        eng.step()
        if eng._blocked_on:
            # The carrier registered the idle robot as blocker although its own
            # move was a side-step (not rejected and not a WAIT).
            assert eng._blocked_on == {0: 1}
            assert eng.rejected_moves == 0
            return
    pytest.fail("idle-robot obstruction was never registered")


# --------------------------------------------------------------------------- #
# Watchdog: oscillation, repeated state, stagnation, honest termination
# --------------------------------------------------------------------------- #
def test_stuck_reflex_run_terminates_honestly_on_stagnation():
    eng = run("large")
    m = eng.metrics()
    assert eng.outcome == "TERMINATED" and not m["completed"]
    assert eng.termination_kind == "stagnation"
    assert "incomplete" in eng.termination_reason and "no task progress" in eng.termination_reason
    assert m["parcels_delivered"] < m["parcels_total"]
    assert m["steps"] < eng.max_steps
    assert m["oscillation_events"] >= 1 and m["repeated_state_events"] >= 1
    assert any(e.kind == "oscillation" for e in eng.event_log)


@pytest.mark.parametrize("layout,robots", [("medium", 2), ("large", 3)])
def test_watchdog_never_cuts_off_progress(layout, robots):
    """Disabling the watchdog must not deliver a single extra parcel."""
    on = run(layout, robots=robots)
    off = run(layout, robots=robots, stagnation_limit=0)
    assert off.termination_kind == "step_limit"
    assert on.metrics()["parcels_delivered"] == off.metrics()["parcels_delivered"]


def test_stagnation_limit_is_configurable():
    eng = run("large", stagnation_limit=30)
    assert eng.termination_kind == "stagnation"
    assert eng.monitor.ticks_without_progress == 30
    assert "stagnation limit 30" in eng.termination_reason


def test_max_steps_is_configurable():
    eng = run("large", max_steps=25)
    assert eng.step_count == 25 and eng.termination_kind == "step_limit"
    assert eng.outcome == "TERMINATED"
    assert eng.max_steps == 25


@pytest.mark.parametrize("bad", [{"max_steps": 0}, {"max_steps": -5}, {"stagnation_limit": -1}])
def test_invalid_limits_rejected(bad):
    with pytest.raises(ValueError):
        SimulationConfig(**bad).validate()


def test_default_stagnation_limit_scales_with_layout():
    small, large = load_predefined("small"), load_predefined("large")
    assert 60 <= default_stagnation_limit(small) < default_stagnation_limit(large)


class _Robot:
    def __init__(self, rid, pos):
        self.robot_id, self.position = rid, pos
        self.assigned_parcel, self.carrying = 0, None


class _Warehouse:
    """Minimal stand-in: one robot with a fixed target and no deliveries."""

    def __init__(self, robot, target):
        self.robots, self.parcels, self._target = [robot], {}, target

    def delivered_count(self):
        return 0

    def parcel_target(self, robot):
        return self._target


def test_monitor_detects_two_cell_oscillation_and_repeated_state():
    robot = _Robot(0, (1, 1))
    wh = _Warehouse(robot, (0, 0))
    mon = ProgressMonitor(stagnation_limit=40, window=6, max_cells=3, repeat_threshold=3)
    dist = lambda cell, target: abs(cell[0] - target[0]) + abs(cell[1] - target[1])  # noqa: E731
    kinds = []
    for step in range(1, 60):  # step 1 sets the baseline; 40 stalled ticks follow
        robot.position = (1, 1) if step % 2 else (1, 2)          # bounce, never closer
        kinds += [e.kind for e in mon.observe(step, wh, dist)]
        if mon.stagnated:
            break
    assert "oscillation" in kinds and "repeated_state" in kinds
    assert kinds.count("oscillation") == 1 and kinds.count("repeated_state") == 1  # reported once
    assert 0 in mon.oscillating
    assert mon.stagnated and mon.ticks_without_progress == 40


def test_monitor_counts_new_best_distance_as_progress():
    robot = _Robot(0, (5, 5))
    wh = _Warehouse(robot, (0, 0))
    mon = ProgressMonitor(stagnation_limit=10, window=4)
    dist = lambda cell, target: abs(cell[0] - target[0]) + abs(cell[1] - target[1])  # noqa: E731
    path = [(5, 5), (5, 6), (5, 7), (5, 6), (5, 5), (4, 5), (3, 5)]   # detour, then closer
    for step, cell in enumerate(path, start=1):
        robot.position = cell
        mon.observe(step, wh, dist)
    assert mon.last_progress_step == len(path)
    assert not mon.stagnated and not mon.oscillating


# --------------------------------------------------------------------------- #
# UI status: RUNNING / COMPLETED / TERMINATED
# --------------------------------------------------------------------------- #
def _finish(ctl: SimulationController) -> None:
    ctl.start()
    assert ctl.display_status == "RUNNING"
    for _ in range(5000):
        ctl.update(1.0)
        if ctl.status == SimStatus.FINISHED:
            return
    pytest.fail("run did not finish")


def test_controller_reports_completed():
    ctl = SimulationController(SimulationConfig(layout="small", agent_type="goal_based"))
    assert ctl.display_status == "READY"
    _finish(ctl)
    assert ctl.display_status == "COMPLETED"


def test_controller_reports_terminated_for_incomplete_run():
    ctl = SimulationController(SimulationConfig(layout="large", agent_type="simple_reflex"))
    _finish(ctl)
    assert ctl.display_status == "TERMINATED"
    assert ctl.engine.metrics()["outcome"] == "TERMINATED"
    ctl.reset()
    assert ctl.display_status == "READY" and ctl.engine.monitor.ticks_without_progress == 0


# --------------------------------------------------------------------------- #
# Fair baseline: the agent itself is unchanged
# --------------------------------------------------------------------------- #
def test_simple_reflex_agent_stays_memoryless_and_search_free():
    eng = run("large")
    agent = eng.agents[0]
    assert isinstance(agent, SimpleReflexAgent)
    assert set(vars(agent)) == {"robot_id", "grid", "pathfinder", "seed", "rng"}
    assert eng.metrics()["nodes_expanded"] == 0 and eng.metrics()["searches"] == 0


@pytest.mark.parametrize("agent", ["goal_based", "utility_based"])
@pytest.mark.parametrize("layout", ["small", "medium", "large"])
def test_planning_agents_still_complete(agent, layout):
    eng = run(layout, agent=agent)
    assert eng.outcome == "COMPLETED" and eng.termination_kind == "completed"
