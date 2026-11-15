"""Simulation engine, controller and headless interface tests."""

from __future__ import annotations

import pygame
import pytest

from config.settings import SimulationConfig
from environment.robot import RobotState
from environment.scenario_generator import load_predefined
from simulation.simulation_controller import SimulationController
from simulation.simulation_engine import SimulationEngine
from simulation.state_manager import SimStatus, StateManager


@pytest.mark.parametrize("algorithm", ["bfs", "astar"])
@pytest.mark.parametrize("agent", ["goal_based", "utility_based"])
@pytest.mark.parametrize("layout", ["small", "medium", "large"])
def test_planning_agents_deliver_everything(layout, agent, algorithm):
    eng = SimulationEngine(load_predefined(layout),
                           SimulationConfig(agent_type=agent, algorithm=algorithm, num_robots=3),
                           record_history=False).run()
    m = eng.metrics()
    assert m["completed"] and m["parcels_delivered"] == m["parcels_total"]
    assert m["completion_pct"] == 100.0 and m["invalid_moves"] == 0
    for robot in eng.warehouse.robots:
        assert robot.state == RobotState.IDLE and robot.carrying is None


def test_bfs_and_astar_give_same_simulation_but_different_effort():
    sc = load_predefined("medium")
    a = SimulationEngine(sc, SimulationConfig(agent_type="goal_based", algorithm="astar")).run().metrics()
    b = SimulationEngine(sc, SimulationConfig(agent_type="goal_based", algorithm="bfs")).run().metrics()
    assert a["total_distance"] == b["total_distance"] or abs(a["total_distance"] - b["total_distance"]) < 20
    assert a["nodes_expanded"] < b["nodes_expanded"]


def test_delivery_counts_match_parcels():
    eng = SimulationEngine(load_predefined("small"), SimulationConfig(num_robots=2)).run()
    assert sum(r.deliveries_completed for r in eng.warehouse.robots) == eng.metrics()["parcels_delivered"]
    delivered = [rec.parcel_id for r in eng.warehouse.robots for rec in r.delivery_history]
    assert len(delivered) == len(set(delivered))  # never delivered twice


def test_reset_restores_identical_initial_state():
    eng = SimulationEngine(load_predefined("medium"), SimulationConfig(agent_type="utility_based"))
    first = eng.run().metrics()
    eng.reset()
    assert eng.step_count == 0 and eng.warehouse.delivered_count() == 0
    second = eng.run().metrics()
    for k in first:
        if k not in ("compute_time_ms", "search_time_ms"):
            assert first[k] == second[k], k


def test_decision_history_is_recorded_with_explanations():
    eng = SimulationEngine(load_predefined("small"), SimulationConfig(agent_type="utility_based", num_robots=2))
    for _ in range(10):
        eng.step()
    hist = list(eng.history[0])
    assert len(hist) == 10
    assert all(h.reason for h in hist)
    moves = [h for h in hist if h.chosen.is_move]
    assert moves and any(c.score is not None for c in moves[0].candidates)
    assert "Position" in moves[0].percept


def test_step_limit_terminates():
    eng = SimulationEngine(load_predefined("large"), SimulationConfig(agent_type="simple_reflex"),
                           max_steps=40).run()
    assert eng.step_count == 40 and "step limit" in eng.termination_reason


def test_state_manager_rules():
    sm = StateManager()
    assert sm.set(SimStatus.RUNNING) and sm.set(SimStatus.PAUSED)
    assert sm.set(SimStatus.FINISHED)
    assert not sm.set(SimStatus.RUNNING)  # finished -> running is illegal
    assert sm.set(SimStatus.READY)


def test_controller_commands_and_config_changes():
    ctl = SimulationController(SimulationConfig(layout="small", num_robots=2))
    assert ctl.status == SimStatus.READY
    ctl.start()
    ctl.update(1.0)  # run some ticks at the default speed
    assert ctl.status == SimStatus.RUNNING and ctl.engine.step_count > 0
    ctl.pause()
    n = ctl.engine.step_count
    ctl.update(1.0)
    assert ctl.engine.step_count == n  # paused: no ticks
    ctl.step_once()
    assert ctl.engine.step_count == n + 1
    ctl.resume()
    assert ctl.status == SimStatus.RUNNING
    ctl.reset()
    assert ctl.engine.step_count == 0 and ctl.status == SimStatus.READY
    ctl.apply_config(agent_type="utility_based", algorithm="bfs", num_robots=3, layout="random", seed=7)
    assert ctl.engine.config.agent_type == "utility_based" and len(ctl.engine.warehouse.robots) == 3
    ctl.apply_config(num_robots=9)  # invalid -> rejected, message set
    assert "num_robots" in ctl.message and len(ctl.engine.warehouse.robots) == 3


def test_controller_runs_to_finish():
    ctl = SimulationController(SimulationConfig(layout="small", num_robots=2))
    ctl.set_speed_index(8)
    ctl.start()
    for _ in range(400):
        ctl.update(0.1)
        if ctl.status == SimStatus.FINISHED:
            break
    assert ctl.status == SimStatus.FINISHED and ctl.engine.metrics()["completed"]


# ------------------------------------------------------------ interface (headless)
def _click(app, rect):
    for etype in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEBUTTONUP):
        app.handle_event(pygame.event.Event(etype, {"pos": rect.center, "button": 1}))


def test_dashboard_buttons_drive_the_real_simulation(tmp_path):
    from interface.dashboard import Dashboard
    app = Dashboard(SimulationConfig(layout="small", num_robots=2), headless=True)
    try:
        app.frame(0.016)
        _click(app, app.controls.btn_start.rect)
        assert app.ctl.status == SimStatus.RUNNING
        for _ in range(30):
            app.frame(0.05)
        assert app.ctl.engine.step_count > 0
        _click(app, app.controls.btn_pause.rect)
        assert app.ctl.status == SimStatus.PAUSED
        steps = app.ctl.engine.step_count
        _click(app, app.controls.btn_step.rect)
        assert app.ctl.engine.step_count == steps + 1
        _click(app, app.controls.btn_resume.rect)
        assert app.ctl.status == SimStatus.RUNNING
        _click(app, app.controls.btn_reset.rect)
        assert app.ctl.engine.step_count == 0
        # Selectors change the configuration.
        _click(app, app.controls.sel_agent._right)
        assert app.ctl.config.agent_type == "utility_based"
        _click(app, app.controls.sel_alg._right)
        assert app.ctl.config.algorithm == "bfs"
        _click(app, app.controls.sel_layout._right)
        assert app.ctl.config.layout == "medium"
        _click(app, app.controls.sel_robots._right)
        assert len(app.ctl.engine.warehouse.robots) == 3
        _click(app, app.controls.sel_alloc._right)
        assert app.ctl.config.allocation == "fixed_order"
        # Robot selection via panel chips and tabs render without errors.
        app.frame(0.016)  # chips are laid out during drawing
        rect, rid = app.robot_panel._chip_rects[2]
        _click(app, rect)
        assert app.selected == 2
        for tab in range(4):
            app.robot_panel.tab = tab
            app.ctl.step_once()
            app.frame(0.016)
        app._on_view("compare", True)
        app.frame(0.016)
        assert app.view.comparison is not None
        # Analytics screen renders.
        _click(app, app._nav_rects["analytics"])
        assert app.screen_mode == "analytics"
        app.frame(0.016)
        app.save_screenshot(tmp_path / "shot.png")
        assert (tmp_path / "shot.png").exists()
    finally:
        pygame.quit()
