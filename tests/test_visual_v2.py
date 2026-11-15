"""GPU-free tests for the V2 3D layer: motion, intelligence, cinematography."""

from __future__ import annotations

import math

import pytest

from config.settings import SimulationConfig
from environment.scenario_generator import load_predefined
from interface3d.camera import TRANSITION_SECONDS, VERTICAL_FOV, CameraController
from interface3d.intelligence import ACTIVITIES, FleetTracker, classify
from interface3d.sync import (
    PresentationDirector,
    VisualSync,
    continues_straight,
    motion_profile,
    world_xy,
)
from simulation.simulation_controller import SimulationController
from simulation.simulation_engine import SimulationEngine


# ------------------------------------------------------------------ motion profile
@pytest.mark.parametrize("ease_in,ease_out", [(True, True), (True, False), (False, True), (False, False)])
def test_motion_profile_endpoints_and_monotonic(ease_in, ease_out):
    assert motion_profile(0.0, ease_in, ease_out) == 0.0
    assert motion_profile(1.0, ease_in, ease_out) == pytest.approx(1.0)
    vals = [motion_profile(i / 100, ease_in, ease_out) for i in range(101)]
    assert all(b >= a - 1e-12 for a, b in zip(vals, vals[1:]))


def test_motion_profile_velocities():
    h = 1e-4
    slope = lambda f, t: (f(t + h) - f(t)) / h  # noqa: E731
    acc = lambda t: motion_profile(t, True, False)  # noqa: E731
    dec = lambda t: motion_profile(t, False, True)  # noqa: E731
    both = lambda t: motion_profile(t, True, True)  # noqa: E731
    assert slope(acc, 0.0) == pytest.approx(0.0, abs=1e-3)          # starts from rest
    assert slope(acc, 1.0 - h) == pytest.approx(1.0, abs=1e-2)      # leaves at cruise speed
    assert slope(dec, 0.0) == pytest.approx(1.0, abs=1e-2)          # arrives at cruise speed
    assert slope(dec, 1.0 - h) == pytest.approx(0.0, abs=1e-2)      # stops
    assert slope(both, 0.0) == pytest.approx(0.0, abs=1e-3)


def test_continues_straight():
    assert continues_straight([(5, 5), (5, 6), (5, 7)], (5, 5), (5, 6))
    assert not continues_straight([(5, 5), (5, 6), (6, 6)], (5, 5), (5, 6))
    assert not continues_straight([(5, 5), (5, 6)], (5, 5), (5, 6))      # plan ends -> decelerate
    assert not continues_straight([], (5, 5), (5, 6))
    assert not continues_straight([(5, 5)], (5, 5), (5, 5))


def test_cruising_robots_do_not_stop_between_straight_cells():
    eng = SimulationEngine(load_predefined("large"), SimulationConfig(agent_type="goal_based", num_robots=3))
    sync = VisualSync()
    cruised = 0
    for _ in range(120):
        eng.step()
        sync.robot_poses(eng, 0.0, 0.016)
        for r in eng.warehouse.robots:
            anim = sync.robots[r.robot_id]
            if not anim.ease_in:
                cruised += 1
                # cruising is only allowed when the previous tick moved in the same direction
                assert r.previous_position != r.position
        for _ in range(3):
            sync.robot_poses(eng, 1.0, 0.016)
    assert cruised > 0


def test_differential_wheels_turn_in_place_and_drive():
    eng = SimulationEngine(load_predefined("medium"), SimulationConfig(agent_type="goal_based", num_robots=3))
    sync = VisualSync()
    saw_turn = saw_drive = False
    for _ in range(80):
        eng.step()
        prev = {rid: (a.wheel_l, a.wheel_r) for rid, a in sync.robots.items()}
        for k in range(1, 21):
            poses = sync.robot_poses(eng, k / 20, 0.016)
            for rid, p in poses.items():
                a = sync.robots[rid]
                if rid in prev:
                    dl, dr = a.wheel_l - prev[rid][0], a.wheel_r - prev[rid][1]
                    if p.turning and abs(dl) + abs(dr) > 1e-6 and k / 20 < 0.3:
                        assert dl * dr <= 1e-9          # opposite spin while rotating in place
                        saw_turn = True
                    elif p.moving and not p.turning and k / 20 > 0.5:
                        assert dl * dr >= 0             # same direction while driving straight
                        saw_drive = saw_drive or abs(dl) > 0
                prev[rid] = (a.wheel_l, a.wheel_r)
    assert saw_turn and saw_drive


# ------------------------------------------------------------------ fleet intelligence
def run_with_tracker(layout="medium", agent="utility_based", robots=3):
    ctl = SimulationController(SimulationConfig(layout=layout, agent_type=agent, num_robots=robots))
    tracker = FleetTracker()
    seen = []
    ctl.tick_listeners.append(tracker.on_tick)
    ctl.tick_listeners.append(lambda e, ev: seen.append(e.step_count))
    ctl.set_speed_index(8)
    ctl.start()
    while not ctl.engine.done:
        ctl.update(0.25)            # several ticks per call
    return ctl, tracker, seen


def test_tick_listeners_called_once_per_real_tick():
    ctl, _, seen = run_with_tracker("small", "goal_based", 2)
    assert seen == list(range(1, ctl.engine.step_count + 1))


def test_fleet_tracker_matches_engine():
    ctl, tr, _ = run_with_tracker()
    eng = ctl.engine
    for r in eng.warehouse.robots:
        u = tr.usage[r.robot_id]
        assert u.ticks == eng.step_count
        assert u.current == classify(r)
        assert 0.0 <= tr.utilization(r.robot_id) <= 1.0
        assert set(u.counts) == set(ACTIVITIES)
    assert sum(tr.activity_counts().values()) == len(eng.warehouse.robots)
    progress = tr.station_progress(eng)
    assert sum(d for d, _ in progress.values()) == eng.warehouse.delivered_count()
    assert sum(t for _, t in progress.values()) == len(eng.warehouse.parcels)
    assert len(tr.delivery_steps) == eng.warehouse.delivered_count()
    ct = tr.cycle_time(eng)
    assert ct is not None and ct > 0
    hot = tr.hotspot()
    assert hot is not None and eng.warehouse.grid.is_walkable(hot[0])


def test_heat_decays_and_conflicts_counted():
    ctl = SimulationController(SimulationConfig(layout="large", agent_type="simple_reflex", num_robots=3))
    tr = FleetTracker()
    ctl.tick_listeners.append(tr.on_tick)
    conflicts = []
    ctl.tick_listeners.append(lambda e, ev: conflicts.extend(x for x in ev if x.kind == "conflict"))
    checked = 0
    for _ in range(150):
        before = dict(tr.heat)
        ctl.step_once()
        eng = ctl.engine
        occupied = {r.position for r in eng.warehouse.robots}
        conflict_cells = {e.position for e in eng.events if e.kind == "conflict"}
        for cell, v in before.items():
            if cell in occupied or cell in conflict_cells:
                continue
            expected = v * 0.96
            if expected < 0.02:
                assert cell not in tr.heat
            else:
                assert tr.heat[cell] == pytest.approx(expected)
                checked += 1
        for cell in occupied:
            assert tr.heat[cell] >= 1.0
    assert checked > 0
    assert sum(tr.conflict_cells.values()) == len(conflicts)
    tr.on_tick(ctl.engine, [])           # duplicate sample of the same tick is ignored
    assert tr.last_step == ctl.engine.step_count


def test_tracker_resets_on_new_run():
    ctl, tr, _ = run_with_tracker("small", "goal_based", 2)
    ctl.reset()
    ctl.step_once()
    assert tr.last_step == 1 and all(u.ticks == 1 for u in tr.usage.values())


# ------------------------------------------------------------------ cinematography
def test_director_event_shots_frame_real_positions():
    eng = SimulationEngine(load_predefined("medium"), SimulationConfig(agent_type="goal_based", num_robots=3))
    director = PresentationDirector()
    while not eng.done:
        events = eng.step()
        shot = director.update(eng, events, 0.5)
        if shot.reason == "event" and director.elapsed == 0.0:
            ev = next(e for e in events if e.kind == shot.kind and e.robot_id == shot.robot_id)
            assert shot.points == [ev.position]
            if shot.kind == "deliver":
                assert eng.warehouse.stations[shot.station_id].position == ev.position
        if shot.kind == "search" and director.elapsed == 0.0:
            r = eng.warehouse.robots[shot.robot_id]
            target = eng.warehouse.parcel_target(r)
            assert shot.points[:len(r.planned_path)] == list(r.planned_path)
            if target:
                assert shot.points[-1] == target
    serials = [s.serial for s in director.log]
    assert serials == sorted(serials) and len(set(serials)) == len(serials)
    assert {"tracking", "follow", "search", "pickup", "deliver"} <= {s.kind for s in director.log}


def test_camera_transitions_are_timed_and_smooth():
    cam = CameraController((10, 6), 20)
    for _ in range(100):
        cam.update(0.05, 0, (3.0, 4.0, 90.0))
    cam.set_mode("topdown")
    assert cam.in_transition
    prev = cam.rig.copy()
    t = 0.0
    max_step = 0.0
    while cam.in_transition:
        cam.update(1 / 60, t, (3.0, 4.0, 90.0))
        max_step = max(max_step, abs(cam.rig.distance - prev.distance), abs(cam.rig.pitch - prev.pitch))
        prev = cam.rig.copy()
        t += 1 / 60
    assert t == pytest.approx(TRANSITION_SECONDS, abs=2 / 60)
    assert max_step < 2.5          # no jumps between frames
    assert cam.rig.pitch < -85


def test_frame_points_keeps_every_point_in_view():
    pts = [(0.0, 0.0), (12.0, 3.0), (6.0, 9.0)]
    rig = CameraController.frame_points(pts, pitch=-45, heading=-20)
    cx, cy, cz = rig.camera_position()
    look = (rig.tx - cx, rig.ty - cy, rig.tz - cz)
    ln = math.sqrt(sum(v * v for v in look))
    for (x, y) in pts:
        v = (x - cx, y - cy, 0.3 - cz)
        lv = math.sqrt(sum(a * a for a in v))
        ang = math.degrees(math.acos(sum(a * b for a, b in zip(v, look)) / (ln * lv)))
        assert ang < VERTICAL_FOV / 2          # inside the vertical half-FOV cone (the tighter one)


def test_shot_points_map_to_world():
    h = 13
    assert world_xy((12, 2), h) == (2.0, 0.0)
