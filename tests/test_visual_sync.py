"""GPU-free tests of the 3D visual layer: everything drawn must follow the real simulation."""

from __future__ import annotations

import math

import pytest

from config.settings import SimulationConfig
from environment.grid import Action
from environment.parcel import ParcelStatus
from environment.scenario_generator import load_predefined
from interface3d.camera import MODES, CameraController
from interface3d.sync import (
    ACTION_HEADING,
    EFFECT_DURATION,
    CameraRig,
    PresentationDirector,
    VisualSync,
    angle_diff,
    heading_between,
    world_xy,
)
from simulation.simulation_engine import SimulationEngine


def engine(layout="small", agent="goal_based", robots=2):
    return SimulationEngine(load_predefined(layout), SimulationConfig(agent_type=agent, num_robots=robots, seed=3))


def test_world_mapping_and_headings():
    assert world_xy((0, 0), 10) == (0.0, 9.0)        # row 0 is the far side
    assert world_xy((9, 4), 10) == (4.0, 0.0)
    assert heading_between((5, 5), (4, 5)) == ACTION_HEADING[Action.UP] == 0.0
    assert heading_between((5, 5), (6, 5)) == 180.0
    assert heading_between((5, 5), (5, 4)) == 90.0
    assert heading_between((5, 5), (5, 6)) == -90.0
    assert heading_between((5, 5), (5, 5)) is None
    # Panda3D heading h faces (-sin h, cos h): check each action moves the right way in world space
    for action, h in ACTION_HEADING.items():
        fx, fy = -math.sin(math.radians(h)), math.cos(math.radians(h))
        a = (5, 5)
        b = (5 + {Action.UP: -1, Action.DOWN: 1}.get(action, 0), 5 + {Action.LEFT: -1, Action.RIGHT: 1}.get(action, 0))
        (ax, ay), (bx, by) = world_xy(a, 10), world_xy(b, 10)
        assert (round(bx - ax), round(by - ay)) == (round(fx), round(fy))


def test_angle_diff_shortest_arc():
    assert angle_diff(10, 350) == pytest.approx(20)
    assert angle_diff(350, 10) == pytest.approx(-20)
    assert abs(angle_diff(180, 0)) == pytest.approx(180)


@pytest.mark.parametrize("agent", ["simple_reflex", "goal_based", "utility_based"])
def test_robot_poses_match_simulation_state(agent):
    eng = engine("medium", agent, 3)
    sync = VisualSync()
    h = eng.warehouse.grid.height
    for _ in range(80):
        eng.step()
        start = sync.robot_poses(eng, 0.0, 0.0)
        for r in eng.warehouse.robots:
            p = start[r.robot_id]
            # at progress 0 the robot is drawn on the cell it occupied before the tick
            assert (p.x, p.y) == pytest.approx(world_xy(r.previous_position, h))
        mid = sync.robot_poses(eng, 0.5, 0.016)
        end = sync.robot_poses(eng, 1.0, 0.016)
        for r in eng.warehouse.robots:
            p = end[r.robot_id]
            assert (p.x, p.y) == pytest.approx(world_xy(r.position, h))   # exact simulated cell
            assert p.cell == r.position and p.carrying == r.is_carrying and p.state == r.state
            q = mid[r.robot_id]
            ax, ay = world_xy(r.previous_position, h)
            bx, by = world_xy(r.position, h)
            # mid-animation positions stay on the segment between the two real cells
            assert min(ax, bx) - 1e-9 <= q.x <= max(ax, bx) + 1e-9
            assert min(ay, by) - 1e-9 <= q.y <= max(ay, by) + 1e-9
            travel = heading_between(r.previous_position, r.position)
            if travel is not None and not p.reversing:
                assert angle_diff(travel, p.heading) == pytest.approx(0, abs=1e-6)
        if eng.done:
            break


def test_reverse_driving_only_while_yielding():
    eng = engine("large", "utility_based", 3)
    sync = VisualSync()
    saw_reverse = False
    while not eng.done:
        eng.step()
        poses = sync.robot_poses(eng, 1.0, 0.016)
        for r in eng.warehouse.robots:
            if poses[r.robot_id].reversing:
                saw_reverse = True
                assert r.is_yielding
    # invariant checked above: reverse driving happens only during coordination retreats
    assert eng.metrics()["completed"]


def test_lift_follows_carrying_flag():
    eng = engine()
    sync = VisualSync()
    checked = 0
    while not eng.done:
        eng.step()
        for _ in range(40):   # let the lift animation settle (40 x 0.05 s)
            poses = sync.robot_poses(eng, 1.0, 0.05)
        for r in eng.warehouse.robots:
            assert poses[r.robot_id].lift == pytest.approx(1.0 if r.is_carrying else 0.0)
            checked += 1
    assert checked > 0


def test_effects_come_only_from_real_events():
    eng = engine("medium", "goal_based", 3)
    sync = VisualSync()
    pickups = delivers = 0
    t = 0.0
    while not eng.done:
        events = eng.step()
        t += 0.1
        created = sync.ingest_events(eng, events, t)
        assert len(created) == sum(1 for e in events if e.kind in EFFECT_DURATION)
        for fx in created:
            ev = next(e for e in events if e.message == fx.message)
            assert (fx.x, fx.y) == world_xy(ev.position, eng.warehouse.grid.height)
            robot = eng.warehouse.robots[fx.robot_id]
            if fx.kind == "pickup":
                pickups += 1
                assert robot.is_carrying and robot.position == ev.position
            if fx.kind == "deliver":
                delivers += 1
                assert not robot.is_carrying and ev.position in eng.warehouse.station_cells()
    assert pickups == delivers == eng.metrics()["parcels_delivered"] == len(eng.warehouse.parcels)
    assert sync.ingest_events(eng, [], t) == []
    assert sync.live_effects(t + 100) == []   # all effects expire


def test_parcel_views_and_dock_stacks():
    eng = engine("medium", "goal_based", 3)
    while not eng.done:
        eng.step()
        views = VisualSync.parcel_views(eng)
        for v in views:
            p = eng.warehouse.parcels[v.parcel_id]
            assert v.visible_on_floor == (p.status in (ParcelStatus.WAITING, ParcelStatus.ASSIGNED))
        stacks = VisualSync.delivered_per_station(eng)
        assert sum(len(s) for s in stacks.values()) == eng.warehouse.delivered_count()
        for sid, pids in stacks.items():
            assert all(eng.warehouse.parcels[pid].destination_id == sid for pid in pids)


def test_camera_rig_geometry_and_blending():
    top = CameraRig(5, 5, 0, 10, heading=0, pitch=-90)
    x, y, z = top.camera_position()
    assert (x, y, z) == pytest.approx((5, 5, 10), abs=1e-6)
    south = CameraRig(0, 0, 0, 10, heading=0, pitch=0).camera_position()
    assert south == pytest.approx((0, -10, 0), abs=1e-6)
    a = CameraRig(0, 0, 0, 10, heading=350, pitch=-40)
    b = CameraRig(10, 0, 0, 20, heading=10, pitch=-60)
    a.blend_toward(b, 0.5)
    assert a.heading == pytest.approx(360) and a.tx == pytest.approx(5)
    for _ in range(60):
        a.blend_toward(b, 0.3)
    assert a.distance == pytest.approx(20, abs=1e-3)


def test_camera_controller_modes_and_intro():
    cam = CameraController((10, 6), 20)
    done = []
    cam.start_intro(lambda: done.append(True))
    t = 0.0
    while cam.in_intro and t < 30:
        cam.update(0.1, t, None)
        t += 0.1
    assert done and not cam.in_intro and t < 13   # flythrough finishes on its own
    for mode in MODES:
        cam.set_mode(mode)
        for _ in range(200):
            rig = cam.update(0.05, t, (3.0, 4.0, 90.0))
        if mode in ("follow", "focus"):
            assert math.hypot(rig.tx - 3.0, rig.ty - 4.0) < 1.5   # tracks the robot
        if mode == "topdown":
            assert rig.pitch < -85
    with pytest.raises(ValueError):
        cam.set_mode("orbit-cam")


def test_presentation_director_uses_real_events():
    eng = engine("medium", "goal_based", 3)
    director = PresentationDirector()
    director.start()
    highlights = []
    while not eng.done:
        events = eng.step()
        shot = director.update(eng, events, 0.5)
        if shot.reason == "event" and (not highlights or highlights[-1] is not shot):
            highlights.append(shot)
            real = [e for e in events if e.kind in ("pickup", "deliver")]
            assert real, "highlight shots must come from a real pickup/delivery in this tick"
            assert shot.robot_id in {e.robot_id for e in real}
    assert highlights
    kinds = {s.mode for s in director.log}
    assert {"overview", "follow", "focus", "topdown"} <= kinds
    quiet = PresentationDirector()
    shot = quiet.update(eng, [], 0.1)
    assert shot.reason == "schedule"
