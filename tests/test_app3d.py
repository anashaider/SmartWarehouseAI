"""Offscreen integration tests of the Panda3D digital twin.

Skipped automatically when Panda3D is missing or no OpenGL context can be
created (e.g. a headless CI machine without a GPU). Panda3D allows a single
ShowBase per process, so one app instance is shared by the module.
"""

from __future__ import annotations

import math

import pytest

panda3d = pytest.importorskip("panda3d")

from config.settings import SimulationConfig  # noqa: E402
from environment.parcel import ParcelStatus  # noqa: E402
from interface3d.sync import world_xy  # noqa: E402
from simulation.state_manager import SimStatus  # noqa: E402


@pytest.fixture(scope="module")
def app():
    from interface3d.app3d import WarehouseApp3D
    try:
        a = WarehouseApp3D(SimulationConfig(layout="small", num_robots=2), offscreen=True,
                           size=(960, 540), quality="low", intro=False)
    except Exception as exc:  # noqa: BLE001
        pytest.skip(f"3D rendering unavailable: {exc}")
    yield a
    a.destroy()


def settle(app, frames=3):
    app.ctl.last_tick_time = -1e9          # animation of the last tick finished
    for _ in range(frames):
        app.frame(1 / 30)


def test_robot_nodes_follow_simulation(app):
    app.cmd_config(layout="small", num_robots=2, agent_type="goal_based")
    eng = app.ctl.engine
    h = eng.warehouse.grid.height
    for _ in range(25):
        app.cmd_step()
        settle(app)
        for r in eng.warehouse.robots:
            pos = app.robots[r.robot_id].root.getPos(app.base.render)
            assert (pos.x, pos.y) == pytest.approx(world_xy(r.position, h), abs=1e-4)
            assert app.robots[r.robot_id].cargo.isHidden() == (not r.is_carrying)


def test_pickup_and_delivery_visuals_match_real_actions(app):
    app.cmd_config(layout="small", num_robots=2, agent_type="goal_based")
    eng = app.ctl.engine
    while not eng.done:
        app.cmd_step()
        settle(app, 1)
        for p in eng.warehouse.parcels.values():
            floor_node, _ring = app.parcel_nodes[p.parcel_id]
            on_floor = p.status in (ParcelStatus.WAITING, ParcelStatus.ASSIGNED)
            assert floor_node.isHidden() == (not on_floor)
            assert (p.parcel_id in app.stack_nodes) == (p.status == ParcelStatus.DELIVERED)
    assert len(app.stack_nodes) == eng.metrics()["parcels_delivered"] == len(eng.warehouse.parcels)
    kinds = [e.kind for e in app.sync.history]
    assert kinds.count("pickup") == kinds.count("deliver") == len(eng.warehouse.parcels)


def test_bfs_astar_overlay_uses_real_search(app):
    app.cmd_config(layout="medium", num_robots=3)
    app.cmd_step()
    app.cmd_step()
    app.show_compare = True
    app._cmp_key = None
    settle(app, 2)
    cmp_ = app.comparison
    assert cmp_ is not None
    assert cmp_["bfs"].cost == cmp_["astar"].cost
    assert len(app.effects.explored_tiles) == len(cmp_["bfs"].explored) + len(cmp_["astar"].explored)
    for _ in range(80):  # reveal animation completes
        app.frame(1 / 30)
    assert all(not node.isHidden() for node, _, _ in app.effects.explored_tiles)
    app.show_compare = False
    settle(app, 1)
    assert app.effects.explored_root.isHidden()


def test_pause_resume_reset_and_buttons(app):
    app.cmd_config(layout="small", num_robots=2)
    hud = app.hud
    settle(app, 1)
    hud.buttons["start"]["command"]()
    assert app.ctl.status == SimStatus.RUNNING
    for _ in range(30):
        app.frame(0.1)
    assert app.ctl.engine.step_count > 0
    hud.buttons["pause"]["command"]()
    n = app.ctl.engine.step_count
    for _ in range(20):
        app.frame(0.1)
    assert app.ctl.status == SimStatus.PAUSED and app.ctl.engine.step_count == n
    hud.buttons["step"]["command"]()
    assert app.ctl.engine.step_count == n + 1
    hud.buttons["resume"]["command"]()
    assert app.ctl.status == SimStatus.RUNNING
    hud.buttons["reset"]["command"]()
    assert app.ctl.engine.step_count == 0 and app.ctl.status == SimStatus.READY
    before = app.ctl.config.agent_type
    hud.buttons["agent-next"]["command"]()
    assert app.ctl.config.agent_type != before
    hud.buttons["robots-next"]["command"]()
    settle(app, 1)
    assert len(app.robots) == len(app.ctl.engine.warehouse.robots)


def test_presentation_mode_does_not_interrupt_simulation(app):
    app.cmd_config(layout="medium", num_robots=3)
    for _ in range(10):
        app.cmd_step()
    steps = app.ctl.engine.step_count
    app.cmd_presentation()
    assert app.presenting and not app.hud.visible
    for _ in range(10):
        app.frame(1 / 30)
    assert app.ctl.engine.step_count == steps      # still paused: nothing changed
    assert app.shot is not None
    app.cmd_resume()
    for _ in range(40):
        app.frame(0.1)
    assert app.ctl.engine.step_count > steps       # runs underneath the presentation
    app.cmd_presentation()
    assert not app.presenting and app.hud.visible
    assert app.ctl.engine.step_count > steps       # exiting does not reset anything


def test_camera_modes_and_occlusion(app):
    app.cmd_config(layout="large", num_robots=3)
    for _ in range(15):
        app.cmd_step()
    for mode in ("overview", "topdown", "follow", "free", "focus"):
        app.cmd_camera(mode)
        for _ in range(90):
            app.frame(1 / 30)
        cam = app.base.camera.getPos(app.base.render)
        assert all(math.isfinite(v) for v in cam)
    app.cmd_camera("follow")
    for _ in range(150):
        app.frame(1 / 30)
    rig = app.camera.rig.copy()
    view = app._unoccluded(rig, 10.0, allow_turn=True)
    assert not app._blocked(view)


def test_quality_levels_and_analytics(app, tmp_path):
    for q in ("medium", "high", "low"):
        app.cmd_quality(q)
        app.frame(1 / 30)
        assert app.quality == q
    app.cmd_analytics()
    assert app.analytics.visible
    app.frame(1 / 30)
    app.analytics.select(2)
    app.frame(1 / 30)
    app.cmd_analytics()
    assert not app.analytics.visible
    out = app.screenshot(tmp_path / "twin.png")
    assert out.exists() and out.stat().st_size > 20_000


def test_picking_selects_robot_under_cursor(app):
    app.cmd_config(layout="small", num_robots=2)
    app.cmd_camera("topdown")
    for _ in range(120):
        app.frame(1 / 30)
    for rid, model in app.robots.items():
        pos = model.root.getPos(app.base.render)
        sp = app.screen_point(pos.x, pos.y, 0.25)
        assert sp is not None
        assert app.pick_robot(sp) == rid


def test_intro_sequence_hands_over_to_hud(app):
    app.start_intro()
    assert app.camera.in_intro and not app.hud.visible
    app.frame(1 / 30)
    app.cmd_escape()                       # skipping restores the command center
    assert not app.camera.in_intro and app.hud.visible
    app.start_intro()
    for _ in range(30):                    # or it finishes on its own (~11 s of camera time)
        app.frame(0.5)
    assert not app.camera.in_intro and app.hud.visible


# ====================================================================== V2 features
def test_v2_chunked_scene_lod_and_frustum_culling(app):
    app.cmd_quality("medium")
    app.cmd_config(layout="large", num_robots=3)
    vis = app.vis
    assert len(vis.chunks) >= 4 and vis.lod_nodes            # spatial chunks + detail LOD
    from panda3d.core import LODNode
    for lod_np in vis.lod_nodes:
        node = lod_np.node()
        assert isinstance(node, LODNode) and node.getNumSwitches() == 1
    app.cmd_camera("overview")
    for _ in range(120):
        app.frame(1 / 30)
    vis_over, total = app.visible_chunks()
    for _ in range(20):
        app.cmd_step()
    app.cmd_camera("follow")
    for _ in range(150):
        app.frame(1 / 30)
    vis_follow, _ = app.visible_chunks()
    assert vis_follow < total and vis_follow <= vis_over      # close-up views cull chunks


def test_v2_shadow_map_refreshes_only_when_casters_move(app):
    app.cmd_quality("medium")
    app.cmd_config(layout="small", num_robots=2)
    for _ in range(10):
        app.frame(1 / 30)
    idle_start = app.shadow_updates
    for _ in range(30):                                       # paused, nothing moves
        app.frame(1 / 30)
    assert app.shadow_updates == idle_start
    app.cmd_step()                                            # robots move -> refresh
    for _ in range(30):
        app.frame(1 / 30)
    assert app.shadow_updates > idle_start
    # floor, decals and overlays are not rendered by the shadow camera
    from interface3d.app3d import SHADOW_MASK
    assert all(np_.isHidden(SHADOW_MASK) for np_ in app.vis.no_shadow)
    assert app.effects.root.isHidden(SHADOW_MASK)


def test_v2_heatmap_indicators_and_docks_follow_engine(app):
    app.cmd_config(layout="medium", num_robots=3, agent_type="goal_based")
    app.show_heatmap = True
    app.show_indicators = True
    app.cmd_camera("overview")
    eng = app.ctl.engine
    while not eng.done:
        app.cmd_step()
        app.frame(1 / 30)
        assert app.heatmap.version == app.tracker.version == eng.step_count
        for rid, (root, tn, _bar) in app.indicators.items():
            act = app.tracker.usage[rid].current
            assert tn.getText() == f"R{rid}  {act.upper()}"
        for sid, st in app.vis.stations.items():
            done, total = app.tracker.station_progress(eng)[sid]
            assert st.sign_text.getText() == f"D{sid}   {done} / {total}"
    # final tick: the robot that made the last delivery is HANDLING, the rest are IDLE
    endings = sorted(tn.getText().split()[-1] for _, tn, _ in app.indicators.values())
    assert endings.count("HANDLING") >= 1 and set(endings) <= {"HANDLING", "IDLE"}
    app.show_heatmap = False


def test_v2_fleet_tab_shows_tracker_values(app):
    app.cmd_config(layout="small", num_robots=2)
    for _ in range(15):
        app.cmd_step()
    app.hud.set_right_tab("fleet")
    app.frame(1 / 30)
    tr = app.tracker
    for act, n in tr.activity_counts().items():
        assert app.hud.texts[f"f:act:{act}"].getText() == str(n)
    for i in range(2):
        assert app.hud.texts[f"f:rpct{i}"].getText() == f"{100 * tr.utilization(i):.0f}%"
    assert app.hud.root.find("**/tab-fleet").isHidden() is False
    assert app.hud.root.find("**/tab-robot").isHidden() is True
    app.hud.set_right_tab("robot")
    app.frame(1 / 30)


def test_v2_resources_released_on_rebuild(app):
    app.cmd_config(layout="medium", num_robots=3)
    app.frame(1 / 30)
    baseline = app.base.render.findAllMatches("**").getNumPaths()
    for layout in ("small", "large", "medium", "small", "medium"):
        app.cmd_config(layout=layout)
        app.frame(1 / 30)
    assert app.base.render.findAllMatches("**").getNumPaths() == baseline
    assert app.base.pixel2d.findAllMatches("**/loading").getNumPaths() == 0   # loading card removed


def test_v2_hud_is_batched(app):
    app.frame(1 / 30)
    hud_root = app.hud.root
    # static labels are merged geometry; only live values remain text nodes
    live_texts = hud_root.findAllMatches("**/+TextNode").getNumPaths()
    assert live_texts <= len(app.hud.texts) + len(app.hud.buttons) + 5
    static_text = hud_root.find("static-text")
    assert not static_text.isEmpty() and static_text.findAllMatches("**/+GeomNode").getNumPaths() <= 4
    app.hud.show_tooltip("Third-Person  (F)", 100, 100)
    assert not app.hud.tooltip_root.isHidden() and app.hud.tooltip.getText() == "Third-Person  (F)"
    app.hud.hide_tooltip()
    app.notify("test message")
    app.frame(1 / 30)
    assert app.hud.texts["banner"].getText() == "test message"


def test_v2_floor_overlays_visible_with_post_processing(app, tmp_path):
    """Regression: with filters on (lower depth precision) floor overlays used to
    lose the depth test against the floor. The heatmap must change the image."""
    import matplotlib.image as mpimg
    app.cmd_quality("medium")
    app.cmd_config(layout="large", num_robots=3, agent_type="simple_reflex")
    for _ in range(40):
        app.cmd_step()
    app.cmd_camera("topdown")
    app.show_heatmap = False
    for _ in range(90):
        app.frame(1 / 30)
    off = mpimg.imread(str(app.screenshot(tmp_path / "off.png")))[..., :3]
    app.show_heatmap = True
    for _ in range(3):
        app.frame(1 / 30)
    on = mpimg.imread(str(app.screenshot(tmp_path / "on.png")))[..., :3]
    changed = (abs(on - off).sum(axis=2) > 0.05).sum()
    assert changed > 1000
    app.show_heatmap = False
    app.cmd_quality("low")
