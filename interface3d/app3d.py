"""3D digital-twin application (Panda3D).

Frame loop (``_update``):
    1. SimulationController.update(dt)  - fixed tick rate, independent of FPS
    2. VisualSync                        - poses / effects from the real engine state
    3. scene update                      - robots, parcels, dock stacks, overlays
    4. camera / presentation director    - camera only, never the simulation
    5. HUD / analytics                   - text and buttons

The simulation engine, agents, search and coordination are the unchanged
modules used by the 2D dashboard and the experiment runner.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

from panda3d.core import (
    AmbientLight,
    AntialiasAttrib,
    DirectionalLight,
    Filename,
    Point2,
    Point3,
    TextureStage,
    WindowProperties,
    loadPrcFileData,
)

from algorithms.pathfinder import compare
from config.settings import AGENT_LABELS, ALGORITHM_LABELS, LAYOUT_LABELS, SimulationConfig
from environment.parcel import ParcelStatus
from interface3d import hud as H
from interface3d.camera import MODE_LABELS, CameraController
from interface3d.effects import EffectsLayer
from interface3d.robot_model import STATE_LIGHT, WAIT_LIGHT, YIELD_LIGHT, RobotModel, build_robot
from interface3d.sync import (PresentationDirector, RobotPose, Shot, VisualSync, angle_diff, damping,
                              world_xy)
from interface3d.warehouse_scene import RACK_HEIGHT, WarehouseVisual, build_warehouse, make_carton, make_marker_ring
from simulation.simulation_controller import SimulationController
from simulation.state_manager import SimStatus

QUALITY_PRESETS = {
    "low": {"shadows": 0, "msaa": False, "bloom": False, "ao": False},
    "medium": {"shadows": 4096, "msaa": True, "bloom": True, "ao": False},
    "high": {"shadows": 4096, "msaa": True, "bloom": True, "ao": True},
}


def configure_panda(offscreen: bool, size: tuple[int, int], title: str = "Smart Warehouse AI - 3D Digital Twin") -> None:
    loadPrcFileData("", "\n".join([
        f"window-title {title}",
        f"win-size {size[0]} {size[1]}",
        "window-type offscreen" if offscreen else "window-type onscreen",
        "textures-power-2 none",
        "sync-video true",
        "audio-library-name null",
        "framebuffer-srgb false",
        "show-frame-rate-meter false",
        "notify-level-display error",
        "notify-level-glgsg error",
        "default-directnotify-level error",
        "basic-shaders-only false",
    ]))


class WarehouseApp3D:
    """Owns a ShowBase instance (Panda3D allows one per process)."""

    def __init__(self, config: Optional[SimulationConfig] = None, offscreen: bool = False,
                 size: tuple[int, int] = (1600, 900), quality: str = "medium", intro: bool = True) -> None:
        configure_panda(offscreen, size)
        from direct.showbase.ShowBase import ShowBase  # imported after PRC configuration

        self.base = ShowBase()
        b = self.base
        b.disableMouse()
        b.setBackgroundColor(0.02, 0.03, 0.05, 1)
        b.render.setShaderAuto()
        b.render.setAntialias(AntialiasAttrib.MAuto)
        b.camLens.setFov(52)
        b.camLens.setNearFar(0.1, 600)
        self.offscreen = offscreen
        self.quality = quality if quality in QUALITY_PRESETS else "medium"
        self.ctl = SimulationController(config)
        self.sync = VisualSync()
        self.director = PresentationDirector()
        self.selected = 0
        self.show_paths = True
        self.show_compare = False
        self.presenting = False
        self.shot: Optional[Shot] = None
        self.filters = None
        self.world = None
        self.vis: Optional[WarehouseVisual] = None
        self.robots: dict[int, RobotModel] = {}
        self.parcel_nodes: dict[int, tuple] = {}
        self.stack_nodes: dict[int, object] = {}
        self.stack_seen: set[int] = set()
        self.transfers: list[tuple] = []
        self.effects: Optional[EffectsLayer] = None
        self.comparison = None
        self._cmp_key = None
        self._scene_key = None
        self._mouse_last: Optional[tuple[float, float]] = None
        self._drag: Optional[str] = None
        self.time = 0.0
        self.frame_count = 0
        self.help_root = None
        self._occl_pitch = 0.0
        self._occl_heading = 0.0

        self._setup_lights()
        self.hud = H.Hud(b, self)
        from interface3d.analytics3d import AnalyticsOverlay
        self.analytics = AnalyticsOverlay(b, self.hud)
        self.presentation_root = None
        self.rebuild_scene()
        self._layout()
        self.apply_quality(self.quality)
        self._bind_input()
        if intro:
            self.start_intro()
        b.taskMgr.add(self._task, "digital-twin-update")

    # ================================================================== scene
    def _setup_lights(self) -> None:
        r = self.base.render
        amb = AmbientLight("ambient")
        amb.setColor((0.26, 0.28, 0.34, 1))
        r.setLight(r.attachNewNode(amb))
        sun = DirectionalLight("sun")
        sun.setColor((0.78, 0.75, 0.70, 1))
        self.sun = r.attachNewNode(sun)
        self.sun.setHpr(-32, -58, 0)
        r.setLight(self.sun)
        fill = DirectionalLight("fill")
        fill.setColor((0.20, 0.24, 0.32, 1))
        fnp = r.attachNewNode(fill)
        fnp.setHpr(150, -35, 0)
        r.setLight(fnp)

    def rebuild_scene(self) -> None:
        """(Re)build the 3D world for the current scenario / robot count."""
        if self.world is not None:
            self.world.removeNode()
        b = self.base
        engine = self.ctl.engine
        self.world = b.render.attachNewNode("world")
        self.vis = build_warehouse(self.ctl.scenario, self.world, self.quality)
        h = engine.warehouse.grid.height
        self.robots = {r.robot_id: build_robot(r.robot_id, self.world) for r in engine.warehouse.robots}
        self.parcel_nodes = {}
        for p in engine.warehouse.parcels.values():
            node = make_carton(f"parcel-{p.parcel_id}")
            node.reparentTo(self.world)
            x, y = world_xy(p.origin, h)
            node.setPos(x, y, 0.0)
            ring = make_marker_ring((1, 1, 1, 0.9), 0.3, 0.36)
            ring.reparentTo(self.world)
            ring.setPos(x, y, 0.011)
            ring.hide()
            self.parcel_nodes[p.parcel_id] = (node, ring)
        self.stack_nodes = {}
        self.stack_seen = set()
        self.transfers = []
        self.effects = EffectsLayer(self.world, h)
        self._cmp_key = None
        self.sync.reset()
        cx, cy = self.vis.center
        if not hasattr(self, "camera"):
            self.camera = CameraController((cx, cy), self.vis.extent)
        else:
            mode = self.camera.mode
            was_intro = self.camera.in_intro
            self.camera = CameraController((cx, cy), self.vis.extent)
            self.camera.mode = mode if mode != "free" else "overview"
            if was_intro:
                self._end_intro()
        # shadow camera covers the warehouse
        lens = self.sun.node().getLens()
        gw, gh = self.vis.width + 2 * 2.5 + 1, self.vis.height + 2 * 2.5 + 1
        lens.setFilmSize(gw * 1.05, gh * 1.25)
        lens.setNearFar(1, 120)
        self.sun.setPos(cx, cy, 0)
        self.sun.setPos(self.sun, 0, -50, 0)
        self._scene_key = (id(self.ctl.scenario), len(engine.warehouse.robots))

    # ================================================================== quality
    def apply_quality(self, quality: str) -> None:
        self.quality = quality if quality in QUALITY_PRESETS else "medium"
        preset = QUALITY_PRESETS[self.quality]
        sun = self.sun.node()
        if preset["shadows"]:
            sun.setShadowCaster(True, preset["shadows"], preset["shadows"])
        else:
            sun.setShadowCaster(False)
        if self.filters is not None:
            self.filters.cleanup()
            self.filters = None
        if preset["bloom"] or preset["ao"] or preset["msaa"]:
            try:
                from direct.filter.CommonFilters import CommonFilters
                f = CommonFilters(self.base.win, self.base.cam)
                ok = True
                if preset["msaa"]:
                    ok = f.setMSAA(4) is not False and ok
                if preset["bloom"]:
                    ok = f.setBloom(blend=(0.3, 0.4, 0.3, 0.0), mintrigger=0.72, maxtrigger=1.0, desat=0.5,
                                    intensity=0.55, size="medium") is not False and ok
                if preset["ao"]:
                    ok = f.setAmbientOcclusion(numsamples=16, radius=0.06, amount=1.6, strength=0.012,
                                               falloff=0.000002) is not False and ok
                self.filters = f
                if not ok:
                    self.ctl.message = "Some post-processing effects are not supported on this GPU."
            except Exception as exc:  # noqa: BLE001 - degrade gracefully on unsupported GPUs
                self.filters = None
                self.ctl.message = f"Post-processing disabled: {exc}"
        self._layout()

    # ================================================================== layout
    def _window_size(self) -> tuple[int, int]:
        win = self.base.win
        return (max(1, win.getXSize()), max(1, win.getYSize()))

    def _layout(self) -> None:
        w, h = self._window_size()
        self.hud.build(w, h)
        self.analytics.build(w, h)
        self._build_presentation(w, h)
        self._apply_film_offset()

    def _apply_film_offset(self) -> None:
        """Centre the warehouse in the visible viewport between the HUD panels."""
        w, h = self._window_size()
        lens = self.base.camLens
        lens.setAspectRatio(w / h)
        if self.hud.visible and not self.presenting:
            left = H.LEFT_W + 2 * H.GAP
            right = H.RIGHT_W + 2 * H.GAP
            top = H.TOP_H + 40
            bottom = H.BOTTOM_H + 2 * H.GAP
            dx = (left + (w - right)) / 2 - w / 2
            dy = (top + (h - bottom)) / 2 - h / 2
        else:
            dx = dy = 0.0
        fw, fh = lens.getFilmSize()
        lens.setFilmOffset(-dx / w * fw, dy / h * fh)

    def _on_window_event(self, window) -> None:
        # ShowBase's own handler keeps pixel2d/aspect2d in sync with the window; our
        # accept() replaced it, so it must be called first.
        self.base.windowEvent(window)
        if window is None or window != self.base.win:
            return
        size = self._window_size()
        if size != self.hud.size and size[0] > 200 and size[1] > 200:
            self._layout()

    # ================================================================== presentation overlay
    def _build_presentation(self, w: int, h: int) -> None:
        if self.presentation_root is not None:
            self.presentation_root.removeNode()
        hud = self.hud
        root = self.base.pixel2d.attachNewNode("presentation")
        saved = hud.root
        hud.root = root
        bar = int(h * 0.085)
        hud._frame(0, 0, w, bar, (0, 0, 0, 0.92))
        hud._frame(0, h - bar, w, bar, (0, 0, 0, 0.92))
        hud.text("p:mode", "SMART WAREHOUSE AI  ·  PRESENTATION MODE", 30, bar / 2 - 8, 13, H.ACCENT_HOVER, bold=True)
        hud.text("p:live", "", w - 30, bar / 2 - 8, 13, H.TEXT2, align="right", bold=True)
        hud._frame(30, h - bar - 96, 6, 70, H.ACCENT)
        hud.text("p:title", "", 48, h - bar - 98, 28, H.TEXT, bold=True)
        hud.text("p:sub", "", 50, h - bar - 54, 15, H.TEXT2, wrap=w * 0.6)
        hud.text("p:stats", "", w / 2, h - bar / 2 - 8, 14, H.TEXT, align="center", bold=True)
        hud.text("p:hint", "P / Esc  exit presentation", w - 30, h - bar / 2 - 6, 11, H.MUTED, align="right")
        hud.root = saved
        self.presentation_root = root
        if not self.presenting:
            root.hide()

    # ================================================================== opening sequence
    def start_intro(self) -> None:
        """Letterboxed opening flythrough; the command-center HUD fades in afterwards."""
        self.hud.set_visible(False)
        self.presentation_root.show()
        self.camera.start_intro(self._end_intro)
        self._apply_film_offset()

    def _end_intro(self) -> None:
        if not self.presenting:
            self.presentation_root.hide()
            self.hud.set_visible(True)
        self._apply_film_offset()

    # ================================================================== commands (HUD / keys)
    def cmd_start(self) -> None:
        self.camera.skip_intro()
        self.ctl.start()

    def cmd_pause(self) -> None:
        self.ctl.pause()

    def cmd_resume(self) -> None:
        self.ctl.resume()

    def cmd_toggle_run(self) -> None:
        self.camera.skip_intro()
        self.ctl.toggle()

    def cmd_step(self) -> None:
        self.camera.skip_intro()
        self.ctl.step_once()

    def cmd_reset(self) -> None:
        self.ctl.reset()
        self.rebuild_scene()

    def cmd_config(self, **changes) -> None:
        self.ctl.apply_config(**changes)
        self.selected = min(self.selected, self.ctl.config.num_robots - 1)
        self.rebuild_scene()

    def cmd_speed(self, index: int) -> None:
        self.ctl.set_speed_index(index)

    def cmd_quality(self, quality: str) -> None:
        self.apply_quality(quality)
        self.rebuild_scene()

    def cmd_select(self, robot_id: int) -> None:
        if robot_id < len(self.ctl.engine.warehouse.robots):
            self.selected = robot_id

    def cmd_camera(self, mode: str) -> None:
        self.camera.skip_intro()
        self.camera.set_mode(mode)

    def cmd_toggle_paths(self) -> None:
        self.show_paths = not self.show_paths

    def cmd_toggle_compare(self) -> None:
        self.show_compare = not self.show_compare
        self._cmp_key = None

    def cmd_toggle_hud(self) -> None:
        if self.presenting:
            return
        self.hud.set_visible(not self.hud.visible)
        self._apply_film_offset()

    def cmd_analytics(self) -> None:
        if self.presenting:
            return
        self.analytics.toggle()

    def cmd_presentation(self) -> None:
        """Toggle Presentation Mode. The simulation keeps its state; a READY
        simulation is started so the presentation has live activity."""
        self.presenting = not self.presenting
        self.camera.skip_intro()
        if self.presenting:
            self.analytics.hide()
            self.hud.set_visible(False)
            self.presentation_root.show()
            self.director.start()
            if self.ctl.status == SimStatus.READY:
                self.ctl.start()
        else:
            self.presentation_root.hide()
            self.hud.set_visible(True)
            self.shot = None
        self._apply_film_offset()

    def cmd_escape(self) -> None:
        if self.help_root is not None:
            self._toggle_help()
        elif self.camera.in_intro:
            self.camera.skip_intro()
        elif self.presenting:
            self.cmd_presentation()
        elif self.analytics.visible:
            self.analytics.hide()

    def _toggle_help(self) -> None:
        if self.help_root is not None:
            self.help_root.removeNode()
            self.help_root = None
            return
        w, h = self._window_size()
        hud = self.hud
        root = self.base.pixel2d.attachNewNode("help")
        saved = hud.root
        hud.root = root
        bw, bh = 620, 470
        x, y = (w - bw) / 2, (h - bh) / 2
        hud._frame(0, 0, w, h, (0, 0, 0, 0.55))
        hud._frame(x, y, bw, bh, H.PANEL_SOLID)
        hud.text(None, "Controls", x + 24, y + 20, 20, H.TEXT, bold=True)
        lines = [("Space", "start / pause / resume"), ("S", "step one tick"), ("R", "reset scenario"),
                 ("1 2 3 / Tab", "select robot (or click it)"), ("O T F V G", "overview / top-down / follow / free / focus"),
                 ("Right drag", "orbit camera"), ("Middle drag", "pan camera"), ("Wheel", "zoom"),
                 ("C", "BFS vs A* explored cells"), ("P", "presentation mode"), ("A", "analytics"),
                 ("H", "hide HUD"), ("Q", "graphics quality"), ("+ / -", "simulation speed"), ("Esc", "close / exit")]
        yy = y + 64
        for k, v in lines:
            hud.text(None, k, x + 28, yy, 13, H.ACCENT_HOVER, bold=True)
            hud.text(None, v, x + 190, yy, 13, H.TEXT2)
            yy += 24
        hud.root = saved
        self.help_root = root

    def _bind_input(self) -> None:
        b = self.base
        acc = b.accept
        acc("space", self.cmd_toggle_run)
        acc("s", self.cmd_step)
        acc("r", self.cmd_reset)
        acc("tab", lambda: self.cmd_select((self.selected + 1) % len(self.ctl.engine.warehouse.robots)))
        for i in range(3):
            acc(str(i + 1), lambda i=i: self.cmd_select(i))
        for key, mode in (("o", "overview"), ("t", "topdown"), ("f", "follow"), ("v", "free"), ("g", "focus")):
            acc(key, lambda m=mode: self.cmd_camera(m))
        acc("c", self.cmd_toggle_compare)
        acc("p", self.cmd_presentation)
        acc("a", self.cmd_analytics)
        acc("h", self.cmd_toggle_hud)
        acc("q", lambda: self.cmd_quality({"low": "medium", "medium": "high", "high": "low"}[self.quality]))
        acc("f1", self._toggle_help)
        acc("escape", self.cmd_escape)
        acc("+", lambda: self.cmd_speed(self.ctl.speed_index + 1))
        acc("=", lambda: self.cmd_speed(self.ctl.speed_index + 1))
        acc("-", lambda: self.cmd_speed(self.ctl.speed_index - 1))
        acc("mouse1", self._on_click)
        acc("mouse3", lambda: self._start_drag("orbit"))
        acc("mouse3-up", self._end_drag)
        acc("mouse2", lambda: self._start_drag("pan"))
        acc("mouse2-up", self._end_drag)
        acc("wheel_up", lambda: self.camera.zoom(0.88))
        acc("wheel_down", lambda: self.camera.zoom(1.14))
        acc("window-event", self._on_window_event)

    def _mouse(self) -> Optional[tuple[float, float]]:
        mw = self.base.mouseWatcherNode
        if mw is None or not mw.hasMouse():
            return None
        m = mw.getMouse()
        return (m.getX(), m.getY())

    def _start_drag(self, kind: str) -> None:
        self.camera.skip_intro()
        self._drag = kind
        self._mouse_last = self._mouse()

    def _end_drag(self) -> None:
        self._drag = None

    def _on_click(self) -> None:
        if self.camera.in_intro:
            self.camera.skip_intro()
            return
        m = self._mouse()
        if m is None or self.analytics.visible:
            return
        rid = self.pick_robot(m)
        if rid is not None:
            self.selected = rid

    def screen_point(self, x: float, y: float, z: float) -> Optional[tuple[float, float]]:
        p3 = self.base.cam.getRelativePoint(self.base.render, Point3(x, y, z))
        p2 = Point2()
        if self.base.camLens.project(p3, p2):
            return (p2.getX(), p2.getY())
        return None

    def pick_robot(self, mouse: tuple[float, float]) -> Optional[int]:
        w, h = self._window_size()
        best, best_d = None, 45.0
        for rid, model in self.robots.items():
            pos = model.root.getPos(self.base.render)
            sp = self.screen_point(pos.x, pos.y, 0.25)
            if sp is None:
                continue
            d = math.hypot((sp[0] - mouse[0]) * w / 2, (sp[1] - mouse[1]) * h / 2)
            if d < best_d:
                best, best_d = rid, d
        return best

    # ================================================================== frame
    def _task(self, task):
        dt = min(self.base.taskMgr.globalClock.getDt(), 0.1)
        self._update(dt)
        return task.cont

    def _update(self, dt: float) -> None:
        self.time += dt
        self.frame_count += 1
        now = self.time
        ctl = self.ctl
        events = ctl.update(dt)
        engine = ctl.engine
        if (id(ctl.scenario), len(engine.warehouse.robots)) != self._scene_key:
            self.rebuild_scene()
        if events:
            self.sync.ingest_events(engine, events, now)
        poses = self.sync.robot_poses(engine, ctl.animation_progress(), dt)
        self.selected = min(self.selected, len(engine.warehouse.robots) - 1)

        if self.presenting:
            self.shot = self.director.update(engine, events, dt)
            if self.shot.robot_id is not None:
                self.selected = self.shot.robot_id

        self._update_robots(poses, now)
        self._update_parcels(engine, poses, now)
        self._update_overlays(engine, poses, now)
        self._update_camera(dt, poses)
        self._update_hud(engine)
        self.analytics.update()

    # ------------------------------------------------------------------ robots
    def _update_robots(self, poses: dict[int, RobotPose], now: float) -> None:
        for rid, pose in poses.items():
            m = self.robots.get(rid)
            if m is None:
                continue
            m.root.setPos(pose.x, pose.y, 0.0)
            m.root.setH(pose.heading)
            for wnode in m.wheels:
                wnode.setP(pose.wheel_angle)
            m.lidar.setH((now * (540 if pose.moving else 220)) % 360)
            m.lift.setZ(0.245 + 0.2 * pose.lift)
            if pose.carrying:
                m.cargo.show()
            else:
                m.cargo.hide()
            # gentle suspension bob while driving
            m.body.setZ(0.006 * math.sin(now * 18) if pose.moving else 0.0)
            if pose.yielding:
                col, pulse = YIELD_LIGHT, 0.5 + 0.5 * math.sin(now * 8)
            elif pose.waiting:
                col, pulse = WAIT_LIGHT, 1.0 if int(now * 3) % 2 == 0 else 0.25
            else:
                col, pulse = STATE_LIGHT[pose.state.value], 0.85 + 0.15 * math.sin(now * 2.5)
            m.status.setColorScale(col[0] * (0.4 + pulse), col[1] * (0.4 + pulse), col[2] * (0.4 + pulse), 1)
            m.halo.show()
            if pose.waiting or pose.yielding:
                m.halo.setColorScale(col[0], col[1], col[2], 0.35 + 0.45 * pulse)
                s = 1.0 + 0.08 * math.sin(now * 5)
            else:
                a = m.accent
                m.halo.setColorScale(a[0], a[1], a[2], 0.38)
                s = 1.0
            m.halo.setScale(s, s, 1)
            if rid == self.selected:
                m.select_ring.show()
                a = m.accent
                m.select_ring.setColorScale(a[0], a[1], a[2], 0.6 + 0.3 * math.sin(now * 4))
            else:
                m.select_ring.hide()

    # ------------------------------------------------------------------ parcels and dock stacks
    def _update_parcels(self, engine, poses: dict[int, RobotPose], now: float) -> None:
        for view in self.sync.parcel_views(engine):
            node, ring = self.parcel_nodes[view.parcel_id]
            if view.visible_on_floor:
                node.show()
                if view.status == ParcelStatus.ASSIGNED and view.assigned_robot in self.robots:
                    a = self.robots[view.assigned_robot].accent
                    ring.show()
                    ring.setColorScale(a[0], a[1], a[2], 0.7 + 0.3 * math.sin(now * 4))
                else:
                    ring.hide()
            else:
                node.hide()
                ring.hide()
        # Delivered parcels: a carton flies from the delivering robot's lift to the
        # dock pallet (0.8 s arc), then stays stacked. Triggered by the real
        # DELIVERED status, so the animation never precedes the simulated action.
        stacks = self.sync.delivered_per_station(engine)
        for sid, pids in stacks.items():
            st = self.vis.stations.get(sid) if self.vis else None
            if st is None:
                continue
            for k, pid in enumerate(pids):
                if pid in self.stack_nodes:
                    continue
                slot = (((k % 4) % 2 - 0.5) * 0.42, ((k % 4) // 2 - 0.5) * 0.42, (k // 4) * 0.3)
                box = make_carton(f"stacked-{pid}", size=0.34)
                box.reparentTo(st.stack_anchor)
                box.setPos(*slot)
                self.stack_nodes[pid] = box
                parcel = engine.warehouse.parcels[pid]
                if pid not in self.stack_seen and parcel.delivered_step == engine.step_count:
                    deliverer = next((r for r in engine.warehouse.robots
                                      if r.delivery_history and r.delivery_history[-1].parcel_id == pid), None)
                    if deliverer is not None and deliverer.robot_id in poses:
                        p = poses[deliverer.robot_id]
                        self.transfers.append((box, (p.x, p.y, 0.55), slot, now))
                self.stack_seen.add(pid)
        alive = []
        for box, start, slot, t0 in self.transfers:
            if box.isEmpty():
                continue
            t = (now - t0) / 0.8
            if t >= 1.0:
                box.setPos(*slot)
                continue
            k = t * t * (3 - 2 * t)
            anchor = box.getParent().getPos(self.base.render)
            ex, ey, ez = anchor.x + slot[0], anchor.y + slot[1], anchor.z + slot[2]
            box.setPos(self.base.render, start[0] + (ex - start[0]) * k, start[1] + (ey - start[1]) * k,
                       start[2] + (ez - start[2]) * k + math.sin(math.pi * k) * 0.6)
            alive.append((box, start, slot, t0))
        self.transfers = alive
        if self.vis:
            for belt in self.vis.belts:
                belt.setTexOffset(TextureStage.getDefault(), 0, -now * 0.6)
            for st in self.vis.stations.values():
                pulse = 0.6 + 0.4 * math.sin(now * 2 + st.station_id)
                st.beacon.setColorScale(pulse, 1, pulse, 1)

    # ------------------------------------------------------------------ overlays
    def _update_overlays(self, engine, poses: dict[int, RobotPose], now: float) -> None:
        fx = self.effects
        assert fx is not None
        show_search = self.show_compare or (self.presenting and self.shot is not None and self.shot.show_search)
        for robot in engine.warehouse.robots:
            m = self.robots[robot.robot_id]
            visible = self.show_paths and (not self.presenting or robot.robot_id == self.selected
                                           or (self.shot is not None and self.shot.mode in ("overview", "topdown")))
            fx.update_path(robot.robot_id, robot.planned_path, m.accent, visible, now)
            fx.update_marker(robot.robot_id, robot.destination, m.accent, visible, now)
        self.comparison = None
        if show_search:
            robot = engine.warehouse.robots[self.selected]
            target = engine.warehouse.parcel_target(robot)
            key = (id(engine), robot.robot_id, robot.position, target)
            if key != self._cmp_key:
                self._cmp_key = key
                self._cmp_results = compare(engine.warehouse.grid, robot.position, target) if target else None
                fx.set_comparison(key, self._cmp_results, now)
            self.comparison = self._cmp_results
        fx.animate_comparison(show_search and self.comparison is not None, now)
        fx.update_events(self.sync.live_effects(now), now)

    # ------------------------------------------------------------------ camera
    def _update_camera(self, dt: float, poses: dict[int, RobotPose]) -> None:
        override = None
        if self.presenting and self.shot is not None:
            override = self.shot.mode
        pose = poses.get(self.selected)
        rp = (pose.x, pose.y, pose.heading) if pose else None
        rig = self.camera.update(dt, self.time, rp, override)
        tracking = self.presenting or self.camera.mode == "follow"
        view = self._unoccluded(rig, dt, allow_turn=tracking and not self.camera.in_intro)
        cam = self.base.camera
        cam.setPos(*view.camera_position())
        cam.lookAt(view.tx, view.ty, view.tz)
        if self._drag is not None:
            m = self._mouse()
            if m is not None and self._mouse_last is not None:
                dx, dy = m[0] - self._mouse_last[0], m[1] - self._mouse_last[1]
                w, h = self._window_size()
                if self._drag == "orbit":
                    self.camera.orbit(-dx * 160, dy * 90)
                else:
                    self.camera.pan(-dx * w / 2, -dy * h / 2)
            self._mouse_last = m

    def _blocked(self, rig) -> bool:
        """True if the line of sight from the rig target to the camera passes through a rack."""
        grid = self.ctl.engine.warehouse.grid
        h = grid.height
        cx, cy, cz = rig.camera_position()
        for i in range(1, 13):
            t = i / 12
            x = rig.tx + (cx - rig.tx) * t
            y = rig.ty + (cy - rig.ty) * t
            z = rig.tz + (cz - rig.tz) * t
            if z > RACK_HEIGHT + 0.25:
                continue
            col, row = int(round(x)), int(round(h - 1 - y))
            if grid.is_shelf((row, col)):
                return True
        return False

    def _unoccluded(self, rig, dt: float, allow_turn: bool = False):
        """Camera occlusion avoidance (camera only, never the simulation).

        For tracking shots the camera first looks for the nearest heading around
        the target with a clear line of sight (usually along an aisle); if none
        exists, or turning is not allowed, it steepens the pitch until clear.
        Both corrections are eased so the camera never jumps.
        """
        want_heading = 0.0
        view = rig.copy()
        if allow_turn and self._blocked(view):
            for off in (20, -20, 45, -45, 70, -70, 90, -90, 135, -135, 180):
                view.heading = rig.heading + off
                if not self._blocked(view):
                    want_heading = float(off)
                    break
        view = rig.copy()
        view.heading = rig.heading + want_heading
        needed = view.pitch
        while needed > -84 and self._blocked(view):
            needed -= 4
            view.pitch = needed
        self._occl_heading += angle_diff(want_heading, self._occl_heading) * damping(dt, 3.0)
        self._occl_pitch += (min(0.0, needed - rig.pitch) - self._occl_pitch) * damping(dt, 5.0)
        view = rig.copy()
        view.heading = rig.heading + self._occl_heading
        view.pitch = max(-89.0, rig.pitch + self._occl_pitch)
        return view

    # ------------------------------------------------------------------ HUD
    def _update_hud(self, engine) -> None:
        hud = self.hud
        ctl = self.ctl
        cfg = ctl.config
        st = ctl.status.value
        m = engine.metrics()
        if self.camera.in_intro:
            sc = self.ctl.scenario
            self.hud.set_text("p:title", "Smart Warehouse AI")
            self.hud.set_text("p:sub", f"{sc.name} · {len(engine.warehouse.robots)} autonomous robots · "
                                       f"{len(engine.warehouse.parcels)} parcels · BFS & A* · Simple Reflex, "
                                       "Goal-Based and Utility-Based agents")
            self.hud.set_text("p:live", "OPENING SEQUENCE  ·  click or press Esc to skip", H.TEXT2)
            self.hud.set_text("p:stats", "")
            self.hud.set_text("p:mode", "SMART WAREHOUSE AI  ·  3D DIGITAL TWIN")
            self.hud.set_text("p:hint", "")
            return
        if self.presenting:
            self.hud.set_text("p:mode", "SMART WAREHOUSE AI  ·  PRESENTATION MODE")
            self.hud.set_text("p:hint", "P / Esc  exit presentation")
            shot = self.shot
            self.hud.set_text("p:title", shot.caption if shot else "")
            self.hud.set_text("p:sub", shot.subcaption if shot else "")
            self.hud.set_text("p:live", f"{st}  ·  STEP {engine.step_count}  ·  "
                                        f"{AGENT_LABELS[cfg.agent_type].upper()} / {ALGORITHM_LABELS[cfg.algorithm]}",
                              H.STATUS_COLORS[st])
            self.hud.set_text("p:stats", f"DELIVERED {m['parcels_delivered']}/{m['parcels_total']}     "
                                         f"DISTANCE {m['total_distance']}     CONFLICTS {m['conflicts']}     "
                                         f"COMPLETION {m['completion_pct']:.0f}%")
            return
        if not hud.visible:
            return
        hud.set_text("top:status", st, H.STATUS_COLORS[st])
        hud.set_text("top:robots", f"{len(engine.warehouse.robots)} AMRs")
        hud.set_text("top:brain", f"{AGENT_LABELS[cfg.agent_type]} · {ALGORITHM_LABELS[cfg.algorithm]}")
        hud.set_text("top:speed", f"{ctl.ticks_per_second:g} ticks/s")
        hud.set_text("top:step", f"{engine.step_count} / {engine.max_steps}")
        cam_label = "Intro flythrough" if self.camera.in_intro else MODE_LABELS[self.camera.mode]
        hud.set_text("top:cam", cam_label)
        hud.set_button("start", enabled=ctl.status == SimStatus.READY)
        hud.set_button("pause", enabled=ctl.status == SimStatus.RUNNING)
        hud.set_button("resume", enabled=ctl.status == SimStatus.PAUSED)
        hud.set_button("step", enabled=ctl.status != SimStatus.FINISHED)
        hud.set_selector("layout", cfg.layout)
        hud.set_selector("agent", cfg.agent_type)
        hud.set_selector("algorithm", cfg.algorithm)
        hud.set_selector("robots", str(cfg.num_robots))
        hud.set_selector("seed", str(cfg.seed), enabled=cfg.layout == "random")
        hud.set_selector("allocation", cfg.allocation)
        hud.set_selector("speed", str(ctl.speed_index))
        hud.set_selector("quality", self.quality)
        hud.set_button("paths", active=self.show_paths)
        hud.set_button("compare", active=self.show_compare)
        for mode in MODE_LABELS:
            hud.set_button(f"cam:{mode}", active=(mode == self.camera.mode and not self.camera.in_intro))
        hud.set_button("analytics", active=self.analytics.visible)
        n = len(engine.warehouse.robots)
        for i in range(3):
            b = hud.buttons.get(f"robot{i}")
            if b is None:
                continue
            if i < n:
                b.show()
                hud.set_button(f"robot{i}", active=i == self.selected)
            else:
                b.hide()
        # metrics
        hud.set_text("m:deliv", str(m["parcels_delivered"]), H.GOOD)
        hud.set_text("m:pending", str(m["parcels_pending"]), H.WARN if m["parcels_pending"] else H.GOOD)
        hud.set_text("m:dist", str(m["total_distance"]))
        hud.set_text("m:steps", str(m["steps"]))
        hud.set_text("m:conf", str(m["conflicts"]), H.WARN if m["conflicts"] else H.TEXT)
        hud.set_text("m:pct", f"{m['completion_pct']:.0f}%", H.GOOD if m["completion_pct"] >= 100 else H.TEXT)
        hud.set_text("m:term", engine.termination_reason if engine.done else LAYOUT_LABELS.get(cfg.layout, ""))
        hud.set_progress(m["completion_pct"] / 100)
        hud.set_text("banner", ctl.message)
        # robot inspector
        robot = engine.warehouse.robots[self.selected]
        wh = engine.warehouse
        hud.set_text("r:agent", AGENT_LABELS[cfg.agent_type])
        state_col = {"IDLE": H.MUTED, "NAVIGATING_TO_PARCEL": H.INFO, "PICKING_UP": H.WARN,
                     "NAVIGATING_TO_DROPOFF": H.ACCENT_HOVER, "DELIVERING": H.GOOD}[robot.state.value]
        state_label = robot.state.label + ("  (yielding)" if robot.is_yielding else "")
        hud.set_text("r:state", f"R{robot.robot_id}  ·  {state_label}", state_col)
        hud.set_text("r:action", robot.current_action.label)
        hud.set_text("r:pos", str(robot.position))
        hud.set_text("r:parcel", f"P{robot.assigned_parcel}" if robot.assigned_parcel is not None else "-")
        hud.set_text("r:carry", f"P{robot.carrying}" if robot.carrying is not None else "nothing")
        if robot.carrying is not None:
            stn = wh.station_for(wh.parcels[robot.carrying])
            dest = f"{stn.label} {stn.position}"
        elif robot.destination is not None:
            dest = f"parcel at {robot.destination}"
        else:
            dest = "-"
        hud.set_text("r:dest", dest)
        hud.set_text("r:dist", f"{robot.distance_travelled} cells")
        hud.set_text("r:deliv", str(robot.deliveries_completed))
        hud.set_text("r:waits", f"{robot.waits} / {robot.conflicts}")
        rec = engine.last_decision.get(robot.robot_id)
        if rec is None:
            hud.set_text("r:decision-head", "Waiting for the first decision")
            hud.set_text("r:decision", "Press START or STEP to run the agents.")
            hud.set_text("r:cands", "")
        else:
            hud.set_text("r:decision-head", f"Step {rec.step}  ·  {rec.chosen.label}  ->  {rec.outcome}")
            reason = rec.reason if len(rec.reason) < 260 else rec.reason[:257] + "..."
            hud.set_text("r:decision", reason)
            lines = []
            for c in rec.candidates[:5]:
                mark = ">" if c.action == rec.chosen else (" " if c.valid else "x")
                score = f"U={c.score:+7.2f}" if c.score is not None else (c.note[:18] if c.note else "")
                lines.append(f"{mark} {c.action.label:<15}{score}")
            hud.set_text("r:cands", "\n".join(lines))
        agent = engine.agents[robot.robot_id]
        stats = agent.pathfinder.stats
        if self.comparison:
            b_, a_ = self.comparison["bfs"], self.comparison["astar"]
            text = (f"{'':<14}{'BFS':>8}{'A*':>8}\n"
                    f"{'path length':<14}{str(b_.cost):>8}{str(a_.cost):>8}\n"
                    f"{'expanded':<14}{b_.nodes_expanded:>8}{a_.nodes_expanded:>8}\n"
                    f"{'runtime ms':<14}{b_.runtime_ms:>8.3f}{a_.runtime_ms:>8.3f}")
        elif agent.uses_search:
            text = (f"{ALGORITHM_LABELS[cfg.algorithm]} · {stats.searches} searches\n"
                    f"{stats.nodes_expanded:,} nodes expanded\n{stats.runtime_ms:.2f} ms total\n"
                    "Press C for BFS vs A* explored cells")
        else:
            text = "Simple Reflex: no search\n(acts on adjacent cells only)\nPress C to see BFS vs A*"
        hud.set_text("r:search", text)

    # ================================================================== public helpers
    def frame(self, dt: float = 1 / 60) -> None:
        """Advance one frame deterministically (used by tests and screenshots)."""
        self._update(dt)
        self.base.graphicsEngine.renderFrame()

    def screenshot(self, path: Path) -> Path:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.base.graphicsEngine.renderFrame()
        self.base.graphicsEngine.renderFrame()
        self.base.win.saveScreenshot(Filename.fromOsSpecific(str(path)))
        return Path(path)

    def run(self) -> None:
        self.base.run()

    def destroy(self) -> None:
        self.base.destroy()


def maximize_window(base) -> None:
    props = WindowProperties()
    props.setMaximized(True)
    base.win.requestProperties(props)
