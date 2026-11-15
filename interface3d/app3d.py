"""3D digital-twin application (Panda3D).

Frame loop (``_update``):
    1. SimulationController.update(dt)  - fixed tick rate, independent of FPS
       (tick listeners feed the FleetTracker once per real tick)
    2. VisualSync                        - poses / effects from the real engine state
    3. scene update                      - robots, parcels, dock stacks, overlays
    4. camera / presentation director    - camera only, never the simulation
    5. shadow scheduling                 - re-render the shadow map only if needed
    6. HUD / analytics                   - text and buttons (cached per tick)

The simulation engine, agents, search and coordination are the unchanged
modules used by the 2D dashboard and the experiment runner.
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Optional

from panda3d.core import (
    AmbientLight,
    AntialiasAttrib,
    BitMask32,
    DirectionalLight,
    Filename,
    Fog,
    Point2,
    Point3,
    TextNode,
    TextureStage,
    WindowProperties,
    loadPrcFileData,
)

from algorithms.pathfinder import compare
from config.settings import AGENT_LABELS, ALGORITHM_LABELS, LAYOUT_LABELS, SimulationConfig
from environment.parcel import ParcelStatus
from interface3d import hud as H
from interface3d.camera import MODE_KEYS, MODE_LABELS, CameraController
from interface3d.effects import EffectsLayer, HeatmapLayer
from interface3d.geometry import MeshBuilder
from interface3d.intelligence import FleetTracker
from interface3d.robot_model import (LIFT_DISC_H, LIFT_REST_Z, LIFT_TRAVEL, STATE_LIGHT, WAIT_LIGHT, YIELD_LIGHT,
                                     RobotModel, build_robot)
from interface3d.sync import (PresentationDirector, RobotPose, Shot, VisualSync, angle_diff, damping,
                              world_xy)
from interface3d.warehouse_scene import (RACK_HEIGHT, STAND_TOP, WarehouseVisual, build_warehouse, dock_color,
                                         make_carton, make_marker_ring)
from simulation.simulation_controller import SimulationController
from simulation.state_manager import SimStatus

#: shadows: shadow-map size (0 = off); shadow_interval: minimum frames between
#: shadow-map refreshes while something moves (the map is never refreshed when
#: nothing that casts a shadow has changed).
QUALITY_PRESETS = {
    "low": {"shadows": 0, "shadow_interval": 0, "msaa": False, "bloom": False, "ao": False, "fog": False},
    "medium": {"shadows": 2048, "shadow_interval": 2, "msaa": True, "bloom": True, "ao": False, "fog": True},
    "high": {"shadows": 4096, "shadow_interval": 1, "msaa": True, "bloom": True, "ao": True, "fog": True},
}
SHADOW_MASK = BitMask32.bit(1)
BANNER_SECONDS = 6.0
TRANSFER_SECONDS = 1.6


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
        b.camLens.setNearFar(0.25, 400)
        self.offscreen = offscreen
        self.quality = quality if quality in QUALITY_PRESETS else "medium"
        self.ctl = SimulationController(config)
        self.sync = VisualSync()
        self.director = PresentationDirector()
        self.tracker = FleetTracker()
        self.ctl.tick_listeners.append(self.tracker.on_tick)
        self.selected = 0
        self.show_paths = True
        self.show_compare = False
        self.show_heatmap = False
        self.show_indicators = True
        self.presenting = False
        self.shot: Optional[Shot] = None
        self.filters = None
        self.world = None
        self.vis: Optional[WarehouseVisual] = None
        self.robots: dict[int, RobotModel] = {}
        self.indicators: dict[int, tuple] = {}
        self.parcel_nodes: dict[int, tuple] = {}
        self.stack_nodes: dict[int, object] = {}
        self.stack_seen: set[int] = set()
        self.transfers: list[tuple] = []
        self.effects: Optional[EffectsLayer] = None
        self.heatmap: Optional[HeatmapLayer] = None
        self.comparison = None
        self._cmp_key = None
        self._cmp_results = None
        self._scene_key = None
        self._mouse_last: Optional[tuple[float, float]] = None
        self._drag: Optional[str] = None
        self.time = 0.0
        self.frame_count = 0
        self.help_root = None
        self._occl_pitch = 0.0
        self._occl_heading = 0.0
        self._tick_cache_step = -1
        self._metrics: dict = {}
        self._parcel_views: list = []
        self._stacks: dict = {}
        self._banner_msg = ""
        self._banner_time = -1e9
        self._last_ctl_message = ""
        self._shadow_sig = None
        self._shadow_force = True
        self._shadow_last_frame = -999
        self.shadow_updates = 0
        self.fog = Fog("depth-fog")
        self.fog.setColor(0.02, 0.03, 0.05)
        self.fog.setExpDensity(0.004)

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
        amb.setColor((0.25, 0.28, 0.34, 1))
        r.setLight(r.attachNewNode(amb))
        sun = DirectionalLight("sun")
        sun.setColor((0.80, 0.76, 0.70, 1))
        sun.setCameraMask(SHADOW_MASK)      # the shadow camera only sees shadow casters
        self.sun = r.attachNewNode(sun)
        self.sun.setHpr(-32, -58, 0)
        r.setLight(self.sun)
        fill = DirectionalLight("fill")
        fill.setColor((0.18, 0.22, 0.30, 1))
        fnp = r.attachNewNode(fill)
        fnp.setHpr(150, -35, 0)
        r.setLight(fnp)

    def _clear_scene(self) -> None:
        """Release every node of the previous scene (layout change / reset)."""
        if self.effects is not None:
            self.effects.destroy()
            self.effects = None
        if self.heatmap is not None:
            self.heatmap.destroy()
            self.heatmap = None
        if self.world is not None:
            self.world.removeNode()
            self.world = None
        self.robots.clear()
        self.indicators.clear()
        self.parcel_nodes.clear()
        self.stack_nodes.clear()
        self.stack_seen.clear()
        self.transfers.clear()

    def rebuild_scene(self) -> None:
        """(Re)build the 3D world for the current scenario / robot count."""
        self._clear_scene()
        b = self.base
        engine = self.ctl.engine
        self.world = b.render.attachNewNode("world")
        self.vis = build_warehouse(self.ctl.scenario, self.world, self.quality)
        for np_ in self.vis.no_shadow:
            np_.hide(SHADOW_MASK)
        h = engine.warehouse.grid.height
        self.robots = {r.robot_id: build_robot(r.robot_id, self.world, self.hud.bold)
                       for r in engine.warehouse.robots}
        for m in self.robots.values():
            for np_ in (m.halo, m.select_ring, m.select_beam, m.status):
                np_.hide(SHADOW_MASK)
            self.indicators[m.robot_id] = self._make_indicator(m)
        for p in engine.warehouse.parcels.values():
            node = make_carton(f"parcel-{p.parcel_id}", label_color=dock_color(p.destination_id))
            node.reparentTo(self.world)
            x, y = world_xy(p.origin, h)
            node.setPos(x, y, STAND_TOP)
            ring = make_marker_ring((1, 1, 1, 0.9), 0.3, 0.36)
            ring.reparentTo(self.world)
            ring.setPos(x, y, 0.011)
            ring.hide()
            ring.hide(SHADOW_MASK)
            self.parcel_nodes[p.parcel_id] = (node, ring)
        self.effects = EffectsLayer(self.world, h)
        self.effects.root.hide(SHADOW_MASK)
        self.heatmap = HeatmapLayer(self.world, engine.warehouse.grid, h)
        self.heatmap.root.hide(SHADOW_MASK)
        self._cmp_key = None
        self._tick_cache_step = -1
        self.sync.reset()
        self.tracker.reset()
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
        lens = self.sun.node().getLens()
        gw, gh = self.vis.width + 2 * 2.5 + 1, self.vis.height + 2 * 2.5 + 1
        lens.setFilmSize(gw * 1.05, gh * 1.25)
        lens.setNearFar(1, 120)
        self.sun.setPos(cx, cy, 0)
        self.sun.setPos(self.sun, 0, -50, 0)
        self._scene_key = (id(self.ctl.scenario), len(engine.warehouse.robots))
        self._shadow_force = True

    def _make_indicator(self, m: RobotModel) -> tuple:
        """Floating status tag above a robot: name, activity and utilisation bar."""
        root = m.root.attachNewNode("indicator")
        root.setZ(1.05)
        root.setBillboardPointEye()
        root.setLightOff(1)
        root.setShaderOff(1)
        root.setDepthWrite(False)
        root.setBin("fixed", 30)
        root.hide(SHADOW_MASK)
        bg = MeshBuilder("tag")
        bg.quad((-0.3, 0, -0.01), (0.3, 0, -0.01), (0.3, 0, 0.15), (-0.3, 0, 0.15), (0, -1, 0),
                (0.03, 0.05, 0.09, 0.8))
        bg.quad((-0.3, 0, -0.01), (-0.28, 0, -0.01), (-0.28, 0, 0.15), (-0.3, 0, 0.15), (0, -1, 0), m.accent)
        bnp = root.attachNewNode(bg.node())
        from panda3d.core import TransparencyAttrib
        bnp.setTransparency(TransparencyAttrib.MAlpha)
        tn = TextNode(f"tag-{m.robot_id}")
        tn.setAlign(TextNode.ALeft)
        tn.setTextColor(1, 1, 1, 1)
        if self.hud.bold is not None:
            tn.setFont(self.hud.bold)
        tnp = root.attachNewNode(tn)
        tnp.setScale(0.11)
        tnp.setPos(-0.34, -0.001, 0.085)
        bar_bg = MeshBuilder("bar-bg")
        bar_bg.quad((-0.34, 0, 0.02), (0.36, 0, 0.02), (0.36, 0, 0.055), (-0.34, 0, 0.055), (0, -1, 0),
                    (0.2, 0.25, 0.35, 1))
        root.attachNewNode(bar_bg.node()).setY(-0.001)
        bar = MeshBuilder("bar")
        bar.quad((0, 0, 0.02), (0.7, 0, 0.02), (0.7, 0, 0.055), (0, 0, 0.055), (0, -1, 0), (1, 1, 1, 1))
        bar_np = root.attachNewNode(bar.node())
        bar_np.setPos(-0.34, -0.002, 0)
        return root, tn, bar_np

    # ================================================================== quality / shadows
    def apply_quality(self, quality: str) -> None:
        self.quality = quality if quality in QUALITY_PRESETS else "medium"
        preset = QUALITY_PRESETS[self.quality]
        sun = self.sun.node()
        if preset["shadows"]:
            sun.setShadowCaster(True, preset["shadows"], preset["shadows"])
        else:
            sun.setShadowCaster(False)
        self._shadow_force = True
        if preset["fog"]:
            self.base.render.setFog(self.fog)
        else:
            self.base.render.clearFog()
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
                                    intensity=0.55, size="small") is not False and ok
                if preset["ao"]:
                    ok = f.setAmbientOcclusion(numsamples=16, radius=0.06, amount=1.6, strength=0.012,
                                               falloff=0.000002) is not False and ok
                self.filters = f
                if not ok:
                    self.notify("Some post-processing effects are not supported on this GPU.")
            except Exception as exc:  # noqa: BLE001 - degrade gracefully on unsupported GPUs
                self.filters = None
                self.notify(f"Post-processing disabled: {exc}")
        self._layout()

    def _shadow_buffer(self):
        if not self.sun.node().isShadowCaster():
            return None
        try:
            return self.sun.node().getShadowBuffer(self.base.win.getGsg())
        except Exception:  # noqa: BLE001 - buffer not created yet
            return None

    def _schedule_shadows(self, poses: dict[int, RobotPose]) -> None:
        """Re-render the shadow map only when a shadow caster moved/changed.

        Static racks never change, so a paused or idle warehouse costs no shadow
        pass at all; while robots move the map refreshes at most every
        ``shadow_interval`` frames (quality preset).
        """
        buf = self._shadow_buffer()
        if buf is None:
            return
        sig = (tuple((round(p.x, 3), round(p.y, 3), round(p.heading, 1), round(p.lift, 2), p.carrying)
                     for p in poses.values()),
               tuple(not n.isHidden() for n, _ in self.parcel_nodes.values()),
               len(self.stack_nodes), bool(self.transfers))
        interval = QUALITY_PRESETS[self.quality]["shadow_interval"] or 1
        changed = sig != self._shadow_sig
        due = self.frame_count - self._shadow_last_frame >= interval
        active = self._shadow_force or (changed and due)
        buf.setActive(active)
        if active:
            self._shadow_sig = sig
            self._shadow_last_frame = self.frame_count
            self._shadow_force = False
            self.shadow_updates += 1

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
        hud.finalize(root)
        hud.root = saved
        self.presentation_root = root
        if not (self.presenting or self.camera.in_intro):
            root.hide()

    # ================================================================== opening sequence
    def start_intro(self) -> None:
        """Letterboxed opening flythrough; the command-center HUD appears afterwards."""
        self.hud.set_visible(False)
        self.presentation_root.show()
        self.camera.start_intro(self._end_intro)
        self._apply_film_offset()

    def _end_intro(self) -> None:
        if not self.presenting:
            self.presentation_root.hide()
            self.hud.set_visible(True)
        self._apply_film_offset()

    # ================================================================== messages
    def notify(self, message: str) -> None:
        """Timed banner message (warnings / errors shown to the user)."""
        self._banner_msg = message
        self._banner_time = self.time

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

    def _with_loading(self, message: str, action) -> None:
        """Show a loading card for the (synchronous) scene rebuild."""
        overlay = None
        if self.hud.size != (0, 0):
            overlay = self.hud.show_loading(message)
            self.base.graphicsEngine.renderFrame()
        try:
            action()
        finally:
            if overlay is not None:
                overlay.removeNode()

    def cmd_config(self, **changes) -> None:
        from config.settings import LAYOUT_LABELS as LL
        target = LL.get(changes.get("layout", self.ctl.config.layout), "warehouse")

        def apply() -> None:
            self.ctl.apply_config(**changes)
            self.selected = min(self.selected, self.ctl.config.num_robots - 1)
            self.rebuild_scene()

        self._with_loading(f"Loading {target}", apply)

    def cmd_speed(self, index: int) -> None:
        self.ctl.set_speed_index(index)

    def cmd_quality(self, quality: str) -> None:
        def apply() -> None:
            self.apply_quality(quality)
            self.rebuild_scene()

        self._with_loading(f"Graphics quality: {quality.title()}", apply)

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

    def cmd_toggle_heatmap(self) -> None:
        self.show_heatmap = not self.show_heatmap

    def cmd_toggle_indicators(self) -> None:
        self.show_indicators = not self.show_indicators

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
        bw, bh = 640, 560
        x, y = (w - bw) / 2, (h - bh) / 2
        hud._frame(0, 0, w, h, (0, 0, 0, 0.6))
        hud._frame(x, y, bw, bh, H.PANEL_SOLID)
        hud._frame(x, y, bw, 3, H.ACCENT)
        hud.text(None, "Keyboard & mouse", x + 24, y + 20, 20, H.TEXT, bold=True)
        cams = "  ".join(f"{MODE_KEYS[m]} {MODE_LABELS[m]}" for m in MODE_KEYS)
        lines = [("Space", "start / pause / resume"), ("S", "step one tick"), ("R", "reset scenario"),
                 ("1 2 3 / Tab", "select robot (or click it)"), ("Cameras", cams),
                 ("Right drag", "orbit camera"), ("Middle drag", "pan camera"), ("Wheel", "zoom"),
                 ("C", "BFS vs A* explored cells"), ("M", "congestion heatmap"), ("I", "robot status tags"),
                 ("P", "presentation mode"), ("A", "analytics"), ("H", "hide HUD"), ("Q", "graphics quality"),
                 ("+ / -", "simulation speed"), ("Esc", "close / skip / exit")]
        yy = y + 64
        for k, v in lines:
            hud.text(None, k, x + 28, yy, 13, H.ACCENT_HOVER, bold=True)
            hud.text(None, v, x + 170, yy, 13, H.TEXT2, wrap=bw - 200)
            yy += 26 if k != "Cameras" else 44
        hud.finalize(root)
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
        for mode, key in MODE_KEYS.items():
            acc(key.lower(), lambda m=mode: self.cmd_camera(m))
        acc("c", self.cmd_toggle_compare)
        acc("m", self.cmd_toggle_heatmap)
        acc("i", self.cmd_toggle_indicators)
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
        acc("wheel_up", lambda: self._zoom(0.88))
        acc("wheel_down", lambda: self._zoom(1.14))
        acc("window-event", self._on_window_event)

    def _mouse(self) -> Optional[tuple[float, float]]:
        mw = self.base.mouseWatcherNode
        if mw is None or not mw.hasMouse():
            return None
        m = mw.getMouse()
        return (m.getX(), m.getY())

    def _mouse_pixels(self, m: tuple[float, float]) -> tuple[float, float]:
        w, h = self._window_size()
        return ((m[0] + 1) / 2 * w, (1 - m[1]) / 2 * h)

    def _over_ui(self) -> bool:
        m = self._mouse()
        return m is not None and (self.analytics.visible or self.hud.blocks(*self._mouse_pixels(m)))

    def _zoom(self, factor: float) -> None:
        if not self._over_ui():
            self.camera.skip_intro()
            self.camera.zoom(factor)

    def _start_drag(self, kind: str) -> None:
        if self._over_ui():
            return
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
        if m is None or self._over_ui():
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

    def _refresh_tick_cache(self, engine) -> None:
        """Data that only changes when the simulation ticks (computed once per tick)."""
        if engine.step_count == self._tick_cache_step:
            return
        self._tick_cache_step = engine.step_count
        self._metrics = engine.metrics()
        self._parcel_views = self.sync.parcel_views(engine)
        self._stacks = self.sync.delivered_per_station(engine)
        self._dock_progress = self.tracker.station_progress(engine)

    def _update(self, dt: float) -> None:
        self.time += dt
        self.frame_count += 1
        now = self.time
        ctl = self.ctl
        events = ctl.update(dt)
        engine = ctl.engine
        if (id(ctl.scenario), len(engine.warehouse.robots)) != self._scene_key:
            self.rebuild_scene()
        if engine.step_count < self.tracker.last_step:
            self.tracker.reset()
        if events:
            self.sync.ingest_events(engine, events, now)
        self._refresh_tick_cache(engine)
        poses = self.sync.robot_poses(engine, ctl.animation_progress(), dt)
        self.selected = min(self.selected, len(engine.warehouse.robots) - 1)

        if self.presenting:
            self.shot = self.director.update(engine, events, dt)
            if self.shot.robot_id is not None:
                self.selected = self.shot.robot_id

        self._update_robots(engine, poses, now)
        self._update_parcels(engine, poses, now)
        self._update_overlays(engine, poses, now)
        self._update_camera(dt, poses, engine)
        self._schedule_shadows(poses)
        self._update_hud(engine)
        self.analytics.update()

    # ------------------------------------------------------------------ robots
    def _update_robots(self, engine, poses: dict[int, RobotPose], now: float) -> None:
        origins = {p.origin: p.parcel_id for p in engine.warehouse.parcels.values()}
        for rid, pose in poses.items():
            m = self.robots.get(rid)
            if m is None:
                continue
            m.root.setPos(pose.x, pose.y, 0.0)
            m.root.setH(pose.heading)
            for wnode in m.wheels_left:
                wnode.setP(pose.wheel_left)
            for wnode in m.wheels_right:
                wnode.setP(pose.wheel_right)
            m.lidar.setH((now * (720 if pose.moving else 300)) % 360)
            lift_z = LIFT_REST_Z + LIFT_TRAVEL * pose.lift
            m.lift.setZ(lift_z)
            if pose.carrying:
                m.cargo.show()
                # While still under its parcel stand, the carton rests on the stand
                # until the rising platform reaches it.
                robot = engine.warehouse.robots[rid]
                on_stand = origins.get(pose.cell) == robot.carrying
                top = lift_z + LIFT_DISC_H
                m.cargo.setZ(LIFT_DISC_H + (max(0.0, STAND_TOP - top) if on_stand else 0.0))
            else:
                m.cargo.hide()
            m.body.setZ(0.004 * math.sin(now * 22) * pose.speed)        # suspension vibration while driving
            if pose.yielding:
                col, pulse = YIELD_LIGHT, 0.5 + 0.5 * math.sin(now * 8)
            elif pose.waiting:
                col, pulse = WAIT_LIGHT, 1.0 if int(now * 3) % 2 == 0 else 0.2
            else:
                col, pulse = STATE_LIGHT[pose.state.value], 0.85 + 0.15 * math.sin(now * 2.5)
            k = 0.45 + 0.75 * pulse
            m.status.setColorScale(col[0] * k, col[1] * k, col[2] * k, 1)
            if pose.waiting or pose.yielding:
                m.halo.setColorScale(col[0], col[1], col[2], 0.35 + 0.45 * pulse)
                s = 1.0 + 0.08 * math.sin(now * 5)
            else:
                a = m.accent
                m.halo.setColorScale(a[0], a[1], a[2], 0.38)
                s = 1.0
            m.halo.setScale(s, s, 1)
            selected = rid == self.selected
            if selected:
                m.select_ring.show()
                a = m.accent
                glow = 0.6 + 0.3 * math.sin(now * 4)
                m.select_ring.setColorScale(a[0], a[1], a[2], glow)
                if not self.presenting:
                    m.select_beam.show()
                    m.select_beam.setColorScale(a[0], a[1], a[2], 0.5 + 0.3 * math.sin(now * 3))
                else:
                    m.select_beam.hide()
            else:
                m.select_ring.hide()
                m.select_beam.hide()
            self._update_indicator(engine, rid, pose)

    def _update_indicator(self, engine, rid: int, pose: RobotPose) -> None:
        root, tn, bar = self.indicators[rid]
        close_up = self.presenting or self.camera.mode in ("follow", "tracking", "focus")
        if not self.show_indicators or (close_up and rid == self.selected):
            root.hide()
            return
        root.show()
        robot = engine.warehouse.robots[rid]
        usage = self.tracker.usage.get(rid)
        act = usage.current if usage else ("idle" if not robot.has_task else "moving")
        util = self.tracker.utilization(rid)
        label = f"R{rid}  {act.upper()}"
        if tn.getText() != label:
            tn.setText(label)
            col = H.ACTIVITY_COLORS[act]
            bar.setColorScale(col[0], col[1], col[2], 1)
        bar.setSx(max(0.01, util))
        root.setZ(1.05 + (0.25 if pose.carrying else 0.0))

    # ------------------------------------------------------------------ parcels and dock stacks
    def _update_parcels(self, engine, poses: dict[int, RobotPose], now: float) -> None:
        for view in self._parcel_views:
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
        # Delivered parcels: the carton moves from the robot's lift onto the dock
        # conveyor, rolls along it and drops onto the pallet stack. Triggered by
        # the real DELIVERED status in the tick it happened.
        for sid, pids in self._stacks.items():
            st = self.vis.stations.get(sid) if self.vis else None
            if st is None:
                continue
            for k, pid in enumerate(pids):
                if pid in self.stack_nodes:
                    continue
                slot = (((k % 4) % 2 - 0.5) * 0.42, ((k % 4) // 2 - 0.5) * 0.42, (k // 4) * 0.3)
                parcel = engine.warehouse.parcels[pid]
                box = make_carton(f"stacked-{pid}", size=0.34, label_color=dock_color(sid))
                box.reparentTo(st.stack_anchor)
                box.setPos(*slot)
                self.stack_nodes[pid] = box
                if pid not in self.stack_seen and parcel.delivered_step == engine.step_count:
                    deliverer = next((r for r in engine.warehouse.robots
                                      if r.delivery_history and r.delivery_history[-1].parcel_id == pid), None)
                    if deliverer is not None and deliverer.robot_id in poses:
                        p = poses[deliverer.robot_id]
                        start = (p.x, p.y, LIFT_REST_Z + LIFT_TRAVEL * p.lift + LIFT_DISC_H)
                        self.transfers.append((box, start, st.conveyor_start, st.conveyor_end, slot, now))
                self.stack_seen.add(pid)
        alive = []
        for box, start, c0, c1, slot, t0 in self.transfers:
            if box.isEmpty():
                continue
            t = (now - t0) / TRANSFER_SECONDS
            if t >= 1.0:
                box.setPos(*slot)
                continue
            anchor = box.getParent().getPos(self.base.render)
            end = (anchor.x + slot[0], anchor.y + slot[1], anchor.z + slot[2])
            if t < 0.25:                                 # lift -> conveyor head
                k = t / 0.25
                k = k * k * (3 - 2 * k)
                pos = tuple(a + (b - a) * k for a, b in zip(start, c0))
                pos = (pos[0], pos[1], pos[2] + math.sin(math.pi * k) * 0.15)
            elif t < 0.7:                                # rolls along the conveyor
                k = (t - 0.25) / 0.45
                pos = tuple(a + (b - a) * k for a, b in zip(c0, c1))
            else:                                        # onto the pallet stack
                k = (t - 0.7) / 0.3
                k = k * k * (3 - 2 * k)
                pos = tuple(a + (b - a) * k for a, b in zip(c1, end))
                pos = (pos[0], pos[1], pos[2] + math.sin(math.pi * k) * 0.3)
            box.setPos(self.base.render, *pos)
            alive.append((box, start, c0, c1, slot, t0))
        self.transfers = alive
        if self.vis:
            for belt in self.vis.belts:
                belt.setTexOffset(TextureStage.getDefault(), 0, -now * 0.6)
            for st in self.vis.stations.values():
                pulse = 0.6 + 0.4 * math.sin(now * 2 + st.station_id)
                st.beacon.setColorScale(pulse, pulse, pulse, 1)
                done, total = self._dock_progress.get(st.station_id, (0, 0))
                label = f"D{st.station_id}   {done} / {total}"
                if st.sign_text is not None and st.sign_text.getText() != label:
                    st.sign_text.setText(label)
                    st.progress_fill.setSx(max(0.001, done / total if total else 0.0))

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
        if self.heatmap is not None:
            self.heatmap.set_visible(self.show_heatmap)
            if self.show_heatmap:
                self.heatmap.update(self.tracker.heat, self.tracker.version)

    # ------------------------------------------------------------------ camera
    def _shot_goal(self, engine, poses: dict[int, RobotPose]):
        """Cinematic framing of real positions for event / pathfinding shots."""
        shot = self.shot
        if shot is None or not shot.points:
            return None
        h = engine.warehouse.grid.height
        pts = [world_xy(c, h) for c in shot.points]
        if shot.robot_id in poses:
            p = poses[shot.robot_id]
            pts.append((p.x, p.y))
        if shot.station_id is not None and self.vis and shot.station_id in self.vis.stations:
            st = self.vis.stations[shot.station_id]
            pts.append(st.conveyor_end[:2])
            anchor = st.stack_anchor.getPos(self.base.render)
            pts.append((anchor.x, anchor.y))
        base_heading = -25.0 + 7.0 * (shot.serial % 5)
        heading = base_heading + shot.orbit * self.director.elapsed
        margin = 1.6 if shot.kind in ("pickup", "deliver") else 1.15
        return self.camera.frame_points(pts, shot.pitch, heading, margin=margin,
                                        min_distance=4.5 if shot.kind in ("pickup", "deliver") else 8.0)

    def _update_camera(self, dt: float, poses: dict[int, RobotPose], engine) -> None:
        override = goal = key = None
        if self.presenting and self.shot is not None:
            override = self.shot.mode
            goal = self._shot_goal(engine, poses)
            key = ("shot", self.shot.serial)
        pose = poses.get(self.selected)
        rp = (pose.x, pose.y, pose.heading) if pose else None
        rig = self.camera.update(dt, self.time, rp, override, goal, key)
        tracking = self.presenting or self.camera.mode in ("follow", "tracking")
        view = self._unoccluded(rig, dt, allow_turn=tracking and not self.camera.in_intro)
        if self.vis is not None and self.vis.overhead is not None:
            # roof trusses would cut across plan views: hide them when looking steeply down
            (self.vis.overhead.hide if view.pitch < -68 else self.vis.overhead.show)()
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
        m = self._metrics
        if ctl.message and ctl.message != self._last_ctl_message:
            self.notify(ctl.message)
        self._last_ctl_message = ctl.message
        if self.camera.in_intro:
            sc = self.ctl.scenario
            hud.set_text("p:title", "Smart Warehouse AI")
            hud.set_text("p:sub", f"{sc.name} · {len(engine.warehouse.robots)} autonomous robots · "
                                  f"{len(engine.warehouse.parcels)} parcels · BFS & A* · Simple Reflex, "
                                  "Goal-Based and Utility-Based agents")
            hud.set_text("p:live", "OPENING SEQUENCE  ·  click or press Esc to skip", H.TEXT2)
            hud.set_text("p:stats", "")
            hud.set_text("p:mode", "SMART WAREHOUSE AI  ·  3D DIGITAL TWIN")
            hud.set_text("p:hint", "")
            return
        if self.presenting:
            hud.set_text("p:mode", "SMART WAREHOUSE AI  ·  PRESENTATION MODE")
            hud.set_text("p:hint", "P / Esc  exit presentation")
            shot = self.shot
            hud.set_text("p:title", shot.caption if shot else "")
            hud.set_text("p:sub", shot.subcaption if shot else "")
            hud.set_text("p:live", f"{st}  ·  STEP {engine.step_count}  ·  "
                                   f"{AGENT_LABELS[cfg.agent_type].upper()} / {ALGORITHM_LABELS[cfg.algorithm]}",
                         H.STATUS_COLORS[st])
            hud.set_text("p:stats", f"DELIVERED {m['parcels_delivered']}/{m['parcels_total']}     "
                                    f"DISTANCE {m['total_distance']}     CONFLICTS {m['conflicts']}     "
                                    f"FLEET UTILIZATION {100 * self.tracker.fleet_utilization():.0f}%     "
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
        hud.set_button("heatmap", active=self.show_heatmap)
        hud.set_button("indicators", active=self.show_indicators)
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
        msg = self._banner_msg if self.time - self._banner_time < BANNER_SECONDS else ""
        hud.set_banner(msg)
        legend = []
        if self.show_compare:
            legend.append("BFS explored = blue tint · A* explored = orange squares")
        if self.show_heatmap:
            hot = self.tracker.hotspot()
            legend.append("Congestion heatmap: red = recent occupancy / waiting / conflicts"
                          + (f" · hotspot {hot[0]}" if hot else ""))
        hud.set_text("legend", "   |   ".join(legend))
        if hud.right_tab == "fleet":
            self._update_fleet_tab(engine)
            return
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
        hud.set_text("r:util", f"{100 * self.tracker.utilization(robot.robot_id):.0f}%")
        hud.set_text("r:waits", f"{robot.waits} / {robot.conflicts}")
        rec = engine.last_decision.get(robot.robot_id)
        if rec is None:
            hud.set_text("r:decision-head", "Waiting for the first decision")
            hud.set_text("r:decision", "Press START or STEP to run the agents.")
            hud.set_text("r:cands", "")
        else:
            hud.set_text("r:decision-head", f"Step {rec.step}  ·  {rec.chosen.label}  ->  {rec.outcome}")
            reason = rec.reason if len(rec.reason) < 230 else rec.reason[:227] + "..."
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

    def _update_fleet_tab(self, engine) -> None:
        hud = self.hud
        tr = self.tracker
        for act, n in tr.activity_counts().items():
            hud.set_text(f"f:act:{act}", str(n), H.ACTIVITY_COLORS[act] if n else H.MUTED)
        robots = engine.warehouse.robots
        for i in range(3):
            if i < len(robots):
                u = tr.usage.get(i)
                act = u.current if u else "idle"
                hud.set_text(f"f:rname{i}", f"R{i}")
                hud.set_text(f"f:rstate{i}", act, H.ACTIVITY_COLORS[act])
                util = tr.utilization(i)
                hud.set_text(f"f:rpct{i}", f"{100 * util:.0f}%")
                hud.set_progress(util, key=f"util{i}", width=H.RIGHT_W - 32, height=6)
            else:
                for k in ("rname", "rstate", "rpct"):
                    hud.set_text(f"f:{k}{i}", "")
                hud.set_progress(0, key=f"util{i}", width=H.RIGHT_W - 32, height=6)
        hud.set_text("f:fu", f"{100 * tr.fleet_utilization():.0f}%")
        hud.set_text("f:tp", f"{tr.throughput():.1f}")
        ct = tr.cycle_time(engine)
        hud.set_text("f:ct", f"{ct:.1f} ticks" if ct is not None else "-")
        hot = tr.hotspot()
        hud.set_text("f:hot", f"{hot[0]}  heat {hot[1]:.1f}" if hot else "-")
        hud.set_text("f:confl", str(len(tr.conflict_cells)))
        prog = self._dock_progress
        for i in range(4):
            if i in prog:
                done, total = prog[i]
                hud.set_text(f"f:dock{i}", f"Dock D{i}")
                hud.set_text(f"f:dockn{i}", f"{done} / {total}")
                hud.set_progress(done / total if total else 0, key=f"dock{i}", width=H.RIGHT_W - 32, height=5)
            else:
                hud.set_text(f"f:dock{i}", "")
                hud.set_text(f"f:dockn{i}", "")
                hud.set_progress(0, key=f"dock{i}", width=H.RIGHT_W - 32, height=5)

    # ================================================================== public helpers
    def visible_chunks(self) -> tuple[int, int]:
        """(chunks intersecting the camera frustum, total chunks) - what frustum culling keeps."""
        if self.vis is None:
            return (0, 0)
        frustum = self.base.camLens.makeBounds()
        frustum.xform(self.base.cam.getMat(self.base.render))
        visible = 0
        for chunk in self.vis.chunks:
            bounds = chunk.getBounds().makeCopy()
            bounds.xform(chunk.getParent().getMat(self.base.render))
            if frustum.contains(bounds):
                visible += 1
        return visible, len(self.vis.chunks)

    def frame(self, dt: float = 1 / 60) -> None:
        """Advance one frame deterministically (used by tests and screenshots)."""
        self._update(dt)
        self.base.graphicsEngine.renderFrame()

    def screenshot(self, path: Path) -> Path:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._shadow_force = True
        self._schedule_shadows(self.sync.robot_poses(self.ctl.engine, self.ctl.animation_progress(), 0.0))
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
