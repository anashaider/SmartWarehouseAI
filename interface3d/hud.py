"""Command-center HUD for the 3D digital twin (Panda3D DirectGUI, pixel layout).

Layout (pixels, rebuilt on window resize):
    top bar      : branding, status, active robots, agent/algorithm, speed, step, camera
    left panel   : run controls, warehouse / agent / algorithm / robots / allocation / seed,
                   speed, graphics quality, view toggles
    right panel  : selected robot inspector + AI decision explanation + BFS/A* stats
    bottom panel : deliveries, pending, distance, steps, conflicts, completion
    camera bar   : five camera modes, presentation mode, analytics
All buttons call real controller / app actions.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Callable, Optional, Sequence

from direct.gui import DirectGuiGlobals as DGG
from direct.gui.DirectGui import DirectButton, DirectFrame
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import Filename, NodePath, TextNode, TransparencyAttrib

from interface3d.geometry import rgba

# Palette (matches the 2D dashboard)
PANEL = (0.055, 0.08, 0.14, 0.88)
PANEL_SOLID = (0.055, 0.08, 0.14, 0.97)
CARD = (0.10, 0.14, 0.23, 0.95)
CARD_HOVER = (0.14, 0.19, 0.31, 1.0)
BORDER = (0.16, 0.22, 0.35, 1)
ACCENT = rgba("#3b82f6")
ACCENT_HOVER = rgba("#5b9bff")
TEXT = rgba("#e6ebf5")
TEXT2 = rgba("#aab4cc")
MUTED = rgba("#6f7c99")
GOOD = rgba("#22c55e")
WARN = rgba("#f59e0b")
BAD = rgba("#ef4444")
INFO = rgba("#38bdf8")
STATUS_COLORS = {"READY": INFO, "RUNNING": GOOD, "PAUSED": WARN, "FINISHED": rgba("#a78bfa")}

TOP_H = 58
LEFT_W = 258
RIGHT_W = 338
BOTTOM_H = 92
GAP = 10


def load_fonts(loader) -> tuple:
    """Segoe UI (Windows) / DejaVu fallbacks; Panda3D default font otherwise."""
    candidates = {
        "regular": ["C:/Windows/Fonts/segoeui.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"],
        "bold": ["C:/Windows/Fonts/segoeuib.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"],
        "mono": ["C:/Windows/Fonts/consola.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"],
    }
    out = []
    for key in ("regular", "bold", "mono"):
        font = None
        for path in candidates[key]:
            if Path(path).exists():
                try:
                    font = loader.loadFont(Filename.fromOsSpecific(path).getFullpath())
                    font.setPixelsPerUnit(64)
                    break
                except Exception:  # noqa: BLE001 - fall back to default font
                    font = None
        out.append(font)
    return tuple(out)


class Hud:
    def __init__(self, base, app) -> None:
        self.base = base
        self.app = app
        self.font, self.bold, self.mono = load_fonts(base.loader)
        self.root: Optional[NodePath] = None
        self.texts: dict[str, OnscreenText] = {}
        self._cache: dict[str, tuple] = {}
        self.buttons: dict[str, DirectButton] = {}
        self.selectors: dict[str, dict] = {}
        self.size = (0, 0)
        self.visible = True

    # ------------------------------------------------------------------ primitives
    def _frame(self, x: float, y: float, w: float, h: float, color=PANEL, parent=None) -> DirectFrame:
        f = DirectFrame(parent=parent or self.root, frameColor=color, frameSize=(0, w, -h, 0), pos=(x, 0, -y),
                        state=DGG.NORMAL if color[3] > 0.5 else DGG.DISABLED)
        f.setTransparency(TransparencyAttrib.MAlpha)
        return f

    def text(self, key: Optional[str], value: str, x: float, y: float, size: float = 13, color=TEXT,
             align: str = "left", bold: bool = False, wrap: Optional[float] = None, mono: bool = False,
             parent=None) -> OnscreenText:
        al = {"left": TextNode.ALeft, "center": TextNode.ACenter, "right": TextNode.ARight}[align]
        font = self.mono if mono else (self.bold if bold else self.font)
        t = OnscreenText(text=value, parent=parent or self.root, pos=(x, -y - size * 0.8), scale=size,
                         fg=color, align=al, font=font, mayChange=True,
                         wordwrap=(wrap / size) if wrap else None)
        if key:
            self.texts[key] = t
        return t

    def set_text(self, key: str, value: str, color=None) -> None:
        t = self.texts.get(key)
        if t is None:
            return
        sig = (value, color)
        if self._cache.get(key) == sig:
            return
        self._cache[key] = sig
        t.setText(value)
        if color is not None:
            t.setFg(color)

    def button(self, key: str, label: str, x: float, y: float, w: float, h: float,
               cmd: Callable[[], None], primary: bool = False, size: float = 13, parent=None) -> DirectButton:
        base = ACCENT if primary else CARD
        hover = ACCENT_HOVER if primary else CARD_HOVER
        b = DirectButton(parent=parent or self.root, text=label, text_font=self.bold, text_scale=size,
                         text_fg=TEXT if not primary else (1, 1, 1, 1),
                         text_pos=(w / 2, -h / 2 - size * 0.35), frameSize=(0, w, -h, 0), pos=(x, 0, -y),
                         frameColor=(base, base, hover, (0.08, 0.1, 0.16, 0.9)), relief=DGG.FLAT,
                         command=cmd, pressEffect=1, rolloverSound=None, clickSound=None)
        b.setTransparency(TransparencyAttrib.MAlpha)
        b._base_colors = (base, hover)  # type: ignore[attr-defined]
        self.buttons[key] = b
        return b

    def set_button(self, key: str, enabled: Optional[bool] = None, active: Optional[bool] = None,
                   label: Optional[str] = None) -> None:
        b = self.buttons.get(key)
        if b is None:
            return
        sig = (enabled, active, label)
        if self._cache.get("btn:" + key) == sig:
            return
        self._cache["btn:" + key] = sig
        if label is not None:
            b["text"] = label
        if enabled is not None:
            b["state"] = DGG.NORMAL if enabled else DGG.DISABLED
            b["text_fg"] = TEXT if enabled else MUTED
        if active is not None:
            base, hover = b._base_colors  # type: ignore[attr-defined]
            if active:
                b["frameColor"] = (ACCENT, ACCENT, ACCENT_HOVER, ACCENT)
                b["text_fg"] = (1, 1, 1, 1)
            else:
                b["frameColor"] = (base, base, hover, (0.08, 0.1, 0.16, 0.9))

    def selector(self, key: str, label: str, options: Sequence[tuple[str, str]], x: float, y: float, w: float,
                 on_change: Callable[[str], None]) -> None:
        self.text(None, label.upper(), x + 2, y, 10, MUTED, bold=True)
        by = y + 16
        h = 32
        self._frame(x, by, w, h, CARD)
        state = {"options": list(options), "on_change": on_change, "value": options[0][0]}

        def shift(d: int) -> None:
            opts = state["options"]
            idx = next((i for i, (v, _) in enumerate(opts) if v == state["value"]), 0)
            state["value"] = opts[(idx + d) % len(opts)][0]
            on_change(state["value"])

        self.button(f"{key}-prev", "‹", x, by, 30, h, lambda: shift(-1), size=20)
        self.button(f"{key}-next", "›", x + w - 30, by, 30, h, lambda: shift(1), size=20)
        self.text(f"sel:{key}", "", x + w / 2, by + 9, 13, TEXT, align="center", bold=True)
        self.selectors[key] = state

    def set_selector(self, key: str, value: str, enabled: bool = True) -> None:
        st = self.selectors.get(key)
        if st is None:
            return
        st["value"] = value
        display = next((d for v, d in st["options"] if v == value), value)
        self.set_text(f"sel:{key}", display, TEXT if enabled else MUTED)
        self.set_button(f"{key}-prev", enabled=enabled)
        self.set_button(f"{key}-next", enabled=enabled)

    def section(self, title: str, x: float, y: float, w: float) -> float:
        self.text(None, title.upper(), x, y, 11, ACCENT_HOVER, bold=True)
        self._frame(x, y + 19, w, 1, BORDER)
        return y + 28

    # ------------------------------------------------------------------ build
    def build(self, width: int, height: int) -> None:
        if self.root is not None:
            self.root.removeNode()
        self.texts.clear()
        self._cache.clear()
        self.buttons.clear()
        self.selectors.clear()
        self.size = (width, height)
        self.root = self.base.pixel2d.attachNewNode("hud")
        app = self.app
        W, H = width, height

        # ---------------- top bar
        self._frame(0, 0, W, TOP_H, PANEL_SOLID)
        self._frame(0, TOP_H - 2, W, 2, (ACCENT[0], ACCENT[1], ACCENT[2], 0.55))
        logo = self._frame(18, 12, 34, 34, ACCENT)
        for i in (1, 2):
            self._frame(18 + 4, 12 + i * 11, 26, 2, (0.11, 0.23, 0.43, 1))
            self._frame(18 + i * 11, 12 + 4, 2, 26, (0.11, 0.23, 0.43, 1))
        self._frame(18 + 21, 12 + 6, 7, 7, (1, 1, 1, 1))
        del logo
        self.text(None, "SMART WAREHOUSE AI", 64, 11, 18, TEXT, bold=True)
        self.text(None, "3D Robotics Digital Twin  ·  AL-2002 AI Lab  ·  FAST-NUCES", 65, 35, 10, MUTED)
        cols = [("status", "STATUS", 0), ("robots", "ACTIVE ROBOTS", 0), ("brain", "AGENT · SEARCH", 0),
                ("speed", "SPEED", 0), ("step", "STEP", 0), ("cam", "CAMERA", 0)]
        x = W - 20
        widths = {"status": 110, "robots": 100, "brain": 190, "speed": 90, "step": 100, "cam": 150}
        for key, label, _ in reversed(cols):
            self.text(None, label, x, 10, 10, MUTED, align="right", bold=True)
            self.text(f"top:{key}", "", x, 28, 14, TEXT, align="right", bold=True)
            x -= widths[key] + 18

        # ---------------- left panel
        lx, ly, lw = GAP, TOP_H + GAP, LEFT_W
        lh = H - ly - GAP
        self._frame(lx, ly, lw, lh, PANEL)
        x0, w0 = lx + 14, lw - 28
        y = self.section("Simulation Control", x0, ly + 12, w0)
        half = (w0 - 8) / 2
        self.button("start", "START", x0, y, half, 34, app.cmd_start, primary=True)
        self.button("pause", "PAUSE", x0 + half + 8, y, half, 34, app.cmd_pause)
        y += 40
        self.button("resume", "RESUME", x0, y, half, 34, app.cmd_resume)
        self.button("step", "STEP", x0 + half + 8, y, half, 34, app.cmd_step)
        y += 40
        self.button("reset", "RESET SIMULATION", x0, y, w0, 30, app.cmd_reset)
        y += 44
        y = self.section("Configuration", x0, y, w0)
        from config.settings import (AGENT_LABELS, AGENT_TYPES, ALGORITHM_LABELS, ALGORITHMS,
                                     ALLOCATION_LABELS, ALLOCATION_POLICIES, LAYOUT_LABELS)
        self.selector("layout", "Warehouse", [(k, LAYOUT_LABELS[k]) for k in ("small", "medium", "large", "random")],
                      x0, y, w0, lambda v: app.cmd_config(layout=v))
        y += 56
        self.selector("agent", "AI Agent", [(a, AGENT_LABELS[a]) for a in AGENT_TYPES], x0, y, w0,
                      lambda v: app.cmd_config(agent_type=v))
        y += 56
        self.selector("algorithm", "Pathfinding", [(a, ALGORITHM_LABELS[a]) for a in ALGORITHMS], x0, y, w0,
                      lambda v: app.cmd_config(algorithm=v))
        y += 56
        self.selector("robots", "Robots", [("2", "2 robots"), ("3", "3 robots")], x0, y, half,
                      lambda v: app.cmd_config(num_robots=int(v)))
        self.selector("seed", "Seed", [(s, s) for s in ("7", "13", "21", "42", "64", "99", "123", "256", "512",
                                                          "777", "1024", "2026")], x0 + half + 8, y, half,
                      lambda v: app.cmd_config(seed=int(v)))
        y += 56
        self.selector("allocation", "Task Allocation", [(a, ALLOCATION_LABELS[a]) for a in ALLOCATION_POLICIES],
                      x0, y, w0, lambda v: app.cmd_config(allocation=v))
        y += 56
        from config.settings import SPEED_STEPS
        self.selector("speed", "Simulation Speed", [(str(i), f"{s:g} ticks/s") for i, s in enumerate(SPEED_STEPS)],
                      x0, y, w0, lambda v: app.cmd_speed(int(v)))
        y += 56
        self.selector("quality", "Graphics Quality", [("low", "Low"), ("medium", "Medium"), ("high", "High")],
                      x0, y, w0, lambda v: app.cmd_quality(v))
        y += 62
        if y + 120 < ly + lh:
            y = self.section("View", x0, y, w0)
            self.button("paths", "Paths & targets", x0, y, w0, 28, app.cmd_toggle_paths)
            y += 33
            self.button("compare", "BFS vs A* explored cells", x0, y, w0, 28, app.cmd_toggle_compare)
            y += 33
            self.button("hud", "Hide HUD  (H)", x0, y, w0, 28, app.cmd_toggle_hud)
            y += 36
        if y + 40 < ly + lh:
            self.text(None, "Mouse: right-drag orbit · middle-drag pan · wheel zoom\n"
                            "Click a robot to select · Space run/pause · F1 help",
                      x0, ly + lh - 40, 10, MUTED, wrap=w0)

        # ---------------- right panel
        rx = W - RIGHT_W - GAP
        ry = TOP_H + GAP
        rh = H - ry - GAP
        self._frame(rx, ry, RIGHT_W, rh, PANEL)
        x1, w1 = rx + 16, RIGHT_W - 32
        self.text(None, "ROBOT INSPECTOR", x1, ry + 12, 11, ACCENT_HOVER, bold=True)
        for i in range(3):
            self.button(f"robot{i}", f"R{i}", x1 + i * 62, ry + 32, 56, 24, lambda i=i: app.cmd_select(i), size=12)
        self.text("r:agent", "", rx + RIGHT_W - 16, ry + 36, 11, TEXT2, align="right")
        y = ry + 68
        self._frame(x1, y, w1, 40, CARD)
        self.text("r:state", "", x1 + 14, y + 11, 15, TEXT, bold=True)
        self.text("r:action", "", x1 + w1 - 12, y + 13, 11, TEXT2, align="right")
        y += 52
        for key, label in (("pos", "Position"), ("parcel", "Assigned parcel"), ("carry", "Carrying"),
                           ("dest", "Destination"), ("dist", "Travelled distance"), ("deliv", "Completed deliveries"),
                           ("waits", "Waits / conflicts")):
            self.text(None, label, x1, y, 12, MUTED)
            self.text(f"r:{key}", "", x1 + w1, y, 12, TEXT, align="right", bold=True)
            y += 21
        y += 6
        y = self.section("AI Decision", x1, y, w1)
        self.text("r:decision-head", "", x1, y, 12, ACCENT_HOVER, bold=True, wrap=w1)
        self.text("r:decision", "", x1, y + 20, 12, TEXT, wrap=w1)
        y += 112
        self.text("r:cands", "", x1, y, 12, TEXT2, mono=True)
        y += 104
        if y + 90 < ry + rh:
            y = self.section("Pathfinding", x1, y, w1)
            self.text("r:search", "", x1, y, 12, TEXT2, mono=True)

        # ---------------- bottom metrics
        bx = lx + LEFT_W + GAP
        bw = rx - GAP - bx
        by = H - GAP - BOTTOM_H
        self.metric_keys = [("deliv", "DELIVERIES"), ("pending", "PENDING PARCELS"), ("dist", "TOTAL DISTANCE"),
                            ("steps", "SIMULATION STEPS"), ("conf", "CONFLICTS")]
        comp_w = 210 if bw > 900 else 170
        cw = (bw - comp_w - GAP * len(self.metric_keys)) / len(self.metric_keys)
        if cw < 125:
            short = {"deliv": "DELIVERED", "pending": "PENDING", "dist": "DISTANCE", "steps": "STEPS",
                     "conf": "CONFLICTS"}
            self.metric_keys = [(k, short[k]) for k, _ in self.metric_keys]
        for i, (key, label) in enumerate(self.metric_keys):
            cx = bx + i * (cw + GAP)
            self._frame(cx, by, cw, BOTTOM_H, PANEL)
            self.text(None, label, cx + 14, by + 14, 10, MUTED, bold=True)
            self.text(f"m:{key}", "", cx + 14, by + 40, 28, TEXT, bold=True)
        cx = bx + bw - comp_w
        self._frame(cx, by, comp_w, BOTTOM_H, PANEL)
        self.text(None, "COMPLETION", cx + 14, by + 14, 10, MUTED, bold=True)
        self.text("m:pct", "", cx + 14, by + 36, 28, TEXT, bold=True)
        self.text("m:term", "", cx + comp_w - 14, by + 44, 10, TEXT2, align="right")
        self._frame(cx + 14, by + BOTTOM_H - 20, comp_w - 28, 7, CARD)
        self.progress = self._frame(cx + 14, by + BOTTOM_H - 20, 1, 7, GOOD)
        self.progress_w = comp_w - 28

        # ---------------- camera bar (floating, top of viewport; adapts to the width)
        from interface3d.camera import MODE_LABELS, MODES
        short = {"overview": "Overview", "topdown": "Top-Down", "follow": "Tracking", "free": "Free",
                 "focus": "Focus"}
        avail = bw - 12
        cam_w = min(112.0, (avail - 7 * 6) / (len(MODES) + 2 * 1.07))
        extra_w = cam_w * 1.07
        compact = cam_w < 100
        total = len(MODES) * (cam_w + 6) + 2 * (extra_w + 6)
        cx0 = bx + (bw - total) / 2
        cy0 = TOP_H + GAP + 4
        self._frame(cx0 - 6, cy0 - 6, total + 6, 40, (0.05, 0.07, 0.12, 0.72))
        for i, mode in enumerate(MODES):
            self.button(f"cam:{mode}", short[mode] if compact else MODE_LABELS[mode], cx0 + i * (cam_w + 6), cy0,
                        cam_w, 28, lambda m=mode: app.cmd_camera(m), size=11)
        px = cx0 + len(MODES) * (cam_w + 6)
        self.button("present", "PRESENT" if compact else "PRESENTATION", px, cy0, extra_w, 28,
                    app.cmd_presentation, primary=True, size=11)
        self.button("analytics", "ANALYTICS", px + extra_w + 6, cy0, extra_w, 28, app.cmd_analytics, size=11)

        # ---------------- message banner
        self.text("banner", "", bx + bw / 2, by - 30, 13, WARN, align="center", bold=True)
        if not self.visible:
            self.root.hide()

    # ------------------------------------------------------------------ show/hide
    def set_visible(self, visible: bool) -> None:
        self.visible = visible
        if self.root is not None:
            (self.root.show if visible else self.root.hide)()

    def set_progress(self, fraction: float) -> None:
        w = max(1.0, self.progress_w * max(0.0, min(1.0, fraction)))
        sig = round(w)
        if self._cache.get("progress") == sig:
            return
        self._cache["progress"] = sig
        self.progress["frameSize"] = (0, w, -7, 0)


def platform_is_windows() -> bool:
    return sys.platform.startswith("win")
