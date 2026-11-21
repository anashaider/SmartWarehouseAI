"""Command-center HUD for the 3D digital twin (Panda3D DirectGUI, pixel layout).

Rendering efficiency: static panel backgrounds and static labels are generated
as plain geometry and flattened into a few batched geoms per layer; only
interactive widgets (buttons) and live values (OnscreenText) remain individual
nodes. Layers are drawn in a fixed order through the "fixed" cull bin:

    hud (0-3) < presentation (20-23) < analytics (40-43) < help / loading / tooltip (60-63)

Layout (pixels, rebuilt on window resize):
    top bar      branding, status, active robots, agent/algorithm, speed, step, camera
    left panel   run controls, configuration selectors, speed, quality, view toggles
    right panel  ROBOT tab (inspector + AI decision + pathfinding) | FLEET tab (activity,
                 utilisation, throughput, congestion, dock progress)
    bottom       deliveries, pending, distance, steps, conflicts, completion
    camera bar   six camera modes, presentation, analytics
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Optional, Sequence

from direct.gui import DirectGuiGlobals as DGG
from direct.gui.DirectGui import DirectButton, DirectFrame
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import CardMaker, Filename, NodePath, TextNode, TransparencyAttrib

from interface3d.geometry import rgba

# Palette (matches the 2D dashboard)
PANEL = (0.055, 0.08, 0.14, 0.9)
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
STATUS_COLORS = {"READY": INFO, "RUNNING": GOOD, "PAUSED": WARN, "FINISHED": rgba("#a78bfa"),
                 "COMPLETED": rgba("#a78bfa"), "TERMINATED": BAD}
ACTIVITY_COLORS = {"moving": rgba("#3b82f6"), "handling": rgba("#22c55e"), "yielding": rgba("#22d3ee"),
                   "waiting": rgba("#f59e0b"), "idle": rgba("#64748b")}

# Type scale
T_SMALL, T_BODY, T_LABEL, T_TITLE, T_METRIC = 10, 12, 13, 18, 28

TOP_H = 58
LEFT_W = 258
RIGHT_W = 338
BOTTOM_H = 92
GAP = 10
LAYER_BASE = {"hud": 0, "presentation": 20, "analytics": 40, "help": 60, "loading": 60, "tooltip": 64}


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
        self.dyn_frames: dict[str, DirectFrame] = {}
        self.blocking_rects: list[tuple[float, float, float, float]] = []
        self.size = (0, 0)
        self.visible = True
        self.right_tab = "robot"
        self.tooltip_root: Optional[NodePath] = None
        self.tooltip: Optional[OnscreenText] = None

    # ------------------------------------------------------------------ layers
    def _base_sort(self) -> int:
        node = self.root
        while node is not None and not node.isEmpty():
            if node.getName() in LAYER_BASE:
                return LAYER_BASE[node.getName()]
            node = node.getParent() if node.hasParent() else None
        return 0

    def _static(self, kind: str) -> NodePath:
        name = f"static-{kind}"
        found = self.root.find(name)
        if found.isEmpty():
            found = self.root.attachNewNode(name)
            found.setBin("fixed", self._base_sort() + (0 if kind == "cards" else 2))
            found.setTransparency(TransparencyAttrib.MAlpha)
            found.setDepthTest(False)
            found.setDepthWrite(False)
        return found

    def finalize(self, root: NodePath) -> None:
        """Merge the static geometry of a layer into a few batched geoms."""
        for kind in ("cards", "text"):
            np_ = root.find(f"static-{kind}")
            if not np_.isEmpty():
                np_.flattenStrong()

    # ------------------------------------------------------------------ primitives
    def _frame(self, x: float, y: float, w: float, h: float, color=PANEL, parent=None,
               dynamic: bool = False, key: Optional[str] = None):
        """Background rectangle. Static ones become batched geometry; ``dynamic``
        ones are DirectFrames whose size/colour can change (progress bars)."""
        if dynamic:
            f = DirectFrame(parent=parent or self.root, frameColor=color, frameSize=(0, w, -h, 0), pos=(x, 0, -y),
                            state=DGG.DISABLED)
            f.setTransparency(TransparencyAttrib.MAlpha)
            f.setBin("fixed", self._base_sort() + 1)
            if key:
                self.dyn_frames[key] = f
            return f
        cm = CardMaker("card")
        cm.setFrame(0, w, -h, 0)
        card = (parent or self._static("cards")).attachNewNode(cm.generate())
        card.setPos(x, 0, -y)
        card.setColor(*color)
        return card

    def text(self, key: Optional[str], value: str, x: float, y: float, size: float = T_LABEL, color=TEXT,
             align: str = "left", bold: bool = False, wrap: Optional[float] = None, mono: bool = False,
             parent=None):
        al = {"left": TextNode.ALeft, "center": TextNode.ACenter, "right": TextNode.ARight}[align]
        font = self.mono if mono else (self.bold if bold else self.font)
        if key is None:
            # static label -> generated geometry (batched at finalize)
            tn = TextNode("label")
            if font is not None:
                tn.setFont(font)
            tn.setText(value)
            tn.setAlign(al)
            tn.setTextColor(*color)
            if wrap:
                tn.setWordwrap(wrap / size)
            np_ = (parent or self._static("text")).attachNewNode(tn.generate())
            np_.setScale(size)
            np_.setPos(x, 0, -y - size * 0.8)
            return np_
        t = OnscreenText(text=value, parent=parent or self.root, pos=(x, -y - size * 0.8), scale=size,
                         fg=color, align=al, font=font, mayChange=True,
                         wordwrap=(wrap / size) if wrap else None)
        t.setBin("fixed", self._base_sort() + 3)
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
               cmd: Callable[[], None], primary: bool = False, size: float = T_LABEL, parent=None,
               tooltip: str = "") -> DirectButton:
        base = ACCENT if primary else CARD
        hover = ACCENT_HOVER if primary else CARD_HOVER
        b = DirectButton(parent=parent or self.root, text=label, text_font=self.bold, text_scale=size,
                         text_fg=TEXT if not primary else (1, 1, 1, 1),
                         text_pos=(w / 2, -h / 2 - size * 0.35), frameSize=(0, w, -h, 0), pos=(x, 0, -y),
                         frameColor=(base, base, hover, (0.08, 0.1, 0.16, 0.9)), relief=DGG.FLAT,
                         command=cmd, pressEffect=1, rolloverSound=None, clickSound=None)
        b.setTransparency(TransparencyAttrib.MAlpha)
        b.setBin("fixed", self._base_sort() + 1)
        b._base_colors = (base, hover)  # type: ignore[attr-defined]
        if tooltip:
            b.bind(DGG.ENTER, lambda _e, t=tooltip, bx=x, by=y, bh=h: self.show_tooltip(t, bx, by + bh + 6))
            b.bind(DGG.EXIT, lambda _e: self.hide_tooltip())
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
                if enabled is not False:
                    b["text_fg"] = TEXT

    def selector(self, key: str, label: str, options: Sequence[tuple[str, str]], x: float, y: float, w: float,
                 on_change: Callable[[str], None]) -> None:
        self.text(None, label.upper(), x + 2, y, T_SMALL, MUTED, bold=True)
        by = y + 16
        h = 30
        self._frame(x, by, w, h, CARD)
        state = {"options": list(options), "on_change": on_change, "value": options[0][0]}

        def shift(d: int) -> None:
            opts = state["options"]
            idx = next((i for i, (v, _) in enumerate(opts) if v == state["value"]), 0)
            state["value"] = opts[(idx + d) % len(opts)][0]
            on_change(state["value"])

        self.button(f"{key}-prev", "‹", x, by, 30, h, lambda: shift(-1), size=20)
        self.button(f"{key}-next", "›", x + w - 30, by, 30, h, lambda: shift(1), size=20)
        self.text(f"sel:{key}", "", x + w / 2, by + 8, T_LABEL, TEXT, align="center", bold=True)
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

    # ------------------------------------------------------------------ tooltip / loading
    def show_tooltip(self, text: str, x: float, y: float) -> None:
        if self.tooltip_root is None or self.tooltip is None:
            return
        self.tooltip_root.show()
        w = max(60, len(text) * 6.6 + 16)
        x = min(x, self.size[0] - w - 6)
        self.tooltip_bg["frameSize"] = (0, w, -22, 0)
        self.tooltip_bg.setPos(x, 0, -y)
        self.tooltip.setText(text)
        self.tooltip.setPos(x + 8, -y - 15)

    def hide_tooltip(self) -> None:
        if self.tooltip_root is not None:
            self.tooltip_root.hide()

    def _build_tooltip_layer(self) -> None:
        if self.tooltip_root is not None:
            self.tooltip_root.removeNode()
        root = self.base.pixel2d.attachNewNode("tooltip")
        saved, self.root = self.root, root
        self.tooltip_bg = self._frame(0, 0, 100, 22, (0.02, 0.03, 0.06, 0.95), dynamic=True)
        self.tooltip = self.text("tooltip", "", 0, 0, 11, TEXT)
        self.root = saved
        self.tooltip_root = root
        root.hide()

    def show_loading(self, message: str) -> NodePath:
        root = self.base.pixel2d.attachNewNode("loading")
        saved, self.root = self.root, root
        w, h = self.size
        self._frame(0, 0, w, h, (0.02, 0.03, 0.05, 0.55))
        self._frame(w / 2 - 190, h / 2 - 44, 380, 88, PANEL_SOLID)
        self._frame(w / 2 - 190, h / 2 - 44, 4, 88, ACCENT)
        self.text(None, message, w / 2, h / 2 - 22, 16, TEXT, align="center", bold=True)
        self.text(None, "building 3D scene from the scenario...", w / 2, h / 2 + 6, 11, TEXT2, align="center")
        self.finalize(root)
        self.root = saved
        return root

    # ------------------------------------------------------------------ helpers for the app
    def blocks(self, px: float, py: float) -> bool:
        """True when a pixel lies on a HUD panel (clicks there must not pick robots)."""
        if not self.visible:
            return False
        return any(x <= px <= x + w and y <= py <= y + h for x, y, w, h in self.blocking_rects)

    def set_progress(self, fraction: float, key: str = "progress", width: Optional[float] = None,
                     height: float = 7) -> None:
        f = self.dyn_frames.get(key)
        if f is None:
            return
        full = width if width is not None else self.progress_w
        w = max(1.0, full * max(0.0, min(1.0, fraction)))
        sig = round(w)
        if self._cache.get("bar:" + key) == sig:
            return
        self._cache["bar:" + key] = sig
        f["frameSize"] = (0, w, -height, 0)

    def set_right_tab(self, tab: str) -> None:
        self.right_tab = tab
        if self.root is None:
            return
        for name in ("robot", "fleet"):
            node = self.root.find(f"**/tab-{name}")
            if not node.isEmpty():
                (node.show if name == tab else node.hide)()
        self.set_button("tab:robot", active=tab == "robot")
        self.set_button("tab:fleet", active=tab == "fleet")

    def set_visible(self, visible: bool) -> None:
        self.visible = visible
        if self.root is not None:
            (self.root.show if visible else self.root.hide)()
        if not visible:
            self.hide_tooltip()

    # ------------------------------------------------------------------ build
    def build(self, width: int, height: int) -> None:
        if self.root is not None:
            self.root.removeNode()
        self.texts.clear()
        self._cache.clear()
        self.buttons.clear()
        self.selectors.clear()
        self.dyn_frames.clear()
        self.blocking_rects = []
        self.size = (width, height)
        self.root = self.base.pixel2d.attachNewNode("hud")
        hud_root = self.root
        app = self.app
        W, H = width, height
        from config.settings import (AGENT_LABELS, AGENT_TYPES, ALGORITHM_LABELS, ALGORITHMS,
                                     ALLOCATION_LABELS, ALLOCATION_POLICIES, LAYOUT_LABELS, SPEED_STEPS)
        from interface3d.camera import MODE_KEYS, MODE_LABELS, MODES

        # ---------------- top bar
        self._frame(0, 0, W, TOP_H, PANEL_SOLID)
        self._frame(0, TOP_H - 2, W, 2, (ACCENT[0], ACCENT[1], ACCENT[2], 0.55))
        self.blocking_rects.append((0, 0, W, TOP_H))
        self._frame(18, 12, 34, 34, ACCENT)
        for i in (1, 2):
            self._frame(18 + 4, 12 + i * 11, 26, 2, (0.11, 0.23, 0.43, 1))
            self._frame(18 + i * 11, 12 + 4, 2, 26, (0.11, 0.23, 0.43, 1))
        self._frame(18 + 21, 12 + 6, 7, 7, (1, 1, 1, 1))
        self.text(None, "SMART WAREHOUSE AI", 64, 11, T_TITLE, TEXT, bold=True)
        self.text(None, "3D Robotics Digital Twin  ·  AL-2002 AI Lab  ·  FAST-NUCES", 65, 35, T_SMALL, MUTED)
        widths = {"status": 100, "robots": 92, "brain": 180, "speed": 84, "step": 92, "cam": 132}
        cols = [("status", "STATUS"), ("robots", "ACTIVE ROBOTS"), ("brain", "AGENT · SEARCH"),
                ("speed", "SPEED"), ("step", "STEP"), ("cam", "CAMERA")]
        x = W - 20
        for key, label in reversed(cols):
            if x - widths[key] < 430:
                break
            self.text(None, label, x, 10, T_SMALL, MUTED, align="right", bold=True)
            self.text(f"top:{key}", "", x, 28, 14, TEXT, align="right", bold=True)
            x -= widths[key] + 18

        # ---------------- left panel
        lx, ly, lw = GAP, TOP_H + GAP, LEFT_W
        lh = H - ly - GAP
        self._frame(lx, ly, lw, lh, PANEL)
        self.blocking_rects.append((lx, ly, lw, lh))
        x0, w0 = lx + 14, lw - 28
        half = (w0 - 8) / 2
        y = self.section("Simulation Control", x0, ly + 12, w0)
        self.button("start", "START", x0, y, half, 32, app.cmd_start, primary=True, tooltip="Start  (Space)")
        self.button("pause", "PAUSE", x0 + half + 8, y, half, 32, app.cmd_pause, tooltip="Pause  (Space)")
        y += 38
        self.button("resume", "RESUME", x0, y, half, 32, app.cmd_resume, tooltip="Resume  (Space)")
        self.button("step", "STEP", x0 + half + 8, y, half, 32, app.cmd_step, tooltip="Advance one tick  (S)")
        y += 38
        self.button("reset", "RESET SIMULATION", x0, y, w0, 28, app.cmd_reset, tooltip="Reset scenario  (R)")
        y += 40
        y = self.section("Configuration", x0, y, w0)
        step_y = 52 if H >= 820 else 49
        self.selector("layout", "Warehouse", [(k, LAYOUT_LABELS[k]) for k in ("small", "medium", "large", "random")],
                      x0, y, w0, lambda v: app.cmd_config(layout=v))
        y += step_y
        self.selector("agent", "AI Agent", [(a, AGENT_LABELS[a]) for a in AGENT_TYPES], x0, y, w0,
                      lambda v: app.cmd_config(agent_type=v))
        y += step_y
        self.selector("algorithm", "Pathfinding", [(a, ALGORITHM_LABELS[a]) for a in ALGORITHMS], x0, y, w0,
                      lambda v: app.cmd_config(algorithm=v))
        y += step_y
        self.selector("robots", "Robots", [("2", "2 robots"), ("3", "3 robots")], x0, y, half,
                      lambda v: app.cmd_config(num_robots=int(v)))
        self.selector("seed", "Seed", [(s, s) for s in ("7", "13", "21", "42", "64", "99", "123", "256", "512",
                                                          "777", "1024", "2026")], x0 + half + 8, y, half,
                      lambda v: app.cmd_config(seed=int(v)))
        y += step_y
        self.selector("allocation", "Task Allocation", [(a, ALLOCATION_LABELS[a]) for a in ALLOCATION_POLICIES],
                      x0, y, w0, lambda v: app.cmd_config(allocation=v))
        y += step_y
        self.selector("speed", "Speed", [(str(i), f"{s:g} t/s") for i, s in enumerate(SPEED_STEPS)],
                      x0, y, half, lambda v: app.cmd_speed(int(v)))
        self.selector("quality", "Graphics", [("low", "Low"), ("medium", "Medium"), ("high", "High")],
                      x0 + half + 8, y, half, lambda v: app.cmd_quality(v))
        y += step_y + 8
        if y + 110 < ly + lh:
            y = self.section("View", x0, y, w0)
            self.button("paths", "Paths", x0, y, half, 26, app.cmd_toggle_paths, size=12, tooltip="Routes & targets")
            self.button("compare", "BFS vs A*", x0 + half + 8, y, half, 26, app.cmd_toggle_compare, size=12,
                        tooltip="Explored cells  (C)")
            y += 31
            self.button("heatmap", "Heatmap", x0, y, half, 26, app.cmd_toggle_heatmap, size=12,
                        tooltip="Congestion heatmap  (M)")
            self.button("indicators", "Indicators", x0 + half + 8, y, half, 26, app.cmd_toggle_indicators, size=12,
                        tooltip="Robot status tags  (I)")
            y += 31
            self.button("hud", "Hide HUD", x0, y, w0, 26, app.cmd_toggle_hud, size=12, tooltip="Hide HUD  (H)")
            y += 34
        if y + 34 < ly + lh:
            self.text(None, "Right-drag orbit · middle-drag pan · wheel zoom\nClick a robot to select · F1 all shortcuts",
                      x0, ly + lh - 34, T_SMALL, MUTED, wrap=w0)

        # ---------------- right panel (tabs)
        rx = W - RIGHT_W - GAP
        ry = TOP_H + GAP
        rh = H - ry - GAP
        self._frame(rx, ry, RIGHT_W, rh, PANEL)
        self.blocking_rects.append((rx, ry, RIGHT_W, rh))
        x1, w1 = rx + 16, RIGHT_W - 32
        tab_w = (w1 - 6) / 2
        self.button("tab:robot", "ROBOT", x1, ry + 12, tab_w, 26, lambda: self.set_right_tab("robot"), size=12)
        self.button("tab:fleet", "FLEET", x1 + tab_w + 6, ry + 12, tab_w, 26, lambda: self.set_right_tab("fleet"),
                    size=12)
        # ROBOT tab
        self.root = hud_root.attachNewNode("tab-robot")
        for i in range(3):
            self.button(f"robot{i}", f"R{i}", x1 + i * 62, ry + 50, 56, 24, lambda i=i: app.cmd_select(i), size=12,
                        tooltip=f"Select robot R{i}  ({i + 1})")
        self.text("r:agent", "", rx + RIGHT_W - 16, ry + 54, 11, TEXT2, align="right")
        y = ry + 84
        self._frame(x1, y, w1, 40, CARD)
        self.text("r:state", "", x1 + 14, y + 11, 15, TEXT, bold=True)
        self.text("r:action", "", x1 + w1 - 12, y + 13, 11, TEXT2, align="right")
        y += 50
        for key, label in (("pos", "Position"), ("parcel", "Assigned parcel"), ("carry", "Carrying"),
                           ("dest", "Destination"), ("dist", "Travelled distance"), ("deliv", "Completed deliveries"),
                           ("util", "Utilization"), ("waits", "Waits / conflicts")):
            self.text(None, label, x1, y, T_BODY, MUTED)
            self.text(f"r:{key}", "", x1 + w1, y, T_BODY, TEXT, align="right", bold=True)
            y += 20
        y += 6
        y = self.section("AI Decision", x1, y, w1)
        self.text("r:decision-head", "", x1, y, T_BODY, ACCENT_HOVER, bold=True, wrap=w1)
        self.text("r:decision", "", x1, y + 20, T_BODY, TEXT, wrap=w1)
        y += 98
        self.text("r:cands", "", x1, y, T_BODY, TEXT2, mono=True)
        y += 96
        if y + 80 < ry + rh:
            y = self.section("Pathfinding", x1, y, w1)
            self.text("r:search", "", x1, y, T_BODY, TEXT2, mono=True)
        self.finalize(self.root)
        # FLEET tab
        self.root = hud_root.attachNewNode("tab-fleet")
        y = ry + 50
        y = self.section("Fleet Activity", x1, y, w1)
        acts = ("moving", "handling", "yielding", "waiting", "idle")
        cw = (w1 - 4 * 6) / 5
        for i, a in enumerate(acts):
            cx = x1 + i * (cw + 6)
            self._frame(cx, y, cw, 50, CARD)
            self._frame(cx, y, cw, 3, ACTIVITY_COLORS[a])
            self.text(f"f:act:{a}", "0", cx + cw / 2, y + 9, 18, TEXT, align="center", bold=True)
            self.text(None, a.upper(), cx + cw / 2, y + 34, 8, MUTED, align="center", bold=True)
        y += 62
        y = self.section("Robot Utilization", x1, y, w1)
        for i in range(3):
            self.text(f"f:rname{i}", "", x1, y, T_BODY, TEXT, bold=True)
            self.text(f"f:rstate{i}", "", x1 + 44, y, 11, TEXT2)
            self.text(f"f:rpct{i}", "", x1 + w1, y, T_BODY, TEXT, align="right", bold=True)
            self._frame(x1, y + 20, w1, 6, CARD)
            self._frame(x1, y + 20, 1, 6, ACCENT, dynamic=True, key=f"util{i}")
            y += 36
        self.text(None, "Utilization = share of ticks spent moving, handling a parcel or yielding.",
                  x1, y, T_SMALL, MUTED, wrap=w1)
        y += 30
        y = self.section("Throughput & Congestion", x1, y, w1)
        for key, label in (("fu", "Fleet utilization"), ("tp", "Deliveries / 100 ticks"),
                           ("ct", "Mean pickup-to-delivery"), ("hot", "Congestion hotspot"),
                           ("confl", "Conflict cells")):
            self.text(None, label, x1, y, T_BODY, MUTED)
            self.text(f"f:{key}", "", x1 + w1, y, T_BODY, TEXT, align="right", bold=True)
            y += 20
        y += 6
        if y + 60 < ry + rh:
            y = self.section("Dock Progress", x1, y, w1)
            for i in range(4):
                if y + 30 > ry + rh:
                    break
                self.text(f"f:dock{i}", "", x1, y, T_BODY, TEXT)
                self.text(f"f:dockn{i}", "", x1 + w1, y, T_BODY, TEXT, align="right", bold=True)
                self._frame(x1, y + 19, w1, 5, CARD)
                self._frame(x1, y + 19, 1, 5, GOOD, dynamic=True, key=f"dock{i}")
                y += 32
        self.finalize(self.root)
        self.root = hud_root

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
            self.text(None, label, cx + 14, by + 14, T_SMALL, MUTED, bold=True)
            self.text(f"m:{key}", "", cx + 14, by + 40, T_METRIC, TEXT, bold=True)
        self.blocking_rects.append((bx, by, bw, BOTTOM_H))
        cx = bx + bw - comp_w
        self._frame(cx, by, comp_w, BOTTOM_H, PANEL)
        self.text(None, "COMPLETION", cx + 14, by + 14, T_SMALL, MUTED, bold=True)
        self.text("m:pct", "", cx + 14, by + 36, T_METRIC, TEXT, bold=True)
        self.text("m:term", "", cx + comp_w - 14, by + 44, T_SMALL, TEXT2, align="right")
        self._frame(cx + 14, by + BOTTOM_H - 20, comp_w - 28, 7, CARD)
        self._frame(cx + 14, by + BOTTOM_H - 20, 1, 7, GOOD, dynamic=True, key="progress")
        self.progress_w = comp_w - 28

        # ---------------- camera bar
        short = {"overview": "Overview", "topdown": "Top-Down", "tracking": "Tracking", "follow": "3rd-Person",
                 "free": "Free", "focus": "Focus"}
        avail = bw - 12
        cam_w = min(108.0, (avail - (len(MODES) + 1) * 6) / (len(MODES) + 2 * 1.05))
        extra_w = cam_w * 1.05
        compact = cam_w < 104
        total = len(MODES) * (cam_w + 6) + 2 * (extra_w + 6)
        cx0 = bx + (bw - total) / 2
        cy0 = TOP_H + GAP + 4
        self._frame(cx0 - 6, cy0 - 6, total + 6, 40, (0.05, 0.07, 0.12, 0.78))
        self.blocking_rects.append((cx0 - 6, cy0 - 6, total + 6, 40))
        for i, mode in enumerate(MODES):
            self.button(f"cam:{mode}", short[mode] if compact else MODE_LABELS[mode], cx0 + i * (cam_w + 6), cy0,
                        cam_w, 28, lambda m=mode: app.cmd_camera(m), size=11,
                        tooltip=f"{MODE_LABELS[mode]}  ({MODE_KEYS[mode]})")
        px = cx0 + len(MODES) * (cam_w + 6)
        self.button("present", "PRESENT" if compact else "PRESENTATION", px, cy0, extra_w, 28,
                    app.cmd_presentation, primary=True, size=11, tooltip="Presentation Mode  (P)")
        self.button("analytics", "ANALYTICS", px + extra_w + 6, cy0, extra_w, 28, app.cmd_analytics, size=11,
                    tooltip="Experiment analytics  (A)")

        # ---------------- message banner + legends
        self.banner_bg = self._frame(bx + bw / 2 - 260, by - 40, 520, 28, (0.25, 0.16, 0.02, 0.85), dynamic=True)
        self.banner_bg.hide()
        self.text("banner", "", bx + bw / 2, by - 33, T_BODY, (1.0, 0.85, 0.5, 1), align="center", bold=True)
        self.text("legend", "", bx + 12, by - 22, 11, TEXT2)
        self.finalize(self.root)
        self._build_tooltip_layer()
        self.set_right_tab(self.right_tab)
        if not self.visible:
            self.root.hide()

    def set_banner(self, message: str) -> None:
        if self._cache.get("banner-msg") == message:
            return
        self._cache["banner-msg"] = message
        self.set_text("banner", message)
        if message:
            self.banner_bg.show()
        else:
            self.banner_bg.hide()
