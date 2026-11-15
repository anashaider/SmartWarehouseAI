"""Dynamic overlays: navigation paths, destinations, BFS/A* explored cells, event effects.

All overlays are rebuilt only when the underlying simulation data changes
(planned path, target, comparison query). Event effects are created from real
``TickEvent`` objects via :class:`interface3d.sync.VisualSync`.
"""

from __future__ import annotations

import math
from typing import Optional

from panda3d.core import (
    NodePath,
    TextNode,
    TextureStage,
    TransparencyAttrib,
)

from algorithms.common import SearchResult
from environment.grid import Position
from interface3d import textures
from interface3d.geometry import MeshBuilder, rgba
from interface3d.sync import VisualEffect, world_xy

BFS_RGBA = rgba("#3987e5")
ASTAR_RGBA = rgba("#d95926")
EFFECT_COLORS = {
    "pickup": rgba("#f59e0b"),
    "deliver": rgba("#22c55e"),
    "conflict": rgba("#ef4444"),
    "invalid": rgba("#ef4444"),
    "yield": rgba("#22d3ee"),
    "assign": rgba("#93c5fd"),
    "wait": rgba("#f59e0b"),
}


def _overlay(np_: NodePath, bin_order: int = 10) -> NodePath:
    np_.setTransparency(TransparencyAttrib.MAlpha)
    np_.setLightOff(1)
    np_.setShaderOff(1)
    np_.setDepthWrite(False)
    np_.setBin("fixed", bin_order)
    return np_


def path_ribbon(path: list[Position], height: int, color, width: float = 0.16, z: float = 0.02) -> Optional[NodePath]:
    """Flat ribbon through cell centres, UV.u runs along the path (for scrolling dashes)."""
    if len(path) < 2:
        return None
    mb = MeshBuilder("path")
    pts = [world_xy(p, height) for p in path]
    u = 0.0
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        dx, dy = bx - ax, by - ay
        L = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / L * width / 2, dx / L * width / 2
        # extend slightly so corners overlap
        ex, ey = dx / L * width / 2, dy / L * width / 2
        p0 = (ax - nx - ex * 0, ay - ny - ey * 0, z)
        p1 = (bx - nx + ex, by - ny + ey, z)
        p2 = (bx + nx + ex, by + ny + ey, z)
        p3 = (ax + nx, ay + ny, z)
        mb.quad(p0, p1, p2, p3, (0, 0, 1), color, ((u, 0), (u + L * 2.5, 0), (u + L * 2.5, 1), (u, 1)))
        u += L * 2.5
    return _overlay(NodePath(mb.node()), 12)


class EffectsLayer:
    def __init__(self, parent: NodePath, grid_height: int) -> None:
        self.root = parent.attachNewNode("effects")
        self.height = grid_height
        self.dash = textures.dash_texture()
        self.glow = textures.soft_glow()
        self.paths: dict[int, tuple[tuple, NodePath]] = {}
        self.markers: dict[int, tuple[Optional[Position], NodePath]] = {}
        self.explored_root = self.root.attachNewNode("explored")
        self.explored_key: Optional[tuple] = None
        self.explored_tiles: list[tuple[NodePath, int, int]] = []   # node, order index, algorithm(0 bfs,1 a*)
        self.explored_started = 0.0
        self.compare_paths = self.root.attachNewNode("compare-paths")
        self.event_nodes: dict[int, NodePath] = {}

    def destroy(self) -> None:
        self.root.removeNode()

    # ------------------------------------------------------------------ paths
    def update_path(self, robot_id: int, path: list[Position], color, visible: bool, now: float) -> None:
        key = tuple(path)
        cur = self.paths.get(robot_id)
        if cur is None or cur[0] != key:
            if cur is not None:
                cur[1].removeNode()
            node = path_ribbon(path, self.height, (color[0], color[1], color[2], 0.85))
            if node is None:
                node = NodePath("empty-path")
            node.reparentTo(self.root)
            node.setTexture(self.dash)
            self.paths[robot_id] = (key, node)
            cur = self.paths[robot_id]
        node = cur[1]
        if visible:
            node.show()
            node.setTexOffset(TextureStage.getDefault(), -now * 1.2, 0)
        else:
            node.hide()

    def update_marker(self, robot_id: int, target: Optional[Position], color, visible: bool, now: float) -> None:
        cur = self.markers.get(robot_id)
        if cur is None:
            node = self.root.attachNewNode(f"marker-{robot_id}")
            ring = MeshBuilder("marker-ring")
            ring.ring(0, 0, 0, 0.34, 0.40, (color[0], color[1], color[2], 0.95))
            ring.ring(0, 0, 0, 0.18, 0.21, (color[0], color[1], color[2], 0.7))
            _overlay(node.attachNewNode(ring.node()), 13)
            beam = MeshBuilder("beam")
            beam.cylinder((0, 0, 0), 0.05, 2.2, (color[0], color[1], color[2], 0.28), segments=12, caps=False)
            b = _overlay(node.attachNewNode(beam.node()), 14)
            b.setTwoSided(True)
            cur = (None, node)
            self.markers[robot_id] = cur
        node = cur[1]
        if target is None or not visible:
            node.hide()
            return
        x, y = world_xy(target, self.height)
        node.show()
        node.setPos(x, y, 0.025)
        pulse = 0.5 + 0.5 * math.sin(now * 3.0 + robot_id)
        node.setScale(0.92 + 0.12 * pulse, 0.92 + 0.12 * pulse, 1)
        node.getChild(0).setH(now * 40)

    # ------------------------------------------------------------------ BFS / A*
    def set_comparison(self, key: Optional[tuple], results: Optional[dict[str, SearchResult]], now: float) -> None:
        """Explored-cell tiles in real expansion order (revealed progressively)."""
        if key == self.explored_key:
            return
        self.explored_key = key
        self.explored_root.getChildren().detach()
        self.compare_paths.getChildren().detach()
        self.explored_tiles = []
        self.explored_started = now
        if not results:
            return
        for algo_idx, (name, color, size, z) in enumerate((("bfs", BFS_RGBA, 0.9, 0.012), ("astar", ASTAR_RGBA, 0.5, 0.03))):
            res = results.get(name)
            if res is None:
                continue
            for i, cell in enumerate(res.explored):
                mb = MeshBuilder("tile")
                x, y = world_xy(cell, self.height)
                mb.flat_rect(x - size / 2, y - size / 2, x + size / 2, y + size / 2, z,
                             (color[0], color[1], color[2], 0.32 if name == "bfs" else 0.75))
                t = _overlay(self.explored_root.attachNewNode(mb.node()), 11)
                t.hide()
                self.explored_tiles.append((t, i, algo_idx))
            ribbon = path_ribbon(res.path, self.height, (color[0], color[1], color[2], 0.95), width=0.1,
                                 z=0.04 + 0.01 * algo_idx)
            if ribbon is not None:
                ribbon.reparentTo(self.compare_paths)
                off = 0.09 if name == "astar" else -0.09
                ribbon.setPos(off, off, 0)
        self.explored_root.flattenLight()

    def animate_comparison(self, visible: bool, now: float, reveal_seconds: float = 2.0) -> None:
        if not visible:
            self.explored_root.hide()
            self.compare_paths.hide()
            return
        self.explored_root.show()
        self.compare_paths.show()
        counts = [0, 0]
        for _, i, a in self.explored_tiles:
            counts[a] = max(counts[a], i + 1)
        longest = max(1, max(counts))
        shown = int(longest * min(1.0, (now - self.explored_started) / reveal_seconds)) + 1
        for node, i, _a in self.explored_tiles:
            if i < shown:
                node.show()
            else:
                node.hide()

    # ------------------------------------------------------------------ events
    def update_events(self, effects: list[VisualEffect], now: float) -> None:
        alive = set()
        for fx in effects:
            key = id(fx)
            alive.add(key)
            node = self.event_nodes.get(key)
            if node is None:
                node = self._make_event_node(fx)
                self.event_nodes[key] = node
            t = max(0.0, min(1.0, fx.progress(now)))
            fade = 1.0 - t
            node.setAlphaScale(fade)
            ring = node.find("**/fx-ring")
            if not ring.isEmpty():
                s = 0.6 + 1.4 * t
                ring.setScale(s, s, 1)
            label = node.find("**/fx-label")
            if not label.isEmpty():
                label.setZ(1.0 + 0.8 * t)
            column = node.find("**/fx-column")
            if not column.isEmpty():
                column.setScale(1, 1, 0.3 + 1.2 * math.sin(math.pi * min(1.0, t * 1.2)))
        for key in list(self.event_nodes):
            if key not in alive:
                self.event_nodes.pop(key).removeNode()

    def _make_event_node(self, fx: VisualEffect) -> NodePath:
        color = EFFECT_COLORS.get(fx.kind, (1, 1, 1, 1))
        node = self.root.attachNewNode(f"fx-{fx.kind}")
        node.setPos(fx.x, fx.y, 0.03)
        _overlay(node, 20)
        ring = MeshBuilder("fx-ring")
        ring.ring(0, 0, 0, 0.36, 0.46, (color[0], color[1], color[2], 0.95))
        node.attachNewNode(ring.node()).setName("fx-ring")
        if fx.kind in ("pickup", "deliver", "yield"):
            col = MeshBuilder("fx-column")
            col.cylinder((0, 0, 0), 0.32, 1.6, (color[0], color[1], color[2], 0.22), segments=20, caps=False)
            c = node.attachNewNode(col.node())
            c.setName("fx-column")
            c.setTwoSided(True)
        text = {"pickup": "PICKED UP", "deliver": "DELIVERED", "conflict": "! CONFLICT",
                "invalid": "! INVALID", "yield": "YIELDING", "assign": "TASK", "wait": "WAIT"}.get(fx.kind, "")
        if fx.kind in ("pickup", "deliver", "conflict", "yield", "invalid"):
            tn = TextNode("fx-label")
            tn.setText(text if fx.kind != "deliver" else "DELIVERED")
            tn.setAlign(TextNode.ACenter)
            tn.setTextColor(color[0], color[1], color[2], 1)
            tn.setShadow(0.06, 0.06)
            tn.setShadowColor(0, 0, 0, 0.8)
            lab = node.attachNewNode(tn)
            lab.setName("fx-label")
            lab.setScale(0.26)
            lab.setBillboardPointEye()
            lab.setZ(1.0)
            lab.setBin("fixed", 40)
            lab.setDepthTest(False)
        return node
