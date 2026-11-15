"""Procedural mesh building (no external model files are needed).

``MeshBuilder`` accumulates triangles with per-vertex position, normal, colour
and UV, then produces a single ``GeomNode``. Static scenery is built with a
few large builders so the scene has a small number of draw calls.
"""

from __future__ import annotations

import math
from typing import Sequence

from panda3d.core import (
    Geom,
    GeomNode,
    GeomTriangles,
    GeomVertexData,
    GeomVertexFormat,
    GeomVertexWriter,
)

Vec = tuple[float, float, float]
RGBA = tuple[float, float, float, float]


def rgba(hex_str: str, alpha: float = 1.0) -> RGBA:
    h = hex_str.lstrip("#")
    return (int(h[0:2], 16) / 255, int(h[2:4], 16) / 255, int(h[4:6], 16) / 255, alpha)


def shade(c: RGBA, f: float) -> RGBA:
    return (min(1.0, c[0] * f), min(1.0, c[1] * f), min(1.0, c[2] * f), c[3])


def chamfered_rect(w: float, h: float, c: float, cx: float = 0.0, cy: float = 0.0) -> list[tuple[float, float]]:
    """Octagon: a w x h rectangle with corners cut by ``c`` (counter-clockwise)."""
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    return [(x0 + c, y0), (x1 - c, y0), (x1, y0 + c), (x1, y1 - c), (x1 - c, y1), (x0 + c, y1), (x0, y1 - c),
            (x0, y0 + c)]


class MeshBuilder:
    def __init__(self, name: str = "mesh") -> None:
        self.name = name
        fmt = GeomVertexFormat.getV3n3c4t2()
        self.vdata = GeomVertexData(name, fmt, Geom.UHStatic)
        self.vw = GeomVertexWriter(self.vdata, "vertex")
        self.nw = GeomVertexWriter(self.vdata, "normal")
        self.cw = GeomVertexWriter(self.vdata, "color")
        self.tw = GeomVertexWriter(self.vdata, "texcoord")
        self.tris = GeomTriangles(Geom.UHStatic)
        self.count = 0

    @property
    def empty(self) -> bool:
        return self.count == 0

    def _vertex(self, p: Vec, n: Vec, c: RGBA, uv: tuple[float, float]) -> int:
        self.vw.addData3(*p)
        self.nw.addData3(*n)
        self.cw.addData4(*c)
        self.tw.addData2(*uv)
        self.count += 1
        return self.count - 1

    def quad(self, p0: Vec, p1: Vec, p2: Vec, p3: Vec, n: Vec, c: RGBA,
             uvs: Sequence[tuple[float, float]] = ((0, 0), (1, 0), (1, 1), (0, 1))) -> None:
        """Quad with counter-clockwise winding p0 -> p1 -> p2 -> p3 (seen from the normal side)."""
        i0 = self._vertex(p0, n, c, uvs[0])
        i1 = self._vertex(p1, n, c, uvs[1])
        i2 = self._vertex(p2, n, c, uvs[2])
        i3 = self._vertex(p3, n, c, uvs[3])
        self.tris.addVertices(i0, i1, i2)
        self.tris.addVertices(i0, i2, i3)

    def box(self, cx: float, cy: float, z0: float, sx: float, sy: float, sz: float, c: RGBA,
            top: RGBA | None = None, uv_scale: float = 1.0, bottom: bool = False) -> None:
        """Axis-aligned box resting on ``z0`` and centred on (cx, cy)."""
        x0, x1 = cx - sx / 2, cx + sx / 2
        y0, y1 = cy - sy / 2, cy + sy / 2
        z1 = z0 + sz
        t = top or c
        u = uv_scale
        self.quad((x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1), (0, 0, 1), t,
                  ((0, 0), (sx * u, 0), (sx * u, sy * u), (0, sy * u)))
        side = shade(c, 0.92)
        self.quad((x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1), (0, -1, 0), side,
                  ((0, 0), (sx * u, 0), (sx * u, sz * u), (0, sz * u)))
        self.quad((x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (0, 1, 0), side,
                  ((0, 0), (sx * u, 0), (sx * u, sz * u), (0, sz * u)))
        self.quad((x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (-1, 0, 0), c,
                  ((0, 0), (sy * u, 0), (sy * u, sz * u), (0, sz * u)))
        self.quad((x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1), (1, 0, 0), c,
                  ((0, 0), (sy * u, 0), (sy * u, sz * u), (0, sz * u)))
        if bottom:
            self.quad((x0, y1, z0), (x1, y1, z0), (x1, y0, z0), (x0, y0, z0), (0, 0, -1), shade(c, 0.6))

    def flat_rect(self, x0: float, y0: float, x1: float, y1: float, z: float, c: RGBA,
                  uvs: Sequence[tuple[float, float]] | None = None) -> None:
        self.quad((x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z), (0, 0, 1), c,
                  uvs or ((0, 0), (1, 0), (1, 1), (0, 1)))

    def cylinder(self, base: Vec, radius: float, length: float, c: RGBA, axis: str = "z",
                 segments: int = 16, caps: bool = True, cap_color: RGBA | None = None) -> None:
        """Cylinder starting at ``base`` extending ``length`` along ``axis``."""

        def to_world(a: float, b: float, h: float) -> Vec:
            bx, by, bz = base
            if axis == "z":
                return (bx + a, by + b, bz + h)
            if axis == "x":
                return (bx + h, by + a, bz + b)
            return (bx + a, by + h, bz + b)

        def normal(a: float, b: float) -> Vec:
            if axis == "z":
                return (a, b, 0.0)
            if axis == "x":
                return (0.0, a, b)
            return (a, 0.0, b)

        for i in range(segments):
            t0 = 2 * math.pi * i / segments
            t1 = 2 * math.pi * (i + 1) / segments
            a0, b0 = math.cos(t0), math.sin(t0)
            a1, b1 = math.cos(t1), math.sin(t1)
            p0 = to_world(a0 * radius, b0 * radius, 0)
            p1 = to_world(a1 * radius, b1 * radius, 0)
            p2 = to_world(a1 * radius, b1 * radius, length)
            p3 = to_world(a0 * radius, b0 * radius, length)
            i0 = self._vertex(p0, normal(a0, b0), c, (i / segments, 0))
            i1 = self._vertex(p1, normal(a1, b1), c, ((i + 1) / segments, 0))
            i2 = self._vertex(p2, normal(a1, b1), c, ((i + 1) / segments, 1))
            i3 = self._vertex(p3, normal(a0, b0), c, (i / segments, 1))
            if axis == "x":
                self.tris.addVertices(i0, i2, i1)
                self.tris.addVertices(i0, i3, i2)
            else:
                self.tris.addVertices(i0, i1, i2)
                self.tris.addVertices(i0, i2, i3)
        if caps:
            cc = cap_color or shade(c, 1.1)
            n_top = {"z": (0, 0, 1), "x": (1, 0, 0), "y": (0, 1, 0)}[axis]
            n_bot = tuple(-v for v in n_top)
            for h, n in ((length, n_top), (0.0, n_bot)):
                center = self._vertex(to_world(0, 0, h), n, cc, (0.5, 0.5))
                ring = [self._vertex(to_world(math.cos(2 * math.pi * i / segments) * radius,
                                              math.sin(2 * math.pi * i / segments) * radius, h), n, cc,
                                     (0.5 + 0.5 * math.cos(2 * math.pi * i / segments),
                                      0.5 + 0.5 * math.sin(2 * math.pi * i / segments)))
                        for i in range(segments)]
                for i in range(segments):
                    a, b = ring[i], ring[(i + 1) % segments]
                    top_side = (h == length)
                    if (axis == "x") ^ (not top_side):
                        self.tris.addVertices(center, b, a)
                    else:
                        self.tris.addVertices(center, a, b)

    def ring(self, cx: float, cy: float, z: float, r_in: float, r_out: float, c: RGBA,
             segments: int = 40) -> None:
        """Flat annulus facing +Z."""
        for i in range(segments):
            t0 = 2 * math.pi * i / segments
            t1 = 2 * math.pi * (i + 1) / segments
            p0 = (cx + math.cos(t0) * r_in, cy + math.sin(t0) * r_in, z)
            p1 = (cx + math.cos(t0) * r_out, cy + math.sin(t0) * r_out, z)
            p2 = (cx + math.cos(t1) * r_out, cy + math.sin(t1) * r_out, z)
            p3 = (cx + math.cos(t1) * r_in, cy + math.sin(t1) * r_in, z)
            self.quad(p0, p1, p2, p3, (0, 0, 1), c, ((i / segments, 0), (i / segments, 1),
                                                    ((i + 1) / segments, 1), ((i + 1) / segments, 0)))

    def disc(self, cx: float, cy: float, z: float, r: float, c: RGBA, segments: int = 32) -> None:
        center = self._vertex((cx, cy, z), (0, 0, 1), c, (0.5, 0.5))
        ring = [self._vertex((cx + math.cos(2 * math.pi * i / segments) * r,
                              cy + math.sin(2 * math.pi * i / segments) * r, z), (0, 0, 1), c,
                             (0.5 + 0.5 * math.cos(2 * math.pi * i / segments),
                              0.5 + 0.5 * math.sin(2 * math.pi * i / segments)))
                for i in range(segments)]
        for i in range(segments):
            self.tris.addVertices(center, ring[i], ring[(i + 1) % segments])

    def prism(self, poly: Sequence[tuple[float, float]], z0: float, z1: float, c: RGBA,
              top: RGBA | None = None, cap_bottom: bool = False) -> None:
        """Vertical extrusion of a convex polygon (counter-clockwise, seen from above)."""
        n = len(poly)
        side = shade(c, 0.95)
        for i in range(n):
            ax, ay = poly[i]
            bx, by = poly[(i + 1) % n]
            ex, ey = bx - ax, by - ay
            L = math.hypot(ex, ey) or 1.0
            nrm = (ey / L, -ex / L, 0.0)
            self.quad((ax, ay, z0), (bx, by, z0), (bx, by, z1), (ax, ay, z1), nrm, side,
                      ((0, 0), (L, 0), (L, z1 - z0), (0, z1 - z0)))
        t = top or c
        center = self._vertex((sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n, z1), (0, 0, 1), t, (0.5, 0.5))
        ring = [self._vertex((x, y, z1), (0, 0, 1), t, (x, y)) for x, y in poly]
        for i in range(n):
            self.tris.addVertices(center, ring[i], ring[(i + 1) % n])
        if cap_bottom:
            cb = self._vertex((sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n, z0), (0, 0, -1), shade(c, 0.6), (0.5, 0.5))
            rb = [self._vertex((x, y, z0), (0, 0, -1), shade(c, 0.6), (x, y)) for x, y in poly]
            for i in range(n):
                self.tris.addVertices(cb, rb[(i + 1) % n], rb[i])

    def node(self) -> GeomNode:
        geom = Geom(self.vdata)
        geom.addPrimitive(self.tris)
        gn = GeomNode(self.name)
        gn.addGeom(geom)
        return gn
