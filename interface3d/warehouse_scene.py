"""Static 3D warehouse built procedurally from a :class:`Scenario`.

Grid cell (row, col) is centred at ``world_xy``; the floor spans the grid plus
a service margin. Racks are generated on shelf cells (uprights, beams, three
storage levels with seeded cartons), loading docks on station cells (hazard
pad, conveyor toward the nearest wall, pallet, sign), charging pads on robot
start cells and yellow floor tape along every aisle/rack boundary.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

from panda3d.core import (
    Material,
    NodePath,
    TextNode,
    TextureStage,
    TransparencyAttrib,
)

from environment.grid import Position
from environment.scenario_generator import Scenario
from interface3d import textures
from interface3d.geometry import MeshBuilder, rgba, shade
from interface3d.sync import CELL, world_xy

RACK_HEIGHT = 2.35
LEVELS = (0.12, 0.85, 1.58)
STEEL_BLUE = rgba("#2f5d9e")
STEEL_ORANGE = rgba("#d9822b")
STEEL_DARK = rgba("#3a4250")
DECK = rgba("#5f6b7c")
CARTONS = [rgba("#b8875a"), rgba("#c49a6c"), rgba("#a9794c"), rgba("#d0a877"), rgba("#8f6a45")]
TAPE = rgba("#e8b923")
WHITE_PAINT = rgba("#d8dde6")

MARGIN = 2.5        # floor beyond the grid (cells)
WALL_HEIGHT = 5.0


def _material(shininess: float, specular: float) -> Material:
    m = Material()
    m.setShininess(shininess)
    m.setSpecular((specular, specular, specular, 1))
    m.setAmbient((1, 1, 1, 1))
    return m


@dataclass
class StationVisual:
    station_id: int
    position: Position
    root: NodePath
    stack_anchor: NodePath
    beacon: NodePath


@dataclass
class WarehouseVisual:
    root: NodePath
    width: int
    height: int
    center: tuple[float, float]
    extent: float
    stations: dict[int, StationVisual] = field(default_factory=dict)
    belts: list[NodePath] = field(default_factory=list)


def _dock_direction(scenario: Scenario, pos: Position) -> tuple[int, int] | None:
    """Direction from a station toward the outside (or a shelf) for the conveyor."""
    grid = scenario.grid
    for dr, dc in ((1, 0), (-1, 0), (0, -1), (0, 1)):
        n = (pos[0] + dr, pos[1] + dc)
        if not grid.in_bounds(n):
            return (dr, dc)
    for dr, dc in ((1, 0), (-1, 0), (0, -1), (0, 1)):
        if grid.is_shelf((pos[0] + dr, pos[1] + dc)):
            return (dr, dc)
    return None


def build_warehouse(scenario: Scenario, parent: NodePath, quality: str = "high") -> WarehouseVisual:
    grid = scenario.grid
    W, H = grid.width, grid.height
    root = parent.attachNewNode(f"warehouse-{scenario.scenario_id}")
    rng = random.Random(scenario.seed * 31 + 5)
    cx, cy = world_xy(((H - 1) / 2, (W - 1) / 2), H)
    vis = WarehouseVisual(root, W, H, (cx, cy), max(W, H) * CELL)

    # ------------------------------------------------------------- floor
    x0, x1 = -0.5 - MARGIN, W - 0.5 + MARGIN
    y0, y1 = -0.5 - MARGIN, H - 0.5 + MARGIN
    floor = MeshBuilder("floor")
    floor.flat_rect(x0, y0, x1, y1, 0.0, (1, 1, 1, 1),
                    ((x0 + 0.5, y0 + 0.5), (x1 + 0.5, y0 + 0.5), (x1 + 0.5, y1 + 0.5), (x0 + 0.5, y1 + 0.5)))
    fnp = root.attachNewNode(floor.node())
    fnp.setTexture(textures.concrete_floor())
    fnp.setMaterial(_material(60, 0.25))

    # ------------------------------------------------------------- floor markings
    marks = MeshBuilder("markings")
    tape_w = 0.06
    for r in range(H):
        for c in range(W):
            if not grid.is_walkable((r, c)):
                continue
            x, y = world_xy((r, c), H)
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                if grid.is_shelf((r + dr, c + dc)):
                    # tape on the aisle side of the rack edge
                    if dr == -1:
                        marks.flat_rect(x - 0.5, y + 0.5 - tape_w - 0.02, x + 0.5, y + 0.5 - 0.02, 0.004, TAPE)
                    elif dr == 1:
                        marks.flat_rect(x - 0.5, y - 0.5 + 0.02, x + 0.5, y - 0.5 + 0.02 + tape_w, 0.004, TAPE)
                    elif dc == -1:
                        marks.flat_rect(x - 0.5 + 0.02, y - 0.5, x - 0.5 + 0.02 + tape_w, y + 0.5, 0.004, TAPE)
                    else:
                        marks.flat_rect(x + 0.5 - tape_w - 0.02, y - 0.5, x + 0.5 - 0.02, y + 0.5, 0.004, TAPE)
    # perimeter walkway line
    for (ax, ay, bx, by) in ((x0 + 0.8, y0 + 0.8, x1 - 0.8, y0 + 0.86), (x0 + 0.8, y1 - 0.86, x1 - 0.8, y1 - 0.8),
                             (x0 + 0.8, y0 + 0.8, x0 + 0.86, y1 - 0.8), (x1 - 0.86, y0 + 0.8, x1 - 0.8, y1 - 0.8)):
        marks.flat_rect(ax, ay, bx, by, 0.004, WHITE_PAINT)
    # pick-face corner marks at parcel origins
    for spec in scenario.parcels:
        x, y = world_xy(spec.position, H)
        s, L, t = 0.42, 0.16, 0.035
        for sx in (-1, 1):
            for sy in (-1, 1):
                px, py = x + sx * s, y + sy * s
                marks.flat_rect(min(px, px - sx * L), min(py, py - sy * t), max(px, px - sx * L),
                                max(py, py - sy * t), 0.005, WHITE_PAINT)
                marks.flat_rect(min(px, px - sx * t), min(py, py - sy * L), max(px, px - sx * t),
                                max(py, py - sy * L), 0.005, WHITE_PAINT)
    mnp = root.attachNewNode(marks.node())
    mnp.setMaterial(_material(10, 0.1))

    # ------------------------------------------------------------- walls & columns
    walls = MeshBuilder("walls")
    wt = 0.25
    walls.box((x0 + x1) / 2, y1 + wt / 2, 0, x1 - x0 + 2 * wt, wt, WALL_HEIGHT, (1, 1, 1, 1), uv_scale=0.25)
    walls.box(x0 - wt / 2, (y0 + y1) / 2, 0, wt, y1 - y0, WALL_HEIGHT, (1, 1, 1, 1), uv_scale=0.25)
    walls.box(x1 + wt / 2, (y0 + y1) / 2, 0, wt, y1 - y0, WALL_HEIGHT, (1, 1, 1, 1), uv_scale=0.25)
    walls.box((x0 + x1) / 2, y0 - wt / 2, 0, x1 - x0 + 2 * wt, wt, 0.9, (1, 1, 1, 1), uv_scale=0.25)
    wnp = root.attachNewNode(walls.node())
    wnp.setTexture(textures.corrugated_wall())
    wnp.setMaterial(_material(20, 0.15))

    steel = MeshBuilder("columns")
    col_color = rgba("#4a5568")
    step = 6
    xs = [x0 + 0.3 + i * (x1 - x0 - 0.6) / max(1, round((x1 - x0) / step)) for i in range(round((x1 - x0) / step) + 1)]
    for xx in xs:
        steel.box(xx, y1 - 0.2, 0, 0.3, 0.3, WALL_HEIGHT + 0.6, col_color)
        steel.box(xx, y1 - 0.2, WALL_HEIGHT + 0.6, 0.34, y1 - y0 + 0.2, 0.18, shade(col_color, 0.9))  # roof truss
    for yy in [y0 + 0.3 + i * (y1 - y0 - 0.6) / 3 for i in range(4)]:
        steel.box(x0 + 0.2, yy, 0, 0.3, 0.3, WALL_HEIGHT + 0.6, col_color)
        steel.box(x1 - 0.2, yy, 0, 0.3, 0.3, WALL_HEIGHT + 0.6, col_color)
    # wall lamps (emissive)
    lamps = MeshBuilder("lamps")
    for xx in xs[:-1]:
        lamps.box(xx + (xs[1] - xs[0]) / 2 if len(xs) > 1 else xx, y1 - 0.12, WALL_HEIGHT - 1.0, 1.2, 0.12, 0.16,
                  rgba("#fff3d6"))
    for yy in [y0 + 2 + i * (y1 - y0 - 4) / 2 for i in range(3)]:
        lamps.box(x0 + 0.12, yy, WALL_HEIGHT - 1.0, 0.12, 1.2, 0.16, rgba("#fff3d6"))
        lamps.box(x1 - 0.12, yy, WALL_HEIGHT - 1.0, 0.12, 1.2, 0.16, rgba("#fff3d6"))
    snp = root.attachNewNode(steel.node())
    snp.setMaterial(_material(50, 0.4))
    lnp = root.attachNewNode(lamps.node())
    lnp.setLightOff(1)
    lnp.setShaderOff(1)

    # ------------------------------------------------------------- racks
    racks = MeshBuilder("racks")
    decks = MeshBuilder("decks")
    cartons = MeshBuilder("cartons")
    up, beam = 0.07, 0.06
    for (r, c) in grid.shelf_cells():
        x, y = world_xy((r, c), H)
        # uprights at the four corners (blue steel), beams (orange) at each level
        for sx in (-1, 1):
            for sy in (-1, 1):
                racks.box(x + sx * (0.5 - up / 2 - 0.02), y + sy * (0.5 - up / 2 - 0.02), 0, up, up,
                          RACK_HEIGHT, STEEL_BLUE)
        for z in LEVELS + (RACK_HEIGHT - 0.1,):
            racks.box(x, y - 0.5 + beam / 2 + 0.03, z, 0.96, beam, 0.09, STEEL_ORANGE)
            racks.box(x, y + 0.5 - beam / 2 - 0.03, z, 0.96, beam, 0.09, STEEL_ORANGE)
            if z in LEVELS:
                # wire-mesh deck: darker grey so cartons read clearly
                decks.box(x, y, z + 0.09, 0.9, 0.88, 0.025, DECK)
        # cartons on each level (seeded)
        count_levels = LEVELS if quality != "low" else LEVELS[:2]
        for z in count_levels:
            zc = z + 0.115
            filled = rng.random()
            if filled < 0.15:
                continue
            n = rng.choice((1, 2, 2, 3))
            widths = [rng.uniform(0.24, 0.34) for _ in range(n)]
            offset = -sum(widths) / 2 - 0.02 * (n - 1) / 2
            for w in widths:
                hgt = rng.uniform(0.28, 0.55)
                d = rng.uniform(0.5, 0.78)
                col = rng.choice(CARTONS)
                cartons.box(x + offset + w / 2, y, zc, w, d, hgt, col, top=shade(col, 1.08))
                offset += w + 0.02
    rnp = root.attachNewNode(racks.node())
    rnp.setMaterial(_material(70, 0.55))
    dnp = root.attachNewNode(decks.node())
    dnp.setMaterial(_material(30, 0.3))
    cnp = root.attachNewNode(cartons.node())
    cnp.setTexture(textures.label_texture())
    cnp.setTexScale(TextureStage.getDefault(), 2.2, 2.2)
    cnp.setMaterial(_material(8, 0.05))

    # ------------------------------------------------------------- charging pads
    pads = MeshBuilder("pads")
    for start in scenario.robot_starts:
        x, y = world_xy(start, H)
        pads.flat_rect(x - 0.44, y - 0.44, x + 0.44, y + 0.44, 0.006, rgba("#1b2436"))
        pads.ring(x, y, 0.008, 0.30, 0.36, rgba("#3b82f6"))
        pads.box(x, y + 0.47, 0, 0.5, 0.06, 0.12, rgba("#2b3445"))
    pnp = root.attachNewNode(pads.node())
    pnp.setMaterial(_material(30, 0.3))

    # ------------------------------------------------------------- loading docks
    hazard = textures.hazard_stripes()
    belt_tex = textures.conveyor_belt()
    for sid, pos in enumerate(scenario.stations):
        x, y = world_xy(pos, H)
        st_root = root.attachNewNode(f"station-{sid}")
        pad = MeshBuilder("dock-pad")
        b = 0.09
        for (ax, ay, bx, by) in ((x - 0.5, y - 0.5, x + 0.5, y - 0.5 + b), (x - 0.5, y + 0.5 - b, x + 0.5, y + 0.5),
                                 (x - 0.5, y - 0.5, x - 0.5 + b, y + 0.5), (x + 0.5 - b, y - 0.5, x + 0.5, y + 0.5)):
            pad.flat_rect(ax, ay, bx, by, 0.007, (1, 1, 1, 1), ((ax * 3, ay * 3), (bx * 3, ay * 3),
                                                              (bx * 3, by * 3), (ax * 3, by * 3)))
        pad_np = st_root.attachNewNode(pad.node())
        pad_np.setTexture(hazard)
        inner = MeshBuilder("dock-inner")
        inner.flat_rect(x - 0.41, y - 0.41, x + 0.41, y + 0.41, 0.006, rgba("#123524"))
        inner.ring(x, y, 0.009, 0.24, 0.29, rgba("#22c55e"))
        st_root.attachNewNode(inner.node()).setMaterial(_material(20, 0.2))

        direction = _dock_direction(scenario, pos) or (1, 0)
        dr, dc = direction
        # Conveyor outside the station cell, pointing away from the floor.
        ox, oy = dc * 1.0, -dr * 1.0
        conv = MeshBuilder("conveyor")
        frame = rgba("#3d4658")
        length = 1.6
        ccx, ccy = x + ox * (0.5 + length / 2), y + oy * (0.5 + length / 2)
        sx_, sy_ = (length, 0.8) if dc else (0.8, length)
        conv.box(ccx, ccy, 0, sx_, sy_, 0.5, frame)
        st_root.attachNewNode(conv.node()).setMaterial(_material(40, 0.4))
        belt = MeshBuilder("belt")
        bx0, bx1 = ccx - sx_ / 2 + 0.06, ccx + sx_ / 2 - 0.06
        by0, by1 = ccy - sy_ / 2 + 0.06, ccy + sy_ / 2 - 0.06
        if dc:
            belt.flat_rect(bx0, by0, bx1, by1, 0.505, (1, 1, 1, 1), ((0, 0), (0, length * 3), (1, length * 3), (1, 0)))
        else:
            belt.flat_rect(bx0, by0, bx1, by1, 0.505, (1, 1, 1, 1), ((0, 0), (1, 0), (1, length * 3), (0, length * 3)))
        belt_np = st_root.attachNewNode(belt.node())
        belt_np.setTexture(belt_tex)
        vis.belts.append(belt_np)
        # pallet next to the station on the conveyor side wall
        pallet = MeshBuilder("pallet")
        side = (-oy, ox) if (dr or dc) else (1, 0)
        palx, paly = x + ox * 0.5 + side[0] * 1.15, y + oy * 0.5 + side[1] * 1.15
        pallet.box(palx, paly, 0, 0.9, 0.9, 0.12, rgba("#8b6b45"))
        for k in (-0.3, 0, 0.3):
            pallet.box(palx + (k if dr else 0), paly + (0 if dr else k), 0.12, 0.85 if not dr else 0.12,
                       0.85 if dr else 0.12, 0.03, rgba("#a07a4f"))
        st_root.attachNewNode(pallet.node()).setMaterial(_material(5, 0.05))
        stack_anchor = st_root.attachNewNode("stack")
        stack_anchor.setPos(palx, paly, 0.15)
        # sign post with label and beacon
        post = MeshBuilder("post")
        spx, spy = x + ox * 0.5 - side[0] * 0.62, y + oy * 0.5 - side[1] * 0.62
        post.box(spx, spy, 0, 0.08, 0.08, 1.9, rgba("#2b3445"))
        post.box(spx, spy, 1.55, 0.62, 0.06, 0.38, rgba("#15803d"))
        st_root.attachNewNode(post.node()).setMaterial(_material(30, 0.3))
        tn = TextNode(f"sign-{sid}")
        tn.setText(f"DOCK D{sid}")
        tn.setAlign(TextNode.ACenter)
        tn.setTextColor(1, 1, 1, 1)
        sign = st_root.attachNewNode(tn)
        sign.setScale(0.17)
        sign.setPos(spx, spy - 0.04, 1.69)
        sign.setLightOff(1)
        sign.setShaderOff(1)
        sign2 = sign.copyTo(st_root)
        sign2.setPos(spx, spy + 0.04, 1.69)
        sign2.setH(180)
        beacon_mb = MeshBuilder("beacon")
        beacon_mb.cylinder((0, 0, 0), 0.07, 0.14, rgba("#22c55e"), segments=12)
        beacon = st_root.attachNewNode(beacon_mb.node())
        beacon.setPos(spx, spy, 1.95)
        beacon.setLightOff(1)
        beacon.setShaderOff(1)
        vis.stations[sid] = StationVisual(sid, pos, st_root, stack_anchor, beacon)

    # Merge static geometry into few batches (huge draw-call reduction).
    for np_ in (rnp, dnp, cnp, mnp, snp, wnp):
        np_.flattenStrong()
    return vis


def make_carton(name: str, color=None, size: float = 0.36) -> NodePath:
    """A parcel carton with tape and label (used for floor parcels and carried parcels)."""
    col = color or rgba("#c8935a")
    mb = MeshBuilder(name)
    mb.box(0, 0, 0, size, size, size * 0.8, col, top=shade(col, 1.07))
    mb.box(0, 0, size * 0.8, size * 0.16, size + 0.004, 0.004, rgba("#e7cfa3"))
    mb.box(0, -size / 2 - 0.002, size * 0.3, size * 0.5, 0.004, size * 0.3, rgba("#f1ede4"))
    np_ = NodePath(mb.node())
    np_.setMaterial(_material(6, 0.05))
    return np_


def make_marker_ring(color, r_in: float, r_out: float) -> NodePath:
    mb = MeshBuilder("ring")
    mb.ring(0, 0, 0, r_in, r_out, color)
    np_ = NodePath(mb.node())
    np_.setTransparency(TransparencyAttrib.MAlpha)
    np_.setLightOff(1)
    np_.setShaderOff(1)
    np_.setDepthWrite(False)
    np_.setBin("fixed", 5)
    return np_
