"""Static 3D warehouse built procedurally from a :class:`Scenario`.

Scene graph (optimised for integrated GPUs):

    warehouse
    ├── floor          one textured mesh, baked ambient occlusion in vertex colours
    ├── markings       tape, walkway, stop lines, aisle IDs, dock chevrons (flattened)
    ├── structure      walls, columns, trusses, high-bay lamps (flattened)
    ├── chunk-R-C ...  8x8-cell spatial chunks -> frustum culling works per chunk
    │   ├── base       racks, decks, cartons, parcel stands (flattened per material)
    │   └── LODNode    near-only detail: bracing, base plates, carton tape/labels
    └── station-N      loading docks (conveyor, rollers, pallet, sign, live progress)

Repeated geometry (thousands of rack parts and cartons) is merged per chunk and
material instead of hardware-instanced: Panda3D's auto-shader does not support
instancing, and merged static batches give fewer draw calls anyway.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

from panda3d.core import (
    LODNode,
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
STEEL_ORANGE = rgba("#e07b24")
STEEL_GALV = rgba("#8a96a8")
DECK = rgba("#5f6b7c")
CARTONS = [rgba("#b8875a"), rgba("#c49a6c"), rgba("#a9794c"), rgba("#d0a877"), rgba("#8f6a45")]
TOTES = [rgba("#2563eb"), rgba("#475569"), rgba("#0f766e")]
TAPE = rgba("#e8b923")
WHITE_PAINT = rgba("#d8dde6")
WALKWAY = rgba("#1f6f43")
DOCK_COLORS = [rgba("#22c55e"), rgba("#14b8a6"), rgba("#eab308"), rgba("#ec4899")]

STAND_TOP = 0.40          # parcel stand top surface (carton sits here)
STAND_UNDERSIDE = 0.38
MARGIN = 2.5              # floor beyond the grid (cells)
WALL_HEIGHT = 5.0
CHUNK = 8                 # cells per chunk side
DETAIL_DISTANCE = 24.0    # LOD: detail layer visible closer than this (world units)
CONVEYOR_LENGTH = 1.2


def dock_color(station_id: int):
    return DOCK_COLORS[station_id % len(DOCK_COLORS)]


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
    color: tuple = (1, 1, 1, 1)
    conveyor_start: tuple = (0.0, 0.0, 0.5)
    conveyor_end: tuple = (0.0, 0.0, 0.5)
    sign_text: TextNode | None = None
    progress_fill: NodePath | None = None


@dataclass
class WarehouseVisual:
    root: NodePath
    width: int
    height: int
    center: tuple[float, float]
    extent: float
    stations: dict[int, StationVisual] = field(default_factory=dict)
    belts: list[NodePath] = field(default_factory=list)
    chunks: list[NodePath] = field(default_factory=list)
    lod_nodes: list[NodePath] = field(default_factory=list)
    no_shadow: list[NodePath] = field(default_factory=list)   # receivers only (floor, decals)
    overhead: NodePath | None = None      # roof trusses + high-bay lamps (hidden in top-down views)


def _dock_direction(scenario: Scenario, pos: Position) -> tuple[int, int] | None:
    """Direction from a station toward the outside (or a shelf) for the conveyor."""
    grid = scenario.grid
    for dr, dc in ((1, 0), (-1, 0), (0, -1), (0, 1)):
        if not grid.in_bounds((pos[0] + dr, pos[1] + dc)):
            return (dr, dc)
    for dr, dc in ((1, 0), (-1, 0), (0, -1), (0, 1)):
        if grid.is_shelf((pos[0] + dr, pos[1] + dc)):
            return (dr, dc)
    return None


def _floor_text(parent: NodePath, text: str, x: float, y: float, h: float, scale: float, color) -> None:
    tn = TextNode("decal")
    tn.setText(text)
    tn.setAlign(TextNode.ACenter)
    tn.setTextColor(*color)
    np_ = parent.attachNewNode(tn.generate())
    np_.setScale(scale)
    np_.setP(-90)
    np_.setH(h)
    np_.setPos(x, y, 0.006)


def build_warehouse(scenario: Scenario, parent: NodePath, quality: str = "high") -> WarehouseVisual:
    grid = scenario.grid
    W, H = grid.width, grid.height
    root = parent.attachNewNode(f"warehouse-{scenario.scenario_id}")
    rng = random.Random(scenario.seed * 31 + 5)
    cx, cy = world_xy(((H - 1) / 2, (W - 1) / 2), H)
    vis = WarehouseVisual(root, W, H, (cx, cy), max(W, H) * CELL)
    x0, x1 = -0.5 - MARGIN, W - 0.5 + MARGIN
    y0, y1 = -0.5 - MARGIN, H - 0.5 + MARGIN
    m = int(MARGIN + 0.5)

    def shelf(r: int, c: int) -> bool:
        return grid.in_bounds((r, c)) and grid.is_shelf((r, c))

    # ------------------------------------------------------------------ floor
    # One quad per cell (grid + margin). Vertex colours bake ambient occlusion:
    # each corner is darkened by the number of rack cells touching it.
    floor = MeshBuilder("floor")
    for r in range(-m, H + m):
        for c in range(-m, W + m):
            x, y = world_xy((r, c), H)
            if shelf(r, c):
                tone = 0.55          # under the racks: deep shadow
            else:
                tone = 1.0
            var = 0.95 + 0.1 * rng.random()

            def corner(dr: int, dc: int) -> tuple:
                occ = sum(shelf(r + a, c + b) for a in (0, dr) for b in (0, dc)) - (1 if shelf(r, c) else 0)
                k = tone * var * (1.0 - 0.16 * occ)
                return (k, k, k * 1.02, 1.0)

            # corners: (dr, dc) direction toward the neighbouring cells of that corner
            cs = [corner(1, -1), corner(1, 1), corner(-1, 1), corner(-1, -1)]
            pts = [(x - 0.5, y - 0.5), (x + 0.5, y - 0.5), (x + 0.5, y + 0.5), (x - 0.5, y + 0.5)]
            uv = [(0, 0), (1, 0), (1, 1), (0, 1)]
            idx = [floor._vertex((px, py, 0.0), (0, 0, 1), col, t) for (px, py), col, t in zip(pts, cs, uv)]
            floor.tris.addVertices(idx[0], idx[1], idx[2])
            floor.tris.addVertices(idx[0], idx[2], idx[3])
    fnp = root.attachNewNode(floor.node())
    fnp.setTexture(textures.concrete_floor())
    fnp.setMaterial(_material(70, 0.22))
    vis.no_shadow.append(fnp)

    # ------------------------------------------------------------------ markings
    marks_root = root.attachNewNode("markings")
    marks = MeshBuilder("markings")
    tape_w = 0.06
    for r in range(H):
        for c in range(W):
            if not grid.is_walkable((r, c)):
                continue
            x, y = world_xy((r, c), H)
            if shelf(r - 1, c):
                marks.flat_rect(x - 0.5, y + 0.5 - tape_w - 0.02, x + 0.5, y + 0.5 - 0.02, 0.004, TAPE)
            if shelf(r + 1, c):
                marks.flat_rect(x - 0.5, y - 0.5 + 0.02, x + 0.5, y - 0.5 + 0.02 + tape_w, 0.004, TAPE)
            if shelf(r, c - 1):
                marks.flat_rect(x - 0.5 + 0.02, y - 0.5, x - 0.5 + 0.02 + tape_w, y + 0.5, 0.004, TAPE)
            if shelf(r, c + 1):
                marks.flat_rect(x + 0.5 - tape_w - 0.02, y - 0.5, x + 0.5 - 0.02, y + 0.5, 0.004, TAPE)
    # green pedestrian walkway along the walls, edged in white
    for (ax, ay, bx, by, horiz) in ((x0 + 0.5, y1 - 1.25, x1 - 0.5, y1 - 0.65, True),
                                    (x0 + 0.5, y0 + 0.65, x0 + 1.1, y1 - 0.65, False),
                                    (x1 - 1.1, y0 + 0.65, x1 - 0.5, y1 - 0.65, False)):
        marks.flat_rect(ax, ay, bx, by, 0.003, WALKWAY)
        if horiz:
            marks.flat_rect(ax, ay - 0.05, bx, ay, 0.004, WHITE_PAINT)
            marks.flat_rect(ax, by, bx, by + 0.05, 0.004, WHITE_PAINT)
        else:
            marks.flat_rect(ax - 0.05, ay, ax, by, 0.004, WHITE_PAINT)
            marks.flat_rect(bx, ay, bx + 0.05, by, 0.004, WHITE_PAINT)
    # pick-face corner marks at parcel origins
    for spec in scenario.parcels:
        x, y = world_xy(spec.position, H)
        s, L, t = 0.46, 0.14, 0.03
        for sx in (-1, 1):
            for sy in (-1, 1):
                px, py = x + sx * s, y + sy * s
                marks.flat_rect(min(px, px - sx * L), min(py, py - sy * t), max(px, px - sx * L),
                                max(py, py - sy * t), 0.005, WHITE_PAINT)
                marks.flat_rect(min(px, px - sx * t), min(py, py - sy * L), max(px, px - sx * t),
                                max(py, py - sy * L), 0.005, WHITE_PAINT)
    # aisles: horizontal runs of floor between two rack rows -> ID + stop lines
    aisle_no = 0
    for r in range(1, H - 1):
        c = 0
        while c < W:
            if grid.is_walkable((r, c)) and shelf(r - 1, c) and shelf(r + 1, c):
                start = c
                while c < W and grid.is_walkable((r, c)) and shelf(r - 1, c) and shelf(r + 1, c):
                    c += 1
                if c - start >= 3:
                    aisle_no += 1
                    for end_c, side in ((start, -1), (c - 1, 1)):
                        ex, ey = world_xy((r, end_c), H)
                        lx = ex + side * 0.5
                        marks.flat_rect(lx - 0.04, ey - 0.45, lx + 0.04, ey + 0.45, 0.005, WHITE_PAINT)
                    ax_, ay_ = world_xy((r, start), H)
                    _floor_text(marks_root, f"A{aisle_no}", ax_ + 0.05, ay_, 0, 0.32, (0.85, 0.87, 0.9, 1))
            else:
                c += 1
    marks_root.attachNewNode(marks.node())
    marks_root.setMaterial(_material(10, 0.1))
    marks_root.setDepthOffset(1)
    vis.no_shadow.append(marks_root)

    # ------------------------------------------------------------------ structure
    structure = root.attachNewNode("structure")
    walls = MeshBuilder("walls")
    wt = 0.25
    walls.box((x0 + x1) / 2, y1 + wt / 2, 0, x1 - x0 + 2 * wt, wt, WALL_HEIGHT, (1, 1, 1, 1), uv_scale=0.25)
    walls.box(x0 - wt / 2, (y0 + y1) / 2, 0, wt, y1 - y0, WALL_HEIGHT, (1, 1, 1, 1), uv_scale=0.25)
    walls.box(x1 + wt / 2, (y0 + y1) / 2, 0, wt, y1 - y0, WALL_HEIGHT, (1, 1, 1, 1), uv_scale=0.25)
    walls.box((x0 + x1) / 2, y0 - wt / 2, 0, x1 - x0 + 2 * wt, wt, 0.9, (1, 1, 1, 1), uv_scale=0.25)
    wnp = structure.attachNewNode(walls.node())
    wnp.setTexture(textures.corrugated_wall())
    wnp.setMaterial(_material(20, 0.15))
    steel = MeshBuilder("columns")
    trusses = MeshBuilder("trusses")
    col_color = rgba("#4a5568")
    n_x = max(1, round((x1 - x0) / 6))
    xs = [x0 + 0.3 + i * (x1 - x0 - 0.6) / n_x for i in range(n_x + 1)]
    for xx in xs:
        steel.box(xx, y1 - 0.2, 0, 0.3, 0.3, WALL_HEIGHT + 0.6, col_color)
        steel.box(xx, y1 - 0.2, 0, 0.42, 0.42, 0.6, rgba("#e8b923"))             # column guard
        trusses.box(xx, (y0 + y1) / 2 - 0.1, WALL_HEIGHT + 0.6, 0.3, y1 - y0 + 0.2, 0.18, shade(col_color, 0.9))
    for yy in [y0 + 0.3 + i * (y1 - y0 - 0.6) / 3 for i in range(4)]:
        for xx in (x0 + 0.2, x1 - 0.2):
            steel.box(xx, yy, 0, 0.3, 0.3, WALL_HEIGHT + 0.6, col_color)
            steel.box(xx, yy, 0, 0.42, 0.42, 0.6, rgba("#e8b923"))
    snp = structure.attachNewNode(steel.node())
    snp.setMaterial(_material(50, 0.4))
    lamps = MeshBuilder("lamps")
    bay_lamps = MeshBuilder("bay-lamps")
    lamp = rgba("#fff6df")
    for xx in xs:                                           # high-bay fixtures under the trusses
        for k in range(1, 4):
            yy = y0 + k * (y1 - y0) / 4
            bay_lamps.box(xx, yy, WALL_HEIGHT + 0.46, 0.5, 0.22, 0.06, lamp)
    for yy in [y0 + 2 + i * (y1 - y0 - 4) / 2 for i in range(3)]:
        lamps.box(x0 + 0.12, yy, WALL_HEIGHT - 1.0, 0.12, 1.2, 0.16, lamp)
        lamps.box(x1 - 0.12, yy, WALL_HEIGHT - 1.0, 0.12, 1.2, 0.16, lamp)
    lnp = structure.attachNewNode(lamps.node())
    lnp.setLightOff(1)
    lnp.setShaderOff(1)
    overhead = root.attachNewNode("overhead")
    overhead.attachNewNode(trusses.node()).setMaterial(_material(50, 0.4))
    bnp = overhead.attachNewNode(bay_lamps.node())
    bnp.setLightOff(1)
    bnp.setShaderOff(1)
    vis.overhead = overhead

    # ------------------------------------------------------------------ chunks: racks, cartons, stands
    chunk_builders: dict[tuple[int, int], dict[str, MeshBuilder]] = {}

    def builders(r: int, c: int) -> dict[str, MeshBuilder]:
        key = (r // CHUNK, c // CHUNK)
        if key not in chunk_builders:
            chunk_builders[key] = {k: MeshBuilder(k) for k in
                                   ("racks", "decks", "cartons", "stands", "detail_steel", "detail_cartons")}
        return chunk_builders[key]

    up, beam = 0.07, 0.06
    detail_levels = quality != "low"
    for (r, c) in grid.shelf_cells():
        b = builders(r, c)
        racks, decks, cartons = b["racks"], b["decks"], b["cartons"]
        dsteel, dcart = b["detail_steel"], b["detail_cartons"]
        x, y = world_xy((r, c), H)
        for sx in (-1, 1):
            for sy in (-1, 1):
                ux, uy = x + sx * (0.5 - up / 2 - 0.02), y + sy * (0.5 - up / 2 - 0.02)
                racks.box(ux, uy, 0, up, up, RACK_HEIGHT, STEEL_BLUE)
                dsteel.box(ux, uy, 0, 0.13, 0.13, 0.012, STEEL_GALV)       # base plates
        for z in LEVELS + (RACK_HEIGHT - 0.1,):
            racks.box(x, y - 0.5 + beam / 2 + 0.03, z, 0.96, beam, 0.09, STEEL_ORANGE)
            racks.box(x, y + 0.5 - beam / 2 - 0.03, z, 0.96, beam, 0.09, STEEL_ORANGE)
            if z in LEVELS:
                decks.box(x, y, z + 0.09, 0.9, 0.88, 0.025, DECK)
        # end frames: diagonal bracing + yellow end guard where a rack run ends at an aisle
        for side in (-1, 1):
            if not shelf(r, c + side) and grid.in_bounds((r, c + side)):
                fx = x + side * 0.47
                for k in range(3):
                    z0_, z1_ = 0.15 + k * 0.72, 0.15 + (k + 1) * 0.72
                    a, bb = (y - 0.42, y + 0.42) if k % 2 == 0 else (y + 0.42, y - 0.42)
                    dsteel.quad((fx, a, z0_), (fx, bb, z1_ - 0.03), (fx, bb, z1_), (fx, a, z0_ + 0.03),
                                (side, 0, 0), STEEL_GALV)
                    dsteel.quad((fx, a, z0_ + 0.03), (fx, bb, z1_), (fx, bb, z1_ - 0.03), (fx, a, z0_),
                                (-side, 0, 0), STEEL_GALV)
                racks.box(x + side * 0.53, y, 0, 0.08, 0.9, 0.32, rgba("#e8b923"))
        # goods on each level (seeded): cartons, plastic totes, wrapped pallets
        levels = LEVELS if detail_levels else LEVELS[:2]
        for z in levels:
            zc = z + 0.115
            roll = rng.random()
            if roll < 0.12:
                continue
            if z == LEVELS[0] and roll > 0.86:
                cartons.box(x, y, zc, 0.86, 0.8, 0.1, rgba("#8b6b45"))
                cartons.box(x, y, zc + 0.1, 0.8, 0.74, 0.5, rgba("#c9ced6"), top=rgba("#dde1e7"))
                continue
            n = rng.choice((1, 2, 2, 3))
            widths = [rng.uniform(0.24, 0.34) for _ in range(n)]
            offset = -sum(widths) / 2 - 0.02 * (n - 1) / 2
            for w in widths:
                d = rng.uniform(0.5, 0.78)
                if rng.random() < 0.18:
                    hgt = rng.uniform(0.2, 0.3)
                    col = rng.choice(TOTES)
                    cartons.box(x + offset + w / 2, y, zc, w, d, hgt, col, top=shade(col, 0.6))
                else:
                    hgt = rng.uniform(0.28, 0.55)
                    col = rng.choice(CARTONS)
                    cartons.box(x + offset + w / 2, y, zc, w, d, hgt, col, top=shade(col, 1.08))
                    dcart.box(x + offset + w / 2, y, zc + hgt, w * 0.18, d + 0.004, 0.004, rgba("#e7cfa3"))
                    dcart.box(x + offset + w / 2, y - d / 2 - 0.003, zc + hgt * 0.35, w * 0.55, 0.004,
                              hgt * 0.32, rgba("#f1ede4"))
                offset += w + 0.02
    # parcel stands (the robots drive underneath and lift the carton off)
    for spec in scenario.parcels:
        r, c = spec.position
        st = builders(r, c)["stands"]
        x, y = world_xy(spec.position, H)
        for sx in (-1, 1):
            for sy in (-1, 1):
                st.box(x + sx * 0.44, y + sy * 0.44, 0, 0.05, 0.05, STAND_UNDERSIDE, rgba("#4b5563"))
        st.box(x, y, STAND_UNDERSIDE, 0.94, 0.94, STAND_TOP - STAND_UNDERSIDE, rgba("#6b7280"), top=rgba("#7c8594"))
        dc_ = dock_color(spec.destination_id)
        st.box(x, y - 0.47, STAND_UNDERSIDE - 0.035, 0.5, 0.012, 0.035, dc_)          # destination tag
        st.box(x, y + 0.47, STAND_UNDERSIDE - 0.035, 0.5, 0.012, 0.035, dc_)

    mats = {"racks": _material(70, 0.55), "decks": _material(30, 0.3), "cartons": _material(8, 0.05),
            "stands": _material(40, 0.35), "detail_steel": _material(70, 0.5), "detail_cartons": _material(8, 0.05)}
    label_tex = textures.label_texture()
    for (cr, cc), bs in sorted(chunk_builders.items()):
        chunk = root.attachNewNode(f"chunk-{cr}-{cc}")
        base = chunk.attachNewNode("base")
        for key in ("racks", "decks", "cartons", "stands"):
            if not bs[key].empty:
                np_ = base.attachNewNode(bs[key].node())
                np_.setMaterial(mats[key])
                if key == "cartons":
                    np_.setTexture(label_tex)
                    np_.setTexScale(TextureStage.getDefault(), 2.2, 2.2)
        base.flattenStrong()
        if quality != "low" and not (bs["detail_steel"].empty and bs["detail_cartons"].empty):
            lod = LODNode(f"lod-{cr}-{cc}")
            lod_np = chunk.attachNewNode(lod)
            detail = lod_np.attachNewNode("detail")
            for key in ("detail_steel", "detail_cartons"):
                if not bs[key].empty:
                    detail.attachNewNode(bs[key].node()).setMaterial(mats[key])
            detail.flattenStrong()
            ccx, ccy = world_xy(((cr + 0.5) * CHUNK - 0.5, (cc + 0.5) * CHUNK - 0.5), H)
            lod.addSwitch(DETAIL_DISTANCE, 0.0)
            lod.setCenter((ccx, ccy, 1.0))
            vis.lod_nodes.append(lod_np)
        vis.chunks.append(chunk)

    # ------------------------------------------------------------------ charging pads
    pads = MeshBuilder("pads")
    for start in scenario.robot_starts:
        x, y = world_xy(start, H)
        pads.flat_rect(x - 0.44, y - 0.44, x + 0.44, y + 0.44, 0.006, rgba("#1b2436"))
        pads.ring(x, y, 0.008, 0.30, 0.36, rgba("#3b82f6"))
        pads.box(x, y + 0.47, 0, 0.5, 0.06, 0.12, rgba("#2b3445"))
    pnp = root.attachNewNode(pads.node())
    pnp.setMaterial(_material(30, 0.3))
    pnp.setDepthOffset(1)

    # ------------------------------------------------------------------ loading docks
    hazard = textures.hazard_stripes()
    belt_tex = textures.conveyor_belt()
    for sid, pos in enumerate(scenario.stations):
        vis.stations[sid] = _build_dock(scenario, sid, pos, root, hazard, belt_tex, vis)

    for np_ in (structure, marks_root):
        np_.flattenStrong()
    if quality == "low" and len(vis.chunks) > 1:
        # Weak GPUs: fewer, larger batches matter more than per-chunk culling.
        merged = root.attachNewNode("chunks-merged")
        for chunk in vis.chunks:
            chunk.reparentTo(merged)
        merged.flattenStrong()
        vis.chunks = [merged]
    return vis


def _build_dock(scenario: Scenario, sid: int, pos: Position, root: NodePath, hazard, belt_tex,
                vis: WarehouseVisual) -> StationVisual:
    H = scenario.grid.height
    x, y = world_xy(pos, H)
    color = dock_color(sid)
    st_root = root.attachNewNode(f"station-{sid}")
    pad = MeshBuilder("dock-pad")
    b = 0.09
    for (ax, ay, bx, by) in ((x - 0.5, y - 0.5, x + 0.5, y - 0.5 + b), (x - 0.5, y + 0.5 - b, x + 0.5, y + 0.5),
                             (x - 0.5, y - 0.5, x - 0.5 + b, y + 0.5), (x + 0.5 - b, y - 0.5, x + 0.5, y + 0.5)):
        pad.flat_rect(ax, ay, bx, by, 0.007, (1, 1, 1, 1), ((ax * 3, ay * 3), (bx * 3, ay * 3),
                                                          (bx * 3, by * 3), (ax * 3, by * 3)))
    pad_np = st_root.attachNewNode(pad.node())
    pad_np.setTexture(hazard)
    pad_np.setDepthOffset(1)
    vis.no_shadow.append(pad_np)
    inner = MeshBuilder("dock-inner")
    inner.flat_rect(x - 0.41, y - 0.41, x + 0.41, y + 0.41, 0.006, shade(color, 0.25))
    inner.ring(x, y, 0.009, 0.24, 0.29, color)
    inner_np = st_root.attachNewNode(inner.node())
    inner_np.setMaterial(_material(20, 0.2))
    inner_np.setDepthOffset(1)
    vis.no_shadow.append(inner_np)

    dr, dc = _dock_direction(scenario, pos) or (1, 0)
    ox, oy = dc * 1.0, -dr * 1.0                         # outward direction (world)
    side = (-oy, ox)
    # approach chevrons painted on the floor in front of the dock
    chev = MeshBuilder("chevrons")
    for k in (1, 2):
        px, py = x - ox * (0.5 + 0.35 * k), y - oy * (0.5 + 0.35 * k)
        for s in (-1, 1):
            a = (px + side[0] * s * 0.28 - ox * 0.12, py + side[1] * s * 0.28 - oy * 0.12, 0.006)
            bb = (px + ox * 0.06, py + oy * 0.06, 0.006)
            n = 0.05
            chev.quad(a, (bb[0], bb[1], 0.006), (bb[0] + ox * n, bb[1] + oy * n, 0.006),
                      (a[0] + ox * n, a[1] + oy * n, 0.006), (0, 0, 1), shade(color, 0.9))
    chev_np = st_root.attachNewNode(chev.node())
    chev_np.setTwoSided(True)
    chev_np.setDepthOffset(1)
    vis.no_shadow.append(chev_np)

    length = CONVEYOR_LENGTH
    ccx, ccy = x + ox * (0.5 + length / 2), y + oy * (0.5 + length / 2)
    sx_, sy_ = (length, 0.8) if dc else (0.8, length)
    conv = MeshBuilder("conveyor")
    conv.box(ccx, ccy, 0, sx_, sy_, 0.46, rgba("#3d4658"))
    for s in (-1, 1):                                      # yellow side rails
        rx, ry = ccx + side[0] * s * 0.42, ccy + side[1] * s * 0.42
        conv.box(rx, ry, 0.46, abs(side[0]) * 0.05 + abs(ox) * length, abs(side[1]) * 0.05 + abs(oy) * length,
                 0.08, rgba("#e8b923"))
    for s in (-1, 1):                                      # bollards at the conveyor mouth
        bx_, by_ = x + ox * 0.62 + side[0] * s * 0.62, y + oy * 0.62 + side[1] * s * 0.62
        conv.cylinder((bx_, by_, 0), 0.07, 0.9, rgba("#e8b923"), segments=12)
        conv.cylinder((bx_, by_, 0.55), 0.072, 0.1, rgba("#111418"), segments=12, caps=False)
    st_root.attachNewNode(conv.node()).setMaterial(_material(40, 0.4))
    rollers = MeshBuilder("rollers")
    n_rollers = int(length / 0.12)
    for k in range(n_rollers):
        t = (k + 0.5) / n_rollers - 0.5
        rx, ry = ccx + ox * t * length, ccy + oy * t * length
        if dc:
            rollers.cylinder((rx, ry - 0.36, 0.475), 0.025, 0.72, rgba("#9ca3af"), axis="y", segments=8, caps=False)
        else:
            rollers.cylinder((rx - 0.36, ry, 0.475), 0.025, 0.72, rgba("#9ca3af"), axis="x", segments=8, caps=False)
    st_root.attachNewNode(rollers.node()).setMaterial(_material(80, 0.7))
    belt = MeshBuilder("belt")
    bx0, bx1 = ccx - sx_ / 2 + 0.08, ccx + sx_ / 2 - 0.08
    by0, by1 = ccy - sy_ / 2 + 0.08, ccy + sy_ / 2 - 0.08
    uvs = ((0, 0), (0, length * 3), (1, length * 3), (1, 0)) if dc else ((0, 0), (1, 0), (1, length * 3), (0, length * 3))
    belt.flat_rect(bx0, by0, bx1, by1, 0.462, (0.8, 0.8, 0.8, 1), uvs)
    belt_np = st_root.attachNewNode(belt.node())
    belt_np.setTexture(belt_tex)
    vis.belts.append(belt_np)

    # pallet at the far end of the conveyor; delivered cartons stack there
    palx, paly = x + ox * (0.5 + length + 0.5), y + oy * (0.5 + length + 0.5)
    pallet = MeshBuilder("pallet")
    pallet.box(palx, paly, 0, 0.9, 0.9, 0.12, rgba("#8b6b45"))
    for k in (-0.3, 0, 0.3):
        pallet.box(palx + (k if dr else 0), paly + (0 if dr else k), 0.12, 0.85 if not dr else 0.12,
                   0.85 if dr else 0.12, 0.03, rgba("#a07a4f"))
    st_root.attachNewNode(pallet.node()).setMaterial(_material(5, 0.05))
    stack_anchor = st_root.attachNewNode("stack")
    stack_anchor.setPos(palx, paly, 0.15)

    # sign post: dock colour panel, live "delivered / total" text and progress bar
    spx, spy = x + ox * 0.3 + side[0] * 0.95, y + oy * 0.3 + side[1] * 0.95
    post = MeshBuilder("post")
    post.box(spx, spy, 0, 0.08, 0.08, 2.1, rgba("#2b3445"))
    post.box(spx, spy, 1.62, 0.9, 0.07, 0.5, shade(color, 0.55))
    post.box(spx, spy, 1.52, 0.8, 0.075, 0.07, rgba("#111827"))
    st_root.attachNewNode(post.node()).setMaterial(_material(30, 0.3))
    fill = MeshBuilder("progress")
    fill.box(0.4, 0, 0, 0.8, 0.08, 0.07, color)
    progress_fill = st_root.attachNewNode(fill.node())
    progress_fill.setPos(spx - 0.4, spy, 1.52)
    progress_fill.setSx(0.001)
    progress_fill.setLightOff(1)
    progress_fill.setShaderOff(1)
    tn = TextNode(f"sign-{sid}")
    tn.setText(f"DOCK D{sid}")
    tn.setAlign(TextNode.ACenter)
    tn.setTextColor(1, 1, 1, 1)
    for k, (dy_, h_) in enumerate(((-0.045, 0), (0.045, 180))):
        sign = st_root.attachNewNode(tn)
        sign.setScale(0.15)
        sign.setPos(spx, spy + dy_, 1.8)
        sign.setH(h_)
        sign.setLightOff(1)
        sign.setShaderOff(1)
    beacon_mb = MeshBuilder("beacon")
    beacon_mb.cylinder((0, 0, 0), 0.07, 0.14, color, segments=12)
    beacon = st_root.attachNewNode(beacon_mb.node())
    beacon.setPos(spx, spy, 2.12)
    beacon.setLightOff(1)
    beacon.setShaderOff(1)
    conveyor_start = (x + ox * 0.55, y + oy * 0.55, 0.49)
    conveyor_end = (x + ox * (0.5 + length - 0.1), y + oy * (0.5 + length - 0.1), 0.49)
    return StationVisual(sid, pos, st_root, stack_anchor, beacon, color, conveyor_start, conveyor_end, tn,
                         progress_fill)


def make_carton(name: str, color=None, size: float = 0.36, label_color=None) -> NodePath:
    """A parcel carton with tape, label and an optional destination-coloured tag."""
    col = color or rgba("#c8935a")
    mb = MeshBuilder(name)
    mb.box(0, 0, 0, size, size, size * 0.8, col, top=shade(col, 1.07))
    mb.box(0, 0, size * 0.8, size * 0.16, size + 0.004, 0.004, rgba("#e7cfa3"))
    mb.box(0, 0, size * 0.8, size + 0.004, size * 0.16, 0.004, rgba("#e7cfa3"))
    mb.box(0, -size / 2 - 0.002, size * 0.3, size * 0.5, 0.004, size * 0.3, rgba("#f1ede4"))
    mb.box(size / 2 + 0.002, 0, size * 0.3, 0.004, size * 0.5, size * 0.3, rgba("#f1ede4"))
    if label_color is not None:
        mb.box(0, -size / 2 - 0.004, size * 0.55, size * 0.5, 0.004, size * 0.08, label_color)
        mb.box(size / 2 + 0.004, 0, size * 0.55, 0.004, size * 0.5, size * 0.08, label_color)
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
    np_.setDepthOffset(2)
    np_.setBin("fixed", 5)
    return np_
