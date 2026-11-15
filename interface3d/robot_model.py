"""Procedural low-profile lifting AMR (Kiva-style), facing +Y.

The robot drives *under* the parcel stands and raises its round lift
platform to take the carton, which is how many warehouse AMRs work. Its
height at rest (0.30) is below the stand's underside (0.38).

Animated sub-nodes (all driven by :class:`interface3d.sync.RobotPose`):
* ``wheels_left`` / ``wheels_right`` - differential drive wheels (opposite spin when turning);
* ``lidar``    - spins inside the front sensor window;
* ``lift``     - platform height follows the real ``carrying`` flag;
* ``cargo``    - carton on the lift, visible exactly while carrying;
* ``status``   - 360-degree emissive LED band coloured by state;
* ``halo`` / ``select_ring`` / ``select_beam`` - identification and selection cues.
"""

from __future__ import annotations

from dataclasses import dataclass

from panda3d.core import Material, NodePath, TextNode, TransparencyAttrib

from interface3d.geometry import MeshBuilder, chamfered_rect, rgba, shade
from interface3d.warehouse_scene import make_carton, make_marker_ring

ROBOT_COLORS_3D = [rgba("#3b82f6"), rgba("#f97316"), rgba("#a855f7")]
STATE_LIGHT = {
    "IDLE": rgba("#94a3b8"),
    "NAVIGATING_TO_PARCEL": rgba("#38bdf8"),
    "PICKING_UP": rgba("#f59e0b"),
    "NAVIGATING_TO_DROPOFF": rgba("#3b82f6"),
    "DELIVERING": rgba("#22c55e"),
}
WAIT_LIGHT = rgba("#f59e0b")
YIELD_LIGHT = rgba("#22d3ee")

LIFT_REST_Z = 0.27       # platform base at rest (top at 0.30)
LIFT_TRAVEL = 0.13       # raised: carton bottom at 0.43 (stand top is 0.40)
LIFT_DISC_H = 0.03


def _mat(shininess: float, spec: float) -> Material:
    m = Material()
    m.setShininess(shininess)
    m.setSpecular((spec, spec, spec, 1))
    m.setAmbient((1, 1, 1, 1))
    return m


def _emissive(np_: NodePath) -> NodePath:
    np_.setLightOff(1)
    np_.setShaderOff(1)
    return np_


@dataclass
class RobotModel:
    robot_id: int
    root: NodePath
    body: NodePath
    wheels_left: list[NodePath]
    wheels_right: list[NodePath]
    lidar: NodePath
    lift: NodePath
    cargo: NodePath
    status: NodePath
    halo: NodePath
    select_ring: NodePath
    select_beam: NodePath
    accent: tuple

    @property
    def wheels(self) -> list[NodePath]:
        return self.wheels_left + self.wheels_right


def build_robot(robot_id: int, parent: NodePath, font=None) -> RobotModel:
    accent = ROBOT_COLORS_3D[robot_id % len(ROBOT_COLORS_3D)]
    root = parent.attachNewNode(f"robot-{robot_id}")
    body = root.attachNewNode("body")
    dark = rgba("#1b212c")
    rubber = rgba("#111418")
    deck = rgba("#3c4452")

    hull = MeshBuilder("hull")
    hull.prism(chamfered_rect(0.70, 0.74, 0.12), 0.035, 0.11, rubber)                          # bumper skirt
    hull.prism(chamfered_rect(0.66, 0.70, 0.11), 0.11, 0.24, accent, top=shade(accent, 1.08))  # shell
    hull.prism(chamfered_rect(0.60, 0.64, 0.10), 0.24, 0.27, deck)                             # top deck
    hull.box(0, 0.352, 0.13, 0.36, 0.012, 0.07, rgba("#05070a"))        # front sensor window
    hull.box(0, -0.352, 0.15, 0.26, 0.012, 0.05, shade(dark, 1.3))      # rear service panel
    for sx in (-1, 1):
        hull.box(sx * 0.331, 0.12, 0.15, 0.012, 0.22, 0.07, rgba("#f1f5f9"))   # side ID plates
    hull.cylinder((-0.2, -0.23, 0.27), 0.03, 0.025, rgba("#dc2626"), segments=12)   # e-stop
    hull.cylinder((-0.2, -0.23, 0.265), 0.04, 0.008, rgba("#facc15"), segments=12)
    hull_np = body.attachNewNode(hull.node())
    hull_np.setMaterial(_mat(70, 0.55))

    lights = MeshBuilder("lights")
    for sx in (-1, 1):
        lights.box(sx * 0.25, 0.356, 0.135, 0.07, 0.008, 0.025, rgba("#f8fafc"))   # headlights
        lights.box(sx * 0.25, -0.356, 0.14, 0.07, 0.008, 0.02, rgba("#ef4444"))   # tail lights
    _emissive(body.attachNewNode(lights.node()))

    led = MeshBuilder("led")
    led.prism(chamfered_rect(0.672, 0.712, 0.112), 0.105, 0.135, (1, 1, 1, 1))
    status = _emissive(body.attachNewNode(led.node()))

    def label(text: str, pos, h: float, scale: float, color) -> None:
        tn = TextNode(f"robot-label-{robot_id}")
        if font is not None:
            tn.setFont(font)
        tn.setText(text)
        tn.setAlign(TextNode.ACenter)
        tn.setTextColor(*color)
        np_ = _emissive(body.attachNewNode(tn))
        np_.setScale(scale)
        np_.setPos(*pos)
        np_.setH(h)

    light_accent = (accent[0] * 0.4 + 0.6, accent[1] * 0.4 + 0.6, accent[2] * 0.4 + 0.6, 1)
    label(f"R{robot_id}", (0, 0.36, 0.156), 180, 0.05, light_accent)
    label(f"R{robot_id}", (0.339, 0.12, 0.165), 90, 0.055, (0.07, 0.09, 0.12, 1))
    label(f"R{robot_id}", (-0.339, 0.12, 0.165), -90, 0.055, (0.07, 0.09, 0.12, 1))

    wheels_left: list[NodePath] = []
    wheels_right: list[NodePath] = []
    for sx, bucket in ((-1, wheels_left), (1, wheels_right)):
        wm = MeshBuilder("wheel")
        wm.cylinder((-0.025, 0, 0), 0.08, 0.05, rgba("#0d1014"), axis="x", segments=16, cap_color=rgba("#6b7280"))
        wm.box(0.0, 0.0, 0.05, 0.052, 0.02, 0.02, rgba("#9ca3af"))   # hub marker makes rotation visible
        w = body.attachNewNode(wm.node())
        w.setPos(sx * 0.30, 0.0, 0.08)
        w.setMaterial(_mat(10, 0.1))
        w.setTwoSided(True)
        bucket.append(w)
    casters = MeshBuilder("casters")
    for sx in (-1, 1):
        for sy in (-1, 1):
            casters.cylinder((sx * 0.22 - 0.015, sy * 0.25, 0.03), 0.03, 0.03, rgba("#20252e"), axis="x",
                             segments=10)
    body.attachNewNode(casters.node()).setMaterial(_mat(10, 0.1))

    lm = MeshBuilder("lidar")
    lm.cylinder((0, 0, 0), 0.035, 0.04, rgba("#1f2937"), segments=14)
    lm.box(0, 0.03, 0.012, 0.03, 0.012, 0.016, rgba("#ef4444"))
    lidar = body.attachNewNode(lm.node())
    lidar.setPos(0, 0.31, 0.145)
    lidar.setMaterial(_mat(90, 0.8))

    lift = body.attachNewNode("lift")
    lift.setZ(LIFT_REST_Z)
    pm = MeshBuilder("platform")
    pm.cylinder((0, 0, 0), 0.24, LIFT_DISC_H, rgba("#4b5563"), segments=28, cap_color=rgba("#6b7280"))
    pm.ring(0, 0, LIFT_DISC_H + 0.001, 0.17, 0.2, rgba("#9ca3af"))
    lift.attachNewNode(pm.node()).setMaterial(_mat(60, 0.5))
    cargo = make_carton(f"cargo-{robot_id}", size=0.36)
    cargo.reparentTo(lift)
    cargo.setPos(0, 0, LIFT_DISC_H)
    cargo.hide()

    halo = make_marker_ring((1, 1, 1, 0.9), 0.46, 0.52)
    halo.reparentTo(root)
    halo.setZ(0.012)
    select_ring = make_marker_ring((1, 1, 1, 0.85), 0.55, 0.61)
    select_ring.reparentTo(root)
    select_ring.setZ(0.014)
    select_ring.hide()
    bm = MeshBuilder("select-beam")
    bm.cylinder((0, 0, 0), 0.05, 1.6, (1, 1, 1, 0.35), segments=12, caps=False)
    select_beam = _emissive(root.attachNewNode(bm.node()))
    select_beam.setZ(0.7)
    select_beam.setTransparency(TransparencyAttrib.MAlpha)
    select_beam.setDepthWrite(False)
    select_beam.setTwoSided(True)
    select_beam.setBin("fixed", 6)
    select_beam.hide()
    return RobotModel(robot_id, root, body, wheels_left, wheels_right, lidar, lift, cargo, status, halo,
                      select_ring, select_beam, accent)
