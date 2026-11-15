"""Procedural autonomous mobile robot (AMR) model and its animated parts.

The model faces +Y. Animated sub-nodes:

* ``wheels``  - rotate with the distance actually travelled;
* ``lidar``   - spins continuously (environmental animation);
* ``lift``    - top platform raised while the robot carries a parcel;
* ``cargo``   - carton on the lift, visible only while ``carrying`` is true;
* ``status``  - emissive beacon whose colour follows the robot state;
* ``ring``    - floor halo used for waiting / yielding / selection cues.
"""

from __future__ import annotations

from dataclasses import dataclass

from panda3d.core import Material, NodePath, TextNode, TransparencyAttrib

from interface3d.geometry import MeshBuilder, rgba, shade
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


def _mat(shininess: float, spec: float) -> Material:
    m = Material()
    m.setShininess(shininess)
    m.setSpecular((spec, spec, spec, 1))
    m.setAmbient((1, 1, 1, 1))
    return m


@dataclass
class RobotModel:
    robot_id: int
    root: NodePath
    body: NodePath
    wheels: list[NodePath]
    lidar: NodePath
    lift: NodePath
    cargo: NodePath
    status: NodePath
    halo: NodePath
    select_ring: NodePath
    accent: tuple


def build_robot(robot_id: int, parent: NodePath) -> RobotModel:
    accent = ROBOT_COLORS_3D[robot_id % len(ROBOT_COLORS_3D)]
    root = parent.attachNewNode(f"robot-{robot_id}")
    body = root.attachNewNode("body")

    hull = MeshBuilder("hull")
    white = rgba("#e5e9f0")
    dark = rgba("#1f2633")
    # chassis: lower dark skirt + white shell + accent band
    hull.box(0, 0, 0.05, 0.68, 0.72, 0.08, dark)
    hull.box(0, 0, 0.13, 0.66, 0.70, 0.09, white)                       # light lower band
    hull.box(0, 0, 0.22, 0.665, 0.705, 0.03, rgba("#111827"))           # shadow line
    hull.box(0, 0, 0.17, 0.655, 0.695, 0.075, accent, top=shade(accent, 1.1))  # accent shell
    # front bumper and sensor fascia
    hull.box(0, 0.355, 0.07, 0.6, 0.03, 0.08, rgba("#11151c"))
    hull.box(0, 0.343, 0.16, 0.42, 0.012, 0.09, rgba("#0b0f16"))  # ID panel backing
    # side covers over wheels
    for sx in (-1, 1):
        hull.box(sx * 0.335, 0, 0.06, 0.02, 0.5, 0.12, shade(dark, 1.2))
    # lidar mast
    hull.box(0, 0.22, 0.285, 0.1, 0.1, 0.06, dark)
    hull_np = body.attachNewNode(hull.node())
    hull_np.setMaterial(_mat(80, 0.6))

    # emissive headlights
    lights = MeshBuilder("headlights")
    for sx in (-1, 1):
        lights.box(sx * 0.24, 0.352, 0.13, 0.08, 0.012, 0.03, rgba("#f8fafc"))
    lnp = body.attachNewNode(lights.node())
    lnp.setLightOff(1)
    lnp.setShaderOff(1)
    # ID text on the front panel and on the top deck
    tn = TextNode(f"robot-id-{robot_id}")
    tn.setText(f"R{robot_id}")
    tn.setAlign(TextNode.ACenter)
    tn.setTextColor(accent[0] * 0.5 + 0.5, accent[1] * 0.5 + 0.5, accent[2] * 0.5 + 0.5, 1)
    front = body.attachNewNode(tn)
    front.setScale(0.075)
    front.setPos(0, 0.351, 0.183)
    front.setH(180)
    front.setLightOff(1)
    front.setShaderOff(1)

    # wheels (4) - rotate around local X
    wheels = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            wm = MeshBuilder("wheel")
            wm.cylinder((-0.03, 0, 0), 0.075, 0.06, rgba("#151922"), axis="x", segments=14,
                        cap_color=rgba("#5b6475"))
            w = body.attachNewNode(wm.node())
            w.setPos(sx * 0.3 + (0.0 if sx > 0 else 0.0), sy * 0.22, 0.075)
            w.setMaterial(_mat(10, 0.1))
            w.setTwoSided(True)
            wheels.append(w)

    # lidar head (spins)
    lm = MeshBuilder("lidar")
    lm.cylinder((0, 0, 0), 0.055, 0.05, rgba("#111827"), segments=16)
    lm.box(0, 0.045, 0.012, 0.05, 0.02, 0.025, rgba("#ef4444"))
    lidar = body.attachNewNode(lm.node())
    lidar.setPos(0, 0.22, 0.345)
    lidar.setMaterial(_mat(90, 0.8))

    # lift platform + cargo
    lift = body.attachNewNode("lift")
    lift.setZ(0.245)
    pm = MeshBuilder("platform")
    pm.box(0, -0.06, 0, 0.5, 0.42, 0.025, rgba("#4b5563"), top=rgba("#6b7280"))
    pl = lift.attachNewNode(pm.node())
    pl.setMaterial(_mat(60, 0.5))
    cargo = make_carton(f"cargo-{robot_id}", size=0.36)
    cargo.reparentTo(lift)
    cargo.setPos(0, -0.06, 0.025)
    cargo.hide()

    # status beacon (emissive)
    sm = MeshBuilder("status")
    sm.cylinder((0, 0, 0), 0.04, 0.07, (1, 1, 1, 1), segments=12)
    status = body.attachNewNode(sm.node())
    status.setPos(-0.24, -0.26, 0.245)
    status.setLightOff(1)
    status.setShaderOff(1)

    halo = make_marker_ring((1, 1, 1, 0.9), 0.46, 0.52)
    halo.reparentTo(root)
    halo.setZ(0.012)
    halo.hide()
    select_ring = make_marker_ring((1, 1, 1, 0.85), 0.55, 0.6)
    select_ring.reparentTo(root)
    select_ring.setZ(0.014)
    select_ring.hide()
    root.setTransparency(TransparencyAttrib.MNone)
    return RobotModel(robot_id, root, body, wheels, lidar, lift, cargo, status, halo, select_ring, accent)
