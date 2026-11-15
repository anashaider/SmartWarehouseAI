"""Simulation -> visual state mapping for the 3D digital twin (no GPU code).

Everything the 3D renderer draws for robots and parcels is computed here from
the live :class:`SimulationEngine`, so it can be unit-tested without Panda3D:

* world coordinates of grid cells;
* robot poses interpolated between the cell occupied *before* and *after* the
  last tick (``previous_position`` -> ``position``), exactly like the 2D view;
* heading animation (turn in place first, then drive; retreats driven in reverse);
* lift height that follows the real ``carrying`` flag;
* status of each robot (state, waiting, yielding) for indicator colours;
* visual effects created only from real :class:`TickEvent` objects.

World frame: +X = increasing column, +Y = decreasing row (row 0 is the far
side of the warehouse), +Z = up. One grid cell = ``CELL`` world units.
Panda3D heading H: 0 faces +Y, 90 faces -X, 180 faces -Y, -90 faces +X.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from environment.grid import Action, Position
from environment.parcel import ParcelStatus
from environment.robot import RobotState
from simulation.simulation_engine import SimulationEngine, TickEvent

CELL = 1.0

ACTION_HEADING: dict[Action, float] = {
    Action.UP: 0.0,
    Action.LEFT: 90.0,
    Action.DOWN: 180.0,
    Action.RIGHT: -90.0,
}

#: Fraction of a tick's animation used to turn in place before driving.
TURN_PHASE = 0.35
LIFT_SPEED = 2.2          # lift height units per second (0..1)
EFFECT_DURATION = {"pickup": 1.2, "deliver": 1.6, "conflict": 0.9, "invalid": 0.9,
                   "yield": 1.4, "assign": 1.0, "wait": 0.6}


def world_xy(cell: tuple[float, float], height: int) -> tuple[float, float]:
    """World (x, y) of the centre of grid ``cell`` (fractional cells allowed)."""
    row, col = cell
    return (col * CELL, (height - 1 - row) * CELL)


def heading_between(a: Position, b: Position) -> Optional[float]:
    """Heading that faces from cell ``a`` toward the adjacent cell ``b``."""
    dr, dc = b[0] - a[0], b[1] - a[1]
    if (dr, dc) == (0, 0):
        return None
    if dr < 0:
        return ACTION_HEADING[Action.UP]
    if dr > 0:
        return ACTION_HEADING[Action.DOWN]
    return ACTION_HEADING[Action.LEFT] if dc < 0 else ACTION_HEADING[Action.RIGHT]


def angle_diff(target: float, current: float) -> float:
    """Signed shortest rotation from ``current`` to ``target`` in degrees (-180, 180]."""
    d = (target - current) % 360.0
    return d - 360.0 if d > 180.0 else d


def smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


WHEEL_RADIUS = 0.08       # drive wheel radius (world units)
WHEEL_TRACK = 0.30        # half distance between the two drive wheels
CRUISE_SPEED = 3.0        # cells / second used to normalise the speed indicator


def motion_profile(t: float, ease_in: bool, ease_out: bool) -> float:
    """Fraction of the cell-to-cell distance covered at normalised time ``t``.

    Always 0 at t=0 and 1 at t=1 (so the robot is exactly on the simulated
    cells at both ends of a tick). Velocity is 0 at an eased end and equals the
    cruise velocity (slope 1) at a non-eased end, so consecutive straight moves
    join without stopping:
        both eased  -> smoothstep (accelerate + decelerate)
        in only     -> t^2 (2 - t)          (accelerate, leave at cruise speed)
        out only    -> 1 - (1-t)^2 (1+t)    (arrive at cruise speed, decelerate)
        neither     -> t                    (cruise)
    """
    t = max(0.0, min(1.0, t))
    if ease_in and ease_out:
        return t * t * (3 - 2 * t)
    if ease_in:
        return t * t * (2 - t)
    if ease_out:
        return 1 - (1 - t) * (1 - t) * (1 + t)
    return t


def continues_straight(plan: list, a: Position, b: Position) -> bool:
    """True when the robot's own plan continues in the direction a -> b after b."""
    if b == a or not plan:
        return False
    try:
        i = plan.index(b)
    except ValueError:
        return False
    if i + 1 >= len(plan):
        return False
    c = plan[i + 1]
    return (c[0] - b[0], c[1] - b[1]) == (b[0] - a[0], b[1] - a[1])


@dataclass
class RobotPose:
    robot_id: int
    x: float
    y: float
    heading: float
    lift: float                 # 0 (down) .. 1 (raised, carrying)
    wheel_angle: float          # mean accumulated drive-wheel rotation (degrees)
    carrying: bool
    moving: bool
    reversing: bool
    state: RobotState
    waiting: bool
    yielding: bool
    cell: Position
    wheel_left: float = 0.0     # left / right drive wheels (differential drive)
    wheel_right: float = 0.0
    turning: bool = False
    speed: float = 0.0          # normalised ground speed (0 = stopped, 1 = cruise)


@dataclass
class VisualEffect:
    kind: str
    robot_id: int
    x: float
    y: float
    start: float
    duration: float
    message: str
    step: int

    def progress(self, now: float) -> float:
        return (now - self.start) / self.duration if self.duration > 0 else 1.0


@dataclass
class _RobotAnim:
    heading: float = 180.0
    tick_start_heading: float = 180.0
    lift: float = 0.0
    wheel: float = 0.0
    last_xy: Optional[tuple[float, float]] = None
    last_heading: float = 180.0
    anim_key: Optional[tuple] = None
    reversing: bool = False
    wheel_l: float = 0.0
    wheel_r: float = 0.0
    ease_in: bool = True
    ease_out: bool = True
    prev_travel: Optional[float] = None
    prev_step: int = -10


@dataclass
class ParcelView:
    parcel_id: int
    status: ParcelStatus
    x: float
    y: float
    assigned_robot: Optional[int]
    destination_id: int
    visible_on_floor: bool


class VisualSync:
    """Maintains animation state derived from the engine (one per 3D app)."""

    def __init__(self) -> None:
        self.robots: dict[int, _RobotAnim] = {}
        self.effects: list[VisualEffect] = []
        self.engine_id: Optional[int] = None
        self.history: list[TickEvent] = []   # recent real events (for the presentation director)

    def reset(self) -> None:
        self.robots.clear()
        self.effects.clear()
        self.history.clear()
        self.engine_id = None

    # ------------------------------------------------------------------ #
    def ingest_events(self, engine: SimulationEngine, events: list[TickEvent], now: float) -> list[VisualEffect]:
        """Create visual effects for real engine events (one effect per event)."""
        h = engine.warehouse.grid.height
        created = []
        for ev in events:
            if ev.kind not in EFFECT_DURATION:
                continue
            x, y = world_xy(ev.position, h)
            fx = VisualEffect(ev.kind, ev.robot_id, x, y, now, EFFECT_DURATION[ev.kind], ev.message, ev.step)
            created.append(fx)
            self.history.append(ev)
        self.effects.extend(created)
        self.history = self.history[-200:]
        return created

    def live_effects(self, now: float) -> list[VisualEffect]:
        self.effects = [e for e in self.effects if e.progress(now) < 1.0]
        return self.effects

    # ------------------------------------------------------------------ #
    def robot_poses(self, engine: SimulationEngine, progress: float, dt: float) -> dict[int, RobotPose]:
        """Poses for every robot.

        ``progress`` is the animation progress of the last tick (0..1). At
        ``progress == 1`` every pose sits exactly on the robot's simulated cell.
        """
        if self.engine_id != id(engine) or len(self.robots) != len(engine.warehouse.robots):
            self.robots = {}
            self.engine_id = id(engine)
        h = engine.warehouse.grid.height
        poses: dict[int, RobotPose] = {}
        for robot in engine.warehouse.robots:
            anim = self.robots.get(robot.robot_id)
            if anim is None:
                anim = _RobotAnim(heading=ACTION_HEADING.get(robot.direction, 180.0))
                anim.tick_start_heading = anim.heading
                anim.lift = 1.0 if robot.is_carrying else 0.0
                self.robots[robot.robot_id] = anim
            a, b = robot.previous_position, robot.position
            key = (engine.step_count, a, b)
            travel = heading_between(a, b)
            if key != anim.anim_key:
                # A new tick began: decide how this move is animated.
                anim.anim_key = key
                anim.tick_start_heading = anim.heading
                anim.reversing = (
                    travel is not None and robot.is_yielding
                    and abs(abs(angle_diff(travel, anim.heading)) - 180.0) < 1.0
                )
                # Acceleration profile: start from rest unless the robot was already
                # cruising in this direction during the previous tick; stop at the end
                # unless its own plan continues straight on.
                cruising = (anim.prev_travel == travel and anim.prev_step == engine.step_count - 1
                            and travel is not None)
                anim.ease_in = not cruising
                anim.ease_out = not continues_straight(robot.planned_path, a, b)
                anim.prev_travel = travel
                anim.prev_step = engine.step_count
            p = max(0.0, min(1.0, progress))
            turning = False
            if travel is None:
                move_t = 1.0
                heading = anim.heading
            else:
                desired = (travel + 180.0) % 360.0 if anim.reversing else travel
                turn = angle_diff(desired, anim.tick_start_heading)
                if abs(turn) > 1.0:
                    # rotate in place first, then drive (always from rest)
                    turn_t = smoothstep(p / TURN_PHASE)
                    heading = anim.tick_start_heading + turn * turn_t
                    turning = turn_t < 1.0
                    move_t = motion_profile((p - TURN_PHASE) / (1.0 - TURN_PHASE), True, anim.ease_out)
                else:
                    heading = desired
                    move_t = motion_profile(p, anim.ease_in, anim.ease_out)
            anim.heading = heading % 360.0
            cell = (a[0] + (b[0] - a[0]) * move_t, a[1] + (b[1] - a[1]) * move_t)
            x, y = world_xy(cell, h)
            speed = 0.0
            if anim.last_xy is not None:
                dist = math.hypot(x - anim.last_xy[0], y - anim.last_xy[1])
                signed = (-1 if anim.reversing else 1) * dist / WHEEL_RADIUS * (180.0 / math.pi)
                dturn = math.radians(angle_diff(anim.heading, anim.last_heading))
                spin = WHEEL_TRACK * dturn / WHEEL_RADIUS * (180.0 / math.pi)
                anim.wheel_l += signed - spin      # differential drive: turning left
                anim.wheel_r += signed + spin      # spins the wheels in opposite directions
                anim.wheel = (anim.wheel_l + anim.wheel_r) / 2
                if dt > 0:
                    speed = min(1.0, dist / dt / CRUISE_SPEED)
            anim.last_xy = (x, y)
            anim.last_heading = anim.heading
            target_lift = 1.0 if robot.is_carrying else 0.0
            step = LIFT_SPEED * max(0.0, dt)
            anim.lift = min(target_lift, anim.lift + step) if anim.lift < target_lift else                 max(target_lift, anim.lift - step)
            poses[robot.robot_id] = RobotPose(
                robot_id=robot.robot_id, x=x, y=y, heading=anim.heading, lift=anim.lift,
                wheel_angle=anim.wheel % 360.0, carrying=robot.is_carrying,
                moving=travel is not None and move_t < 1.0, reversing=anim.reversing,
                state=robot.state,
                waiting=robot.current_action == Action.WAIT and robot.has_task and not robot.is_yielding,
                yielding=robot.is_yielding, cell=robot.position,
                wheel_left=anim.wheel_l % 360.0, wheel_right=anim.wheel_r % 360.0,
                turning=turning, speed=speed,
            )
        return poses

    # ------------------------------------------------------------------ #
    @staticmethod
    def parcel_views(engine: SimulationEngine) -> list[ParcelView]:
        h = engine.warehouse.grid.height
        out = []
        for p in engine.warehouse.parcels.values():
            x, y = world_xy(p.origin, h)
            out.append(ParcelView(p.parcel_id, p.status, x, y, p.assigned_robot, p.destination_id,
                                  p.status in (ParcelStatus.WAITING, ParcelStatus.ASSIGNED)))
        return out

    @staticmethod
    def delivered_per_station(engine: SimulationEngine) -> dict[int, list[int]]:
        out: dict[int, list[int]] = {sid: [] for sid in engine.warehouse.stations}
        for p in sorted(engine.warehouse.parcels.values(), key=lambda p: p.delivered_step or 0):
            if p.status == ParcelStatus.DELIVERED:
                out[p.destination_id].append(p.parcel_id)
        return out


# --------------------------------------------------------------------------- #
# Camera rig maths (pure, testable)
# --------------------------------------------------------------------------- #
@dataclass
class CameraRig:
    """Orbit rig: the camera looks at ``target`` from a sphere of ``distance``."""

    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0
    distance: float = 30.0
    heading: float = 0.0   # degrees, 0 = camera south of target looking north
    pitch: float = -50.0   # degrees, negative = looking down

    def camera_position(self) -> tuple[float, float, float]:
        h, p = math.radians(self.heading), math.radians(self.pitch)
        horiz = self.distance * math.cos(p)
        return (self.tx + horiz * math.sin(h),
                self.ty - horiz * math.cos(h),
                self.tz - self.distance * math.sin(p))

    def blend_toward(self, goal: "CameraRig", amount: float) -> None:
        """Move a fraction ``amount`` (0..1) of the way to ``goal`` (shortest heading arc)."""
        a = max(0.0, min(1.0, amount))
        self.tx += (goal.tx - self.tx) * a
        self.ty += (goal.ty - self.ty) * a
        self.tz += (goal.tz - self.tz) * a
        self.distance += (goal.distance - self.distance) * a
        self.heading += angle_diff(goal.heading, self.heading) * a
        self.pitch += (goal.pitch - self.pitch) * a

    def copy(self) -> "CameraRig":
        return CameraRig(self.tx, self.ty, self.tz, self.distance, self.heading, self.pitch)


def damping(dt: float, sharpness: float) -> float:
    """Frame-rate independent exponential smoothing factor."""
    return 1.0 - math.exp(-sharpness * max(0.0, dt))


# --------------------------------------------------------------------------- #
# Presentation director (pure, testable)
# --------------------------------------------------------------------------- #
@dataclass
class Shot:
    mode: str                     # overview | topdown | follow | focus
    robot_id: Optional[int]
    caption: str
    subcaption: str = ""
    duration: float = 6.0
    show_search: bool = False
    reason: str = "schedule"      # schedule | event
    #: Cinematic framing: grid cells that must stay in view (real positions).
    points: list = field(default_factory=list)
    pitch: float = -40.0
    orbit: float = 0.0            # degrees / second of slow orbit around the framed points
    station_id: Optional[int] = None
    kind: str = ""                # overview | tracking | follow | search | stats | pickup | deliver
    serial: int = 0               # unique per shot (camera transitions on change)


@dataclass
class PresentationDirector:
    """Chooses camera shots for Presentation Mode from real simulation events.

    A fixed rotation (overview -> robot tracking -> pathfinding -> top-down
    statistics) is interrupted whenever a real pickup or delivery event
    occurs, so highlighted sequences always show something that actually
    happened in the simulation.
    """

    shot: Optional[Shot] = None
    elapsed: float = 0.0
    rotation_index: int = 0
    seen_events: int = 0
    last_highlight_step: int = -999
    log: list[Shot] = field(default_factory=list)
    _serial: int = 0

    def start(self) -> None:
        self.shot = None
        self.elapsed = 0.0
        self.rotation_index = 0

    def _scheduled(self, engine: SimulationEngine) -> Shot:
        robots = engine.warehouse.robots
        busy = [r for r in robots if r.has_task] or robots
        rid = busy[self.rotation_index % len(busy)].robot_id
        m = engine.metrics()
        kind = ["overview", "tracking", "follow", "search", "stats"][self.rotation_index % 5]
        self.rotation_index += 1
        r = next(x for x in robots if x.robot_id == rid)
        what = (f"carrying P{r.carrying} to its dock" if r.carrying is not None else
                f"heading to parcel P{r.assigned_parcel}" if r.assigned_parcel is not None else "idle")
        if kind == "overview":
            return Shot("overview", None, "Smart Warehouse Digital Twin",
                        f"{len(robots)} autonomous robots · {m['parcels_delivered']}/{m['parcels_total']} parcels delivered",
                        7.0, kind="overview")
        if kind == "tracking":
            return Shot("tracking", rid, f"Robot R{rid} · {r.state.label}", f"Tracking shot - {what}", 6.0,
                        kind="tracking")
        if kind == "follow":
            return Shot("follow", rid, f"Robot R{rid} · third-person", f"Chase camera - {what}", 6.0, kind="follow")
        if kind == "search":
            target = engine.warehouse.parcel_target(r)
            route = list(r.planned_path) + ([target] if target else []) or [r.position]
            return Shot("focus", rid, f"Pathfinding · R{rid}",
                        "BFS (blue) vs A* (orange) explored cells for the robot's real goal; the dashed line is "
                        "its planned route", 8.0, show_search=True, points=route, pitch=-62.0, orbit=3.0,
                        kind="search")
        return Shot("topdown", None, "Performance",
                    f"Step {m['steps']} · distance {m['total_distance']} · conflicts {m['conflicts']} · "
                    f"completion {m['completion_pct']:.0f}%", 6.0, kind="stats")

    def update(self, engine: SimulationEngine, events: list[TickEvent], dt: float) -> Shot:
        self.elapsed += dt
        highlight = None
        for ev in events:
            if ev.kind in ("pickup", "deliver") and ev.step - self.last_highlight_step >= 6:
                verb = "picked up" if ev.kind == "pickup" else "delivered"
                station = None
                if ev.kind == "deliver":
                    station = next((st.station_id for st in engine.warehouse.stations.values()
                                    if st.position == ev.position), None)
                highlight = Shot("focus", ev.robot_id, f"R{ev.robot_id} {verb} a parcel",
                                 f"{ev.message} · step {ev.step}", 4.5, reason="event",
                                 points=[ev.position], pitch=-30.0, orbit=9.0, station_id=station,
                                 kind=ev.kind)
                self.last_highlight_step = ev.step
        if highlight is not None and (self.shot is None or self.shot.reason != "event"):
            self.shot, self.elapsed = highlight, 0.0
            self._serial += 1
            highlight.serial = self._serial
            self.log.append(highlight)
        elif self.shot is None or self.elapsed >= self.shot.duration:
            self.shot, self.elapsed = self._scheduled(engine), 0.0
            self._serial += 1
            self.shot.serial = self._serial
            self.log.append(self.shot)
        self.log = self.log[-50:]
        return self.shot
