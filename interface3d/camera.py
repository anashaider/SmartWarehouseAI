"""Cinematic camera system.

Modes: ``overview`` (slow cinematic orbit), ``topdown``, ``follow`` (third-person
robot tracking), ``free`` (mouse orbit / pan / zoom), ``focus`` (orbit around
the selected robot). Every mode produces a *goal* :class:`CameraRig`; the
actual rig eases toward it each frame, which gives smooth transitions between
modes and smooth tracking. The intro flythrough is a keyframed rig sequence.
"""

from __future__ import annotations

import math
from typing import Callable, Optional

from interface3d.sync import CameraRig, damping, smoothstep

MODES = ("overview", "topdown", "follow", "free", "focus")
MODE_LABELS = {
    "overview": "Cinematic Overview",
    "topdown": "Top-Down",
    "follow": "Robot Tracking",
    "free": "Free Camera",
    "focus": "Robot Focus",
}


class CameraController:
    def __init__(self, center: tuple[float, float], extent: float) -> None:
        self.center = center
        self.extent = extent
        self.mode = "overview"
        self.rig = self.overview_goal(0.0)
        self.free_rig = self.rig.copy()
        self.focus_offset = CameraRig(distance=6.5, heading=25.0, pitch=-38.0)
        self.orbit_angle = 0.0
        self.intro: Optional[list[tuple[float, CameraRig]]] = None
        self.intro_time = 0.0
        self.on_intro_done: Optional[Callable[[], None]] = None
        self.sharpness = 3.2

    # ------------------------------------------------------------------ goals
    def overview_goal(self, t: float) -> CameraRig:
        return CameraRig(self.center[0], self.center[1], 0.0, self.extent * 1.55 + 4,
                         heading=-24 + 16 * math.sin(t * 0.06), pitch=-50)

    def topdown_goal(self) -> CameraRig:
        return CameraRig(self.center[0], self.center[1] + 0.001, 0.0, self.extent * 1.62 + 5, heading=0.0, pitch=-89.0)

    @staticmethod
    def follow_goal(x: float, y: float, robot_heading: float) -> CameraRig:
        # behind the robot: camera heading = robot heading (camera south of target when heading 0)
        ahead = math.radians(robot_heading)
        tx = x - math.sin(ahead) * 1.2
        ty = y + math.cos(ahead) * 1.2
        return CameraRig(tx, ty, 0.35, 4.6, heading=robot_heading, pitch=-24)

    def focus_goal(self, x: float, y: float) -> CameraRig:
        o = self.focus_offset
        return CameraRig(x, y, 0.3, o.distance, o.heading, o.pitch)

    # ------------------------------------------------------------------ intro
    def start_intro(self, on_done: Optional[Callable[[], None]] = None) -> None:
        cx, cy, e = self.center[0], self.center[1], self.extent
        self.intro = [
            (0.0, CameraRig(cx, cy, 0, e * 2.4, heading=-160, pitch=-82)),
            (3.0, CameraRig(cx, cy, 0, e * 1.9, heading=-95, pitch=-70)),
            (6.0, CameraRig(cx - e * 0.25, cy + e * 0.1, 0.6, e * 0.75, heading=-50, pitch=-30)),
            (8.5, CameraRig(cx + e * 0.15, cy - e * 0.05, 0.4, e * 0.55, heading=-10, pitch=-26)),
            (11.0, self.overview_goal(0.0)),
        ]
        self.intro_time = 0.0
        self.rig = self.intro[0][1].copy()
        self.on_intro_done = on_done

    @property
    def in_intro(self) -> bool:
        return self.intro is not None

    def skip_intro(self) -> None:
        if self.intro is not None:
            self.intro = None
            if self.on_intro_done:
                self.on_intro_done()

    def _intro_rig(self) -> Optional[CameraRig]:
        assert self.intro is not None
        frames = self.intro
        if self.intro_time >= frames[-1][0]:
            return None
        for (t0, r0), (t1, r1) in zip(frames, frames[1:]):
            if t0 <= self.intro_time <= t1:
                k = smoothstep((self.intro_time - t0) / (t1 - t0))
                rig = r0.copy()
                rig.blend_toward(r1, k)
                return rig
        return frames[-1][1].copy()

    # ------------------------------------------------------------------ user input (free / focus)
    def orbit(self, dh: float, dp: float) -> None:
        if self.mode == "focus":
            self.focus_offset.heading += dh
            self.focus_offset.pitch = max(-85, min(-8, self.focus_offset.pitch + dp))
            return
        self._ensure_free()
        self.free_rig.heading += dh
        self.free_rig.pitch = max(-89, min(-5, self.free_rig.pitch + dp))

    def pan(self, dx: float, dy: float) -> None:
        self._ensure_free()
        h = math.radians(self.free_rig.heading)
        scale = self.free_rig.distance * 0.0016
        self.free_rig.tx += (math.cos(h) * dx - math.sin(h) * dy) * scale
        self.free_rig.ty += (math.sin(h) * dx + math.cos(h) * dy) * scale

    def zoom(self, factor: float) -> None:
        if self.mode == "focus":
            self.focus_offset.distance = max(2.0, min(30.0, self.focus_offset.distance * factor))
            return
        self._ensure_free()
        self.free_rig.distance = max(2.5, min(self.extent * 3.5, self.free_rig.distance * factor))

    def _ensure_free(self) -> None:
        if self.mode != "free":
            self.free_rig = self.rig.copy()
            self.mode = "free"

    def set_mode(self, mode: str) -> None:
        if mode not in MODES:
            raise ValueError(mode)
        if mode == "free":
            self.free_rig = self.rig.copy()
        self.mode = mode

    # ------------------------------------------------------------------ update
    def update(self, dt: float, t: float, robot_pose: Optional[tuple[float, float, float]],
               mode_override: Optional[str] = None) -> CameraRig:
        """Advance the rig. ``robot_pose`` = (x, y, heading) of the tracked robot."""
        if self.intro is not None:
            self.intro_time += dt
            rig = self._intro_rig()
            if rig is None:
                self.skip_intro()
            else:
                self.rig = rig
                return self.rig
        mode = mode_override or self.mode
        if mode in ("follow", "focus") and robot_pose is None:
            mode = "overview"
        if mode == "overview":
            goal = self.overview_goal(t)
        elif mode == "topdown":
            goal = self.topdown_goal()
        elif mode == "follow":
            goal = self.follow_goal(*robot_pose)  # type: ignore[misc]
        elif mode == "focus":
            goal = self.focus_goal(robot_pose[0], robot_pose[1])  # type: ignore[index]
        else:
            goal = self.free_rig
        sharp = self.sharpness * (1.6 if mode == "follow" else 1.0)
        self.rig.blend_toward(goal, damping(dt, sharp))
        return self.rig
