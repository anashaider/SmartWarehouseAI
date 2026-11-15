"""Central warehouse visualisation.

Everything drawn here is read from the live :class:`SimulationEngine`:
robot positions (interpolated between the cells occupied before and after
the last tick), planned paths returned by the agents, reservations made by the
conflict manager, and pickup/delivery/conflict events emitted by the engine.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import pygame

from algorithms.common import SearchResult
from environment.grid import Action, Position
from environment.parcel import ParcelStatus
from environment.robot import Robot
from interface import theme as T
from simulation.simulation_engine import SimulationEngine, TickEvent


@dataclass
class Effect:
    kind: str
    cell: Position
    start: float
    color: T.Color
    text: str = ""
    duration: float = 0.8


def ease(t: float) -> float:
    return t * t * (3 - 2 * t)  # smoothstep


class WarehouseView:
    def __init__(self, rect: pygame.Rect) -> None:
        self.rect = pygame.Rect(rect)
        self.cell = 20
        self.origin = (0, 0)
        self._static: Optional[pygame.Surface] = None
        self._static_key: tuple = ()
        self.effects: list[Effect] = []
        self.show_paths = True
        self.show_reservations = True
        self.comparison: Optional[dict[str, SearchResult]] = None
        self.hover_cell: Optional[Position] = None

    # ------------------------------------------------------------------ #
    def set_rect(self, rect: pygame.Rect) -> None:
        self.rect = pygame.Rect(rect)
        self._static_key = ()

    def _layout(self, engine: SimulationEngine) -> None:
        grid = engine.warehouse.grid
        pad = 16
        avail_w, avail_h = self.rect.width - 2 * pad, self.rect.height - 2 * pad - 22
        self.cell = max(8, min(avail_w // grid.width, avail_h // grid.height, 56))
        w, h = self.cell * grid.width, self.cell * grid.height
        self.origin = (self.rect.x + (self.rect.width - w) // 2,
                       self.rect.y + 22 + (self.rect.height - 22 - h) // 2)

    def cell_rect(self, cell: Position) -> pygame.Rect:
        return pygame.Rect(self.origin[0] + cell[1] * self.cell, self.origin[1] + cell[0] * self.cell,
                           self.cell, self.cell)

    def cell_center(self, cell: tuple[float, float]) -> tuple[float, float]:
        return (self.origin[0] + (cell[1] + 0.5) * self.cell, self.origin[1] + (cell[0] + 0.5) * self.cell)

    def cell_at(self, pos: tuple[int, int], engine: SimulationEngine) -> Optional[Position]:
        grid = engine.warehouse.grid
        c = (pos[0] - self.origin[0]) // self.cell
        r = (pos[1] - self.origin[1]) // self.cell
        if 0 <= r < grid.height and 0 <= c < grid.width:
            return (int(r), int(c))
        return None

    def robot_at_point(self, pos: tuple[int, int], engine: SimulationEngine) -> Optional[int]:
        cell = self.cell_at(pos, engine)
        if cell is None:
            return None
        robot = engine.warehouse.robot_at(cell)
        return robot.robot_id if robot else None

    # ------------------------------------------------------------------ #
    def add_events(self, events: list[TickEvent]) -> None:
        now = time.perf_counter()
        for ev in events:
            if ev.kind == "pickup":
                self.effects.append(Effect("ring", ev.position, now, T.WARN, "PICK", 0.9))
            elif ev.kind == "deliver":
                self.effects.append(Effect("burst", ev.position, now, T.GOOD, "+1", 1.2))
            elif ev.kind in ("conflict", "invalid"):
                self.effects.append(Effect("cross", ev.position, now, T.BAD, "", 0.7))
            elif ev.kind == "yield":
                self.effects.append(Effect("ring", ev.position, now, T.INFO, "YIELD", 1.0))
        self.effects = self.effects[-60:]

    # ------------------------------------------------------------------ #
    def _build_static(self, engine: SimulationEngine) -> pygame.Surface:
        wh = engine.warehouse
        grid = wh.grid
        c = self.cell
        surf = pygame.Surface((grid.width * c, grid.height * c))
        surf.fill(T.FLOOR)
        for r in range(grid.height):
            for col in range(grid.width):
                rect = pygame.Rect(col * c, r * c, c, c)
                if grid.is_shelf((r, col)):
                    pygame.draw.rect(surf, T.SHELF_EDGE, rect)
                    inner = rect.inflate(-2, -2)
                    pygame.draw.rect(surf, T.SHELF, inner, border_radius=max(2, c // 8))
                    # rack levels
                    for k in (1, 2):
                        y = inner.y + inner.height * k // 3
                        pygame.draw.line(surf, T.SHELF_TOP, (inner.x + 2, y), (inner.right - 3, y))
                else:
                    if (r + col) % 2:
                        pygame.draw.rect(surf, T.FLOOR_ALT, rect)
                    pygame.draw.rect(surf, T.GRID_LINE, rect, 1)
        for start in engine.scenario.robot_starts[: len(wh.robots)]:
            rect = pygame.Rect(start[1] * c, start[0] * c, c, c).inflate(-c // 3, -c // 3)
            pygame.draw.rect(surf, T.BORDER, rect, 1, border_radius=4)
        for station in wh.stations.values():
            r, col = station.position
            rect = pygame.Rect(col * c, r * c, c, c).inflate(-2, -2)
            pygame.draw.rect(surf, T.STATION_DARK, rect, border_radius=max(3, c // 6))
            pygame.draw.rect(surf, T.STATION, rect, 2, border_radius=max(3, c // 6))
            stripe = rect.inflate(-c // 3, -c // 3)
            for k in range(0, stripe.width, max(4, c // 5)):
                pygame.draw.line(surf, T.STATION, (stripe.x + k, stripe.bottom), (stripe.x + k + c // 6, stripe.bottom - c // 6), 2)
            if c >= 18:
                T.draw_text(surf, station.label, (rect.centerx, rect.y + rect.height * 0.38), max(9, c // 3),
                            (220, 255, 230), bold=True, anchor="center")
        return surf

    # ------------------------------------------------------------------ #
    def draw(self, surface: pygame.Surface, engine: SimulationEngine, progress: float,
             selected: Optional[int]) -> None:
        self._layout(engine)
        T.rounded_rect(surface, T.PANEL, self.rect, 12, 1, T.BORDER)
        key = (id(engine.scenario), self.cell, len(engine.warehouse.robots), self.rect.size)
        if self._static is None or self._static_key != key:
            self._static = self._build_static(engine)
            self._static_key = key
        sc = engine.scenario
        T.draw_text(surface, f"{sc.name}  ·  {sc.width}×{sc.height} grid  ·  "
                    f"{len(engine.warehouse.parcels)} parcels  ·  {len(engine.warehouse.stations)} stations",
                    (self.rect.x + 16, self.rect.y + 10), 12, T.TEXT_SECONDARY)
        surface.blit(self._static, self.origin)

        overlay = pygame.Surface(self._static.get_size(), pygame.SRCALPHA)
        ox, oy = self.origin
        wh = engine.warehouse
        c = self.cell

        def local(cell: tuple[float, float]) -> tuple[float, float]:
            x, y = self.cell_center(cell)
            return (x - ox, y - oy)

        # Comparison overlay: explored regions and both paths.
        if self.comparison:
            bfs, ast = self.comparison.get("bfs"), self.comparison.get("astar")
            if bfs:
                for cell in bfs.explored:
                    pygame.draw.rect(overlay, (*T.BFS_COLOR, 45), pygame.Rect(cell[1] * c, cell[0] * c, c, c))
            if ast:
                for cell in ast.explored:
                    pygame.draw.rect(overlay, (*T.ASTAR_COLOR, 120),
                                     pygame.Rect(cell[1] * c, cell[0] * c, c, c).inflate(-c // 2, -c // 2),
                                     border_radius=2)
            for res, col, off in ((bfs, T.BFS_COLOR, -max(2, c // 8)), (ast, T.ASTAR_COLOR, max(2, c // 8))):
                if res and len(res.path) > 1:
                    pts = [(local(p)[0] + off, local(p)[1] + off) for p in res.path]
                    pygame.draw.lines(overlay, (*col, 235), False, pts, max(2, c // 9))

        # Occupied cells and this tick's reservations.
        for robot in wh.robots:
            col = T.robot_color(robot.robot_id)
            pygame.draw.rect(overlay, (*col, 38), pygame.Rect(robot.position[1] * c, robot.position[0] * c, c, c))
        if self.show_reservations and c >= 14:
            for cell, rid in engine.conflicts.reservations.vertex.items():
                col = T.robot_color(rid)
                rect = pygame.Rect(cell[1] * c, cell[0] * c, c, c).inflate(-3, -3)
                L = max(3, c // 5)
                for (x, y, dx, dy) in ((rect.x, rect.y, 1, 1), (rect.right, rect.y, -1, 1),
                                       (rect.x, rect.bottom, 1, -1), (rect.right, rect.bottom, -1, -1)):
                    pygame.draw.line(overlay, (*col, 200), (x, y), (x + dx * L, y), 2)
                    pygame.draw.line(overlay, (*col, 200), (x, y), (x, y + dy * L), 2)

        # Planned paths and destinations.
        now = time.perf_counter()
        if self.show_paths:
            for robot in wh.robots:
                col = T.robot_color(robot.robot_id)
                path = robot.planned_path
                alpha = 210 if robot.robot_id == selected or selected is None else 110
                if len(path) > 1:
                    pts = [local(p) for p in path]
                    pygame.draw.lines(overlay, (*col, alpha), False, pts, max(2, c // 10))
                    for p in pts[1:-1]:
                        pygame.draw.circle(overlay, (*col, alpha), p, max(2, c // 10))
                if robot.destination is not None:
                    cx, cy = local(robot.destination)
                    pulse = 0.5 + 0.5 * math.sin(now * 4 + robot.robot_id)
                    rad = c * (0.36 + 0.06 * pulse)
                    pygame.draw.circle(overlay, (*col, alpha), (cx, cy), rad, 2)
                    pygame.draw.line(overlay, (*col, alpha), (cx - rad, cy), (cx - rad / 2, cy), 2)
                    pygame.draw.line(overlay, (*col, alpha), (cx + rad / 2, cy), (cx + rad, cy), 2)
                    pygame.draw.line(overlay, (*col, alpha), (cx, cy - rad), (cx, cy - rad / 2), 2)
                    pygame.draw.line(overlay, (*col, alpha), (cx, cy + rad / 2), (cx, cy + rad), 2)
        surface.blit(overlay, self.origin)

        # Parcels waiting / assigned.
        for parcel in wh.parcels.values():
            if parcel.status in (ParcelStatus.WAITING, ParcelStatus.ASSIGNED):
                outline = T.robot_color(parcel.assigned_robot) if parcel.assigned_robot is not None else None
                self._draw_parcel(surface, self.cell_center(parcel.position), c * 0.56, outline, parcel.label)

        # Delivered counters on stations.
        delivered: dict[int, int] = {}
        for parcel in wh.parcels.values():
            if parcel.status == ParcelStatus.DELIVERED:
                delivered[parcel.destination_id] = delivered.get(parcel.destination_id, 0) + 1
        for sid, n in delivered.items():
            r = self.cell_rect(wh.stations[sid].position)
            badge = pygame.Rect(0, 0, max(14, c // 2), max(12, c // 3))
            badge.bottomright = (r.right - 1, r.bottom - 1)
            T.rounded_rect(surface, T.GOOD, badge, 5)
            T.draw_text(surface, str(n), badge.center, max(9, c // 4), (5, 30, 15), bold=True, anchor="center")

        # Robots.
        p = ease(progress)
        for robot in sorted(wh.robots, key=lambda r: r.robot_id == selected):
            a, b = robot.previous_position, robot.position
            pos = (a[0] + (b[0] - a[0]) * p, a[1] + (b[1] - a[1]) * p)
            self._draw_robot(surface, robot, self.cell_center(pos), robot.robot_id == selected, now)

        self._draw_effects(surface, now)

        # Hover cell outline.
        mouse = T.mouse_pos()
        self.hover_cell = self.cell_at(mouse, engine) if self.rect.collidepoint(mouse) else None
        if self.hover_cell is not None:
            pygame.draw.rect(surface, T.TEXT_MUTED, self.cell_rect(self.hover_cell), 1)

    # ------------------------------------------------------------------ #
    def _draw_parcel(self, surface: pygame.Surface, center: tuple[float, float], size: float,
                     outline: Optional[T.Color], label: str = "") -> None:
        s = int(size)
        rect = pygame.Rect(0, 0, s, s)
        rect.center = (int(center[0]), int(center[1]))
        if outline:
            pygame.draw.rect(surface, outline, rect.inflate(6, 6), 2, border_radius=5)
        pygame.draw.rect(surface, T.PARCEL_DARK, rect.move(0, 2), border_radius=3)
        pygame.draw.rect(surface, T.PARCEL, rect, border_radius=3)
        pygame.draw.line(surface, T.PARCEL_TAPE, (rect.centerx, rect.y + 1), (rect.centerx, rect.bottom - 2), max(2, s // 6))
        pygame.draw.line(surface, T.PARCEL_DARK, (rect.x + 2, rect.y + s // 3), (rect.right - 3, rect.y + s // 3), 1)
        if label and self.cell >= 30:
            T.draw_text(surface, label, (rect.centerx, rect.bottom + 1), 9, T.TEXT_SECONDARY, anchor="midtop")

    def _draw_robot(self, surface: pygame.Surface, robot: Robot, center: tuple[float, float],
                    selected: bool, now: float) -> None:
        c = self.cell
        col = T.robot_color(robot.robot_id)
        size = int(c * 0.78)
        body = pygame.Rect(0, 0, size, size)
        body.center = (int(center[0]), int(center[1]))
        if selected:
            glow = 0.5 + 0.5 * math.sin(now * 5)
            T.alpha_circle(surface, col, body.center, size * 0.82, int(70 + 60 * glow))
            pygame.draw.rect(surface, (255, 255, 255), body.inflate(6, 6), 2, border_radius=size // 3)
        pygame.draw.rect(surface, T.mix(col, (0, 0, 0), 0.45), body.move(0, 2), border_radius=size // 3)
        pygame.draw.rect(surface, col, body, border_radius=size // 3)
        screen = body.inflate(-size // 3, -size // 3)
        pygame.draw.rect(surface, T.mix(col, (0, 0, 0), 0.6), screen, border_radius=max(2, size // 6))
        T.draw_text(surface, str(robot.robot_id), screen.center, max(9, int(size * 0.42)),
                    (255, 255, 255), bold=True, anchor="center")
        # Heading notch.
        dr, dc = {Action.UP: (-1, 0), Action.DOWN: (1, 0), Action.LEFT: (0, -1), Action.RIGHT: (0, 1)}.get(
            robot.direction, (1, 0))
        tip = (body.centerx + dc * size * 0.62, body.centery + dr * size * 0.62)
        base = (body.centerx + dc * size * 0.44, body.centery + dr * size * 0.44)
        perp = (-dr * size * 0.16, dc * size * 0.16)
        pygame.draw.polygon(surface, (255, 255, 255),
                            [tip, (base[0] + perp[0], base[1] + perp[1]), (base[0] - perp[0], base[1] - perp[1])])
        if robot.is_carrying:
            self._draw_parcel(surface, (body.right - size * 0.12, body.y + size * 0.12), size * 0.42, None)
        if robot.is_yielding:
            T.alpha_circle(surface, T.INFO, body.center, size * 0.78, 220, 2)
        elif robot.current_action == Action.WAIT and robot.has_task:
            pulse = 0.5 + 0.5 * math.sin(now * 6)
            badge = pygame.Rect(0, 0, max(10, size // 3), max(10, size // 3))
            badge.center = (body.x + 2, body.y + 2)
            T.alpha_circle(surface, T.WARN, badge.center, badge.width * 0.65, int(150 + 100 * pulse))
            bw = max(1, badge.width // 6)
            pygame.draw.rect(surface, (40, 25, 0), (badge.centerx - 2 * bw, badge.y + 3, bw, badge.height - 6))
            pygame.draw.rect(surface, (40, 25, 0), (badge.centerx + bw, badge.y + 3, bw, badge.height - 6))

    def _draw_effects(self, surface: pygame.Surface, now: float) -> None:
        alive = []
        for fx in self.effects:
            t = (now - fx.start) / fx.duration
            if t >= 1:
                continue
            alive.append(fx)
            cx, cy = self.cell_center(fx.cell)
            alpha = int(255 * (1 - t))
            if fx.kind == "ring":
                T.alpha_circle(surface, fx.color, (cx, cy), self.cell * (0.4 + 0.6 * t), alpha, 3)
            elif fx.kind == "burst":
                for k in range(8):
                    ang = k * math.pi / 4
                    d = self.cell * (0.3 + 0.7 * t)
                    T.alpha_circle(surface, fx.color, (cx + math.cos(ang) * d, cy + math.sin(ang) * d),
                                   max(2, self.cell // 10), alpha)
                T.alpha_circle(surface, fx.color, (cx, cy), self.cell * (0.4 + 0.4 * t), alpha, 3)
            elif fx.kind == "cross":
                r = self.cell * 0.3
                layer = pygame.Surface((self.cell, self.cell), pygame.SRCALPHA)
                m = self.cell / 2
                pygame.draw.line(layer, (*fx.color, alpha), (m - r, m - r), (m + r, m + r), 3)
                pygame.draw.line(layer, (*fx.color, alpha), (m - r, m + r), (m + r, m - r), 3)
                surface.blit(layer, (cx - m, cy - m))
            if fx.text:
                T.draw_text(surface, fx.text, (cx, cy - self.cell * (0.55 + 0.5 * t)), 12,
                            T.mix(fx.color, (255, 255, 255), 0.3), bold=True, anchor="center")
        self.effects = alive
