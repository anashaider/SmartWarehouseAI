"""Right information panel: robot inspector, decision history, search info, events."""

from __future__ import annotations

from typing import Callable, Optional

import pygame

from algorithms.common import SearchResult
from config.settings import AGENT_LABELS, ALGORITHM_LABELS
from environment.grid import Position
from interface import theme as T
from interface.widgets import ScrollState, Tabs, key_value, section_title
from simulation.simulation_engine import DecisionRecord, SimulationEngine

TAB_NAMES = ["Robot", "Decisions", "Search", "Events"]


class RobotPanel:
    def __init__(self, rect: pygame.Rect, on_select: Callable[[int], None]) -> None:
        self.rect = pygame.Rect(rect)
        self.on_select = on_select
        self.tab = 0
        self.tabs = Tabs(pygame.Rect(0, 0, 10, 10), TAB_NAMES, 0, self._set_tab)
        self.history_scroll = ScrollState()
        self.events_scroll = ScrollState()
        self.inspect_step: Optional[int] = None  # pinned history entry (None = latest)
        self._history_rows: list[tuple[pygame.Rect, int]] = []
        self._chip_rects: list[tuple[pygame.Rect, int]] = []
        self.comparison: Optional[dict[str, SearchResult]] = None
        self.comparison_query: Optional[tuple[Position, Position]] = None
        self.layout()

    def _set_tab(self, i: int) -> None:
        self.tab = i

    def set_rect(self, rect: pygame.Rect) -> None:
        self.rect = pygame.Rect(rect)
        self.layout()

    def layout(self) -> None:
        self.tabs.rect = pygame.Rect(self.rect.x + 14, self.rect.y + 62, self.rect.width - 28, 36)
        self.body = pygame.Rect(self.rect.x + 18, self.rect.y + 110, self.rect.width - 36,
                                self.rect.height - 124)

    # ------------------------------------------------------------------ #
    def handle(self, event: pygame.event.Event, engine: SimulationEngine) -> bool:
        if self.tabs.handle(event):
            return True
        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            for rect, rid in self._chip_rects:
                if rect.collidepoint(event.pos):
                    self.on_select(rid)
                    self.inspect_step = None
                    return True
            if self.tab == 1:
                for rect, step in self._history_rows:
                    if rect.collidepoint(event.pos):
                        self.inspect_step = None if self.inspect_step == step else step
                        return True
        if self.tab == 1 and self.history_scroll.handle_wheel(event, self.body, 10_000):
            return True
        if self.tab == 3 and self.events_scroll.handle_wheel(event, self.body, 10_000):
            return True
        return False

    # ------------------------------------------------------------------ #
    def draw(self, surface: pygame.Surface, engine: SimulationEngine, selected: int) -> None:
        T.rounded_rect(surface, T.PANEL, self.rect, 12, 1, T.BORDER)
        x = self.rect.x + 18
        T.draw_text(surface, "ROBOT INSPECTOR", (x, self.rect.y + 14), 11, T.ACCENT_HOVER, bold=True)
        # robot chips
        self._chip_rects = []
        cx = x
        for robot in engine.warehouse.robots:
            r = pygame.Rect(cx, self.rect.y + 32, 62, 24)
            col = T.robot_color(robot.robot_id)
            active = robot.robot_id == selected
            T.rounded_rect(surface, col if active else T.CARD, r, 12, 1, col)
            T.draw_text(surface, f"R{robot.robot_id}", r.center, 12,
                        (255, 255, 255) if active else T.TEXT, bold=True, anchor="center")
            self._chip_rects.append((r, robot.robot_id))
            cx += 70
        agent = AGENT_LABELS[engine.config.agent_type]
        T.draw_text(surface, agent, (self.rect.right - 18, self.rect.y + 38), 12, T.TEXT_SECONDARY,
                    anchor="topright")
        self.tabs.index = self.tab
        self.tabs.draw(surface)
        robot = next((r for r in engine.warehouse.robots if r.robot_id == selected), engine.warehouse.robots[0])
        clip = surface.get_clip()
        surface.set_clip(self.body)
        [self._draw_robot_tab, self._draw_decisions_tab, self._draw_search_tab, self._draw_events_tab][self.tab](
            surface, engine, robot)
        surface.set_clip(clip)

    # ------------------------------------------------------------------ #
    def _draw_robot_tab(self, surface, engine: SimulationEngine, robot) -> None:
        b = self.body
        x, w, y = b.x, b.width, b.y
        wh = engine.warehouse
        col = T.STATE_COLORS.get(robot.state.value, T.TEXT)
        badge = pygame.Rect(x, y, w, 40)
        T.rounded_rect(surface, T.CARD, badge, 10, 1, T.BORDER)
        pygame.draw.circle(surface, col, (badge.x + 18, badge.centery), 6)
        T.draw_text(surface, robot.state.label, (badge.x + 32, badge.centery), 15, T.TEXT, bold=True, anchor="midleft")
        T.draw_text(surface, robot.current_action.label, (badge.right - 12, badge.centery), 12,
                    T.TEXT_SECONDARY, anchor="midright")
        y += 52
        parcel = f"P{robot.assigned_parcel}" if robot.assigned_parcel is not None else "—"
        dest = "—"
        target = wh.parcel_target(robot)
        if robot.carrying is not None:
            st = wh.station_for(wh.parcels[robot.carrying])
            dest = f"{st.label} {st.position}"
        elif target is not None:
            dest = f"parcel {target}"
        rows = [
            ("Position", str(robot.position)),
            ("Assigned parcel", parcel),
            ("Carrying", f"P{robot.carrying}" if robot.carrying is not None else "nothing"),
            ("Current destination", dest),
            ("Distance travelled", f"{robot.distance_travelled} cells"),
            ("Deliveries completed", str(robot.deliveries_completed)),
            ("Waits / conflicts", f"{robot.waits} / {robot.conflicts}"),
            ("Invalid moves", str(robot.invalid_moves)),
        ]
        for k, v in rows:
            y = key_value(surface, k, v, x, y, w)
        y += 4
        y = section_title(surface, "Agent decision", x, y, w)
        rec = engine.last_decision.get(robot.robot_id)
        box = pygame.Rect(x, y, w, 118)
        T.rounded_rect(surface, T.CARD, box, 10, 1, T.BORDER)
        if rec:
            T.draw_text(surface, f"Step {rec.step} · {rec.chosen.label} → {rec.outcome}",
                        (box.x + 12, box.y + 10), 12, T.ACCENT_HOVER, bold=True, max_width=box.width - 24)
            T.draw_wrapped(surface, rec.reason, pygame.Rect(box.x + 12, box.y + 32, box.width - 24, box.height - 38),
                           12, T.TEXT, max_lines=5)
        else:
            T.draw_text(surface, "No decision yet. Press Start or Step.", (box.x + 12, box.y + 12), 12, T.TEXT_MUTED)
        y = box.bottom + 14
        y = section_title(surface, "Delivery history", x, y, w)
        if not robot.delivery_history:
            T.draw_text(surface, "No deliveries yet.", (x, y), 12, T.TEXT_MUTED)
        for rec_d in reversed(robot.delivery_history[-6:]):
            T.draw_text(surface, f"P{rec_d.parcel_id} → D{rec_d.station_id}", (x, y), 12, T.TEXT)
            T.draw_text(surface, f"picked t{rec_d.picked_step} · delivered t{rec_d.delivered_step}",
                        (x + w, y), 12, T.TEXT_SECONDARY, anchor="topright")
            y += 20

    # ------------------------------------------------------------------ #
    def _draw_decisions_tab(self, surface, engine: SimulationEngine, robot) -> None:
        b = self.body
        x, w = b.x, b.width
        history = list(engine.history[robot.robot_id])[::-1]
        list_h = min(232, b.height // 2 - 10)
        T.draw_text(surface, f"{len(history)} decisions recorded (newest first) · click to inspect",
                    (x, b.y), 11, T.TEXT_MUTED)
        area = pygame.Rect(x, b.y + 18, w, list_h)
        T.rounded_rect(surface, T.PANEL_ALT, area, 8)
        row_h = 22
        visible = area.height // row_h
        max_off = max(0, len(history) - visible)
        self.history_scroll.offset = min(self.history_scroll.offset, max_off)
        self._history_rows = []
        clip = surface.get_clip()
        surface.set_clip(area.clip(clip))
        for i, rec in enumerate(history[self.history_scroll.offset:self.history_scroll.offset + visible]):
            r = pygame.Rect(area.x, area.y + i * row_h, area.width, row_h)
            pinned = self.inspect_step == rec.step
            if pinned:
                pygame.draw.rect(surface, T.ACCENT_DIM, r)
            elif r.collidepoint(T.mouse_pos()):
                pygame.draw.rect(surface, T.CARD, r)
            T.draw_text(surface, f"t{rec.step:>4}", (r.x + 8, r.y + 4), 12, T.TEXT_MUTED, fnt=T.mono(12))
            T.draw_text(surface, rec.chosen.label, (r.x + 62, r.y + 3), 12, T.TEXT, bold=True)
            T.draw_text(surface, rec.outcome, (r.right - 8, r.y + 4), 11,
                        T.WARN if "conflict" in rec.outcome or "wait" in rec.outcome else T.TEXT_SECONDARY,
                        anchor="topright", max_width=130)
            self._history_rows.append((r, rec.step))
        surface.set_clip(clip)
        if max_off:
            frac = self.history_scroll.offset / max_off
            bar_h = max(20, area.height * visible // max(1, len(history)))
            bar = pygame.Rect(area.right - 4, area.y + int((area.height - bar_h) * frac), 3, bar_h)
            pygame.draw.rect(surface, T.BORDER, bar, border_radius=2)

        rec: Optional[DecisionRecord] = None
        if self.inspect_step is not None:
            rec = next((h for h in history if h.step == self.inspect_step), None)
        if rec is None and history:
            rec = history[0]
        y = area.bottom + 12
        if rec is None:
            T.draw_text(surface, "Run the simulation to record decisions.", (x, y), 12, T.TEXT_MUTED)
            return
        y = section_title(surface, f"Decision at step {rec.step}", x, y, w)
        for k, v in rec.percept.items():
            T.draw_text(surface, k, (x, y), 11, T.TEXT_MUTED)
            T.draw_text(surface, v, (x + 98, y), 11, T.TEXT, max_width=w - 98)
            y += 16
        y += 6
        T.draw_text(surface, "AVAILABLE ACTIONS", (x, y), 10, T.TEXT_MUTED, bold=True)
        y += 16
        scores = [c.score for c in rec.candidates if c.score is not None]
        lo, hi = (min(scores), max(scores)) if scores else (0.0, 1.0)
        for cand in rec.candidates:
            chosen = cand.action == rec.chosen
            colr = T.GOOD if chosen else (T.TEXT if cand.valid else T.TEXT_MUTED)
            mark = "●" if chosen else ("○" if cand.valid else "×")
            T.draw_text(surface, f"{mark} {cand.action.label}", (x, y), 12, colr, bold=chosen)
            if cand.score is not None:
                bar_w = 70
                t = (cand.score - lo) / (hi - lo) if hi > lo else 1.0
                bar = pygame.Rect(x + 120, y + 4, max(2, int(bar_w * t)), 8)
                pygame.draw.rect(surface, T.CARD, (x + 120, y + 4, bar_w, 8), border_radius=3)
                pygame.draw.rect(surface, T.GOOD if chosen else T.ACCENT, bar, border_radius=3)
                T.draw_text(surface, f"U={cand.score:+.2f}", (x + 196, y), 11, colr)
            else:
                T.draw_text(surface, cand.note, (x + 120, y + 1), 11, T.TEXT_MUTED, max_width=w - 120)
            y += 18
        y += 6
        T.draw_text(surface, "REASON", (x, y), 10, T.TEXT_MUTED, bold=True)
        y += 16
        T.draw_wrapped(surface, rec.reason, pygame.Rect(x, y, w, b.bottom - y), 12, T.TEXT)

    # ------------------------------------------------------------------ #
    def _draw_search_tab(self, surface, engine: SimulationEngine, robot) -> None:
        b = self.body
        x, w, y = b.x, b.width, b.y
        agent = engine.agents[robot.robot_id]
        stats = agent.pathfinder.stats
        alg = ALGORITHM_LABELS[engine.config.algorithm]
        y = section_title(surface, "Robot pathfinding", x, y, w)
        if not agent.uses_search:
            T.draw_wrapped(surface, "The Simple Reflex agent does not search: it reacts to adjacent "
                           "cells only. The comparison below still shows what BFS and A* would do.",
                           pygame.Rect(x, y, w, 60), 12, T.WARN)
            y += 52
        else:
            last = agent.pathfinder.last_result
            y = key_value(surface, "Algorithm", alg, x, y, w)
            y = key_value(surface, "Searches performed", str(stats.searches), x, y, w)
            y = key_value(surface, "Total nodes expanded", f"{stats.nodes_expanded:,}", x, y, w)
            y = key_value(surface, "Total search time", f"{stats.runtime_ms:.2f} ms", x, y, w)
            if last:
                y = key_value(surface, "Last query", f"{last.start} → {last.goal}", x, y, w)
                y = key_value(surface, "Last path length", str(last.cost) if last.found else "no path", x, y, w)
                y = key_value(surface, "Last nodes expanded", str(last.nodes_expanded), x, y, w)
        y += 6
        y = section_title(surface, "BFS vs A* (current position → target)", x, y, w)
        cmp_ = self.comparison
        if not cmp_:
            T.draw_text(surface, "Robot has no target right now.", (x, y), 12, T.TEXT_MUTED)
            return
        bfs, ast = cmp_["bfs"], cmp_["astar"]
        cols = [x, x + int(w * 0.45), x + int(w * 0.75)]
        T.draw_text(surface, "Metric", (cols[0], y), 11, T.TEXT_MUTED, bold=True)
        for cx_, label, colr in ((cols[1], "BFS", T.BFS_COLOR), (cols[2], "A*", T.ASTAR_COLOR)):
            pygame.draw.rect(surface, colr, (cx_, y + 3, 10, 10), border_radius=2)
            T.draw_text(surface, label, (cx_ + 15, y), 12, T.TEXT, bold=True)
        y += 22
        rows = [
            ("Path length", str(bfs.cost), str(ast.cost)),
            ("Nodes expanded", str(bfs.nodes_expanded), str(ast.nodes_expanded)),
            ("Nodes generated", str(bfs.nodes_generated), str(ast.nodes_generated)),
            ("Runtime (ms)", f"{bfs.runtime_ms:.3f}", f"{ast.runtime_ms:.3f}"),
        ]
        for label, a, c in rows:
            T.draw_text(surface, label, (cols[0], y), 12, T.TEXT_SECONDARY)
            T.draw_text(surface, a, (cols[1] + 15, y), 12, T.TEXT, bold=True)
            T.draw_text(surface, c, (cols[2] + 15, y), 12, T.TEXT, bold=True)
            y += 20
        y += 8
        same = bfs.cost == ast.cost
        saving = 100 * (1 - ast.nodes_expanded / bfs.nodes_expanded) if bfs.nodes_expanded else 0
        msg = (f"Both found an optimal path of {bfs.cost} moves. " if same and bfs.found else
               "Path lengths differ. " if bfs.found else "No path exists. ")
        if bfs.found and saving > 0:
            msg += (f"A* expanded {saving:.0f}% fewer nodes because h(n) = Manhattan distance "
                    "guides it toward the goal.")
        elif bfs.found:
            msg += "On this short query both expanded the same number of nodes."
        T.draw_wrapped(surface, msg, pygame.Rect(x, y, w, 70), 12, T.TEXT)
        y += 70
        T.draw_wrapped(surface, "Enable 'BFS vs A* overlay' to draw both paths and their explored "
                       "cells on the map (blue tint = BFS explored, orange squares = A* explored).",
                       pygame.Rect(x, y, w, 60), 11, T.TEXT_MUTED)

    # ------------------------------------------------------------------ #
    def _draw_events_tab(self, surface, engine: SimulationEngine, robot) -> None:
        b = self.body
        events = list(engine.event_log)[::-1]
        T.draw_text(surface, f"{len(events)} recent events (newest first)", (b.x, b.y), 11, T.TEXT_MUTED)
        row_h = 20
        area = pygame.Rect(b.x, b.y + 18, b.width, b.height - 18)
        visible = area.height // row_h
        self.events_scroll.offset = min(self.events_scroll.offset, max(0, len(events) - visible))
        colors = {"pickup": T.WARN, "deliver": T.GOOD, "conflict": T.BAD, "invalid": T.BAD,
                  "yield": T.INFO, "assign": T.ACCENT_HOVER, "wait": T.TEXT_MUTED}
        y = area.y
        for ev in events[self.events_scroll.offset:self.events_scroll.offset + visible]:
            T.draw_text(surface, f"t{ev.step:>4}", (b.x, y + 1), 11, T.TEXT_MUTED, fnt=T.mono(11))
            pygame.draw.circle(surface, colors.get(ev.kind, T.TEXT), (b.x + 50, y + 9), 4)
            T.draw_text(surface, ev.message, (b.x + 60, y), 12, T.TEXT, max_width=b.width - 60)
            y += row_h
        if not events:
            T.draw_text(surface, "No events yet.", (b.x, area.y), 12, T.TEXT_MUTED)
