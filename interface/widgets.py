"""Minimal retained-mode widgets for the Pygame dashboard.

Each widget has a ``rect``, draws itself in :meth:`draw` and reacts to mouse
events in :meth:`handle`. Callbacks are plain Python callables, so every
button is wired to a real controller action.
"""

from __future__ import annotations

from typing import Callable, Optional, Sequence

import pygame

from interface import theme as T


class Widget:
    def __init__(self, rect: pygame.Rect) -> None:
        self.rect = pygame.Rect(rect)
        self.visible = True
        self.enabled = True

    def handle(self, event: pygame.event.Event) -> bool:  # pragma: no cover - interface
        return False

    def draw(self, surface: pygame.Surface) -> None:  # pragma: no cover - interface
        pass

    def hovered(self) -> bool:
        return self.rect.collidepoint(T.mouse_pos())


class Button(Widget):
    def __init__(self, rect: pygame.Rect, label: str, callback: Callable[[], None],
                 primary: bool = False, icon: str = "", tooltip: str = "") -> None:
        super().__init__(rect)
        self.label = label
        self.callback = callback
        self.primary = primary
        self.icon = icon
        self.tooltip = tooltip
        self.active = False  # for toggle-style buttons

    def handle(self, event: pygame.event.Event) -> bool:
        if not (self.visible and self.enabled):
            return False
        if event.type == pygame.MOUSEBUTTONUP and event.button == 1 and self.rect.collidepoint(event.pos):
            self.callback()
            return True
        return False

    def draw(self, surface: pygame.Surface) -> None:
        if not self.visible:
            return
        hover = self.enabled and self.hovered()
        if not self.enabled:
            bg, fg, border = T.PANEL_ALT, T.TEXT_MUTED, T.DIVIDER
        elif self.primary or self.active:
            bg = T.ACCENT_HOVER if hover else T.ACCENT
            fg, border = (255, 255, 255), bg
        else:
            bg = T.CARD_HOVER if hover else T.CARD
            fg, border = T.TEXT, T.BORDER
        T.rounded_rect(surface, bg, self.rect, 8, 1, border)
        if self.icon:
            f = T.font(14, True)
            tw = f.size(self.label)[0]
            total = tw + 22
            ix = self.rect.centerx - total // 2
            draw_icon(surface, self.icon, pygame.Rect(ix, self.rect.centery - 7, 14, 14), fg)
            T.draw_text(surface, self.label, (ix + 22, self.rect.centery), 14, fg, bold=True,
                        anchor="midleft", max_width=self.rect.width - 40)
        else:
            T.draw_text(surface, self.label, self.rect.center, 14, fg, bold=True, anchor="center",
                        max_width=self.rect.width - 10)


def draw_icon(surface: pygame.Surface, name: str, r: pygame.Rect, color: T.Color) -> None:
    """Small vector icons (fonts on Windows lack the media-control glyphs)."""
    if name == "play":
        pygame.draw.polygon(surface, color, [(r.x + 2, r.y), (r.right, r.centery), (r.x + 2, r.bottom)])
    elif name == "pause":
        pygame.draw.rect(surface, color, (r.x + 2, r.y, 4, r.height), border_radius=1)
        pygame.draw.rect(surface, color, (r.right - 6, r.y, 4, r.height), border_radius=1)
    elif name == "step":
        pygame.draw.polygon(surface, color, [(r.x, r.y), (r.right - 4, r.centery), (r.x, r.bottom)])
        pygame.draw.rect(surface, color, (r.right - 3, r.y, 3, r.height))
    elif name == "reset":
        pygame.draw.arc(surface, color, r.inflate(-1, -1), 0.6, 5.6, 2)
        tip = (r.right - 2, r.centery - 2)
        pygame.draw.polygon(surface, color, [(tip[0] - 5, tip[1] - 3), (tip[0] + 3, tip[1] - 4), (tip[0], tip[1] + 4)])


class Selector(Widget):
    """Label + value with previous/next arrows (cycles through ``options``)."""

    def __init__(self, rect: pygame.Rect, label: str, options: Sequence[tuple[str, str]],
                 value: str, on_change: Callable[[str], None]) -> None:
        super().__init__(rect)
        self.label = label
        self.options = list(options)  # (value, display)
        self.value = value
        self.on_change = on_change

    @property
    def _box(self) -> pygame.Rect:
        return pygame.Rect(self.rect.x, self.rect.y + 18, self.rect.width, self.rect.height - 18)

    @property
    def _left(self) -> pygame.Rect:
        b = self._box
        return pygame.Rect(b.x, b.y, 30, b.height)

    @property
    def _right(self) -> pygame.Rect:
        b = self._box
        return pygame.Rect(b.right - 30, b.y, 30, b.height)

    def _index(self) -> int:
        for i, (v, _) in enumerate(self.options):
            if v == self.value:
                return i
        return 0

    def set_value(self, value: str) -> None:
        self.value = value

    def handle(self, event: pygame.event.Event) -> bool:
        if not (self.visible and self.enabled):
            return False
        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            delta = 0
            if self._left.collidepoint(event.pos):
                delta = -1
            elif self._right.collidepoint(event.pos) or self._box.collidepoint(event.pos):
                delta = 1
            if delta:
                idx = (self._index() + delta) % len(self.options)
                self.value = self.options[idx][0]
                self.on_change(self.value)
                return True
        if event.type == pygame.MOUSEWHEEL and self._box.collidepoint(T.mouse_pos()):
            idx = (self._index() - event.y) % len(self.options)
            self.value = self.options[idx][0]
            self.on_change(self.value)
            return True
        return False

    def draw(self, surface: pygame.Surface) -> None:
        if not self.visible:
            return
        T.draw_text(surface, self.label.upper(), (self.rect.x + 2, self.rect.y), 11, T.TEXT_MUTED, bold=True)
        box = self._box
        hover = self.enabled and box.collidepoint(T.mouse_pos())
        T.rounded_rect(surface, T.CARD_HOVER if hover else T.CARD, box, 8, 1, T.BORDER)
        for r, glyph in ((self._left, "‹"), (self._right, "›")):
            col = T.ACCENT_HOVER if r.collidepoint(T.mouse_pos()) else T.TEXT_SECONDARY
            T.draw_text(surface, glyph, r.center, 20, col, bold=True, anchor="center")
        display = self.options[self._index()][1]
        T.draw_text(surface, display, box.center, 14, T.TEXT if self.enabled else T.TEXT_MUTED,
                    bold=True, anchor="center", max_width=box.width - 64)


class Slider(Widget):
    def __init__(self, rect: pygame.Rect, label: str, steps: Sequence[float], index: int,
                 on_change: Callable[[int], None], fmt: Callable[[float], str] = str) -> None:
        super().__init__(rect)
        self.label = label
        self.steps = list(steps)
        self.index = index
        self.on_change = on_change
        self.fmt = fmt
        self.dragging = False

    @property
    def _track(self) -> pygame.Rect:
        return pygame.Rect(self.rect.x + 8, self.rect.y + 32, self.rect.width - 16, 6)

    def _set_from_x(self, x: int) -> None:
        tr = self._track
        t = (x - tr.x) / max(1, tr.width)
        idx = round(max(0.0, min(1.0, t)) * (len(self.steps) - 1))
        if idx != self.index:
            self.index = idx
            self.on_change(idx)

    def handle(self, event: pygame.event.Event) -> bool:
        if not self.visible:
            return False
        hit = pygame.Rect(self.rect.x, self.rect.y + 20, self.rect.width, 28)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and hit.collidepoint(event.pos):
            self.dragging = True
            self._set_from_x(event.pos[0])
            return True
        if event.type == pygame.MOUSEMOTION and self.dragging:
            self._set_from_x(event.pos[0])
            return True
        if event.type == pygame.MOUSEBUTTONUP and event.button == 1 and self.dragging:
            self.dragging = False
            return True
        return False

    def draw(self, surface: pygame.Surface) -> None:
        T.draw_text(surface, self.label.upper(), (self.rect.x + 2, self.rect.y), 11, T.TEXT_MUTED, bold=True)
        T.draw_text(surface, self.fmt(self.steps[self.index]), (self.rect.right - 2, self.rect.y),
                    12, T.TEXT, bold=True, anchor="topright")
        tr = self._track
        T.rounded_rect(surface, T.CARD, tr, 3)
        t = self.index / max(1, len(self.steps) - 1)
        filled = pygame.Rect(tr.x, tr.y, int(tr.width * t), tr.height)
        T.rounded_rect(surface, T.ACCENT, filled, 3)
        for i in range(len(self.steps)):
            x = tr.x + int(tr.width * i / max(1, len(self.steps) - 1))
            pygame.draw.circle(surface, T.BORDER if i > self.index else T.ACCENT_HOVER, (x, tr.centery), 2)
        knob = (tr.x + int(tr.width * t), tr.centery)
        pygame.draw.circle(surface, (255, 255, 255), knob, 8)
        pygame.draw.circle(surface, T.ACCENT, knob, 8, 2)


class Toggle(Widget):
    def __init__(self, rect: pygame.Rect, label: str, value: bool,
                 on_change: Callable[[bool], None]) -> None:
        super().__init__(rect)
        self.label = label
        self.value = value
        self.on_change = on_change

    def handle(self, event: pygame.event.Event) -> bool:
        if self.visible and event.type == pygame.MOUSEBUTTONUP and event.button == 1 \
                and self.rect.collidepoint(event.pos):
            self.value = not self.value
            self.on_change(self.value)
            return True
        return False

    def draw(self, surface: pygame.Surface) -> None:
        sw = pygame.Rect(self.rect.right - 38, self.rect.centery - 10, 38, 20)
        T.rounded_rect(surface, T.ACCENT if self.value else T.CARD, sw, 10, 1,
                       T.ACCENT if self.value else T.BORDER)
        kx = sw.right - 10 if self.value else sw.x + 10
        pygame.draw.circle(surface, (255, 255, 255), (kx, sw.centery), 7)
        T.draw_text(surface, self.label, (self.rect.x + 2, self.rect.centery), 13,
                    T.TEXT if self.hovered() else T.TEXT_SECONDARY, anchor="midleft",
                    max_width=self.rect.width - 50)


class Tabs(Widget):
    def __init__(self, rect: pygame.Rect, labels: Sequence[str], index: int,
                 on_change: Callable[[int], None]) -> None:
        super().__init__(rect)
        self.labels = list(labels)
        self.index = index
        self.on_change = on_change

    def _tab_rects(self) -> list[pygame.Rect]:
        w = self.rect.width // len(self.labels)
        return [pygame.Rect(self.rect.x + i * w, self.rect.y, w, self.rect.height)
                for i in range(len(self.labels))]

    def handle(self, event: pygame.event.Event) -> bool:
        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            for i, r in enumerate(self._tab_rects()):
                if r.collidepoint(event.pos):
                    self.index = i
                    self.on_change(i)
                    return True
        return False

    def draw(self, surface: pygame.Surface) -> None:
        T.rounded_rect(surface, T.PANEL_ALT, self.rect, 8)
        for i, (r, label) in enumerate(zip(self._tab_rects(), self.labels)):
            inner = r.inflate(-6, -6)
            if i == self.index:
                T.rounded_rect(surface, T.CARD_HOVER, inner, 6, 1, T.BORDER)
                col = T.TEXT
            else:
                col = T.TEXT_SECONDARY if r.collidepoint(T.mouse_pos()) else T.TEXT_MUTED
            T.draw_text(surface, label, inner.center, 13, col, bold=True, anchor="center")


class ScrollState:
    """Scroll offset helper for list-like panels."""

    def __init__(self) -> None:
        self.offset = 0

    def handle_wheel(self, event: pygame.event.Event, area: pygame.Rect, max_offset: int,
                     step: int = 3) -> bool:
        if event.type == pygame.MOUSEWHEEL and area.collidepoint(T.mouse_pos()):
            self.offset = max(0, min(max_offset, self.offset - event.y * step))
            return True
        return False


def section_title(surface: pygame.Surface, text: str, x: int, y: int, width: int) -> int:
    T.draw_text(surface, text.upper(), (x, y), 11, T.ACCENT_HOVER, bold=True)
    pygame.draw.line(surface, T.DIVIDER, (x, y + 20), (x + width, y + 20))
    return y + 28


def key_value(surface: pygame.Surface, key: str, value: str, x: int, y: int, width: int,
              value_color: Optional[T.Color] = None, size: int = 13) -> int:
    T.draw_text(surface, key, (x, y), size, T.TEXT_MUTED)
    T.draw_text(surface, value, (x + width, y), size, value_color or T.TEXT, bold=True,
                anchor="topright", max_width=int(width * 0.62))
    return y + size + 9
