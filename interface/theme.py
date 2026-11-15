"""Colours, fonts and small drawing helpers for the Pygame interface."""

from __future__ import annotations

from functools import lru_cache

import pygame

Color = tuple[int, int, int]


def hex_color(value: str) -> Color:
    value = value.lstrip("#")
    return (int(value[0:2], 16), int(value[2:4], 16), int(value[4:6], 16))


# Surfaces (dark navy / charcoal)
BG = hex_color("#0b1120")
PANEL = hex_color("#111a2e")
PANEL_ALT = hex_color("#151f36")
CARD = hex_color("#18233d")
CARD_HOVER = hex_color("#1e2b4a")
BORDER = hex_color("#25314f")
DIVIDER = hex_color("#1f2a44")

# Text
TEXT = hex_color("#e6ebf5")
TEXT_SECONDARY = hex_color("#aab4cc")
TEXT_MUTED = hex_color("#6f7c99")

# Accent
ACCENT = hex_color("#3b82f6")
ACCENT_HOVER = hex_color("#5b9bff")
ACCENT_DIM = hex_color("#1d3a6e")

# Status
GOOD = hex_color("#22c55e")
WARN = hex_color("#f59e0b")
BAD = hex_color("#ef4444")
INFO = hex_color("#38bdf8")

# Warehouse
FLOOR = hex_color("#0f1729")
FLOOR_ALT = hex_color("#111b30")
GRID_LINE = hex_color("#18233b")
SHELF = hex_color("#2c3a5c")
SHELF_TOP = hex_color("#3a4b74")
SHELF_EDGE = hex_color("#22304d")
STATION = hex_color("#16a34a")
STATION_DARK = hex_color("#0f3d26")
PARCEL = hex_color("#d39b5f")
PARCEL_DARK = hex_color("#8a5a2b")
PARCEL_TAPE = hex_color("#f3d3a6")

ROBOT_COLORS: list[Color] = [hex_color("#4f8cff"), hex_color("#ff7a45"), hex_color("#b36bff")]

# Search comparison overlay (validated categorical slots 1-2)
BFS_COLOR = hex_color("#3987e5")
ASTAR_COLOR = hex_color("#d95926")

STATUS_COLORS = {
    "READY": INFO,
    "RUNNING": GOOD,
    "PAUSED": WARN,
    "FINISHED": hex_color("#a78bfa"),
}

STATE_COLORS = {
    "IDLE": TEXT_MUTED,
    "NAVIGATING_TO_PARCEL": INFO,
    "PICKING_UP": WARN,
    "NAVIGATING_TO_DROPOFF": ACCENT,
    "DELIVERING": GOOD,
}


def robot_color(robot_id: int) -> Color:
    return ROBOT_COLORS[robot_id % len(ROBOT_COLORS)]


def mix(a: Color, b: Color, t: float) -> Color:
    return (
        int(a[0] + (b[0] - a[0]) * t),
        int(a[1] + (b[1] - a[1]) * t),
        int(a[2] + (b[2] - a[2]) * t),
    )


@lru_cache(maxsize=64)
def font(size: int, bold: bool = False) -> pygame.font.Font:
    """Segoe UI on Windows, falling back to other sans fonts / pygame default."""
    for name in ("segoeui", "segoe ui", "helveticaneue", "arial", "dejavusans"):
        path = pygame.font.match_font(name, bold=bold)
        if path:
            try:
                return pygame.font.Font(path, size)
            except (OSError, RuntimeError):
                continue
    f = pygame.font.Font(None, int(size * 1.35))
    f.set_bold(bold)
    return f


@lru_cache(maxsize=8)
def mono(size: int) -> pygame.font.Font:
    for name in ("consolas", "cascadiamono", "couriernew", "dejavusansmono"):
        path = pygame.font.match_font(name)
        if path:
            try:
                return pygame.font.Font(path, size)
            except (OSError, RuntimeError):
                continue
    return pygame.font.Font(None, int(size * 1.35))


def draw_text(surface: pygame.Surface, text: str, pos: tuple[int, int], size: int = 14,
              color: Color = TEXT, bold: bool = False, anchor: str = "topleft",
              max_width: int | None = None, fnt: pygame.font.Font | None = None) -> pygame.Rect:
    f = fnt or font(size, bold)
    if max_width is not None and f.size(text)[0] > max_width:
        while text and f.size(text + "…")[0] > max_width:
            text = text[:-1]
        text += "…"
    img = f.render(text, True, color)
    rect = img.get_rect(**{anchor: pos})
    surface.blit(img, rect)
    return rect


def wrap_text(text: str, fnt: pygame.font.Font, width: int) -> list[str]:
    lines: list[str] = []
    for paragraph in text.split("\n"):
        words = paragraph.split(" ")
        line = ""
        for word in words:
            trial = f"{line} {word}".strip()
            if fnt.size(trial)[0] <= width:
                line = trial
            else:
                if line:
                    lines.append(line)
                while fnt.size(word)[0] > width and len(word) > 1:
                    cut = len(word)
                    while cut > 1 and fnt.size(word[:cut])[0] > width:
                        cut -= 1
                    lines.append(word[:cut])
                    word = word[cut:]
                line = word
        lines.append(line)
    return lines


def draw_wrapped(surface: pygame.Surface, text: str, rect: pygame.Rect, size: int = 13,
                 color: Color = TEXT_SECONDARY, line_gap: int = 3, max_lines: int | None = None) -> int:
    """Draw word-wrapped text inside ``rect``; returns the height used."""
    f = font(size)
    lines = wrap_text(text, f, rect.width)
    if max_lines is not None and len(lines) > max_lines:
        lines = lines[:max_lines]
        lines[-1] = lines[-1].rstrip() + " …"
    y = rect.y
    for line in lines:
        if y + f.get_height() > rect.bottom:
            break
        surface.blit(f.render(line, True, color), (rect.x, y))
        y += f.get_height() + line_gap
    return y - rect.y


def rounded_rect(surface: pygame.Surface, color: Color, rect: pygame.Rect, radius: int = 8,
                 border: int = 0, border_color: Color | None = None) -> None:
    pygame.draw.rect(surface, color, rect, border_radius=radius)
    if border and border_color:
        pygame.draw.rect(surface, border_color, rect, border, border_radius=radius)


def alpha_rect(surface: pygame.Surface, color: Color, rect: pygame.Rect, alpha: int,
               radius: int = 0) -> None:
    layer = pygame.Surface(rect.size, pygame.SRCALPHA)
    pygame.draw.rect(layer, (*color, alpha), layer.get_rect(), border_radius=radius)
    surface.blit(layer, rect.topleft)


def alpha_circle(surface: pygame.Surface, color: Color, center: tuple[float, float],
                 radius: float, alpha: int, width: int = 0) -> None:
    r = max(1, int(radius))
    layer = pygame.Surface((r * 2 + 4, r * 2 + 4), pygame.SRCALPHA)
    pygame.draw.circle(layer, (*color, alpha), (r + 2, r + 2), r, width)
    surface.blit(layer, (center[0] - r - 2, center[1] - r - 2))


# --------------------------------------------------------------------------- #
# Logical-canvas mouse mapping
# --------------------------------------------------------------------------- #
# The dashboard renders to a fixed logical canvas that is scaled to the window.
# ``set_mouse_transform`` is called by the app whenever the window size changes.
_MOUSE_SCALE = (1.0, 1.0)
_MOUSE_OFFSET = (0, 0)


def set_mouse_transform(scale: tuple[float, float], offset: tuple[int, int]) -> None:
    global _MOUSE_SCALE, _MOUSE_OFFSET
    _MOUSE_SCALE, _MOUSE_OFFSET = scale, offset


def to_logical(pos: tuple[int, int]) -> tuple[int, int]:
    return (int((pos[0] - _MOUSE_OFFSET[0]) / _MOUSE_SCALE[0]),
            int((pos[1] - _MOUSE_OFFSET[1]) / _MOUSE_SCALE[1]))


def mouse_pos() -> tuple[int, int]:
    return to_logical(pygame.mouse.get_pos())
