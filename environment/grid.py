"""Two-dimensional warehouse grid.

Positions are ``(row, col)`` tuples. Row 0 is the top of the warehouse.
Only static structure (floor vs. shelf) lives in the grid; parcels, stations
and robots are separate entities managed by :class:`environment.warehouse.Warehouse`.
"""

from __future__ import annotations

from collections import deque
from enum import IntEnum
from typing import Iterable, Iterator, Sequence

import numpy as np

Position = tuple[int, int]


class Cell(IntEnum):
    """Static cell types."""

    FLOOR = 0
    SHELF = 1


class Action(IntEnum):
    """Every action a robot can perform (PEAS actuators)."""

    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3
    PICKUP = 4
    DELIVER = 5
    WAIT = 6

    @property
    def is_move(self) -> bool:
        return self in MOVE_DELTAS

    @property
    def label(self) -> str:
        return {
            Action.UP: "Move Up",
            Action.DOWN: "Move Down",
            Action.LEFT: "Move Left",
            Action.RIGHT: "Move Right",
            Action.PICKUP: "Pick Up Parcel",
            Action.DELIVER: "Deliver Parcel",
            Action.WAIT: "Wait",
        }[self]


#: Fixed expansion order (Up, Down, Left, Right) keeps every search deterministic.
MOVE_DELTAS: dict[Action, tuple[int, int]] = {
    Action.UP: (-1, 0),
    Action.DOWN: (1, 0),
    Action.LEFT: (0, -1),
    Action.RIGHT: (0, 1),
}
MOVE_ACTIONS: tuple[Action, ...] = tuple(MOVE_DELTAS)


def apply_move(pos: Position, action: Action) -> Position:
    """Return the cell reached by ``action`` from ``pos`` (no validity check)."""
    dr, dc = MOVE_DELTAS[action]
    return (pos[0] + dr, pos[1] + dc)


def action_between(a: Position, b: Position) -> Action:
    """Return the move action leading from ``a`` to the adjacent cell ``b``."""
    delta = (b[0] - a[0], b[1] - a[1])
    for action, d in MOVE_DELTAS.items():
        if d == delta:
            return action
    raise ValueError(f"Cells {a} and {b} are not 4-neighbours")


def manhattan(a: Position, b: Position) -> int:
    """Manhattan distance ``|r1 - r2| + |c1 - c2|``."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class Grid:
    """Static warehouse floor plan backed by a NumPy array."""

    def __init__(self, cells: np.ndarray) -> None:
        if cells.ndim != 2 or cells.size == 0:
            raise ValueError("Grid must be a non-empty 2D array")
        self.cells: np.ndarray = cells.astype(np.int8, copy=True)
        self.height, self.width = self.cells.shape

    # ------------------------------------------------------------------ #
    # Construction helpers
    # ------------------------------------------------------------------ #
    @classmethod
    def empty(cls, width: int, height: int) -> "Grid":
        return cls(np.zeros((height, width), dtype=np.int8))

    @classmethod
    def from_strings(cls, rows: Sequence[str], shelf_chars: str = "#") -> "Grid":
        """Build a grid from text rows; ``shelf_chars`` mark shelves."""
        if not rows:
            raise ValueError("Layout map is empty")
        width = len(rows[0])
        if any(len(r) != width for r in rows):
            raise ValueError("All layout rows must have the same length")
        arr = np.zeros((len(rows), width), dtype=np.int8)
        for r, row in enumerate(rows):
            for c, ch in enumerate(row):
                if ch in shelf_chars:
                    arr[r, c] = Cell.SHELF
        return cls(arr)

    # ------------------------------------------------------------------ #
    # Queries
    # ------------------------------------------------------------------ #
    def in_bounds(self, pos: Position) -> bool:
        r, c = pos
        return 0 <= r < self.height and 0 <= c < self.width

    def is_shelf(self, pos: Position) -> bool:
        return self.in_bounds(pos) and self.cells[pos] == Cell.SHELF

    def is_walkable(self, pos: Position) -> bool:
        """True when ``pos`` is inside the grid and not a shelf."""
        return self.in_bounds(pos) and self.cells[pos] == Cell.FLOOR

    def set_shelf(self, pos: Position, shelf: bool = True) -> None:
        if not self.in_bounds(pos):
            raise IndexError(f"{pos} is outside the grid")
        self.cells[pos] = Cell.SHELF if shelf else Cell.FLOOR

    def neighbors(self, pos: Position) -> Iterator[Position]:
        """Walkable 4-neighbours of ``pos`` in fixed (Up, Down, Left, Right) order."""
        r, c = pos
        for dr, dc in MOVE_DELTAS.values():
            nxt = (r + dr, c + dc)
            if self.is_walkable(nxt):
                yield nxt

    def walkable_cells(self) -> list[Position]:
        rows, cols = np.nonzero(self.cells == Cell.FLOOR)
        return [(int(r), int(c)) for r, c in zip(rows, cols)]

    def shelf_cells(self) -> list[Position]:
        rows, cols = np.nonzero(self.cells == Cell.SHELF)
        return [(int(r), int(c)) for r, c in zip(rows, cols)]

    def blocked_neighbor_count(self, pos: Position) -> int:
        """Number of 4-neighbours that are shelves or outside the grid."""
        r, c = pos
        return sum(
            0 if self.is_walkable((r + dr, c + dc)) else 1
            for dr, dc in MOVE_DELTAS.values()
        )

    def reachable_from(
        self, start: Position, blocked: Iterable[Position] = ()
    ) -> set[Position]:
        """Flood fill of walkable cells reachable from ``start``."""
        if not self.is_walkable(start):
            return set()
        blocked_set = set(blocked)
        seen = {start}
        queue = deque([start])
        while queue:
            cur = queue.popleft()
            for nxt in self.neighbors(cur):
                if nxt not in seen and nxt not in blocked_set:
                    seen.add(nxt)
                    queue.append(nxt)
        return seen

    def copy(self) -> "Grid":
        return Grid(self.cells)

    def to_strings(self) -> list[str]:
        return [
            "".join("#" if v == Cell.SHELF else "." for v in row)
            for row in self.cells
        ]

    def __repr__(self) -> str:  # pragma: no cover - debugging helper
        return f"Grid({self.width}x{self.height})"
