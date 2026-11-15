"""Shared types for the search algorithms."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from environment.grid import Position


@dataclass
class SearchResult:
    """Outcome and metrics of one search.

    ``path`` includes both start and goal (``[start]`` when start == goal) and
    is empty when no path exists. ``cost`` is the number of moves.
    """

    algorithm: str
    start: Position
    goal: Position
    path: list[Position] = field(default_factory=list)
    nodes_expanded: int = 0
    nodes_generated: int = 0
    runtime_ms: float = 0.0
    #: Expansion order, recorded only when the search is called with ``trace=True``
    #: (used by the UI to visualise the explored region).
    explored: list[Position] = field(default_factory=list)

    @property
    def found(self) -> bool:
        return bool(self.path)

    @property
    def cost(self) -> Optional[int]:
        return len(self.path) - 1 if self.path else None


def reconstruct_path(
    parent: dict[Position, Optional[Position]], goal: Position
) -> list[Position]:
    """Follow parent pointers from ``goal`` back to the start (parent ``None``)."""
    path = [goal]
    node = parent[goal]
    while node is not None:
        path.append(node)
        node = parent[node]
    path.reverse()
    return path
