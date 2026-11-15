"""Pathfinder facade: selects BFS or A* and accumulates search statistics."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable

from algorithms.astar import astar
from algorithms.bfs import bfs
from algorithms.common import SearchResult
from environment.grid import Grid, Position

SearchFn = Callable[[Grid, Position, Position, Iterable[Position]], SearchResult]

SEARCH_FUNCTIONS: dict[str, SearchFn] = {"bfs": bfs, "astar": astar}


def get_search_function(name: str) -> SearchFn:
    try:
        return SEARCH_FUNCTIONS[name]
    except KeyError as exc:
        raise ValueError(f"Unknown search algorithm {name!r}") from exc


@dataclass
class SearchStats:
    searches: int = 0
    nodes_expanded: int = 0
    runtime_ms: float = 0.0
    failures: int = 0

    def add(self, result: SearchResult) -> None:
        self.searches += 1
        self.nodes_expanded += result.nodes_expanded
        self.runtime_ms += result.runtime_ms
        if not result.found:
            self.failures += 1


class Pathfinder:
    """Runs the selected algorithm and records every search it performs."""

    def __init__(self, grid: Grid, algorithm: str = "astar") -> None:
        self.grid = grid
        self.algorithm = algorithm
        self._search = get_search_function(algorithm)
        self.stats = SearchStats()
        self.last_result: SearchResult | None = None

    def find_path(
        self, start: Position, goal: Position, blocked: Iterable[Position] = ()
    ) -> SearchResult:
        result = self._search(self.grid, start, goal, blocked)
        self.stats.add(result)
        self.last_result = result
        return result


def compare(grid: Grid, start: Position, goal: Position,
            blocked: Iterable[Position] = ()) -> dict[str, SearchResult]:
    """Run both algorithms on the same query (used by the UI comparison overlay)."""
    blocked = tuple(blocked)
    return {name: fn(grid, start, goal, blocked) for name, fn in SEARCH_FUNCTIONS.items()}
