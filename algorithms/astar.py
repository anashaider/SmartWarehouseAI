"""A* search on the warehouse grid (implemented from scratch).

f(n) = g(n) + h(n), with h(n) = |r1 - r2| + |c1 - c2| (Manhattan distance).

With 4-directional moves of unit cost, Manhattan distance never overestimates
the true remaining cost (admissible) and satisfies h(n) <= 1 + h(n') for every
neighbour n' (consistent). With a consistent heuristic a node's g-score is
optimal when it is first popped, so A* with a closed set returns a shortest
path.

* Open set: binary heap of ``(f, h, insertion_counter, node)``. Ties on f are
  broken toward the smaller h (deeper nodes), then FIFO, which is deterministic.
* Closed set: nodes already expanded; stale heap entries are skipped.
* ``nodes_expanded`` counts nodes popped from the open set and expanded.
"""

from __future__ import annotations

import heapq
import itertools
import time
from typing import Iterable, Optional

from algorithms.common import SearchResult, reconstruct_path
from environment.grid import Grid, Position, manhattan


def astar(
    grid: Grid,
    start: Position,
    goal: Position,
    blocked: Iterable[Position] = (),
) -> SearchResult:
    """Shortest path from ``start`` to ``goal`` using A* with the Manhattan heuristic."""
    t0 = time.perf_counter()
    result = SearchResult("astar", start, goal)
    blocked_set = set(blocked)
    blocked_set.discard(start)

    if not grid.is_walkable(start) or not grid.is_walkable(goal) or goal in blocked_set:
        result.runtime_ms = (time.perf_counter() - t0) * 1000
        return result

    counter = itertools.count()
    h0 = manhattan(start, goal)
    open_heap: list[tuple[int, int, int, Position]] = [(h0, h0, next(counter), start)]
    g_score: dict[Position, int] = {start: 0}
    parent: dict[Position, Optional[Position]] = {start: None}
    closed: set[Position] = set()
    result.nodes_generated = 1

    while open_heap:
        _f, _h, _, node = heapq.heappop(open_heap)
        if node in closed:
            continue  # stale entry superseded by a cheaper path
        if node == goal:
            result.path = reconstruct_path(parent, goal)
            break
        closed.add(node)
        result.nodes_expanded += 1
        g_next = g_score[node] + 1
        for child in grid.neighbors(node):
            if child in closed or child in blocked_set:
                continue
            if g_next < g_score.get(child, 1 << 30):
                g_score[child] = g_next
                parent[child] = node
                h = manhattan(child, goal)
                heapq.heappush(open_heap, (g_next + h, h, next(counter), child))
                result.nodes_generated += 1

    result.runtime_ms = (time.perf_counter() - t0) * 1000
    return result
