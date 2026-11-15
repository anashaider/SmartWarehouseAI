"""Breadth-First Search on the warehouse grid (implemented from scratch).

* Frontier: FIFO queue (``collections.deque``).
* Visited set: a cell is marked when it is *generated*, so it enters the
  queue at most once.
* Goal test: applied when a node is generated (early goal test, AIMA Fig. 3.9).
  On a grid with unit step costs this still returns a shortest path because
  BFS generates nodes in non-decreasing depth order.
* ``nodes_expanded`` counts nodes removed from the queue whose successors were
  generated.
"""

from __future__ import annotations

import time
from collections import deque
from typing import Iterable, Optional

from algorithms.common import SearchResult, reconstruct_path
from environment.grid import Grid, Position


def bfs(
    grid: Grid,
    start: Position,
    goal: Position,
    blocked: Iterable[Position] = (),
) -> SearchResult:
    """Shortest path from ``start`` to ``goal`` avoiding shelves and ``blocked`` cells.

    ``blocked`` typically holds cells occupied by other robots. The start cell
    is never treated as blocked; a blocked goal makes the goal unreachable.
    """
    t0 = time.perf_counter()
    result = SearchResult("bfs", start, goal)
    blocked_set = set(blocked)
    blocked_set.discard(start)

    if not grid.is_walkable(start) or not grid.is_walkable(goal) or goal in blocked_set:
        result.runtime_ms = (time.perf_counter() - t0) * 1000
        return result
    if start == goal:
        result.path = [start]
        result.runtime_ms = (time.perf_counter() - t0) * 1000
        return result

    parent: dict[Position, Optional[Position]] = {start: None}
    frontier: deque[Position] = deque([start])
    result.nodes_generated = 1

    while frontier:
        node = frontier.popleft()
        result.nodes_expanded += 1
        for child in grid.neighbors(node):
            if child in parent or child in blocked_set:
                continue
            parent[child] = node
            result.nodes_generated += 1
            if child == goal:
                result.path = reconstruct_path(parent, goal)
                result.runtime_ms = (time.perf_counter() - t0) * 1000
                return result
            frontier.append(child)

    result.runtime_ms = (time.perf_counter() - t0) * 1000
    return result
