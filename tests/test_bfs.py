"""BFS correctness tests."""

from __future__ import annotations

import itertools
from collections import deque

import pytest

from algorithms.bfs import bfs
from environment.grid import Grid, manhattan
from environment.scenario_generator import generate_random_scenario


def reference_distance(grid: Grid, start, goal, blocked=()):
    """Independent distance computation (plain flood fill) used as an oracle."""
    if start == goal:
        return 0
    seen = {start: 0}
    q = deque([start])
    while q:
        cur = q.popleft()
        for n in grid.neighbors(cur):
            if n not in seen and n not in blocked:
                seen[n] = seen[cur] + 1
                if n == goal:
                    return seen[n]
                q.append(n)
    return None


def assert_valid_path(grid: Grid, path, start, goal, blocked=()):
    assert path[0] == start and path[-1] == goal
    for a, b in zip(path, path[1:]):
        assert manhattan(a, b) == 1
        assert grid.is_walkable(b) and b not in blocked
    assert len(set(path)) == len(path)  # no cycles


def test_open_grid_shortest_path(open_grid):
    res = bfs(open_grid, (0, 0), (5, 7))
    assert res.found and res.cost == 12
    assert_valid_path(open_grid, res.path, (0, 0), (5, 7))
    assert res.nodes_expanded > 0 and res.runtime_ms >= 0


def test_start_equals_goal(open_grid):
    res = bfs(open_grid, (2, 2), (2, 2))
    assert res.path == [(2, 2)] and res.cost == 0


def test_obstacle_avoidance(maze_grid):
    res = bfs(maze_grid, (0, 0), (4, 4))
    assert res.found
    assert_valid_path(maze_grid, res.path, (0, 0), (4, 4))
    assert res.cost == reference_distance(maze_grid, (0, 0), (4, 4))


def test_unreachable_destination():
    g = Grid.from_strings(["..#..", "..#..", "..#.."])
    res = bfs(g, (0, 0), (0, 4))
    assert not res.found and res.path == [] and res.cost is None
    assert res.nodes_expanded == 6  # the whole left region was explored


def test_goal_on_shelf_or_outside(open_grid):
    open_grid.set_shelf((3, 3))
    assert not bfs(open_grid, (0, 0), (3, 3)).found
    assert not bfs(open_grid, (0, 0), (99, 99)).found


def test_blocked_cells_respected():
    g = Grid.from_strings(["...", "...", "..."])
    res = bfs(g, (0, 0), (0, 2), blocked={(0, 1)})
    assert res.cost == 4 and (0, 1) not in res.path
    assert not bfs(g, (0, 0), (0, 2), blocked={(0, 2)}).found  # occupied goal


def test_path_reconstruction_parent_chain(maze_grid):
    res = bfs(maze_grid, (5, 0), (5, 9))
    assert_valid_path(maze_grid, res.path, (5, 0), (5, 9))
    assert len(res.path) == res.cost + 1


@pytest.mark.parametrize("seed", range(5))
def test_optimal_against_oracle_on_generated_layouts(seed):
    sc = generate_random_scenario(seed, "medium")
    cells = sc.grid.walkable_cells()
    for a, b in itertools.islice(itertools.combinations(cells[::7], 2), 60):
        res = bfs(sc.grid, a, b)
        assert res.cost == reference_distance(sc.grid, a, b)
        if res.found:
            assert_valid_path(sc.grid, res.path, a, b)


def test_trace_records_expansion_order(open_grid):
    res = bfs(open_grid, (0, 0), (3, 3), trace=True)
    assert len(res.explored) == res.nodes_expanded and res.explored[0] == (0, 0)
