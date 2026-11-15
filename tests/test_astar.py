"""A* correctness tests (optimality, heuristic properties, efficiency vs BFS)."""

from __future__ import annotations

import itertools

import pytest

from algorithms.astar import astar
from algorithms.bfs import bfs
from algorithms.pathfinder import Pathfinder, compare, get_search_function
from environment.grid import Grid, manhattan
from environment.scenario_generator import generate_random_scenario, load_predefined
from tests.test_bfs import assert_valid_path, reference_distance


def test_open_grid_optimal(open_grid):
    res = astar(open_grid, (0, 0), (5, 7))
    assert res.cost == 12
    assert_valid_path(open_grid, res.path, (0, 0), (5, 7))


def test_straight_line_expands_only_path_cells(open_grid):
    res = astar(open_grid, (0, 0), (0, 7))
    assert res.cost == 7
    assert res.nodes_expanded == 7  # perfectly informed heuristic on an open row


def test_obstacles_and_unreachable(maze_grid):
    res = astar(maze_grid, (0, 0), (4, 4))
    assert res.cost == reference_distance(maze_grid, (0, 0), (4, 4))
    g = Grid.from_strings([".#.", ".#.", ".#."])
    res2 = astar(g, (0, 0), (0, 2))
    assert not res2.found and res2.cost is None


def test_blocked_cells_and_occupied_goal():
    g = Grid.empty(3, 3)
    res = astar(g, (0, 0), (0, 2), blocked=[(0, 1)])
    assert res.cost == 4 and (0, 1) not in res.path
    assert not astar(g, (0, 0), (0, 2), blocked=[(0, 2)]).found


def test_start_equals_goal(open_grid):
    assert astar(open_grid, (1, 1), (1, 1)).path == [(1, 1)]


def test_manhattan_heuristic_admissible_and_consistent():
    sc = load_predefined("large")
    grid = sc.grid
    goal = sc.stations[0]
    for cell in grid.walkable_cells()[::5]:
        true = reference_distance(grid, cell, goal)
        if true is None:
            continue
        assert manhattan(cell, goal) <= true  # admissible
        for n in grid.neighbors(cell):
            assert manhattan(cell, goal) <= 1 + manhattan(n, goal)  # consistent


@pytest.mark.parametrize("size,seed", [(s, k) for s in ("small", "medium", "large") for k in range(3)])
def test_astar_matches_bfs_cost_and_expands_no_more(size, seed):
    sc = generate_random_scenario(seed, size)
    cells = sc.grid.walkable_cells()
    total_a = total_b = 0
    for a, b in itertools.islice(itertools.combinations(cells[::9], 2), 40):
        ra, rb = astar(sc.grid, a, b), bfs(sc.grid, a, b)
        assert ra.cost == rb.cost
        if ra.found:
            assert_valid_path(sc.grid, ra.path, a, b)
        total_a += ra.nodes_expanded
        total_b += rb.nodes_expanded
    assert total_a <= total_b


def test_deterministic_tie_breaking(open_grid):
    assert astar(open_grid, (0, 0), (4, 4)).path == astar(open_grid, (0, 0), (4, 4)).path


def test_pathfinder_accumulates_stats(open_grid):
    pf = Pathfinder(open_grid, "astar")
    pf.find_path((0, 0), (2, 2))
    pf.find_path((0, 0), (5, 7))
    open_grid.set_shelf((3, 3))
    pf.find_path((0, 0), (3, 3))
    assert pf.stats.searches == 3 and pf.stats.failures == 1
    assert pf.stats.nodes_expanded > 0 and pf.last_result is not None
    with pytest.raises(ValueError):
        get_search_function("dijkstra")


def test_compare_runs_both_with_trace(maze_grid):
    res = compare(maze_grid, (0, 0), (4, 4))
    assert set(res) == {"bfs", "astar"}
    assert res["bfs"].cost == res["astar"].cost
    assert len(res["astar"].explored) == res["astar"].nodes_expanded
