"""Agent tests: reflex rules, goal-based planning/replanning, utility computation."""

from __future__ import annotations

import pytest

from agents import create_agent
from agents.base_agent import FREE, ROBOT, SHELF, WALL, Percept
from agents.goal_based import GoalBasedAgent
from agents.simple_reflex import SimpleReflexAgent
from agents.utility_based import UtilityBasedAgent
from algorithms.pathfinder import Pathfinder
from config.settings import SimulationConfig, UtilityWeights
from environment.grid import MOVE_ACTIONS, Action, Grid, apply_move
from environment.robot import RobotState
from simulation.simulation_engine import SimulationEngine
from tests.conftest import make_scenario


def percept_for(grid: Grid, pos, target, state=RobotState.NAVIGATING_TO_PARCEL,
                others=None, heading=Action.DOWN, blocked_streak=0, carrying=None) -> Percept:
    others = others or {}
    occupied = {p: rid for rid, p in others.items()}
    neighbors, neighbor_robot = {}, {}
    for a in MOVE_ACTIONS:
        c = apply_move(pos, a)
        if not grid.in_bounds(c):
            neighbors[a] = WALL
        elif grid.is_shelf(c):
            neighbors[a] = SHELF
        elif c in occupied:
            neighbors[a] = ROBOT
            neighbor_robot[a] = occupied[c]
        else:
            neighbors[a] = FREE
    return Percept(0, 1, pos, state, carrying, target, "parcel" if target else None,
                   "P0" if target else "", heading, neighbors, neighbor_robot, others, blocked_streak)


def agent(cls, grid, alg="astar", **kw):
    return cls(0, grid, Pathfinder(grid, alg), seed=1, **kw)


# ------------------------------------------------------------ common rules
@pytest.mark.parametrize("cls", [SimpleReflexAgent, GoalBasedAgent, UtilityBasedAgent])
def test_pickup_and_deliver_rules(cls, open_grid):
    a = agent(cls, open_grid)
    d = a.decide(percept_for(open_grid, (1, 1), (1, 1), state=RobotState.PICKING_UP))
    assert d.action == Action.PICKUP and "pickup" in d.reason.lower()
    d = a.decide(percept_for(open_grid, (2, 2), (2, 2), state=RobotState.DELIVERING, carrying=3))
    assert d.action == Action.DELIVER
    d = a.decide(percept_for(open_grid, (2, 2), None, state=RobotState.IDLE))
    assert d.action == Action.WAIT


# ------------------------------------------------------------ simple reflex
def test_reflex_greedy_move_toward_target(open_grid):
    a = agent(SimpleReflexAgent, open_grid)
    d = a.decide(percept_for(open_grid, (0, 0), (4, 1)))
    assert d.action == Action.DOWN and "greedy" in d.reason.lower()
    d = a.decide(percept_for(open_grid, (0, 0), (1, 5)))
    assert d.action == Action.RIGHT  # larger offset on the column axis


def test_reflex_blocked_chooses_other_valid_move():
    g = Grid.from_strings(["...", ".#.", "..."])
    a = agent(SimpleReflexAgent, g)
    d = a.decide(percept_for(g, (0, 1), (2, 1), heading=Action.RIGHT))
    assert d.action in (Action.LEFT, Action.RIGHT)  # either free side step is legal
    assert "blocked" in d.reason.lower()


def test_reflex_avoids_robot_and_waits_when_boxed_in():
    g = Grid.from_strings(["#.#", "#.#", "###"])
    a = agent(SimpleReflexAgent, g)
    d = a.decide(percept_for(g, (1, 1), (0, 1), others={1: (0, 1)}))
    assert d.action == Action.WAIT


def test_reflex_does_not_search(open_grid):
    a = agent(SimpleReflexAgent, open_grid)
    for _ in range(5):
        a.decide(percept_for(open_grid, (0, 0), (5, 7)))
    assert a.pathfinder.stats.searches == 0


# ------------------------------------------------------------ goal based
def test_goal_based_plans_shortest_route(maze_grid):
    a = agent(GoalBasedAgent, maze_grid)
    d = a.decide(percept_for(maze_grid, (0, 0), (4, 4)))
    assert d.search is not None and d.search.found
    assert len(d.plan) == d.search.cost + 1 and d.plan[0] == (0, 0)
    assert "computed a shortest route" in d.reason


@pytest.mark.parametrize("alg", ["bfs", "astar"])
def test_goal_based_follows_plan_without_replanning(open_grid, alg):
    a = agent(GoalBasedAgent, open_grid, alg)
    pos = (0, 0)
    for _ in range(12):
        d = a.decide(percept_for(open_grid, pos, (5, 7)))
        pos = apply_move(pos, d.action)
    assert pos == (5, 7)
    assert a.pathfinder.stats.searches == 1


def test_goal_based_waits_then_replans_when_blocked():
    g = Grid.empty(5, 3)
    a = agent(GoalBasedAgent, g)
    a.decide(percept_for(g, (1, 0), (1, 4)))
    first_next = a.plan[0]
    blocker = {1: first_next}
    d = a.decide(percept_for(g, (1, 0), (1, 4), others=blocker, blocked_streak=0))
    assert d.action == Action.WAIT and "Waiting" in d.reason
    d = a.decide(percept_for(g, (1, 0), (1, 4), others=blocker, blocked_streak=1))
    assert d.action.is_move and apply_move((1, 0), d.action) != first_next
    assert "Replanned" in d.reason


def test_goal_based_unreachable_goal():
    g = Grid.from_strings([".#.", ".#.", ".#."])
    a = agent(GoalBasedAgent, g)
    d = a.decide(percept_for(g, (0, 0), (0, 2)))
    assert d.action == Action.WAIT and "unreachable" in d.reason


def test_goal_update_after_pickup():
    sc = make_scenario(["R.....", "......", "P....D", "R....."])
    eng = SimulationEngine(sc, SimulationConfig(agent_type="goal_based", num_robots=2))
    targets = []
    while not eng.done:
        eng.step()
        r0 = eng.warehouse.robots[0]
        targets.append(r0.destination)
    assert (2, 0) in targets and (2, 5) in targets
    assert targets.index((2, 0)) < targets.index((2, 5))


# ------------------------------------------------------------ utility based
def test_utility_formula_matches_weights(open_grid):
    w = UtilityWeights(progress=3, distance=2, move=1, obstacle=0.5, conflict=10,
                       congestion=0.25, wait=4, revisit=0.1)
    a = agent(UtilityBasedAgent, open_grid, weights=w)
    p = percept_for(open_grid, (0, 0), (0, 3))
    f = a.features(p, Action.RIGHT)
    assert f["progress"] == 1 and f["distance"] == 2 and f["move_cost"] == 1
    expected = 3 * 1 - 2 * 2 - 1 * 1 - 0.5 * f["obstacle"] - 0 - 0 - 0 - 0
    assert a.utility(f) == pytest.approx(expected)
    fw = a.features(p, Action.WAIT)
    assert fw["wait"] == 1 and fw["move_cost"] == 0 and fw["progress"] == 0


def test_utility_chooses_highest_scoring_action(open_grid):
    a = agent(UtilityBasedAgent, open_grid)
    d = a.decide(percept_for(open_grid, (2, 2), (2, 6)))
    assert d.action == Action.RIGHT
    best = max((c for c in d.candidates if c.valid), key=lambda c: c.score)
    assert best.action == d.action
    assert "highest utility" in d.reason
    assert set(d.features) >= {"RIGHT", "WAIT"}


def test_utility_excludes_shelves_and_penalises_conflicts():
    g = Grid.from_strings(["....", ".#..", "...."])
    a = agent(UtilityBasedAgent, g)
    p = percept_for(g, (0, 1), (2, 1), others={1: (0, 0)})
    d = a.decide(p)
    down = next(c for c in d.candidates if c.action == Action.DOWN)
    left = next(c for c in d.candidates if c.action == Action.LEFT)
    assert not down.valid  # shelf
    assert left.valid and "CONFLICT" in left.note
    assert d.action != Action.LEFT


def test_utility_congestion_feature(open_grid):
    a = agent(UtilityBasedAgent, open_grid)
    p = percept_for(open_grid, (2, 2), (2, 6), others={1: (0, 3), 2: (4, 3)})
    assert a.features(p, Action.RIGHT)["congestion"] == 2


def test_utility_wait_penalty_grows(open_grid):
    a = agent(UtilityBasedAgent, open_grid)
    a.consecutive_waits = 3
    f = a.features(percept_for(open_grid, (2, 2), (2, 6)), Action.WAIT)
    assert f["wait"] == 4


def test_factory_and_unknown_agent(open_grid):
    pf = Pathfinder(open_grid, "bfs")
    assert isinstance(create_agent("utility_based", 0, open_grid, pf), UtilityBasedAgent)
    with pytest.raises(ValueError):
        create_agent("learning_agent", 0, open_grid, pf)


@pytest.mark.parametrize("agent_type", ["simple_reflex", "goal_based", "utility_based"])
def test_every_agent_works_in_same_environment(agent_type):
    sc = make_scenario([
        "R....R",
        ".##.#.",
        ".P..P.",
        ".##.#.",
        "..D..D",
    ])
    eng = SimulationEngine(sc, SimulationConfig(agent_type=agent_type, num_robots=2, seed=3))
    eng.run()
    m = eng.metrics()
    assert m["parcels_delivered"] >= 1
    if agent_type != "simple_reflex":
        assert m["completed"]
