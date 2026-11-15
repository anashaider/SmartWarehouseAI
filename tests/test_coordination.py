"""Multi-robot coordination tests: conflicts, swaps, waiting, deadlocks, allocation."""

from __future__ import annotations

import pytest

from config.settings import SimulationConfig
from coordination.conflict_manager import ConflictManager, MoveOutcome
from coordination.reservations import ReservationTable
from coordination.task_allocator import TaskAllocator
from environment.grid import Action
from environment.parcel import ParcelStatus
from environment.scenario_generator import build_experiment_scenarios, load_predefined
from environment.warehouse import Warehouse
from simulation.simulation_engine import SimulationEngine
from tests.conftest import make_scenario


def corridor_warehouse():
    sc = make_scenario(["R...R", "#####", "..PD."], robots=[(0, 0), (0, 4)])
    return Warehouse.from_scenario(sc, 2)


def test_reservation_table_vertex_and_edge():
    t = ReservationTable()
    t.reserve(0, (0, 0), (0, 1))
    assert t.is_reserved((0, 1)) and t.holder((0, 1)) == 0
    assert t.swap_conflict((0, 1), (0, 0), robot_id=1) == 0
    with pytest.raises(ValueError):
        t.reserve(1, (1, 1), (0, 1))
    t.clear()
    assert not t.is_reserved((0, 1))


def test_same_cell_conflict_detected():
    wh = corridor_warehouse()
    r0, r1 = wh.robots
    wh.robots[1].position = (0, 2)
    cm = ConflictManager(wh)
    cm.begin_tick()
    cm.commit(r1, (0, 2), (0, 1))  # r1 already claimed (0,1) this tick
    check = cm.check_move(r0, Action.RIGHT)
    assert check.outcome == MoveOutcome.VERTEX_CONFLICT and check.other_robot == 1


def test_occupied_cell_conflict():
    wh = corridor_warehouse()
    wh.robots[1].position = (0, 1)
    cm = ConflictManager(wh)
    cm.begin_tick()
    assert cm.check_move(wh.robots[0], Action.RIGHT).outcome == MoveOutcome.VERTEX_CONFLICT


def test_position_swap_prevented():
    wh = corridor_warehouse()
    r0, r1 = wh.robots
    r1.position = (0, 1)
    cm = ConflictManager(wh)
    cm.begin_tick()
    # r1 has reserved the edge (0,1) -> (0,0) this tick; r0 at (0,0) now tries the
    # reverse edge (0,0) -> (0,1): the two robots would pass through each other.
    cm.commit(r1, (0, 1), (0, 0))
    check = cm.check_move(r0, Action.RIGHT)
    assert check.outcome == MoveOutcome.SWAP_CONFLICT


def test_invalid_move_into_shelf():
    wh = corridor_warehouse()
    cm = ConflictManager(wh)
    cm.begin_tick()
    assert cm.check_move(wh.robots[0], Action.DOWN).outcome == MoveOutcome.INVALID
    assert cm.check_move(wh.robots[0], Action.UP).outcome == MoveOutcome.INVALID


def test_turn_order_rotates():
    wh = Warehouse.from_scenario(load_predefined("small"), 3)
    cm = ConflictManager(wh)
    assert cm.turn_order(0) == [0, 1, 2]
    assert cm.turn_order(1) == [1, 2, 0]
    assert cm.turn_order(5) == [2, 0, 1]


def test_rejected_move_is_counted_and_robot_waits():
    sc = make_scenario(["R.R", "#.#", "PD."], robots=[(0, 0), (0, 2)])
    eng = SimulationEngine(sc, SimulationConfig(agent_type="goal_based", num_robots=2))
    eng.step()  # allocation + first moves
    wh = eng.warehouse
    # Force robot 1 next to robot 0 and make robot 0 attempt to enter its cell.
    r0, r1 = wh.robots
    r0.position, r1.position = (0, 0), (0, 1)
    from agents.base_agent import Decision
    percept = eng.build_percept(r0)
    eng.conflicts.begin_tick()
    eng._execute(r0, percept, Decision(Action.RIGHT, "test"), [])
    assert r0.position == (0, 0)
    assert eng.rejected_moves == 1 and r0.current_action == Action.WAIT


def test_head_on_corridor_deadlock_is_resolved():
    # Dead-end aisle: robot 0 must get out while robot 1 wants in.
    sc = make_scenario([
        "R.R....",
        ".......",
        "#.....#",
        "#P#####",
        "#.#####",
        "#D#####",
    ], parcels=[((3, 1), 0)], robots=[(0, 0), (0, 2)])
    for agent_type in ("goal_based", "utility_based"):
        eng = SimulationEngine(sc, SimulationConfig(agent_type=agent_type, num_robots=2), max_steps=200)
        eng.run()
        assert eng.metrics()["completed"], agent_type


def test_known_deadlock_scenario_completes():
    """Scenario S22 originally livelocked in a dead-end aisle (regression test)."""
    sc = build_experiment_scenarios(22, 2026)[21]
    eng = SimulationEngine(sc, SimulationConfig(agent_type="goal_based", num_robots=2, seed=sc.seed))
    eng.run()
    m = eng.metrics()
    assert m["completed"] and m["deadlocks_resolved"] >= 1


def test_idle_robot_blocking_station_yields():
    # Robot 1 has nothing to do and starts on the only station.
    sc = make_scenario(["R.P..", ".....", "..D.."], parcels=[((0, 2), 0)],
                       robots=[(0, 0), (2, 2)])
    eng = SimulationEngine(sc, SimulationConfig(agent_type="goal_based", num_robots=2), max_steps=100)
    eng.run()
    assert eng.metrics()["completed"]
    assert any(e.kind == "yield" for e in eng.event_log)


def test_waiting_behaviour_records_blocker():
    sc = make_scenario(["R.R", "#.#", "#P#", "#D#"], parcels=[((2, 1), 0)], robots=[(0, 0), (0, 2)])
    eng = SimulationEngine(sc, SimulationConfig(agent_type="goal_based", num_robots=2), max_steps=100)
    eng.run()
    assert eng.metrics()["completed"]


def test_task_allocation_no_duplicates_and_policies():
    sc = make_scenario(["R.P.P", ".....", "R.P.D"])
    wh = Warehouse.from_scenario(sc, 2)
    alloc = TaskAllocator(wh, "nearest")
    pairs = alloc.allocate()
    assert len(pairs) == 2 and len({p for _, p in pairs}) == 2
    assert alloc.allocate() == []  # busy robots get nothing new
    assigned = [p for p in wh.parcels.values() if p.status == ParcelStatus.ASSIGNED]
    assert len(assigned) == 2
    # Fixed-order policy takes the lowest parcel id regardless of distance.
    wh2 = Warehouse.from_scenario(sc, 2)
    TaskAllocator(wh2, "fixed_order").allocate()
    assert wh2.robots[0].assigned_parcel == 0 and wh2.robots[1].assigned_parcel == 1
    with pytest.raises(ValueError):
        TaskAllocator(wh2, "auction")


def test_nearest_policy_picks_closest():
    sc = make_scenario(["R...P", ".....", "P...D"], robots=[(2, 1)])
    wh = Warehouse.from_scenario(sc, 1)
    TaskAllocator(wh, "nearest").allocate()
    assert wh.parcels[wh.robots[0].assigned_parcel].position == (2, 0)


def test_unreachable_parcel_not_assigned():
    sc = make_scenario(["R..#P", "...##", "D...."])
    wh = Warehouse.from_scenario(sc, 1)
    assert TaskAllocator(wh, "nearest").allocate() == []


@pytest.mark.parametrize("agent_type", ["simple_reflex", "goal_based", "utility_based"])
@pytest.mark.parametrize("layout", ["small", "medium", "large"])
def test_no_collisions_ever(agent_type, layout):
    sc = load_predefined(layout)
    eng = SimulationEngine(sc, SimulationConfig(agent_type=agent_type, num_robots=3, seed=4),
                           max_steps=250, record_history=False)
    while not eng.done:
        eng.step()  # step() calls check_invariants() after every tick
        cells = [r.position for r in eng.warehouse.robots]
        assert len(set(cells)) == len(cells)
        for r in eng.warehouse.robots:
            assert eng.warehouse.grid.is_walkable(r.position)
