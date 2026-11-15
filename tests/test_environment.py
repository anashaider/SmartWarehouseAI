"""Environment tests: grid, movement, shelves, parcels, stations, robots, layouts."""

from __future__ import annotations

import json

import pytest

from config.settings import PREDEFINED_LAYOUTS
from environment.grid import Action, Cell, Grid, apply_move, action_between, manhattan
from environment.parcel import InvalidParcelTransition, Parcel, ParcelStatus
from environment.robot import InvalidStateTransition, Robot, RobotState
from environment.scenario_generator import (
    SIZE_PRESETS,
    ScenarioError,
    generate_random_scenario,
    load_layout_file,
    load_predefined,
    save_scenario,
    scenario_from_dict,
    validate_scenario,
)
from environment.warehouse import Warehouse, WarehouseInvariantError
from tests.conftest import make_scenario


# ------------------------------------------------------------------ grid
def test_grid_initialisation():
    g = Grid.empty(7, 4)
    assert (g.width, g.height) == (7, 4)
    assert len(g.walkable_cells()) == 28
    assert g.shelf_cells() == []


def test_grid_from_strings_marks_shelves():
    g = Grid.from_strings(["..#", "#.."])
    assert g.is_shelf((0, 2)) and g.is_shelf((1, 0))
    assert g.cells[0, 0] == Cell.FLOOR
    with pytest.raises(ValueError):
        Grid.from_strings(["...", ".."])


def test_bounds_and_walkability(open_grid):
    assert open_grid.in_bounds((0, 0)) and not open_grid.in_bounds((-1, 0))
    assert not open_grid.is_walkable((6, 0))  # row 6 is outside a 6-row grid
    open_grid.set_shelf((2, 2))
    assert not open_grid.is_walkable((2, 2))


def test_neighbors_fixed_order_and_shelf_exclusion():
    g = Grid.from_strings(["...", ".#.", "..."])
    assert list(g.neighbors((0, 1))) == [(0, 0), (0, 2)]  # down is a shelf, up is outside
    assert list(g.neighbors((1, 0))) == [(0, 0), (2, 0)]
    assert g.blocked_neighbor_count((0, 0)) == 2


def test_move_helpers():
    assert apply_move((3, 3), Action.UP) == (2, 3)
    assert apply_move((3, 3), Action.RIGHT) == (3, 4)
    assert action_between((3, 3), (4, 3)) == Action.DOWN
    with pytest.raises(ValueError):
        action_between((0, 0), (1, 1))
    assert manhattan((0, 0), (3, 4)) == 7


# ------------------------------------------------------------------ parcels
def test_parcel_life_cycle_and_no_reuse():
    p = Parcel(0, (1, 1), destination_id=0)
    p.assign(5)
    with pytest.raises(InvalidParcelTransition):
        p.assign(6)  # never assigned to two robots
    with pytest.raises(InvalidParcelTransition):
        p.pick_up(6, 1)  # only the assigned robot may pick it up
    p.pick_up(5, 1)
    p.deliver(5, (9, 9), 7)
    assert p.status == ParcelStatus.DELIVERED and p.delivered_step == 7
    for action in (lambda: p.assign(1), p.unassign, lambda: p.pick_up(5, 8)):
        with pytest.raises(InvalidParcelTransition):
            action()


# ------------------------------------------------------------------ robots
def test_robot_valid_state_sequence():
    r = Robot(0, (0, 0))
    for st in (RobotState.NAVIGATING_TO_PARCEL, RobotState.PICKING_UP,
               RobotState.NAVIGATING_TO_DROPOFF, RobotState.DELIVERING, RobotState.IDLE):
        r.transition(st)
    assert r.state == RobotState.IDLE


@pytest.mark.parametrize("bad", [RobotState.PICKING_UP, RobotState.DELIVERING,
                                 RobotState.NAVIGATING_TO_DROPOFF])
def test_robot_rejects_invalid_transition(bad):
    with pytest.raises(InvalidStateTransition):
        Robot(0, (0, 0)).transition(bad)


# ------------------------------------------------------------------ warehouse
def small_warehouse():
    sc = make_scenario([
        "R.R",
        ".#.",
        "P.D",
    ])
    return Warehouse.from_scenario(sc, 2)


def test_robot_positioning_and_move():
    wh = small_warehouse()
    r0, r1 = wh.robots
    assert r0.position == (0, 0) and r1.position == (0, 2)
    wh.move_robot(r0, (0, 1))
    assert r0.position == (0, 1) and r0.distance_travelled == 1
    with pytest.raises(WarehouseInvariantError):
        wh.move_robot(r0, (1, 1))  # shelf collision
    with pytest.raises(WarehouseInvariantError):
        wh.move_robot(r0, (0, 2))  # occupied by r1
    with pytest.raises(WarehouseInvariantError):
        wh.move_robot(r1, (2, 2))  # not adjacent


def test_full_pickup_and_delivery_cycle():
    wh = small_warehouse()
    r0 = wh.robots[0]
    parcel = wh.parcels[0]
    wh.assign(r0, parcel)
    assert r0.state == RobotState.NAVIGATING_TO_PARCEL
    assert not wh.pick_up(r0, 1)  # not on the parcel yet -> invalid action
    wh.move_robot(r0, (1, 0))
    wh.move_robot(r0, (2, 0))
    assert r0.state == RobotState.PICKING_UP
    assert wh.pick_up(r0, 3)
    assert r0.state == RobotState.NAVIGATING_TO_DROPOFF and r0.carrying == 0
    wh.move_robot(r0, (2, 1))
    wh.move_robot(r0, (2, 2))
    assert r0.state == RobotState.DELIVERING
    assert wh.deliver(r0, 5)
    assert r0.state == RobotState.IDLE and r0.deliveries_completed == 1
    assert parcel.status == ParcelStatus.DELIVERED and wh.all_delivered()
    assert wh.waiting_parcels() == []  # delivered parcel is never available again


def test_parcel_cannot_be_assigned_to_two_robots():
    wh = small_warehouse()
    wh.assign(wh.robots[0], wh.parcels[0])
    with pytest.raises(InvalidParcelTransition):
        wh.assign(wh.robots[1], wh.parcels[0])


def test_invariants_detect_shared_cells():
    wh = small_warehouse()
    wh.robots[1].position = wh.robots[0].position
    with pytest.raises(WarehouseInvariantError):
        wh.check_invariants()


def test_robots_cannot_spawn_inside_shelves():
    sc = make_scenario(["R#", "PD"], robots=[(0, 1)])
    with pytest.raises(WarehouseInvariantError):
        Warehouse.from_scenario(sc, 1)


# ------------------------------------------------------------------ validation
def test_validation_catches_bad_layouts():
    sc = make_scenario(["R.#", "###", "P.D"])  # station/parcel unreachable
    with pytest.raises(ScenarioError, match="not accessible|not reachable"):
        validate_scenario(sc)
    sc2 = make_scenario(["R..", "...", "..D"], parcels=[((0, 2), 0)], robots=[(0, 0)])
    sc2.parcels.append(sc2.parcels[0])
    with pytest.raises(ScenarioError, match="share"):
        validate_scenario(sc2)


def test_parcel_inside_obstacle_rejected():
    sc = make_scenario(["R.#", "...", "..D"], parcels=[((0, 2), 0)])
    with pytest.raises(ScenarioError, match="obstacle"):
        validate_scenario(sc)


@pytest.mark.parametrize("name", PREDEFINED_LAYOUTS)
def test_predefined_layouts_load_and_validate(name):
    sc = load_predefined(name)
    assert "warning" not in sc.metadata
    validate_scenario(sc, min_robots=3)
    assert len(sc.robot_starts) >= 3 and len(sc.parcels) >= 6


def test_layout_sizes_increase():
    sizes = [load_predefined(n).grid.cells.size for n in PREDEFINED_LAYOUTS]
    assert sizes == sorted(sizes) and len(set(sizes)) == 3


def test_corrupted_layout_falls_back(tmp_path):
    (tmp_path / "small.json").write_text("{not json", encoding="utf-8")
    sc = load_predefined("small", layout_dir=tmp_path)
    assert "warning" in sc.metadata and "corrupted" in sc.metadata["warning"]
    with pytest.raises(ScenarioError):
        load_layout_file(tmp_path / "missing.json")


def test_unknown_map_characters_rejected():
    with pytest.raises(ScenarioError, match="Unknown map characters"):
        scenario_from_dict({"map": ["RRR", "P?D"]})


def test_save_and_reload_roundtrip(tmp_path):
    sc = generate_random_scenario(9, "small")
    path = tmp_path / "saved.json"
    save_scenario(sc, path)
    again = load_layout_file(path)
    assert again.grid.to_strings() == sc.grid.to_strings()
    assert [p.position for p in again.parcels] == [p.position for p in sc.parcels]
    assert [p.destination_id for p in again.parcels] == [p.destination_id for p in sc.parcels]
    assert json.loads(path.read_text())["seed"] == 9


@pytest.mark.parametrize("size", list(SIZE_PRESETS))
def test_random_generation_is_valid_and_seeded(size):
    for seed in range(10):
        a = generate_random_scenario(seed, size)
        validate_scenario(a, min_robots=3)
        b = generate_random_scenario(seed, size)
        assert a.grid.to_strings() == b.grid.to_strings()
        assert a.parcels == b.parcels and a.robot_starts == b.robot_starts
    assert generate_random_scenario(1, size).grid.to_strings() != \
        generate_random_scenario(2, size).grid.to_strings()
