"""Shared fixtures and helpers for the test suite."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

from environment.grid import Grid  # noqa: E402
from environment.scenario_generator import ParcelSpec, Scenario  # noqa: E402


def make_scenario(rows: list[str], parcels: list[tuple[tuple[int, int], int]] | None = None,
                  robots: list[tuple[int, int]] | None = None,
                  stations: list[tuple[int, int]] | None = None, name: str = "test") -> Scenario:
    """Build a scenario from a text map.

    Map characters: ``#`` shelf, ``D`` station, ``R`` robot start, ``P`` parcel
    (destination = station 0 unless ``parcels`` is given explicitly).
    """
    grid = Grid.from_strings(rows)
    st, rb, pc = [], [], []
    for r, row in enumerate(rows):
        for c, ch in enumerate(row):
            if ch == "D":
                st.append((r, c))
            elif ch == "R":
                rb.append((r, c))
            elif ch == "P":
                pc.append(((r, c), 0))
    return Scenario(
        scenario_id=name, name=name, seed=0, grid=grid,
        stations=stations if stations is not None else st,
        parcels=[ParcelSpec(p, d) for p, d in (parcels if parcels is not None else pc)],
        robot_starts=robots if robots is not None else rb,
    )


@pytest.fixture
def open_grid() -> Grid:
    return Grid.empty(8, 6)


@pytest.fixture
def maze_grid() -> Grid:
    return Grid.from_strings([
        "..........",
        ".########.",
        ".#......#.",
        ".#.####.#.",
        ".#....#.#.",
        "...#..#...",
        "####..####",
    ])
