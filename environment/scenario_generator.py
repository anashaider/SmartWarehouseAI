"""Scenario definitions: predefined JSON layouts and seeded random layouts.

A :class:`Scenario` is an immutable *specification* (floor plan, station
positions, parcel pickups/destinations, robot start cells). The mutable
runtime state is created from it by :class:`environment.warehouse.Warehouse`,
which guarantees that every agent/algorithm combination starts from exactly
the same initial conditions.
"""

from __future__ import annotations

import json
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from config.settings import LAYOUT_DIR, LAYOUT_LABELS, MAX_ROBOTS
from environment.grid import Grid, Position


class ScenarioError(ValueError):
    """Raised when a layout is malformed or fails validation."""


@dataclass(frozen=True)
class ParcelSpec:
    position: Position
    destination_id: int


@dataclass
class Scenario:
    scenario_id: str
    name: str
    seed: int
    grid: Grid
    stations: list[Position]
    parcels: list[ParcelSpec]
    robot_starts: list[Position]
    size_class: str = "custom"
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def width(self) -> int:
        return self.grid.width

    @property
    def height(self) -> int:
        return self.grid.height

    @property
    def shelf_count(self) -> int:
        return len(self.grid.shelf_cells())

    def to_dict(self) -> dict[str, Any]:
        """Serialisable description (used for saved scenarios / experiment logs)."""
        return {
            "scenario_id": self.scenario_id,
            "name": self.name,
            "seed": self.seed,
            "size_class": self.size_class,
            "width": self.width,
            "height": self.height,
            "map": self.grid.to_strings(),
            "stations": [list(p) for p in self.stations],
            "parcels": [
                {"position": list(p.position), "destination": p.destination_id}
                for p in self.parcels
            ],
            "robot_starts": [list(p) for p in self.robot_starts],
        }


# --------------------------------------------------------------------------- #
# Validation
# --------------------------------------------------------------------------- #
def validate_scenario(scenario: Scenario, min_robots: int = 1) -> None:
    """Validate a scenario; raise :class:`ScenarioError` describing every problem."""
    grid = scenario.grid
    problems: list[str] = []

    if len(scenario.robot_starts) < min_robots:
        problems.append(f"needs at least {min_robots} robot start positions")
    if not scenario.stations:
        problems.append("needs at least one delivery station")
    if not scenario.parcels:
        problems.append("needs at least one parcel")

    for i, pos in enumerate(scenario.robot_starts):
        if not grid.is_walkable(pos):
            problems.append(f"robot start {i} at {pos} is inside a shelf/outside grid")
    if len(set(scenario.robot_starts)) != len(scenario.robot_starts):
        problems.append("robot start positions must be unique")

    for i, pos in enumerate(scenario.stations):
        if not grid.is_walkable(pos):
            problems.append(f"station {i} at {pos} is not on a walkable cell")

    station_set = set(scenario.stations)
    seen: set[Position] = set()
    for i, spec in enumerate(scenario.parcels):
        if not grid.is_walkable(spec.position):
            problems.append(f"parcel {i} at {spec.position} is inside an obstacle")
        if spec.position in station_set:
            problems.append(f"parcel {i} is placed on a delivery station")
        if spec.position in seen:
            problems.append(f"two parcels share cell {spec.position}")
        seen.add(spec.position)
        if not 0 <= spec.destination_id < len(scenario.stations):
            problems.append(f"parcel {i} has an unknown destination")

    if not problems and scenario.robot_starts:
        reach = grid.reachable_from(scenario.robot_starts[0])
        for i, pos in enumerate(scenario.robot_starts):
            if pos not in reach:
                problems.append(f"robot start {i} is disconnected from robot 0")
        for i, pos in enumerate(scenario.stations):
            if pos not in reach:
                problems.append(f"station {i} at {pos} is not accessible")
        for i, spec in enumerate(scenario.parcels):
            if spec.position not in reach:
                problems.append(f"parcel {i} at {spec.position} is not reachable")

    if problems:
        raise ScenarioError(f"{scenario.name}: " + "; ".join(problems))


# --------------------------------------------------------------------------- #
# JSON layouts
# --------------------------------------------------------------------------- #
LEGEND = {".": "floor", "#": "shelf", "D": "station", "R": "robot", "P": "parcel"}


def scenario_from_dict(data: dict[str, Any], scenario_id: str | None = None) -> Scenario:
    """Build a scenario from the JSON layout schema documented in ``data/layouts``."""
    try:
        rows: list[str] = list(data["map"])
        name = str(data.get("name", "Custom Layout"))
        seed = int(data.get("seed", 0))
    except (KeyError, TypeError, ValueError) as exc:
        raise ScenarioError(f"Layout is missing a valid 'map': {exc}") from exc
    if not rows or not all(isinstance(r, str) for r in rows):
        raise ScenarioError("Layout 'map' must be a non-empty list of strings")
    width = len(rows[0])
    if any(len(r) != width for r in rows):
        raise ScenarioError("All rows of the layout map must have the same length")
    unknown = {ch for row in rows for ch in row} - set(LEGEND)
    if unknown:
        raise ScenarioError(f"Unknown map characters: {sorted(unknown)}")

    grid = Grid.from_strings(rows)
    stations: list[Position] = []
    robots: list[Position] = []
    pickups: list[Position] = []
    for r, row in enumerate(rows):
        for c, ch in enumerate(row):
            if ch == "D":
                stations.append((r, c))
            elif ch == "R":
                robots.append((r, c))
            elif ch == "P":
                pickups.append((r, c))

    # Destinations: explicit list in JSON, otherwise a seeded assignment.
    explicit = data.get("destinations")
    rng = random.Random(seed)
    parcels: list[ParcelSpec] = []
    for i, pos in enumerate(pickups):
        if isinstance(explicit, list) and i < len(explicit):
            dest = int(explicit[i])
        else:
            dest = rng.randrange(len(stations)) if stations else 0
        parcels.append(ParcelSpec(pos, dest))

    scenario = Scenario(
        scenario_id=scenario_id or str(data.get("id", name.lower().replace(" ", "_"))),
        name=name,
        seed=seed,
        grid=grid,
        stations=stations,
        parcels=parcels,
        robot_starts=robots,
        size_class=str(data.get("size_class", "custom")),
        metadata={"description": data.get("description", "")},
    )
    validate_scenario(scenario, min_robots=MAX_ROBOTS)
    return scenario


def load_layout_file(path: Path) -> Scenario:
    """Load and validate a JSON layout file (raises :class:`ScenarioError`)."""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ScenarioError(f"Layout file not found: {path}") from exc
    except (OSError, json.JSONDecodeError) as exc:
        raise ScenarioError(f"Layout file {path} is corrupted: {exc}") from exc
    if not isinstance(data, dict):
        raise ScenarioError(f"Layout file {path} must contain a JSON object")
    return scenario_from_dict(data, scenario_id=Path(path).stem)


def load_predefined(name: str, layout_dir: Path = LAYOUT_DIR) -> Scenario:
    """Load ``small``/``medium``/``large``.

    If the JSON file is missing or corrupted, a seeded generated layout of the
    same size class is returned instead and the reason is stored in
    ``scenario.metadata['warning']`` so the UI can show it.
    """
    path = Path(layout_dir) / f"{name}.json"
    try:
        return load_layout_file(path)
    except ScenarioError as exc:
        if name not in SIZE_PRESETS:
            raise
        fallback = generate_random_scenario(seed=1000 + list(SIZE_PRESETS).index(name), size=name)
        fallback.name = f"{LAYOUT_LABELS.get(name, name)} (generated fallback)"
        fallback.metadata["warning"] = str(exc)
        return fallback


def save_scenario(scenario: Scenario, path: Path) -> None:
    """Persist a scenario in the same JSON layout format used by the loader."""
    rows = [list(r) for r in scenario.grid.to_strings()]
    for r, c in scenario.stations:
        rows[r][c] = "D"
    for r, c in scenario.robot_starts:
        rows[r][c] = "R"
    for spec in scenario.parcels:
        r, c = spec.position
        rows[r][c] = "P"
    data = {
        "id": scenario.scenario_id,
        "name": scenario.name,
        "seed": scenario.seed,
        "size_class": scenario.size_class,
        "map": ["".join(r) for r in rows],
        "destinations": [p.destination_id for p in scenario.parcels],
    }
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")


# --------------------------------------------------------------------------- #
# Seeded random generation
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class SizePreset:
    width: int
    height: int
    parcels: int
    stations: int


SIZE_PRESETS: dict[str, SizePreset] = {
    "small": SizePreset(width=14, height=10, parcels=6, stations=2),
    "medium": SizePreset(width=22, height=14, parcels=10, stations=3),
    "large": SizePreset(width=30, height=20, parcels=14, stations=4),
}


def generate_random_scenario(
    seed: int,
    size: str = "medium",
    scenario_id: str | None = None,
    num_parcels: int | None = None,
) -> Scenario:
    """Generate a warehouse-style layout that is fully determined by ``seed``.

    Structure: a free top dock row (robot starts), a free bottom row (delivery
    stations), free left/right service aisles, and double-depth rack blocks with
    seeded lengths, cross-aisles, missing rack cells and stray pallets.
    The layout is validated; on the (rare) failure the generator retries with a
    derived seed, so the result is still deterministic.
    """
    if size not in SIZE_PRESETS:
        raise ValueError(f"Unknown size class {size!r}; choose from {list(SIZE_PRESETS)}")
    preset = SIZE_PRESETS[size]
    for attempt in range(50):
        rng = random.Random(seed * 1009 + attempt)
        try:
            scenario = _generate_once(rng, preset, num_parcels or preset.parcels)
        except ScenarioError:
            continue
        scenario.seed = seed
        scenario.size_class = size
        scenario.scenario_id = scenario_id or f"{size}-s{seed}"
        scenario.name = f"Random {size.title()} (seed {seed})"
        scenario.metadata["attempt"] = attempt
        try:
            validate_scenario(scenario, min_robots=MAX_ROBOTS)
        except ScenarioError:
            continue
        return scenario
    raise ScenarioError(f"Could not generate a valid {size} layout for seed {seed}")


def _generate_once(rng: random.Random, preset: SizePreset, n_parcels: int) -> Scenario:
    w, h = preset.width, preset.height
    grid = Grid.empty(w, h)

    # Rack rows: 2 shelf rows followed by a 1-row aisle, inside rows 2..h-3.
    top, bottom = 2, h - 3
    r = top
    while r + 1 <= bottom:
        depth = 2 if r + 1 <= bottom else 1
        c = 2
        while c < w - 2:
            length = rng.randint(3, 6)
            end = min(c + length, w - 2)
            for cc in range(c, end):
                for rr in range(r, r + depth):
                    grid.set_shelf((rr, cc))
            c = end + rng.choice((1, 1, 2))  # cross-aisle width
        r += depth + 1

    # Randomly remove some rack cells and drop a few stray pallets.
    for pos in grid.shelf_cells():
        if rng.random() < 0.08:
            grid.set_shelf(pos, False)
    for _ in range(max(1, (w * h) // 120)):
        rr, cc = rng.randint(top, bottom), rng.randint(2, w - 3)
        if not grid.is_shelf((rr, cc)):
            grid.set_shelf((rr, cc))

    # Delivery stations along the bottom row, robots along the top dock row.
    spacing = w // (preset.stations + 1)
    stations = []
    for i in range(preset.stations):
        col = spacing * (i + 1) + rng.randint(-1, 1)
        stations.append((h - 1, max(1, min(w - 2, col))))
    stations = sorted(set(stations), key=lambda p: p[1])

    starts_cols = sorted(rng.sample(range(1, w - 1), MAX_ROBOTS))
    robot_starts = [(0, c) for c in starts_cols]

    # Parcels sit on reachable floor cells adjacent to a rack (pick faces).
    reach = grid.reachable_from(robot_starts[0])
    candidates = [
        p
        for p in sorted(reach)
        if top - 1 <= p[0] <= bottom + 1
        and any(grid.is_shelf((p[0] + dr, p[1] + dc)) for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)))
    ]
    if len(candidates) < n_parcels:
        raise ScenarioError("not enough pickup locations")
    pickups = rng.sample(candidates, n_parcels)
    parcels = [ParcelSpec(p, rng.randrange(len(stations))) for p in pickups]

    return Scenario(
        scenario_id="tmp",
        name="tmp",
        seed=0,
        grid=grid,
        stations=stations,
        parcels=parcels,
        robot_starts=robot_starts,
    )


def build_experiment_scenarios(count: int, base_seed: int) -> list[Scenario]:
    """Deterministic scenario set: sizes cycle small -> medium -> large."""
    sizes = list(SIZE_PRESETS)
    scenarios = []
    for i in range(count):
        size = sizes[i % len(sizes)]
        seed = base_seed + i
        scenarios.append(
            generate_random_scenario(seed, size, scenario_id=f"S{i + 1:02d}-{size}-{seed}")
        )
    return scenarios
