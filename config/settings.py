"""Central configuration for the Smart Warehouse Robot Simulator.

All tunable constants live here so that experiments, the simulator and the
documentation refer to a single source of truth.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path

# --------------------------------------------------------------------------- #
# Paths
# --------------------------------------------------------------------------- #
PROJECT_ROOT: Path = Path(__file__).resolve().parent.parent
DATA_DIR: Path = PROJECT_ROOT / "data"
LAYOUT_DIR: Path = DATA_DIR / "layouts"
RESULTS_DIR: Path = DATA_DIR / "results"
CHARTS_DIR: Path = RESULTS_DIR / "charts"

# --------------------------------------------------------------------------- #
# Identifiers used throughout the project
# --------------------------------------------------------------------------- #
AGENT_TYPES: tuple[str, ...] = ("simple_reflex", "goal_based", "utility_based")
AGENT_LABELS: dict[str, str] = {
    "simple_reflex": "Simple Reflex",
    "goal_based": "Goal-Based",
    "utility_based": "Utility-Based",
}

ALGORITHMS: tuple[str, ...] = ("bfs", "astar")
ALGORITHM_LABELS: dict[str, str] = {"bfs": "BFS", "astar": "A*"}

ALLOCATION_POLICIES: tuple[str, ...] = ("nearest", "fixed_order")
ALLOCATION_LABELS: dict[str, str] = {
    "nearest": "Nearest Parcel",
    "fixed_order": "Fixed Order",
}

PREDEFINED_LAYOUTS: tuple[str, ...] = ("small", "medium", "large")
LAYOUT_LABELS: dict[str, str] = {
    "small": "Small Warehouse",
    "medium": "Medium Warehouse",
    "large": "Large Warehouse",
    "random": "Random (seeded)",
}

# --------------------------------------------------------------------------- #
# Simulation defaults
# --------------------------------------------------------------------------- #
MIN_ROBOTS: int = 2
MAX_ROBOTS: int = 3
DEFAULT_ROBOTS: int = 3
DEFAULT_SEED: int = 42

#: Hard upper bound on simulation ticks. The effective limit for a scenario is
#: ``max(MIN_STEP_LIMIT, STEP_LIMIT_PER_CELL * free_cells)``.
MIN_STEP_LIMIT: int = 300
STEP_LIMIT_PER_CELL: float = 2.0

#: Consecutive blocked ticks after which a robot is considered deadlocked and
#: the coordination layer forces a "yield" (side step).
DEADLOCK_THRESHOLD: int = 3

#: Ticks a yielding robot holds its refuge cell after retreating.
YIELD_HOLD_TICKS: int = 2

#: Consecutive blocked ticks after which the Goal-Based agent replans while
#: treating every other robot as an obstacle.
REPLAN_AFTER_BLOCKED: int = 1

#: Number of decision records kept per robot (for the explainability panel).
DECISION_HISTORY_LIMIT: int = 300


@dataclass(frozen=True)
class UtilityWeights:
    """Weights of the Utility-Based agent's linear utility function.

    U(a) = w_progress   * progress(a)
         - w_distance   * dist(next, target)
         - w_move       * move_cost(a)
         - w_obstacle   * obstacle_proximity(next)
         - w_conflict   * conflict(next)
         - w_congestion * congestion(next)
         - w_wait       * wait_penalty(a)
         - w_revisit    * recent_visits(next)

    See ``docs/ALGORITHMS.md`` for the definition of every feature.
    """

    progress: float = 10.0
    distance: float = 1.0
    move: float = 0.5
    obstacle: float = 0.3
    conflict: float = 25.0
    congestion: float = 1.5
    wait: float = 2.0
    revisit: float = 1.2

    def as_dict(self) -> dict[str, float]:
        return asdict(self)


DEFAULT_UTILITY_WEIGHTS = UtilityWeights()


@dataclass
class SimulationConfig:
    """User-selectable configuration for one simulation run."""

    layout: str = "medium"
    agent_type: str = "goal_based"
    algorithm: str = "astar"
    allocation: str = "nearest"
    num_robots: int = DEFAULT_ROBOTS
    seed: int = DEFAULT_SEED
    utility_weights: UtilityWeights = field(default_factory=UtilityWeights)

    def validate(self) -> None:
        """Raise ``ValueError`` for unsupported values."""
        if self.agent_type not in AGENT_TYPES:
            raise ValueError(f"Unknown agent type: {self.agent_type!r}")
        if self.algorithm not in ALGORITHMS:
            raise ValueError(f"Unknown algorithm: {self.algorithm!r}")
        if self.allocation not in ALLOCATION_POLICIES:
            raise ValueError(f"Unknown allocation policy: {self.allocation!r}")
        if not MIN_ROBOTS <= self.num_robots <= MAX_ROBOTS:
            raise ValueError(
                f"num_robots must be between {MIN_ROBOTS} and {MAX_ROBOTS}"
            )


# --------------------------------------------------------------------------- #
# Experiment defaults
# --------------------------------------------------------------------------- #
EXPERIMENT_BASE_SEED: int = 2026
EXPERIMENT_SCENARIOS: int = 24  # proposal requires >= 20
#: Repetitions per search when benchmarking runtime (minimum is reported).
SEARCH_TIMING_REPEATS: int = 3

# --------------------------------------------------------------------------- #
# Interface
# --------------------------------------------------------------------------- #
SPEED_STEPS: tuple[float, ...] = (1, 2, 4, 6, 8, 12, 16, 24, 32)  # ticks / second
DEFAULT_SPEED_INDEX: int = 3
TARGET_FPS: int = 60
