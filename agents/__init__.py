"""Agent implementations and a small factory."""

from __future__ import annotations

from agents.base_agent import BaseAgent, Decision, Percept
from agents.goal_based import GoalBasedAgent
from agents.simple_reflex import SimpleReflexAgent
from agents.utility_based import UtilityBasedAgent
from algorithms.pathfinder import Pathfinder
from config.settings import DEFAULT_UTILITY_WEIGHTS, UtilityWeights
from environment.grid import Grid

AGENT_CLASSES: dict[str, type[BaseAgent]] = {
    "simple_reflex": SimpleReflexAgent,
    "goal_based": GoalBasedAgent,
    "utility_based": UtilityBasedAgent,
}


def create_agent(
    agent_type: str,
    robot_id: int,
    grid: Grid,
    pathfinder: Pathfinder,
    seed: int = 0,
    weights: UtilityWeights = DEFAULT_UTILITY_WEIGHTS,
) -> BaseAgent:
    try:
        cls = AGENT_CLASSES[agent_type]
    except KeyError as exc:
        raise ValueError(f"Unknown agent type {agent_type!r}") from exc
    if cls is UtilityBasedAgent:
        return UtilityBasedAgent(robot_id, grid, pathfinder, seed, weights=weights)
    return cls(robot_id, grid, pathfinder, seed)


__all__ = [
    "AGENT_CLASSES",
    "BaseAgent",
    "Decision",
    "GoalBasedAgent",
    "Percept",
    "SimpleReflexAgent",
    "UtilityBasedAgent",
    "create_agent",
]
