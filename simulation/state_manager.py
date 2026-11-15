"""Simulation run-status state machine used by the interface."""

from __future__ import annotations

from enum import Enum


class SimStatus(str, Enum):
    READY = "READY"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    FINISHED = "FINISHED"


_ALLOWED: dict[SimStatus, frozenset[SimStatus]] = {
    SimStatus.READY: frozenset({SimStatus.RUNNING, SimStatus.PAUSED, SimStatus.FINISHED, SimStatus.READY}),
    SimStatus.RUNNING: frozenset({SimStatus.PAUSED, SimStatus.FINISHED, SimStatus.READY}),
    SimStatus.PAUSED: frozenset({SimStatus.RUNNING, SimStatus.FINISHED, SimStatus.READY, SimStatus.PAUSED}),
    SimStatus.FINISHED: frozenset({SimStatus.READY}),
}


class StateManager:
    """Tracks READY / RUNNING / PAUSED / FINISHED and rejects illegal changes."""

    def __init__(self) -> None:
        self.status = SimStatus.READY

    def can(self, new: SimStatus) -> bool:
        return new in _ALLOWED[self.status]

    def set(self, new: SimStatus) -> bool:
        if not self.can(new):
            return False
        self.status = new
        return True

    @property
    def running(self) -> bool:
        return self.status == SimStatus.RUNNING
