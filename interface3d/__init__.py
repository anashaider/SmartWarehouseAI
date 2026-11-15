"""3D digital-twin visualisation layer (Panda3D).

This package only *reads* the simulation (via SimulationController /
SimulationEngine); it never changes agents, search, coordination or metrics.
``sync.py`` holds all simulation -> visual mapping and is importable without
Panda3D, so it can be unit-tested on machines without a GPU.
"""

from __future__ import annotations


def panda3d_available() -> tuple[bool, str]:
    """Return (available, reason)."""
    try:
        import panda3d  # noqa: F401
        from panda3d.core import PandaSystem  # noqa: F401
    except ImportError as exc:
        return False, f"Panda3D is not installed ({exc}). Run: pip install -r requirements.txt"
    return True, ""
