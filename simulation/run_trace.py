"""Plain-text per-tick trace of a run (``python main.py --headless --trace``).

For every robot it logs the position, task state, assigned target, the action
the agent chose versus the one executed, the outcome, the neighbour percept and
the agent's stated rule, followed by delivery progress and watchdog status.
It only reads the engine; it never changes the simulation.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from simulation.simulation_engine import SimulationEngine


def format_tick(engine: "SimulationEngine") -> str:
    wh = engine.warehouse
    mon = engine.monitor
    lines = [f"--- step {engine.step_count} | delivered {wh.delivered_count()}/{len(wh.parcels)} | "
             f"no progress for {mon.ticks_without_progress}/{engine.stagnation_limit} ticks"]
    for robot in wh.robots:
        rec = engine.last_decision.get(robot.robot_id)
        target = wh.parcel_target(robot)
        task = (f"carrying P{robot.carrying}" if robot.carrying is not None
                else f"to P{robot.assigned_parcel}" if robot.assigned_parcel is not None else "idle")
        flag = "  [OSCILLATING]" if robot.robot_id in mon.oscillating else ""
        if rec is None:
            lines.append(f"  R{robot.robot_id} @{robot.position} {task}{flag}")
            continue
        lines.append(
            f"  R{robot.robot_id} {robot.previous_position}->{robot.position} {task} target={target} "
            f"chose={rec.chosen.name} did={rec.executed.name} ({rec.outcome}) "
            f"blocked_streak={robot.blocked_streak}{flag}")
        nb = rec.percept.get("Neighbours")
        if nb:
            lines.append(f"      percept: {nb}")
        lines.append(f"      reason:  {rec.reason}")
    for ev in engine.events:
        if ev.kind in ("assign", "pickup", "deliver", "yield", "oscillation", "repeated_state"):
            lines.append(f"  * {ev.kind}: {ev.message}")
    if engine.done:
        lines.append(f"=== {engine.outcome}: {engine.termination_reason}")
    return "\n".join(lines)
