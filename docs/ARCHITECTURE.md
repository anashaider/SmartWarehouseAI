# Architecture

## 1. Layers

```
     ┌──── interface3d/ (Panda3D, default) ────┐  ┌────── interface/ (Pygame, 2D fallback) ──────┐
     │ app3d · scene · robots · effects ·      │  │ dashboard · control_panel · warehouse_view · │
     │ camera · hud · analytics3d · sync       │  │ robot_panel · analytics                      │
     └────────────────────┬────────────────────┘  └──────────────────┬───────────────────────────┘
                          └──────────────┬───────────────────────────┘
                ┌────────────────────────┴────────────────────────────────────────────┐
                │  both front-ends only read state and send commands                  │
                └───────────────┬───────────────────────────────────┬─────────────────┘
                                │ commands / reads state            │ runs incrementally
                  simulation/simulation_controller          experiments/experiment_runner
                  (status, tick timing, interpolation)       (scenarios × configurations)
                                │                                   │
                                └──────────► simulation/simulation_engine ◄──────────┘
                                              (one tick = allocate → yield → act → verify)
                     ┌────────────────────────┬─────────┴──────────┬──────────────────────┐
              coordination/              agents/               algorithms/          environment/
       task_allocator, conflict_       simple_reflex,         bfs, astar,          grid, parcel, robot,
       manager, reservations           goal_based,            pathfinder            warehouse, scenario
                                       utility_based                                generator
```

Both interfaces (the 3D digital twin in `interface3d/` and the 2D dashboard in `interface/`) and the experiment runner use the **same** `SimulationEngine`,
so everything shown on screen is the same computation that produces the
experiment data. The engine never renders and never reads the wall clock for
its logic (only to measure compute time).

## 2. Modules

| Module | Responsibility |
|---|---|
| `config/settings.py` | All constants: identifiers, step limits, deadlock threshold, utility weights, experiment seeds, UI speeds. `SimulationConfig` validates user selections. |
| `environment/grid.py` | NumPy-backed static map (`FLOOR`/`SHELF`), the `Action` enum, move deltas in fixed order (Up, Down, Left, Right), Manhattan distance, flood fill. |
| `environment/parcel.py` | `Parcel` life cycle WAITING → ASSIGNED → CARRIED → DELIVERED with guarded transitions; `DeliveryStation`. |
| `environment/robot.py` | `Robot` data and its finite-state machine (see §4). |
| `environment/warehouse.py` | Runtime state. Enforces physical invariants (no shelf entry, no shared cells, one robot per parcel, no re-delivery). |
| `environment/scenario_generator.py` | `Scenario` spec; JSON layout loading/saving with validation; seeded random generator; experiment scenario set. |
| `algorithms/` | `bfs()`, `astar()` returning `SearchResult` (path, cost, nodes expanded/generated, runtime, optional expansion trace); `Pathfinder` facade accumulating statistics. |
| `agents/` | `BaseAgent`, `Percept`, `Decision`, `CandidateAction`; the three agents; `create_agent` factory. |
| `coordination/task_allocator.py` | `nearest` and `fixed_order` policies; assigns only WAITING parcels; skips unreachable ones. |
| `coordination/conflict_manager.py` | Turn order, move validation (invalid / vertex / swap), yield selection, refuge-path BFS. |
| `coordination/reservations.py` | Per-tick vertex and edge reservation table. |
| `simulation/simulation_engine.py` | The tick algorithm, percept construction, action execution, conflict measurement, decision history, events, metrics. |
| `simulation/simulation_controller.py` | Start / pause / resume / step / reset, configuration changes, fixed-timestep tick scheduling, animation progress. |
| `simulation/state_manager.py` | READY / RUNNING / PAUSED / FINISHED status machine. The UI shows a finished run as **COMPLETED** (every parcel delivered) or **TERMINATED** (stopped incomplete) via `SimulationController.display_status`. |
| `simulation/progress_monitor.py` | Run watchdog (observes only, never steers an agent): per-robot oscillation, repeated world states and stagnation (no task progress for `stagnation_limit` ticks). |
| `simulation/run_trace.py` | Per-tick text trace for `python main.py --headless --trace`. |
| `experiments/` | Runner (generator-based), metric definitions and summaries, CSV/JSON export/import, Matplotlib charts. |
| `interface/` | All Pygame drawing and input handling. |

## 3. The tick (SimulationEngine.step)

```
1. TaskAllocator.allocate()           idle robots receive WAITING parcels (state IDLE → NAVIGATING_TO_PARCEL)
2. ConflictManager.begin_tick()       clear reservations
3. order = turn_order(step)           [k, k+1, ..., k-1] with k = step mod n  (rotating priority)
4. _plan_yields()                     choose robots that must retreat (deadlock handling, §5)
5. for robot in order:
       if robot is yielding:  follow refuge path / hold          (coordination override)
       else:
           percept  = build_percept(robot)                         (sensors)
           decision = agent.decide(percept)                         (agent program)
           _execute(robot, decision)                                (actuators, validated)
           _measure_conflict(robot, ...)                            (metric + idle-blocker detection)
6. Warehouse.check_invariants()       raises if two robots share a cell, a robot is on a shelf,
                                      or a parcel is held by two robots
7. ProgressMonitor.observe()          oscillation / repeated-state / stagnation detection
8. termination check                  COMPLETED: all delivered
                                      TERMINATED: step limit | stagnation | remaining parcels unreachable
```

**Idle-blocker detection.** `_measure_conflict` registers an *idle* robot that
occupies a cell on a working robot's shortest route as that robot's blocker,
even when the working robot side-stepped instead of bumping into it. Without
this, the Simple Reflex agent (whose rule 5 side-steps rather than waits) never
produced a rejected move or WAIT, the "idle robot yields at once" rule never
fired, and the carrier looped around an idle robot parked on its drop-off
station until the step limit.

**Watchdog.** Task progress = a pickup, a delivery, or a robot reaching a new
best true distance to its current target. A run with no progress for
`stagnation_limit` ticks (default `max(60, 0.5 × free cells)`, configurable,
`0` disables) is terminated as incomplete. `max_steps` (default
`max(300, 2 × free cells)`) remains the hard limit. Both are fields of
`SimulationConfig` and CLI flags (`--max-steps`, `--stagnation-limit`).

Robots act **sequentially within a tick**. A robot may only enter a cell that is
free at the moment it acts and is not reserved by a robot that already acted.
Because of this, two robots can never end up in the same cell, and they can
never swap through each other: a swap would require entering an occupied cell.
The edge-reservation check makes the swap rule explicit and is unit-tested
directly.

Rejected actions are converted to WAIT. A move into a shelf or wall counts as
an invalid move. A move into another robot's cell counts as a rejected move
and a conflict.

## 4. Robot state machine

```
IDLE ──assign──► NAVIGATING_TO_PARCEL ──arrive──► PICKING_UP ──PICKUP──► NAVIGATING_TO_DROPOFF
  ▲                     │ (cancel)                                              │ arrive
  │                     ▼                                                        ▼
  └─────────────────── IDLE                         IDLE ◄──DELIVER── DELIVERING
```

`Robot.transition()` raises `InvalidStateTransition` for any other change. The
warehouse moves robots into PICKING_UP / DELIVERING when they arrive. Agents
then issue PICKUP / DELIVER, which the warehouse executes only if the robot is
really on its assigned parcel or destination station. Otherwise the action is
counted as invalid.

## 5. Multi-robot coordination

| Situation | Handling |
|---|---|
| Two robots want the same cell | The robot acting later finds the cell occupied/reserved → move rejected → WAIT (conflict counted). Planning agents usually see the robot in their percept and wait or detour first. |
| Two robots try to exchange cells | Impossible under sequential execution; additionally rejected by the edge-reservation check (`SWAP_CONFLICT`). |
| Occupied destination cell | Goal-Based: waits one tick, then replans with all robots as obstacles; if no route exists it keeps waiting. Utility-Based: the conflict feature (weight 25) makes that action unattractive. |
| Idle robot standing on someone's route (e.g. on a station) | The blocked robot reports its blocker; an idle blocker is told to yield immediately. |
| Deadlock (robots blocking each other for `DEADLOCK_THRESHOLD = 3` ticks) | The robot with the lower right of way (`carrying > has task > idle`, then lower id) computes a BFS path to the nearest *refuge* cell outside the other robot's route, retreats there, holds for `YIELD_HOLD_TICKS = 2` ticks and then hands control back to its agent. If it is trapped in a dead end, the roles are swapped. A random free side-step is the last resort. |
| Task conflicts | A parcel can only be assigned while WAITING, so it can never belong to two robots. A delivered parcel is final. |
| Starvation | The turn order rotates every tick. |

**Limitations (stated honestly).** This is a decentralised, greedy,
reactive scheme. It guarantees safety (verified after every tick and in the
tests) and resolved every deadlock in the 288 experiment runs. It does not
compute globally optimal joint plans (e.g. Conflict-Based Search), so robots
may take detours or retreat when a centralised planner would not. The
retreat heuristic could fail in adversarial maps, for example long single-width
loops with more robots than refuges.

## 6. Interface design

- Renders to a fixed **1440 × 900 logical canvas**, scaled (aspect preserved) to the window. Mouse events are mapped back to logical coordinates, so the layout is identical on every display and the window is resizable.
- **Update/render separation:** `SimulationController.update(dt)` uses a fixed-timestep accumulator (1-32 ticks/s, at most 8 ticks per frame), while rendering runs at 60 FPS. Robot positions are drawn interpolated between `previous_position` and `position` with smoothstep easing, so the animation always reflects the real cells.
- Experiments launched from the Analytics screen run in the main loop with a 30 ms budget per frame (generator `iter_run`). No threads are used.
- Static layers (floor, shelves, stations) are cached; dynamic overlays are drawn on one alpha surface per frame.

## 6b. 3D digital twin

`interface3d/` is a read-only visualisation layer over `SimulationController`. All simulation → visual mapping is in the GPU-free `interface3d/sync.py`. Details, camera system, Presentation Mode and validation: [VISUALIZATION_3D.md](VISUALIZATION_3D.md). `main.py` starts the 3D app by default and falls back to the 2D dashboard if Panda3D or OpenGL is unavailable.

## 7. Data formats

**Layout JSON** (`data/layouts/*.json`):

```json
{
  "id": "small", "name": "Small Warehouse", "seed": 11, "size_class": "small",
  "map": ["R.....R.....R.", "..............", ".##P.##..##...", "..."],
  "destinations": [0, 1, 0],                       // optional, row-major parcel order
  "parcels": [{"position": [2, 3], "destination": 0}]   // optional, explicit order
}
```

`.` floor · `#` shelf · `D` delivery station · `R` robot start · `P` parcel pickup.
Without explicit destinations, parcels are assigned to stations with the layout's
seed. Every layout is validated: no robots, parcels or stations inside shelves,
unique cells, and every station and parcel reachable from the robot starts. A
missing or corrupted predefined file falls back to a generated layout of the
same size and shows a warning in the UI.

**Experiment outputs:** see [EXPERIMENTS.md](EXPERIMENTS.md).

## 8. Reproducibility

- Scenario generation: `random.Random(seed * 1009 + attempt)`. Scenario `i` uses seed `2026 + i`.
- Simple Reflex randomness: `random.Random(seed * 7919 + robot_id)`. Yield side-steps: `random.Random(seed * 104729 + 17)`.
- Searches break ties deterministically (fixed neighbour order; A* breaks ties on f, then h, then insertion order).
- All metrics except wall-clock timings are bit-for-bit reproducible, which `tests/test_experiments.py::test_fixed_seed_reproducibility` checks.
