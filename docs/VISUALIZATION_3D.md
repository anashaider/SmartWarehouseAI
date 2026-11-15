# 3D Digital Twin

The 3D digital twin (`interface3d/`, Panda3D) is a **visualisation layer** on top
of the unchanged simulation. It reads the same `SimulationController` and
`SimulationEngine` that the 2D dashboard and the experiment runner use. It
never moves a robot, changes a decision or alters a metric. Every robot
position, pickup, delivery, path, explored cell and statistic on screen comes
from the live engine.

![Overview](screenshots/3d_overview.png)

## 1. Why this design

| Requirement | Decision |
|---|---|
| Do not break working AI logic | No changes to `environment/`, `algorithms/`, `agents/`, `coordination/`, `simulation/`, `experiments/`. The 3D package only reads their state. |
| Realistic 3D in Python, offline | **Panda3D 1.10** (pip wheel, no system dependencies). It provides per-pixel lighting (auto-shader), shadow-mapped lights, post-processing (MSAA, bloom, SSAO), offscreen rendering and DirectGUI. All geometry and textures are procedural (NumPy), so there are no asset files to download or license. |
| Animations must reflect real state | All simulation → visual mapping lives in `interface3d/sync.py`, which is pure Python and unit-tested without a GPU. |
| Separate rendering from simulation | The controller runs fixed-rate ticks (1-32 ticks/s, at most 8 per frame); rendering runs at display rate (vsync). Animations interpolate the last real tick. |
| Fallback | `python main.py` tries 3D first. If Panda3D is missing or no OpenGL context can be created, it prints the reason and starts the 2D Pygame dashboard. `--2d` forces 2D. |

## 2. Modules

| File | Responsibility |
|---|---|
| `interface3d/sync.py` | Grid ↔ world mapping, robot poses (interpolation, turn-then-drive, reverse driving during retreats, wheel rotation, lift), parcel/dock views, effects from real `TickEvent`s, camera-rig maths, Presentation Mode director. **No Panda3D import.** |
| `interface3d/geometry.py` | `MeshBuilder`: boxes, cylinders, rings, discs with normals, vertex colours and UVs. |
| `interface3d/textures.py` | Procedural textures: polished concrete tiles, hazard stripes, corrugated wall, conveyor belt, path chevrons, carton labels. |
| `interface3d/warehouse_scene.py` | Floor, walls, steel columns and trusses, wall lamps, racks (blue uprights, orange beams, three levels, seeded cartons), yellow floor tape along aisles, pick-face markings, charging pads, loading docks (hazard pad, conveyor, pallet, sign, beacon). Static geometry is flattened into a few batches. |
| `interface3d/robot_model.py` | AMR model: chassis, accent shell, bumper, headlights, front ID panel, 4 wheels, spinning lidar, lift platform with cargo carton, status beacon, identification halo, selection ring. |
| `interface3d/effects.py` | Animated path ribbons (scrolling chevrons), destination markers with light beams, BFS/A* explored-cell tiles revealed in real expansion order, pickup/delivery/conflict/yield effects. |
| `interface3d/camera.py` | Camera modes, smooth transitions (frame-rate independent damping), mouse orbit/pan/zoom, opening flythrough. |
| `interface3d/hud.py` | Command-center HUD (DirectGUI, pixel layout, rebuilt on resize). |
| `interface3d/analytics3d.py` | Analytics overlay: recorded charts and summary tables; runs experiments incrementally. |
| `interface3d/app3d.py` | The application: lights, quality presets, frame loop, input, picking, camera occlusion avoidance, presentation overlay. |

## 3. How animation stays faithful to the simulation

For every robot and every frame, `VisualSync.robot_poses(engine, progress, dt)` returns a pose:

- **Position:** an eased interpolation from `previous_position` (the cell before the last tick) to `position` (the cell after it). At `progress = 1` the robot is exactly on its simulated cell. Between those points it stays on the segment between the two real cells. Both facts are tested for every agent type.
- **Turning:** when the travel direction changes, the first 35% of the tick animation rotates the robot in place and the rest drives it. During a coordination retreat (`robot.is_yielding`), a move opposite to the robot's heading is driven **in reverse**, as real AMRs back out of aisles.
- **Lift and cargo:** the lift height eases toward `1 if robot.carrying else 0`, and the carton on the lift is visible exactly while `robot.carrying` is set.
- **Parcels:** floor cartons are visible only while the parcel is WAITING or ASSIGNED. ASSIGNED parcels get a ring in the assigned robot's colour. A DELIVERED parcel appears on its destination dock's pallet. In the tick it is delivered, a carton flies from the robot's lift to the pallet.
- **Effects:** created only from engine `TickEvent`s (pickup, deliver, conflict, invalid, yield, wait), at the event's cell.
- **Status beacon:** colour follows `robot.state`; amber blinking means waiting for another robot, cyan pulsing means yielding.

## 4. Cameras

| Mode | Key | Behaviour |
|---|---|---|
| Cinematic Overview | O | Slow drifting orbit of the whole warehouse |
| Top-Down | T | Plan view (best for BFS/A* explored cells) |
| Robot Tracking | F | Third-person chase camera behind the selected robot |
| Free Camera | V | Right-drag orbit, middle-drag pan, wheel zoom |
| Robot Focus | G | Orbit around the selected robot (drag/zoom adjust the offset) |

All transitions ease with frame-rate-independent exponential damping.
**Occlusion avoidance:** if racks block the line of sight to the target, the
camera first tries nearby headings, usually finding an aisle-aligned view
(tracking shots only), and otherwise smoothly steepens its pitch. This only
changes the camera.

**Opening sequence:** a letterboxed, keyframed flythrough from high above into
the aisles and out to the overview. Click or press Esc to skip; the HUD then
appears.

## 5. Presentation Mode (P)

HUD hidden, letterbox bars, captions and live statistics. The
`PresentationDirector` cycles through overview → robot tracking → pathfinding
(BFS vs A* explored cells for that robot's real goal) → top-down performance.
A real pickup or delivery interrupts the rotation with a highlight shot of the
robot that performed it, captioned with the actual event and step. Entering or
leaving Presentation Mode never pauses, resets or steps the simulation. If the
simulation is still READY when presentation starts, it is started so there is
something to show.

## 6. Graphics quality

| Preset | Shadows | Post-processing | Measured FPS* |
|---|---|---|---|
| Low | off | none | 60 |
| Medium (default) | 4096 shadow map | MSAA ×4, bloom | 59 |
| High | 4096 shadow map | MSAA ×4, bloom, SSAO | 55 |

\*Large warehouse, 3 robots, simulation running, 1600×900, Intel Iris Xe (integrated), vsync on.
Switch with the Graphics Quality selector or `Q`. Unsupported effects are disabled with an on-screen message.

## 7. Controls

`Space` run/pause · `S` step · `R` reset · `1 2 3`/`Tab`/click select robot ·
`O T F V G` cameras · `C` BFS vs A* explored cells · `P` presentation ·
`A` analytics · `H` hide HUD · `Q` quality · `+`/`-` speed · `F1` help · `Esc` close/skip/exit.

## 8. Validation

| Check | Test |
|---|---|
| Robot nodes equal simulated cells after every tick | `test_app3d.py::test_robot_nodes_follow_simulation`, `test_visual_sync.py::test_robot_poses_match_simulation_state` |
| Pickup/delivery visuals equal real actions | `test_app3d.py::test_pickup_and_delivery_visuals_match_real_actions`, `test_visual_sync.py::test_effects_come_only_from_real_events` |
| BFS/A* overlay uses the real searches | `test_app3d.py::test_bfs_astar_overlay_uses_real_search` |
| Pause / resume / step / reset via HUD buttons | `test_app3d.py::test_pause_resume_reset_and_buttons` |
| Presentation does not interrupt the simulation | `test_app3d.py::test_presentation_mode_does_not_interrupt_simulation`, `test_visual_sync.py::test_presentation_director_uses_real_events` |
| Cameras, occlusion, picking, quality, analytics, screenshots, intro | remaining `test_app3d.py` tests |
| 2D fallback | `test_launcher.py` |

The Panda3D tests render offscreen and are skipped automatically on machines
without OpenGL.

## 9. Limitations

- Geometry is procedural and stylised (no photogrammetry or PBR texture sets). Lighting uses Panda3D's auto-shader (Blinn-Phong with shadow maps) rather than a full PBR pipeline, chosen for reliability on integrated GPUs.
- SSAO from `CommonFilters` is approximate and subtle.
- If the frame rate drops below the selected tick rate (e.g. 32 ticks/s on a slow GPU), the animation skips intermediate cells. The simulation itself is unaffected.
