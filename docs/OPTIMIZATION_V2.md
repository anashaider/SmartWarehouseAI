# 3D Optimization V2 — Audit, Changes and Measurements

Rollback point: the git tag `pre-3d-v2`. V2 is on branch `3d-optimization-v2`.
No AI, search, coordination, experiment or metric logic was changed. The only
change outside `interface3d/` is a UI-level `tick_listeners` observer list and
a longer animation window in `SimulationController`, plus a ninth chart
derived from the existing recorded columns. CSV and JSON export formats are
unchanged.

## 1. Method

`tools/bench3d.py` opens a real on-screen window (1600×900, vsync off) on the
**large** warehouse with the Utility-Based agent at 8 ticks/s, and measures:

- frame time (mean / p50 / p95) and FPS;
- CPU time of the Python per-frame update;
- process memory (Windows working set and private bytes);
- scene size (geoms, triangles);
- the cost of repeated layout rebuilds.

Baseline numbers come from the `pre-3d-v2` tag checked out in a separate
worktree with the same benchmark script.

This laptop (Intel Iris Xe, integrated GPU) throttles under sustained GPU load,
and single runs varied by up to 2×. The final figures are therefore
**medians of 4–5 interleaved before/after runs after a cool-down period**.
Absolute FPS depends on hardware and thermal state; the before/after ratios
are the meaningful result.

## 2. Audit (measured before changing anything)

| Measurement (medium preset unless noted) | Result | Conclusion |
|---|---|---|
| Python update per frame | 0.8 ms of ~12 ms | Not CPU-bound; the GPU dominates |
| Everything off | 5.0 ms | Baseline |
| + 4096 shadow map | +3.3 ms | Shadow pass re-rendered every frame, even when paused |
| + 2048 shadow map | +1.9 ms | |
| + MSAA (filter pipeline) | +3.2 ms | Native window MSAA was worse: the driver gave 16 samples |
| + bloom | +0.4 ms | |
| + SSAO | +2.9 ms | |
| HUD visible vs hidden | +1.9 ms | 87 text nodes plus DirectGUI frames, each a separate draw call |
| Static scene | fully merged | Very few draw calls, but never frustum-culled |
| Explored-cell overlay | one node per cell (hundreds) | Per-cell show/hide every frame |
| Parcels / metrics | recomputed every frame | Only change when the simulation ticks |

## 3. Changes

**Rendering performance**

- **On-demand shadows:** the shadow buffer is re-rendered only when a shadow caster changes (robot pose, lift, parcel visibility, dock stack, carton transfer), and at most every 2nd frame on Medium. A paused or idle warehouse costs no shadow pass.
- **Shadow camera mask:** the floor, decals and all overlays are excluded from the shadow camera.
- **Medium preset:** 2048 shadow map with a frustum fitted to the floor, and bloom at "small" size. High keeps 4096.
- **HUD batching:** static panels and labels are generated as geometry and flattened into a few geoms per layer; only buttons and live values stay individual nodes. Layers are ordered with fixed cull bins.
- **Spatial chunks:** static racks, cartons and stands are split into 8×8-cell chunks merged per material, so frustum culling discards off-screen chunks in close-up cameras. On Low, chunks are merged back into the fewest batches, because draw calls matter more there.
- **Level of detail:** near-only detail (rack bracing, base plates, carton tape and labels) sits in a `LODNode` per chunk, shown within 24 units.
- **Instancing:** not used. Panda3D's auto-shader does not support hardware instancing, and merged static batches give fewer draw calls for this scene.
- **Batched overlays:** BFS/A* explored cells are batched into 24 reveal buckets per algorithm. Event-effect child nodes are cached, and the heatmap rewrites vertex colours in place without creating nodes.
- **Per-tick caches:** metrics, parcel views, dock stacks and dock progress are recomputed only when the simulation ticks.
- **Resource cleanup:** a rebuild destroys the effects, heatmap, indicators and world nodes. Render-node count returns to baseline (tested).
- **Depth precision:** the near plane went from 0.1 to 0.25, and floor overlays get a polygon depth offset. This fixed a V1 bug where the heatmap and other floor overlays could vanish under post-processing; it is regression-tested.

**Visual quality**

- **Floor:** concrete tiles with per-tile variation and ambient occlusion baked into vertex colours around the racks; depth fog.
- **Racks:** end guards, bracing, base plates, and cartons with tape and labels, plus plastic totes and shrink-wrapped pallets.
- **Robots:** a new low-profile lifting AMR (see `VISUALIZATION_3D.md`) that drives under parcel stands and lifts the carton.
- **Parcels:** cartons with tape crosses and labels in the colour of their destination dock.
- **Docks:** roller conveyors with side rails, bollards and approach chevrons, plus colour-coded signs with live progress bars.
- **Safety markings:** a green pedestrian walkway, aisle IDs with stop lines, and yellow column guards.
- **Lighting:** high-bay lamps, and roof trusses hidden in plan views.

**Animation, cameras, intelligence, analytics and UX**

These are described in `VISUALIZATION_3D.md`:
- motion profile, differential drive wheels, and the stand pickup and conveyor delivery sequences;
- six camera modes, timed transitions, and cinematic event shots framed around real positions;
- the congestion heatmap, utilisation, robot tags, dock progress and Fleet tab;
- analytics KPI tiles and chart 9;
- tooltips, the loading card and timed banners.

## 4. Results (medians, Intel Iris Xe, 1600×900, vsync off)

| Configuration | FPS before → after | p50 frame time | p95 frame time |
|---|---|---|---|
| Medium, overview, robots moving | 78.3 → **102.3** (+31%) | 12.07 → 9.62 ms | 18.02 → 14.35 ms |
| Medium, overview, paused | 87.2 → **123.3** (+41%) | 11.31 → 7.97 ms | 13.07 → 9.39 ms |
| Medium, third-person camera | 84.5 → **105.5** (+25%) | 11.15 → 9.41 ms | 17.46 → 14.14 ms |
| Low, overview | 186.3 → **195.0** (+5%) | 5.07 → 4.84 ms | 7.52 → 6.71 ms |
| High, overview | 69.0 → 68.5 (no change) | 13.51 → 13.46 ms | 21.73 → 23.23 ms |

| Memory (working set / private bytes) | Before | After |
|---|---|---|
| Medium | 625 / 779 MB | **433 / 636 MB** (−31% / −18%) |
| High | 652 / 806 MB | 664 / 815 MB |
| Low | 272 / 558 MB | 278 / 563 MB |
| Growth after 3 layout rebuilds (medium) | +10 MB | +18 MB (render-node count returns to baseline) |

| Other | Before | After |
|---|---|---|
| Python update CPU per frame | 0.55–1.19 ms | 0.61–1.33 ms (unchanged) |
| Scene triangles (large) | 48.6 k | 83.3 k (+71%, added detail) |
| Geoms (medium, large layout) | 121 | 192 (chunking; culled in close-ups) |
| Startup | 1.23 s | 1.58 s |
| Layout rebuild | 0.20 s | 0.34 s (more procedural geometry) |

**Interpretation**

- **Medium (the default) gained 25–41% FPS and cut memory by about a third** while drawing about 70% more triangles of detail. The gain comes from on-demand shadows, the right-sized shadow map and HUD batching.
- **High is unchanged:** it keeps the 4096 shadow map and SSAO, and its savings were spent on the added detail.
- **Startup and rebuild got slower** (by 0.35 s and 0.14 s) because more geometry is generated. Both are one-off costs, and the rebuild shows a loading card.
- **Frustum culling is verified by a test:** close-up cameras keep fewer chunks in view than the overview.

## 5. Reproduce

```bash
git worktree add ../before pre-3d-v2 && cp tools/bench3d.py ../before/tools/
python tools/bench3d.py --label after  --quality medium --camera overview            # and --paused, --quality low/high
(cd ../before && python tools/bench3d.py --label before --quality medium --camera overview)
```

Results are appended to `data/results/bench3d.jsonl`.
