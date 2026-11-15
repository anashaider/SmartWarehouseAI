# Demonstration Guide and Viva Preparation

## 1. Before the viva (5 minutes)

```bash
pip install -r requirements.txt
python -m pytest -q                 # expect: 181 passed
python main.py --experiments        # ~1 minute, regenerates data/results and charts
python main.py                      # 3D digital twin (python main.py --2d for the 2D dashboard)
```

Press **F1** in the app for the legend and shortcuts.

## 2. 3D demo script (≈ 6 minutes)

| Step | Action | What to say |
|---|---|---|
| 1 | `python main.py` and let the opening flythrough play | "A digital twin of the warehouse: racks, docks and three AMRs, generated from the same scenario the AI uses." |
| 2 | **START**, camera **Overview** | "Robots move exactly to the cells the simulation engine computes; the 3D layer only reads the engine." |
| 3 | Click a robot, **Third-Person** (F) or **Robot Tracking** (K) | Chase camera: the robot drives under the parcel stand and lifts the carton, the LED band follows the state, the inspector shows the AI decision. |
| 4 | **Top-Down** (T) + `C` | BFS (blue) vs A* (orange) explored cells appear in the real expansion order; the panel shows path lengths and node counts. |
| 5 | Agent **Utility-Based**, **Robot Focus** (G) | Candidate actions and U-scores update live. |
| 5b | Press `M`, open the **FLEET** tab | Congestion heatmap and utilisation come from the engine every tick; dock signs count real deliveries. Try Simple Reflex to see hotspots form where robots get stuck. |
| 6 | **PRESENTATION** (P) | Automatic shots; every pickup/delivery highlight is a real event. P again returns; the simulation never stopped. |
| 7 | **ANALYTICS** (A) | Recorded charts and summary tables from the 288-run experiment. |

If the GPU struggles, press `Q` until Quality = Low, or run `python main.py --2d`.

## 2b. 2D demo script (≈ 8 minutes)

| Step | Action | What to say |
|---|---|---|
| 1 | Medium Warehouse, Goal-Based, A*, 3 robots → **Start** | "Robots get parcels from the allocator, plan with A*, pick up, deliver. Every number at the bottom is live engine data." |
| 2 | **Pause**, click a robot, open **Robot** tab | State machine (Navigating To Parcel → Picking Up → …), assigned parcel, destination, distance, deliveries, and the decision explanation. |
| 3 | **Step** a few times | "One click = one tick. The robot glides between the two cells it really occupied." |
| 4 | Open **Search** tab, toggle **BFS vs A\* overlay** (C) | Same shortest length, but BFS (blue tint) explored far more cells than A* (orange squares). Read the table: nodes expanded, runtime. |
| 5 | Switch agent to **Utility-Based**, open **Decisions** tab, Step | Show the five candidate actions with their U values, the chosen one in green, and the reason sentence. Explain the formula (§4 below). |
| 6 | Switch to **Simple Reflex**, Large layout, speed 24 → Start | Robots oscillate behind shelf rows: no planning. Note "Pathfinding (unused by reflex)". |
| 7 | Small layout, Goal-Based, 3 robots, speed 4 | Watch amber pause badges (waiting) and cyan rings (yielding). Open **Events** tab: conflicts, yields, pickups, deliveries. |
| 8 | Layout **Random**, change seed | Seeded generator: same seed, same warehouse. |
| 9 | **Analytics** (A) | Charts 1-3 (required) plus extras; summary tables; "equal path length in 960/960 queries". Optionally click **Run Experiments** to show it runs live. |

## 3. Key numbers to remember

- 24 seeded scenarios, 288 simulation runs, 960 BFS/A* query pairs.
- BFS and A* have equal path lengths in 960/960 queries; A* expands ~6× fewer nodes (131.2 vs 22.0).
- Goal-Based and Utility-Based: 100% delivery in all runs. Simple Reflex: 35.8% average, 6/96 complete.
- Nearest allocation vs fixed order: Goal-Based distance 322 → 278 cells.
- 181 automated tests (134 core + 47 for the 3D layer and launcher).

## 4. Viva questions and answers

**Q: What is the PEAS description?**
Performance: deliveries, distance, steps, nodes expanded, runtime, conflicts. Environment: grid warehouse with shelves, parcels, stations, robots. Actuators: 4 moves, pick up, deliver, wait. Sensors: own position/state/heading, 4 adjacent cells, target, other robots' positions.

**Q: Classify the environment.**
Fully observable map, deterministic actions, discrete, sequential, multi-agent and cooperative, and dynamic, because the other robots move while one decides.

**Q: Why is BFS optimal here?**
All steps cost 1, and BFS expands nodes in non-decreasing depth order, so the goal is first reached at minimum depth.

**Q: Why is Manhattan distance admissible and consistent?**
Each move changes row or column by exactly one, so you need at least |Δr| + |Δc| moves (admissible). Between neighbours h changes by exactly 1 = the step cost, so h(n) ≤ c(n,n′) + h(n′) (consistent). Consistency means a closed set is safe and the first expansion of a node is optimal.

**Q: If both are optimal, why use A\*?**
Same path, much less work: A* only expands nodes with f ≤ C*. We measured ~6× fewer expansions, and the gap grows with the warehouse size.

**Q: When would A\* not beat BFS?**
In very short or straight queries both expand nearly the same nodes. When racks force long detours, Manhattan distance is less informed and A* expands more nodes than in open space. With a consistent heuristic it expands only nodes with f(n) ≤ C*, which all have g(n) ≤ C*. BFS must expand almost all of those too, so A* can only lose by a few tie-broken nodes on the last layer, plus heap overhead per node. With h = 0, A* becomes uniform-cost search, which behaves like BFS on unit costs.

**Q: Difference between the three agents?**
Reflex: maps the current percept to an action with rules; no memory or goal. Goal-based: has an explicit goal and searches for a sequence of actions to reach it. Utility-based: compares the desirability of outcomes with a numeric utility, so it can trade off competing objectives (distance vs congestion vs waiting).

**Q: Write the utility function.**
U(a) = 10·progress − 1·dist − 0.5·move − 0.3·obstacle − 25·conflict − 1.5·congestion − 2·wait − 1.2·revisit. The agent takes argmax over the legal actions. Example: forward = +5.5, wait = −7, backward = −16.5.

**Q: How do you prevent collisions?**
Robots act in a rotating turn order. A move is allowed only into a cell that is free and unreserved at that moment, and reverse-edge reservations forbid swaps. The engine asserts after every tick that no two robots share a cell.

**Q: How do you handle deadlocks?**
If robots block each other for 3 ticks, the lower-priority robot (not carrying, then higher id) runs BFS to the nearest cell outside the other robot's route, retreats there, holds for 2 ticks and resumes. Before adding this, 14 runs livelocked in dead-end aisles; after it, none did.

**Q: Limitations of your coordination?**
It is decentralised and greedy, not an optimal multi-agent planner (e.g. CBS). Robots may detour or retreat unnecessarily, and adversarial maps could still defeat the heuristic.

**Q: How are results reproducible?**
Every random choice uses a seeded `random.Random`; searches break ties deterministically. Re-running yields identical counts; only wall-clock times differ. A test checks this.

**Q: Is the 3D animation fake?**
No. The 3D layer reads the engine every frame. Robots are drawn between the cell they occupied before and after the last real tick (tested to equal the simulated cell), the lift and cargo follow `robot.carrying`, and effects are created only from engine events. The AI, search and coordination code is the same one used by the 2D dashboard and the experiments.

**Q: Why did the reflex agent have 0 invalid moves?**
It perceives the four neighbouring cells and its rules only choose free ones. The invalid-move counter is still implemented and tested; it would count a move into a shelf/wall or an illegal pickup.

**Q: Why are Goal-Based and Utility-Based so close?**
Both use exact shortest-path distances, so with 2-3 robots they mostly choose the same routes. Utility-Based detours around congestion a bit more: slightly lower distance and conflicts, at ~18× more search effort.

## 5. Division of work (from the proposal)

- **Anas Haider (24K-2002):** project coordination, warehouse environment, simulator integration, testing, documentation.
- **Shayan Hassan (24K-2050):** search algorithms, agent logic, performance measurement, charts, testing.

## 6. Troubleshooting

| Problem | Fix |
|---|---|
| `ModuleNotFoundError: pygame` | `pip install -r requirements.txt` (inside the virtual environment) |
| Window too large/small | The window is resizable; content scales automatically |
| Analytics shows "No saved experiment results" | Click **Run Experiments** or run `python main.py --experiments` |
| A layout JSON was edited and broke | The app falls back to a generated layout and shows a warning; run `python tools/build_layouts.py` to restore the originals |
