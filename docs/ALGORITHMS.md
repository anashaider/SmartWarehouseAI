# Algorithms and Agents

Coordinates are `(row, col)`. Robots move in 4 directions with unit cost.
Shelves and the outside of the grid are impassable. Both search algorithms can
additionally treat a set of `blocked` cells (cells occupied by other robots)
as obstacles.

## 1. Breadth-First Search (`algorithms/bfs.py`)

```
BFS(grid, start, goal, blocked):
    if start or goal not walkable, or goal ∈ blocked: return failure
    if start == goal: return [start]
    parent ← {start: None};  frontier ← FIFO queue [start]
    while frontier not empty:
        node ← frontier.pop_front();  expanded += 1
        for child in neighbours(node) in order Up, Down, Left, Right:
            if child ∈ parent or child ∈ blocked: continue
            parent[child] ← node
            if child == goal: return reconstruct(parent, goal)     # early goal test
            frontier.push_back(child)
    return failure
```

- **Visited tracking:** a node is marked (`parent`) when generated, so it is queued at most once.
- **Optimality:** with unit step costs BFS generates nodes in non-decreasing depth order, so the first time the goal is generated it is at minimum depth. The early goal test (AIMA Fig. 3.9) keeps optimality and saves one layer of expansions.
- **Complexity:** O(V + E) time and O(V) memory for a grid with V free cells.
- **Metrics:** `nodes_expanded` = nodes dequeued and expanded; `nodes_generated` = nodes added to `parent`; `runtime_ms` from `time.perf_counter()`.

## 2. A* Search (`algorithms/astar.py`)

```
A*(grid, start, goal, blocked):
    g[start] ← 0;  open ← heap [(h(start), h(start), 0, start)];  closed ← ∅
    while open not empty:
        (f, h, _, node) ← pop_min(open)
        if node ∈ closed: continue                  # stale entry
        if node == goal: return reconstruct(parent, goal)
        closed.add(node);  expanded += 1
        for child in neighbours(node):
            if child ∈ closed or child ∈ blocked: continue
            if g[node] + 1 < g.get(child, ∞):
                g[child] ← g[node] + 1;  parent[child] ← node
                push(open, (g[child] + h(child), h(child), counter++, child))
    return failure
```

**Heuristic:** `h(n) = |r_n − r_goal| + |c_n − c_goal|` (Manhattan distance).

- **Admissible:** every move changes the row or the column by exactly 1, so at least `h(n)` moves are needed even with no obstacles: `h(n) ≤ h*(n)`.
- **Consistent:** for neighbours n, n′, Manhattan distance changes by exactly 1, so `h(n) ≤ c(n, n′) + h(n′) = 1 + h(n′)`. With a consistent heuristic f is non-decreasing along any path, a node's g-value is optimal when it is first popped, and a closed set is safe. A* is therefore **optimal** and expands each node at most once.
- **Tie-breaking:** equal f → smaller h first (prefers nodes closer to the goal), then insertion order. This is deterministic.
- Both properties are verified on real layouts in `tests/test_astar.py::test_manhattan_heuristic_admissible_and_consistent`.

### BFS vs A* - what to expect and what we measured

Both are optimal here, so path lengths must be identical. They were identical in
960/960 benchmark queries. A* differs in effort: it expands only nodes with
`f(n) ≤ C*`, a "lens" around the straight line, while BFS expands every node
with `g(n) < C*`, a "diamond" around the start. Measured averages: BFS 131.2
vs A* 22.0 nodes expanded (about 6× fewer). The gap grows with warehouse size
(small 3.6×, medium 5.7×, large 6.5×). Shelves make Manhattan distance less
informed (detours around racks), so A* is not perfectly guided, but it is still
far more focused.

In the full simulations, BFS and A* sometimes return *different* shortest paths
of equal length (different tie-breaking), so robot interactions, and therefore
steps and distance, can differ slightly between algorithms for the same agent.

## 3. Simple Reflex Agent (`agents/simple_reflex.py`)

Condition-action rules on the current percept only. It keeps no plan, no map
memory and runs no search.

| # | Condition | Action |
|---|---|---|
| 1 | standing on the assigned parcel (state PICKING_UP) | PICKUP |
| 2 | carrying and standing on the destination (DELIVERING) | DELIVER |
| 3 | no task | WAIT |
| 4 | the adjacent cell toward the target (larger offset axis first) is free | move there |
| 5 | preferred cells blocked (shelf/wall/robot) | random free move, avoiding an immediate U-turn (seeded) |
| 6 | nothing free | WAIT |

Rule 5 uses the robot's own heading, which is part of its percept, and seeded
randomisation, the textbook way for a memoryless agent to escape loops
(Russell & Norvig §2.4.2). Expected weakness: dead ends behind shelf rows. It
completed 6 of 96 experiment runs.

## 4. Goal-Based Agent (`agents/goal_based.py`)

1. **Goal formulation:** goal = assigned parcel cell; after pickup, goal = the parcel's destination station.
2. **Planning:** `Pathfinder.find_path(position, goal)` with BFS or A* on the static map. The plan is the list of remaining cells.
3. **Execution:** move to `plan[0]`. Executed steps are consumed. If the plan is invalidated (goal changed, robot displaced by a yield), the agent replans.
4. **Blocked:** if `plan[0]` holds a robot, wait one tick (`REPLAN_AFTER_BLOCKED = 1`). If still blocked, replan with all other robots as obstacles and take the detour. If no detour exists, keep the static plan and wait.
5. **Goal update:** pickup/delivery resets the plan, so the next goal triggers a new search.

## 5. Utility-Based Agent (`agents/utility_based.py`)

For every candidate action `a ∈ {Up, Down, Left, Right, Wait}` that does not
enter a shelf or wall, with `n` the resulting cell (current cell for Wait):

```
U(a) = w_p · progress(a)  −  w_d · d(n)  −  w_m · move(a)  −  w_o · obstacle(n)
       −  w_c · conflict(n)  −  w_g · congestion(n)  −  w_w · wait(a)  −  w_r · revisit(n)
```

| Feature | Definition | Weight (`config/settings.py`) |
|---|---|---|
| progress(a) | d(current) − d(n) ∈ {−1, 0, +1} (delivery progress toward the current goal) | w_p = 10.0 |
| d(n) | true shortest-path distance from n to the target, computed with the selected BFS/A* (cached per cell & target) | w_d = 1.0 |
| move(a) | 1 for a move, 0 for wait (movement cost) | w_m = 0.5 |
| obstacle(n) | blocked neighbours of n / 4 (obstacle proximity) | w_o = 0.3 |
| conflict(n) | 1 if n is occupied by another robot | w_c = 25.0 |
| congestion(n) | number of other robots within Manhattan distance 2 of n | w_g = 1.5 |
| wait(a) | 1 + consecutive waits, for Wait only (growing waiting penalty) | w_w = 2.0 |
| revisit(n) | times n appears among the last 8 visited cells (anti-oscillation) | w_r = 1.2 |

The agent picks `argmax U(a)`, with ties broken by the order Up, Down, Left,
Right, Wait. The decision inspector shows every U(a).

**Worked example** (free corridor, target 5 cells away, no robots nearby):
moving forward gives `10·1 − 1·4 − 0.5 = 5.5` (minus a small obstacle term);
waiting gives `−1·5 − 2·1 = −7`; moving backward gives
`−10 − 6 − 0.5 = −16.5`. The agent moves forward. If the forward cell holds a
robot, that action scores about −19.5, so the agent detours or waits. Each
further wait raises the wait penalty, so after several blocked ticks it backs
off. This trade-off is what the weights encode.

The weights were chosen by hand to give a clear ordering: progress ≫ congestion
> waiting > movement. They are not tuned on the test scenarios. With
distances from a real search, the agent follows shortest routes unless robots
make other cells more attractive. In the experiments that gave the lowest
average distance (294.9 vs 300.1 for Goal-Based) at a much higher search cost
(up to 5 distance queries per step).

## 6. Task allocation baselines (`coordination/task_allocator.py`)

- **Fixed order:** idle robots (by id) take the waiting parcel with the lowest id.
- **Nearest parcel:** idle robots (by id) take the waiting parcel with the smallest Manhattan distance.

Measured over all runs, nearest-parcel allocation reduced average distance from
682.4 to 656.6 cells and steps from 259.2 to 254.0. For the planning agents
alone the reduction is larger, e.g. Goal-Based distance 322.4 → 277.8.

## 7. Agent comparison at a glance

| | Simple Reflex | Goal-Based | Utility-Based |
|---|---|---|---|
| Uses | current percept | goal + search plan | utility over actions + search distances |
| Memory | none (heading only) | current plan | plan cache, recent cells, wait streak |
| Search | none | one search per goal (+ replans) | one search per distinct (cell, goal) |
| Handles robots | avoids occupied neighbours | wait → replan around robots | conflict & congestion penalties |
| Strength | trivial, zero planning cost | optimal routes, cheap | trades distance against congestion/waiting |
| Weakness | gets trapped behind shelves | reacts to robots only when adjacent | many more searches; weights need tuning |
