"""Utility-Based agent: scores every legal action and picks the best.

For each candidate action ``a`` leading to cell ``n`` (``n`` = current cell for
WAIT) the agent computes

    U(a) = w_progress   * progress(a)          (d(cur) - d(n), +1 closer / -1 farther)
         - w_distance   * d(n)                  true path distance to target
         - w_move       * move_cost(a)          1 for a move, 0 for wait
         - w_obstacle   * obstacle_prox(n)      blocked neighbours of n / 4
         - w_conflict   * conflict(n)           1 if n is occupied by a robot
         - w_congestion * congestion(n)         robots within Manhattan radius 2 of n
         - w_wait       * wait_penalty(a)       (1 + consecutive waits) for WAIT
         - w_revisit    * recent_visits(n)      visits of n in the last 8 cells

``d(n)`` is the length of the shortest path from ``n`` to the target computed
with the selected algorithm (BFS or A*) on the static shelf map; results are
cached per (cell, target) so each distinct query is searched once.
Ties are broken by the fixed action order Up, Down, Left, Right, Wait.
"""

from __future__ import annotations

from collections import deque
from typing import Optional

from agents.base_agent import (
    FREE,
    ROBOT,
    BaseAgent,
    CandidateAction,
    Decision,
    Percept,
)
from algorithms.common import SearchResult
from config.settings import ALGORITHM_LABELS, DEFAULT_UTILITY_WEIGHTS, UtilityWeights
from environment.grid import MOVE_ACTIONS, Action, Position, apply_move, manhattan

RECENT_WINDOW = 8


class UtilityBasedAgent(BaseAgent):
    agent_type = "utility_based"
    display_name = "Utility-Based"
    uses_search = True

    def __init__(self, *args, weights: UtilityWeights = DEFAULT_UTILITY_WEIGHTS, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.weights = weights
        self.recent: deque[Position] = deque(maxlen=RECENT_WINDOW)
        self.consecutive_waits = 0
        self._cache: dict[Position, SearchResult] = {}
        self._cache_target: Optional[Position] = None
        self.last_search: Optional[SearchResult] = None

    def reset_task_memory(self) -> None:
        self._cache.clear()
        self._cache_target = None
        self.recent.clear()

    # ------------------------------------------------------------------ #
    def distance(self, cell: Position, target: Position) -> Optional[int]:
        """Static shortest-path distance (cached); ``None`` if unreachable."""
        if self._cache_target != target:
            self._cache.clear()
            self._cache_target = target
        if cell not in self._cache:
            self._cache[cell] = self.pathfinder.find_path(cell, target)
            self.last_search = self._cache[cell]
        return self._cache[cell].cost

    def features(self, percept: Percept, action: Action) -> Optional[dict[str, float]]:
        """Feature vector for ``action`` or ``None`` when the action is illegal/useless."""
        assert percept.target is not None
        cur = percept.position
        d_cur = self.distance(cur, percept.target)
        if d_cur is None:
            return None
        if action == Action.WAIT:
            nxt, status = cur, FREE
        else:
            status = percept.neighbors[action]
            if status not in (FREE, ROBOT):
                return None
            nxt = apply_move(cur, action)
        d_next = self.distance(nxt, percept.target)
        if d_next is None:
            return None
        others = percept.other_robots.values()
        return {
            "progress": float(d_cur - d_next),
            "distance": float(d_next),
            "move_cost": 0.0 if action == Action.WAIT else 1.0,
            "obstacle": self.grid.blocked_neighbor_count(nxt) / 4.0,
            "conflict": 1.0 if status == ROBOT else 0.0,
            "congestion": float(sum(1 for p in others if manhattan(p, nxt) <= 2)),
            "wait": float(1 + self.consecutive_waits) if action == Action.WAIT else 0.0,
            "revisit": float(sum(1 for p in self.recent if p == nxt)) if action != Action.WAIT else 0.0,
        }

    def utility(self, f: dict[str, float]) -> float:
        w = self.weights
        return (
            w.progress * f["progress"]
            - w.distance * f["distance"]
            - w.move * f["move_cost"]
            - w.obstacle * f["obstacle"]
            - w.conflict * f["conflict"]
            - w.congestion * f["congestion"]
            - w.wait * f["wait"]
            - w.revisit * f["revisit"]
        )

    def decide(self, percept: Percept) -> Decision:
        rule = self.task_rule(percept)
        if rule is not None:
            self.reset_task_memory()
            self.consecutive_waits = 0
            return rule

        if not self.recent or self.recent[-1] != percept.position:
            self.recent.append(percept.position)

        candidates: list[CandidateAction] = []
        feats: dict[str, dict[str, float]] = {}
        best: Optional[tuple[float, Action]] = None
        for action in (*MOVE_ACTIONS, Action.WAIT):
            f = self.features(percept, action)
            if f is None:
                note = percept.neighbors.get(action, "unreachable target")
                candidates.append(CandidateAction(action, False, None, note))
                continue
            score = self.utility(f)
            feats[action.name] = f
            candidates.append(
                CandidateAction(
                    action, True, round(score, 2),
                    f"d={int(f['distance'])} prog={f['progress']:+.0f} "
                    f"cong={f['congestion']:.0f}"
                    + (" CONFLICT" if f["conflict"] else ""),
                )
            )
            if best is None or score > best[0]:
                best = (score, action)

        if best is None:
            self.consecutive_waits += 1
            return Decision(
                Action.WAIT,
                f"Target {percept.target_label} is unreachable from here. Waiting.",
                candidates,
                search=self.last_search,
            )

        score, action = best
        ranked = sorted((c for c in candidates if c.valid), key=lambda c: -(c.score or 0))
        runner_up = ranked[1] if len(ranked) > 1 else None
        self.consecutive_waits = self.consecutive_waits + 1 if action == Action.WAIT else 0

        nxt = percept.position if action == Action.WAIT else apply_move(percept.position, action)
        route = self._cache.get(nxt)
        plan = ([percept.position] + route.path) if route and route.found and action != Action.WAIT else (
            route.path if route else []
        )
        verb = "Waiting" if action == Action.WAIT else f"{action.name.title()} movement"
        reason = (
            f"{verb} selected because it has the highest utility "
            f"U={score:.2f}"
            + (f" (next best: {runner_up.action.name.title()} U={runner_up.score:.2f})" if runner_up else "")
            + f". Distance to {percept.target_label} computed with "
            f"{ALGORITHM_LABELS[self.pathfinder.algorithm]}."
        )
        return Decision(action, reason, candidates, plan=plan, search=self.last_search, features=feats)
