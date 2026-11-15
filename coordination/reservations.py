"""Per-tick reservation table (vertex + edge reservations).

Each tick the conflict manager records which cell every robot will occupy
at the end of the tick (vertex reservation) and which directed edge it
traverses (edge reservation). A move is refused when its destination cell is
already reserved (vertex conflict) or when another robot traverses the same
edge in the opposite direction (swap / edge conflict).
"""

from __future__ import annotations

from typing import Optional

from environment.grid import Position


class ReservationTable:
    def __init__(self) -> None:
        self.vertex: dict[Position, int] = {}
        self.edges: dict[tuple[Position, Position], int] = {}

    def clear(self) -> None:
        self.vertex.clear()
        self.edges.clear()

    def holder(self, cell: Position) -> Optional[int]:
        return self.vertex.get(cell)

    def is_reserved(self, cell: Position, by_other_than: Optional[int] = None) -> bool:
        holder = self.vertex.get(cell)
        return holder is not None and holder != by_other_than

    def swap_conflict(self, src: Position, dst: Position, robot_id: int) -> Optional[int]:
        """Robot id that reserved the reverse edge ``dst -> src``, if any."""
        other = self.edges.get((dst, src))
        return other if other is not None and other != robot_id else None

    def reserve(self, robot_id: int, src: Position, dst: Position) -> None:
        holder = self.vertex.get(dst)
        if holder is not None and holder != robot_id:
            raise ValueError(f"Cell {dst} already reserved by robot {holder}")
        self.vertex[dst] = robot_id
        if src != dst:
            self.edges[(src, dst)] = robot_id
