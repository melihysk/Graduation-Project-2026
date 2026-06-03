"""
İDKR Çatışma Sınıflandırıcısı — 3 tip çatışma tespiti.

Verma, Olm, Suárez (IEEE Access, 2024) Definition 7:

  - HEAD_ON (i):      Zıt yönde aynı edge/kavşak girişi → Res1.
  - INTERSECTION (ii): Rotalar bir node'da buluşup ayrılıyor → Res1.
  - PURSUIT (iii):    Aynı yönde paylaşılan rota → takipçi bekler.

Loop conflict (iv) SFP ve deadlock_detector tarafından ele alınır.
"""

from __future__ import annotations

from enum import Enum, auto
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .cp_manager import CPManager


class ConflictType(Enum):
    HEAD_ON = auto()
    INTERSECTION = auto()
    PURSUIT = auto()


@dataclass
class ConflictInfo:
    conflict_type: ConflictType
    robot_a: str
    robot_b: str
    resource_id: str
    junction_node: int | None = None


class ConflictClassifier:
    """Deny durumunda çatışma tipini belirler; Res1 kararı için kullanılır."""

    def __init__(self, cp_manager: "CPManager"):
        self._cp_manager = cp_manager

    def classify(
        self,
        requesting_robot: str,
        blocking_robot: str,
        blocked_resource: str,
        requesting_path: list[int] | None = None,
        blocking_path: list[int] | None = None,
    ) -> ConflictInfo:
        if blocked_resource.startswith("cp_"):
            return self._classify_cp_conflict(
                requesting_robot, blocking_robot, blocked_resource,
                requesting_path, blocking_path,
            )

        if blocked_resource.startswith("edge_"):
            return self._classify_edge_conflict(
                requesting_robot, blocking_robot, blocked_resource,
                blocking_path,
            )

        if blocked_resource.startswith("node_"):
            return ConflictInfo(
                conflict_type=ConflictType.INTERSECTION,
                robot_a=requesting_robot,
                robot_b=blocking_robot,
                resource_id=blocked_resource,
            )

        return ConflictInfo(
            conflict_type=ConflictType.PURSUIT,
            robot_a=requesting_robot,
            robot_b=blocking_robot,
            resource_id=blocked_resource,
        )

    def _classify_cp_conflict(
        self,
        requesting_robot: str,
        blocking_robot: str,
        blocked_resource: str,
        requesting_path: list[int] | None,
        blocking_path: list[int] | None,
    ) -> ConflictInfo:
        parts = blocked_resource.split("_")
        junction_node = int(parts[1]) if len(parts) >= 3 else None

        if self._is_head_on_at_junction(
            requesting_path, blocking_path, junction_node,
        ):
            return ConflictInfo(
                conflict_type=ConflictType.HEAD_ON,
                robot_a=requesting_robot,
                robot_b=blocking_robot,
                resource_id=blocked_resource,
                junction_node=junction_node,
            )

        return ConflictInfo(
            conflict_type=ConflictType.INTERSECTION,
            robot_a=requesting_robot,
            robot_b=blocking_robot,
            resource_id=blocked_resource,
            junction_node=junction_node,
        )

    def _classify_edge_conflict(
        self,
        requesting_robot: str,
        blocking_robot: str,
        blocked_resource: str,
        blocking_path: list[int] | None,
    ) -> ConflictInfo:
        parts = blocked_resource.split("_")
        if len(parts) == 3:
            from_n, to_n = int(parts[1]), int(parts[2])

            if self._robot_holds_reverse_edge(blocking_path, from_n, to_n):
                return ConflictInfo(
                    conflict_type=ConflictType.HEAD_ON,
                    robot_a=requesting_robot,
                    robot_b=blocking_robot,
                    resource_id=blocked_resource,
                )

            if self._is_pursuit_conflict(blocking_path, from_n, to_n):
                return ConflictInfo(
                    conflict_type=ConflictType.PURSUIT,
                    robot_a=requesting_robot,
                    robot_b=blocking_robot,
                    resource_id=blocked_resource,
                )

        return ConflictInfo(
            conflict_type=ConflictType.PURSUIT,
            robot_a=requesting_robot,
            robot_b=blocking_robot,
            resource_id=blocked_resource,
        )

    def _is_head_on_at_junction(
        self,
        requesting_path: list[int] | None,
        blocking_path: list[int] | None,
        junction_node: int | None,
    ) -> bool:
        if not requesting_path or not blocking_path or junction_node is None:
            return False

        req_from: int | None = None
        req_to: int | None = None
        if junction_node in requesting_path:
            j = requesting_path.index(junction_node)
            if j > 0:
                req_from = requesting_path[j - 1]
            if j < len(requesting_path) - 1:
                req_to = requesting_path[j + 1]

        blk_from: int | None = None
        blk_to: int | None = None
        if junction_node in blocking_path:
            j = blocking_path.index(junction_node)
            if j > 0:
                blk_from = blocking_path[j - 1]
            if j < len(blocking_path) - 1:
                blk_to = blocking_path[j + 1]

        if req_from is not None and blk_to is not None and req_from == blk_to:
            return True
        if blk_from is not None and req_to is not None and blk_from == req_to:
            return True

        return False

    @staticmethod
    def _robot_holds_reverse_edge(
        blocking_path: list[int] | None,
        from_n: int,
        to_n: int,
    ) -> bool:
        if not blocking_path:
            return False

        for i in range(len(blocking_path) - 1):
            if blocking_path[i] == to_n and blocking_path[i + 1] == from_n:
                return True
        return False

    @staticmethod
    def _is_pursuit_conflict(
        blocking_path: list[int] | None,
        from_n: int,
        to_n: int,
    ) -> bool:
        if not blocking_path:
            return False

        for i in range(len(blocking_path) - 1):
            if blocking_path[i] == from_n and blocking_path[i + 1] == to_n:
                return True
        return False

    def should_attempt_res1(self, conflict: ConflictInfo) -> bool:
        if conflict.junction_node is None:
            return False
        if not self._cp_manager.is_junction(conflict.junction_node):
            return False
        if conflict.conflict_type == ConflictType.PURSUIT:
            return False
        return conflict.conflict_type in (
            ConflictType.HEAD_ON,
            ConflictType.INTERSECTION,
        )
