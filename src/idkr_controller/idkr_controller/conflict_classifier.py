"""
İDKR Çatışma Sınıflandırıcısı — 4 tip çatışma tespiti.

Çatışma Tipleri:
  - HEAD_ON:      Aynı edge üzerinde zıt yönde iki robot.
                  Çözüm: Reverse-edge blocking zaten engeller. Kavşakta ise Res1.
  - INTERSECTION: Farklı yönlerden aynı kavşağa yaklaşan robotlar, aynı CP'yi istiyorlar.
                  Çözüm: Res1 — bekleyen robotu boş CP'ye kaydır.
  - PURSUIT:      Aynı yönde ardışık iki robot (aynı edge'e girmeye çalışıyor).
                  Çözüm: Arkadaki robot bekler (edge mutex bunu doğal sağlar).
  - LOOP:         Dairesel bekleme zinciri (deadlock).
                  Çözüm: DFS döngü tespiti + kurban seçimi (deadlock_detector).
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
    LOOP = auto()


@dataclass
class ConflictInfo:
    """Tespit edilen bir çatışmanın detayları."""
    conflict_type: ConflictType
    robot_a: str
    robot_b: str
    resource_id: str
    junction_node: int | None = None
    resolution_hint: str = ""


class ConflictClassifier:
    """
    Aktif yollar ve kaynak talepleri üzerinden çatışma tipini belirler.
    Traffic manager deny durumunda bu sınıflandırıcıyı çağırır.
    """

    def __init__(self, cp_manager: "CPManager"):
        self._cp_manager = cp_manager

    def classify(
        self,
        requesting_robot: str,
        blocking_robot: str,
        blocked_resource: str,
        requesting_path: list[int] | None = None,
        blocking_path: list[int] | None = None,
        requesting_segment_idx: int = 0,
        blocking_segment_idx: int = 0,
    ) -> ConflictInfo:
        """
        Deny olan bir çatışmayı sınıflandır.

        Args:
            requesting_robot: Talep eden (deny alan) robot
            blocking_robot: Kaynağı tutan robot
            blocked_resource: Deny olan kaynak ID'si
            requesting_path: Talep eden robotun yol düğümleri
            blocking_path: Engelleyen robotun yol düğümleri
            requesting_segment_idx: Talep edenin mevcut segment indeksi
            blocking_segment_idx: Engelleyenin mevcut segment indeksi
        """
        if blocked_resource.startswith("cp_"):
            return self._classify_cp_conflict(
                requesting_robot, blocking_robot, blocked_resource,
                requesting_path, blocking_path,
                requesting_segment_idx, blocking_segment_idx,
            )

        if blocked_resource.startswith("edge_"):
            return self._classify_edge_conflict(
                requesting_robot, blocking_robot, blocked_resource,
                requesting_path, blocking_path,
                requesting_segment_idx, blocking_segment_idx,
            )

        if blocked_resource.startswith("node_"):
            return ConflictInfo(
                conflict_type=ConflictType.INTERSECTION,
                robot_a=requesting_robot,
                robot_b=blocking_robot,
                resource_id=blocked_resource,
                resolution_hint="node_mutex_wait",
            )

        return ConflictInfo(
            conflict_type=ConflictType.PURSUIT,
            robot_a=requesting_robot,
            robot_b=blocking_robot,
            resource_id=blocked_resource,
            resolution_hint="unknown_resource_wait",
        )

    def _classify_cp_conflict(
        self,
        requesting_robot: str,
        blocking_robot: str,
        blocked_resource: str,
        requesting_path: list[int] | None,
        blocking_path: list[int] | None,
        requesting_segment_idx: int,
        blocking_segment_idx: int,
    ) -> ConflictInfo:
        """CP kaynağı üzerindeki çatışma — INTERSECTION tipi."""
        parts = blocked_resource.split("_")
        junction_node = int(parts[1]) if len(parts) >= 3 else None

        if self._is_head_on_at_junction(
            requesting_path, blocking_path,
            requesting_segment_idx, blocking_segment_idx,
            junction_node,
        ):
            return ConflictInfo(
                conflict_type=ConflictType.HEAD_ON,
                robot_a=requesting_robot,
                robot_b=blocking_robot,
                resource_id=blocked_resource,
                junction_node=junction_node,
                resolution_hint="res1_relocate",
            )

        return ConflictInfo(
            conflict_type=ConflictType.INTERSECTION,
            robot_a=requesting_robot,
            robot_b=blocking_robot,
            resource_id=blocked_resource,
            junction_node=junction_node,
            resolution_hint="res1_relocate",
        )

    def _classify_edge_conflict(
        self,
        requesting_robot: str,
        blocking_robot: str,
        blocked_resource: str,
        requesting_path: list[int] | None,
        blocking_path: list[int] | None,
        requesting_segment_idx: int,
        blocking_segment_idx: int,
    ) -> ConflictInfo:
        """Edge kaynağı üzerindeki çatışma — HEAD_ON veya PURSUIT."""
        parts = blocked_resource.split("_")
        if len(parts) == 3:
            from_n, to_n = int(parts[1]), int(parts[2])
            reverse_edge = f"edge_{to_n}_{from_n}"

            if self._robot_holds_reverse_edge(
                blocking_robot, blocking_path, blocking_segment_idx,
                from_n, to_n,
            ):
                return ConflictInfo(
                    conflict_type=ConflictType.HEAD_ON,
                    robot_a=requesting_robot,
                    robot_b=blocking_robot,
                    resource_id=blocked_resource,
                    resolution_hint="edge_mutex_wait",
                )

        return ConflictInfo(
            conflict_type=ConflictType.PURSUIT,
            robot_a=requesting_robot,
            robot_b=blocking_robot,
            resource_id=blocked_resource,
            resolution_hint="edge_mutex_wait",
        )

    def _is_head_on_at_junction(
        self,
        requesting_path: list[int] | None,
        blocking_path: list[int] | None,
        requesting_segment_idx: int,
        blocking_segment_idx: int,
        junction_node: int | None,
    ) -> bool:
        """İki robot kavşakta zıt yönlerden mi geliyor kontrol et.

        HEAD_ON: bir robotun giriş yönü diğerinin çıkış yönü (veya tersi).
        """
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

    def _robot_holds_reverse_edge(
        self,
        blocking_robot: str,
        blocking_path: list[int] | None,
        blocking_segment_idx: int,
        from_n: int,
        to_n: int,
    ) -> bool:
        """Engelleyen robot ters yöndeki edge'i mi tutuyor."""
        if not blocking_path:
            return False

        for i in range(len(blocking_path) - 1):
            if blocking_path[i] == to_n and blocking_path[i + 1] == from_n:
                return True
        return False

    def should_attempt_res1(self, conflict: ConflictInfo) -> bool:
        """Bu çatışma tipi için Res1 denemeli mi?"""
        if conflict.junction_node is None:
            return False
        if not self._cp_manager.is_junction(conflict.junction_node):
            return False
        return conflict.conflict_type in (
            ConflictType.HEAD_ON,
            ConflictType.INTERSECTION,
        )
