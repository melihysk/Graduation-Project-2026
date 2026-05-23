"""
Merkezi kaynak tahsis sunucusu (DKR).

Her nav graph node ve edge'i birer mutex kaynak olarak yönetir.
All-or-nothing semantik: ya tüm istenen kaynaklar verilir ya hiçbiri.
Reserve öncesi deadlock_detector ile döngü kontrolü yapılır.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .deadlock_detector import DeadlockDetector


@dataclass
class ResourceState:
    resource_id: str
    owner: str | None = None
    lock_time: float = 0.0


@dataclass
class ReservationResult:
    granted: bool
    blocking_robot: str = ""
    blocking_resources: list[str] = field(default_factory=list)
    reason: str = ""


class ReservationServer:
    """
    Merkezi kaynak yöneticisi.

    Thread-safe değil — tek ROS 2 executor thread'inde çalışacak şekilde
    tasarlanmıştır (SingleThreadedExecutor).
    """

    def __init__(self, all_resource_ids: list[str]):
        self._resources: dict[str, ResourceState] = {
            rid: ResourceState(resource_id=rid)
            for rid in all_resource_ids
        }
        self._deadlock_detector: DeadlockDetector | None = None

        # Robot → currently held resources (for quick lookup)
        self._robot_holdings: dict[str, set[str]] = {}

        # Bidirectional edge pairing: edge_A_B ↔ edge_B_A share a physical
        # corridor. Reserving one must block the other to prevent head-on
        # collisions.
        self._reverse_edge: dict[str, str] = {}
        for rid in all_resource_ids:
            rev = self._make_reverse_edge_id(rid)
            if rev and rev in self._resources:
                self._reverse_edge[rid] = rev

    @staticmethod
    def _make_reverse_edge_id(resource_id: str) -> str | None:
        if not resource_id.startswith("edge_"):
            return None
        parts = resource_id.split("_")
        if len(parts) == 3:
            return f"edge_{parts[2]}_{parts[1]}"
        return None

    def set_deadlock_detector(self, detector: "DeadlockDetector") -> None:
        self._deadlock_detector = detector

    @property
    def robot_holdings(self) -> dict[str, set[str]]:
        return self._robot_holdings

    def get_owner(self, resource_id: str) -> str | None:
        state = self._resources.get(resource_id)
        return state.owner if state else None

    def get_all_reservations(self) -> dict[str, str | None]:
        """Return {resource_id: owner_or_None} for all locked resources."""
        return {
            rid: s.owner
            for rid, s in self._resources.items()
            if s.owner is not None
        }

    def get_robot_resources(self, robot_name: str) -> set[str]:
        return self._robot_holdings.get(robot_name, set())

    def reserve(
        self, robot_name: str, resource_ids: list[str]
    ) -> ReservationResult:
        """
        Atomik kaynak tahsisi (all-or-nothing).

        1. Robot zaten sahipse skip (idempotent)
        2. Boş mu kontrol et
        3. Deadlock döngü kontrolü
        4. Hepsini ver veya hiçbirini verme
        """
        if not resource_ids:
            return ReservationResult(granted=True)

        # Filter out resources already owned by this robot (idempotent re-request)
        needed: list[str] = []
        for rid in resource_ids:
            state = self._resources.get(rid)
            if state is None:
                return ReservationResult(
                    granted=False,
                    reason=f"Unknown resource: {rid}",
                )
            if state.owner != robot_name:
                needed.append(rid)

        if not needed:
            return ReservationResult(granted=True)

        # Check availability — find first blocker (including reverse edges)
        for rid in needed:
            for check_rid in self._rid_and_reverse(rid):
                state = self._resources.get(check_rid)
                if state is None:
                    continue
                if state.owner is not None and state.owner != robot_name:
                    if self._deadlock_detector:
                        would_cycle = self._deadlock_detector.would_cause_cycle(
                            requesting_robot=robot_name,
                            blocking_robot=state.owner,
                        )
                        if would_cycle:
                            return ReservationResult(
                                granted=False,
                                blocking_robot=state.owner,
                                blocking_resources=[rid],
                                reason=f"deadlock_risk: cycle with {state.owner}",
                            )

                    return ReservationResult(
                        granted=False,
                        blocking_robot=state.owner,
                        blocking_resources=[rid],
                        reason=f"resource_busy: {check_rid} held by {state.owner}",
                    )

        # All needed resources are free — grant atomically (plus reverse edges)
        now = time.time()
        granted_extras: list[str] = []
        for rid in needed:
            state = self._resources[rid]
            state.owner = robot_name
            state.lock_time = now
            rev = self._reverse_edge.get(rid)
            if rev:
                rev_state = self._resources.get(rev)
                if rev_state and rev_state.owner is None:
                    rev_state.owner = robot_name
                    rev_state.lock_time = now
                    granted_extras.append(rev)

        if robot_name not in self._robot_holdings:
            self._robot_holdings[robot_name] = set()
        self._robot_holdings[robot_name].update(needed)
        self._robot_holdings[robot_name].update(granted_extras)

        return ReservationResult(granted=True)

    def _rid_and_reverse(self, rid: str) -> list[str]:
        """Return [rid] or [rid, reverse_edge] for corridor check."""
        rev = self._reverse_edge.get(rid)
        return [rid, rev] if rev else [rid]

    def release(self, robot_name: str, resource_ids: list[str]) -> bool:
        """
        Kaynakları serbest bırak (ters yönlü edge'ler dahil).

        Sadece robot kendi sahip olduğu kaynakları release edebilir.
        """
        released_any = False
        all_to_release: list[str] = []
        for rid in resource_ids:
            all_to_release.append(rid)
            rev = self._reverse_edge.get(rid)
            if rev:
                all_to_release.append(rev)

        for rid in all_to_release:
            state = self._resources.get(rid)
            if state and state.owner == robot_name:
                state.owner = None
                state.lock_time = 0.0
                released_any = True

        if robot_name in self._robot_holdings:
            self._robot_holdings[robot_name] -= set(all_to_release)
            if not self._robot_holdings[robot_name]:
                del self._robot_holdings[robot_name]

        return released_any

    def release_all(self, robot_name: str) -> list[str]:
        """Robot'un tüm kaynaklarını serbest bırak."""
        held = list(self._robot_holdings.get(robot_name, set()))
        if held:
            self.release(robot_name, held)
        return held

    def force_release(self, resource_id: str) -> str | None:
        """Deadlock çözümü için kaynağı zorla serbest bırak. Eski sahibi döner."""
        state = self._resources.get(resource_id)
        if state and state.owner:
            prev_owner = state.owner
            state.owner = None
            state.lock_time = 0.0
            if prev_owner in self._robot_holdings:
                self._robot_holdings[prev_owner].discard(resource_id)
            return prev_owner
        return None
