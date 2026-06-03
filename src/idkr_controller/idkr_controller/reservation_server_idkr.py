"""
İDKR Merkezi Kaynak Tahsis Sunucusu — CP-destekli rezervasyon + Res1.

DKR'nin ReservationServer'ını genişletir:
  - Edge kaynakları: aynı mutex davranışı (ters yön engelleme dahil)
  - Normal düğümler (node_X): aynı mutex davranışı
  - CP kaynakları (cp_X_Y): her biri bağımsız mutex, aynı kavşaktaki
    farklı CP'ler eş zamanlı farklı robotlara verilebilir
  - Res1: Kavşakta bir CP dolu iken, o CP'deki robotu başka boş CP'ye
    mantıksal olarak kaydırarak yeni robota yer açma

All-or-nothing semantik korunur.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .cp_manager import CPManager
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
    res1_applied: bool = False
    res1_blocking_robot: str = ""
    res1_original_cp: str = ""
    res1_new_cp: str = ""


class ReservationServerIDKR:
    """
    İDKR merkezi kaynak yöneticisi.

    CP-aware: kavşak düğümlerindeki CP'ler bağımsız mutex kaynak olarak
    yönetilir. Res1 mekanizması ile engelleyen robot boş CP'ye kaydırılabilir.
    """

    def __init__(self, all_resource_ids: list[str], cp_manager: "CPManager"):
        self._resources: dict[str, ResourceState] = {
            rid: ResourceState(resource_id=rid)
            for rid in all_resource_ids
        }
        self._cp_manager = cp_manager
        self._deadlock_detector: "DeadlockDetector | None" = None
        self._robot_holdings: dict[str, set[str]] = {}

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

    def reserve(
        self, robot_name: str, resource_ids: list[str]
    ) -> ReservationResult:
        """
        Atomik kaynak tahsisi (all-or-nothing).

        1. Robot zaten sahipse skip (idempotent)
        2. Boş mu kontrol et (CP + edge + node)
        3. Deadlock döngü kontrolü
        4. Hepsini ver veya hiçbirini verme
        """
        if not resource_ids:
            return ReservationResult(granted=True)

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

        for rid in needed:
            if rid.startswith("cp_"):
                parts = rid.split("_")
                if len(parts) >= 3:
                    jnode = int(parts[1])
                    if self._count_junction_occupants(jnode, robot_name) >= 2:
                        blocker = self._any_junction_occupant(jnode, robot_name)
                        return ReservationResult(
                            granted=False,
                            blocking_robot=blocker or "",
                            blocking_resources=[rid],
                            reason=f"junction_full: max 2 robots at junction {jnode}",
                        )

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

    def try_res1(
        self,
        requesting_robot: str,
        blocked_cp_id: str,
    ) -> ReservationResult:
        """
        Res1 mekanizması: engelli CP'deki robotu boş bir CP'ye kaydır.

        1. blocked_cp_id'nin sahibini (blocking_robot) bul
        2. Aynı kavşakta boş CP var mı kontrol et
        3. Varsa: blocking_robot'un rezervasyonunu boş CP'ye taşı
        4. blocked_cp_id artık boş → requesting_robot'a ver

        Returns:
            ReservationResult — granted=True ise Res1 başarılı, CP tahsis edildi.
        """
        if not blocked_cp_id.startswith("cp_"):
            return ReservationResult(
                granted=False,
                reason="res1_not_applicable: not a CP resource",
            )

        state = self._resources.get(blocked_cp_id)
        if state is None:
            return ReservationResult(granted=False, reason="unknown_cp")

        blocking_robot = state.owner
        if blocking_robot is None:
            result = self.reserve(requesting_robot, [blocked_cp_id])
            result.res1_applied = False
            return result

        if blocking_robot == requesting_robot:
            return ReservationResult(granted=True)

        parts = blocked_cp_id.split("_")
        if len(parts) < 3:
            return ReservationResult(
                granted=False,
                blocking_robot=blocking_robot,
                reason="res1_invalid_cp_format",
            )
        junction_node = int(parts[1])

        junction_info = self._cp_manager.get_junction_info(junction_node)
        if junction_info is None:
            return ReservationResult(
                granted=False,
                blocking_robot=blocking_robot,
                reason="res1_not_junction",
            )

        free_cp_id: str | None = None
        for cp in junction_info.control_points:
            if cp.resource_id == blocked_cp_id:
                continue
            cp_state = self._resources.get(cp.resource_id)
            if cp_state and cp_state.owner is None:
                free_cp_id = cp.resource_id
                break

        if free_cp_id is None:
            return ReservationResult(
                granted=False,
                blocking_robot=blocking_robot,
                blocking_resources=[blocked_cp_id],
                reason=f"res1_no_free_cp: junction {junction_node} full",
            )

        now = time.time()
        state.owner = None
        state.lock_time = 0.0
        if blocking_robot in self._robot_holdings:
            self._robot_holdings[blocking_robot].discard(blocked_cp_id)

        free_state = self._resources[free_cp_id]
        free_state.owner = blocking_robot
        free_state.lock_time = now
        if blocking_robot not in self._robot_holdings:
            self._robot_holdings[blocking_robot] = set()
        self._robot_holdings[blocking_robot].add(free_cp_id)

        state.owner = requesting_robot
        state.lock_time = now
        if requesting_robot not in self._robot_holdings:
            self._robot_holdings[requesting_robot] = set()
        self._robot_holdings[requesting_robot].add(blocked_cp_id)

        return ReservationResult(
            granted=True,
            res1_applied=True,
            reason=f"res1: {blocking_robot} moved from {blocked_cp_id} to {free_cp_id}",
            res1_blocking_robot=blocking_robot,
            res1_original_cp=blocked_cp_id,
            res1_new_cp=free_cp_id,
        )

    def undo_res1(self, res1_result: "ReservationResult") -> None:
        """Res1 sonrası kalan kaynaklar başarısız olursa orijinal durumu geri yükle."""
        if not res1_result.res1_applied:
            return

        br = res1_result.res1_blocking_robot
        orig_cp = res1_result.res1_original_cp
        new_cp = res1_result.res1_new_cp

        new_state = self._resources.get(new_cp)
        if new_state and new_state.owner == br:
            new_state.owner = None
            new_state.lock_time = 0.0
            if br in self._robot_holdings:
                self._robot_holdings[br].discard(new_cp)

        orig_state = self._resources.get(orig_cp)
        if orig_state and orig_state.owner is None:
            orig_state.owner = br
            orig_state.lock_time = 0.0
            if br not in self._robot_holdings:
                self._robot_holdings[br] = set()
            self._robot_holdings[br].add(orig_cp)

    def _count_junction_occupants(self, junction_node: int, exclude: str) -> int:
        """Kavşaktaki farklı robot sayısı (exclude hariç)."""
        info = self._cp_manager.get_junction_info(junction_node)
        if info is None:
            return 0
        occupants: set[str] = set()
        for cp in info.control_points:
            st = self._resources.get(cp.resource_id)
            if st and st.owner and st.owner != exclude:
                occupants.add(st.owner)
        return len(occupants)

    def _any_junction_occupant(self, junction_node: int, exclude: str) -> str | None:
        info = self._cp_manager.get_junction_info(junction_node)
        if info is None:
            return None
        for cp in info.control_points:
            st = self._resources.get(cp.resource_id)
            if st and st.owner and st.owner != exclude:
                return st.owner
        return None

    def _rid_and_reverse(self, rid: str) -> list[str]:
        rev = self._reverse_edge.get(rid)
        return [rid, rev] if rev else [rid]

    def release(self, robot_name: str, resource_ids: list[str]) -> bool:
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
        held = list(self._robot_holdings.get(robot_name, set()))
        if held:
            self.release(robot_name, held)
        return held

