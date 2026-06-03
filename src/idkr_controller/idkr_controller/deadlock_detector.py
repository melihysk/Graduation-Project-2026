"""
Wait-for graph üzerinde DFS ile deadlock (döngü) tespiti.

Wait-for graph:
  - Node: robot adı
  - Edge: A → B  ⟺  robot A, robot B'nin tuttuğu bir kaynağı bekliyor

Döngü = deadlock. Çözüm: döngüdeki en düşük öncelikli robot geri çekilir.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class WaitEdge:
    waiting_robot: str
    blocking_robot: str
    contested_resource: str


class DeadlockDetector:
    """
    DKR deadlock algılama ve çözüm.

    Öncelik: daha düşük priority değeri = daha yüksek öncelik.
    Varsayılan olarak robot adının sıra numarası kullanılır
    (warehouseRobot1 > warehouseRobot2 > ...).
    """

    def __init__(self):
        # wait-for graph: robot → set of robots it's waiting for
        self._wait_for: dict[str, dict[str, str]] = {}
        # robot priorities (lower value = higher priority)
        self._priorities: dict[str, int] = {}
        self._deadlock_count: int = 0

    @property
    def deadlock_count(self) -> int:
        return self._deadlock_count

    def set_priority(self, robot_name: str, priority: int) -> None:
        self._priorities[robot_name] = priority

    def get_priority(self, robot_name: str) -> int:
        if robot_name in self._priorities:
            return self._priorities[robot_name]
        # Extract numeric suffix as default priority
        digits = "".join(c for c in robot_name if c.isdigit())
        return int(digits) if digits else 99

    def add_wait(
        self, waiting_robot: str, blocking_robot: str, resource: str
    ) -> None:
        """Robot A, robot B'nin tuttuğu kaynağı bekliyor."""
        if waiting_robot not in self._wait_for:
            self._wait_for[waiting_robot] = {}
        self._wait_for[waiting_robot][blocking_robot] = resource

    def remove_wait(self, waiting_robot: str, blocking_robot: str | None = None) -> None:
        """Bekleme kaydını sil (grant alındığında veya robot yol değiştirdiğinde)."""
        if blocking_robot is None:
            self._wait_for.pop(waiting_robot, None)
        elif waiting_robot in self._wait_for:
            self._wait_for[waiting_robot].pop(blocking_robot, None)
            if not self._wait_for[waiting_robot]:
                del self._wait_for[waiting_robot]

    def clear_robot(self, robot_name: str) -> None:
        """Robot ile ilgili tüm wait kayıtlarını temizle."""
        self._wait_for.pop(robot_name, None)
        # Also remove edges pointing TO this robot
        for waiter in list(self._wait_for.keys()):
            self._wait_for[waiter].pop(robot_name, None)
            if not self._wait_for[waiter]:
                del self._wait_for[waiter]

    def update_waits_after_res1(
        self, blocking_robot: str, old_resource: str,
    ) -> None:
        """Res1 sonrası eski kaynak sahipliğine bağlı wait edge'lerini temizle.

        blocking_robot artık old_resource'u tutmuyor; bu kaynağa dayanan
        wait edge'leri geçersiz. Bir sonraki retry döngüsünde yeni sahibine
        göre güncel edge'ler oluşturulacak.
        """
        for waiter in list(self._wait_for.keys()):
            blockers = self._wait_for.get(waiter)
            if not blockers:
                continue
            if blockers.get(blocking_robot) == old_resource:
                del blockers[blocking_robot]
                if not blockers:
                    del self._wait_for[waiter]

    def would_cause_cycle(
        self, requesting_robot: str, blocking_robot: str
    ) -> bool:
        """
        requesting_robot → blocking_robot edge eklense döngü oluşur mu?

        DFS: blocking_robot'tan başlayarak requesting_robot'a ulaşılabilir mi?
        """
        if requesting_robot == blocking_robot:
            return True

        visited: set[str] = set()
        stack: list[str] = [blocking_robot]

        while stack:
            current = stack.pop()
            if current == requesting_robot:
                return True
            if current in visited:
                continue
            visited.add(current)

            # current'ın beklediği robotları stack'e ekle
            for next_robot in self._wait_for.get(current, {}):
                if next_robot not in visited:
                    stack.append(next_robot)

        return False

    def detect_cycle(self) -> list[str] | None:
        """
        Wait-for graph'ta herhangi bir döngü var mı?

        DFS-based cycle detection. Döngüdeki robotları döndürür veya None.
        """
        visited: set[str] = set()
        rec_stack: set[str] = set()
        parent_map: dict[str, str] = {}

        def _dfs(node: str) -> list[str] | None:
            visited.add(node)
            rec_stack.add(node)

            for neighbor in self._wait_for.get(node, {}):
                if neighbor not in visited:
                    parent_map[neighbor] = node
                    result = _dfs(neighbor)
                    if result is not None:
                        return result
                elif neighbor in rec_stack:
                    # Cycle found — reconstruct
                    cycle = [neighbor]
                    cur = node
                    while cur != neighbor:
                        cycle.append(cur)
                        cur = parent_map.get(cur, neighbor)
                    cycle.append(neighbor)
                    return cycle

            rec_stack.discard(node)
            return None

        for robot in list(self._wait_for.keys()):
            if robot not in visited:
                parent_map.clear()
                cycle = _dfs(robot)
                if cycle is not None:
                    self._deadlock_count += 1
                    return cycle

        return None

    def resolve_deadlock(self, cycle_robots: list[str]) -> str | None:
        """
        Döngüdeki en düşük öncelikli (en yüksek priority değeri) robotu seç.

        Bu robot geri çekilecek (yield) — kaynakları release edilecek ve
        yolun başına geri dönecek.
        """
        if not cycle_robots:
            return None

        # Remove duplicate start/end from cycle path
        unique = list(dict.fromkeys(cycle_robots))

        worst_robot = max(unique, key=lambda r: self.get_priority(r))
        return worst_robot

    def get_wait_graph_snapshot(self) -> list[WaitEdge]:
        """Mevcut wait-for graph'ın bir kopyasını döndür (loglama/debug için)."""
        edges: list[WaitEdge] = []
        for waiter, blockers in self._wait_for.items():
            for blocker, resource in blockers.items():
                edges.append(WaitEdge(waiter, blocker, resource))
        return edges
