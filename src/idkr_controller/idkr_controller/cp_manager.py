"""
Kontrol Noktası (CP) Yöneticisi — İDKR kavşak alt-bölümleme sistemi.

Nav graph topolojisinden kavşak düğümlerini tespit eder (derece >= 3) ve
her kavşağı yönsel Kontrol Noktalarına böler.

CP Modeli:
  - Her kavşak düğümü, komşu sayısı kadar CP'ye sahiptir.
  - Her CP bir yöne (komşu düğüme) karşılık gelir.
  - Bir robot kavşağa belirli bir yönden girdiğinde, o yönün CP'sini alır.
  - Farklı yönlerden giren robotlar farklı CP'leri kullanır → eş zamanlı geçiş.
  - Aynı CP'yi isteyen iki robot → Res1 çözümü (birini boş CP'ye kaydır).

Kaynak ID formatı: "cp_{junction_node_idx}_{cp_index}"
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class ControlPoint:
    """Bir kavşak düğümü içindeki tek bir kontrol noktası."""
    junction_node: int
    cp_index: int
    direction_neighbor: int
    resource_id: str


@dataclass
class JunctionInfo:
    """Kavşak düğümü hakkında tüm CP bilgisi."""
    node_idx: int
    junction_type: str  # "T" | "X" | "MULTI"
    control_points: list[ControlPoint] = field(default_factory=list)

    @property
    def capacity(self) -> int:
        return len(self.control_points)

    def get_cp_for_neighbor(self, neighbor_node: int) -> ControlPoint | None:
        """Belirli bir komşu yönüne karşılık gelen CP'yi döndür."""
        for cp in self.control_points:
            if cp.direction_neighbor == neighbor_node:
                return cp
        return None

    def get_all_resource_ids(self) -> list[str]:
        """Bu kavşağın tüm CP kaynak ID'lerini döndür."""
        return [cp.resource_id for cp in self.control_points]


class CPManager:
    """
    Nav graph topolojisinden kavşak düğümlerini tespit eder ve
    CP alt-kaynaklarını yönetir.
    """

    def __init__(self, nodes: dict[int, object], edges: list[tuple[int, int]]):
        """
        Args:
            nodes: {idx: node_object} — node_object'in .neighbors listesi olmalı
                   veya edges'ten hesaplanacak.
            edges: [(from_node, to_node), ...] — yönlü kenarlar listesi
        """
        self._junctions: dict[int, JunctionInfo] = {}
        self._node_neighbors: dict[int, list[int]] = {}

        self._build_neighbor_map(nodes, edges)
        self._detect_junctions()

    def _build_neighbor_map(
        self,
        nodes: dict[int, object],
        edges: list[tuple[int, int]],
    ) -> None:
        """Her düğüm için benzersiz komşu listesi oluştur."""
        for idx in nodes:
            self._node_neighbors[idx] = []

        for from_n, to_n in edges:
            if to_n not in self._node_neighbors.get(from_n, []):
                if from_n in self._node_neighbors:
                    self._node_neighbors[from_n].append(to_n)
            if from_n not in self._node_neighbors.get(to_n, []):
                if to_n in self._node_neighbors:
                    self._node_neighbors[to_n].append(from_n)

    def _detect_junctions(self) -> None:
        """Derece >= 3 olan düğümleri kavşak olarak işaretle ve CP'ler oluştur."""
        for node_idx, neighbors in self._node_neighbors.items():
            unique_neighbors = sorted(set(neighbors))
            degree = len(unique_neighbors)

            if degree < 3:
                continue

            if degree == 3:
                jtype = "T"
            elif degree == 4:
                jtype = "X"
            else:
                jtype = "MULTI"

            cps = []
            for cp_idx, neighbor in enumerate(unique_neighbors):
                cp = ControlPoint(
                    junction_node=node_idx,
                    cp_index=cp_idx,
                    direction_neighbor=neighbor,
                    resource_id=f"cp_{node_idx}_{cp_idx}",
                )
                cps.append(cp)

            self._junctions[node_idx] = JunctionInfo(
                node_idx=node_idx,
                junction_type=jtype,
                control_points=cps,
            )

    def is_junction(self, node_idx: int) -> bool:
        return node_idx in self._junctions

    def get_junction_info(self, node_idx: int) -> JunctionInfo | None:
        return self._junctions.get(node_idx)

    def get_entry_cp(self, junction_node: int, from_node: int) -> ControlPoint | None:
        """
        Robot junction_node'a from_node yönünden girdiğinde hangi CP'yi alacak.
        Giriş CP'si = geldiği komşu yönünün CP'si.
        """
        info = self._junctions.get(junction_node)
        if info is None:
            return None
        return info.get_cp_for_neighbor(from_node)

    @property
    def junctions(self) -> dict[int, JunctionInfo]:
        return self._junctions

    def find_simple_cycles(self, max_length: int = 8) -> list[list[int]]:
        """Graf'taki basit cycle'ları bul (SFP kontrolü için).

        Sadece kavşak düğümlerini içeren cycle'lar döndürülür.
        max_length ile arama derinliği sınırlanır.
        """
        cycles: list[list[int]] = []
        junction_nodes = set(self._junctions.keys())

        if not junction_nodes:
            return cycles

        visited_cycles: set[tuple[int, ...]] = set()

        def _dfs(start: int, current: int, path: list[int], depth: int):
            if depth > max_length:
                return

            for neighbor in self._node_neighbors.get(current, []):
                if neighbor == start and len(path) >= 3:
                    canonical = tuple(sorted(path))
                    if canonical not in visited_cycles:
                        visited_cycles.add(canonical)
                        cycles.append(list(path))
                    continue

                if neighbor in path:
                    continue

                if neighbor not in junction_nodes:
                    continue

                path.append(neighbor)
                _dfs(start, neighbor, path, depth + 1)
                path.pop()

        for node in sorted(junction_nodes):
            _dfs(node, node, [node], 1)

        return cycles

    def __repr__(self) -> str:
        counts = {"T": 0, "X": 0, "MULTI": 0}
        for j in self._junctions.values():
            counts[j.junction_type] += 1
        total_cps = sum(j.capacity for j in self._junctions.values())
        return (
            f"CPManager({len(self._junctions)} junctions: "
            f"{counts['T']}×T, {counts['X']}×X, {counts['MULTI']}×MULTI, "
            f"{total_cps} total CPs)"
        )
