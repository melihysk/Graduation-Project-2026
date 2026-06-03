"""
İDKR Kaynak Grafiği — CP-destekli kaynak modeli.

DKR'nin ResourceGraph'ını genişletir:
  - Kavşak düğümleri (derece >= 3): node_X yerine cp_X_0, cp_X_1, ... kullanılır
  - Normal düğümler (derece <= 2): DKR ile aynı → node_X (mutex, kapasite 1)
  - Edge'ler: DKR ile aynı → edge_A_B (mutex, kapasite 1)

Yol kaynakları hesaplanırken kavşak düğümleri CP kaynak ID'lerine dönüştürülür.
Hangi CP'nin seçileceği gelinen/gidilen yöne bağlıdır.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .cp_manager import CPManager


@dataclass
class NodeResource:
    idx: int
    name: str
    x: float
    y: float
    neighbors: list[int] = field(default_factory=list)

    @property
    def resource_id(self) -> str:
        return f"node_{self.idx}"


@dataclass
class EdgeResource:
    from_node: int
    to_node: int

    @property
    def resource_id(self) -> str:
        return f"edge_{self.from_node}_{self.to_node}"


class ResourceGraphIDKR:
    """
    İDKR kaynak grafiği — kavşak düğümlerini CP alt-kaynaklarına böler.
    """

    def __init__(self):
        self.nodes: dict[int, NodeResource] = {}
        self.edges: dict[str, EdgeResource] = {}
        self._name_to_idx: dict[str, int] = {}
        self._cp_manager: CPManager | None = None

    @property
    def cp_manager(self) -> CPManager:
        if self._cp_manager is None:
            raise RuntimeError("CP manager not initialized — call from_graph_file first")
        return self._cp_manager

    @classmethod
    def from_nav_graph_yaml(cls, filepath: str | Path) -> "ResourceGraphIDKR":
        filepath = Path(filepath)
        with open(filepath, "r") as f:
            data = yaml.safe_load(f)

        graph = cls()

        level_data = None
        for ldata in data.get("levels", {}).values():
            if "vertices" in ldata and "lanes" in ldata:
                level_data = ldata
                break

        if level_data is None:
            raise ValueError(f"No level with vertices/lanes in {filepath}")

        vertices = level_data.get("vertices", [])
        lanes = level_data.get("lanes", [])

        for idx, v in enumerate(vertices):
            x = float(v[0])
            y = float(v[1])
            props = v[2] if len(v) > 2 else {}
            if isinstance(props, dict):
                name = props.get("name", "") or f"v{idx}"
            else:
                name = f"v{idx}"

            graph.nodes[idx] = NodeResource(idx=idx, name=name, x=x, y=y)
            if name and not name.startswith("v"):
                graph._name_to_idx[name] = idx

        edge_tuples: list[tuple[int, int]] = []
        for lane in lanes:
            from_idx = int(lane[0])
            to_idx = int(lane[1])

            if from_idx not in graph.nodes or to_idx not in graph.nodes:
                continue

            edge = EdgeResource(from_node=from_idx, to_node=to_idx)
            graph.edges[edge.resource_id] = edge
            edge_tuples.append((from_idx, to_idx))

            if to_idx not in graph.nodes[from_idx].neighbors:
                graph.nodes[from_idx].neighbors.append(to_idx)

        graph._cp_manager = CPManager(graph.nodes, edge_tuples)
        return graph

    @classmethod
    def from_graph_file(cls, filepath: str | Path) -> "ResourceGraphIDKR":
        filepath = Path(filepath)
        with open(filepath, "r") as f:
            data = yaml.safe_load(f)

        for ldata in data.get("levels", {}).values():
            verts = ldata.get("vertices", [])
            if verts and len(verts[0]) >= 2:
                first = verts[0]
                if len(first) == 3 and isinstance(first[2], dict):
                    return cls.from_nav_graph_yaml(filepath)

        raise ValueError(f"Unrecognized graph format: {filepath}")

    def node_idx_by_name(self, name: str) -> int | None:
        return self._name_to_idx.get(name)

    def get_path_resources_idkr(
        self, waypoint_indices: list[int]
    ) -> list[str]:
        """
        Waypoint dizisini İDKR kaynak ID listesine dönüştürür.

        Kavşak düğümleri: sadece CP kaynağı (cp_X_Y) — eş zamanlı geçiş mümkün.
        Normal düğümler: node_X (mutex, kapasite 1).
        Edge'ler: edge_A_B (mutex).
        """
        if not waypoint_indices:
            return []

        resources: list[str] = []

        first_idx = waypoint_indices[0]
        resources.append(self._node_resource_id(first_idx, prev_node=None))

        for i in range(len(waypoint_indices) - 1):
            from_idx = waypoint_indices[i]
            to_idx = waypoint_indices[i + 1]
            resources.append(f"edge_{from_idx}_{to_idx}")
            resources.append(self._node_resource_id(to_idx, prev_node=from_idx))

        return resources

    def get_segment_resources_idkr(
        self, from_node_idx: int, to_node_idx: int
    ) -> list[str]:
        """
        Tek segment kaynakları (İDKR versiyonu).

        Kavşak düğümleri: [edge_from_to, cp_to_Y] — CP eş zamanlı geçiş sağlar.
        Normal düğümler: [edge_from_to, node_to] — tek mutex.
        """
        resources = [f"edge_{from_node_idx}_{to_node_idx}"]
        node_rid = self._node_resource_id(to_node_idx, prev_node=from_node_idx)
        resources.append(node_rid)
        return resources

    def _node_resource_id(self, node_idx: int, prev_node: int | None) -> str:
        """
        Düğüm için uygun kaynak ID'sini döndür.

        Kavşaksa ve prev_node biliniyorsa → CP resource ID
        Kavşaksa ama prev_node bilinmiyorsa → ilk CP (cp_X_0)
        Normal düğümse → node_X
        """
        if self._cp_manager and self._cp_manager.is_junction(node_idx):
            if prev_node is not None:
                cp = self._cp_manager.get_entry_cp(node_idx, prev_node)
                if cp:
                    return cp.resource_id
            info = self._cp_manager.get_junction_info(node_idx)
            if info and info.control_points:
                return info.control_points[0].resource_id
        return f"node_{node_idx}"

    def find_path_bfs(self, start_idx: int, goal_idx: int) -> list[int] | None:
        """BFS ile iki node arasında en kısa yolu bulur."""
        if start_idx == goal_idx:
            return [start_idx]
        if start_idx not in self.nodes or goal_idx not in self.nodes:
            return None

        visited: set[int] = {start_idx}
        queue: deque[list[int]] = deque([[start_idx]])

        while queue:
            path = queue.popleft()
            current = path[-1]

            for neighbor in self.nodes[current].neighbors:
                if neighbor == goal_idx:
                    return path + [neighbor]
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(path + [neighbor])

        return None

    def all_resource_ids(self) -> list[str]:
        """
        Tüm kaynak ID'leri.

        Normal düğümler: node_X (mutex, kapasite 1)
        Kavşak düğümleri: sadece cp_X_0, cp_X_1, ... (node_X YOK)
            Farklı CP'ler farklı robotlara eş zamanlı verilebilir.
        Edge'ler: edge_A_B (mutex, kapasite 1)
        """
        ids: list[str] = []

        for node in self.nodes.values():
            if self._cp_manager and self._cp_manager.is_junction(node.idx):
                info = self._cp_manager.get_junction_info(node.idx)
                if info:
                    ids.extend(info.get_all_resource_ids())
            else:
                ids.append(node.resource_id)

        ids.extend(e.resource_id for e in self.edges.values())
        return ids

    def __repr__(self) -> str:
        cp_info = ""
        if self._cp_manager:
            cp_info = f", {self._cp_manager}"
        return (
            f"ResourceGraphIDKR(nodes={len(self.nodes)}, "
            f"edges={len(self.edges)}{cp_info})"
        )
