"""
Nav graph → resource graph dönüşümü.

RMF nav_graphs/0.yaml (metre, fleet adapter ile aynı koordinatlar) veya
building.yaml (piksel) dosyasından kaynak modeli oluşturur.

DKR çalıştırırken nav_graphs/0.yaml kullanın — PathRequest koordinatları
bununla uyumludur.

Kaynak modeli:
  - Her nav graph node'u bir NodeResource (aynı anda 1 robot)
  - Her lane yönü bir EdgeResource (aynı anda 1 robot)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml


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

    @property
    def length(self) -> float:
        """Placeholder — actual length set by ResourceGraph after construction."""
        return 0.0


class ResourceGraph:
    """
    DKR kaynak grafiği.

    Nav graph'taki node ve edge'leri birer mutex kaynağa dönüştürür.
    Robot bir kaynağı kullanmadan önce reserve etmeli, geçtikten sonra
    release etmelidir.
    """

    def __init__(self):
        self.nodes: dict[int, NodeResource] = {}
        self.edges: dict[str, EdgeResource] = {}
        self._edge_lengths: dict[str, float] = {}
        # Waypoint name → node idx lookup
        self._name_to_idx: dict[str, int] = {}

    @classmethod
    def from_building_yaml(cls, filepath: str | Path) -> "ResourceGraph":
        """Parse a building.yaml and build the resource graph."""
        filepath = Path(filepath)
        with open(filepath, "r") as f:
            data = yaml.safe_load(f)

        graph = cls()

        level_data = None
        levels = data.get("levels", {})
        for level_name, ldata in levels.items():
            if "lanes" in ldata:
                level_data = ldata
                break

        if level_data is None:
            raise ValueError(f"No level with lanes found in {filepath}")

        vertices = level_data.get("vertices", [])
        lanes = level_data.get("lanes", [])

        # Parse vertices — only those used in nav graph lanes are relevant.
        # Vertex format: [x, y, z, name, {params}] or [x, y, z, ""]
        lane_vertex_indices: set[int] = set()
        for lane in lanes:
            lane_vertex_indices.add(lane[0])
            lane_vertex_indices.add(lane[1])

        for idx in sorted(lane_vertex_indices):
            if idx >= len(vertices):
                continue
            v = vertices[idx]
            x = float(v[0])
            y = float(v[1])
            name = v[3] if len(v) > 3 and v[3] else f"v{idx}"

            node = NodeResource(idx=idx, name=name, x=x, y=y)
            graph.nodes[idx] = node
            if name and name != f"v{idx}":
                graph._name_to_idx[name] = idx

        # Parse lanes — build edge resources and neighbor lists.
        for lane in lanes:
            from_idx = lane[0]
            to_idx = lane[1]
            params = lane[2] if len(lane) > 2 else {}

            bidirectional = True
            if isinstance(params, dict):
                bi_param = params.get("bidirectional", [4, True])
                if isinstance(bi_param, list) and len(bi_param) >= 2:
                    bidirectional = bool(bi_param[1])
                elif isinstance(bi_param, bool):
                    bidirectional = bi_param

            # Forward edge
            edge_fwd = EdgeResource(from_node=from_idx, to_node=to_idx)
            graph.edges[edge_fwd.resource_id] = edge_fwd

            if from_idx in graph.nodes and to_idx not in graph.nodes[from_idx].neighbors:
                graph.nodes[from_idx].neighbors.append(to_idx)

            # Reverse edge if bidirectional
            if bidirectional:
                edge_rev = EdgeResource(from_node=to_idx, to_node=from_idx)
                graph.edges[edge_rev.resource_id] = edge_rev

                if to_idx in graph.nodes and from_idx not in graph.nodes[to_idx].neighbors:
                    graph.nodes[to_idx].neighbors.append(from_idx)

        # Compute edge lengths
        for eid, edge in graph.edges.items():
            n1 = graph.nodes.get(edge.from_node)
            n2 = graph.nodes.get(edge.to_node)
            if n1 and n2:
                dist = math.hypot(n2.x - n1.x, n2.y - n1.y)
                graph._edge_lengths[eid] = dist

        return graph

    @classmethod
    def from_nav_graph_yaml(cls, filepath: str | Path) -> "ResourceGraph":
        """
        RMF nav_graphs/0.yaml formatını parse eder.

        Vertex format: [x, y, {name: ..., ...}]
        Lane format: [from_idx, to_idx, {}] — her kayıt tek yönlü edge.
        """
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

        for lane in lanes:
            from_idx = int(lane[0])
            to_idx = int(lane[1])

            if from_idx not in graph.nodes or to_idx not in graph.nodes:
                continue

            edge = EdgeResource(from_node=from_idx, to_node=to_idx)
            graph.edges[edge.resource_id] = edge

            if to_idx not in graph.nodes[from_idx].neighbors:
                graph.nodes[from_idx].neighbors.append(to_idx)

        for eid, edge in graph.edges.items():
            n1 = graph.nodes.get(edge.from_node)
            n2 = graph.nodes.get(edge.to_node)
            if n1 and n2:
                graph._edge_lengths[eid] = math.hypot(n2.x - n1.x, n2.y - n1.y)

        return graph

    @classmethod
    def from_graph_file(cls, filepath: str | Path) -> "ResourceGraph":
        """Dosya formatına göre uygun parser'ı seçer."""
        filepath = Path(filepath)
        with open(filepath, "r") as f:
            data = yaml.safe_load(f)

        # nav_graph: levels.L1.vertices = [[x,y,{name:...}], ...]
        for ldata in data.get("levels", {}).values():
            verts = ldata.get("vertices", [])
            if verts and len(verts[0]) >= 2:
                first = verts[0]
                # building.yaml: [x,y,z,name,...] — 4+ elements, z often 0
                # nav_graph: [x,y,{props}] — 3 elements, props is dict
                if len(first) == 3 and isinstance(first[2], dict):
                    return cls.from_nav_graph_yaml(filepath)
                if len(first) >= 4:
                    return cls.from_building_yaml(filepath)

        raise ValueError(f"Unrecognized graph format: {filepath}")

    def get_edge_length(self, edge_id: str) -> float:
        return self._edge_lengths.get(edge_id, 0.0)

    def node_idx_by_name(self, name: str) -> int | None:
        return self._name_to_idx.get(name)

    def get_path_resources(self, waypoint_indices: list[int]) -> list[str]:
        """
        Bir waypoint dizisini kaynak ID listesine dönüştürür.

        Dönen liste: [node_start, edge_start_next, node_next, edge_next_..., ..., node_end]
        Robot bu kaynakları sırasıyla reserve etmelidir.
        """
        if not waypoint_indices:
            return []

        resources: list[str] = []
        resources.append(f"node_{waypoint_indices[0]}")

        for i in range(len(waypoint_indices) - 1):
            from_idx = waypoint_indices[i]
            to_idx = waypoint_indices[i + 1]
            resources.append(f"edge_{from_idx}_{to_idx}")
            resources.append(f"node_{to_idx}")

        return resources

    def get_segment_resources(
        self, from_node_idx: int, to_node_idx: int
    ) -> list[str]:
        """
        Tek bir segment (edge + hedef node) için gereken kaynak ID'leri.

        Return: [edge_from_to, node_to]
        """
        return [
            f"edge_{from_node_idx}_{to_node_idx}",
            f"node_{to_node_idx}",
        ]

    def find_path_bfs(self, start_idx: int, goal_idx: int) -> list[int] | None:
        """BFS ile iki node arasında en kısa yolu bulur (waypoint indeks listesi)."""
        if start_idx == goal_idx:
            return [start_idx]
        if start_idx not in self.nodes or goal_idx not in self.nodes:
            return None

        from collections import deque

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

    def closest_node_to_position(self, x: float, y: float) -> int | None:
        """Verilen koordinata en yakın node indeksini döndürür."""
        best_idx = None
        best_dist = float("inf")
        for idx, node in self.nodes.items():
            dist = math.hypot(node.x - x, node.y - y)
            if dist < best_dist:
                best_dist = dist
                best_idx = idx
        return best_idx

    def all_resource_ids(self) -> list[str]:
        """Tüm kaynak ID'lerini döndürür."""
        ids = [n.resource_id for n in self.nodes.values()]
        ids.extend(e.resource_id for e in self.edges.values())
        return ids

    def __repr__(self) -> str:
        return (
            f"ResourceGraph(nodes={len(self.nodes)}, "
            f"edges={len(self.edges)}, "
            f"resources={len(self.nodes) + len(self.edges)})"
        )
