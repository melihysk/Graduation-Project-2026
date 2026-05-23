"""Minimal nav graph loader for static RViz visualization."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class VizNode:
    idx: int
    name: str
    x: float
    y: float


@dataclass(frozen=True)
class VizEdge:
    from_node: int
    to_node: int


@dataclass(frozen=True)
class NavGraphSnapshot:
    nodes: dict[int, VizNode]
    edges: list[VizEdge]

    def __repr__(self) -> str:
        return (
            f"NavGraphSnapshot(nodes={len(self.nodes)}, edges={len(self.edges)})"
        )


def load_nav_graph(filepath: str | Path) -> NavGraphSnapshot:
    """Load node positions and edges from an RMF nav graph YAML file."""
    filepath = Path(filepath)
    with open(filepath, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    level_data = None
    for ldata in data.get("levels", {}).values():
        if "vertices" in ldata and "lanes" in ldata:
            level_data = ldata
            break

    if level_data is None:
        raise ValueError(f"No level with vertices/lanes in {filepath}")

    nodes: dict[int, VizNode] = {}
    for idx, vertex in enumerate(level_data.get("vertices", [])):
        x = float(vertex[0])
        y = float(vertex[1])
        props = vertex[2] if len(vertex) > 2 else {}
        if isinstance(props, dict):
            name = props.get("name", "") or f"v{idx}"
        else:
            name = f"v{idx}"
        nodes[idx] = VizNode(idx=idx, name=name, x=x, y=y)

    edges: list[VizEdge] = []
    for lane in level_data.get("lanes", []):
        from_idx = int(lane[0])
        to_idx = int(lane[1])
        if from_idx not in nodes or to_idx not in nodes:
            continue
        edges.append(VizEdge(from_node=from_idx, to_node=to_idx))

    return NavGraphSnapshot(nodes=nodes, edges=edges)
