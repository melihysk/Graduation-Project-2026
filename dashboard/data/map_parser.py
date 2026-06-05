"""Parse warehouse_starter.building.yaml into vertices and lanes for visualization."""

from pathlib import Path
from dataclasses import dataclass

import yaml


BUILDING_YAML = (
    Path.home() / "Desktop" / "graduation_project" / "src"
    / "rmf_demos" / "rmf_demos_maps" / "maps" / "warehouse_starter"
    / "warehouse_starter.building.yaml"
)

VERTEX_COLORS = {
    "PICK": "#89b4fa",
    "DROP": "#fab387",
    "T_NORTH": "#f38ba8",
    "X_CENTER": "#f38ba8",
    "X_SOUTH": "#f38ba8",
    "charger": "#a6e3a1",
    "WAIT_HUB": "#f9e2af",
    "HOLD": "#f9e2af",
    "N_": "#7f849c",
}

DEFAULT_COLOR = "#585b70"


@dataclass
class Vertex:
    index: int
    x: float
    y: float
    name: str
    color: str
    is_charger: bool = False
    is_pickup: bool = False
    is_dropoff: bool = False


@dataclass
class Lane:
    v1: int
    v2: int
    bidirectional: bool


def _vertex_color(name: str) -> str:
    for prefix, color in VERTEX_COLORS.items():
        if prefix in name:
            return color
    return DEFAULT_COLOR


def parse_map(path: Path | None = None) -> tuple[list[Vertex], list[Lane]]:
    path = path or BUILDING_YAML
    if not path.exists():
        return [], []

    data = yaml.safe_load(path.read_text())
    level = data.get("levels", {}).get("L1", {})
    raw_verts = level.get("vertices", [])
    raw_lanes = level.get("lanes", [])

    vertices: list[Vertex] = []
    for i, v in enumerate(raw_verts):
        x = float(v[0])
        y = float(v[1])
        name = v[3] if len(v) > 3 else ""
        params = v[4] if len(v) > 4 else {}

        is_charger = bool(params.get("is_charger", [None, False])[-1]) if isinstance(params, dict) else False
        is_pickup = "pickup_dispenser" in params if isinstance(params, dict) else False
        is_dropoff = "dropoff_ingestor" in params if isinstance(params, dict) else False

        color = _vertex_color(name) if name else DEFAULT_COLOR
        vertices.append(Vertex(
            index=i, x=x, y=y, name=name,
            color=color, is_charger=is_charger,
            is_pickup=is_pickup, is_dropoff=is_dropoff,
        ))

    lanes: list[Lane] = []
    for ln in raw_lanes:
        v1 = ln[0]
        v2 = ln[1]
        params = ln[2] if len(ln) > 2 else {}
        bidir_val = params.get("bidirectional", [None, True])
        bidir = bidir_val[-1] if isinstance(bidir_val, list) else bool(bidir_val)
        lanes.append(Lane(v1=v1, v2=v2, bidirectional=bidir))

    return vertices, lanes
