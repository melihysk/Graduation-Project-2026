"""Active robot route marker builders."""

from __future__ import annotations

from dataclasses import dataclass

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from . import colors
from .nav_graph_loader import VizNode
from .topics import MARKER_FRAME, ROUTE_MARKER_LIFETIME_SEC


@dataclass(frozen=True)
class RoutePath:
    robot_name: str
    waypoint_node_indices: list[int]
    current_segment_idx: int


def _rgba(r: float, g: float, b: float, a: float) -> ColorRGBA:
    return ColorRGBA(r=r, g=g, b=b, a=a)


def build_route_markers(
    paths: list[RoutePath],
    nodes: dict[int, VizNode],
    *,
    robot_colors: list[tuple[float, float, float]] | None = None,
) -> MarkerArray:
    """Build active route markers for the given robot paths."""
    markers = MarkerArray()
    marker_id = 0
    lifetime = DurationMsg(sec=ROUTE_MARKER_LIFETIME_SEC, nanosec=0)
    palette = robot_colors or colors.ROBOT_COLORS

    sorted_paths = sorted(paths, key=lambda path: path.robot_name)
    for path_index, path in enumerate(sorted_paths):
        if len(path.waypoint_node_indices) < 2:
            continue

        color = palette[path_index % len(palette)]

        route_marker = Marker()
        route_marker.header.frame_id = MARKER_FRAME
        route_marker.ns = f"route_{path.robot_name}"
        route_marker.id = marker_id
        marker_id += 1
        route_marker.type = Marker.LINE_STRIP
        route_marker.action = Marker.ADD
        route_marker.lifetime = lifetime
        route_marker.scale.x = 0.15
        route_marker.color = _rgba(color[0], color[1], color[2], 0.8)
        route_marker.pose.orientation.w = 1.0

        for node_idx in path.waypoint_node_indices[path.current_segment_idx:]:
            node = nodes.get(node_idx)
            if node:
                route_marker.points.append(Point(x=node.x, y=node.y, z=0.1))

        markers.markers.append(route_marker)

        goal_idx = path.waypoint_node_indices[-1]
        goal_node = nodes.get(goal_idx)
        if goal_node:
            goal_marker = Marker()
            goal_marker.header.frame_id = MARKER_FRAME
            goal_marker.ns = f"goal_{path.robot_name}"
            goal_marker.id = marker_id
            marker_id += 1
            goal_marker.type = Marker.CYLINDER
            goal_marker.action = Marker.ADD
            goal_marker.lifetime = lifetime
            goal_marker.pose.position.x = goal_node.x
            goal_marker.pose.position.y = goal_node.y
            goal_marker.pose.position.z = 0.05
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = goal_marker.scale.y = 0.6
            goal_marker.scale.z = 0.1
            goal_marker.color = _rgba(color[0], color[1], color[2], 0.6)
            markers.markers.append(goal_marker)

    return markers
