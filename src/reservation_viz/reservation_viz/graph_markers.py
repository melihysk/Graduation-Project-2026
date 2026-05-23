"""Static nav graph marker builders."""

from __future__ import annotations

from typing import Protocol

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from . import colors
from .nav_graph_loader import NavGraphSnapshot, VizEdge, VizNode
from .topics import MARKER_FRAME, STATIC_MARKER_LIFETIME_SEC


class GraphLike(Protocol):
    nodes: dict[int, VizNode]
    edges: list[VizEdge] | dict[str, VizEdge]


def _rgba(r: float, g: float, b: float, a: float) -> ColorRGBA:
    return ColorRGBA(r=r, g=g, b=b, a=a)


def _waypoint_color(name: str) -> ColorRGBA:
    if name.startswith("PICK"):
        return _rgba(*colors.PICK_COLOR)
    if name.startswith("DROP"):
        return _rgba(*colors.DROP_COLOR)
    if name.endswith("_charger"):
        return _rgba(*colors.CHARGER_COLOR)
    return _rgba(*colors.DEFAULT_WAYPOINT_COLOR)


def _iter_edges(graph: GraphLike):
    edge_values = graph.edges.values() if isinstance(graph.edges, dict) else graph.edges
    for edge in edge_values:
        yield edge


def build_graph_markers(graph: GraphLike | NavGraphSnapshot) -> MarkerArray:
    """Build static nav graph markers from a graph snapshot or duck-typed graph."""
    markers = MarkerArray()
    marker_id = 0
    lifetime = DurationMsg(sec=STATIC_MARKER_LIFETIME_SEC, nanosec=0)

    edge_marker = Marker()
    edge_marker.header.frame_id = MARKER_FRAME
    edge_marker.ns = "nav_graph_edges"
    edge_marker.id = marker_id
    marker_id += 1
    edge_marker.type = Marker.LINE_LIST
    edge_marker.action = Marker.ADD
    edge_marker.scale.x = 0.08
    edge_marker.color = _rgba(*colors.EDGE_COLOR)
    edge_marker.lifetime = lifetime
    edge_marker.pose.orientation.w = 1.0

    seen_edges: set[tuple[int, int]] = set()
    for edge in _iter_edges(graph):
        pair = (min(edge.from_node, edge.to_node), max(edge.from_node, edge.to_node))
        if pair in seen_edges:
            continue
        seen_edges.add(pair)
        node_a = graph.nodes.get(edge.from_node)
        node_b = graph.nodes.get(edge.to_node)
        if node_a and node_b:
            edge_marker.points.append(Point(x=node_a.x, y=node_a.y, z=0.0))
            edge_marker.points.append(Point(x=node_b.x, y=node_b.y, z=0.0))

    markers.markers.append(edge_marker)

    for node in graph.nodes.values():
        is_pick = node.name.startswith("PICK")
        is_drop = node.name.startswith("DROP")
        is_charger = node.name.endswith("_charger")

        sphere = Marker()
        sphere.header.frame_id = MARKER_FRAME
        sphere.ns = "waypoints"
        sphere.id = marker_id
        marker_id += 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.lifetime = lifetime
        sphere.pose.position.x = node.x
        sphere.pose.position.y = node.y
        sphere.pose.position.z = 0.05
        sphere.pose.orientation.w = 1.0
        size = 0.5 if (is_pick or is_drop) else 0.3
        sphere.scale.x = sphere.scale.y = sphere.scale.z = size
        sphere.color = _waypoint_color(node.name)
        markers.markers.append(sphere)

        if is_pick or is_drop or is_charger:
            label = Marker()
            label.header.frame_id = MARKER_FRAME
            label.ns = "waypoint_labels"
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.lifetime = lifetime
            label.pose.position.x = node.x
            label.pose.position.y = node.y
            label.pose.position.z = 0.8
            label.pose.orientation.w = 1.0
            label.scale.z = 0.4
            label.color = _rgba(*colors.LABEL_COLOR)
            label.text = node.name
            markers.markers.append(label)

    return markers
