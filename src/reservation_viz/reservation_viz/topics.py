"""Topic names and QoS profiles for reservation traffic markers."""

from rclpy.qos import (
    QoSDurabilityPolicy as Durability,
    QoSHistoryPolicy as History,
    QoSProfile,
    QoSReliabilityPolicy as Reliability,
)

MARKER_FRAME = "map"
STATIC_MARKER_LIFETIME_SEC = 0
ROUTE_MARKER_LIFETIME_SEC = 2
DEFAULT_TOPIC_PREFIX = "dkr"


def graph_markers_topic(prefix: str = DEFAULT_TOPIC_PREFIX) -> str:
    return f"/{prefix}_markers"


def route_markers_topic(prefix: str = DEFAULT_TOPIC_PREFIX) -> str:
    return f"/{prefix}_route_markers"


def graph_markers_qos() -> QoSProfile:
    return QoSProfile(
        history=History.KEEP_LAST,
        depth=1,
        reliability=Reliability.RELIABLE,
        durability=Durability.TRANSIENT_LOCAL,
    )
