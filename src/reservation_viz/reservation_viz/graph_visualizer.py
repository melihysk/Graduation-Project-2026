"""
Statik nav graph görselleştiricisi.

Simülasyon launch'ı ile başlar (warehouse_starter_standalone.launch.xml).
Nav graph kenarlarını, PICK/DROP/charger düğümlerini /{prefix}_markers topic'ine yazar.
Aktif robot rotaları traffic manager tarafından /{prefix}_route_markers'a publish edilir.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

from .graph_markers import build_graph_markers
from .nav_graph_loader import load_nav_graph
from .topics import graph_markers_qos, graph_markers_topic


class GraphVisualizer(Node):

    def __init__(self):
        super().__init__("graph_visualizer")

        self.declare_parameter("nav_graph_file", "")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("topic_prefix", "dkr")

        nav_graph_file = self.get_parameter("nav_graph_file").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        topic_prefix = self.get_parameter("topic_prefix").value

        if not nav_graph_file:
            self.get_logger().error("nav_graph_file parameter is required")
            raise SystemExit(1)

        self._graph = load_nav_graph(nav_graph_file)
        self.get_logger().info(f"Graph visualizer loaded: {self._graph}")

        topic = graph_markers_topic(topic_prefix)
        self._marker_pub = self.create_publisher(
            MarkerArray, topic, graph_markers_qos(),
        )

        period = 1.0 / max(publish_rate_hz, 0.1)
        self.create_timer(period, self._publish_graph_markers)
        self._publish_graph_markers()

    def _publish_graph_markers(self) -> None:
        self._marker_pub.publish(build_graph_markers(self._graph))


def main(args=None):
    rclpy.init(args=args)
    node = GraphVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
