"""
Basit fleet_states aggregatörü.

Slotcar plugin'den gelen bireysel /robot_state mesajlarını toplar,
tek bir /fleet_states (FleetState) mesajı olarak yayınlar.

Bu node olmadan rmf_visualization robotları RViz'de gösteremez
ve metric_logger görev tamamlanmasını izleyemez.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy as History,
    QoSReliabilityPolicy as Reliability,
    QoSDurabilityPolicy as Durability,
)
from rmf_fleet_msgs.msg import FleetState, RobotState


class FleetStatePublisher(Node):

    def __init__(self):
        super().__init__("fleet_state_publisher")

        self.declare_parameter("fleet_name", "warehouseRobot")
        self.declare_parameter("publish_rate", 2.0)

        self._fleet_name = self.get_parameter("fleet_name").value
        rate = self.get_parameter("publish_rate").value

        self._robot_states: dict[str, RobotState] = {}

        reliable_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.VOLATILE,
        )

        self.create_subscription(
            RobotState, "robot_state", self._on_robot_state, reliable_qos,
        )

        self._fleet_pub = self.create_publisher(
            FleetState, "fleet_states", reliable_qos,
        )

        self.create_timer(1.0 / rate, self._publish_fleet_state)
        self.get_logger().info(
            f"Fleet state publisher started for fleet '{self._fleet_name}'"
        )

    def _on_robot_state(self, msg: RobotState):
        self._robot_states[msg.name] = msg

    def _publish_fleet_state(self):
        if not self._robot_states:
            return
        fleet_msg = FleetState()
        fleet_msg.name = self._fleet_name
        fleet_msg.robots = list(self._robot_states.values())
        self._fleet_pub.publish(fleet_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FleetStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
