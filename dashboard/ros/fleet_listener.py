"""ROS 2 fleet_states subscriber running in a QThread."""

from PyQt6.QtCore import QThread, pyqtSignal


class FleetListener(QThread):
    """Subscribes to /fleet_states and emits robot position updates."""

    fleet_updated = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = False
        self._node = None

    def run(self):
        try:
            import rclpy
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
            from rmf_fleet_msgs.msg import FleetState
        except ImportError:
            return

        rclpy.init()
        self._node = rclpy.create_node("dashboard_fleet_listener")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._node.create_subscription(FleetState, "/fleet_states", self._on_fleet, qos)
        self._running = True

        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)

        self._node.destroy_node()
        rclpy.shutdown()

    def _on_fleet(self, msg):
        data = {}
        for robot in msg.robots:
            data[robot.name] = {
                "x": robot.location.x,
                "y": robot.location.y,
                "yaw": robot.location.yaw,
                "mode": robot.mode.mode,
                "task_id": getattr(robot, "task_id", ""),
            }
        self.fleet_updated.emit(data)

    def stop(self):
        self._running = False
        self.wait(3000)
