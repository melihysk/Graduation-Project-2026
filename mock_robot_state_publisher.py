# #!/usr/bin/env python3
# """
# Mock robot simulator for warehouse robots.

# Gazebo olmadan RMF'in tam hareket dongusunu simule eder:
#   - fleet_manager'dan gelen PathRequest dinler
#   - Her timer tick'inde bir sonraki waypoint'e isinlanir (lineer hareket yok)
#   - Her adimda robot_state yayimlar
#   - Son waypoint'e gelince MODE_IDLE bildirir

# Robot baslangic konumlari nav graph charger noktalarindan alinmistir:
#   warehouseRobot1_charger: (10.0,    -26.5)
#   warehouseRobot2_charger: (14.5915, -27.4479)
#   warehouseRobot3_charger: (19.4547, -27.5)
#   warehouseRobot4_charger: (24.2124, -26.949)

# Kullanim:
#   cd ~/Desktop/graduation_project && source install/setup.bash
#   python3 mock_robot_state_publisher.py

#   # Hizli test — her 0.05s'de bir waypoint gec (cok hizli):
#   python3 mock_robot_state_publisher.py --ros-args -p waypoint_interval_sec:=0.05

#   # Yavas/gercekci — her 2.0s'de bir waypoint gec:
#   python3 mock_robot_state_publisher.py --ros-args -p waypoint_interval_sec:=2.0
# """

# import rclpy
# from rclpy.node import Node
# from rmf_fleet_msgs.msg import RobotState, Location, RobotMode, PathRequest

# # Baslangic konumlari: nav_graphs/0.yaml icindeki charger koordinatlarindan
# ROBOTS = [
#     {'name': 'warehouseRobot1', 'x': 10.0,    'y': -26.5,    'yaw': 0.0},
#     {'name': 'warehouseRobot2', 'x': 14.5915, 'y': -27.4479, 'yaw': 0.0},
#     {'name': 'warehouseRobot3', 'x': 19.4547, 'y': -27.5,    'yaw': 0.0},
#     {'name': 'warehouseRobot4', 'x': 24.2124, 'y': -26.949,  'yaw': 0.0},
# ]

# MAP_NAME = 'L1'

# # Varsayilan: her 0.1s'de bir waypoint ilerle
# # (10 waypoint'lik bir path yaklasik 1 saniyede biter)
# DEFAULT_WAYPOINT_INTERVAL_SEC = 0.1


# class RobotSim:
#     """Tek bir robotun konum ve gorev durumunu tutar."""

#     def __init__(self, name: str, x: float, y: float, yaw: float):
#         self.name = name
#         self.x = x
#         self.y = y
#         self.yaw = yaw
#         self.task_id = '0'
#         self.battery = 95.0
#         self.mode = RobotMode.MODE_IDLE

#         self.path: list = []
#         self.current_target_idx: int = 0

#     def step(self) -> None:
#         """Bir sonraki waypoint'e isinla; path bitmisse IDLE'a don."""
#         if not self.path or self.current_target_idx >= len(self.path):
#             self.mode = RobotMode.MODE_IDLE
#             return

#         target = self.path[self.current_target_idx]
#         self.x = target.x
#         self.y = target.y
#         self.yaw = target.yaw
#         self.current_target_idx += 1

#         if self.current_target_idx >= len(self.path):
#             self.path = []
#             self.current_target_idx = 0
#             self.mode = RobotMode.MODE_IDLE
#         else:
#             self.mode = RobotMode.MODE_MOVING

#     def set_path(self, path_request: PathRequest) -> None:
#         """fleet_manager'dan gelen PathRequest'i isle.

#         RMF convention: path[0] mevcut konum, path[1:] hedefler.
#         """
#         self.path = list(path_request.path[1:]) if len(path_request.path) > 1 else []
#         self.current_target_idx = 0
#         self.task_id = path_request.task_id
#         self.mode = RobotMode.MODE_MOVING if self.path else RobotMode.MODE_IDLE


# class MockRobotSimulator(Node):

#     def __init__(self):
#         super().__init__('mock_robot_simulator')

#         self.declare_parameter('waypoint_interval_sec', DEFAULT_WAYPOINT_INTERVAL_SEC)
#         interval: float = self.get_parameter('waypoint_interval_sec').value

#         if interval <= 0.0:
#             self.get_logger().warn(
#                 f'waypoint_interval_sec={interval} gecersiz, '
#                 f'{DEFAULT_WAYPOINT_INTERVAL_SEC}s kullaniliyor.'
#             )
#             interval = DEFAULT_WAYPOINT_INTERVAL_SEC

#         self.robots: dict[str, RobotSim] = {
#             r['name']: RobotSim(r['name'], r['x'], r['y'], r['yaw'])
#             for r in ROBOTS
#         }

#         self.state_pub = self.create_publisher(RobotState, 'robot_state', 10)

#         self.path_sub = self.create_subscription(
#             PathRequest,
#             'robot_path_requests',
#             self._path_request_cb,
#             10,
#         )

#         self.timer = self.create_timer(interval, self._step_and_publish)

#         self.get_logger().info(
#             f'Mock simulator hazir | robotlar: {list(self.robots.keys())}'
#         )
#         self.get_logger().info(
#             f'Waypoint interval: {interval}s | '
#             'robot_path_requests dinleniyor, robot_state yayimlaniyor.'
#         )

#     def _path_request_cb(self, msg: PathRequest) -> None:
#         robot = self.robots.get(msg.robot_name)
#         if robot is None:
#             self.get_logger().warn(
#                 f'Bilinmeyen robot adi: {msg.robot_name!r} — PathRequest yoksayildi.'
#             )
#             return
#         robot.set_path(msg)
#         self.get_logger().info(
#             f'{msg.robot_name}: yeni path alindi '
#             f'({len(msg.path)} nokta, task_id={msg.task_id})'
#         )

#     def _step_and_publish(self) -> None:
#         for robot in self.robots.values():
#             robot.step()
#             self._publish_state(robot)

#     def _publish_state(self, robot: RobotSim) -> None:
#         msg = RobotState()
#         msg.name = robot.name
#         msg.task_id = robot.task_id
#         msg.battery_percent = robot.battery

#         loc = Location()
#         loc.x = robot.x
#         loc.y = robot.y
#         loc.yaw = robot.yaw
#         loc.level_name = MAP_NAME
#         msg.location = loc

#         mode = RobotMode()
#         mode.mode = robot.mode
#         msg.mode = mode

#         self.state_pub.publish(msg)


# def main():
#     rclpy.init()
#     node = MockRobotSimulator()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()
