#!/usr/bin/env python3
"""
Mock robot simulator for warehouse robots.

Gazebo olmadan RMF'in tam hareket dongusunu simule eder:
  - fleet_manager'dan gelen PathRequest dinler
  - Robotu hedef noktaya lineer interpolasyonla hareket ettirir
  - Her adimda robot_state yayimlar
  - Hedefe ulasinca MODE_IDLE bildirir

Kullanim:
  cd ~/Desktop/graduation_project && source install/setup.bash
  python3 mock_robot_state_publisher.py
"""

import math
import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import RobotState, Location, RobotMode, PathRequest

ROBOTS = [
    {'name': 'warehouseRobot1', 'x':  2.222, 'y': -8.0,    'yaw': 0.0},
    {'name': 'warehouseRobot2', 'x': 18.889, 'y': -8.0,    'yaw': 3.14},
    {'name': 'warehouseRobot3', 'x':  7.778, 'y': -15.556, 'yaw': 1.57},
]

MAP_NAME = 'L1'
SPEED = 1.0        # m/s — lineer interpolasyon hizi
UPDATE_HZ = 10.0   # robot_state yayim frekansi


class RobotSim:
    """Tek bir robotun konum ve gorev durumunu tutar."""

    def __init__(self, name, x, y, yaw):
        self.name = name
        self.x = x
        self.y = y
        self.yaw = yaw
        self.task_id = '0'
        self.battery = 95.0
        self.mode = RobotMode.MODE_IDLE

        # Aktif yol: hedef noktalarin listesi
        self.path = []
        self.current_target_idx = 0

    def update(self, dt: float):
        """dt saniye gec; path varsa bir adim ilerle."""
        if not self.path or self.current_target_idx >= len(self.path):
            self.mode = RobotMode.MODE_IDLE
            return

        self.mode = RobotMode.MODE_MOVING
        target = self.path[self.current_target_idx]
        tx, ty = target.x, target.y

        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        step = SPEED * dt

        if dist <= step:
            # Bu noktaya ulastik
            self.x = tx
            self.y = ty
            self.yaw = target.yaw
            self.current_target_idx += 1
            if self.current_target_idx >= len(self.path):
                self.path = []
                self.current_target_idx = 0
                self.mode = RobotMode.MODE_IDLE
        else:
            # Hedefe dogru adim at
            ratio = step / dist
            self.x += dx * ratio
            self.y += dy * ratio
            self.yaw = math.atan2(dy, dx)

    def set_path(self, path_request: PathRequest):
        """fleet_manager'dan gelen PathRequest'i isle."""
        # Ilk eleman mevcut konum, geri kalani hedefler
        self.path = list(path_request.path[1:]) if len(path_request.path) > 1 else []
        self.current_target_idx = 0
        self.task_id = path_request.task_id
        self.mode = RobotMode.MODE_MOVING


class MockRobotSimulator(Node):

    def __init__(self):
        super().__init__('mock_robot_simulator')

        self.robots: dict[str, RobotSim] = {
            r['name']: RobotSim(r['name'], r['x'], r['y'], r['yaw'])
            for r in ROBOTS
        }

        self.state_pub = self.create_publisher(RobotState, 'robot_state', 10)

        self.path_sub = self.create_subscription(
            PathRequest,
            'robot_path_requests',
            self.path_request_cb,
            10,
        )

        dt = 1.0 / UPDATE_HZ
        self.dt = dt
        self.timer = self.create_timer(dt, self.update_and_publish)

        self.get_logger().info(
            f'Mock simulator hazir. Robotlar: {list(self.robots.keys())}'
        )
        self.get_logger().info(
            'robot_path_requests dinleniyor, robot_state yayimlaniyor.'
        )

    def path_request_cb(self, msg: PathRequest):
        robot = self.robots.get(msg.robot_name)
        if robot is None:
            return
        robot.set_path(msg)
        self.get_logger().info(
            f'{msg.robot_name} icin yeni path alindi: '
            f'{len(msg.path)} nokta, task_id={msg.task_id}'
        )

    def update_and_publish(self):
        for robot in self.robots.values():
            robot.update(self.dt)
            self._publish_state(robot)

    def _publish_state(self, robot: RobotSim):
        msg = RobotState()
        msg.name = robot.name
        msg.task_id = robot.task_id
        msg.battery_percent = robot.battery

        loc = Location()
        loc.x = robot.x
        loc.y = robot.y
        loc.yaw = robot.yaw
        loc.level_name = MAP_NAME
        msg.location = loc

        mode = RobotMode()
        mode.mode = robot.mode
        msg.mode = mode

        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = MockRobotSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
