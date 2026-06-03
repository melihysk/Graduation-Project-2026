"""
İDKR Standalone launch — CP-destekli trafik yönetimi.

Simülasyon (Gazebo + RViz) ayrı terminal'de çalışır:
  ros2 launch rmf_demos_gz warehouse_starter_standalone.launch.xml

Bu launch dosyası:
  1. İDKR Standalone Traffic Manager (CP-aware görev atama + Res1 + rota planlama)
  2. Metric Logger (deney metrikleri, traffic_mode=idkr)
  3. Task Dispatcher (senaryo görevlerini İDKR'ye gönderir)

Kullanım:
  ros2 launch idkr_controller idkr_standalone.launch.py \\
      scenario_id:=normal run_id:=1 expected_tasks:=10 \\
      scenario_file:=$(ros2 pkg prefix metric_logger)/share/metric_logger/config/scenarios/scenario_normal.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_graph_default = PathJoinSubstitution([
        FindPackageShare("rmf_demos_maps"),
        "maps", "warehouse_starter", "nav_graphs", "0.yaml",
    ])

    idkr_node = Node(
        package="idkr_controller",
        executable="idkr_standalone_manager",
        name="idkr_traffic_manager",
        output="screen",
        parameters=[{
            "nav_graph_file": LaunchConfiguration("nav_graph_file"),
            "retry_interval_sec": LaunchConfiguration("retry_interval_sec"),
            "deadlock_check_interval_sec": LaunchConfiguration("deadlock_check_interval_sec"),
            "arrival_tolerance": LaunchConfiguration("arrival_tolerance"),
            "fleet_name": "warehouseRobot",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    metric_logger_node = Node(
        package="metric_logger",
        executable="metric_logger_node",
        name="metric_logger",
        output="screen",
        parameters=[{
            "traffic_mode": "idkr",
            "scenario_id": LaunchConfiguration("scenario_id"),
            "run_id": LaunchConfiguration("run_id"),
            "expected_tasks": LaunchConfiguration("expected_tasks"),
            "expected_robots": LaunchConfiguration("expected_robots"),
            "auto_finish_timeout_sec": LaunchConfiguration("auto_finish_timeout_sec"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    task_dispatcher_node = Node(
        package="idkr_controller",
        executable="task_dispatcher_idkr",
        name="task_dispatcher_idkr",
        output="screen",
        parameters=[{
            "scenario_file": LaunchConfiguration("scenario_file"),
            "delay_between_tasks_sec": LaunchConfiguration("delay_between_tasks_sec"),
            "max_pending": LaunchConfiguration("max_pending"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription([
        # IDKR parameters
        DeclareLaunchArgument(
            "nav_graph_file",
            default_value=nav_graph_default,
            description="RMF nav graph YAML (metre coords)",
        ),
        DeclareLaunchArgument(
            "retry_interval_sec", default_value="0.5",
        ),
        DeclareLaunchArgument(
            "deadlock_check_interval_sec", default_value="2.0",
        ),
        DeclareLaunchArgument(
            "arrival_tolerance", default_value="1.5",
        ),

        # Scenario parameters
        DeclareLaunchArgument("scenario_id", default_value="normal"),
        DeclareLaunchArgument("run_id", default_value="1"),
        DeclareLaunchArgument("expected_tasks", default_value="0"),
        DeclareLaunchArgument("expected_robots", default_value="4"),
        DeclareLaunchArgument("scenario_file", default_value=""),
        DeclareLaunchArgument("delay_between_tasks_sec", default_value="2.0"),
        DeclareLaunchArgument("max_pending", default_value="6"),
        DeclareLaunchArgument("auto_finish_timeout_sec", default_value="120.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        RegisterEventHandler(
            OnProcessExit(
                target_action=metric_logger_node,
                on_exit=[Shutdown()],
            )
        ),

        idkr_node,
        metric_logger_node,
        task_dispatcher_node,
    ])
