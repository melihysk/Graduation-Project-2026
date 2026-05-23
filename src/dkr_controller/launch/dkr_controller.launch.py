"""
DKR Controller launch dosyası.

RMF altyapısı (fleet adapter, task dispatcher) çalışırken DKR trafik
yönetimini başlatır. rmf_traffic_schedule ve rmf_traffic_blockade
başlatılMAZ — trafik kontrolü tamamen DKR'ye bırakılır.

Kullanım:
  # Terminal 1 — RMF (traffic schedule OLMADAN):
  #   (Aşağıdaki özel launch veya manual node start gerekir)

  # Terminal 2 — DKR + Metric Logger:
  ros2 launch dkr_controller dkr_controller.launch.py \
      scenario_id:=normal run_id:=1 \
      scenario_file:=$(ros2 pkg prefix metric_logger)/share/metric_logger/config/scenarios/scenario_normal.yaml

Not: Bu launch dosyası fleet_manager'ın topic remap'ini yapmaz —
fleet_manager zaten warehouse_starter_dkr.launch.xml içinde remap
ile başlatılır (robot_path_requests → robot_path_requests_raw).
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

    dkr_node = Node(
        package="dkr_controller",
        executable="dkr_traffic_manager",
        name="dkr_traffic_manager",
        output="screen",
        parameters=[{
            "nav_graph_file": LaunchConfiguration("nav_graph_file"),
            "lookahead": LaunchConfiguration("lookahead"),
            "retry_interval_sec": LaunchConfiguration("retry_interval_sec"),
            "deadlock_check_interval_sec": LaunchConfiguration("deadlock_check_interval_sec"),
            "arrival_tolerance": LaunchConfiguration("arrival_tolerance"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    metric_logger_node = Node(
        package="metric_logger",
        executable="metric_logger_node",
        name="metric_logger",
        output="screen",
        parameters=[{
            "traffic_mode": "dkr",
            "scenario_id": LaunchConfiguration("scenario_id"),
            "run_id": LaunchConfiguration("run_id"),
            "expected_tasks": LaunchConfiguration("expected_tasks"),
            "auto_finish_timeout_sec": LaunchConfiguration("auto_finish_timeout_sec"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    task_dispatcher_node = Node(
        package="metric_logger",
        executable="task_dispatcher_node",
        name="task_dispatcher_scenario",
        output="screen",
        parameters=[{
            "scenario_file": LaunchConfiguration("scenario_file"),
            "delay_between_tasks_sec": LaunchConfiguration("delay_between_tasks_sec"),
            "max_pending": LaunchConfiguration("max_pending"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription([
        # DKR parameters
        DeclareLaunchArgument(
            "nav_graph_file",
            default_value=nav_graph_default,
            description="RMF nav graph YAML (metre coords, same as fleet adapter)",
        ),
        DeclareLaunchArgument(
            "lookahead", default_value="1",
            description="Number of segments to reserve ahead (1 = less deadlock risk)",
        ),
        DeclareLaunchArgument(
            "retry_interval_sec", default_value="0.5",
            description="Retry interval for waiting robots (seconds)",
        ),
        DeclareLaunchArgument(
            "deadlock_check_interval_sec", default_value="2.0",
            description="Deadlock detection check interval (seconds)",
        ),
        DeclareLaunchArgument(
            "arrival_tolerance", default_value="1.5",
            description="Distance threshold for segment arrival detection (metres/pixels)",
        ),

        # Scenario parameters
        DeclareLaunchArgument(
            "scenario_id", default_value="normal",
            description="Scenario identifier: normal | bottleneck | high_density",
        ),
        DeclareLaunchArgument(
            "run_id", default_value="1",
            description="Run number for this experiment repetition",
        ),
        DeclareLaunchArgument(
            "expected_tasks", default_value="0",
            description="Number of expected tasks (0 = timeout-based finish)",
        ),
        DeclareLaunchArgument(
            "scenario_file", default_value="",
            description="Path to scenario YAML file with task definitions",
        ),
        DeclareLaunchArgument(
            "delay_between_tasks_sec", default_value="2.0",
            description="Delay between dispatching consecutive tasks",
        ),
        DeclareLaunchArgument(
            "max_pending", default_value="6",
            description="Max tasks in queue before waiting for completions",
        ),
        DeclareLaunchArgument(
            "auto_finish_timeout_sec", default_value="60.0",
            description="Seconds of inactivity before auto-finish",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use simulation time from /clock topic",
        ),

        # Shutdown all when metric_logger finishes
        RegisterEventHandler(
            OnProcessExit(
                target_action=metric_logger_node,
                on_exit=[Shutdown()],
            )
        ),

        dkr_node,
        metric_logger_node,
        task_dispatcher_node,
    ])
