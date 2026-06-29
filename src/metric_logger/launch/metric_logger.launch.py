"""
Metrik toplama launch dosyası.

warehouse_starter.launch.xml ile birlikte AYRI çalıştırılır.
RMF (ve gerekirse mock sim) zaten çalışıyor olmalı.

Kullanım — Gazebo ile:
  # Terminal 1:
  ros2 launch rmf_demos warehouse_starter.launch.xml

  # Terminal 2:
  ros2 launch metric_logger metric_logger.launch.py \
      traffic_mode:=rmf scenario_id:=normal run_id:=1 \
      scenario_file:=$(ros2 pkg prefix metric_logger)/share/metric_logger/config/scenarios/scenario_normal.yaml

Kullanım — Gazebo kapalı hızlı test:
  # Terminal 1a:
  ros2 launch rmf_demos warehouse_starter.launch.xml

  # Örnek1:
  ros2 launch metric_logger metric_logger.launch.py \
      traffic_mode:=rmf scenario_id:=normal run_id:=1 \
      scenario_file:=$(ros2 pkg prefix metric_logger)/share/metric_logger/config/scenarios/scenario_normal.yaml

    # Örnek2:
  ros2 launch metric_logger metric_logger.launch.py \
      traffic_mode:=rmf scenario_id:=bottleneck run_id:=1 \
      expected_tasks:=12 scenario_file:=$(ros2 pkg prefix metric_logger)/share/metric_logger/config/scenarios/scenario_bottleneck.yaml 

   # Örnek3:
  ros2 launch metric_logger metric_logger.launch.py \
      traffic_mode:=rmf scenario_id:=high_density run_id:=1 \
      expected_tasks:=16 scenario_file:=$(ros2 pkg prefix metric_logger)/share/metric_logger/config/scenarios/scenario_high_density.yaml \


"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    metric_logger_node = Node(
        package='metric_logger',
        executable='metric_logger_node',
        name='metric_logger',
        output='screen',
        parameters=[{
            'traffic_mode': LaunchConfiguration('traffic_mode'),
            'scenario_id': LaunchConfiguration('scenario_id'),
            'run_id': LaunchConfiguration('run_id'),
            'expected_tasks': LaunchConfiguration('expected_tasks'),
            'output_dir': LaunchConfiguration('output_dir'),
            'auto_finish_timeout_sec': LaunchConfiguration('auto_finish_timeout_sec'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'traffic_mode', default_value='rmf',
            description='Traffic management mode: rmf | dkr | idkr'
        ),
        DeclareLaunchArgument(
            'scenario_id', default_value='normal',
            description='Scenario identifier: normal | bottleneck | high_density'
        ),
        DeclareLaunchArgument(
            'run_id', default_value='1',
            description='Run number for this experiment repetition'
        ),
        DeclareLaunchArgument(
            'expected_tasks', default_value='0',
            description='Number of expected tasks (0 = use timeout-based finish)'
        ),
        DeclareLaunchArgument(
            'scenario_file', default_value='',
            description='Path to scenario YAML file with task definitions'
        ),
        DeclareLaunchArgument(
            'auto_finish_timeout_sec', default_value='60.0',
            description='Seconds of inactivity after last task completion before auto-finish'
        ),
        DeclareLaunchArgument(
            'output_dir', default_value='',
            description='Output directory for results (default: <workspace>/results)'
        ),
        DeclareLaunchArgument(
            'delay_between_tasks_sec', default_value='2.0',
            description='Delay between dispatching consecutive tasks'
        ),
        DeclareLaunchArgument(
            'max_pending', default_value='6',
            description='Max tasks in RMF queue before waiting for completions (0=unlimited)'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time from /clock topic'
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=metric_logger_node,
                on_exit=[Shutdown()],
            )
        ),
        metric_logger_node,

        # Task Dispatcher Node (sends predefined tasks)
        Node(
            package='metric_logger',
            executable='task_dispatcher_node',
            name='task_dispatcher_scenario',
            output='screen',
            parameters=[{
                'scenario_file': LaunchConfiguration('scenario_file'),
                'delay_between_tasks_sec': LaunchConfiguration(
                    'delay_between_tasks_sec'
                ),
                'max_pending': LaunchConfiguration('max_pending'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
