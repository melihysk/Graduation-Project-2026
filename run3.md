# DKR Standalone

## Build

```bash
cd ~/Desktop/graduation_project
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dkr_controller rmf_demos_gz metric_logger
source install/setup.bash
```

## Terminal 1 — Gazebo + RViz

```bash
cd ~/Desktop/graduation_project && source install/setup.bash
ros2 launch rmf_demos_gz warehouse_starter_standalone.launch.xml
```

## Terminal 2 — DKR

### normal (10 görev)

```bash
cd ~/Desktop/graduation_project && source install/setup.bash
ros2 launch dkr_controller dkr_standalone.launch.py \
    scenario_id:=normal \
    run_id:=1 \
    expected_tasks:=10 \
    scenario_file:=$(ros2 pkg prefix metric_logger)/share/metric_logger/config/scenarios/scenario_normal.yaml
```

### bottleneck (12 görev)

```bash
cd ~/Desktop/graduation_project && source install/setup.bash
ros2 launch dkr_controller dkr_standalone.launch.py \
    scenario_id:=bottleneck \
    run_id:=1 \
    expected_tasks:=12 \
    scenario_file:=$(ros2 pkg prefix metric_logger)/share/metric_logger/config/scenarios/scenario_bottleneck.yaml
```

### high_density (16 görev)

```bash
cd ~/Desktop/graduation_project && source install/setup.bash
ros2 launch dkr_controller dkr_standalone.launch.py \
    scenario_id:=high_density \
    run_id:=1 \
    expected_tasks:=16 \
    scenario_file:=$(ros2 pkg prefix metric_logger)/share/metric_logger/config/scenarios/scenario_high_density.yaml
```

