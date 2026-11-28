# riggu_navigation

Mapping, localisation, and (Phase 8) Nav2 for riggu. Runs on the Pi next to `riggu_bringup`, which owns `odom → base_footprint` (EKF); these launches add `map → odom`.

| File | Purpose |
|---|---|
| `launch/slam.launch.py` | slam_toolbox online async under `lifecycle_manager_slam` → `/map`, `map → odom`. Args: `use_sim_time`, `slam_config`, `robot_config`, `rviz` (`false`) |
| `launch/localization.launch.py` | map_server + AMCL under `lifecycle_manager_localization` → `/map`, `/amcl_pose`, `/particle_cloud`, `map → odom`. Args: `map` (**required**, `.yaml`), `use_sim_time`, `params_file`, `robot_config`, `rviz` (`false`) |
| `config/slam_toolbox.yaml` | Jazzy online-async defaults; 5 cm, `base_footprint`, map update 2 s, interactive mode off |
| `config/nav2_params.yaml` | `map_server`, `amcl` (DifferentialMotionModel, likelihood field, 60 beams, 500–2000 particles). Phase 8 adds the rest |
| `maps/house.{yaml,pgm}` | Sim reference map of `riggu_gazebo/worlds/house.sdf`. Map origin = sim spawn (world `2.5 -1.5`, yaw 0). Real maps live in `~/riggu_maps` on the Pi |
| `rviz/mapping.rviz`, `rviz/navigation.rviz` | Fixed frame `map`: map, scan, model, TF; navigation adds AMCL particles/pose, 2D Pose Estimate, Goal Pose |
| `test/test_navigation_config.py` | pytest: frames, resolution, motion model, launch laser range, reference map |
| `test/test_slam_mock.py`, `test/test_localization_mock.py` | launch_test on `mode:=mock` with a synthetic `/scan`: slam_toolbox maps a box room; AMCL converges on the reference map from a 36 cm / 10° guess (no-motion updates). `ROS_DOMAIN_ID` 88/89 |

- **Laser range** (`min/max_laser_range`, `laser_min/max_range`) comes from `riggu_description/config/robot.yaml` (`lidar.range_*`, C1 0.05–12 m), set by the launches.
- **Initial pose:** AMCL starts without one (`set_initial_pose: false`). Give it with RViz *2D Pose Estimate* (or `/initialpose`), then drive a metre or so: AMCL updates every 0.25 m / 0.2 rad.
- **Saving:** `scripts/save_map.sh <name>` writes `<name>.yaml/.pgm` (map_saver) and `<name>.posegraph/.data` (slam_toolbox, to continue mapping later) to `${RIGGU_MAP_DIR:-~/riggu_maps}`. Run it on the machine running slam_toolbox.
- Nav2 (Phase 8) publishes `/cmd_vel_nav`, which reaches the controller only through `twist_mux` and `collision_monitor` (see `riggu_bringup`).

## Sim

```bash
ros2 launch riggu_gazebo sim.launch.py rviz:=false
ros2 launch riggu_navigation slam.launch.py use_sim_time:=true rviz:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p frame_id:=base_footprint -p use_sim_time:=true -r cmd_vel:=/cmd_vel_joy
scripts/save_map.sh --dir /tmp house

ros2 launch riggu_navigation localization.launch.py use_sim_time:=true rviz:=true \
  map:=$(ros2 pkg prefix riggu_navigation)/share/riggu_navigation/maps/house.yaml
```

## Real robot

`riggu-base` must be running (lidar, EKF, pad). Drive with the pad.

```bash
# Pi (source scripts/env.sh robot)
ros2 launch riggu_navigation slam.launch.py
scripts/save_map.sh <name>                      # -> ~/riggu_maps/<name>.*
ros2 launch riggu_navigation localization.launch.py map:=$HOME/riggu_maps/<name>.yaml

# Laptop, in the container (source scripts/env.sh remote)
rviz2 -d "$(ros2 pkg prefix riggu_navigation)/share/riggu_navigation/rviz/mapping.rviz"
rviz2 -d "$(ros2 pkg prefix riggu_navigation)/share/riggu_navigation/rviz/navigation.rviz"
```
