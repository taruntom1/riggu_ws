# src/ — ROS 2 packages

| Package | Owns |
|---|---|
| `riggu_description` | URDF/xacro, ros2_control hardware selection |
| `riggu_bringup` | Top-level launch, controllers, EKF, collision monitor, systemd units |
| `riggu_gazebo` | Worlds, sim launch, gz bridges |
| `riggu_navigation` | slam_toolbox, AMCL, Nav2, maps, waypoints |
| `riggu_teleop` | joy, teleop_twist_joy, twist_mux, stop latch |

## Conventions
- **Build types:** config and launch packages use `ament_cmake`. Python nodes are installed through `ament_cmake_python` or as scripts.
- **Launch files:** written in Python (`*.launch.py`). Every launch declares `use_sim_time` and passes it down.
- **Parameters:** live in `config/*.yaml`. Launch files hold only arguments and wiring.
- **Dependencies:** declare every runtime dep in `package.xml` with rosdep keys. CI and the Pi install from it.
- **Tests:** each package has `ament_lint_auto`. Keep `colcon test` green.
- **Prefer upstream:** use upstream Jazzy packages over custom nodes. Custom code stays small and in Python.
