# riggu_gazebo

Gazebo Harmonic simulation of riggu. Dev-only: `setup_local.sh` and `deploy.sh` skip it on the Pi.

| File | Purpose |
|---|---|
| `launch/sim.launch.py` | gz sim + spawn (`ros_gz_sim create` from `/robot_description`) + `ros_gz_bridge` + `riggu_bringup` (`mode:=sim`) + RViz. Args: `world`, `headless`, `rviz`, `use_sim_time` (`true`), `x`/`y`/`yaw` (spawn, default `2.5 -1.5 0`) |
| `worlds/house.sdf` | Default world: 10×8 m house, 4 rooms (living SE, kitchen NE, bedroom NW, office SW), 1 m doorways, static box/cylinder furniture. Primitives only, no Fuel downloads |
| `config/bridge.yaml` | Bridges `/clock` and `/scan`. The IMU and joints go through `gz_ros2_control` |
| `rviz/sim.rviz` | Fixed frame `odom`: model, TF, `/scan`, `/odom` |

**Simulated sensors** (defined in `riggu_description/urdf/sensors.xacro`, values in `robot.yaml`)

| Sensor | Model |
|---|---|
| Lidar | `gpu_lidar`, C1: 10 Hz, 500 beams (0.72°), 0.05–12 m, Gaussian noise σ 1 cm |
| IMU | gz IMU, 100 Hz (read by the 50 Hz controller manager); gyro/accel Gaussian noise and bias from BNO055 figures |

```bash
ros2 launch riggu_gazebo sim.launch.py                 # GUI + RViz
ros2 launch riggu_gazebo sim.launch.py headless:=true  # server only, headless rendering
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p frame_id:=base_footprint -p use_sim_time:=true -r cmd_vel:=/cmd_vel_joy
```

The full cmd_vel chain runs in sim (twist_mux, stop latch, collision monitor on the sim `/scan`); keyboard teleop stands in for the pad on `/cmd_vel_joy`.

Wheels collide as spheres in sim (see `riggu_description`), so in-place turns match `wheel.separation`.
