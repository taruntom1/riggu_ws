# riggu_gazebo

Gazebo Harmonic simulation of riggu. *Planned — Phase 3.*

| File | Purpose |
|---|---|
| `launch/sim.launch.py` | gz sim + spawn + `ros_gz_bridge` + `riggu_bringup` (`mode:=sim`). Args: `world`, `headless`, `x/y/yaw` |
| `worlds/house.sdf` | Default small indoor world (rooms, doorways) |
| `config/bridge.yaml` | Bridges `/clock` and `/scan`. The IMU and joints go through `gz_ros2_control` |

**Simulated sensors**

| Sensor | Model |
|---|---|
| Lidar | `gpu_lidar` tuned to the C1: 10 Hz, 0.72°, 0.05–12 m, Gaussian noise |
| IMU | gz IMU with noise and bias |

```bash
ros2 launch riggu_gazebo sim.launch.py               # GUI
ros2 launch riggu_gazebo sim.launch.py headless:=true # CI
```
