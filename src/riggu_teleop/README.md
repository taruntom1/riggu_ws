# riggu_teleop

Manual override with a generic Bluetooth gamepad on the Pi. *Planned — Phase 5.*

| File | Purpose |
|---|---|
| `launch/teleop.launch.py` | `joy` + `teleop_twist_joy` + `twist_mux` + `stop_latch` |
| `config/joy_generic.yaml` | Axis and button indices, deadman, normal/turbo scales |
| `config/twist_mux.yaml` | Inputs: joy (100), nav (10). Lock: `/estop` (255) |
| `riggu_teleop/stop_latch.py` | Stop button toggles latched `/estop` (`std_msgs/Bool`) |

**Behaviour**
- The robot moves only while the deadman is held, and joystick input always overrides Nav2.
- Stop button: zero output until it is pressed again.
- Link loss: joy timeout → twist_mux timeout → `cmd_vel_timeout` → DriveLink watchdog (200 ms).

**Mapping a new pad:** run `ros2 topic echo /joy`, press each control, and write the indices into `joy_generic.yaml`.
