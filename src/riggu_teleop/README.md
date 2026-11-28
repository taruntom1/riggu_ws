# riggu_teleop

Manual override with a generic Bluetooth gamepad on the Pi: pad teleop, `twist_mux`, latching stop, pad arming. Included by `riggu_bringup/bringup.launch.py`; don't launch it on its own next to bringup.

| File | Purpose |
|---|---|
| `launch/teleop.launch.py` | `twist_mux` + `stop_latch`; `joy:=true`: `joy_node` + `teleop_twist_joy`; `arm:=true`: `arm_switch`. Args: `cmd_vel_out` (`/cmd_vel_mux`), `nav_in` (`/cmd_vel_nav`), `joy_config`, `mux_config`, `use_sim_time` |
| `config/joy_generic.yaml` | Every button/axis index, deadman, normal/turbo scales, stop button, arm combo |
| `config/twist_mux.yaml` | Inputs: joy (100, 0.25 s), nav (10, 0.5 s). Lock: `/estop` (255, 0.5 s heartbeat) |
| `riggu_teleop/stop_latch.py` | Stop button toggles `/estop` (`std_msgs/Bool`, 10 Hz heartbeat). `~/set` (`std_srvs/SetBool`) latches/releases without the pad |
| `riggu_teleop/arm_switch.py` | Hold `arm_buttons` for `arm_hold` s → activate `diff_drive_controller` (**arms**); `disarm_button` → deactivate |
| `test/` | Button logic (latch, arm combo); safety semantics in the configs |

**Default pad mapping** (riggu's USB pad: PlayStation shape, enumerates as an Xbox 360 pad, `xpad`)

| Control | Action |
|---|---|
| LB / L1 (held) | Deadman: left stick drives (0.2 m/s, 1.0 rad/s) |
| RB / R1 (held) | Turbo deadman (0.5 m/s, 2.0 rad/s) |
| LB + Start, 1 s | Arm (real robot) |
| Back / Select | Disarm |
| B | Stop latch on/off |

**Behaviour**
- Motion only while a deadman is held; the joystick always beats `/cmd_vel_nav`.
- Stop latch: twist_mux output is zero (joy and nav) until pressed again. If `stop_latch` dies, the missing heartbeat engages the lock.
- Boot comes up disarmed. Arming needs the combo held; the launch files never arm.
- Link loss: pad off → `joy_node` stops → `teleop_twist_joy` stops → twist_mux joy timeout (0.25 s) → `cmd_vel_timeout` (0.5 s) brakes → DriveLink watchdog (200 ms) if the Pi side dies.

```bash
ros2 service call /stop_latch/set std_srvs/srv/SetBool '{data: true}'   # latch from the laptop
ros2 topic echo /estop --once
```

## Bluetooth pad on the Pi

A USB pad needs nothing: plug it in, `joy_node` opens `/dev/input/js0` (user in `input`). For Bluetooth:

```bash
bluetoothctl
  power on
  agent on
  default-agent
  scan on            # pad in pairing mode
  pair XX:XX:XX:XX:XX:XX
  trust XX:XX:XX:XX:XX:XX   # reconnects on its own after a reboot
  connect XX:XX:XX:XX:XX:XX
  scan off
ls /dev/input/js*    # appears once connected
```

Cheap clones that pair but never create an input device may need `ClassicBondedOnly=false` in `/etc/bluetooth/input.conf`, then `sudo systemctl restart bluetooth`.

## Mapping a new pad

1. Keep the robot disarmed (press the disarm button, or check `ros2 control list_controllers`). `riggu-base` already runs `joy_node`; without it, `ros2 run joy joy_node`.
2. `ros2 topic echo /joy` and press each control: note the index that changes in `buttons[]` / `axes[]`. Stick up and stick left must read `+1`; otherwise negate the scale.
3. Write the indices into `config/joy_generic.yaml` (`teleop_twist_joy_node`, `stop_latch`, `arm_switch`), rebuild, `sudo systemctl restart riggu-base`.
