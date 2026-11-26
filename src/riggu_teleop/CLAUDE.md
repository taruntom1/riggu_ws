# riggu_teleop

- **Safety semantics** (don't change without asking the user):
  - `twist_mux` priorities: `/estop` lock 255, joy 100, nav 10.
  - A deadman button is required for joystick motion.
  - The stop button latches `/estop` until it is pressed again.
- **Where joy runs:** `joy` runs **on the Pi** (generic Bluetooth gamepad), so the override survives Wi‑Fi loss.
- **Button mapping:** all button and axis indices live in `config/joy_generic.yaml`. No indices in code.
- **Loss of link** stops the robot through layered timeouts:
  - joy timeout
  - `twist_mux` timeout
  - `cmd_vel_timeout`
  - DriveLink watchdog (200 ms)
