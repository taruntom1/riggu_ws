# riggu_teleop

- **Safety semantics** (don't change without asking the user; `test/test_teleop_config.py` checks them):
  - `twist_mux` priorities: `/estop` lock 255, joy 100, nav 10.
  - A deadman button is required for joystick motion.
  - The stop button latches `/estop` until it is pressed again. `/estop` is a heartbeat (lock `timeout` > 0): a dead `stop_latch` must engage the lock.
  - Arming from the pad: the arm combo includes the deadman and is held for `arm_hold`; disarm is one press. `arm_switch` is the only node that activates `diff_drive_controller` automatically; bring-up never does. Its pre-arm `recover` call restores the hardware disarmed; it must never arm without the held combo.
- **Where joy runs:** `joy` runs **on the Pi** (generic Bluetooth gamepad), so the override survives Wi‑Fi loss.
- **Button mapping:** all button and axis indices live in `config/joy_generic.yaml`. No indices in code or tests (the mock launch test reads the yaml).
- **Wiring:** `riggu_bringup/bringup.launch.py` includes `teleop.launch.py` and sets `cmd_vel_out`/`nav_in` for the collision monitor; `joy`/`arm` default to on only for `mode:=real`.
- **Velocity messages** are `TwistStamped` (`publish_stamped_twist`, twist_mux `use_stamped`). Stamps matter: `diff_drive_controller` times out on the stamp, so publishers must use the controller's clock (sim time in sim).
- **Python nodes:** installed as scripts (`install(PROGRAMS ... RENAME)`); the button logic is a plain class tested without ROS.
- **Loss of link** stops the robot through layered timeouts:
  - joy timeout
  - `twist_mux` timeout
  - `cmd_vel_timeout`
  - DriveLink watchdog (200 ms)
