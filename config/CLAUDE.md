# config/

- **`zenoh/`** holds one profile per role:
  - `robot`: a router on the Pi listening on `tcp/[::]:7447`; local sessions connect to it.
  - `remote`: a laptop session in **client mode**, connecting to `${RIGGU_ZENOH_ENDPOINT:-tcp/127.0.0.1:17447}` (the Windows port-forward to the Pi).
  - `sim`: a local router only, never connected to the robot.
- **Pi address:** the Pi is on the Windows mobile hotspot, which WSL mirrored mode doesn't mirror. Don't bind local port 17447 or 2222 in WSL; those belong to the port-forward.
- **`udev/`:** stable device names. `/dev/rplidar` is the C1's CP210x adapter. DriveLink's own packaging provides `/dev/drivelink/<serial>`; don't duplicate it.
