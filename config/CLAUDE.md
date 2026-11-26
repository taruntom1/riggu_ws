# config/

- **`zenoh/`** holds one profile per role:
  - `robot`: a router on the Pi listening on `tcp/[::]:7447`; local sessions connect to it.
  - `remote`: a laptop session connecting to `tcp/${RIGGU_HOST}:7447`.
  - `sim`: a local router only, never connected to the robot.
- **Pi address:** `RIGGU_HOST` defaults to `riggu.local`. WSL2 NAT may block mDNS, so keep the IP override working.
- **`udev/`:** stable device names. `/dev/rplidar` is the C1's CP210x adapter. DriveLink's own packaging provides `/dev/drivelink/<serial>`; don't duplicate it.
