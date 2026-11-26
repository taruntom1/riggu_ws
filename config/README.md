# config/

Machine-level configuration that isn't part of a ROS package. *Planned — Phase 1.*

| Path | Purpose |
|---|---|
| `zenoh/robot_router.json5` | Pi router, listening on `tcp/[::]:7447` |
| `zenoh/robot_session.json5` | Pi sessions, connecting to the local router |
| `zenoh/remote_session.json5` | Laptop client session, connecting to `${RIGGU_ZENOH_ENDPOINT:-tcp/127.0.0.1:17447}` (Windows port-forward to Pi:7447) |
| `zenoh/sim_*.json5` | Isolated local router and session for simulation |
| `udev/99-riggu.rules` | `/dev/rplidar` (C1, CP210x). DriveLink ships its own `/dev/drivelink/<serial>` |

Profiles are selected by `source scripts/env.sh <profile>`.
