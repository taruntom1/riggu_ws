# config/

- **Zenoh profiles** have no files here. They are rmw_zenoh's defaults plus `ZENOH_CONFIG_OVERRIDE`, set in `scripts/env.sh`:
  - `robot`: the router on the Pi (`rmw-zenohd.service`, `tcp/[::]:7447`); local sessions connect to it.
  - `remote`: a laptop session in **client mode**, connecting to `${RIGGU_ZENOH_ENDPOINT}`. The default is derived from `ssh -G ${RIGGU_HOST:-riggu.local}` (the tailnet name), port 7447. Offline fallback: `tcp/127.0.0.1:17447`.
  - `sim`: a local router only, never connected to the robot.
  - Only add a json5 file if an override string can't express the change; then copy the installed default and edit it.
- **Pi address:** primary is Tailscale. The fallback uses the Windows hotspot port-forward, so don't bind local port 17447 or 2222 in WSL.
- **`tailscale/riggu-policy.hujson`:** the robot tailnet policy. Only people → `tag:robot` on 22, 7447, 47620 and ICMP; nothing from the robot. Changing it is a security decision, so ask the user.
- **`udev/`:** stable device names. `/dev/rplidar` is the C1's CP210x adapter. DriveLink's own packaging provides `/dev/drivelink/<serial>`; don't duplicate it.
