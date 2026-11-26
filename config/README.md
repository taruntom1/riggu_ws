# config/

Machine-level configuration that isn't part of a ROS package. *Planned — Phase 1.*

| Path | Purpose |
|---|---|
| `zenoh/robot_router.json5` | Pi router, listening on `tcp/[::]:7447` |
| `zenoh/robot_session.json5` | Pi sessions, connecting to the local router |
| `zenoh/remote_session.json5` | Laptop client session, connecting to `${RIGGU_ZENOH_ENDPOINT}` (default: tailnet name of `riggu.local`, port 7447; offline: `tcp/127.0.0.1:17447`) |
| `tailscale/riggu-policy.hujson` | **Available.** Policy for the dedicated riggu tailnet: `autogroup:member` + `autogroup:shared` → `tag:robot` on tcp 22, 7447, 47620 and ICMP; the robot gets no outbound rules |
| `zenoh/sim_*.json5` | Isolated local router and session for simulation |
| `udev/99-riggu.rules` | `/dev/rplidar` (C1, CP210x). DriveLink ships its own `/dev/drivelink/<serial>` |

Profiles are selected by `source scripts/env.sh <profile>`.

## Robot access over Tailscale
- **Topology:** the Pi is the only node in a dedicated tailnet, tagged `tag:robot`, with no key expiry. It is shared to each person's own Tailscale account. Shared machines are quarantined: they can't initiate connections into the recipient's tailnet.
- **Join or re-join the Pi:** `scripts/setup_tailscale.sh` on the Pi. Open the printed URL as the riggu tailnet admin.
- **Onboard a collaborator:**
  1. In the riggu admin console: Machines → riggu → Share, to their account.
  2. Append their SSH public key to `~riggu/.ssh/authorized_keys` on the Pi.
  3. They add a `Host riggu.local` entry with `HostName` set to the Pi's tailnet name.
- **Offboard a collaborator:** revoke the share and remove their key.
