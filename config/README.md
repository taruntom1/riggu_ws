# config/

Machine-level configuration that isn't part of a ROS package.

| Path | Purpose |
|---|---|
| `tailscale/riggu-policy.hujson` | Policy for the dedicated riggu tailnet: `autogroup:member` + `autogroup:shared` → `tag:robot` on tcp 22, 7447, 47620 and ICMP; the robot gets no outbound rules |
| `udev/99-riggu.rules` | `/dev/rplidar` (C1, CP210x `10c4:ea60`). DriveLink (CH343 `1a86:55d3`) ships its own `/dev/drivelink/<serial>` |

## Zenoh profiles
No config files: every profile uses rmw_zenoh's defaults (router listens on `tcp/[::]:7447`; sessions connect to `tcp/localhost:7447`; multicast scouting off), plus `ZENOH_CONFIG_OVERRIDE` set by `source scripts/env.sh <profile>`.

| Profile | Where | Router | Override |
|---|---|---|---|
| `robot` | Pi | `rmw-zenohd.service` | none |
| `sim` | container | `rmw_zenohd` started by `env.sh` if port 7447 is free | none; isolated (no connect, no multicast) |
| `remote` | laptop | the Pi's | `mode="client"; connect/endpoints=["$RIGGU_ZENOH_ENDPOINT"]`. Default: `tcp/<HostName from ssh -G $RIGGU_HOST>:7447`; offline: `tcp/127.0.0.1:17447` |

## Robot access over Tailscale
- **Topology:** the Pi is the only node in a dedicated tailnet, tagged `tag:robot`, with no key expiry. It is shared to each person's own Tailscale account. Shared machines are quarantined: they can't initiate connections into the recipient's tailnet.
- **Join or re-join the Pi:** `scripts/setup_tailscale.sh` on the Pi. Open the printed URL as the riggu tailnet admin.
- **Onboard a collaborator:**
  1. In the riggu admin console: Machines → riggu → Share, to their account.
  2. Append their SSH public key to `~riggu/.ssh/authorized_keys` on the Pi.
  3. They add a `Host riggu.local` entry with `HostName` set to the Pi's tailnet name.
- **Offboard a collaborator:** revoke the share and remove their key.
