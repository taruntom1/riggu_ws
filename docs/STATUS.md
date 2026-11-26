# Status

Updated 2026-11-26. Next phase: **1**. Run a phase with `/implement-phase <n>`.

| # | Phase | State | Branch / PR |
|---|---|---|---|
| 0 | Host tooling and repo | done | `phase/0-host-tooling` · [PR #1](https://github.com/taruntom1/riggu_ws/pull/1) |
| 1 | Workspace, Docker, setup scripts, CI | in progress | `phase/1-workspace` |
| 2 | Robot description (URDF) | todo | — |
| 3 | Simulated drive | todo | — |
| 4 | Real base bring-up | todo | — |
| 5 | Joystick, twist_mux, safety | todo | — |
| 6 | EKF fusion | todo | — |
| 7 | Mapping and localisation | todo | — |
| 8 | Navigation | todo | — |
| 9 | Hardening and docs | todo | — |

States: `todo` · `in progress` · `review` (PR open) · `done` (merged) · `blocked`

## Needs from user
- **Phase 2:** robot measurements (list at the end of `docs/REQUIREMENTS.md`).

## Known environment facts
- The dev host is WSL2 Ubuntu 26.04 (`resolute`) with ROS **Lyrical** installed, so build in the Jazzy container.
- RTX 4050 + WSLg; systemd running; WSL `networkingMode=Mirrored` (LAN 172.16.32.0/21).
- Docker Engine 29.10.1 with the `nvidia` runtime: `docker run --gpus all` works. The user is in the `docker` group.
- `sudo` is passwordless (sudo-rs).
- GitHub SSH port 22 is blocked. `~/.ssh/config` routes `github.com` through `ssh.github.com:443`.
- Repo: https://github.com/taruntom1/riggu_ws (public). Git author is `taruntom1`, set in the repo-local config.
- Name history: the old repo named `riggu_ws` was renamed to `riggu_ws_old`, and the new repo took the name.
- **Pi network:** user `riggu` on the laptop's Windows mobile hotspot (192.168.137.126). WSL can't reach the hotspot subnet directly.
- **Primary access:** Tailscale. The Pi is the only machine in a dedicated riggu tailnet (tag `tag:robot`, no key expiry) and is shared into the user's personal tailnet.
  - The Windows Tailscale client is visible to WSL through mirrored mode.
  - `~/.ssh/config` maps `riggu`/`riggu.local` to the Pi's tailnet name.
  - The personal tailnet has a stale offline machine also named `riggu`, so always use the full name.
- **Fallback:** `ssh riggu-local` and `tcp/127.0.0.1:17447`, through the Windows portproxy.
- Pi: Ubuntu 24.04.4 aarch64, avahi active, key SSH works, passwordless sudo, Tailscale 1.104.7.

## Decisions / deviations
- Phase 0: the baseline commit (docs, `.gitignore`) went straight to `main`. The empty git repo in `/home/tarun` was removed, at the user's request.
- Phase 0: host installs are in `scripts/setup_host_tools.sh`, which runs on any Ubuntu.
- **Phase 0, Tailscale:** the Pi is in its own tailnet and shared to people, so it can't reach anyone's tailnet. Collaborators also get shares, not membership.
  - The policy allows only 22, 7447, 47620 and ICMP to the robot.
  - The hotspot port-forward is kept as the offline fallback.
  - The `remote` zenoh profile is client mode.
- **Phase 0, Pi sudo:** passwordless, as the user chose.
