# Status

Updated 2026-11-26. Next phase: **0**. Run a phase with `/implement-phase <n>`.

| # | Phase | State | Branch / PR |
|---|---|---|---|
| 0 | Host tooling and repo | review (SSH to Pi pending) | `phase/0-host-tooling` · PR pending |
| 1 | Workspace, Docker, setup scripts, CI | todo | — |
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
- **Phase 0 (Pi only):**
  - On the Pi: `sudo hostnamectl set-hostname riggu && sudo apt install -y avahi-daemon`.
  - From WSL: `ssh-copy-id <pi-user>@<pi-ip>` (key `~/.ssh/id_ed25519` already exists).
  - Tell the agent the Pi user and IP.
- **Phase 2:** robot measurements (list at the end of `docs/REQUIREMENTS.md`).

## Known environment facts
- The dev host is WSL2 Ubuntu 26.04 (`resolute`) with ROS **Lyrical** installed, so build in the Jazzy container.
- RTX 4050 + WSLg; systemd running; WSL `networkingMode=Mirrored` (LAN 172.16.32.0/21).
- Docker Engine 29.10.1 with the `nvidia` runtime: `docker run --gpus all` works. The user is in the `docker` group.
- `sudo` is passwordless (sudo-rs).
- GitHub SSH port 22 is blocked. `~/.ssh/config` routes `github.com` through `ssh.github.com:443`.
- Repo: https://github.com/taruntom1/riggu_ws (public). Git author is `taruntom1`, set in the repo-local config.
- Name history: the old repo named `riggu_ws` was renamed to `riggu_ws_old`, and the new repo took the name.

## Decisions / deviations
- Phase 0: the baseline commit (docs, `.gitignore`) went straight to `main`. The empty git repo in `/home/tarun` was removed, at the user's request.
- Phase 0: host installs are in `scripts/setup_host_tools.sh`, which runs on any Ubuntu.
