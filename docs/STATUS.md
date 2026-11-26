# Status

Updated 2026-11-26. Next phase: **0**. Run a phase with `/implement-phase <n>`.

| # | Phase | State | Branch / PR |
|---|---|---|---|
| 0 | Host tooling and repo | in progress | `phase/0-host-tooling` |
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
- **Phase 0:**
  - Run `sudo scripts/setup_host_tools.sh`, then open a new shell.
  - Quit Docker Desktop, or disable its WSL integration for this distro.
  - Create the public GitHub repo `taruntom1/riggu_ws` (empty, no README) and run `gh auth login`.
  - Set up an SSH key from WSL to the Pi.
  - Set the Pi hostname to `riggu`.
- **Phase 2:** robot measurements (list at the end of `docs/REQUIREMENTS.md`).

## Known environment facts
- The dev host is WSL2 Ubuntu 26.04 (`resolute`) with ROS **Lyrical** installed, so build in the Jazzy container.
- An RTX 4050 and WSLg are available, and systemd is running.
- Before Phase 0, `docker` resolved to Docker Desktop's Windows CLI with a dead context.
- `sudo` needs a password, so agents can't install system packages; the user runs those steps.
- The repo is `riggu_ws`. Git author is `taruntom1`, set in the repo-local config.

## Decisions / deviations
- Phase 0: the baseline commit (docs, `.gitignore`) went straight to `main`. The empty git repo in `/home/tarun` was removed, at the user's request.
- Phase 0: host installs are in `scripts/setup_host_tools.sh`, which runs on any Ubuntu.
