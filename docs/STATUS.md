# Status

Updated 2026-11-26. Next phase: **0**. Run a phase with `/implement-phase <n>`.

| # | Phase | State | Branch / PR |
|---|---|---|---|
| 0 | Host tooling and repo | todo | — |
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
  - Create the public GitHub repo `taruntom1/riggu_ws` and run `gh auth login`.
  - Set up an SSH key from WSL to the Pi.
  - Set the Pi hostname to `riggu`.
- **Phase 2:** robot measurements (list at the end of `docs/REQUIREMENTS.md`).

## Known environment facts
- The dev host is WSL2 Ubuntu 26.04 with ROS **Lyrical** installed, so build in the Jazzy container.
- An RTX 4050 and WSLg are available.
- The Docker CLI points at a dead Docker Desktop context, and `gh` is not installed (both fixed in Phase 0).
- The local branch is `master` with no commits and no remote yet.

## Decisions / deviations
- none yet
