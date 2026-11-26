---
name: implement-phase
description: Implement one phase of docs/IMPLEMENTATION_PLAN.md end to end — gate checks, branch, build, verify the "Done when" checks, update docs/STATUS.md, open a PR. Use when the user runs /implement-phase <n> or asks to start, continue or resume phase <n>.
argument-hint: "<phase-number> [notes]"
---

# Implement a riggu phase

Arguments: `$ARGUMENTS`. The first token is the phase number; anything after it is extra notes from the user and overrides the plan where they conflict. With no number, read `docs/STATUS.md`, pick the first phase that is not `done`, and confirm it with the user before starting.

## 1. Orient
- Read `docs/STATUS.md`, then from `docs/IMPLEMENTATION_PLAN.md` the target phase plus "Working agreements" and "Cross-cutting conventions". Read `docs/REQUIREMENTS.md` only for the parts the phase touches.
- Read `README.md` and `CLAUDE.md` of every module the phase touches.
- Check `git status`, `git branch -a`, and (once `gh` exists) `gh pr list --state all`.

## 2. Gate — stop and tell the user if any fails
- **Earlier phases:** each must be `done`. If the previous phase is `review`, check `gh pr view <pr> --json state`. If it is merged, set it to `done` in STATUS; otherwise stop.
- **User inputs:** STATUS "Needs from user" items for this phase must be resolved. Ask for missing ones (Phase 2 measurements, for example), and never invent physical values.
- **Robot reachable (Phase 4 onwards, real-robot tasks):** `ssh -o ConnectTimeout=5 "${RIGGU_HOST:-riggu.local}" true`.
- **Ambiguity:** if anything is unclear or the plan conflicts with what you find, ask with AskUserQuestion: 1–4 questions, options with your recommendation first. Don't ask about what the plan already decides.

## 3. Branch
- If `phase/<n>-*` exists, resume it: read its log and STATUS notes, then carry on.
- Otherwise `git fetch` and branch `phase/<n>-<slug>` from `origin/main`. Phase 0 is the exception: it creates `main` and the remote.
- Set the phase to `in progress` in STATUS.

## 4. Implement
- Turn the phase's task list into your task list and work through it.
- Follow the root and module `CLAUDE.md` rules. Prefer upstream packages and config over custom code.
- Commit in logical chunks, with messages `phase <n>: <what>`.

## 5. Verify every "Done when" item
- **Build and test** in the Jazzy container (once Phase 1 exists):
  `scripts/docker_run.sh bash -lc 'colcon build --symlink-install && colcon test && colcon test-result --verbose'`
  Never use the host's `/opt/ros/lyrical`.
- **Sim checks:** run headless where possible and read topics and TF rather than relying on the GUI.
- **Real robot,** over SSH:
  - Before anything that can arm DriveLink or move the robot, say what will happen and get the user's go-ahead. The first motion of any new config runs with the wheels off the ground.
  - You can't see the robot. Ask the user to observe and report.
- Record each check as **pass**, **fail**, or **needs user**. Don't call a phase finished while any check fails.

## 6. Docs
- If you changed a module's interfaces, files, or rules, update its `README.md` (expert audience, concise) and `CLAUDE.md`.
- If you deviated from the plan, edit the plan and add one line under STATUS "Decisions / deviations".

## 7. Status and PR
- Push, then `gh pr create`:
  - Title: `Phase <n>: <name>`.
  - Body: short summary, the "Done when" checklist with results, and any actions the user must take.
- Set STATUS to `review` with the PR link, or to `blocked` with the reason. Keep STATUS short: edit rows in place, no narrative.
- Never merge, and never push to `main`. A phase becomes `done` only after the user merges it; the next run records that.

## 8. Report
Tell the user what was done, what is unverified, and exactly what they need to do next (review or merge the PR, observe the robot, provide inputs).
