# TIC-VLA-TEST branch: executed modified-copy configuration

The user requested publishing all current changes as branch `TIC-VLA-TEST`
on `minju5054/TIC-VLA` on 2026-10-14 (Asia/Seoul). This branch starts from
`e772ac62e6a9351df951176226180f32506fa6b6` and retains every preceding research
commit. The original `hardcase-probe` branch and its historical baseline remain
separate. The explicit branch request supersedes the workspace's usual
`hardcase-probe` working-branch preference for this publication.

The Git-free sibling `TIC-VLA-TEST` contains 36 source files whose only change
from the source repository was `RayTracedLighting` -> `RealTimePathTracing`.
Those existing changes are copied byte-for-byte here so this branch includes
the renderer configuration actually used in the recent experiments. All
other Python/shell source differences were checked; no additional API,
controller, model or geometry change was found in that comparison.
Newer reports, tests and saved-only analysis tools from this repository are
retained rather than replaced by the older sibling documentation.

This is the modified-copy runtime branch, not an unchanged official Isaac 5
reproduction. Renderer equivalence has not been established: RGB can differ,
which can affect predictions and navigation. The replacement also changes
explicit probe/calibration/viewer launch settings listed below, including the
`research/sim.py` launch and recorded runtime renderer. Existing provenance receipts may reject
these source hashes; prepare fresh receipts and IDs for future experiments.
No old receipt, raw output, simulator installation, dependency or model changed.

Two upstream source files are modified: `DynaNav/benchmark.py` and
`DynaNav/collect_data.py`. Reason: preserve the user's already executed
modified-copy renderer choice. Behavior difference: SimulationApp launch uses
RealTimePathTracing instead of RayTracedLighting. Research impact: visual
observations and reproducibility relative to the official renderer may differ.
Every other changed source file below is a research/probe addition.

## Exact renderer-change inventory

- `DynaNav/benchmark.py` (2 replacements)
- `DynaNav/collect_data.py` (1 replacement)
- `research/audit_hospital_lights.py` (1 replacement)
- `research/audit_nova_asset.py` (1 replacement)
- `research/calibrate_hospital_lights.py` (1 replacement)
- `research/calibrate_nova_bright.py` (1 replacement)
- `research/e16_c19_manual.py` (1 replacement)
- `research/e16_cart_preflight.py` (1 replacement)
- `research/e16_cart_run.py` (1 replacement)
- `research/e16_zero_repeatability.py` (1 replacement)
- `research/h05_stationary_run.py` (1 replacement)
- `research/hospital_late_handoff.py` (1 replacement)
- `research/hospital_lights_run.py` (1 replacement)
- `research/hospital_no_cart_reproduction.py` (1 replacement)
- `research/hospital_no_cart_speed.py` (1 replacement)
- `research/nova_bright_run.py` (1 replacement)
- `research/nova_dynamic.py` (1 replacement)
- `research/nova_hospital.py` (1 replacement)
- `research/paperlike_run.py` (1 replacement)
- `research/preflight_hospital.py` (1 replacement)
- `research/preflight_hospital_occlusion.py` (1 replacement)
- `research/preflight_nova_dynamic.py` (1 replacement)
- `research/preflight_static_detour.py` (1 replacement)
- `research/prepare_blind_corner.py` (1 replacement)
- `research/prepare_hospital_fixture_lights.py` (1 replacement)
- `research/qualify_obstacle_reveal.py` (1 replacement)
- `research/render_hospital_occlusion.py` (1 replacement)
- `research/search_reveal_window.py` (1 replacement)
- `research/sim.py` (2 replacements)
- `research/two_hz_preflight_isaac.py` (1 replacement)
- `research/two_hz_preflight_render.py` (1 replacement)
- `research/view_nova_dynamic.py` (1 replacement)
- `research/view_nova_occlusion.py` (1 replacement)
- `research/view_reveal_window_search.py` (1 replacement)
- `research/view_static_detour.py` (1 replacement)
- `scripts/probe_isaac6.py` (1 replacement)

## Local launcher and publication boundaries

The existing user-authored `run_test.sh` is included without content changes.
It uses the current working directory as the repository, sources the existing
local environment file, creates a fresh output directory, and runs episode 16
through the official benchmark child entry point. It is a host-specific smoke
launcher, not the C24 controlled-harness command; it was syntax-checked and
was not executed during publication. `.env.testing.local` remains untracked.

The branch includes experiment source code, tests and reports. Generated
outputs, RGB/GIF recordings, model weights, checkpoints, data and caches are
excluded. This task starts no simulation or inference. The exact publication
commit and remote verification are saved in the ignored publication audit.

Validation: 46 focused CPU tests PASS; all 36 copied sources parse and exactly
match the executed TEST bytes; `bash -n run_test.sh` PASS; current C24 config
exactly matches the completed TEST run. No new simulation or inference.


## Separate VS Code worktree (2026-10-14)

The formerly Git-free sibling is now a linked Git worktree. Open each folder
in its own VS Code window; its checked-out branch determines where commits go.
Activating a Python virtual environment does not select a Git branch.

| Open folder | Working branch |
|---|---|
| `/home/gpuadmin/Workspace/TIC-VLA` | `hardcase-probe` |
| `/home/gpuadmin/Workspace/TIC-VLA-TEST` | `TIC-VLA-TEST` |

```bash
code --new-window /home/gpuadmin/Workspace/TIC-VLA-TEST
git -C /home/gpuadmin/Workspace/TIC-VLA-TEST branch --show-current
git -C /home/gpuadmin/Workspace/TIC-VLA-TEST status --short --branch
```

In the TEST window, Source Control commits staged files to `TIC-VLA-TEST`;
the lower-left branch indicator should show that name. Its existing upstream
is `origin/TIC-VLA-TEST`. A local commit and a remote push remain separate
operations. This setup makes no push and does not transfer or rewrite existing
`hardcase-probe` commits.

The worktrees share Git history/remotes, but have separate files, HEADs and
staging indexes. The TEST `.git` is a pointer file into the original repository's
worktree metadata; keep the original repository in place.

The attachment preserved existing source bytes. It filled only 29 missing
tracked paths from the existing TEST branch, including `.gitignore`, and
appended missing committed history to the older work log after verifying that
the old log was an exact prefix. The TEST branch's `AGENTS.md` now selects
`TIC-VLA-TEST`; the original workspace rule is unchanged. Existing untracked
scripts/tools remain available for separate review and commits.

Ignored, folder-local `.vscode/settings.json` uses the following settings so
this window discovers its own repository without automatically opening other
registered worktrees or repositories belonging to outside editor tabs:

```json
{
  "git.autoRepositoryDetection": "subFolders",
  "git.openRepositoryInParentFolders": "never",
  "git.detectWorktrees": false
}
```

Existing ignores cover `.venv`, outputs, model weights, checkpoints and caches;
`*.bak` additionally excludes the copy's local source backups. No virtual
environment, simulator, inference dependency or experiment output was changed.
This is a Git/editor setup validation, not runtime or navigation validation.
