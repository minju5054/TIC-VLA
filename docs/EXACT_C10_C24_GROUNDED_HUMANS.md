# Exact original C10/C24, two grounded stationary people

The requested single scene was executed once with two people at the exact
original no-added-obstacle C10/C24 XY coordinates. **Placement/grounding and
runtime PASS; navigation FAIL.** The robot passed C10 without reported contact,
then collided with C24 and never entered the strict 2 m goal region.

The user explicitly selected one combined scene and the earlier unobstructed
reference, superseding the previous request for separate human/cart scenes.
The reference is `outputs/nova-e16-hospital-lights-baseline-20261010-01`, whose
configuration contains no added actor. It is not the later H05 chart.

| Person | Exact original world X [m] | Exact original world Y [m] | Measured result |
| --- | ---: | ---: | --- |
| C10 | 6.824887275695801 | 7.096357822418213 | Passed at 6.316667 simulation s; 0 contact reports |
| C24 | -1.7567275762557983 | 10.988419532775879 | Not passed; first contact at 12.116667 simulation s |

Both authored and measured XY errors are **0 m**, with **0 m measured motion**
and **0 quaternion component change** over 1,519 robot samples / 3,038 human
samples. The original C10/C24 labels identify reference observations, not times
or positions of the new run's C10/C24 observations. There is no horizontal
offset, additional legacy H05 person, or added cart. Original Hospital props
remain present.

## Grounding

Actual model-free USD/skinning inspection found that the old helper's broad
authored root bounding box started at Z=-0.1574196815 m. Aligning that bound
with the floor lifted the visible workboot soles to Z=0.1574196581 m.
The additive actor lowers visual Z by 0.1574196581 m. Visual X/Y and root XY
remain unchanged. The original radius0.25 m, cylinder-height1.2 m capsule is
preserved, with its centre lowered to Z0.85 m so its bottom reaches the floor.
The original helper, asset and historical records are unchanged.

Skinned workboot minima and actual original-Hospital PhysX floor rays agree
within **0.016971 mm (C10)** and **0.015957 mm (C24)**. Four frozen-time
feet/body diagnostic views were visually reviewed. The original rest/T-pose is
retained. Its arms extend beyond the capsule; this capsule is not a detailed
full-body collision mesh.

The first model-free preflight failed because a floor ray originating inside
the grounded capsule hit that same capsule at distance zero. Only the floor
query was corrected to retain original `/Root/` Hospital hits using
`raycast_all`; no position or acceptance tolerance was relaxed. All 30 failed
attempt files and the old source bytes are preserved. A new `-p2` preflight
passed all 35 placement/geometry/render checks before model preparation.
Preflight performs settling but no navigation or inference; diagnostic cameras
render at frozen tick0/time0 without physics advance. Local bypass probes found
at least one geometric alternative for each person; they did not command it.

## Measured outcome and failure behavior

Run: `TIC-VLA-TEST/outputs/exact-c10-c24-run-20261013T144528Z-e4a298df`.
Exactly one 48-prediction navigation run; no retry or extended window.
Runtime completed with finite native30x2 arrays and actual motion over
25.300001 simulation seconds. Goal minimum distance was **10.639718 m**,
final distance **10.700763 m**; goal<2 m was never reached.

C24 has 1,263 contact reports across 14 collider-pair contact spans, with
maximum contact impulse norm61.447812 N·s and minimum separation-0.000485743 m.
These are persistent reports, not 1,263 independent impacts. All non-floor
contact targets are `/World/ExactHumanC24/Collider`, not Hospital structures.
C10's minimum sampled centre distance is0.820124 m; conservative enclosing
disks overlap slightly, but no actual C10 contact was reported. Disk overlap
does not establish physical penetration.

Saved RGB shows C10 ahead at observation1 and C24 by observation12. At
observations20–22, before the first contact, the issued v stays1.5 m/s and
w decreases0.048572→0.017010→0.011769 rad/s. Their observation times are
10.800001/11.300001/11.800001 s; application times are
10.900001/11.400001/11.900001 s. First contact follows at12.116667 s.
Observation24 applies0.235085 m/s and1.2 rad/s at12.900001 s, after contact.
Thus sufficient lateral clearance or slowing was not achieved before impact.
This is measured command/path behavior, not proof of which model component
caused it. The run does not isolate visual perception, stochastic output,
reasoning age, or the controller as a unique cause.

## Visibility logging limitation and corrected saved-only review

The new individual label matcher produced **96 empty per-person masks** even
though the combined `occluded_human` mask is nonempty for47/48 observations and
people are visible in the original RGB. The gate checked authored USD labels,
not successful rendered per-person label matching. The exact annotator-label
failure mechanism was not established. Raw ID rasters/maps were not archived,
so no per-person pixel mask or exact first visibility time is reconstructed.

The initial frozen analyzer's C10 `passed_and_stationary=false` incorrectly
depends on those empty visibility masks. Its null first-visible fields do not
mean the people were absent. All initial derived files are retained. The
supplement `tools/exact_two_humans_review.py` recalculates passage/contact/goal
from measured records independently, marks individual visibility UNUSABLE,
and retains the overall scenario FAIL. This is a post-run reporting correction,
not a new simulation or a changed decision rule used to turn failure into success.
The frozen runtime bundle remains the executed source; future visibility-based
experiments require repairing and validating per-person render-label logging.

## Evidence and reproduction identity

- Original workspace source base: `63416ec94a1c8cd5215f740e8c893aca81e1a290`,
  branch `hardcase-probe`; upstream `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`.
- TEST has no Git. Preparation freezes relative source/reference content hashes,
  configuration, external runtimes/weights and prerequisite preflight files.
  Original dirty status, empty tracked-diff SHA and added source SHA manifests
  are in `outputs/exact-two-humans-preparation-20261013T144528Z-e4a298df/`.
  `floor-query-fix.json` records the one preflight source correction.
- Simulation: Isaac6.0.1-rc.7, Python3.12.13, NumPy2.3.1,
  `RealTimePathTracing`. Separate model-only Isaac5 Python3.11.13,
  torch2.7.0+cu128, torchvision0.22.0+cu128, NumPy1.26.0, transformers4.57.6.
  No second SimulationApp, package install, driver or runtime changes.
- Base weight SHA256:
  `a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`;
  checkpoint SHA256:
  `376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`.
- Original controller/continuous loop preserved: 60 Hz physics, target2 Hz
  inference, one outstanding IPC future, OLD retention, FRESH observation
  anchor, 48 predictions. Native outputs are forward/left displacements with
  no yaw. No reconciliation, emergency controller or model training was added.
- Original Hospital origin, XYZ metres, +Z up; world yawCCW from+X. Robot
  quaternionwxyz, measured human PhysX quaternionxyzw. USD default-pose skinning
  produces skeleton-space vertices then uses the skeleton-to-world row matrix.
  Passage projects world robot-minus-human XY along the original adjacent
  observation tangent beyond the sum of enclosing radii. Simulation time is
  distinct from wall time, absolute Isaac time, inference start/ready, reasoning
  generation/poll, and command application; their existing records are retained.
- Historical path overlay is descriptive: historical renderer/wrapper
  equivalence is unestablished. This is modified Isaac6 simulation evidence,
  not unchanged Isaac5 reproduction, a DynaNav IRA port, real-world validation,
  or general avoidance qualification.

Executed model command (already consumed; do not reuse the run directory):

```bash
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/exact-c10-c24-run-20261013T144528Z-e4a298df-setup/run.sh
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/exact-c10-c24-run-20261013T144528Z-e4a298df-setup/analyze.sh
```

Saved-only supplemental command, run from the original workspace:

```bash
python3 -B tools/exact_two_humans_review.py \
  --run-dir /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/exact-c10-c24-run-20261013T144528Z-e4a298df \
  --output-dir outputs/exact-two-humans-preparation-20261013T144528Z-e4a298df/saved-review
```

This output also already exists; use a fresh output directory for another
saved-only review. It does not launch Isaac or inference. Full preparation and
launch commands are recorded in the audit and each setup's process-result.

Navigation figures/GIF are in the run's `derived/exact-two-humans/`;
the reviewed six-frame sheet and corrected summary are in the audit's
`saved-review/`. The GIF contains48 actual saved observation images held by
simulation timestamps, not intermediate rendered video. Grounded feet views
are in `exact-c10-c24-preflight-20261013T144528Z-e4a298df-p2/diagnostics/`.

Validation: **17 focused CPU tests PASS**. Independent saved-artifact audit
verifies539 raw files,164 frozen source files,379 reference/prerequisite files,
48 finite native arrays,48 RGB images, measured observation/tick agreement,
two exact stationary people at every tick, grounded feet/capsules and the
48-frame25.3 s GIF. Individual mask recount verifies the logging failure, not
visibility success.166 previous source files,1,727 prior output files and
30 failed-preflight files are unchanged. No official upstream source edits;
the user's untracked `run_test.sh` remains untouched. No push.
