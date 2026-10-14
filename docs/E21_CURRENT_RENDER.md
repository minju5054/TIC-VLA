# E21 current-render RGB and right-turn verification

**Right turn PASS; RGB quality FAIL.** One actual 48-prediction no-added-obstacle
run at the user-selected **1.5 m/s** exited the room and turned right without
recorded non-floor contact. The unchanged quantitative RGB test failed in
C1–C17. This is a diagnostic under failed image quality, not a qualified
combined baseline or an unchanged Isaac 5 reproduction.

Run: `e21-current-render-20261014T044932Z-68ab22` in `/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/`.
Original workspace audit: `outputs/e21-preparation-20261014T044932Z-68ab22`.

## Conditions and the preflight distinction

Official Hospital episode_21, no added people, wheelchairs, carts or occluders.
Existing Hospital props remain. Start `(23.3, -0.9, 0.01)` and yaw 180 degrees;
the established Nova wrapper applies spawn Z=0.11 m and then settles on the
floor. Goal `(20,24,0.01)`. Exact unchanged instruction:

> Exit the room and turn right to enter the hallway. Continue straight ahead and to the end of the hallway.

Direct controller: max_v=1.5 m/s, max_w=1.2 rad/s, lookahead=1.0 m,
angular_gain=0.8, filter_gain=0.35, epsilon=0.001. 60 Hz physics, target 2 Hz
observations, one pending model request, OLD target retention, real-time pacing.
48 native `(30,2)` outputs, body-forward/left metres anchored at their own
observation pose; no native yaw or SE(2) claim. No control, exposure, light,
geometry or model tuning after the result. Config differs from historical E21
only in three refreshed runner-source SHA fields (see source-validation.json).

First strict model-free preflight `e21-current-render-20261014T044932Z-68ab22-preflight` passed 9
operational checks but rejected brightness: mean 228.389687, saturation
48.459105%. It made **zero model calls**; its failure.json uses the inherited
`IMPLEMENTATION_FAILURE` label, but the actual exception is an intentional
`E21_PREFLIGHT_FAILED: ['brightness']`, not an undefined symbol/runtime bug.
The image is legible but visibly washed out. This result and its frozen source
bundle `tools/e21_current_render/` remain intact.

To complete the user's separate request to measure the actual turn under the
current render, an explicitly labelled additive diagnostic bundle
`tools/e21_rgb_diagnostic/` was created. Its separate model-free preflight
`e21-current-render-20261014T044932Z-68ab22-operational-preflight` passed all **9 operational wheel/camera checks** and
recorded `rgb_quality_pass=false`. It gates model calls on unchanged source,
robot/wheels, supported spawn, camera geometry/clearance, 1920×1080 RGB, lighting
identity and completed render with zero physics advance. Brightness failure is
an independent outcome, never relabelled PASS or tuned away. Exactly **one**
real-model diagnostic followed. No human preflight, dynamic execution or
navigation qualification is inferred from this continuation.

## Results

| Measure | Historical E21 | Current run |
|---|---:|---:|
| Intended right-turn region | C11 at 6.700000 s | C11 at 6.333334 s |
| Following observation | C12 | C12 |
| Non-floor contact through following application | 0 | 0, through 6.950000 s |
| RGB failures in required C1–C12 window | 12/12 | 12/12 |
| RGB failures across all 48 observations | 16/48 | 17/48 |
| C1 mean luminance (maximum 215) | 228.279536 | 232.186253 |
| C1 saturated fraction (maximum 5%) | 48.3463% | 52.0362% |
| Complete-run non-floor contact spans | 2 | 0 |
| Closest goal XY distance | 1.031097 m | 1.022769 m |

Existing right-turn criterion is X<=20 m, 0.5<=Y<=5 m, yaw 45–135 degrees,
plus the following observation, with no relevant contact through its command
application. C11 measured position is `(17.493481, 0.743369)` m,
yaw 124.396395 degrees; yaw change from measured start is
-55.603030 degrees (right/clockwise).
C12 application is at 6.950000 s.
There are **zero non-floor contact reports**, zero camera-proximity flags and
zero near-black flags across the complete new run. This is evidence from
measured PhysX/robot logs, not authored waypoints or static source inspection.

First strict <2 m goal entry occurs at **20.950001 s**;
minimum distance is **1.022769 m**. Fixed-budget execution continues
past the goal, ends at `(18.880772,28.916250)`, and final
distance is **5.042042 m**. Goal stopping was not validated.
Total measured interval is 25.333335 s. First observation gap
is 1.833333429 s (bootstrap), subsequent gaps are approximately 0.5 simulation
seconds. Wall latency, observation time, inference start/ready and command
application remain distinct in raw requests; no exact wall-clock 2 Hz claim.

C1–C17 exceed the frozen mean-luminance ceiling; C1–C5 also exceed saturation
limit (see brightness.json for every threshold). C18–C48 pass. RGB is legible
for this turn but retains overexposure, especially the white room floor/walls.
The unchanged `bright_pass` rule remains authoritative; visual readability
does not override it. C1 model input differs slightly from initial preflight
because it is a later completed render; both images are archived separately.

## Runtime and provenance

Historical renderer **RayTracedLighting** is established by the exact archived
metadata hash matching `research/hospital_lights_run.py` at historical SHA
`390e14b408bdd4fe5c7ae055b151c660b999ff64`; see historical-renderer-proof.json.
Current **RealTimePathTracing** uses the same 153-fixture Hospital profile,
front Hawk left camera, resolution and source asset geometry. A renderer-name
change alone did not make this scene pass RGB criteria. This is not an isolated
renderer-causality experiment: renderer state/warm-up, timing and physical paths
also differ, and current closed-loop C# images are not pose-matched to history.

Simulation: Isaac `6.0.1-rc.7+release.42383.32955d8d.gl`, Python 3.12.13,
NumPy 2.3.1. Separate model-only Isaac 5 Python: 3.11.13, torch 2.7.0+cu128,
torchvision 0.22.0+cu128, NumPy 1.26.0, transformers 4.57.6;
strict checkpoint load, no second SimulationApp. RTX 5060 Ti 16 GB,
driver 595.99.02. No package/driver/runtime installation changed.

Source branch `hardcase-probe`; starting local SHA `167e8c7f9213361317591a60682e2ebbdb63704c`;
upstream SHA `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. New/untracked bundle identities,
dirty status and tracked-diff SHA are stored in both preparation provenance
and execution metadata, alongside 165 current-copy source hashes. Released
base weight SHA `a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`;
checkpoint SHA `376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`.
All exact prepare/launch/analyze commands are in audit `commands.json`; actual
launch argv and console are preserved in each setup folder. New run and
analysis destinations reject existing paths.

Coordinates use the original Hospital world origin, metres, +Z up; yaw is CCW
from world +X, quaternions wxyz. No world registration/trajectory alignment is
applied. Robot body axes are forward/left/up. Asset camera transforms retain
USD row-vector local-to-world convention (camera optical forward is -Z).
Capture explicitly waits for render completion and verifies no simulation-time
advance; request observations were matched to measured physics ticks.

## Artifacts and verification

- Actual RGB playback: `/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/e21-current-render-20261014T044932Z-68ab22/derived/e21-current-render/rgb-observations.gif`
- RGB contact sheet, per-frame statistics, brightness-comparison.png and
  summary.json: same derived directory.
- Reviewed full-path and turn figure: `/home/gpuadmin/Workspace/TIC-VLA/outputs/e21-preparation-20261014T044932Z-68ab22/trajectory-reviewed.png`.
  Original analyzer plot remains preserved; reviewed figure only improves
  legend placement and overlapping initial labels.
- Unmodified raw PNG, native requests, contacts and robot_state.csv: run folder.

**32 focused existing CPU tests PASS** (blind corner, Hospital lights,
continuous handoff). Separate source-global checks PASS. Recomputed all 48
controller commands and all 48 archived RGB statistics; max numeric statistics
difference 2.56e-13. Verified 1,521 consecutive robot rows and
observation-pose/tick agreement, 293 raw hashes, 165 source hashes,
365 reference hashes and 22 external files. **6,686 protected prior files
remain byte-identical.** Both new original-workspace bundles match the executed
TEST files. GIF has 48 frames, 25.330 s encoded duration (centisecond-quantized;
raw time span 25.333335 s). Actual run has no Python traceback or NULL-material
warnings. Startup deprecation warnings remain in console.

Only additive tools and documentation are changed. No upstream DynaNav,
model, controller or existing research runtime source is changed in this task.
No experiment output, model, cache or recording is committed. This single
simulation establishes this observed turn, not general avoidance performance,
OLD/FRESH inconsistency, reconciliation performance or real-world behavior.
