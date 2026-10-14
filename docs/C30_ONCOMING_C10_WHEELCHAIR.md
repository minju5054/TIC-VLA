# C30 oncoming human and C10 stationary wheelchair

2026-10-14 (Asia/Seoul). **Runtime PASS; FAIL_OR_UNQUALIFIED_ENCOUNTER.** One real model trial,
90 predictions. User explicitly accepted translation of the existing human in
its authored rest pose; no leg gait animation. No navigation retry or tuning.

The intended corner encounter **was reproduced**, but avoidance **failed**.
C10 was passed without contact at 10.383334 s. The moving human came within
2 m in the corner at 14.950001 s; first human contact was 15.866667 s, at robot
XY [3.064622, 11.230109] m and human XY [2.656090, 11.072854] m.
The robot later returned toward the lobby and never entered the goal region.
The analyzer's combined `FAIL_OR_UNQUALIFIED_ENCOUNTER` status is a qualified
encounter with failed avoidance in this particular run.

The human was visible at observation C19 (10.366667 s), clearly visible by C20
(10.866667 s). C27-C29 retained **0.8 m/s** with w=0.350264/0.250737/0.260500
rad/s. C30's 0.029531 m/s, 1.071319 rad/s command was applied at 15.966667 s,
**0.100 s after first contact**. This establishes late slowing in the saved
command sequence, not the model's internal reason for missing avoidance.
The first-contact RGB sheet shows the approaching person directly ahead.
Human-specific reports include positive impulse from 15.866667 s, maximum
64.450678 N s, and minimum separation -0.001515 m. These are physical contact
records, not solely zero-impulse proximity callbacks.

## Fixed scenario

Robot maximum forward command **0.8 m/s**, maximum angular command 1.2 rad/s,
90 predictions, unchanged controller/model/robot/camera/lighting and instruction:

> Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.

One original Hospital wheelchair remains stationary at exact original C10
[6.824887275695801, 7.096357822418213, 0] m. The C24 wheelchair is absent.
One person starts at exact original C30
[-6.1780195236206055, 10.137574195861816, 0] m, then travels at **1 m/s** along
the reversed saved original C30-to-C12 observation positions. Original reference:
`nova-e16-hospital-lights-baseline-20261010-01` with no added obstacles.
No other added people/cart/wheelchairs. Existing Hospital props remain.

The schedule was frozen before inference: start **6.928525 sim s**,
planned encounter **15.933334 sim s** near
[2.72192645072937, 11.062365531921387, 0.0] m. This aligns the human's original C18
with observation 30 of the previous successful 0.8 m/s static trial
`two-wheelchairs-v08-20261014T015420Z-a56068`. These are two different observation
sequences. Timing is derived from prior records, never current predictions or
robot position feedback. The human stops at original C12 after 13.502088 m.
Planned timing is not evidence that an encounter actually occurred.

Operational encounter criterion, fixed before the trial: robot XY inside
[1, 8] to [6.5, 12.5] m, robot-human centre separation <=2 m, and measured human
translation speed >0.1 m/s. Full raw poses/RGB remain available for inspection.

## Measured result

| Measurement | Result |
| --- | ---: |
| Corner encounter observed | True |
| First qualifying encounter, sim s | 14.950001 |
| Closest robot-human centre distance, m | 0.375470 |
| Closest approach, sim s | 25.383335 |
| Human first visible / clear RGB request | 19 / 20 |
| Human contact reports | 1575 |
| Human passed, sim s | 17.933334 |
| Human measured path length, m | 13.502017 |
| Human maximum target error, m | 0.000000524 |
| C10 wheelchair contact reports | 0 |
| C10 passed, sim s | 10.383334 |
| C10 wheelchair displacement, m | 0.000000 |
| Goal first <2 m entry, sim s | Not observed |
| Closest / final goal distance, m | 15.323622 / 23.104632 |
| Non-floor spans before goal entry/end / all | 85 / 85 |
| First non-floor report, sim s | 15.866667 |

At closest approach, robot XY [6.345253944396973, 8.820277214050293] m and human XY
[6.219180107116699, 8.466606140136719] m. Centre distances are not surface clearance.
Human passage means robot is beyond the conservative combined radius behind
the human's current travel direction; C10 passage uses its fixed original
reference-tangent plane. The moving human has a kinematic capsule of radius
0.25 m and cylinder height 1.2 m. Its rest-pose arms are visual geometry, not
separate limb colliders; this limits physical-human equivalence.
The 17.933334 s passage-plane crossing follows collision and is not successful
avoidance. Closest separation while the person was moving was 0.380649 m at
16.250001 s. The table's smaller global minimum at 25.383335 s occurred after
the person had stopped at the scripted endpoint; it is not the first encounter.

There are 2887 persistent-included non-floor reports;
maximum impulse 64.450678 N s, minimum separation
-0.002331 m. Contact targets:
- `/Root/Geo_M1_SideWall24/Geo_M1_SideWall/Geo_M1_SideWall/Section0`
- `/Root/Geo_M1_SideWall24/Geo_M1_SideWall/Geo_M1_SideWall/Section1`
- `/Root/Geo_M1_TrimStrateSide_448/Geo_M1_TrimStrateSide/Geo_M1_TrimStrateSide`
- `/Root/Geo_M1_TrimStrate_440/Geo_M1_TrimStrate/Geo_M1_TrimStrate`
- `/Root/Geo_M1_WallClose_1032/Geo_M1_WallClose/Geo_M1_WallClose/Section0`
- `/Root/Geo_M1_WallClose_1032/Geo_M1_WallClose/Geo_M1_WallClose/Section1`
- `/Root/Geo_M_CornerTrim_8/Geo_M_CornerTrim/Geo_M_CornerTrim`
- `/World/OncomingHumanC30/Collider`

Runtime success proves finite predictions and measured motion, not navigation
success. The PASS condition requires the measured encounter, contact-free
passage of both actors and goal-region entry. It does not establish stopping:
the unchanged fixed 90-call loop continues after entering the goal region.
A single scripted trial does not establish general avoidance competence,
model intent, a causal speed effect, or an OLD/FRESH inconsistency.

## Gates and implementation

New additive `tools/c30_oncoming_c10_wheelchair/`: exact C10 geometry reuses the
frozen 0.8 m/s wheelchair bundle; human visual/kinematic authoring reuses
`research/human_actor.py`, with measured boot grounding and heading along the
path. Root local -Y is the character's visible forward direction, confirmed by
body views; root yaw = path heading + pi/2. Human path configuration, 60 Hz
updates, PhysX snapshots and per-tick target/actual positions are new. RGB/pose
capture completes rendering without a physics step; saved segmentation IDs and
masks are diagnostics, not model inputs. Runner preserves the one-future OLD
retention/FRESH observation anchoring loop and separate model process.
No upstream source or prior scenario was changed.

Pre-model scene gate **29 PASS**; model-free full-motion gate
**5 PASS**, 1232 physics steps, max target error
0.000000524 m. Grounded body/feet/wheel views inspected.
The waiting robot receives zero commands; natural drift
0.021702 m is retained and reported.
The actual trial uses a fresh simulator at the original spawn.

Two model-free preflight failures are preserved with exact source snapshots:

1. A NumPy boolean could not be serialized. The rotated foot check also split
   boots in world X, inadvertently grouping heel/toe vertices. Fixed explicit
   bool conversion and partitioning by human-root local X; sole minima remain
   world Z. Added a rotated-foot regression test. Geometry/motion unchanged.
2. All 29 scene checks and human motion/end/target checks passed, but an added
   2 cm waiting-robot drift guard rejected 2.1702 cm of natural uncommanded drift.
   No non-floor contact or robot command occurred. Replaced that arbitrary
   requirement with all-tick zero-command and human/waiting-robot separation
   checks; drift remains visible in evidence. No physics/model changes.

Fresh `-preflight-p3` passed before the only actual model run. No model calls
were made during failed preflights. 20 focused CPU tests PASS; initial unit-test
fixture correction and preflight revisions are recorded in the audit.

## Provenance and evidence

Branch `TIC-VLA-TEST`, starting local SHA
`8cec935525eb28e859840e4c77e7f9c8e28e028c`; upstream SHA
`9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Dirty status, diff identity, initial
and final source hashes, exact failed-source snapshots and commands are in the
audit. Final focused local commit is in `git-finalization.json`; outputs are
uncommitted and no push was requested for this trial.

Modified Isaac6.0.1-rc.7 simulator (Python3.12.13, NumPy2.3.1,
RealTimePathTracing), separate Isaac5.0.0-rc.45 model-only Python3.11.13,
torch2.7.0+cu128, torchvision0.22.0+cu128, NumPy1.26.0, transformers4.57.6.
Preserved checkpoint SHA256
`376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`, base weights
`a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`.
No second SimulationApp for inference; no package, host, driver, asset or model
installation/change. Modified simulator evidence, not unchanged Isaac5
reproduction, IRA human/crowd behavior, or real-world evidence.

Original Hospital world origin; XYZ metres, +Z up, yaw CCW from +X. USD
row-vector transforms map root/skeleton local points to world. Human PhysX
quaternion xyzw; robot quaternion wxyz. Native 30x2 forward/left offsets contain
no yaw. Observation, inference start/ready, reasoning generation/ready/poll and
command application clocks remain separate; simulation time is not wall time.

Audit: 1093 raw, 170 source, 500 reference hashes;
1441 protected prior files unchanged; 2783
paired robot/human/wheelchair rows, 90 native arrays and independently recomputed
commands, 180 reconstructed semantic masks, 90 decoded 1920x1080 RGB images,
90-frame GIF (46.36 s). Applied forward command cap .8 m/s;
max measured body-forward speed 0.803153 m/s.
Console has 2647 PhysX NULL-material warnings and 0
Python tracebacks. All raw outputs remain separate from derived plots.
The saved-only quaternion audit initially compared a recomputed post-step path
heading at an exact polyline vertex with the authored pre-step target heading.
At tick 956, floating-point simulation-step timing chose the preceding segment
for one tick (0.113566 rad heading difference); translation differed by less
than 1 nanometre. The final audit checks PhysX quaternion against the recorded
authored heading and allows an adjacent segment only at a verified vertex.
Measured yaw matches the authored target within 2.44e-7 rad. Raw/source files
were not edited and the model trial was not rerun for this audit correction.

## Commands and artifacts

Historical IDs below are already used; use fresh IDs for reruns.

```bash
cd /home/gpuadmin/Workspace/TIC-VLA-TEST
python3 -B tools/c30_oncoming_c10_wheelchair/runner.py prepare --run-id c30-oncoming-c10-wheelchair-20261014T031747Z-a5e1bf-preflight-p3 --session /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c30-oncoming-c10-wheelchair-20261014T031747Z-a5e1bf-preflight-p3-setup --preflight
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c30-oncoming-c10-wheelchair-20261014T031747Z-a5e1bf-preflight-p3-setup/run.sh
python3 -B tools/c30_oncoming_c10_wheelchair/runner.py prepare --run-id c30-oncoming-c10-wheelchair-20261014T031747Z-a5e1bf --session /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c30-oncoming-c10-wheelchair-20261014T031747Z-a5e1bf-setup --prerequisite outputs/c30-oncoming-c10-wheelchair-20261014T031747Z-a5e1bf-preflight-p3
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c30-oncoming-c10-wheelchair-20261014T031747Z-a5e1bf-setup/run.sh
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c30-oncoming-c10-wheelchair-20261014T031747Z-a5e1bf-setup/analyze.sh
```

- [RGB GIF](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c30-oncoming-c10-wheelchair-20261014T031747Z-a5e1bf/derived/c30-oncoming-c10-wheelchair/rgb-observations.gif)
- [First-contact trajectory and corner detail](/home/gpuadmin/Workspace/TIC-VLA/outputs/c30-oncoming-preparation-20261014T031747Z-a5e1bf/first-contact-trajectory.png)
- [First corner encounter RGB](/home/gpuadmin/Workspace/TIC-VLA/outputs/c30-oncoming-preparation-20261014T031747Z-a5e1bf/first-corner-encounter-rgb.png)
- [Full trajectory with global closest approach](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c30-oncoming-c10-wheelchair-20261014T031747Z-a5e1bf/derived/c30-oncoming-c10-wheelchair/trajectory.png)
- [Later global closest-approach RGB](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c30-oncoming-c10-wheelchair-20261014T031747Z-a5e1bf/derived/c30-oncoming-c10-wheelchair/encounter-rgb.png)
- [Result JSON](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c30-oncoming-c10-wheelchair-20261014T031747Z-a5e1bf/derived/c30-oncoming-c10-wheelchair/summary.json)
- [Console](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c30-oncoming-c10-wheelchair-20261014T031747Z-a5e1bf-setup/console.log)
- [Audit](/home/gpuadmin/Workspace/TIC-VLA/outputs/c30-oncoming-preparation-20261014T031747Z-a5e1bf)
