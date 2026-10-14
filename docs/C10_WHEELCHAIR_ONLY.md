# One stationary wheelchair at exact original C10

2026-10-14 (Asia/Seoul). **Runtime PASS; wheelchair passed without reported
contact; overall navigation FAIL.** One requested 48-prediction trial completed.
The robot passed the wheelchair at 6.766667 simulation s, then
contacted a corridor door at 10.283334 s and never
entered the goal region. No navigation retry or parameter tuning was performed.

The exact initial instruction was retained in all 48 requests:

> Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.

## Scenario and implementation

Only one added wheelchair is present, at exact original C10 XY
**(6.824887275695801, 7.096357822418213) m**, lowest visible mesh vertex at
Z=0 m. No added people, C24 wheelchair or cart. Original Hospital props remain.
Reference: `nova-e16-hospital-lights-baseline-20261010-01`, request 10 at
5.800000302 simulation s, from the original run without added obstacles.
Its `pose` third component is yaw, not height. Requests 9 and 11 define the
local path tangent. The visible world bounding-box centre is placed exactly
at C10, without a horizontal offset.

`tools/c10_wheelchair_only/` is an additive independent bundle derived from the
frozen mixed-object tool. It removes human creation and human state logging,
places the original Hospital wheelchair at C10, and records only wheelchair
state/masks. Its analyzer reports wheelchair passage/contact and goal reach.
`tests/test_c10_wheelchair_only.py` checks single-object configuration, exact
reference consistency, original instruction, no input mutation and missing
globals. Previous bundles are unchanged. No upstream source changes in this task.
All non-actor configuration matches the previous C30 human run: instruction,
model, camera, robot, controller, lighting, renderer, goal and timing.

The unchanged source `/Root/SM_WheelChair_01a_37` is referenced at
`/World/ExactWheelchairC10`. Existing geometry, material and static collision
mesh are retained; no scaling, dynamic rigid body or movement script. Instance
proxies are included when computing visible mesh bounds. Long axis is parallel
to the saved C9-to-C11 tangent (yaw 1.898480540267 rad).
World bounds are X [6.321429869, 7.328344682], Y [6.473929952, 7.718785693],
Z [0, 1.159056490] m. Original mesh half extents in local XY are
[0.552719161, 0.412630205] m.

Simulator-only preflight and actual model run each passed **20 checks** before
model load: exact XY, floor grounding, unchanged source/robot/camera/wheels,
absence of old added actors, fixed wheelchair transform, no initial overlaps,
lighting, completed render without physics advancement, semantic visibility
and existence of a local bypass. Floor ray Z=-0.000000804663 m gives a
0.000805 mm grounding error. Body/wheel diagnostic views were visually reviewed.
Both local bypass probes passed with 0.1 m proxy clearance; they never steer
the robot. They do not validate the entire route to the goal.

## Measured outcome

| Measurement | Result |
| --- | ---: |
| Predictions / paired robot-wheelchair rows | 48 / 1,528 each |
| Simulation duration | 25.450001 s |
| Exact XY error / wheelchair displacement / transform change | 0 / 0 / 0 |
| First visible / clearly visible observation | C1 / C1 |
| Observations containing wheelchair pixels | 9 of 48 |
| Minimum robot-wheelchair centre distance | 0.974372 m |
| Wheelchair contact reports | 0 |
| Wheelchair passage | 6.766667 s |
| First non-floor contact | 10.283334 s |
| Non-floor collider-pair spans / contact reports | 46 / 3,289 |
| Goal <2 m entered | No |
| Closest / final goal distance | 14.542965 / 14.542965 m |

Passage means the robot centre crossed the saved tangent plane beyond the
wheelchair long-axis half extent plus the robot conservative radius. Centre
distance is not surface clearance. Contact reports include persistent callbacks,
not distinct impacts. Static wheelchair position comes from its authored geometry
with the world USD matrix verified at every recorded tick; it is not a dynamic
PhysX rigid-body measurement.

RGB and trajectory show the robot passing on the lower-world-X side of the
wheelchair, then approaching the corridor door. First contact is
`/Root/SM_Door_01b8/SM_Door_02b` against right wheel `Cylinder_04`; later contacts
also involve `/Root/Geo_M_DoorFrame52/Geo_M_DoorFrame/Geo_M_DoorFrame`.
Maximum recorded contact impulse norm is 67.181784 N s;
minimum separation is -0.006363 m.

C16/C17/C18 command 1.5 m/s with yaw rates 0.024174/0.226483/0.236198 rad/s,
applied at 9.050000/9.550000/10.050001 s, before first contact. C19 still
commands 1.5 m/s after contact; C20 slows to 0.760363 m/s at 11.033334 s.
The measured route did not maintain door clearance. This does not identify
why the model chose those actions or demonstrate a handoff inconsistency.
Wheelchair non-contact passage in this trial does not imply general avoidance
competence. Goal entry would not prove stopping in this fixed 48-call loop.

There are 5,496 `PxShape::getMaterialFromInternalFaceIndex`
NULL-material warnings (`NpShape.cpp:598`) and 0 Python tracebacks.
Effects on post-contact material response remain unquantified. Warnings were
retained; no host, asset or simulator change was made to suppress them.

## Provenance and verification

Branch `TIC-VLA-TEST`, base SHA `b632e93dd2602cd1e4cd07a4e067a02f2c452abc`;
upstream SHA `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Dirty status/diff
identity and new source hashes are saved in the preparation audit. TEST remains
Git-free; exclusive fresh receipts bind source/config/reference/runtime/weights.
Final local commit identity is recorded separately in `git-finalization.json`.

Isaac 6.0.1-rc.7 / Python 3.12.13 / NumPy 2.3.1 / RealTimePathTracing simulation;
separate Isaac 5.0.0-rc.45 inference-only Python 3.11.13 / torch 2.7.0+cu128 /
torchvision 0.22.0+cu128 / NumPy 1.26.0 / transformers 4.57.6. Strict checkpoint
load; no second SimulationApp. GPU RTX5060Ti, driver 595.99.02. Base model SHA256
`a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`;
checkpoint SHA256
`376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`.
No installations, training, model downloads or host environment changes.

Original Hospital origin, world XYZ metres, +Z up, yaw CCW from +X; robot
quaternion wxyz. Wheelchair local points map to world by USD row-vector
matrices. Native 30x2 forward/left offsets have no yaw. Observation, action
inference ready, reasoning generation/poll and command application are distinct
recorded times. 60 Hz physics, target 2 Hz action requests, one outstanding IPC
future and OLD retention are unchanged; FRESH remains observation-anchored.
Simulation duration is not elapsed wall time. This is the modified Isaac6 harness,
not unchanged official Isaac5 reproduction or real-world evidence.

**11 focused CPU tests PASS**, including relevant existing wheelchair/exact-human
tests. Evidence audit independently verified 539 raw hashes,
167 source hashes, 378 reference hashes,
48 exact instruction/native finite 30x2 action records and decoded 1920x1080 RGBs,
48 semantic masks from saved ID rasters/maps, and the 48-frame GIF (25.45 s).
776 protected source/prior result files are
unchanged. Raw/derived outputs are separate and uncommitted. No old run was
reused or overwritten. Trajectory and both RGB contact sheets were reviewed.

## Commands and artifacts

Executed from the modified copy, with fresh exclusive setup directories:

```bash
cd /home/gpuadmin/Workspace/TIC-VLA-TEST
python3 -B tools/c10_wheelchair_only/runner.py prepare --run-id c10-wheelchair-only-20261014T011550Z-75cf4a-preflight --session /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c10-wheelchair-only-20261014T011550Z-75cf4a-preflight-setup --preflight
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c10-wheelchair-only-20261014T011550Z-75cf4a-preflight-setup/run.sh
python3 -B tools/c10_wheelchair_only/runner.py prepare --run-id c10-wheelchair-only-20261014T011550Z-75cf4a --session /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c10-wheelchair-only-20261014T011550Z-75cf4a-setup --prerequisite outputs/c10-wheelchair-only-20261014T011550Z-75cf4a-preflight
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c10-wheelchair-only-20261014T011550Z-75cf4a-setup/run.sh
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c10-wheelchair-only-20261014T011550Z-75cf4a-setup/analyze.sh
```

These historical IDs are already used. Re-execution requires fresh run/setup IDs.
Launches are bounded to 900 wall seconds plus a 30-second termination grace.
Independent verifier and contact-review scripts are archived with their commands.

- [Trajectory](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c10-wheelchair-only-20261014T011550Z-75cf4a/derived/c10-wheelchair-only/trajectory.png)
- [RGB observation GIF](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c10-wheelchair-only-20261014T011550Z-75cf4a/derived/c10-wheelchair-only/rgb-observations.gif)
- [RGB contact sheet](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c10-wheelchair-only-20261014T011550Z-75cf4a/derived/c10-wheelchair-only/rgb-contact-sheet.png)
- [Door-contact RGB](/home/gpuadmin/Workspace/TIC-VLA/outputs/c10-wheelchair-only-preparation-20261014T011550Z-75cf4a/encounter-rgb.png)
- [Saved result](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c10-wheelchair-only-20261014T011550Z-75cf4a/derived/c10-wheelchair-only/summary.json)
- [Console log](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c10-wheelchair-only-20261014T011550Z-75cf4a-setup/console.log)
- [Preparation and validation audit](/home/gpuadmin/Workspace/TIC-VLA/outputs/c10-wheelchair-only-preparation-20261014T011550Z-75cf4a)
