# One stationary human at exact original C30

2026-10-14 (Asia/Seoul). **Runtime PASS, navigation FAIL** for one requested
48-prediction run. The robot contacted the corridor door/wall before reaching
C30. Human contact was zero, but the person was never passed and the goal
region was never entered. This does not establish successful human avoidance.

Instruction retained from the previous run and verified in all 48 requests:

> Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.

## Scenario and implementation

Only one fixed human is added, at exact original C30 XY
**(-6.1780195236206055, 10.137574195861816) m**. The reference is request 30
(observation simulation time 15.800000824 s) from the original no-added-obstacle
`nova-e16-hospital-lights-baseline-20261010-01`. Original requests 29 and 31
define the passage tangent. The saved pose's third component is yaw, not Z;
human root Z is 0 m, checked against the actual Hospital floor.
No C10/C24 person or added wheelchair/cart exists. Original Hospital props remain.

Added `tools/c30_human_only/` separately from the frozen C24 tool. Configuration
is identical to the preceding C24 run except for the single human's placement,
identity and saved-reference fields. Model, instruction, source human asset,
zero yaw, capsule size, robot/camera, controller, renderer, lighting, timing and
goal are preserved. Prior code, receipts and raw outputs remain unchanged.
No upstream source file changed in this task.

A fresh simulator-only preflight and the actual model run each passed all
24 checks before model load: exact XY, sole/capsule grounding, fixed actor,
absence of old actors, original source asset, camera/wheels, spawn, lighting,
render completion without physics advancement and existence of a local bypass.
Diagnostic feet/body views were reviewed. Bypass probes do not steer the robot.
The 0.25 m radius / 1.2 m cylinder capsule and T/rest-pose human are unchanged;
visual sole height correction is reused. Measured sole heights are 0 and
0.000016165672 m, floor 0.000000745058 m: maximum error about 0.0154 mm.
Visual arms can extend beyond the capsule proxy.

## Actual outcome

| Measurement | Result |
| --- | ---: |
| Predictions / matched robot-human rows | 48 / 1,522 each |
| Simulation duration | 25.350001 s |
| Measured person XY error / displacement / quaternion change | 0 / 0 / 0 |
| First visible / clearly visible observation | C12 / C18 |
| Observations containing human pixels | 13 of 48 |
| Closest robot-human centre distance | 8.295654 m |
| Human contact reports / passage | 0 / No |
| First non-floor contact | 10.350001 simulation s |
| Non-floor collider-pair spans / reports | 36 / 2,593 |
| Goal <2 m entered | No |
| Closest / final goal distance | 14.233705 / 14.254377 m |

Observation indices C12/C18 refer to this run's images, not additional obstacle
placements. Contact reports include persistent callbacks, not independent impacts.
The first target is `/Root/SM_Door_01b8/SM_Door_02b`, contacted by the right wheel
`Cylinder_05`; later targets include doorframe, trim and wall sections.
Maximum recorded impulse norm is 38.593071 N s and minimum separation is
-0.002963 m. The saved RGB confirms approach to the right-hand corridor door,
then a heavily obstructed view.

C16/C17/C18 command 1.5 m/s with positive yaw rates 0.350512/0.522664/0.593641
rad/s; these turning commands apply before first contact. They did not provide
enough clearance along the measured route. C19 still requests 1.5 m/s;
C21 slows to 0.756226 m/s at 11.433334 s, after contact. This is insufficient
clearance during corridor entry, not a demonstrated collision with the human.
It does not identify the internal reason for the model's action choice or prove
a handoff defect. Moving C24 to C30 is not isolated causally by these two
asynchronous trials. No navigation retry, tuning or extra model trial was run.

The console contains 5,157 PhysX warnings:
`PxShape::getMaterialFromInternalFaceIndex received 0xFFFFffff as input - returning NULL`
(`NpShape.cpp:598`), and zero Python tracebacks. Effects on post-contact material
response are unquantified. The warnings were recorded without suppressing them
or changing the host, simulator or asset.

## Provenance and validation

Branch `TIC-VLA-TEST`; base SHA `193c7a7b0cc261e59e0ea18242b374bec7121ae2`;
upstream `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Dirty status/diff identity
and added source hashes are recorded in the preparation audit. TEST remains
Git-free; its fresh receipts bind source/config/reference/model/runtime hashes.

Isaac 6.0.1-rc.7 / Python 3.12.13 / NumPy 2.3.1 / RealTimePathTracing simulation;
separate Isaac 5.0.0-rc.45 model-only Python 3.11.13 / torch 2.7.0+cu128 /
torchvision 0.22.0+cu128 / NumPy 1.26.0 / transformers 4.57.6. Strict checkpoint
load, no second SimulationApp. Base weight SHA256
`a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`;
checkpoint SHA256
`376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`.
No installations, driver/CUDA/system Python or environment changes.

Original Hospital origin; world XYZ metres, +Z up, yaw CCW from +X. Human PhysX
quaternion xyzw; robot quaternion wxyz. Skinned vertices map from skeleton space
to world using USD row matrices. Passage uses the world-XY reference tangent.
Native 30x2 forward/left offsets have no yaw and remain anchored to their
observation pose. Physics 60 Hz, target predictions 2 Hz, one IPC future,
OLD command retention and 48-prediction limit are unchanged. Observation,
action inference, reasoning generation/poll and command application clocks are
separately logged; wall and simulation times are distinct. This is modified
Isaac 6 simulation evidence, not unchanged Isaac 5 reproduction or general
avoidance/reconciliation validation. Goal entry would not imply stopping.

**12 focused CPU tests PASS**, including single-actor/exact-reference/config
checks and runtime-global checks. Source comparison reviewed. Independent audit
verifies 490 raw / 167 source / 376 reference files, all 48 original instructions,
finite 30x2 arrays and RGB, 1,522 paired stationary actor/robot rows, 48 masks
recomputed from saved ID arrays/maps, and the 48-frame 25.35 s GIF. All
773 protected previous source/output files are unchanged. Body/feet, trajectory,
RGB overview and encounter images were visually reviewed. Raw and derived
artifacts remain separate; no old output was overwritten. Local focused commit;
this new experiment has not been pushed.

Run: `TIC-VLA-TEST/outputs/c30-human-only-20261013T161234Z-34d314`.
Audit: `outputs/c30-human-only-preparation-20261013T161234Z-34d314`.
Derived summary, contacts, trajectory, RGB sheet and GIF: `derived/c30-human-only/`.
The GIF holds actual observations by simulation timestamps; it does not invent
intermediate video frames. Audit contains `encounter-rgb.png` and
`contact-command-review.json` with the saved observation/application timings.

Actual consumed commands; use a fresh ID for any further run:

```bash
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c30-human-only-20261013T161234Z-34d314-preflight-setup/run.sh
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c30-human-only-20261013T161234Z-34d314-setup/run.sh
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c30-human-only-20261013T161234Z-34d314-setup/analyze.sh
```

Exact prepare commands are in the audit's `preparation.json` and
`model-preparation.json`; the final local commit SHA is recorded separately.
