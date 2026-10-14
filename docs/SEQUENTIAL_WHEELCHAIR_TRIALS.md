# Three sequential static wheelchair trials

Executed **1 → 2 → 3**, once per condition, with fresh preflight and model run
IDs. Runtime completed for all three; navigation and encounter outcomes below
are separate from process exit/runtime PASS. No retries or outcome-driven
position/controller/renderer tuning were performed.

## Requested conditions

| Step | Episode / obstacles | max_v | Predictions | Instruction |
|---|---|---:|---:|---|
| 1 | E21; one static wheelchair at C10 | 0.8 m/s | 90 | Original E21 |
| 2 | E16; C10 and C24 static wheelchairs; added C24 cart removed | 0.5 m/s | 144 | Original E16 |
| 3 | Identical to 2 | 0.5 m/s | 144 | Original E16 plus requested English sentence |

Only the previously added `/World/E16Cart` copy is absent in 2 and 3; native
Hospital props remain. No people or moving actors are added. Fixed budgets
preserve the earlier proportional nominal travel allowance (0.8×90 and
0.5×144); actual speed can be below its cap. max_w=1.2 rad/s, lookahead=1.0 m,
angular_gain=0.8, filter_gain=0.35, epsilon=0.001 remain unchanged. Direct wheel
command adaptation and model/checkpoint are unchanged. Goal entry is <2 m XY;
fixed-budget execution does not stop automatically on entry.

E21 C10 uses the latest no-added-obstacle current-render run
`e21-current-render-20261014T044932Z-68ab22`, C10 **(17.984498978,0.179084986,0)** m.
E16 placements retain the original no-added-obstacle reference
`nova-e16-hospital-lights-baseline-20261010-01`:
C10 **(6.824887276,7.096357822,0)** and C24 **(-1.756727576,10.988419533,0)** m.
They are exact saved observation XY, not approximate locations or the new
run's C10/C24 poses. Actual mesh-bounds centre is placed there, lowest vertex
on raycast floor Z=0; unchanged source wheelchairs use their original collision
mesh, scale and material. Long axes follow each reference C9→C11 or C23→C25
tangent. Static USD transform equality was measured each physics tick.

E21 instruction:

> Exit the room and turn right to enter the hallway. Continue straight ahead and to the end of the hallway.

E16 step 2 instruction:

> Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.

Step 3 appends exactly:

> If you see the wheelchair at C24, navigate around it on the left.

The user explicitly requested English after initial preparation; the correction
was applied before any prepared execution/source receipt. Step 2 and 3 config
objects compare equal after removing **only this appended instruction**; actor
asset receipts compare equal. Every request's actual model instruction was
verified. The unchanged official geometry guard receives the original text
only while constructing the scene; the requested text is retained in metadata,
`sim.cfg`, model configuration and all actual prediction requests. No control
policy is forced left. C24 is a placement label, not an RGB text overlay or a
semantic mask input to the model, so this test does not establish that the
symbolic label was understood.

## Measured results

| Step | C10 passage / contacts | C24 passage | Closest goal | Goal region entered | Non-floor spans |
|---|---|---|---:|---|---:|
| 1 | right / 0 | Not placed | 12.886 m | No | 12 |
| 2 | left / 0 | NOT REACHED | 17.424 m | No | 36 |
| 3 | left / 0 | NOT REACHED | 17.242 m | No | 6 |

All three runs passed C10 without wheelchair contact but failed to reach the
goal. E21 later contacted the native wet-floor sign and stopped making useful
progress. Both E16 runs missed the left hallway: step 2 continued north beyond
the staircase, while step 3 looped around its east side and returned toward
the starting area. Neither traversed the C24 obstacle section. Thus C24 left
avoidance followed by goal arrival was not demonstrated.

### 1. e21_c10_v08

Run `seq1-e21-c10-v08-20261014T051203Z-aca2c4`. **FAIL_THIS_SCENARIO**; 90 actual finite
predictions, 2783 measured robot rows, 46.366669 s.
C10 centre-plane side: **right**, signed left distance
-0.9859567785638146 m;
fully beyond the enclosing passage threshold at 10.216667199507356 s.
C10 contacts: 0; minimum robot/obstacle centre distance
0.953670 m (not an exact surface clearance).

First goal entry: None; closest goal distance
**12.885903 m**, final distance **13.150324 m**.
Non-floor collider-pair spans: 12; callbacks with
near-horizontal contact normals: 4212.
First positive-impulse contact: 20.75000108219683 s,
`/World/Robots/Nova_Carter/wheel_right/Cylinder_05` ↔ `/Root/S_WetWarningSign/S_WetFloorSign`.
Raw callbacks and pair spans are not counts of independent collision events.
RGB criterion failures: 17/90; no quality
criterion was loosened. Full per-frame outcomes are in derived brightness.json.

### 2. e16_no_cart_v05

Run `seq2-e16-no-cart-v05-20261014T051203Z-aca2c4`. **FAIL_THIS_SCENARIO**; 144 actual finite
predictions, 4398 measured robot rows, 73.283337 s.
C10 centre-plane side: **left**, signed left distance
1.500113219200976 m;
fully beyond the enclosing passage threshold at 15.383334135636687 s.
C10 contacts: 0; minimum robot/obstacle centre distance
1.494943 m (not an exact surface clearance).

C24 first nonzero semantic pixels: C29; first CLEAR:
None. Centre-plane passage: **NOT REACHED**;
fully-passed time None; contacts 0;
minimum centre distance 7.002544 m.
C24 left passage and collision-free goal result: False.
Without passage, the requested C24-side comparison was not completed.

First goal entry: None; closest goal distance
**17.423574 m**, final distance **21.092811 m**.
Non-floor collider-pair spans: 36; callbacks with
near-horizontal contact normals: 4349.
First positive-impulse contact: 40.15000209398568 s,
`/World/Robots/Nova_Carter/wheel_left/Cylinder_01` ↔ `/Root/Geo_W_TrimStrateBase2_179/Geo_W_TrimStrateBase2_8/Geo_W_TrimStrateBase2_8`.
Raw callbacks and pair spans are not counts of independent collision events.
RGB criterion failures: 75/144; no quality
criterion was loosened. Full per-frame outcomes are in derived brightness.json.

### 3. e16_no_cart_v05_left

Run `seq3-e16-no-cart-v05-left-20261014T051203Z-aca2c4`. **FAIL_THIS_SCENARIO**; 144 actual finite
predictions, 4400 measured robot rows, 73.316670 s.
C10 centre-plane side: **left**, signed left distance
1.0096780056065655 m;
fully beyond the enclosing passage threshold at 15.333334133028984 s.
C10 contacts: 0; minimum robot/obstacle centre distance
0.988045 m (not an exact surface clearance).

C24 first nonzero semantic pixels: C27; first CLEAR:
35. Centre-plane passage: **NOT REACHED**;
fully-passed time None; contacts 0;
minimum centre distance 6.740139 m.
C24 left passage and collision-free goal result: False.
Without passage, the requested C24-side comparison was not completed. Step 3
satisfies the fixed numeric CLEAR rule at C35; actual RGB shows the wheelchair
partly visible at the far left edge. This does not establish understanding of
the C24 label or intentional rejection of the instruction.

First goal entry: None; closest goal distance
**17.242414 m**, final distance **24.993754 m**.
Non-floor collider-pair spans: 6; callbacks with
near-horizontal contact normals: 227.
First positive-impulse contact: 71.80000374466181 s,
`/World/Robots/Nova_Carter/wheel_right/Cylinder_04` ↔ `/Root/Geo_M3_TrimStrateSide4/Geo_M3_TrimStrateSide/Geo_M3_TrimStrateSide`.
Raw callbacks and pair spans are not counts of independent collision events.
RGB criterion failures: 42/144; no quality
criterion was loosened. Full per-frame outcomes are in derived brightness.json.

## Gating, clocks and limitations

Operational/geometry preflight checks: **21/21** for E21, **31/31** for each
E16 run; source mesh semantics and diagnostic grounding views passed with zero
model calls/navigation steps after settling. The actual model run repeats the
placement/camera gate before loading inference. Existing E21 overexposure was
retained as an independent RGB failure, consistent with the user's explicitly
requested static-object diagnostic; this is not a qualified RGB baseline.
Initial E16 RGB passed. No human/dynamic execution or general avoidance claim.

Current renderer is RealTimePathTracing, unchanged 153-fixture Hospital lighting,
front Hawk left camera at 1920×1080. Completed Replicator capture uses zero
simulation-time advance; each recorded observation was checked against its
measured physics row. Camera/body extrinsics and wheels match historical source.
Simulation is Isaac 6.0.1-rc.7, Python 3.12.13, NumPy 2.3.1. Model-only worker is
Isaac 5.0.0 Python 3.11.13, torch 2.7.0+cu128, torchvision 0.22.0+cu128,
NumPy 1.26.0, transformers 4.57.6, strict released checkpoint, no second
SimulationApp. No package/runtime/driver or host environment changes.

60 Hz physics, target 2 Hz observation cadence, one pending IPC future with
OLD target retention and FRESH anchored to its own observation. Simulation time
is separate from wall time, inference start/ready, reasoning generation/poll and
command application. The exact wall-clock order of measured robot traces was
verified 1 before 2 before 3. Same conditions do not imply matched asynchronous
reasoning/timing realizations; three runs cannot prove a causal effect of
cart removal, speed or instruction alone.

All coordinates are metres in original Hospital world coordinates, +Z up;
robot yaw is CCW from +X, quaternion wxyz. Native model outputs are 30×2
body-forward/left offsets with no native yaw, not an SE(2) trajectory.
USD transforms use row-vector local→world convention. Passage side is measured
at the first forward crossing of the fixed reference centre plane, with
forward=(cos yaw,sin yaw), left=(-sin yaw,cos yaw); E16 C24 left is approximately
world -Y. This is observed passage geometry, not inferred route intention or
exact swept-body clearance. No graph/reconciliation/correspondence method.

## Artifacts and validation

All raw outputs remain under `/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/`; the run IDs above are distinct.
Each has `derived/sequential-wheelchairs/summary.json`, brightness.json,
contacts.json, trajectory.png, rgb-contact-sheet.png and rgb-observations.gif.
Raw lossless RGB, native action chunks, event clocks, per-tick robot/static
wheelchair traces and semantic masks/ID maps are preserved separately.
Overview figure: `/home/gpuadmin/Workspace/TIC-VLA/outputs/sequential-wheelchair-preparation-20261014T051203Z-aca2c4/three-trial-comparison-reviewed.png`.
Audit: `/home/gpuadmin/Workspace/TIC-VLA/outputs/sequential-wheelchair-preparation-20261014T051203Z-aca2c4`. Each preparation command is archived there; each setup folder
contains immutable config/receipt, exact bounded launch argv, console.log,
run.sh/analyze.sh/verify.sh and process-result.json. Existing paths are rejected.

**32 focused CPU tests PASS** (blind-corner, Hospital lighting, continuous
handoff), source syntax/global checks PASS, exact configuration checks PASS.
All **378 native outputs and controller commands**, instruction strings,
observation-to-physics-row mappings and static transforms verified.
Verified raw file counts by stage: 1001, 1740, 1740;
170 source hashes and 22 external files per receipt; prerequisite/reference
hashes verified separately. GIF frame counts match 90/144/144. No Python
traceback in any actual model run. All **7084 prior protected files**
remain unchanged, and executed TEST bundle equals original workspace source.
PhysX material warnings occur in contact-heavy runs; counts are in validation
and warning-count-correction.json. The initial read-only helper used an
incorrect warning substring for stages 1/2; the appended correction reads
actual `PxShape::getMaterialFromInternalFaceIndex` occurrences. This did not
affect execution, contact measurements, source, configurations or outcome gates.

Starting branch `hardcase-probe`; local SHA `8f74d6759ff8cac30f04c6135cd00f6f375c4f76`;
upstream SHA `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Dirty status/diff hash, added-source
identities and current-copy source hashes are preserved in preparation and
execution receipts. Base weights SHA
`a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`;
checkpoint SHA
`376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`.
Stage 2 versus prior cart run removes adjacent_cart and refreshes two stale
Nova source identity hashes; no other semantic configuration changes. Stage 3
versus actual stage 2 differs only in instruction, with equal mesh receipts.

Only additive `tools/sequential_wheelchair_trials/` and documentation change.
No existing upstream/model/controller/research source or prior output is edited.
Generated outputs, model weights, caches and recordings remain ignored.
