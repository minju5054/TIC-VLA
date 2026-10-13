# C24 wheelchair with explicit obstacle-avoidance instruction

2026-10-14 (Asia/Seoul). **One requested run completed: runtime PASS,
navigation FAIL.** C10 was passed without contact; the robot contacted the
stationary C24 wheelchair and never reached the strict2 m goal region.

The exact user instruction was verified in **all48 actual prediction requests**:

> Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway. If an obstacle blocks your path, navigate around it.

New run:
`TIC-VLA-TEST/outputs/c24-wheelchair-avoid-instruction-20261013T152928Z-e2af9b`.
Previous original-instruction run:
`TIC-VLA-TEST/outputs/c10-human-c24-wheelchair-20261013T150824Z-fc819a`.
Full audit and saved-only comparison:
`outputs/wheelchair-instruction-preparation-20261013T152928Z-e2af9b/`.

## What changed

The complete configuration differs in exactly **one key: `instruction`**.
All scene/actor positions, grounding, original Hospital mesh/collider/material,
robot/camera, goal, lighting, renderer, seed, weights, controller and timing
settings match the previous wheelchair run. Both added obstacles remain fixed.

C10 human XY is `(6.824887275695801,7.096357822418213)` m. C24 wheelchair
footprint-centre XY is `(-1.7567275762557983,10.988419532775879)` m, both taken
from the original unobstructed path. Measured C10 error0; numerical C24 error
2.22e-16 m. Human position/quaternion change0; static wheelchair world-transform
change0 over1,521 paired samples each. The wheelchair is a static USD collider,
not a measured dynamic rigid body. Geometry and grounding are detailed in
[C24_WHEELCHAIR.md](C24_WHEELCHAIR.md).

Additive `tools/wheelchair_instruction/` retains the unchanged official episode
guard: during scene construction only, it supplies a deep copy with the old
instruction after requiring every other configuration field to match the
previous run. No inference occurs in that constructor. It then restores the
actual new instruction in `sim.cfg`; metadata, model config and every request
contain the new text. `instruction-intervention.json` explicitly records this
projection. Neither the official episode definition nor its validator changed.
This is an authorized research instruction intervention, not an unchanged
official-episode reproduction.

The completed prior geometry/semantic preflight was reused only after checking
exact equality of all non-instruction configuration fields. Its files are
hashed in the new receipt. All32 geometry/render checks ran again in the actual
new simulator before model load; the wheelchair asset report exactly matches
the prior preflight. No new placement search, model-free simulator rerun,
navigation retry, controller tuning, or additional model run was performed.

## Comparison

| Measured result | Original instruction | Added avoidance sentence |
| --- | ---: | ---: |
| Predictions | 48 | 48 |
| C10 contact-free passage [simulation s] | 6.333334 | 6.350000 |
| C24 first visible / clear observation | 11 / 12 | 12 / 12 |
| First wheelchair contact [simulation s] | 11.950001 | 11.866667 |
| Wheelchair contact reports | 801 | 1,080 |
| Non-floor collider-pair spans | 3 | 7 |
| Wheelchair passed | No | No |
| Strict2 m goal entry | No | No |
| Closest goal distance [m] | 11.035151 | 11.012410 |
| Final goal distance [m] | 11.035152 | 11.720150 |

Contact reports include persistent callbacks and are not independent impacts.
All non-floor contact targets in both runs are the original wheelchair mesh
at `/World/ExactWheelchairC24/SM_WheelChair_01a`. New-run maximum contact impulse
norm69.958838 N·s and minimum separation-0.001149029 m support actual contact.
No C10 contact or Hospital structural contact was reported.

The new run's C20/C21 commands remain1.5 m/s with w0.124610/0.086523 rad/s.
C22 observes at11.833334 s and issues a larger turn w0.500424 rad/s at
11.933334 s, **after** the first contact at11.866667 s. Before that switch the
previous command remains active, as required by the original continuous loop.
Slowing to0.666533 m/s occurs with C23 at12.433334 s, also after contact.
These timings document insufficient clearance/slowing before impact; they do
not by themselves identify a reasoning, perception, or handoff defect.

The added sentence did **not produce successful avoidance in this trial**.
This is one asynchronous stochastic run per condition, not a causal or
statistical instruction-effect estimate. Initial robot position is identical,
but initial RGB is not pixel-identical (mean absolute difference1.339555/255,
maximum156/255). Renderer variation and scheduling/model sampling remain
uncontrolled. Post-contact behavior diverges; it is not successful passage.

The new console also contains3,216 PhysX warnings:
`PxShape::getMaterialFromInternalFaceIndex received 0xFFFFffff as input - returning NULL`
from `NpShape.cpp:598`, versus0 in the previous run. No runtime exception
occurred, and actual contact records/RGB remain available. The warning's impact
on contact-material response is unquantified; physics equivalence for the
divergent contact sequence is not established. No host/asset patch or extra
trial was attempted to suppress it.

## Provenance, scope and validation

Source base `7854909ecf762bace42284b3e6a11615439966a8`, `hardcase-probe`;
upstream `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. TEST is Git-free; current
source/reference/config/model/runtime identities are frozen in its new receipt.
Original dirty status/diff identity and added source hashes are in the audit.

Same Isaac6.0.1-rc.7/Python3.12.13/NumPy2.3.1/RealTimePathTracing simulation;
separate Isaac5 model-only Python3.11.13/torch2.7.0+cu128/NumPy1.26.0/
transformers4.57.6, without another SimulationApp. Base weight SHA256
`a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`;
checkpoint SHA256
`376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`.
No upstream/model/controller/package/driver/Isaac-installation changes.

Original Hospital origin, worldXYZ metres,+Z up, yawCCW from+X. Human PhysX
quaternionxyzw; robot quaternionwxyz. Native30x2 forward/left offsets retain
their observation origin; no native yaw or SE(2) data invented. USD mesh points
use local-to-world row matrices. Passage uses original world-XY tangent planes.
60 Hz physics, target2 Hz predictions, one IPC future, OLD command retention
and48-prediction limit are unchanged. Simulation ends at25.333335 s; wall
time, inference start/ready, reasoning start/poll and application clocks remain
distinct. No reconciliation or real-world performance claim.

Actual commands, already consumed:

```bash
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-wheelchair-avoid-instruction-20261013T152928Z-e2af9b-setup/run.sh
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-wheelchair-avoid-instruction-20261013T152928Z-e2af9b-setup/analyze.sh
```

Per-run derived results are under `derived/c24-wheelchair/`. The audit's
`comparison/` contains the two-run summary and `instruction-comparison.png`.
The48-frame GIF contains only actual saved observations held by simulation
timestamps, not reconstructed intermediate video. Raw and prior outputs remain
unchanged; repeated commands require fresh output IDs.

Validation: **22 focused CPU tests PASS**; source symbol/AST and generated
shell checks PASS. Independent audit verifies589 raw/171 source/1,041 reference
files,48 finite native arrays/RGB,48 exact new instructions,1,521 matched
robot/human/wheelchair rows,96 masks recomputed from archived ID arrays/maps,
and48-frame25.33 s GIF.874 prior source/output files remain unchanged. Current
RGB sheet and comparison figure were visually reviewed. The user's untracked
`run_test.sh` is untouched. Focused local commit, no push.
