# C10 and C24 wheelchairs with a lower forward speed limit

2026-10-14 (Asia/Seoul). **Runtime PASS; FAIL_THIS_SCENARIO.** One requested
60-prediction model trial completed. User requested a modest controller speed
reduction with one stationary wheelchair at each original C10 and C24, no people.
No navigation retry or post-result parameter tuning was performed.

C10 was passed without contact at 7.733334 simulation s. The robot rounded
the corridor corner without any Hospital door/wall contact, then contacted the
C24 wheelchair at 14.300001 s. It did not pass C24; RGB and measured trajectory
show it turning back into the lobby. The goal region was never entered.

## Frozen changes

- Maximum forward command: **1.5 -> 1.2 m/s** (20% lower).
- Maximum yaw command stays **1.2 rad/s**; lookahead 1 m, angular gain .8,
  bearing filter .35 and epsilon .001 are unchanged. Same controller formula
  and direct Nova differential-wheel adapter. Lower `v` also changes the
  formula's `0.5*v*curvature` contribution to `w`; this is not a separate yaw retune.
- C10 remains at **[6.824887275695801, 7.096357822418213, 0] m**; add C24 at
  **[-1.7567275762557983, 10.988419532775879, 0] m**. No people. Floor-grounded,
  static original Hospital wheelchair geometry/material/collider, no scaling
  or dynamic body. Both copies use `/Root/SM_WheelChair_01a_37`, at
  `/World/ExactWheelchairC10` and `/World/ExactWheelchairC24`.
- Fixed call budget: **48 -> 60** at target 2 Hz. The 25% larger budget compensates
  the 20% speed reduction in nominal maximum travel distance; it does not guarantee
  enough time for every detour. All other simulation/model/scene settings unchanged.
- Instruction is exactly unchanged in all 60 requests:

> Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.

References are requests 10 and 24 from the original unobstructed
`nova-e16-hospital-lights-baseline-20261010-01`, with adjacent requests defining
the tangents. Place the actual visible world bounds centre at exact saved XY,
lowest mesh vertex at Z=0. Long axes follow the respective original tangents;
yaws are 1.898480540267 and -2.986313828422 rad.
Original Hospital props remain. No upstream source changed. The independent
`tools/two_wheelchairs_slow/` bundle extends the frozen single-wheelchair tool;
prior tools, receipts and raw outputs remain unchanged.

## Placement and runtime verification

First simulator-only preflight stopped before model load because the renderer
normalized `stationary_wheelchair_C10` to lowercase. Case-sensitive diagnostic
mask matching incorrectly counted zero pixels. Geometry checks passed. This
implementation bug was corrected by authoring explicit lowercase per-object
labels; no physics/geometry/controller change or relaxed gate. Failed output,
receipt and exact source snapshot are preserved in the audit. A regression test
checks two independently matched semantic labels.

Fresh preflight `-preflight-p2` and the model run passed all **30 placement and
runtime gates**. Both actual body/wheel views were visually reviewed; source
mesh/collider identity, exact XY, grounding, fixed world transforms, no people,
robot/wheels/camera, brightness, render completion without physics advance,
1.2 m/s setting and 60-call budget were checked. Local bypass probes pass both
sides of C10 and one side of C24, checking the Hospital and both wheelchairs.
They never steer the robot and do not prove a complete valid navigation route.
Preflight floor Z is -0.000000804663 m (C10), 0.000000208616 m (C24).

Recorded target and applied forward commands never exceed
**1.200000 m/s**. All 60 commands were independently recomputed
from saved native actions and the 1.2 m/s configuration. Largest measured body
forward velocity is 1.204008 m/s; physical velocity
can differ from the target due to dynamics/contact.

## Actual outcome

| Wheelchair | Passage sim s | Contact reports | First contact sim s | Min centre distance m | Displacement m |
| --- | ---: | ---: | ---: | ---: | ---: |
| C10 | 7.733334 | 0 | Not observed | 1.114266 | 0.0 |
| C24 | Not observed | 619 | 14.300001 | 0.748393 | 0.0 |

| Navigation measurement | Result |
| --- | ---: |
| Predictions / robot rows / wheelchair rows | 60 / 1886 / 3772 |
| End simulation time | 31.416668 s |
| First goal entry (<2 m) | Not observed |
| Closest / final goal distance | 11.129198 / 22.840990 m |
| Non-floor contact spans / reports | 9 / 619 |
| Non-floor spans through first goal entry or end | 9 |
| First non-floor contact | 14.300001 s |
| C10 / C24 visible observation count | 31 / 24 |

First reported non-floor contact: `/World/ExactWheelchairC24/SM_WheelChair_01a`, `/World/Robots/Nova_Carter/chassis_link/chassis_link`.
Maximum reported contact impulse norm 57.087220 N s;
minimum separation -0.001103 m. Contact reports
are persistent callbacks, not separate impacts. Passage is a conservative plane
crossing beyond the reference tangent, wheelchair half-length plus robot radius.
Centre distance is not surface clearance. Both fixed world transforms remained
unchanged; these are verified static USD transforms, not dynamic PhysX measurements.

Saved command context around the first contact (or closest approach if none):

| Observation | Observation sim s | Application sim s | v m/s | w rad/s |
| --- | ---: | ---: | ---: | ---: |
| C24 | 12.916667 | 13.016667 | 1.200000 | 0.079326 |
| C25 | 13.416667 | 13.516667 | 1.200000 | 0.065417 |
| C26 | 13.916667 | 14.016667 | 1.200000 | -0.001088 |
| C27 | 14.416667 | 14.516667 | 0.025114 | 1.100251 |
| C28 | 14.916667 | 15.016667 | 0.631869 | 1.029951 |
| C29 | 15.416667 | 15.516667 | 0.530847 | 0.985718 |

C24 was clearly visible from observation C14. C24-C26 (observations in this
run, not placement names) still command 1.2 m/s immediately before impact,
with only small yaw commands. C27 slows to 0.025114 m/s and turns at
1.100251 rad/s at 14.516667 s, after the first contact. The commanded route
did not supply sufficient wheelchair clearance before impact. This is saved
execution evidence, not a claim about why the model selected those actions.

The console contains 2455 PhysX
`PxShape::getMaterialFromInternalFaceIndex` NULL-material warnings at
`NpShape.cpp:598`, and 0 Python tracebacks. Post-contact material-response
effects are unquantified. No host/asset changes were made to suppress warnings.

Speed, second obstacle and run budget changed together; this single asynchronous
trial cannot isolate speed as the cause of any outcome difference. A successful
wheelchair passage is specific to this trial. Goal entry does not establish a stop:
the fixed 60-call loop continues. This is modified Isaac6 simulation, not unchanged
Isaac5 reproduction, general avoidance certification or real-world evidence.

## Provenance and checks

Branch `TIC-VLA-TEST`; source base `de0d48b4552a9ef294a83d08efca92b172790cfa`;
upstream `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Audit records dirty status,
tracked diff and exact new-source hashes (including the corrected preflight-p2
snapshot). TEST is Git-free. Fresh receipts bind source/config/reference/runtime
and weights. Final local commit is recorded separately in `git-finalization.json`.

Isaac6.0.1-rc.7 / Python3.12.13 / NumPy2.3.1 / RealTimePathTracing simulation;
Isaac5.0.0-rc.45 inference-only Python3.11.13 / torch2.7.0+cu128 /
torchvision0.22.0+cu128 / NumPy1.26.0 / transformers4.57.6, strict checkpoint load,
no second SimulationApp. GPU RTX5060Ti, driver595.99.02. Base weight SHA256
`a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`;
checkpoint SHA256
`376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`.
No package/model/driver/CUDA/system Python changes, training or downloads.

Original Hospital origin, world XYZ metres, +Z up; yaw CCW from +X, robot
quaternion wxyz. Wheelchair local mesh points map to world using USD row matrices.
Native 30x2 forward/left offsets contain no yaw. 60 Hz physics, one IPC future,
OLD retention and FRESH observation anchoring remain unchanged. Observation,
action-inference ready, reasoning generation/poll and command application clocks
remain separate; simulation seconds differ from wall-clock elapsed time.

**15 focused CPU tests PASS**. Evidence audit checked 167
source hashes, 383 reference hashes, 732 raw hashes,
60 finite native action records, RGB dimensions/decoding and exact instruction,
120 independently reconstructed object masks, 60 GIF frames
(31.41 s), paired state rows and unchanged static matrices.
830 protected source/prior-result files plus the failed
preflight remain unchanged. Raw and derived artifacts are separate and uncommitted.

## Commands and artifacts

Executed in the modified copy; these IDs are already used and cannot be reused:

```bash
cd /home/gpuadmin/Workspace/TIC-VLA-TEST
python3 -B tools/two_wheelchairs_slow/runner.py prepare --run-id two-wheelchairs-slow-20261014T012825Z-59d63c-preflight-p2 --session /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-slow-20261014T012825Z-59d63c-preflight-p2-setup --preflight
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-slow-20261014T012825Z-59d63c-preflight-p2-setup/run.sh
python3 -B tools/two_wheelchairs_slow/runner.py prepare --run-id two-wheelchairs-slow-20261014T012825Z-59d63c --session /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-slow-20261014T012825Z-59d63c-setup --prerequisite outputs/two-wheelchairs-slow-20261014T012825Z-59d63c-preflight-p2
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-slow-20261014T012825Z-59d63c-setup/run.sh
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-slow-20261014T012825Z-59d63c-setup/analyze.sh
```

Launch bounds: 900 wall seconds plus 30-second termination grace. Audit scripts
and commands are archived alongside preparation/visual-review/validation records.

- [Trajectory](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-slow-20261014T012825Z-59d63c/derived/two-wheelchairs-slow/trajectory.png)
- [RGB GIF](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-slow-20261014T012825Z-59d63c/derived/two-wheelchairs-slow/rgb-observations.gif)
- [RGB overview](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-slow-20261014T012825Z-59d63c/derived/two-wheelchairs-slow/rgb-contact-sheet.png)
- [Contact context RGB](/home/gpuadmin/Workspace/TIC-VLA/outputs/two-wheelchairs-slow-preparation-20261014T012825Z-59d63c/encounter-rgb.png)
- [Result JSON](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-slow-20261014T012825Z-59d63c/derived/two-wheelchairs-slow/summary.json)
- [Console log](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-slow-20261014T012825Z-59d63c-setup/console.log)
- [Preparation audit](/home/gpuadmin/Workspace/TIC-VLA/outputs/two-wheelchairs-slow-preparation-20261014T012825Z-59d63c)
