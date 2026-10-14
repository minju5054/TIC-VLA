# C10/C24 wheelchairs at a 1.0 m/s controller limit

2026-10-14 (Asia/Seoul). **Runtime PASS; FAIL_THIS_SCENARIO.** One user-requested
72-prediction trial completed. The full measured outcome is below; no navigation
retry or post-result tuning was performed.

C10 was passed without contact at 8.666667 simulation s. The robot contacted
the corridor wall/door trim at 14.516667 s, before reaching C24; the closest
robot-to-C24 centre distance was 3.340528 m. It never passed C24 or entered the
goal region. RGB after contact becomes heavily obstructed by nearby geometry.
Zero C24 contact is therefore not evidence of successful C24 avoidance.

## Change and controls

Maximum forward command **1.2 -> 1.0 m/s** (16.7% lower). Call budget **60 -> 72**
(20% larger), retaining the same nominal maximum travel-distance budget at
2 Hz. Maximum yaw command 1.2 rad/s, lookahead 1 m, angular gain .8, bearing
filter .35 and epsilon .001 are unchanged. Same controller formula and direct
Nova differential-wheel adapter. Lower v also lowers the formula's v-dependent
curvature term in w; no independent angular retuning was introduced.

All other configuration is identical to the preceding 1.2 m/s run, including:

- Exact C10 **[6.824887275695801, 7.096357822418213, 0] m**.
- Exact C24 **[-1.7567275762557983, 10.988419532775879, 0] m**.
- No people; two fixed, floor-grounded copies of original Hospital wheelchair
  `/Root/SM_WheelChair_01a_37` at `/World/ExactWheelchairC10` and `C24`.
  Same geometry, source bytes, materials, collision mesh, orientation, dimensions,
  transforms and lowercase per-object diagnostic labels. No scaling or dynamic
  body. Original Hospital props remain.
- Same original instruction, verified in all 72 requests:

> Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.

The reference is `nova-e16-hospital-lights-baseline-20261010-01`, requests 10/24,
without added obstacles. Visible mesh world-bounds centres use exact reference
XY; lowest vertex Z=0. Long axes follow the adjacent-request tangents.
The new `tools/two_wheelchairs_v1/scene.py` is byte-identical to the prior scene
except `MAX_V=1.0` and `PREDICTIONS=72`. Runner/analyzer changes are namespace,
speed descriptions and call-count labels. No upstream source or prior bundle
changed; old receipts/raw outputs remain intact.

## Measured outcome

| Wheelchair | Passage sim s | Contact reports | First contact sim s | Min centre distance m | Displacement m |
| --- | ---: | ---: | ---: | ---: | ---: |
| C10 | 8.666667 | 0 | Not observed | 1.240206 | 0.0 |
| C24 | Not observed | 0 | Not observed | 3.340528 | 0.0 |

| Measurement | Prior 1.2 m/s | Current 1.0 m/s |
| --- | ---: | ---: |
| Predictions | 60 | 72 |
| Goal entry (<2 m), sim s | Not observed | Not observed |
| Closest goal distance, m | 11.129198 | 13.736755 |
| Final goal distance, m | 22.840990 | 13.782937 |
| C24 contact reports | 619 | 0 |
| Non-floor collider-pair spans | 9 | 18 |
| First non-floor contact, sim s | 14.300001 | 14.516667 |

First non-floor contact: `/Root/Geo_M2_TrimDoor5/Geo_M2_TrimDoor/Geo_M2_TrimDoor`, `/World/Robots/Nova_Carter/wheel_right/Cylinder_05`.
There are 7461 non-floor contact reports and
18 spans through goal entry/end.
Maximum recorded impulse norm 35.270399 N s;
minimum separation -0.000037 m.
Reports include persistent callbacks, not independent impacts. Centre distance
is not surface clearance; passage is the conservative reference-tangent plane
crossing beyond wheelchair half-length plus robot radius.

Saved command context around first contact (or closest approach if none):

| Observation | Observation sim s | Application sim s | v m/s | w rad/s |
| --- | ---: | ---: | ---: | ---: |
| C24 | 12.783334 | 12.883334 | 1.000000 | 0.216891 |
| C25 | 13.283334 | 13.383334 | 1.000000 | 0.205065 |
| C26 | 13.783334 | 13.883334 | 1.000000 | 0.149423 |
| C27 | 14.283334 | 14.383334 | 1.000000 | 0.113671 |
| C28 | 14.783334 | 14.883334 | 1.000000 | -0.062352 |
| C29 | 15.283334 | 15.383334 | 1.000000 | -0.274058 |

Immediately before first contact, observations C24-C27 still produce 1.0 m/s
with yaw commands 0.216891/0.205065/0.149423/0.113671 rad/s. C28-C29 continue
at 1.0 m/s after impact. The recorded route approached the corridor boundary
without sufficient clearance; this does not identify the model's internal cause
or prove that the speed change alone caused the outcome.

Recorded target/applied forward commands max out at **1.000000 m/s**;
all 72 commands were independently recomputed from saved native actions using
the new config. Largest measured body-forward speed is
1.003504 m/s; physical velocity and commanded speed
are distinct. 2238 robot rows match 4476 wheelchair rows,
through 37.283335 simulation s. Both static world matrices remained
unchanged at every tick and are identical to prior preflight/model asset records.
These are verified static USD transforms, not dynamic PhysX body measurements.
C10/C24 are visible in 12/13
observations respectively.

The console has 21833 PhysX NULL-material warnings
(`PxShape::getMaterialFromInternalFaceIndex`, `NpShape.cpp:598`) and
0 Python tracebacks. Post-contact material effects are unquantified.
No host or asset changes were made to suppress warnings.

One asynchronous trial per speed does not isolate causality or establish a
success rate. Slower motion changes later RGB, physical timing and reasoning
availability; this is not replay of identical action outputs. Goal entry would
not establish stopping in the fixed 72-call loop. Runtime PASS is separate
from navigation success, general avoidance competence or handoff inconsistency.

## Gates, provenance and validation

Fresh model-free preflight and model run each passed **30 checks**, including
source/robot/camera/wheels, floor grounding, exact XY, static matrices, no people,
no overlaps, local bypass, completed RGB without physics advancement, speed
limit and call budget. Actual body/wheel images were reviewed. Local bypass
probes never command the robot. Preflight asset dictionaries match the preceding
1.2 m/s run exactly. No implementation failure or model retry occurred this task.

Branch `TIC-VLA-TEST`, base SHA `3c0d2c52e82d017955f02c5462969e4df1cb8239`;
upstream `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Exact sources, dirty/diff
identity, commands and receipts are in the audit. TEST remains Git-free.
Final focused local commit is recorded separately in `git-finalization.json`.

Simulation: Isaac6.0.1-rc.7, Python3.12.13, NumPy2.3.1, RealTimePathTracing.
Separate inference-only Isaac5.0.0-rc.45 Python3.11.13, torch2.7.0+cu128,
torchvision0.22.0+cu128, NumPy1.26.0, transformers4.57.6; strict checkpoint
load, no second SimulationApp. GPU RTX5060Ti / driver595.99.02. Base weight SHA256
`a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`;
checkpoint SHA256
`376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`.
No installation, training, model download or host environment change.

Original Hospital origin, world XYZ metres, +Z up, yaw CCW from +X; robot
quaternion wxyz. Wheelchair local points map to world with USD row-vector
matrices. Native 30x2 forward/left offsets contain no yaw. 60 Hz physics,
one outstanding IPC future, OLD retention and FRESH observation anchoring
unchanged. Observation, inference-ready, reasoning-generation/poll and command
application times remain distinct; simulation time differs from wall time.
This is the modified Isaac6 harness, not unchanged official Isaac5 reproduction
or real-world evidence.

15 existing focused tests PASS; actual old/new config equality and new-module
symbol tables checked. Source review proves only the two scene constants changed.
Independent evidence audit: 876 raw hashes, 167
source hashes, 383 reference hashes, 72 finite 30x2 native
records, exact instruction, 1920x1080 RGB decoding, 144 reconstructed per-object
semantic masks, 72-frame GIF (37.28 s), and speed/paired-state checks.
1045 protected previous source/result files unchanged.
All raw and derived artifacts are separate and uncommitted.

## Commands and artifacts

Executed commands (historical IDs already used; fresh IDs required for reruns):

```bash
cd /home/gpuadmin/Workspace/TIC-VLA-TEST
python3 -B tools/two_wheelchairs_v1/runner.py prepare --run-id two-wheelchairs-v1-20261014T013912Z-47b6f5-preflight --session /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v1-20261014T013912Z-47b6f5-preflight-setup --preflight
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v1-20261014T013912Z-47b6f5-preflight-setup/run.sh
python3 -B tools/two_wheelchairs_v1/runner.py prepare --run-id two-wheelchairs-v1-20261014T013912Z-47b6f5 --session /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v1-20261014T013912Z-47b6f5-setup --prerequisite outputs/two-wheelchairs-v1-20261014T013912Z-47b6f5-preflight
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v1-20261014T013912Z-47b6f5-setup/run.sh
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v1-20261014T013912Z-47b6f5-setup/analyze.sh
```

Each launch has a 900-wall-second bound plus 30-second termination grace.

- [RGB GIF](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v1-20261014T013912Z-47b6f5/derived/two-wheelchairs-v1/rgb-observations.gif)
- [Trajectory](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v1-20261014T013912Z-47b6f5/derived/two-wheelchairs-v1/trajectory.png)
- [RGB overview](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v1-20261014T013912Z-47b6f5/derived/two-wheelchairs-v1/rgb-contact-sheet.png)
- [Contact context](/home/gpuadmin/Workspace/TIC-VLA/outputs/two-wheelchairs-v1-preparation-20261014T013912Z-47b6f5/encounter-rgb.png)
- [Result JSON](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v1-20261014T013912Z-47b6f5/derived/two-wheelchairs-v1/summary.json)
- [Console log](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v1-20261014T013912Z-47b6f5-setup/console.log)
- [Audit](/home/gpuadmin/Workspace/TIC-VLA/outputs/two-wheelchairs-v1-preparation-20261014T013912Z-47b6f5)
