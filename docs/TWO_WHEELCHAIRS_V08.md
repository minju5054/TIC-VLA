# C10/C24 wheelchairs at a 0.8 m/s controller limit

2026-10-14 (Asia/Seoul). **Runtime PASS; PASS_THIS_SCENARIO.** One requested trial,
90 predictions, no navigation retry or post-result tuning.

C10 and C24 were passed without contact at 10.450001 and 23.433335 simulation s.
The robot first entered the goal's strict 2 m radius at 32.583335 s, reaching a
closest distance of 0.347256 m. It continued driving and ended 8.984652 m away.
Thus the analyzer's PASS means contact-free passage and goal-region entry;
the instruction's requirement to stop in front of the bed was not achieved.

## Configuration and scope

Maximum forward command **1.0 -> 0.8 m/s**. Prediction budget **72 -> 90**
compensates the 20% lower cap, preserving the nominal maximum distance budget
at target 2 Hz. Maximum angular command remains **1.2 rad/s**. Lookahead 1 m,
angular gain .8, bearing filter .35, epsilon .001, controller formula and Nova
wheel adapter are unchanged. Lower v also reduces the v-dependent curvature
term of w; angular control was not independently retuned.

All other actual configuration equals the preceding 1.0 m/s run. No people;
two stationary original Hospital wheelchairs at exact unobstructed baseline
`nova-e16-hospital-lights-baseline-20261010-01` requests 10/24:

- C10 centre [6.824887275695801, 7.096357822418213, 0] m.
- C24 centre [-1.7567275762557983, 10.988419532775879, 0] m.

Visible mesh world-bounds centres use the exact reference XY; lowest mesh Z=0.
Long axes follow the adjacent-request tangents. Original Hospital
`/Root/SM_WheelChair_01a_37` geometry/materials/collision meshes are copied to
`/World/ExactWheelchairC10` and `C24`, without scaling or dynamic bodies.
Original props remain. Static transforms and full asset records match the
previous run exactly; both body/wheel grounding images were reviewed.

Original instruction, verified in all 90 requests:

> Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.

New additive `tools/two_wheelchairs_v08/scene.py` differs from frozen v1 only
in `MAX_V=0.8` and `PREDICTIONS=90`. Runner/analyzer changes are namespace,
speed descriptions and call-count labels. No upstream source, previous bundle,
model, asset, package, host environment or installation changed.

## Measured outcome

| Wheelchair | Passage sim s | Contact reports | First contact sim s | Min centre distance m | Displacement m |
| --- | ---: | ---: | ---: | ---: | ---: |
| C10 | 10.450001 | 0 | Not observed | 1.110344 | 0.0 |
| C24 | 23.433335 | 0 | Not observed | 0.809862 | 0.0 |

| Measurement | Previous 1.0 m/s | Current 0.8 m/s |
| --- | ---: | ---: |
| Predictions | 72 | 90 |
| Goal entry (<2 m), sim s | Not observed | 32.583335 |
| Closest goal distance m | 13.736755 | 0.347256 |
| Final goal distance m | 13.782937 | 8.984652 |
| Non-floor collider-pair spans | 18 | 1 |
| First non-floor contact sim s | 14.516667 | 40.433335 |

There are 65 non-floor contact reports,
0 spans through goal entry/end.
Maximum impulse norm 0.000000 N s,
minimum separation 0.005329 m.
Persistent callbacks are included; report/span counts are not independent impacts.
All 65 reports occur after goal entry, starting at 40.433335 s against corridor
trim. Their maximum impulse is zero and minimum separation is positive
(5.329 mm), so these proximity/contact callbacks do not establish a physical
impact or penetration. Before first goal entry, no non-floor contact was reported.
Non-floor contact targets:

- `/Root/Geo_M2_TrimStrate6_57/Geo_M2_TrimStrate/Geo_M2_TrimStrate`

Saved command context (First reported nonfloor contact at 40.433 s):

| Observation | Observation sim s | Application sim s | v m/s | w rad/s |
| --- | ---: | ---: | ---: | ---: |
| C76 | 38.933335 | 39.033335 | 0.800000 | -0.064348 |
| C77 | 39.433335 | 39.533335 | 0.800000 | -0.049804 |
| C78 | 39.933335 | 40.016669 | 0.800000 | -0.059551 |
| C79 | 40.433335 | 40.533335 | 0.800000 | -0.071226 |
| C80 | 40.933335 | 41.033335 | 0.800000 | -0.052450 |
| C81 | 41.433335 | 41.533335 | 0.800000 | -0.053179 |

Maximum applied forward command **0.800000 m/s**; maximum
measured body-forward velocity 0.802116 m/s.
2787 robot rows and 5574 paired wheelchair rows,
through 46.433336 simulation s. Both static world matrices stayed
unchanged at every tick. These are static USD geometry measurements, not dynamic
PhysX poses. C10/C24 visible in 15/24 observations.
Centre distances are not surface clearance. Passage uses the reference-tangent
plane beyond the conservative wheelchair half-length plus robot radius.

Console: 0 PhysX NULL-material lookup warnings
(`PxShape::getMaterialFromInternalFaceIndex`, `NpShape.cpp:598`),
0 Python tracebacks. Material effects on contact are unquantified.

One asynchronous trial per speed does not isolate speed causality or establish a
success rate. Later RGB, physical timing and reasoning availability change.
Runtime PASS is distinct from navigation success and handoff consistency.
Goal-region entry does not prove stopping: the fixed 90-call loop continues.

## Provenance and validation

Branch `TIC-VLA-TEST`, base local SHA
`e8d1f0b7821af96eab2c236803e34e0166c30219`; upstream SHA
`9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Dirty status, diff identity,
new-source SHA256, exact commands and receipts are saved in the audit; final
focused local commit in `git-finalization.json`. TEST is Git-free.

Simulation: Isaac6.0.1-rc.7, Python3.12.13, NumPy2.3.1, RealTimePathTracing.
Separate model-only Isaac5.0.0-rc.45, Python3.11.13, torch2.7.0+cu128,
torchvision0.22.0+cu128, NumPy1.26.0, transformers4.57.6; no second SimulationApp.
GPU RTX5060Ti, driver595.99.02. Base weight SHA256
`a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`;
checkpoint SHA256
`376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`.

Original Hospital origin, XYZ metres, +Z up; yaw CCW from +X, robot quaternion
wxyz. USD row-vector matrices map wheelchair local points to world. Native
30x2 forward/left offsets contain no yaw. 60 Hz physics, target 2 Hz requests,
one outstanding IPC future, OLD retention and FRESH observation anchoring remain.
Observation, inference start/ready, reasoning generation/ready/poll and command
application retain distinct timestamps; simulation and wall clocks differ.
Modified Isaac6 harness evidence, not unchanged Isaac5 reproduction or real-world evidence.

Model-free preflight and actual pre-model gate each passed **30 checks**.
15 existing focused tests plus actual-config equality, exact scene diff and
new-module symbol-table checks PASS. Independent audit verified
1092 raw / 167 source / 383 reference hashes,
90 finite 30x2 native arrays and independently recomputed commands, exact
instructions, 90 RGB files at 1920x1080, 180 reconstructed masks,
90-frame GIF (46.43 s), speed cap and paired static states.
1204 protected previous files are unchanged.
Raw and derived artifacts remain separate and uncommitted.

## Executed commands and artifacts

Historical IDs below are already used; reruns require fresh IDs.

```bash
cd /home/gpuadmin/Workspace/TIC-VLA-TEST
python3 -B tools/two_wheelchairs_v08/runner.py prepare --run-id two-wheelchairs-v08-20261014T015420Z-a56068-preflight --session /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v08-20261014T015420Z-a56068-preflight-setup --preflight
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v08-20261014T015420Z-a56068-preflight-setup/run.sh
python3 -B tools/two_wheelchairs_v08/runner.py prepare --run-id two-wheelchairs-v08-20261014T015420Z-a56068 --session /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v08-20261014T015420Z-a56068-setup --prerequisite outputs/two-wheelchairs-v08-20261014T015420Z-a56068-preflight
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v08-20261014T015420Z-a56068-setup/run.sh
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v08-20261014T015420Z-a56068-setup/analyze.sh
```

Launch bound: 900 wall seconds plus 30-second termination grace.

- [RGB GIF](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v08-20261014T015420Z-a56068/derived/two-wheelchairs-v08/rgb-observations.gif)
- [Trajectory separating goal entry from continued motion](/home/gpuadmin/Workspace/TIC-VLA/outputs/two-wheelchairs-v08-preparation-20261014T015420Z-a56068/trajectory-goal-phases.png)
- [Original analyzer trajectory](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v08-20261014T015420Z-a56068/derived/two-wheelchairs-v08/trajectory.png)
- [RGB overview](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v08-20261014T015420Z-a56068/derived/two-wheelchairs-v08/rgb-contact-sheet.png)
- [Contact context](/home/gpuadmin/Workspace/TIC-VLA/outputs/two-wheelchairs-v08-preparation-20261014T015420Z-a56068/encounter-rgb.png)
- [Result JSON](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v08-20261014T015420Z-a56068/derived/two-wheelchairs-v08/summary.json)
- [Console](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/two-wheelchairs-v08-20261014T015420Z-a56068-setup/console.log)
- [Audit](/home/gpuadmin/Workspace/TIC-VLA/outputs/two-wheelchairs-v08-preparation-20261014T015420Z-a56068)
