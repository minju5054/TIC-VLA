# One stationary human at exact C24, original instruction

2026-10-14 (Asia/Seoul). One requested run completed: **runtime PASS, navigation FAIL**.
The robot contacted the Hospital door trim/wall before reaching C24. Zero human
contact therefore does not demonstrate successful human avoidance.

Exact instruction, verified in all 48 actual requests:

> Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.

## Scene and execution

C10 has no added actor. One stationary grounded human is placed at the original
unobstructed C24 XY **(-1.7567275762557983, 10.988419532775879) m**. No added
wheelchair, cart, old H05 human, or C10 human exists. Original Hospital props
are retained. The coordinate source is
`nova-e16-hospital-lights-baseline-20261010-01`, request 24, not the later H05 path.

New additive `tools/c24_human_only/` reuses the existing exact human authoring,
mesh skinning/grounding, controller and inference loop. Compared with the earlier
original-instruction two-human config, the only configuration change is removal
of C10 from `fixed_humans`. No official instruction projection is needed.
The sole human uses the single `occluded_human` semantic class. Full semantic
ID arrays/maps are archived to independently audit visibility masks.

A fresh simulator-only preflight and the actual run each pass all 24 placement,
grounding, source, camera, wheel, lighting, actor-exclusivity and local-bypass
checks before inference. Frozen diagnostic body/feet views were reviewed.
Human measured XY error is 0 m; visual sole heights are 0 and 0.000016165672 m,
with raycast floor at 0.000000208616 m (maximum error about 0.016 mm).
The unchanged capsule has radius 0.25 m, cylinder height 1.2 m and centre Z 0.85 m.
Visual Z correction is -0.157419658065 m; no XY offset or size change.

## Measured result

| Measurement | Result |
| --- | ---: |
| Predictions | 48 |
| Dense robot/human paired rows | 1,524 each |
| Simulation duration | 25.383335 s |
| Human position / quaternion component change | 0 / 0 |
| First visible / clearly visible human observation | C10 / C11 |
| Human contact reports | 0 |
| Closest robot-to-human centre distance | 6.034731 m |
| Human passed | No |
| First structural contact | 9.600001 simulation s |
| Non-floor contact spans / reports | 127 / 4,024 |
| Goal region (<2 m) entered | No |
| Closest / final goal distance | 16.507836 / 16.514651 m |

C10/C11 in the visibility row are **this run's observation indices**, not a
second person at the historical C10 placement.
First contact is a right-wheel collider against `/Root/Geo_M2_TrimDoor5/...`;
later contacts include the nearby door wall and corner trim. Maximum recorded
impulse norm is 60.752973 N s and minimum separation is -0.013128 m. Contact
reports include persistent callbacks, not independent impacts.

C15, C16 and C17 retain 1.5 m/s with yaw commands -0.040201, -0.153543 and
-0.091080 rad/s. C17 applies at 9.466667 s, before contact. C18's stronger
left turn (+0.567242 rad/s) applies at 9.966667 s, after first contact;
slowing to 0.821559 m/s occurs at C19 application 10.466667 s. Saved RGB shows
approach to the wall followed by an obstructed view. These records establish
insufficient turning/clearance before collision, not the internal cause of the
model's action choice or a handoff defect. One asynchronous run cannot isolate
the effect of removing C10 or changing the obstacle type/instruction.

The console contains **9,309** PhysX material-lookup warnings,
`PxShape::getMaterialFromInternalFaceIndex received 0xFFFFffff as input - returning NULL`
(`NpShape.cpp:598`), with zero Python tracebacks. The effect on post-contact
material response is unquantified. No suppression patch, navigation retry,
controller tuning or additional model trial was performed.

## Sensor use in this run

TIC-VLA receives only the front Hawk **left RGB image** (1920x1080, target 2 Hz
in simulation time) as external sensor imagery, plus instruction/history and
robot state read directly from simulator APIs. It consumes no LiDAR scan,
point cloud, depth image, stereo disparity, IMU or semantic mask.
The asset audit includes four Hawk stereo pairs, four Owl cameras, front/rear
RPLIDAR_S2E and PandarXT_32_10hz prims plus a third-person camera; their presence
does not establish use by the current navigation pipeline.
PhysX pose/velocity and wheel-joint velocity reads are simulator truth, not
physical odometry or IMU measurements. Semantic masks, contact reports and
96 structural-distance rays are diagnostics only and never steer commands.

## Provenance and validation

Source base **93199e2dfc272439f8917bd78cd2d819dcfd2961**, branch `hardcase-probe`;
upstream **9fa6f8b66b9e121d5df5df071297bba8e5353ebb**. Git-free TEST receipts
bind the actual source/config/reference/model/runtime bytes; original dirty
status, diff identity and new-file hashes are in the preparation audit.

Simulation: Isaac 6.0.1-rc.7, Python 3.12.13, NumPy 2.3.1,
RealTimePathTracing. Separate model-only Isaac 5 Python: 3.11.13,
torch 2.7.0+cu128, torchvision 0.22.0+cu128, NumPy 1.26.0,
transformers 4.57.6; strict checkpoint load, no second SimulationApp.
Base weights SHA256 `a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`;
checkpoint SHA256 `376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`.
No upstream/model/controller/package/driver/Isaac-installation changes.

Original Hospital origin; world XYZ metres, +Z up, yaw CCW from +X. Robot
quaternion wxyz; human PhysX quaternion xyzw. Mesh/skeleton vertices use USD
local-to-world row matrices. Passage uses original adjacent-observation world-XY
tangent. Native 30x2 forward/left offsets retain the observation origin and
have no yaw. Physics 60 Hz, target inference 2 Hz, one IPC future, OLD command
retention and the fixed 48-prediction limit are unchanged. Observation,
inference ready, reasoning start/ready/poll, application and wall clocks remain
separate. This is a modified Isaac 6 simulation, not unchanged Isaac 5
reproduction, real-world evidence or reconciliation validation.

**30 focused CPU tests PASS**. Independent audit verifies 490 raw, 167 source,
376 reference hashes, all 48 exact instructions/finite native arrays/RGB,
1,524 matched actor/robot rows, 48 masks recomputed from raw IDs/maps (human
visible in 6 observations), and the 48-frame 25.38 s GIF. All 2,194 protected
prior source/output files are unchanged. Body/feet, trajectory, RGB overview
and contact-approach figures were visually reviewed. User `run_test.sh` untouched.

Run: `TIC-VLA-TEST/outputs/c24-human-only-20261013T154624Z-e26c7c`.
Preflight: same name with `-preflight` suffix.
Audit: `outputs/c24-human-only-preparation-20261013T154624Z-e26c7c`.
Derived summary, contacts, trajectory, RGB sheet and GIF: `derived/c24-human-only/`.
GIF uses actual captured observations held by simulation timestamps; no
intermediate video frames are synthesized. Raw and derived artifacts are separate.
The fixed loop does not implement stopping on goal-region entry.

Actual consumed commands (fresh IDs required for another run):

```bash
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-human-only-20261013T154624Z-e26c7c-preflight-setup/run.sh
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-human-only-20261013T154624Z-e26c7c-setup/run.sh
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-human-only-20261013T154624Z-e26c7c-setup/analyze.sh
```

Exact preparation commands are in `preparation.json` and `model-preparation.json`.
Local focused commit only; no push. Its SHA is recorded separately in the audit.
