# Exact C10 human and C24 stationary wheelchair

2026-10-14 (Asia/Seoul). The user requested replacing the previous C24 person
with a wheelchair. C10 remains the same grounded stationary person. **One
48-prediction run completed; navigation failed at the wheelchair.** No model
retry, altered instruction, controller change, or obstacle relocation occurred.

Run in the Git-free modified copy:
`TIC-VLA-TEST/outputs/c10-human-c24-wheelchair-20261013T150824Z-fc819a`.
Original workspace source base `173af194dac13b668d13938063ef1fb79b6c509a`,
branch `hardcase-probe`; upstream `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`.
The preparation audit is
`outputs/c24-wheelchair-preparation-20261013T150824Z-fc819a/`.
It stores dirty/source identities, exact commands, failed preflight preservation,
visual review, contact diagnosis and final verification.

## Instruction actually passed to all 48 requests

> Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.

This is the original episode16 instruction, identical across all48 saved
request JSON files. No person/wheelchair avoidance wording was added. This
fact alone does not establish why avoidance failed.

## Placement and checks

- C10 person: original unobstructed reference XY
  `(6.824887275695801,7.096357822418213)` m. Its configuration is identical to
  the preceding two-person run, including the visible-sole grounding correction.
- C24 wheelchair: original reference XY
  `(-1.7567275762557983,10.988419532775879)` m. Actual mesh world-footprint
  centre error is2.22e-16 m (floating-point rounding), with no intentional offset.
- Original Hospital source `/Root/SM_WheelChair_01a_37`, asset
  `Props/SM_WheelChair_01a.usd`, copied under `/World/ExactWheelchairC24`.
  Mesh, material bindings and the original static collision geometry are
  retained. The original prop is untouched. No rigid body, invisible substitute
  collider, scaling or runtime pose command was added.
- Longest local horizontal mesh axis follows the original C23→C25 travel
  tangent. Original Hospital origin, metres, +Z up; yawCCW from+X. Visible mesh
  points are mapped by USD row-vector local-to-world matrices. Translation
  centres the rotated world footprint at exact C24 and sets minimum meshZ=0.
  Actual floor ray Z=2.086e-7 m. Body/wheel closeups were visually reviewed.
- First model-free preflight failed because the reused human mesh reader did
  not traverse USD instance proxies. The new wheelchair-specific reader does.
  The failed17 files and old source bytes remain intact. Fresh `-preflight-p2`
  passed32 placement/geometry/render gates and a separate rendered wheelchair
  label test before inference. At least one local bypass probe passed.
- C24 human absent; C10 human and wheelchair remain fixed. The human uses
  measured PhysX position/quaternionxyzw. The wheelchair uses verified unchanged
  static USD collider transforms, not a dynamic-body pose measurement.
- Single lowercase semantic classes separately identify C10 and the wheelchair.
  Full semantic ID arrays/maps are archived with masks. Both rendered labels
  were checked before inference, addressing the previous empty per-person-mask
  limitation without changing the old experiment or claiming its cause proven.

## Measured results

| Measure | C10 person | C24 wheelchair |
| --- | ---: | ---: |
| First visible / clearly visible observation | 1 / 1 | 11 / 12 |
| Position change | 0 m | 0 m |
| Rotation / static-transform change | 0 | 0 |
| Passed in original travel direction | 6.333334 simulation s | Never |
| Contact reports | 0 | 801 |
| First contact | None | 11.950001 simulation s |

There are3 non-floor collider-pair contact spans, all against the wheelchair;
801 records include persistent contact callbacks, not801 distinct impacts.
Maximum impulse norm66.340750 N·s and minimum separation-0.001961228 m
confirm actual contact. Closest goal distance11.035151 m, final11.035152 m;
strict2 m goal entry never occurs. The robot remains at the wheelchair through
the end of the25.316668 simulation-second window. Saved RGB shows the chair
ahead before impact and its wheel immediately beside the camera afterward.

Runtime PASS and navigation FAIL are separate. This is one stochastic,
asynchronous modified Isaac6 scenario, not general avoidance qualification,
an isolated semantic-object causal test, unchanged Isaac5 reproduction or
real-world evidence. Historical path overlays are descriptive. Footprint
rectangles and passage planes are conservative geometric summaries, not a
full-body continuous collision proof. The fixed48 loop does not stop on goal.

## Preserved execution and artifacts

Isaac6.0.1-rc.7/Python3.12.13/NumPy2.3.1/RealTimePathTracing simulation;
separate Isaac5 Python3.11.13/torch2.7.0+cu128/torchvision0.22.0+cu128/
NumPy1.26.0/transformers4.57.6 model-only inference, without a second
SimulationApp. Model/checkpoint, robot/camera, controller, lighting and
scene/instruction configurations match the preceding run. Native30x2
forward/left offsets retain their observation anchor; no yaw is invented.
60 Hz physics, target2 Hz inference, one IPC future and OLD command retention
are unchanged. Observation, inference start/ready, reasoning start/poll and
command-application clocks remain separate.

Base weight SHA256
`a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`;
checkpoint SHA256
`376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`.
Each setup receipt includes source/reference/runtime/weight identities and the
completed preflight manifest. No upstream source, package, driver, runtime or
environment changes; no push. The user's `run_test.sh` remains untouched.

Executed commands (already consumed; use new IDs for a future run):

```bash
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c10-human-c24-wheelchair-20261013T150824Z-fc819a-setup/run.sh
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c10-human-c24-wheelchair-20261013T150824Z-fc819a-setup/analyze.sh
```

`derived/c24-wheelchair/` contains summary, contacts, trajectory PNG, RGB sheet
and48-observation GIF held by simulation timestamps. The GIF is not an
intermediate-frame video. All raw records remain separate and unchanged.

Validation:19 focused CPU tests PASS; actual-artifact audit PASS with48 finite
native arrays/RGB frames,1,520 matched robot/human/wheelchair samples,96 masks
recomputed from archived ID arrays/maps,48-frame25.31 s GIF decoded. Human is
visible in9 observations; wheelchair in38.589 raw,168 source and383 reference
files hash-verified;772 prior source/output files and17 failed-preflight files
unchanged. Body/wheel images, trajectory and final RGB sheet visually reviewed.
