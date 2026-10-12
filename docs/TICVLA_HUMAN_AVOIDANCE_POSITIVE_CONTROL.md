# Paper-like TIC-VLA human avoidance positive control

This is an **adapted Isaac 6 prerequisite experiment**, not an official
DynaNav reproduction or a reconciliation experiment. One no-human E16 run is
authorized; one stationary-human run is conditional on its frozen gate.
No controller, seed, checkpoint, lighting, placement or cadence tuning is
permitted after observing predictions.

## Repository-confirmed facts

Starting clean local and freshly fetched `origin/hardcase-probe`:
`5f6a6f2da63cbc143028182fda3de696635c4f5c`. Fetched `upstream/main`:
`9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. `DynaNav/` remains byte-identical
to upstream. Original Isaac 5 RTX startup remains blocked; it was not retried.
Neither Isaac installation, CUDA, driver, system Python nor LightNav was changed.

The unchanged official Nova behavior calls action inference synchronously
inside `on_update`, every third sensor update under its 30-frame/s convention.
The comments describing an asynchronous action queue are not executable behavior.
Background reasoning/KV generation remains asynchronous inside `DynaNav/ticvla.py`.
Source inspection does not establish the unexecuted official benchmark's actual
physics or wall-time cadence.

The executable E16 timeout is **85 s**: YAML timeout 85 plus
`TIMEOUT_EXTENSION_SECONDS = 0`. The old benchmark comment saying “add 50” is
stale. Similarly, 150 stuck frames mean **5 s**, not the behavior comment's
3 s; backup is 300 frames / **10 s**. `_inference_timeout_sec=3.0` is declared
but never used. The implemented stale-command check is the source's recovery
generation-start heuristic, not a new three-second age cutoff.

### Checkpoint provenance

**PUBLIC_CHECKPOINT_ONLY_RL_PARITY_UNRESOLVED.** The freshly inspected artifact
has Lightning epoch 9, global step 39310, version 2.5.2, 1,324 state-dict entries,
`val_ade` callbacks, action horizon 30 and action layers 3. The top-level README
describes a pretrained release trained on the released reasoning dataset.
`rl/README.md` describes separate PPO training and an export step that overlays
RL action-expert weights onto a supervised template. The local checkpoint has
no immutable linkage to the paper's final RL evaluation. Its metadata does not
prove that RL never occurred, either. No training or weight replacement occurred.

| Artifact | Identity |
|---|---|
| Public checkpoint | `handsomeYun/TIC-VLA`, revision `671de608f39ccae41b66cbb24ab923c0275f8fb4`, `TIC-VLA-model.ckpt` |
| Checkpoint SHA256 | `376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036` |
| Base | `OpenGVLab/InternVL3-1B`, revision `4415a3b810e636d11dfa86b0e9ba40bb00535aa8` |
| Base weights SHA256 | `a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484` |

Audit evidence: `outputs/paperlike-human-control-20261012-01/` contains
`checkpoint-metadata.json`, `checkpoint-parity-audit.json`, `host.json` and
`protocol.json`. Metadata inspection used Isaac 5 Python without SimulationApp
or inference; weights are independently hash-checked again by the strict-loading
model service before navigation predictions.

## Adaptations and remaining differences

| Aspect | Official DynaNav | This positive-control path | Parity status |
|---|---|---|---|
| Simulator | Isaac 5 benchmark/behavior lifecycle | Isaac 6.0.1 World physics/render; separate Isaac 5 model Python | Adaptation; official startup blocked |
| Model/checkpoint | Original TICVLA, strict Nova loading | Same unmodified `DynaNav/ticvla.py`, released base/checkpoint, strict load | Model path preserved; final RL weight parity unresolved |
| Action cadence | Inline call every third update, nominal 10 Hz | Explicit 10 Hz simulation-time sensor clock, synchronous IPC | Nominal schedule restored; measured wall rate reported separately |
| Physics/control | Benchmark frame convention 30 Hz; actual timing unvalidated | 60 Hz PhysX, 30 Hz control, physics frozen during capture/action | Explicit adaptation; no continuous-latency experiment |
| Reasoning | Original asynchronous KV, bootstrap wait, completion poll | Original model worker unchanged; separate process and capture overhead | Mechanism preserved; reasoning lifecycle differs with wall scheduling |
| Image history | JPEG current and 90/60/30 sensor entries back | Same JPEG conversion/entry selection; backup images buffered without action calls | Selection parity; all originals archived instead of deleting old buffers |
| Robot state/delay | Body velocity with explicit lateral sign flip; old reasoning-start displacement; frame/30 delay | Same sign/rotation convention, 30 Hz model-step IDs, measured sim-time delay | Preserved construction; tiny PhysX time rounding and wrapper pose clock differ |
| Motion history | Previous one-second displacements in previous body frame | Same text and quaternion transform at 60 Hz measured one-second boundaries | Nominal convention preserved; control-clock phase explicit |
| Lookahead | 1 m polyline arc length; yaw filter .35; curvature feed-forward; max 1.5/1.2 | Same equations; exact source-block numerical tests | Numerical parity; native points unchanged |
| Slew/recovery | Signed delta slew; 5 s stuck / 10 s backup; wait/fresh priority | Additive state machine, same ordering/constants; float32 final command | Adapted/tested; no full behavior-script parity claim |
| Stale actions | During recovery, check last tracked reasoning start before appending new start | Same heuristic and command rejection; log acceptance and mode | Heuristic preserved; not an exact KV-to-action identity claim |
| Wheel geometry | Controller radius .152 m / base .413 m | **.152/.413 command conversion** on unchanged asset audited at **.14/.4132** | Source constants restored; physical geometry identity not claimed |
| Camera/lighting | Native Hawk/latest annotator; original Hospital lights | Same Hawk 1920×1080; explicit render completion; existing 153-fixture lights-on profile | Input/capture/lighting adaptation, no outcome tuning |
| Seed | Benchmark config 666 | Existing research baseline seed 36, frozen for both stages | Explicit deviation |
| Human system | E17 has ten people via legacy people/IRA integration | Conditional single stationary official-family character/rest pose plus .25 m capsule | Not an E17 reproduction |
| Collision metric | Human center-distance <.2 m; chassis sustained force criterion | Raw PhysX contacts plus Nova .606982542 m + human .25 m disk proxy | Different conservative research checker; not benchmark metric parity |
| Goal | Benchmark XY distance <1.5 m | User-requested <=1.5 m, terminate at first measured entry | Same radius; inclusive boundary and physics-tick sampling differ |
| Timeout | E16 85 s plus zero extension | 85 measured simulation seconds; no fixed-48 budget | Bound preserved, independently measured clock |

World coordinates are metres, +Z up. Robot planar yaw is CCW from world +X;
quaternions are **w,x,y,z**. Native `(30,2)` is **forward,left**, metres, all
points relative to the request's observation body origin. BF16 values are stored
losslessly as float32. No native yaw or time channel is added.

For plotting only, `p_world = xy_observation + R(yaw_observation) p_native`.
Application/ready poses are never substituted. Nominal +.1…3.0 s timing is
source-assigned, not a native timestamp channel; no temporal/spatial matching,
OLD/FRESH score, correction or graph computation is performed here.

Observation, action-start, service-ready, response-detected and application
events retain host monotonic/wall clocks separately from simulation time.
Exact reasoning-worker completion and a simultaneous ready-pose sample remain
unknown. Synchronous calls have zero measured simulation-time transport; this
does not imply zero wall latency or realtime 10 Hz.

## Frozen protocol and model-free gate

Versioned policy: `configs/research/paperlike_human_positive_control.json`.
Setup: **`outputs/paperlike-human-control-20261012-01`**. Expanded A/B configs,
source/code/weight hashes, initial dirty-state identity and single authorized
run IDs were written before the first prediction. The historical baseline and
manual-human evidence are protected by before/after hashes.

Stage A must enter the official-size goal region, complete the unchanged
spatial corner gate plus one subsequent observation, and have no nonfloor
contact, with valid data/logging. This is a deliberately stronger, preregistered
gate than substantial progress alone. It cannot be weakened after execution.

The model-free Stage A simulator/wheel/camera check passed: measured XY
displacement **.363178389 m** and positive yaw **.241073267 rad**, no nonfloor
contact or logger errors. Initial native Hawk RGB was visually inspected.
This short command check is not a second no-human policy/navigation trial.

Stage B's frozen human is `(1.9812549352645874, 11.580365562438965, 0)` m,
yaw `-.11106107381082708` rad: historical baseline C19.X, C19.Y+.40 world metres,
wrap(C19.yaw+pi). No reveal requirement or placement search is used. If gated in,
it must pass fresh floor/static-overlap/initial-clearance/bypass/relevance and
actual initial RGB checks. The original `VISIBLE` helper label denotes the same
CLEAR criterion: fraction >=.002 and bbox >=20×80; zero pixels means HIDDEN.

The local avoidance checker uses all measured path segments, not just predicted
endpoints. It requires positive whole-path combined-radius clearance, no human
contact, crossing the downstream plane at one combined radius past the human,
and **at least 1 m net downstream progress** after crossing. This progress bound
was fixed before either run. Full navigation separately requires goal entry
and no human contact; structural contacts remain separately reported.

## Stage A no-human result

**PAPERLIKE_NO_HUMAN_CONTROL_FAILED.** The sole run
`outputs/paperlike-human-control-20261012-01-stage-a` ended at its frozen
85 s timeout. It did not complete the original left-turn/corner gate or reach
the goal region. No parameter change, retry or human run followed.

| Measurement | Saved result |
|---|---:|
| Technical runtime / strict checkpoint load | PASS / PASS |
| Unique finite native `(30,2)` predictions | 536 |
| Sensor captures / backup-only buffered captures | 850 / 314 |
| Measured robot rows / physics steps | 5,101 / 5,100 |
| Final simulation time | 85.000004433 s |
| Goal entry / corner-plus-one gate | No / No |
| Minimum / final goal distance | 17.330459 / 19.120078 m |
| Final world XY | (5.302155, 17.818954) m |
| Executed path length | 49.687873 m |
| Nonfloor structural contact spans | 71 |
| First nonfloor contact | 16.683334203 s |

The first contact was the right wheel's `Cylinder_04` against
`/Root/Geo_W_TrimStrateBase1_453/Geo_W_TrimStrateBase1_2/Geo_W_TrimStrateBase1_2`.
Contact spans are collider-pair FOUND-to-LOST records, not 71 independent
collision episodes. The positive-control gate was already disqualified then;
the same frozen Stage A continued to its predeclared bound to retain recovery
behavior. Backup occurred at 24.9–34.9, 44.233333–54.233333,
65.366667–75.366667 and 83.533333–85 s, the last truncated by timeout.
WAIT_FRESH followed the three completed backups for .1, .166667 and .133333 s.

### Measured cadence and clocks

| Measurement | Result |
|---|---:|
| Requested sensor/action cadence | 10 Hz simulation-time target |
| All 849 sensor-to-sensor intervals | .100000005215 s |
| 535 action-observation gaps, median / mean / max | .100000005 / .156074774 / 10.100000527 sim s |
| Action gaps above target | 3, each spanning a completed backup |
| Action call, median / mean / max | .054517 / .085850 / 1.994759 wall s |
| Nonbootstrap action call, median / max | .054513 / .530725 wall s |
| Observation-to-application, median / max | .353110 / 10.478919 wall s |
| Nonbootstrap observation-to-application, median / max | .352465 / .869837 wall s |
| Action calls / observation-to-application above .1 wall s | 76/536 / 536/536 |
| Outstanding maximum / queued / catch-up bursts | 1 / 0 / 0 |
| Updates / whole 85 s simulation window | **6.305882 Hz** |
| Updates / whole 534.235342 s loop wall window | **1.003303 Hz** |

The evaluator also reports first-to-last-application frequencies: 535 intervals
over 83.500004 sim s = **6.407185 Hz**, and over the corresponding application
wall window = **1.019974 Hz**. These exclude the terminal backup and have different
denominators from whole-run counts above. Action-enabled simulation intervals
meet the nominal 10 Hz schedule; backup intentionally suppresses action calls
while retaining sensor history. No claim of uninterrupted or realtime 10 Hz is
supported. C1's observation precedes cold inference-service initialization, so
its 10.478919 s application latency includes that startup and the 1.994759 s
bootstrap action call. Simulation time stays frozen across each action/capture.

### RGB quality context

Only **218/850** saved captures meet the existing descriptive lights-on brightness
profile; **632** do not. The first failure is C105/sensor105 at 10.400000542 s
(p95 93.0642, below the existing 100 threshold). There are **69** existing
`degraded_near_black` flags (p95 <=20), and zero logged camera-proximity overlaps.
No capture triggers the hard shape/blank-image stop. The initial view, C71 and
C167 were visually reviewed: C167, just before the first contact, is a dark
close-wall view with little context. Thus technical logging PASS is not a claim
that every image remains useful for navigation. Zero camera-overlap reports do
not rule out visual occlusion; these observations do not establish the failure's
cause. Neither lighting nor pose was adjusted after this result.

## Stage B human result

**NOT_EXECUTED_STAGE_A_GATE_FAILED.** The observed Stage A structural contact
at 16.683334203 s already violates the frozen contact-free prerequisite. No
human preflight/navigation/model call, changed placement or retry follows.
Human-conditioned predictions, actual human bypass, human clearance and full
human-navigation outcomes are **unmeasured**, not zero or a human-avoidance FAIL.

## Positive evidence

The model-free source-constant wheel command path, original camera and contact
logger are executable in the adapted Isaac 6 environment. Source-extracted
lookahead/slew tests establish numerical behavior, not navigation success.
All 536 saved target commands and every applied slew update agree with the
independently extracted upstream equations (maximum numerical error zero).

## Negative evidence

The one restored-controller condition did not meet the contact-free Stage A
prerequisite. A pipeline PASS must not be reported as a navigation PASS.

## Implementation limitations

This is one scene and one realization, with adapted simulator, illumination,
control lifecycle and image timing. Synchronous freezing is an explicit choice
motivated by the source's inline call; unexecuted Isaac 5 physics timing is not
experimentally reproduced. The loaded physical wheel dimensions differ from
the official command constants. The source freshness heuristic is preserved
despite its imperfect association with the actually consumed KV state.

The existing raw nonfloor classifier uses contact normals with |normal.z|<.5,
retaining spans until LOST; it is not a complete mesh penetration or sustained
force oracle. A capsule/disk human proxy would not represent full human shape,
articulated motion or DynaNav's crowd interactions. Stage B code remains gated
and has no new runtime validation when Stage A fails.

Only one existing research file changes in the final commit:
`inference_service.py` gains an opt-in paperlike image-buffer/history/30 Hz step
branch, preserving default behavior. New controller/runner/evaluator/replay code
is additive. No upstream source changes are made. No
trajectory smoothing, blending, artificial delay, correspondence, graph or
reconciliation is implemented or evaluated.

## Insufficient evidence

This gated outcome cannot establish human-avoidance capability, official
TIC-VLA performance, checkpoint equivalence, or why the no-human path differs
from historical 2 Hz direct-controller runs. Several execution conditions
changed together deliberately; one run cannot isolate causation.

## Validation, artifacts and commands

**75 focused/regression tests PASS**: 15 paperlike, 27 Hospital, 11 Nova,
7 continuous and 15 occlusion. Source-extracted lookahead, slew, history and
wheel conversion references, recovery boundaries, scheduler no-catch-up,
observation-frame projection, goal/timeout, immutable output and geometry/bypass
checks are covered. The 15th test protects identical saved-evaluation receipt
reuse against changed decisions; it was added after navigation for the
saved-only analyzer. No synthetic test is navigation evidence.

Independent saved checks cover all 536 native arrays, exact measured observation
poses, clocks, 850 sensor IDs/history selections, targets, applied commands and
wheel conversion. A scalar world-transform oracle differs by at most
**3.55e-15 m**. All **504 protected historical files**, both weight artifacts,
configs and source-run files remain unchanged. Immediately after navigation,
all **139 frozen source hashes** matched. `independent-validation.json` records
that result. Subsequent saved-only analyzer changes permit identical receipt
reuse, split simulation gaps from wall latency in the figure and give raw-run
hashes a distinct key from code-provenance hashes; metrics and
gate classification remain exactly equal. A redundant module-level PIL import
was removed (the original function already imports it). Both pre-edit files
are retained under `executed-analysis-source/`; this is not a rerun or a change
to the executed controller/configuration. `final-verification.json` records the
final source identity and replay/analysis checks.

Runtime versions are recorded in the source `simulation-runtime.json` and
`inference-runtime.json`: Isaac 6 `6.0.1-rc.7+release.42383.32955d8d.gl`, Python
3.12.13, NumPy 2.3.1; separate Isaac 5 Python 3.11.13, torch 2.7.0+cu128,
torchvision 0.22.0+cu128, NumPy 1.26.0, no model-process SimulationApp. Host:
RTX 5060 Ti, NVIDIA 595.99.02, kernel 7.0.0-38-generic. Frozen expanded config
SHA256 A: `f8c0dfba409c3bb01053e21f9eedfec4bb8094c86876f91a3a1bec9206a5be41`;
B: `23600118e081dd307e78c91741b28c7965cff0295e3458ba46fcc4190ce570ce`.
Starting dirty-state/diff and individual code identities are in `protocol.json`
and source `metadata.json`; final Git identity is in `git-finalization.json`.

All paths below are relative to the repository. Final saved-only figures are
under `outputs/paperlike-human-control-20261012-01/analysis-A-final/figures/`:

- `world_trajectory.png`: actual Stage A path, goal and structural context.
- `interaction_zoom.png`: original corner, actual path and fixed time-spaced
  native predictions at their own observation anchors. No human is inserted.
- `commands_cadence.png`: target/applied commands, application times, simulation
  gaps and separate wall latencies; sparse samples are not joined across backup.
- `clearance_goal.png`: measured goal distance; human clearance marked undefined.

The exact `plot_coordinates.json`, `prediction_metrics.csv/json`, immutable
derived world arrays and `validation.json` back these figures. All four original
figures and the revised cadence figure were visually inspected; the other three
review figures are byte-identical. The earlier `analysis-A/` and
`analysis-A-review/` outputs are retained.
The replay contains all 536 model observations and original archived PNG links,
with exact measured poses/native world curves; its Play advances one observation
every 200 ms and is **not wall-time or simulation-time playback**. It is a local
XY/RGB browser viewer, not an Isaac re-execution.

Inspect and replay existing data (no model/physics):

```bash
cd /home/gpuadmin/Workspace/TIC-VLA
cat outputs/paperlike-human-control-20261012-01/evaluation-A.json
xdg-open /home/gpuadmin/Workspace/TIC-VLA/outputs/paperlike-human-control-20261012-01/replay-A/index.html
```

Recompute derived analysis into a **new** directory, preserving the gate receipt:

```bash
python3 research/paperlike_analyze.py --setup-dir outputs/paperlike-human-control-20261012-01 --stage A --output-dir outputs/paperlike-human-control-20261012-01/analysis-A-independent
```

Exact execution commands already used (audit only; no additional trial authorized):

```bash
python3 research/paperlike_run.py --phase freeze --output-dir outputs/paperlike-human-control-20261012-01
bash scripts/isaac6_python.sh research/paperlike_run.py --phase preflight --stage A --output-dir outputs/paperlike-human-control-20261012-01
bash scripts/isaac6_python.sh research/paperlike_run.py --phase run --stage A --output-dir outputs/paperlike-human-control-20261012-01
python3 research/paperlike_analyze.py --setup-dir outputs/paperlike-human-control-20261012-01 --stage A --output-dir outputs/paperlike-human-control-20261012-01/analysis-A
python3 research/paperlike_analyze.py --setup-dir outputs/paperlike-human-control-20261012-01 --stage A --output-dir outputs/paperlike-human-control-20261012-01/analysis-A-review
python3 research/paperlike_analyze.py --setup-dir outputs/paperlike-human-control-20261012-01 --stage A --output-dir outputs/paperlike-human-control-20261012-01/analysis-A-final
python3 research/paperlike_replay.py --run-dir outputs/paperlike-human-control-20261012-01-stage-a --output-dir outputs/paperlike-human-control-20261012-01/replay-A
python3 -m unittest tests.test_paperlike -v
python3 -m unittest discover -s tests -p 'test_hospital*.py' -v
python3 -m unittest discover -s tests -p 'test_nova.py' -v
python3 -m unittest discover -s tests -p 'test_continuous.py' -v
python3 -m unittest discover -s tests -p 'test_nova_occlusion.py' -v
```

## Exact next uncertainty

Whether the public checkpoint is identical to the paper's final RL-evaluation
checkpoint remains unresolved and must not be assumed when interpreting this
adapted negative control result.
