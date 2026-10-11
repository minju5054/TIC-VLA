# Hospital E16: current-host no-cart 1.5 m/s reproduction

**Case 1 — the intended left turn, west-hallway entry and goal-region entry
reproduced without measured nonfloor contact.** The new path is not identical:
it turns wider toward the north wall and satisfies the original corner gate
later. Execution and corner validation pass; this single run does not establish
deterministic behavior or a causal speed effect. Goal-region entry is recorded
separately from stopping: the fixed 48-prediction harness continues past the
goal and does not validate the instruction's final stopping behavior.

## Evidence and exact configuration

| Role | Saved run |
|---|---|
| A: historical no-cart 1.5, primary reference | `outputs/nova-e16-hospital-lights-baseline-20261010-01` |
| B: NEW no-cart 1.5, exactly one new experiment | `outputs/nova-e16-no-cart-15-repro-20261011-01` |
| C: recent no-cart 1.0, secondary comparison | `outputs/nova-e16-no-cart-speed10-20261011-01` |

Started clean local/fetched origin `hardcase-probe` at
`cb18a80affa8c026b3a101ce1fa3059309fa8aac`; fetched upstream/main remains
`9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Setup evidence is under
`outputs/e16-no-cart-15-repro-setup-20261011-01`. A new authorization and
receipt live in `outputs/e16-no-cart-15-repro-freeze-20261011-01`.
Neither the previous receipt nor a previous run ID was reused.

The new config is copied directly from **A's saved metadata**, not C.
Full recursive A/B config diff is **`[]`**, including exact instruction:

> Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.

Exact instruction equality also passes for all 144 saved requests across
A/B/C. Hospital original USD, Nova Carter, start `(7.38,1.49,0.11)` with
yaw pi/2, goal `(-12.23,10.19,0.01)`, front Hawk 1920x1080 RGB, fixture
lighting, seed 36, 60 Hz physics/control, 2 Hz target inference and 48
predictions are unchanged. max_v=1.5, max_w=1.2, lookahead=1.0,
angular gain=0.8, filter gain=0.35, epsilon=0.001. Native processing,
history/KV mechanisms and continuous OLD retention are unchanged.

`research/hospital_no_cart_reproduction.py` is an additive exact-config
wrapper. It retains original `hospital_lights_run.verify`, simulation,
model loop and controller, and reuses the previous read-only stage audit.
Original sources and defaults are not edited. No custom actor config,
actor factory or actor instance; `/World/E16Cart` is absent before inference
and after execution. All 2,161 original Hospital colliders are preserved.
Complete source/fixture-light/renderer manifests match A exactly, as do
simulation/controller runtime manifests. Collider inventory SHA256:
`c6a68de8362e31f0255088ce24824510d8466dcc1d033c2c5a7c396649fd8e3b`.

The receipt binds 111 executed Python/shell files and 355 source files,
including A, weights, user authorization, startup result and pre-execution
analysis rules. Run metadata records code hashes, local/upstream SHA, dirty
status/diff identity and argv; the additive wrapper was uncommitted during
execution, so source identity is not represented by Git HEAD alone.

## Runtime versus navigation

Current NVIDIA **595.99.02**, RTX 5060 Ti, kernel 7.0.0-38; nvidia-smi PASS.
One simulator-only Isaac 6 startup: ready/shutdown/exit0 PASS, **8.278 wall s**.
Then exactly one real closed-loop run: **99.275 wall s**, no timeout/retry,
**48 finite native `(30,2)` predictions**, 1,519 measured ticks,
0..25.300001 simulation s. Continuous OLD retention PASS. No prediction
extension, tuning, DynaNav port, Isaac 5 RTX startup or environment change.

Simulation: Isaac 6.0.1, Python 3.12.13, NumPy 2.3.1. Inference:
Isaac 5 Python 3.11.13 **without SimulationApp**, torch 2.7.0+cu128,
torchvision 0.22.0+cu128, NumPy 1.26.0, transformers 4.57.6.
Original TIC-VLA strict checkpoint loading PASS. Model source unchanged;
all three manifests match base/checkpoint SHA256:

```text
base:       a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484
checkpoint: 376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036
```

A was recorded on driver 595.84/kernel 7.0.0-34; B and C share the current
595.99.02/7.0.0-38 host. Nothing was upgraded by this task. A/B test whether
the behavior reproduces in the current environment, not identical-host
determinism. B/C are a descriptive same-host speed comparison, one sample
per current condition, with different observation/inference histories.

Executed command, for provenance (the consumed run ID cannot be rerun):

```bash
bash scripts/isaac6_python.sh research/hospital_no_cart_reproduction.py --phase run --receipt-dir outputs/e16-no-cart-15-repro-freeze-20261011-01
```

## Navigation measurements, original gates unchanged

| Measurement | A historical 1.5 | B NEW 1.5 | C recent 1.0 |
|---|---:|---:|---:|
| Real predictions | 48 | 48 | 48 |
| Pre-contact XY speed median [m/s] | 1.500322 | 1.500313 | 1.000245 |
| Corner entry [s] | 7.616667 | 7.533334 | 10.183334 |
| First qualifying left-motion interval [s] | 7.8..8.3 | 8.8..9.3 | none |
| Original gate crossing / next C | C19/C20 | C23/C24 | none |
| Gate completion at next application [s] | 10.883334 | 12.883334 | none |
| Intended left turn completed | YES | YES | NO |
| West hallway entered | YES | YES | NO |
| TrimDoor5 region passed [s] | 11.433334 | 12.266667 | none |
| TrimDoor5 contact | NO | NO | NO |
| Any measured nonfloor contact | none | none | 21.016668s, other trim |
| Goal region first entry [s] | 18.566668 | 19.050001 | none |
| Travel distance [m] | 34.928365 | 34.927983 | 19.507722 |
| Final XY [m] | (-20.345802,10.179739) | (-19.651411,10.086081) | (10.822265,17.876226) |
| Final yaw [deg] | 167.465632 | 172.843261 | -105.986398 |

B body-forward velocity median/mean/max is 1.500078/1.378885/1.503243 m/s;
XY speed max=1.503492 m/s. Mean includes stationary initialization. Measured
absolute yaw-rate median/max is 0.077007/0.429888 rad/s. Final yaw is
3.016684 rad. Commands, dense world/body velocities and measured yaw are
retained separately; physical tracking overshoot is not a changed speed cap.

The original exit gate requires an observation at centre X<=2.552104 m,
Y in `[9.457836,11.741243]`, plus one subsequent observation/application.
The original actual-left criterion requires two consecutive clean measured
intervals with positive yaw and episode-frame left-dominant displacement.
B qualifies with fresh IDs 17..25 (and later 39..40); no criterion is relaxed.
All 48 B brightness checks pass, with no degraded RGB or camera-proximity hits.

B's wider turn reaches Y=12.055090 m versus A's 11.226068 m in the corner.
It initially travels north of the gate's allowable centre band, then enters
it at C23 `(−0.431619,11.720494)`; C24 application completes the gate.
TrimDoor5 passage uses dense measured ticks, requiring the entire original
0.606983 m enclosing disk west of trim-min-X and centre within the corridor:
X<=0.297028 m. B satisfies that at 12.266667 s, before the later gate
observation/application. These differently sampled events are not conflicting
definitions or retuned thresholds. Conservative disk bands are not exact
contact geometry; no B PhysX nonfloor contact was measured.

The unchanged nonfloor rule is environment contact with `abs(normal.z)<0.5`,
retained through CONTACT_LOST: vertical/steep-surface evidence, not a complete
collision classifier. Goal-region entry uses original source XY distance
strictly below 2 m. B minimum goal distance is 0.543070 m, but its final
distance is 7.422139 m after continuing past the goal for 48 predictions.
Do not call this successful stopping or official benchmark navigation success.

## Model, controller and reasoning differences

World XYZ metres, +Z up; body XY is forward/left, yaw CCW from world +X,
saved quaternion ordering w,x,y,z. Every predicted world curve is
`observation_xy + R(observation_yaw) @ native_point`. No ready/application
re-anchoring. Native output contains neither yaw nor timestamps.

OLD/FRESH uses source-assigned nominal 0.1..3.0 s target times, common
absolute-time interpolation at FRESH knots plus overlap endpoints, without
extrapolation/spatial correspondence. Signed lateral revisions use FRESH
body axes. RAW seams use switch-time measured position and derived segment
tangents, not native yaw. The diagnostic tau=0 observation anchor is separate
from the original 30 native points. A/B have 47 pre-contact pairs each;
C has 39, with 8 post-contact pairs excluded from primary interpretation.

First FRESH observation inside the original physical corner region selects
A C14, B C14 and C C19. Equal A/B request IDs are a consequence of selection,
not assumed correspondence; poses/yaws differ. The paired plots show each
run's preceding OLD and current FRESH:

| Metric at first corner observation | A C14 | B C14 | C C19 |
|---|---:|---:|---:|
| World XY [m] | (5.330324,9.670548) | (5.466828,9.805817) | (5.711205,9.624272) |
| Observation time [s] | 7.800000 | 7.800000 | 10.350001 |
| Robot yaw [deg] | 134.125 | 124.878 | 102.719 |
| Native lookahead index, zero-based | 11 | 15 | 14 |
| Native lookahead forward/left [m] | 1.187500/0.202148 | 1.140625/0.083496 | 1.171875/0.023438 |
| Applied v/w | 1.5/0.314227 | 1.5/0.156746 | 1.0/0.012883 |
| Within-run OLD/FRESH RMSE [m] | 0.594075 | 0.387017 | 0.199556 |
| Mean/max abs lateral revision [m] | 0.075022/0.182351 | 0.072639/0.251874 | 0.063934/0.202308 |
| RAW position / derived tangent seam | 0.012019m/0.434175deg | 0.020028m/0.556934deg | 0.023018m/1.187658deg |

All-route pre-contact RMSE medians A/B/C are 0.436717/0.425018/0.231029 m;
these are within-run descriptive distributions over different state histories,
not navigation quality or reconciliation requirements.

**Earliest numerical difference is already C1**, at the exactly equal saved
observation pose. A/B archived RGB hashes, native tensor hashes and reasoning
text hashes differ; recorded PNG mean absolute channel difference is
2.232349 on the 0..255 scale, with 86.6798% of pixels differing somewhere.
C1 applied w is 0.050420 versus 0.045322 rad/s (delta=-0.005098), although
both switch at 1.800000 simulation s. This is not evidence for a meaningful
causal onset or identical model input: PNG differences are not preprocessed
model tensor hashes, and complete KV tensors were not archived.

Before execution, the descriptive divergence inspection rule was fixed:
first of two adjacent common 1 m approach-Y bins in which median applied-w
sign **and** median native-lookahead-left sign disagree A/B, using original
numerical sign tolerances. **No such persistent direction disagreement occurs
for A/B.** Magnitudes and path curvature still differ. The divergence figure
explicitly reports this and shows first-corner RGB/native/lookahead/command
inspection, rather than inventing an earliest significant failure point.
All bins, including empty counts, are exported. For common physical progress:

| Approach world Y bin | Median w A | Median w B | Median w C |
|---|---:|---:|---:|
| [7,8) m | +0.193321 | +0.152197 | -0.036171 |
| [8,9) m | +0.213668 | +0.144015 | -0.087998 |
| [9,10) m | +0.299670 | +0.176850 | +0.010084 |

B and C show different steering directions in Y=7..9 m. B later increases
left steering and completes the turn; C continues north/east. That observation
does not isolate speed as the cause: v also enters the unchanged angular
formula, while native output, views, history and reasoning state differ.

All saved reasoning text, generation-start/poll steps, KV-available flags,
delay references, sampled image IDs, robot-state inputs and prior-motion
prompts are exported in `*_reasoning_native.json`. At corner A/B both report
start/poll step468, sampled IDs `[2,8,14]`, delay2.0s, but the reasoning output
and predicted geometry differ. A/B each record24 generation starts and23
distinct completion-poll steps; matching counts/steps do not establish equal
KV contents. Exact worker-ready timestamps and durable consumed KV IDs are
unavailable and are not inferred. Generated text describes having completed
the left turn even at intermediate poses; it is not ground-truth localization
or a causal explanation of the commands.

Nonbootstrap median action-wall latency A/B is 53.820/55.324 ms,
observation-to-switch wall528.589/535.174 ms, pending simulation0.1/0.1s,
pending XY transport0.150096/0.150099m. These clocks remain separate. No
latency cause is established by similar or differing timing values.

## Artifacts, replay and validation

Final directory: `outputs/e16-no-cart-15-repro-analysis-20261011-02`.
Analysis -01 is preserved; -02 adds measured bootstrap differences and common
native-plot axes without changing frozen selection/navigation rules. Seven
PNG/PDF pairs: `world_xy`, `corner_zoom`, `yaw_velocity_time`,
`angular_command_response`, `corner_observations`, `corner_old_fresh`,
`divergence_diagnostics`. Figures separate executed paths and predictions;
all observation markers in the displayed region are retained. Dense coordinates,
48-observation/47-pair CSV+JSON per run, original native chunks, lookahead,
reasoning, temporal/world derived curves, bins, summary and hashes are exported.

Saved-only analysis reproduction, fresh destination required:

```bash
python3 research/analyze_no_cart_reproduction.py --historical-run outputs/nova-e16-hospital-lights-baseline-20261010-01 --run-dir outputs/nova-e16-no-cart-15-repro-20261011-01 --speed10-run outputs/nova-e16-no-cart-speed10-20261011-01 --output-dir outputs/e16-no-cart-15-repro-analysis-NEW
```

Existing unchanged saved viewer reached GUI_READY in
`outputs/e16-no-cart-15-repro-gui-20261011-01`, paused around C13->C14.
Recorded-pose, camera, original RGB and guide checks pass; human absent,
timeline stopped, zero model calls and no physics re-execution. Its generic
human/reveal labels are inactive context. Fresh-directory replay command:

```bash
cd /home/gpuadmin/Workspace/TIC-VLA && bash scripts/isaac6_python.sh research/view_nova_occlusion.py --run-dir outputs/nova-e16-no-cart-15-repro-20261011-01 --analysis-dir outputs/e16-no-cart-15-repro-analysis-20261011-02/saved_replay --output-dir "outputs/e16-no-cart-15-repro-gui-$(date +%Y%m%d-%H%M%S)" --self-test
```

**26 focused tests PASS**: exact-config/no-actor guards, original controller,
lighting and corner tests, plus physical-progress selection and persistent-sign
rule tests. Independent validation recomputes144 commands/filter states/lookahead
indices,144 exact observation poses,141 nominal-overlap RMSE/lateral pairs and
144 finite archived RGB images. All seven final PDFs were rendered and reviewed;
GUI overview reviewed. Original111 executed files,355 frozen sources,1,320
protected prior files and all analysis inputs remain hash-identical. Validation:
`outputs/e16-no-cart-15-repro-setup-20261011-01/final-verification.json`.

Only additive code/config/tests, this report and an append-only work-log entry
are committed. Normal push to origin/hardcase-probe; final commit/push receipt
is stored in setup Git finalization and the task response. No raw outputs,
figures or weights are committed, and no history is rewritten.

## Next single task, not executed

Repeat the unchanged **no-cart 1.0 m/s, 48-prediction condition once on the
same current host**, preserving all other settings, to check whether its
failure repeats before tuning anything. A/B now support reproducible corner
behavior at1.5 in two observed runs across hosts, not deterministic success.
B/C alone cannot prove speed caused the different route. No additional run,
graph optimization, correspondence or reconciliation is implemented here.
