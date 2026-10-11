# Hospital E16: one no-cart max_v=1.0 diagnostic

**STRUCTURAL CONTACT BEFORE CORNER PASSAGE.** The robot reached the corner
entry region but did not complete the intended left turn. It continued north,
then turned east and contacted another original Hospital trim at 21.016668 s.
There was **no TrimDoor5 contact and no passage past TrimDoor5**. This is a
navigation failure in this run, not a successful low-speed corner traversal
and not an observation-window shortage. The original goal region was not reached.

## Scope, runtime and provenance

Exactly one real model experiment, 48 predictions, no retries or budget extension:

- Reference: `outputs/nova-e16-hospital-lights-baseline-20261010-01`.
- New: `outputs/nova-e16-no-cart-speed10-20261011-01`.
- Context only: `outputs/nova-e16-c26-cart-speed12-20261011-01`.
- Setup: `outputs/e16-no-cart-speed10-setup-20261011-01`.
- Frozen authorization/config: `outputs/e16-no-cart-speed10-freeze-20261011-01`.
- Final saved-only analysis: `outputs/e16-no-cart-speed10-analysis-20261011-02`.
  Analysis -01 is preserved; -02 fixes one clipped observation label and adds
  existing-viewer data exports without changing selection or measured results.

Starting clean local/fetched origin `hardcase-probe`:
`cafc9316837a3e1fdb8213d5518a3d2bdbfca1d6`; fetched upstream/main:
`9fa6f8b66b9e121d5df5df071297bba8e5353ebb`.
The execution receipt binds 109 Python/shell files and 354 source files,
including the exact user instruction, weights, baseline, startup evidence and
analysis rules. Runtime metadata records dirty status/diff identity and these
file hashes because the new runner was not yet committed during execution.

The **entire recursive primary config diff** is exactly:

```json
[{"path":"controller.max_v","old":1.5,"new":1.0}]
```

Start `(7.38,1.49,0.11)`, yaw pi/2, goal `(-12.23,10.19,0.01)`, original
instruction, seed 36, Nova Carter, Hawk RGB, fixture lighting, 60 Hz physics,
2 Hz target inference, 48 requests, lookahead 1.0 m, max_w 1.2 rad/s,
angular gain 0.8, filter gain 0.35 and epsilon 0.001 are unchanged. So are
native processing, model/history/KV mechanisms and continuous OLD retention.
The controller contains `w = clip(0.5*v*curvature + angular_gain*yaw_filter)`;
changing max_v can change steering, not just execution duration.

The additive `research/hospital_no_cart_speed.py` uses the unchanged original
`hospital_lights_run.verify`, original simulation and continuous loop, with
`actor_factory=None`. Neither `stationary_cart` nor any custom actor key is
present. Before inference and after the run, `/World/E16Cart` is absent and
the measured original Hospital collider inventory matches the reference:
`c6a68de8362e31f0255088ce24824510d8466dcc1d033c2c5a7c396649fd8e3b`.
All 2,161 original collider records and original Hospital props are preserved.
The complete lighting/renderer/source manifest and simulation/controller
runtime manifests are equal between the two primary runs.

Current simulator-only startup PASS (9.082 wall s), then one 90.307 wall s
closed-loop process. Isaac **6.0.1** sim, Python 3.12.13/NumPy 2.3.1;
Isaac **5.0.0 Python only** model service, no SimulationApp, Python 3.11.13,
torch 2.7.0+cu128, torchvision 0.22.0+cu128, NumPy 1.26.0,
transformers 4.57.6. Strict checkpoint PASS. Base SHA256
`a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`;
checkpoint SHA256
`376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`.
Both match all three run manifests. No upstream/default/controller/environment
changes, training, model substitutions or Isaac 5 RTX startup.

The host had already changed between the historical reference and current
experiments: NVIDIA 595.84 -> 595.99.02, kernel 7.0.0-34 -> 7.0.0-38.
The agent changed neither. Although speed is the sole config difference,
host state, asynchronous reasoning/history and stochastic execution are
additional limits on attributing a single-run outcome to speed alone.

## Measured execution and classification

| Measurement | No-cart 1.5 reference | No-cart 1.0 new | Cart 1.2 context |
|---|---:|---:|---:|
| Finite real predictions | 48 | 48 | 48 |
| End simulation time [s] | 25.300001 | 25.350001 | 25.433335 |
| Pre-contact measured XY speed median [m/s] | 1.500322 | 1.000245 | 1.200275 |
| Corner entry [s] | 7.616667 | 10.183334 | 8.900000 |
| Original corner passage completion [s] | 10.883334 | none | none |
| TrimDoor5 region passage [s] | 11.433334 | none | none |
| First nonfloor contact [s] | none | 21.016668 | 11.833334 |
| TrimDoor5 contact | NO | NO | YES |
| Nonfloor contact spans | 0 | 11 | 30 |
| Travelled distance [m] | 34.928365 | 19.507722 | 12.985102 |
| Final XY [m] | (-20.345802,10.179739) | (10.822265,17.876226) | (3.233176,12.226213) |
| Goal region entered | YES,18.566668s | NO | NO |

New evidence comprises 1,522 measured ticks and all 48 finite native `(30,2)`
chunks. Body-forward pre-contact median/mean/max is
0.999945/0.903976/1.001915 m/s; world-XY speed maximum is 1.002754 m/s.
Small physical tracking overshoot is distinct from the commanded 1.0 cap.
Mean includes stationary initialization. Final yaw is -1.849812 rad
(-105.986398 deg); minimum/final goal distances are 17.693515/24.299897 m.

Corner entry is the first measured centre inside the original gate Y band
`[9.457836,11.741243]`, east of X=2.552104 and west of start-X plus the
original 0.606983 m enclosing robot radius. New entry pose is
`(5.747856,9.461455,1.792025 rad)` at tick 611.
Passage retains the original crossing observation **plus next observation**
rule, evaluated through that next application. No threshold is retuned.
The original two-consecutive-interval actual-left-motion criterion passes
for the reference and does not pass for the new run. The new robot does
initially yaw left somewhat (observation yaw peaks near 110.02 deg before
contact); this is not zero steering, but not a completed intended left turn.
It does not enter the westward hallway.

TrimDoor5 passage requires the entire original robot disk west of the trim's
minimum world X, centre inside the same corridor band, after corner entry:
centre X <= 0.297028 m. New run never satisfies this. Absence of its contact
cannot count as successful passage.

First contact at tick 1261, **21.0166677628 s**:

- Environment prim:
  `/Root/Geo_W_TrimStrateBase1_453/Geo_W_TrimStrateBase1_2/Geo_W_TrimStrateBase1_2`.
- Robot prim: `/World/Robots/Nova_Carter/wheel_right/Cylinder_04`.
- Exact measured robot pose: `(10.885658,17.889475,0.731402 rad)`.
- Reported contact XYZ points [m]: `(11.132980,17.794174,0.140047)`,
  `(11.132773,17.793791,0.151989)`, `(11.133616,17.793449,0.156710)`.

Contacts use the unchanged PhysX environment-contact rule `abs(normal.z)<0.5`
and FOUND/PERSIST/LOST spans: measured vertical/steep surface evidence, not
a complete collision classifier. Contact time/point are simulation evidence;
the reported robot pose matches its exact saved tick, without interpolation.

Goal-region entry uses the original source `_is_goal_reached` XY distance
strictly below 2 m. The reference enters then continues because this research
harness always uses 48 predictions; this is not an official benchmark success
evaluation. The new run fails both corner passage and goal-region entry.

## OLD/FRESH and common physical progress

World axes/units remain original Hospital XYZ metres, +Z up. Robot body
coordinates are +forward/+left, yaw radians CCW about world +Z from +X;
saved robot quaternion ordering is w,x,y,z. Every native point is projected
as `world = observation_xy + R(observation_yaw) @ [forward,left]`, using its
own **observation** pose, never ready or application pose. Native tensors
contain no yaw or timestamps.

Waypoint timing is assigned from the source-confirmed 0.1..3.0 s target
convention. For non-grid continuous observation gaps, the existing analyzer
interpolates both curves at common absolute nominal times (FRESH knots plus
common endpoints), without extrapolation or spatial matching. Lateral
revision is `(FRESH_world-OLD_world)` rotated into FRESH body axes.
RAW seams compare actual switch position/preceding measured path direction
with the observation-anchored FRESH curve at elapsed nominal time.
The diagnostic's tau=0 observation anchor and segment tangent are derived;
neither is appended to native data or called native TIC-VLA yaw.

Primary pre-contact pairs require both FRESH observation and application
strictly before first nonfloor contact. There are 47 reference, 39 new and
20 contextual pairs; respectively 0/8/27 later pairs are excluded. All-route
median temporal RMSE is 0.436717/0.231029/0.286075 m, but these distributions
cover different physical regions and do not measure improvement.

The selection rule was frozen before execution: first pre-contact FRESH
observation within the fixed corner entry region. This selects reference
C13->C14 and new C18->C19, **not same-index matching**:

| Metric | Reference C13->14 | New C18->19 |
|---|---:|---:|
| FRESH observation [s] | 7.800000 | 10.350001 |
| FRESH world XY [m] | (5.330324,9.670548) | (5.711205,9.624272) |
| Aligned RMSE [m] | 0.594075 | 0.199556 |
| Mean/max abs lateral revision [m] | 0.075022/0.182351 | 0.063934/0.202308 |
| Mean signed lateral revision [m] | +0.072063 | -0.063721 |
| FRESH lookahead native forward/left [m] | 1.187500/0.202148 | 1.171875/0.023438 |
| FRESH applied w [rad/s] | 0.314227 | 0.012883 |
| RAW position seam [m] | 0.012019 | 0.023018 |
| Executed-to-FRESH derived tangent seam [deg] | 0.434175 | 1.187658 |

New selected pair has delta-v=0, delta-w=+0.005597 rad/s; action call
60.059 ms, observation-to-switch wall 531.997 ms, pending simulation
0.100000 s, measured transport 0.100101 m. Both curves point mainly north
there; the small revision and small steering command do not supply a
completed left-corner route. This is an observation, not a diagnosis of
model attention, latency causality or a need for reconciliation.

`physical_location_bins.csv` compares fixed 1 m approach-Y and corridor-X
bins, with counts and pre-contact metrics. In common approach Y=[9,10) m,
reference/new each have two pairs: median RMSE 0.499473/0.234925 m,
maximum lateral revision 0.292575/0.385602 m. Similar progress is not
identical pose/view/history. Whole execution is also plotted against simulation
time. For nonbootstrap requests, median action-call latency is
53.820/55.882 ms, wall observation-to-switch 528.589/515.556 ms,
pending simulation 0.1/0.1 s for reference/new. New bootstrap takes
1.818415 action-wall s and 1.850000 pending-simulation s. Continuous OLD
retention and all clocks/commands/pending transport are preserved and exported.

The 1.2 cart run contacted TrimDoor5 at 11.833334 s before cart approach;
the 1.0 no-cart run takes a different path and contacts another structure
later. **Both speed and added-cart presence change in this secondary
comparison.** Neither removal of the cart nor delayed contact establishes
a causal avoidance benefit.

## Outputs, saved replay and validation

Final analysis directory above contains PNG/PDF pairs:
`world_xy`, `corner_trimdoor5_zoom`, `linear_speed_time`,
`yaw_angular_velocity_time`, `observation_positions`, `precontact_old_fresh`.
Actual execution is distinct from predicted curves; all 48 observation
markers are shown for each primary run. Original collider bounds are light
context, TrimDoor5 is magenta and the original exit gate is green.
`trajectory_coordinates.json` contains dense measured XY and exact C poses;
per-run observations/pairs/turn-interval CSV+JSON contain timing, commands,
lookahead, yaw, pending motion and raw seams. `*_details.json` preserves full
world-projected curves and aligned points. Raw dense velocities/quaternions
remain directly referenced in each source `robot_state.csv`.

Existing no-human `view_nova_occlusion.py` is supported unchanged through
the analyzer's `saved_replay/` exports. GUI self-test reached GUI_READY in
`outputs/e16-no-cart-speed10-gui-20261011-01`: exact recorded pose replay,
baseline human absent, fixed own-observation guides, cameras, archived RGB,
first-contact jump, timeline stopped, zero model calls/physics re-execution.
Its generic stationary-human/reveal wording is inactive context; this run
has no human/cart. The inspected scene uses the original saved lights.
Reopen using a fresh output ID (the command below generates one):

```bash
cd /home/gpuadmin/Workspace/TIC-VLA && bash scripts/isaac6_python.sh research/view_nova_occlusion.py --run-dir outputs/nova-e16-no-cart-speed10-20261011-01 --analysis-dir outputs/e16-no-cart-speed10-analysis-20261011-02/saved_replay --output-dir "outputs/e16-no-cart-speed10-gui-$(date +%Y%m%d-%H%M%S)" --self-test
```

Reproduce only the saved-data analysis with a new output directory:

```bash
python3 research/analyze_no_cart_speed.py --reference-run outputs/nova-e16-hospital-lights-baseline-20261010-01 --run-dir outputs/nova-e16-no-cart-speed10-20261011-01 --context-run outputs/nova-e16-c26-cart-speed12-20261011-01 --output-dir outputs/e16-no-cart-speed10-analysis-NEW
```

Validation PASS: 22 focused tests (config/actor guards, outcome boundaries,
fixed corner bounds, original Hospital-light and controller checks). Independent
saved checks cover 144 commands/filter states/lookahead indices, 144 exact
observation coordinates, 141 temporal-overlap RMSE/lateral pairs and 144
archived finite 1920x1080 RGB images. All six final PDFs were rendered and
visually reviewed, including label correction; GUI overview was reviewed.
920 protected prior files, 109 executed code files, 354 frozen source files
and every analysis input hash remain unchanged. Evidence:
`outputs/e16-no-cart-speed10-setup-20261011-01/final-verification.json`.
No generated outputs or checkpoints are committed. Focused commit and normal
origin/hardcase-probe push; final SHA/status are recorded in setup Git
finalization and the task response, avoiding a self-referential commit hash.

## Next single experiment, not executed

Repeat the exact **no-cart max_v=1.5, 48-prediction baseline once on the
current host**, with identical scene/instruction/controller/model settings.
This checks whether the successful historical left turn reproduces on the
current runtime before attributing its difference from 1.0 to speed. Do not
simultaneously change duration, angular gains or obstacle placement.
The present single-run failure supports no general navigation/avoidance
improvement claim and validates no graph optimization, correspondence,
reconciliation or latency-induced hard-case mechanism.
