# Bright Hospital baseline and occluded-human prerequisite

## Decision and scope

**INSUFFICIENT EVIDENCE.** Exactly one new bright no-human model run completed
48 finite predictions and a measured left turn. It **failed the preregistered
baseline contact gate**. Stationary-human placement preflight and the human
model run were therefore **not executed**, as required by the experiment's
stop condition. There is no primary reveal pair or stationary-human avoidance
claim. The baseline remains available in an actually validated Isaac GUI.

The preceding [moving-human experiment](NOVA_DYNAMIC_HARD_CASE.md) remains
unchanged: its C15/C16 RGB p95 was 19/12, and the initially visible human plus
different closed-loop histories confounded the intended intervention. Its
approximately .75 m pre-trigger pose divergence was not resolved by relabeling
another pair. This task attempted a different, explicitly bright observation
condition and retained the prerequisite failure as evidence.

Starting local and freshly fetched origin `hardcase-probe`:
`1592ef7cef19f1eeadda0b02be993b644b182bd1`.
Upstream `main`: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`.
There were no pre-existing working-tree edits. Exact final commit/push status
is recorded after commit in the ignored
`outputs/nova-occlusion-setup-20261010/git-finalization.txt`.

## Fixed platform and imaging calibration

The platform is **official DynaNav Hospital geometry/instruction + simulated
Nova Carter + research-fixed bright observation profile**, under Isaac Sim
6.0.1. It is not unchanged official DynaNav rendering or an Isaac 5 benchmark
reproduction. All pre-existing base config fields are unchanged: E16, verified
.14 m drive radius / .4132 m wheel base, actual front Hawk left camera at
1920×1080, 1 m lookahead, direct DifferentialController path, released strict
checkpoint, seed 36, 60 Hz continuous physics, 2 Hz target observations, one
outstanding inference, and OLD target retention while FRESH is pending.

The existing Isaac 5 Python model service starts no SimulationApp. Its archived
checkpoint SHA-256 is
`376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`;
base-model identity is
`a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`.
Runtime versions and strict-load evidence are in the new run's
`inference-runtime.json` and `simulation-runtime.json`.

Installed primary API source was inspected before calibration:
`omni.rtx.settings.core-0.7.1+f9bf0dda/.../widgets/post_widgets.py` defines the
tonemap exposure/ISO/f-number controls and histogram autoexposure. Clamp
tonemapping skips exposure; candidate profiles explicitly use Reinhard (op 2).
`isaacsim.core.utils.semantics.add_labels` and Replicator's uncolorized semantic
segmentation structure were also inspected. These source inspections do not
constitute human visibility validation.

`research/calibrate_nova_bright.py` renders all 48 recorded root/camera poses
from `outputs/nova-e16-closed-loop-20261010-01`, with physics stopped and zero
model calls. It preserves recorded XYZ and WXYZ quaternion, checks loaded
body/root translation consistency, retains actual asset camera transforms,
and waits for rendering with zero simulation delta. The old trajectory is
used only for imaging calibration, never as a human-placement source.

Criteria were fixed before calibration/model calls. On archived 8-bit RGB,
Rec.709 luminance is `.2126 R + .7152 G + .0722 B` (a pixel statistic, not a
linear radiometric measurement):

| Requirement | Frozen value |
| --- | ---: |
| Mean | 40–215 |
| Standard deviation | ≥15 |
| Median / p50 | ≥25 |
| p95 | ≥100 |
| Dark fraction, luminance <15 | ≤.35 |
| Saturated fraction, any RGB channel ≥250 | ≤.05 |

Mean, std, p05, p50, p95, p99, dark and saturation fractions are archived for
every calibration render and every actual baseline observation. Human
silhouette recognizability was not evaluated because the baseline gate
subsequently prevented human preflight.

| Model-free calibration | Result |
| --- | --- |
| `outputs/nova-bright-calibration-20261010-01`: fixed exposure +2/+4/+6/+8 stops | No candidate passes all 48 poses; even +8 has minimum mean 12.2113 and p95 27.988 |
| `outputs/nova-bright-calibration-20261010-02`: fixed world lights at intensity 1500/5000/15000, exposure +2 | 1500 fails C43–48; 5000 and 15000 pass all 48 |
| Selected 5000 profile | Minimum mean 83.6931, minimum p95 113.2046, maximum saturation .00166233 |

Selection is the lowest passing fixed-light intensity, followed by manual
review before model execution. The fallback receipt's generic selection text
still says "Lowest exposure"; its candidate type/list and the pre-model
`calibration-visual-review.json` explicitly resolve this to lowest passing
light intensity. The original receipt was preserved. All 48 selected renders
were reviewed as a contact sheet, with full C12/C16/C24 views: floor, stairs,
walls, corner, hallway and bed are distinguishable without severe saturation.

## Frozen bright profile

Versioned config: [`nova_e16_bright.yaml`](../configs/research/nova_e16_bright.yaml).
Its SHA-256 is
`f1abb9e2d24881d989c83f62149908138779317a8bf4954f24f42398e5bd9e37`.
Pre-model receipt:
`outputs/nova-occlusion-setup-20261010/baseline-freeze.json`, SHA-256
`729152c466903a022c29c64ae93b2213cd2057178ea436b6b36e833ed0162414`.
It binds 51 research code files, 17 calibration/source files, the config,
brightness/baseline rules, visual review and single authorized run ID.

The actual observation profile is fixed Reinhard, ISO 100, exposure
`.07999999821186066 s` (+2 stops from `.019999999552965164 s`), f-number 5,
responsivity `1.1026709079742432`, whitepoint `[1,1,1]`, color mode 0,
sRGB-to-gamma enabled, autoexposure disabled, motion blur disabled, dither 0.
It adds **16 world-stationary SphereLights**, each intensity 5000, radius .3 m
and Z=2.1 m. Every XYZ is explicit in the config. They are placed at fixed
approximately ≥2 m traveled-distance samples along the calibration route;
no light follows the robot, camera, request or prediction. No collider or
Hospital occluding geometry was added/changed. Actual renderer readback
exactly matches the frozen settings.

Calibration PASS at previously recorded poses does not guarantee clean images
along a newly sampled closed-loop trajectory. The following baseline gate
tests that separate requirement.

## New bright baseline and failed gate

Run: `outputs/nova-e16-bright-baseline-20261010-01`.
Saved analysis: `outputs/nova-e16-bright-baseline-analysis-20261010-01`.
Additional saved-only plots: `outputs/nova-bright-audit-20261010-02`.
Earlier audit `-01` is preserved.

The run's infrastructure summary is PASS: 48 unique finite `(30,2)` stored
chunks (native tensor `(1,30,2)`), forward/left meters in observation-body
frame, no native yaw, current original RGB and continuous OLD retention.
This does **not** imply the stricter research baseline gate passed.

Before inference, the gate was fixed to require the existing stable-left and
actual-turn criteria plus clean bright RGB and no relevant non-floor contact
from C1 through **one observation after the first two qualifying actual-turn
intervals**. The actual interval group is FRESH IDs `[17,18]`, so this rule
selects **C1–C19** without tuning to the outcome. All 19 brightness checks
pass, but **six non-floor contact spans begin within that gate window**.
The first begins at simulation `10.266667202 s`, just before C19 observation
`10.283333870 s`, between Nova's right wheel `Cylinder_05` and
`/Root/Geo_M_DoorFrame52/Geo_M_DoorFrame/Geo_M_DoorFrame`.
Other spans involve existing door-wall-corner and trim geometry. The first
report has positive contact-offset separation and zero impulse; subsequent
trim reports include nonzero impulse. These are logged contact artifacts,
not a claim that every report proves physical penetration. The conservative
classifier uses environment contact with `abs(normal.z)<.5` until lost.

The full run has 57 non-floor contact spans and no logged camera-proximity
hits. The latter is not a complete containment/occlusion oracle. Later RGB
has a large black obstruction; its exact geometric identity is not established.
No controller, light, camera, seed or threshold was changed to obtain a
replacement model run, and C1–C18 was not substituted for the frozen C1–C19
gate after observing failure.

| Baseline quantity | Measured value |
| --- | ---: |
| Predictions | 48 |
| Turn decision | NOVA CARTER E16 TURN EXECUTION VALIDATED |
| Bright baseline decision | FAIL / INSUFFICIENT EVIDENCE |
| Clean stable-left prediction ranges before contact | C5–C7, C10–C11, C13–C18 |
| First / last useful turn prediction before contact | C5 / C18 |
| Actual qualifying intervals | C16→C17 and C17→C18 |
| C16→C17 episode Δforward / Δleft / Δyaw | .519522 m / .540394 m / +7.595421° |
| C17→C18 episode Δforward / Δleft / Δyaw | .436185 m / .609571 m / +8.128592° |
| Accepted C16 / C17 measured-yaw-to-command-integral ratio | 1.038362 / 1.055322 |
| Maximum / final signed yaw change | +67.552009° / −1.476929° |
| Executed XY path length | 13.640884 m |
| Final world XY | (1.491049, 12.202758) m |
| All-48 mean luminance: minimum / median | 11.487062 / 110.607916 |
| All-48 p95 luminance: minimum / median | 105.851600 / 143.170500 |
| C1–C19 mean range | 110.180254–136.792292 |
| C1–C19 p95 range | 176.2768–185.9920 |
| C1–C19 maximum dark / saturation fraction | .00277006 / .00167872 |

Brightness failures are C20–C25 and C28–C48. C20 initially fails low texture
(std 12.796), whereas C48 has mean 11.61294, p50 0 and dark fraction .853299
despite p95 106.8516. A p95-only rule would miss this predominantly black image.
C26/C27 brightness recovery does not erase prior contact or qualify a new
human experiment. The original RGB montage and brightness/contact timeline
make these distinct failure modes visible.

## Stationary-human stage: not executed

The baseline failure stops Stage B. No stationary XYZ/yaw/radius was selected;
no Hospital occluder, bypass, wall overlap, floor support, hidden-to-visible
region, silhouette or RGB-leakage result is claimed. No new human asset was
loaded in an experiment, no stationary-human run ID exists, and prediction
count for that stage is zero. Measured human velocity/transport is **N/A**,
not a fabricated zero. No `primary_reveal_pair.json` is produced.

Preparatory helpers support the previously used official People asset family,
fixed initial position/yaw with no update/motion trigger, segmentation logging
outside model input, finite-difference measured velocity and source-immutable
replay. Their proposed visibility rule is hidden=0 pixels; clearly visible
requires fraction ≥.002 and bounding-box height/width ≥80/20 pixels. Proposed
preflight requires ≥3 hidden observations, paired absent/present RGB mean
absolute difference ≤1/255 and fraction with any-channel difference >10 ≤.01,
plus manual no-cue review. At 1920×1080 the visible fraction requires at least
4148 pixels. These values have **synthetic tests only**; there is no actual
Hospital placement preflight or human freeze receipt. They must not be treated
as qualified thresholds for a real human run.

The event helper selects the first clearly visible observation Ck and its
immediate predecessor C(k−1). If OLD is marginal/visible or the first event is
C1, it records an invalid transition and never substitutes a later favorable
pair. A corridor-grid candidate helper uses measured poses, not one predicted
waypoint. Full candidate occluder/raycast/bypass qualification and the
human-specific saved analyzer were not completed because their prerequisite
failed. No physical occlusion test is passed by a synthetic fixture.

## Saved OLD/FRESH inspection and RAW seam

The baseline inspection pair **C16→C17** comes from the first measured turn
interval. It is not a primary human event. The compatibility filename
`primary_pair.json` explicitly stores `primary: false`, absent human fields
and `SAVED NO-HUMAN DESCRIPTIVE CONTROL`. `request_metrics.json` includes all
48 requests. Its inherited `clean_visibility` / `rgb_p95` fields use the older
channel-p95 context; the separate Rec.709 `brightness.json` and baseline gate
are authoritative for the new bright criterion.

Each chunk uses its own observation pose only:
`p_world = [x_obs,y_obs] + R(yaw_obs) p_native`. World X/Y are stage axes;
native X/Y mean forward/left, units meters, positive yaw counterclockwise.
Signed revision is `R(yaw_FRESH)^T (p_FRESH_world − p_OLD_world)`.
Source-confirmed nominal targets +.1…+3.0 s are assigned in derived analysis;
the native tensor contains no timestamp channel. Common absolute nominal time
uses linear interpolation with FRESH knots and exact common endpoints, no
extrapolation or spatial correspondence. No ready/switch re-anchoring,
rigid fitting, NN, ICP, DTW, reconciliation or compensation is performed.

| Baseline C16→C17 quantity | Value |
| --- | ---: |
| OLD / FRESH observation time | 8.783333791 / 9.283333817 s |
| Common absolute nominal domain | 9.383333817–11.783333791 s; 25 points |
| Overlap RMSE / mean / max | .507066 / .501745 / .634926 m |
| Mean / max absolute forward revision | .500807 / .634755 m |
| Mean / max absolute lateral revision | .024495 / .079561 m |
| Mean signed lateral revision | −.011094 m |
| Common-time endpoint shift | .573516 m |
| Full endpoint shift, different nominal times | .861623 m |
| Controller-selected lookahead shift, not same-time correspondence | .729602 m |
| Mean / max absolute derived tangent revision | 46.003961° / 171.269012° |
| Δv / Δw | 0 m/s / −.017002989 rad/s |

The revision is predominantly nominal forward progress. Large segment tangent
differences reflect jagged/short waypoint segments and do not imply equally
large executed robot yaw. Both observations are bright and before contact,
but the pair cannot supply a hidden-human/new-information comparison.

For the RAW diagnostic, B is the measured application pose immediately after
setting the new wheel target and before the next physics step. FRESH stays
anchored to its observation. An explicitly derived observation XY at tau=0
is added only for temporal interpolation to the measured pending duration;
it is never inserted into raw native data. The executed tangent uses the last
.1 s clipped to the observation-to-switch interval; a displacement ≤1e−6 m
is undefined. Tangents are **derived geometry, not native TIC-VLA yaw**.

| C17 pending/switch context | Value |
| --- | ---: |
| OLD source | C16 |
| Action call wall time | .054264726 s |
| Observation→switch wall / simulation time | .488470189 / .100000005 s |
| Pending physics ticks / ticks strictly inside action call | 6 / 3 |
| Robot translation / yaw transport | .150025443 m / +1.706310° |
| Human transport | N/A: absent |
| RAW position gap | .020414603 m |
| RAW executed→FRESH tangent gap | 3.638954° |
| RAW heading→FRESH tangent gap | 2.921272° |
| Executed tangent-window displacement | .150025435 m |

These are measured baseline quantities, without a severity threshold or a
claim of latency-induced hard case. There is no human-vs-baseline causal
comparison, OLD-human clearance, FRESH-human clearance or avoidance gain.

## Reproduction and artifacts

The archived run records the exact one authorized model invocation:

```bash
bash scripts/isaac6_python.sh research/nova_bright_run.py --config configs/research/nova_e16_bright.yaml --freeze-receipt outputs/nova-occlusion-setup-20261010/baseline-freeze.json --run-id nova-e16-bright-baseline-20261010-01 --mode baseline
```

This is provenance, not an instruction to rerun the completed experiment.
Existing run/output directories are rejected. Saved-only analysis can be
reproduced in fresh directories with ordinary NumPy/Matplotlib/PyYAML/Pillow:

```bash
python3 research/analyze_nova_bright.py --run-dir outputs/nova-e16-bright-baseline-20261010-01 --output-dir outputs/nova-e16-bright-baseline-analysis-REVIEW
python3 research/plot_nova_bright_audit.py --run-dir outputs/nova-e16-bright-baseline-20261010-01 --analysis-dir outputs/nova-e16-bright-baseline-analysis-20261010-01 --output-dir outputs/nova-bright-audit-REVIEW
```

The analyzer reuses unchanged projection, temporal-overlap, RAW-seam and
Nova turn/response helpers. `turn/` contains source/code hashes, 48-request
CSV/JSON, world paths, predicted geometry, measured yaw/command response and
timing figures. The extra audit contains original RGB montage,
brightness/contact timeline, overlap-vs-horizon and world/RAW seam figures.
No human visibility/clearance plot is fabricated for an absent human.

## Isaac GUI replay

Viewer: [`research/view_nova_occlusion.py`](../research/view_nova_occlusion.py).
Validated non-headless output: `outputs/nova-bright-baseline-gui-20261010-01`,
including `ready.json`, scene snapshot, overview, desktop capture with the
original RGB panel, frame manifest and **48 PNGs at 20 fps** over simulation
8.533333791–10.883333791 s. ffmpeg is unavailable; no installation or MP4.

The GUI replays saved past-tick root XYZ/WXYZ poses with the timeline stopped,
robot rigid bodies disabled and zero model/socket/physics execution. It uses
the exact fixed bright profile. Original recorded RGB is displayed unchanged;
retrospective chunk guides/markers and roof-clipping overview are display-only.
Wheel animation is not reconstructed. Cyan is executed robot path, orange OLD,
green FRESH, white B, yellow FRESH observation. Human markers are absent for
this no-human run; the panel says **HUMAN ABSENT**.

Controls include play/pause/restart, .25/.5/1/2×, timeline, previous/next
request, OLD/FRESH selection, overview/follow/front-Hawk cameras. Here the
event button is truthfully **Jump to baseline turn**, starting .25 s before
C16 observation at .5×; default launch is paused. There is no first-reveal
jump because no human reveal exists. Stationary-human replay and first-event
jump are synthetic-tested only. `ready.json`'s `stationary_human_unchanged:
false` means inapplicable for this absent-human run; `baseline_human_absent:
true` is the relevant passing check.

One-line command (fresh output required):

```bash
bash scripts/isaac6_python.sh research/view_nova_occlusion.py --run-dir outputs/nova-e16-bright-baseline-20261010-01 --analysis-dir outputs/nova-e16-bright-baseline-analysis-20261010-01 --output-dir outputs/nova-occlusion-gui-REVIEW
```

Optional `--self-test --export-primary` exercises saved poses/cameras and
exports the baseline inspection window; the compatibility flag name does
not make it a primary reveal event.

## Verification, interpretation and limits

All **94 pure tests pass** (79 retained +15 new). New tests cover deterministic
brightness statistics/profile equality, rejected black/white frames, fixed
pose/zero velocity, semantic counts, hidden/marginal/first-clear rules without
favorable substitution, RGB leakage audit, candidate independence from native
waypoints, synthetic conflict/clearance, failed-baseline launch rejection,
stationary replay, event jump, source immutability and viewer no model/physics.
Existing temporal-overlap, RAW-seam and no-extrapolation tests are retained.
Actual Hospital occlusion/bypass and human render qualification remain untested.

Independent saved-data calculations verify all 48 native contracts and RGB
sidecars (maximum statistic error 8.53e−14), unchanged base config, exact
renderer readback, freeze preceding every observation, .5070664204654003 m
overlap RMSE and .0204146027631427 m RAW position gap, with tangent agreement
within 1e−10 degrees. All 301 analysis source hashes and GUI source hashes
match. All 2,639 protected earlier files match before the append-only work-log
entry; no prior raw/report/runtime/checkpoint/official source was modified.

There is positive evidence for improved brightness along the calibrated
route and a clean early actual left turn. There is negative evidence for the
new baseline's full prerequisite: early contact and later unusable RGB.
The bright profile changed observations and therefore can change policy/state
history; this single run does not isolate illumination as the cause of contact.
The research decision remains **INSUFFICIENT EVIDENCE** for occlusion-triggered
human revision. No threshold relaxation, replacement pair or extra model run
repairs that conclusion in this task. There is no claim of reconciliation or
graph benefit/necessity, automatic correspondence, general or real-world VLA
performance, full DynaNav benchmark validity, or successful navigation.
