# Hospital lights-on occlusion experiment

## Scope and provenance

This experiment replaces Bright Hospital v1's path lights with illumination
derived from the actual Hospital light fixtures. It preserves the original
camera exposure and downstream Nova controller. A clean no-human baseline is
required before any stationary-human placement preflight, and an actual
occlusion preflight must pass before one human model run can be authorized.

Starting `hardcase-probe` and freshly fetched `origin/hardcase-probe`:
`1013ccd872994f4e6cca1a35c4ca62b33a863cb6`.
Freshly fetched `upstream/main`: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`.
The starting worktree was clean. Setup evidence and hashes are in
`outputs/hospital-lights-setup-20261010/`. Generated evidence is ignored by Git.

Bright v1 remains preserved as failed prerequisite evidence: its 16 path-derived
SphereLights and +2-stop exposure did not implement the requested Hospital
lighting, and its baseline contacted Hospital geometry. This task does not edit
the v1 configuration, its evidence, or its report.

## Actual authored-light and fixture audit

`outputs/hospital-native-light-audit-20261010-01` loads the actual official
Hospital stage before any model call. `lights.json` records every UsdLux LightAPI
prim's type, activation/visibility, intensity/exposure, color/temperature,
radius/size, world matrix and fixture ancestry. The audit found **5 authored
lights**, all active, loaded and inherited-visible:

| Type | Count | Paths / state |
|---|---:|---|
| SphereLight | 4 | `BP_DrinksMachine_2/{PointLight,PointLight1}` and `BP_DrinksMachine2/{PointLight,PointLight1}` under `/Root`; intensity 10000, exposure 0, radius 0.1 m |
| DomeLight | 1 | `/Root/DomeLight`; intensity 10000, exposure 0 |
| Rect / Disk / Cylinder / Distant | 0 | No authored ceiling area lights |

These indoor SphereLights belong to vending machines, not ceiling fixtures.
All four receive the same candidate multiplier; the Dome remains unchanged.
There were 1571 keyword-path matches, 448 fixture/keyword mesh records and 281
emission-named material records; these broad keyword counts are **not** counts
of useful ceiling emitters. Full paths, positions, bounds and bindings remain in
the audit JSONs, along with 2161 enabled Hospital collision prim records.

The two actual ceiling fixture asset families supply **153 mesh instances**:
49 round `Geo_M_Light2_low` fixtures and 104 `Light_test` panels. All instances
in the Hospital, including the upper floor, are selected by asset family,
without a robot-route input. Fixture material inspection in
`outputs/hospital-fixture-profile-20261010-01` resolved MDL constants and textures
that are not exposed as USD emissive input attributes:

| Material | Emissive texture | MDL value / source SHA256 |
|---|---|---|
| `M_LightCircle.mdl` | `Geo_M_Light2_low_Tex_Light_Circle_Emissive.png` | B = 1000; `bed5b4f52f00aa8696ec8da765f163951d9ed3f7fce512214c5d857103adb81e` |
| `M_Light.mdl` | `Geo_M_Light2x2_Low_Tex_Light_Emissive.png` | B = 1000; `235bf35f8221a3b280e8c209a8db03a962b032f1ad8200db6ca27299bddd8d5c` |

Fixture mesh paths, world locations, matrices, sizes and material bindings are
listed individually; no unverified room names are assigned. The official root
USD byte SHA256 is
`83e5d606fbb2de873314379e0ff74900930735eb3ae99b312c9bf17db0033383`.
The original USD is never saved or modified. Runtime lighting overrides are
authored in the session layer, then the previous edit target is restored.

## Lighting calibration and frozen selection

All lighting candidates were inspected with **zero model calls**, using the
48 saved body/camera poses from `outputs/nova-e16-closed-loop-20261010-01`.
Physics remained stopped during these renders. This use of an old successful
trace is illumination inspection, not a new navigation outcome or human
placement source.

The brightness rule was fixed before calibration: Rec.709 luminance on 8-bit
RGB, mean 40–215, std ≥15, p50 ≥25, p95 ≥100, dark fraction (`Y<15`) ≤0.35,
and saturation fraction (any channel ≥250) ≤0.05. Every image retains mean,
std, p05/p50/p95/p99, dark fraction and saturation fraction. These are image
qualification criteria, not hard-case severity thresholds.

| Candidate | All 48 poses pass? | Minimum mean | Minimum p95 |
|---|---|---:|---:|
| Four native indoor lights ×1 | No | 0.703 | 1.928 |
| Same four ×2 | No | 0.716 | 1.928 |
| Same four ×4 | No | 0.646 | 1.928 |
| 153 fixture lights, intensity 5000 | No | 3.097 | 5.928 |
| Same fixtures, 15000 | No | 11.927 | 23.127 |
| Same fixtures, 45000 | No | 35.677 | 61.205 |
| Same fixtures, 90000 | No; C21–C25 fail | 59.094 | 97.345 |
| **Same fixtures, 180000** | **Yes** | **97.827** | **149.205** |

The additional 90000/180000 global candidates were registered after the lower
model-free candidates failed and before any new inference. No per-room tuning
or model-output-dependent lighting was used. Profile 180000 is the least
intense **tested** profile passing all poses; this is not an optimized minimum.
Its maximum calibrated mean is 206.400 and maximum saturation fraction 4.692%.
All 48 images and full-resolution approach/corner frames were visually reviewed:
walls, floor, staircase and hallway objects are readable; early views are bright
with localized highlights, without gross white clipping. Human silhouette
visibility is a separate preflight condition.

Selected policy in `configs/research/nova_e16_hospital_lights_on.yaml`:

- Existing four indoor sources retain ×1 intensity; the Dome is unchanged.
- Add 49 DiskLights and 104 RectLights at actual fixture footprints, all with
  intensity 180000, exposure 0, white color and `normalize=false`.
- Each light uses its fixture's actual world transform and footprint size,
  with emission facing downward. A uniform 2 mm offset below the fixture's
  lower face avoids self-shadowing. All source and light matrices are recorded.
- No path lights, following lights, request-dependent transforms or extra
  viewer illumination. Fixture placement has no trajectory parameter.
- Original renderer: ACES (`op=6`), ISO 100, exposure time
  `0.019999999552965164` s, f/5, responsivity `1.1026709079742432`;
  original sRGB/gamma setting, whitepoint `[1,1,1]`, histogram and motion blur
  disabled. No Bright-v1 +2-stop exposure.

The model-free runtime probe
`outputs/hospital-lights-runtime-probe-20261010-01` and the actual baseline both
verify the source USD identity, all light transforms, unchanged 2161-collider
inventory, and no CollisionAPI/RigidBodyAPI on the added lights. Normalized
collision inventory SHA256:
`c6a68de8362e31f0255088ce24824510d8466dcc1d033c2c5a7c396649fd8e3b`.

## One no-human baseline

Run: `outputs/nova-e16-hospital-lights-baseline-20261010-01`.
Saved-only analysis: `outputs/nova-e16-hospital-lights-analysis-20261010-01`.
Exactly one new baseline was executed: 48 finite native `(30,2)` chunks,
forward/left metres in each observation body frame. The source `(1,30,2)` batch
dimension is removed only for archival storage. Isaac 6.0.1, Nova, original E16
instruction, strict checkpoint, seed, Hawk 1920×1080, direct DifferentialController,
wheel radius/base 0.14/0.4132, lookahead/gains/speed limits, 2 Hz target cadence,
60 Hz control and one outstanding request with OLD retention are unchanged.

Config SHA256:
`b28a898ad14a9874abd69ab7d11ae9b50f52d68318d41ac9eb188ec4da9c038a`.
Freeze receipt SHA256:
`7ec9652f88082d30eea431aee502104a25c5aa9d611404e23843429490f98bfa`.
The receipt binds 62 research files, 56 source files, the selected profile,
original exposure, visual review, 106 passing tests and the single authorized
baseline ID. The independent baseline-completion check found all frozen files
identical. During final review, a metadata-key collision was corrected in the
calibration and two later preflight CLIs: `source_evidence_sha256` now remains
separate from `provenance()`'s code `source_sha256`. This changes provenance
serialization only, not illumination, poses, physics, criteria or model calls.
The exact executed CLI sources are preserved in the setup directory's
`executed-preflight-sources/`, and the completed evidence's explicit full-file
manifest is in the occlusion review directory's `source_evidence_sha256.json`.

### Spatial prerequisite, declared before inference

The actual inner corner's east face is world X = 3.15908622548 m, from
`/Root/Geo_M2_BaseWallCorner2_578/Geo_M2_BaseWallCorner/Geo_M2_BaseWallCorner/Section0`.
The north hallway wall begins at Y = 12.34822601318 m, from
`/Root/Geo_M2_DoorWall2Corner2_656/Geo_M2_DoorWall2Corner/Geo_M2_DoorWall2Corner/Section0`.
With the previously audited conservative robot radius 0.6069825421953869 m,
the robot center must satisfy X ≤ **2.55210368328** and
Y in **[9.45783611886, 11.74124347099]**, then reach one subsequent observation.
This is a fixed physical region, not a copied request-ID threshold.

In the new baseline, first crossing happens at **C19** (10.300000537 s), and
the required subsequent observation is **C20** (10.800000563 s). The context
window extends through C20 application at 10.883333901 s. All required frames
pass brightness/proximity checks; all **48** frames also pass. No relevant
non-floor contact is recorded anywhere in the run. Stage C **PASS**.

Actual mean luminance range: **92.497–204.572**; minimum p95 **143.205**;
maximum dark fraction **0.790%**; maximum saturation **4.870%**. Independent
recomputation from the original PNGs matches recorded statistics within 1e-10.
Visual review covers all 48 original inputs. Stable left predictions and
subsequent actual left motion pass the existing execution rule. Net yaw change
is **+77.466°**, with maximum accumulated signed change **+102.311°**.

### Wall clearance context

Raw observations include 96 PhysX horizontal rays at heights 0.15/0.35/0.60 m,
excluding the robot and retaining actual structural wall/doorframe/trim hits.
A second saved-only trace compares the conservative robot disk to audited
structural collider world AABBs at every recorded physics tick.

The required spatial window's minimum AABB/disk proxy is **+0.497043 m** at
10.850000566 s, robot XY (1.157579, 11.226068), yaw 3.139021 rad,
target (v,w) = (1.5, 0.186709). The whole-run minimum is **−0.037687 m** at
21.316667778 s against `Geo_M2_TrimDoor4`, XY (−14.425719, 9.438142),
yaw −3.132875 rad, target (1.5, −0.080257). The minimum sampled ray/disk proxy
is −0.034575 m at C41. **Negative conservative proxies do not establish contact
or penetration**: the enclosing disk can overlap bounds while the actual robot
does not. Measured contact count remains zero; first-contact time/prim are null.

### Comparison to original successful E16

Both runs are summarized in the same fixed physical turn region, selecting
C12–C18 in each by measured position, not by favorable output.

| Turn-region / execution metric | Original successful | Lights-on v2 |
|---|---:|---:|
| Median native endpoint left [m] | 0.613281 | 0.373047 |
| Median lookahead left [m] | 0.178711 | 0.153320 |
| Median lookahead bearing [deg] | 9.151247 | 7.707850 |
| Median target w [rad/s] | 0.269725 | 0.278189 |
| Maximum target w in region [rad/s] | 0.672341 | 0.314227 |
| Whole-run net yaw change [deg] | 84.985845 | 77.466206 |
| Maximum signed yaw change [deg] | 98.596330 | 102.311103 |
| Relevant non-floor contact spans | 0 | 0 |
| Fixed spatial crossing / next observation | C19 / C20 | C19 / C20 |

Prediction geometry differs, controller extraction is unchanged, scene lighting
differs, and asynchronous runtime timing/reasoning state can differ. These are
separate closed loops, not identical model-input trials. The comparison does
not isolate a causal lighting effect or validate general navigation performance.
The robot passes the goal and does not establish instruction-completion success.

## Stationary-human preflight: prerequisite failed after baseline PASS

Stage D was authorized by the clean new baseline. Stage E was **not executed**:
the occlusion preflight failed. This is **not a baseline failure**; a report
label saying `BASELINE PREREQUISITE FAILED` would misdescribe this outcome.
No stationary-human config/freeze receipt or human model run was issued.

Only the new lights-on baseline supplies reconstructed camera poses, native
futures and temporal checks. The actual corner's hallway band defines a 0.25 m
grid over the first four metres behind the corner: 192 candidates, ordered
east-to-west then south-to-north. Downward PhysX rays and footprint sphere
queries assess floor support and static overlap; conservative robot-sphere
queries assess a local two-metre bypass strip. These are local feasibility
checks, not a navmesh or full bypass navigation proof. Of the grid, 191 have a
clear human footprint/floor and 162 have a clear local bypass strip. The 48
rendered candidates additionally have initial center-ray occlusion and at least
one later baseline OLD future intersecting the fixed human proxy. Center rays
only screen candidates; semantic pixels decide actual visibility.

Evidence:

- `outputs/nova-hospital-occlusion-geometry-20261010-03`: actual PhysX query
  paths, saved-pose camera matrices, all candidate decisions and preregistration.
- `outputs/nova-hospital-occlusion-render-20261010-01`: paired absent/present
  full-resolution RGB, masks, pixel counts/fractions/bounds, brightness and
  leakage results for every rendered candidate through its first clear event.
- `outputs/nova-hospital-occlusion-review-20261010-01`: aggregate outcome,
  visibility/clearance figure and diagnostic candidate 138 image sheet.

The existing visibility rule was retained: hidden means zero human pixels;
clear means fraction ≥0.002 and bounding box at least 80 pixels high ×20 wide.
At least three prior observations must be hidden, all pre-reveal observations
must remain hidden without paired-RGB leakage, and the immediately preceding
OLD must be hidden. FRESH is **always the first clear event**, never a later
favorable pair. The OLD remaining nominal future must have negative clearance
against robot radius 0.6069825422 m plus human radius 0.25 m. RGB leakage bounds
remain mean absolute channel difference ≤1 and fraction with any channel
difference >10 ≤0.01.

**All 48 candidates fail**. None has a hidden-to-first-clear adjacent pair;
there is already partial (`MARGINAL`) visibility at the preceding observation.
First clear observations span C4–C11, before the baseline future reaches the
human: OLD remaining clearance at those events is **+5.521151 to +9.059684 m**,
so none supplies the requested OLD conflict. All 125 strictly hidden paired
observations also fail the conservative RGB-difference gate (mean differences
3.194–9.896 on 0–255 channels). These differences include renderer variation;
they do not establish that a human shadow/reflection caused the difference.
The independent silhouette-transition and OLD-conflict failures already block
human inference, even if the paired-render difference measurement were improved.

Diagnostic candidate 138, world (0.25, 10.5, 0) m, minimizes first-clear OLD
clearance among this grid. This is an explanatory example, not an experiment
pair selection: C1–C7 hidden, C8–C9 marginal, first clear C10. C9 contains 1089
human pixels (0.0525%); C10 contains 9000 (0.4340%), bbox [211,316,311,558].
Its C9→C10 OLD clearance is **+5.521151 m**. Actual images show a readable
human emerging progressively from the existing corner, not a sudden hidden
OLD / conflicting future event. The official character retains its authored
rest pose; the 0.25 m capsule is a proxy and does not enclose its extended arms.
No claim of exact human-mesh clearance follows from these footprint queries.

No new wall, visibility toggle, delayed spawn or human motion was used to force
a reveal. Preflight variants are independent stopped-time scene inspections;
they are not human trajectories. The real-run actor wrapper remains fixed from
before reset and has no update authoring, but that human run was not executed.

Two earlier model-free diagnostics are retained: `occlusion-preflight-...-01`
had stale paused-physics camera transforms and is **invalid as placement
evidence**. `occlusion-geometry-...-02` corrected camera matrices but used a
center-ray/conflict filter that selected no render candidates. The final
geometry pass includes both wall-side bands and sends candidates to actual
pixel qualification without treating first center-ray visibility as first clear
visibility. The final renderer uses the previously validated `SavedRenderer`,
with per-pose matrix agreement against the audited body-to-camera transform
and full saved body quaternion within 1e-6. These corrections did not rerun the
baseline or change its lighting/controller. All preflights used zero model calls;
only the geometry process's initial settling steps ran physics, never navigation.

## Decision and limits

**INSUFFICIENT EVIDENCE** for an occlusion-triggered reconciliation-relevant
hard case. There is positive evidence for the lights-on, clean-turn baseline.
The finite stationary placement search did not satisfy the reveal/conflict
prerequisite; it does not prove that every possible Hospital placement fails.
No human-run OLD/FRESH revision, RAW seam or clearance improvement is claimed.

For existing saved baseline geometry, native points remain anchored to their
own observation pose: `p_W = t_observation + R(yaw_observation) p_body`.
Native rows contain no timestamp or yaw. Derived timing uses the source-confirmed
nominal +0.1…+3.0 s convention; tangent headings are derived geometry only.
There is no ready/switch re-anchor, spatial fitting, ICP, DTW, nearest-neighbor
correspondence, smoothing or reconciliation. Actual continuous handoff retains
OLD during inference; no artificial latency was added. Baseline nonbootstrap
median action wall time is 0.053820 s, observation→switch wall time 0.528589 s,
observation→switch simulation time 0.100000 s, and robot transport 0.150096 m.
Those are measured context, not proof latency caused a failure.

This remains one scenario and one new baseline with 48 predictions. Lighting,
closed-loop observations, stochastic reasoning state and runtime timing are
not independently controlled causal factors. No controller/physics tuning,
new dynamic scenario, navigation-performance validation, correspondence
validation or reconciliation validation was performed.

## Saved GUI and reproducibility

`research/view_nova_occlusion.py` now reads the actual Hospital fixture-light
profile, retaining the existing v1 branch for old evidence. It adds an optional
red first-contact marker and first-contact jump for runs containing contact.
The validated v2 baseline has no contact, so that control/marker is absent.

`outputs/nova-hospital-lights-gui-20261010-01/ready.json` confirms recorded robot
poses, C14→C15 baseline-turn jump, camera controls, fixed own-observation
OLD/FRESH guides, archived RGB source, stopped timeline and source immutability.
This is a turn inspection pair, not a reveal pair. The GUI is left open.
Use **Jump to baseline turn**, **Follow robot camera** or **Nova front Hawk**;
the panel always shows the original saved model observation. Orange = OLD,
green = FRESH, cyan = recorded robot path, white = application boundary B.
The roof-clipped overview is only a viewing aid; no display light is added.
Model calls = **0**, physics reexecuted = **false**.

The validated GUI output contains `desktop.png`, `overview.png`, `frames.json`
and 48 exported viewport PNGs at a 20 fps replay time grid. MP4 was not produced
because ffmpeg was unavailable; no package was installed for optional video.

Saved-only baseline analysis:

```bash
MPLCONFIGDIR=/tmp/ticvla-matplotlib python3 research/analyze_hospital_lights.py --run-dir outputs/nova-e16-hospital-lights-baseline-20261010-01 --original-run-dir outputs/nova-e16-closed-loop-20261010-01 --audit-dir outputs/hospital-native-light-audit-20261010-01 --output-dir outputs/nova-e16-hospital-lights-analysis-REVIEW
```

Actual model-free preflight commands (these completed output IDs are immutable;
use new IDs when intentionally repeating diagnostics):

```bash
bash scripts/isaac6_python.sh research/preflight_hospital_occlusion.py --baseline-run outputs/nova-e16-hospital-lights-baseline-20261010-01 --baseline-analysis outputs/nova-e16-hospital-lights-analysis-20261010-01 --output-dir outputs/nova-hospital-occlusion-geometry-20261010-03
bash scripts/isaac6_python.sh research/render_hospital_occlusion.py --baseline-run outputs/nova-e16-hospital-lights-baseline-20261010-01 --geometry-dir outputs/nova-hospital-occlusion-geometry-20261010-03 --output-dir outputs/nova-hospital-occlusion-render-20261010-01
```

All **110 tests pass**, including the previous 94 plus 16 lighting/spatial-gate/
preflight/source-preservation checks. Coverage includes authored-light audit
contracts, deterministic all-indoor scaling, source fixture transforms, exposure
identity, no light physics APIs, spatial gate independent of request IDs,
first-contact selection, physical-region comparison across different request
IDs, unchanged source files, both human prerequisites, hidden/marginal/first-clear
rules and GUI contact/reveal controls. Synthetic tests are not experiment evidence.

Exact GUI command with the actual source/analysis IDs and a fresh output ID:

```bash
bash scripts/isaac6_python.sh research/view_nova_occlusion.py --run-dir outputs/nova-e16-hospital-lights-baseline-20261010-01 --analysis-dir outputs/nova-e16-hospital-lights-analysis-20261010-01 --output-dir outputs/nova-hospital-lights-gui-REVIEW
```
