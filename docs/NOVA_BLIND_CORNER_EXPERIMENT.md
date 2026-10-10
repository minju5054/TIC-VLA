# Hospital E21 blind-corner experiment

**BASELINE FAIL — STOP BEFORE HUMAN PREFLIGHT.** Exactly one no-human run
physically turned out of the room, but failed the frozen RGB brightness gate.
Stage C and the stationary-human model run were **not executed**. Decision:
**INSUFFICIENT EVIDENCE** for a strict reveal or reconciliation-relevant response.

## Selected geometry and frozen route

Starting local/fetched origin `hardcase-probe`:
`390e14b408bdd4fe5c7ae055b151c660b999ff64`; upstream/main:
`9fa6f8b66b9e121d5df5df071297bba8e5353ebb`.

The selected route is official Hospital **episode_21**, leaving the room through
existing DoorFrame26 near world `(20.82, -0.06)` and turning right/north into the
hallway. Official start `(23.3, -0.9, 0.01)`, yaw 180 degrees; Nova spawn Z is
0.11 m using the existing runner offset. Goal `(20, 24, 0.01)`. Exact instruction:

> Exit the room and turn right to enter the hallway. Continue straight ahead and to the end of the hallway.

The full Hospital USD, 153-light fixture profile, Nova asset, front Hawk camera,
checkpoint, controller and 2 Hz target cadence are unchanged. No walls or
occluders were added. The failed E16 exhaustive search was not resumed.

Stage A used small, deterministic geometry proposals before any model call.
Official E1/E6/E11 are straight routes; E16 was excluded. All five bounded screens
are retained under `outputs/blind-corner-selection-20261010-01` through `-05`:

| Screen | Proposal count | Geometry valid | Ray transition possible | Reason / action before inference |
|---|---:|---:|---:|---|
| 01 | 15 | 0 | 0 | Circumcircle unnecessarily excluded the doorway |
| 02 | 15 | 0 | 0 | Oriented source bounds; existing door sill failed 2 cm support check |
| 03 | 15 | 10 | 0 | Audited sill admitted; early human exposure |
| 04 | 9 | 0 | 0 | One custom DoorFrame27 room-entry proposal blocked by door/bed; rejected |
| 05 | 9 | 9 | 3 | Tighter official E21 arcs and fixed approach-facing human orientation |

These were adaptive **geometry-only** screens, not five model trials or a claim
that the first geometry proposal succeeded. No custom route was executed. Screen
04's definition and source are preserved, explicitly distinct from official E21.
Screen 05 has three reference lanes X=19.25/19.5/19.75 m, turn radius 1.2 m and
three human Y=2.5/3/3.5 m positions at X=20 m. Ranking is largest newly exposed
sample count, then distance closest to 2 m, then reference variant and XYZ.

Selected candidate **7**: reference lane X=19.75 m, proposed human
`(20, 2.5, 0)`, yaw `-pi/2`, 17 newly exposed samples at reference index 6
(zero-based geometric sample, **not a model request ID**), distance **3.245178 m**.
The 1–3 m preference was not converted into a hard threshold. Prior ray hits
include the existing `/Root/SM_Door_02b2/SM_Door_02b` and
`/Root/trashcan_6/SM_TrashCan`; this is not attributed solely to a wall.
Sampled bypass clearance is **0.143017 m** using the unchanged combined proxy.

Robot route screening uses yaw-oriented source collision XY bounds plus 3 cm,
Z=0.05–1.25 m. Existing `DoorFloor23` measured height 0.0253609 m and upward
normal >=0.9038 justify a DoorFloor-only 3 cm support tolerance; ordinary support
and human placement retain 2 cm. The human-conflict radii remain Nova
0.6069825421953869 m + human 0.25 m. No shape or collision setting was changed.
All 2,161 Hospital collider records match the prior audit.

Seventeen head/torso/leg/arm rays use source camera extrinsics at geometric
reference body poses, with a broad aperture envelope. Three preceding samples
must be unexposed and the next gain exposure after turning begins. This is only
a possibility screen: no semantic HIDDEN/CLEAR labels, native timing or actual
model trajectory are inferred. Reference paths are not controller waypoints.
Stage A performed zero model calls, RGB renders or navigation physics steps.
Three subsequent no-human pose previews were the only pre-baseline RGB renders.

Coordinates are metres in the Hospital Z-up world. Body X is forward and Y
left; yaw is radians counterclockwise about +Z. Native point projection is
`world_xy = observation_xy + R(observation_yaw) @ [forward, left]`.
USD camera transforms use row vectors: `camera_to_world = camera_to_body @
body_to_world`; the camera looks along local -Z with +X right and +Y up. Rays
start at that camera origin and end at the human's world sample points. The
reference screen assumes planar body Z=0; it does not substitute for a future
saved-full-pose preflight. Preview/replay quaternions are explicitly `w,x,y,z`.

## Single baseline and strict stop

Freeze: `outputs/blind-corner-route-freeze-20261010-01/`.
Config SHA256 `70290cf9986258c8604e94e6874a9c9a02c9737311bd7c16b62cb471d8ea95de`;
freeze receipt SHA256
`fb6f1919b097dcd91dac70b18f390ced073f73374b5a9df04eb6c59a6d87232a`.
The receipt binds all research source hashes, config, source identities and the
one authorized baseline ID. Startup-room preview already failed brightness;
turn previews passed. This did not waive the actual baseline gate. No light,
exposure, route or controller tuning followed the result.

Run: **`outputs/blind-e21-baseline-20261010-01`**. All **48** native `(30,2)`
chunks are finite; continuous OLD command retention was verified. Observation
times are 0, 2.200000115, 2.700000141, …, 25.283334652 s. The 2.2 s bootstrap
and subsequent 0.500000026–0.583333364 s gaps are retained, not relabelled as an
exact 0.5 s schedule. Physics continues during inference in this run.

The preregistered exit region is X<=20 m, 0.5<=Y<=5 m and yaw between 45 and
135 degrees, followed by one more observation. RGB and contacts are checked
through that following request's application.

| Baseline gate | Result | Saved evidence |
|---|---|---|
| Complete finite chunks | PASS | 48/48 |
| Intended turn and exit | PASS | C11 at 6.700000349 s, `(17.485689, 0.929322)`, yaw 121.84 degrees |
| Following observation | PASS | C12 at 7.200000376 s, `(17.176893, 1.610505)` |
| Relevant non-floor contact through region | PASS | None through C12 application, 7.283333713 s |
| Frozen useful-RGB criterion | **FAIL** | C1–C12 mean luminance >215; C1–C4 saturation >5% |

C1 mean=228.279536, saturated fraction=0.483463; C11 mean=221.719229 and
C12 mean=219.248538. C5–C12 saturation passes, but mean luminance still fails.
The doorway/corridor is visually legible; **FAIL refers to the predeclared
quantitative gate**, not a claim that every RGB pixel is unusable. Frozen
`bright_pass` rules were neither loosened nor replaced by subjective review.
Two later contact spans exist in the complete run; the first is the wet-floor
sign/right wheel at 13.416667366 s, outside the required turn window. This is
not a collision-free full-navigation result or a goal-success claim.

Stage C: **NOT EXECUTED — Stage B failed**. No local refinement, human semantic
preflight or fixed placement was validated. Stage D: **NOT EXECUTED**. There is
no primary human OLD/FRESH pair, human-response clearance, conflict time or
revision metric. GUI C10→C11 is a **baseline inspection pair only**. The saved
human position remains an unexecuted geometry proposal.

The unchanged semantic rule for any future separately authorized preflight is
HIDDEN=0 pixels; CLEAR fraction>=0.002, bbox height>=80 and width>=20; intermediate
nonzero visibility is MARGINAL. First CLEAR only, immediate OLD and three prior
HIDDEN observations, current clearance, OLD remaining conflict, application
before conflict and local bypass would all still be required. These gates were
not evaluated or claimed here. No later-pair substitution is made.

## Saved artifacts and replay

Final analysis: **`outputs/blind-e21-analysis-20261010-02`**:

- `trajectory_result.png` and `trajectory_result.pdf`: complete actual baseline
  plus enlarged doorway/turn, equal world XY scales in metres, proposed human,
  combined proxy, collider AABB context and exact failed gate.
- `trajectory_coordinates.json`: underlying actual XY/time, gate observation
  poses, corner/human positions, local bounds and collider polygons.
- `summary.json`, `brightness.json`, `request_metrics.json`, `primary_pair.json`
  and `metadata.json`: saved-only gate, baseline inspection and source hashes.

The initial `-01` analysis is preserved; `-02` only improves the plot/report
layout and provenance. Executed analyzer source was archived before the display
change in `outputs/blind-corner-setup-20261010/baseline-executed-code/`.
Plots contain no hypothetical FRESH human response. No revision plot is warranted.
Hospital polygons are collider XY bounding-box proxies, not a reconstructed map.

Reproduce saved analysis with a **new** output directory:

```bash
MPLCONFIGDIR=/tmp/tic-vla-mpl python3 research/analyze_blind_corner.py --run-dir outputs/blind-e21-baseline-20261010-01 --output-dir outputs/blind-e21-analysis-REVIEW
```

Saved replay is open and self-tested at `outputs/blind-e21-gui-20261010-01`.
It provides baseline turn/contact jumps, OLD/FRESH guides, actual path, archived
RGB and overview/follow/Hawk cameras. `GUI_READY`, source immutability, stopped
timeline, **model_calls=0**, **physics_reexecuted=false** are recorded. The
overhead roof cutaway is display-only. No human or semantic evidence is invented.

```bash
bash scripts/isaac6_python.sh research/view_nova_occlusion.py --run-dir outputs/blind-e21-baseline-20261010-01 --analysis-dir outputs/blind-e21-analysis-20261010-02 --output-dir outputs/blind-e21-gui-REVIEW
```

## Validation and scope

**155 tests PASS**: existing 146 plus nine covering deterministic geometry
selection, explicit E21 opt-in, first CLEAR/no favourable substitution, nominal
segment conflict, own-observation projection, following-observation gate,
output/source protection and actual plot coordinates. PNG and rasterized PDF
were visually reviewed. All protected prior outputs remain byte-identical.

Only `research/hospital_episode.py`, `research/nova_source.py` and
`research/nova_simulation.py` change existing runtime code: explicit episode
selection/source audit, with E16 as the default. Official DynaNav model,
benchmark, behavior and controller source are unchanged. New tools are additive.

One simulated scenario and one baseline do not establish model behavior under
occlusion. Native chunks remain body-frame forward/left metres with each
request's own observation anchor; nominal row timing is source-assigned
`+0.1…3.0 s`, not a native timestamp/yaw channel. No spatial correspondence,
graph optimization, reconciliation, latency-causal hard-case, navigation
performance or human-causal claim is made. The mandatory stop leaves the
scientific response question unanswered.
