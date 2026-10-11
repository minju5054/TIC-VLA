# Hospital E16 fixed C26 cart: one reduced-speed diagnostic

## Scope and execution contract

The question is whether lowering the controller's maximum forward speed gives
TIC-VLA useful obstacle observations and a genuinely different, safe FRESH path,
not simply a later collision. This is the separately authorized Isaac **6.0.1**
continuous research harness with Isaac **5 Python model inference only**. No
Isaac 5 SimulationApp or official DynaNav episode is run by this task.

Reference: `outputs/nova-e16-c26-cart-20261011-01`.
Reduced-speed run: `outputs/nova-e16-c26-cart-speed12-20261011-01`.
Immutable preflight: `outputs/e16-c26-preflight-20261011-01`.
Setup evidence: `outputs/e16-c26-speed-setup-20261011-01`.
New scoped authorization/config: `outputs/e16-c26-speed-freeze-20261011-01`.

Starting local/fetched origin `hardcase-probe`:
`f1db555790cb78bbde8adb3bb7cfd7c1a155032b`.
Upstream/main: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`.
The execution receipt records the precise starting revision, dirty state and
individual source hashes. Delivery SHA is recorded after commit in the setup
folder's Git finalization receipt.

The **only config difference** is `controller.max_v: 1.5 -> 1.2` m/s.
Both runs retain 48 predictions, seed 36, target inference 2 Hz, physics/control
60 Hz, lookahead 1.0 m, max_w 1.2 rad/s, angular gain .8, filter gain .35,
epsilon .001, original instruction, camera, fixture lighting, model weights,
history/state/KV inputs and continuous OLD retention. Angular command can change
through the original formula's dependence on v; steering response is not held
equal. `research/control.py`, original runner and DynaNav are unchanged.

C26 denotes the **original no-cart baseline's world location**, not the new
run's C26. Cart center is (-3.231875658, 10.711772919, 0.000000387) m, world yaw
-2.935430760 rad, same frozen asset/material/scale/static collision. Floor,
static overlap and initial clearance still pass; **local bypass still FAILS**,
north/right 0/10 and south/left 0/10 passing strips. The new authorization scope
`ONE_C26_MAX_V_1_2_RUN_DESPITE_BYPASS_FAILURE` records the user's explicit single
diagnostic exception and binds run ID, speed, budget, source and config hashes.
It does not change these gate results or reuse the old authorization.

## Analysis conventions

Saved native output is finite (30,2), forward/left metres in the robot body
frame at observation. Each curve is projected using its own observation pose:
`p_world = [x_obs,y_obs] + R(yaw_obs) p_native`. World axes, metre units and
positive CCW yaw about world +Z are unchanged; saved quaternions are wxyz.
No ready/application pose replaces that anchor. No yaw or intrinsic timestamp
channel is added to native data.

Waypoint timing is assigned from the source-confirmed nominal target convention
(+0.1 ... +3.0 s). The existing temporal-overlap routine evaluates the two world
polylines at the same absolute nominal times, interpolating linearly when actual
request gaps are not exactly 0.5 s. It uses FRESH knots plus common endpoints,
with no extrapolation or spatial matching. Lateral revision is FRESH-minus-OLD
rotated into the FRESH observation frame; positive is left. Tangents are derived
geometric segment directions, **not native TIC-VLA yaw**; tiny segments can
produce large tangent changes. Native full curves contain exactly 30 points;
observation markers are separate.

Primary inspection excludes every pair whose FRESH observation **or application**
is at/after first nonfloor contact. Post-contact metrics are exported separately.
The common physical approach interval is cart-frame forward progress
`[-(cart_long_half + robot_radius + 3*1.5), cart_long_half + robot_radius]`, using
the reference speed for **both** runs. The chosen pair has largest maximum
absolute lateral revision in this interval before contact, with earliest pair
breaking ties. This is descriptive, without a hard-case threshold. One-metre
progress bins also compare the runs; same C index is not the same location.

Cart clearance uses a conservative enclosing Nova disk (radius 0.606982542 m)
against the frozen oriented cart footprint. Negative proxy clearance is not
measured penetration. Contact evidence is saved PhysX contact spans and points.
Centre-plane crossing, downstream footprint/disk clearance and first cart or
structural contact are separate. Complete bypass requires downstream clearance
before any contact and a geometrically lateral crossing. A horizon that merely
stops short of the cart is not a safe bypass. Candidate FRESH bypasses must also
clear enabled Hospital collider AABBs intersecting body height; this conservative
geometric screen is not a new PhysX preflight or a navigation success claim.

Actual speed is measured world XY speed, with body-forward velocity, body yaw
rate and measured yaw exported separately. Wall-clock action latency,
observation-to-application latency, pending simulation time and measured robot
transport are separate clocks/quantities. Bootstrap is reported separately.
Visibility retains the frozen semantic HIDDEN/MARGINAL/CLEAR criterion and
recounts archived 1920x1080 masks; visibility does not prove model understanding.

## Results

**Runtime PASS; navigation outcome OTHER STRUCTURAL CONTACT BEFORE CART.**
The one authorized execution completed in 187.922 wall seconds with 48 finite
predictions, 1,527 measured ticks and continuous OLD retention PASS. Startup
probe returned from Isaac 6 SimulationApp and shut down normally (14.336 s).
Strict checkpoint load passed in the separate Isaac 5 Python worker, with
`simulation_app_started=false`. No retry or second model run occurred.

| Measured quantity | Reference 1.5 | Reduced 1.2 |
|---|---:|---:|
| Predictions / last observation [sim s] | 48 / 24.866668 | 48 / 24.933335 |
| End of measured execution [sim s] | 25.366668 | 25.433335 |
| Pre-contact XY speed median / mean [m/s] | 1.500406 / 1.263134 | 1.200275 / 0.984503 |
| Pre-contact body-forward median [m/s] | 1.500071 | 1.199969 |
| Pre-contact absolute body yaw rate median / max [rad/s] | .110373 / .431761 | .076345 / .630675 |
| All-run XY speed mean [m/s], includes stalls | .777113 | .570723 |
| Travel distance [m] | 18.990498 | 12.985102 |
| Minimum cart-center distance [m] | .642220 | 6.599683 |
| Minimum cart-footprint clearance proxy [m] | -.560424 | +5.439693 |
| Final world XY [m] | (-1.575922, 12.202748) | (3.233176, 12.226213) |
| First cart contact [sim s] | 13.333334 | NONE |
| First structural contact [sim s] | 18.983334 | 11.833334 |
| Cart / structural / total contact spans | 11 / 18 / 29 | 0 / 30 / 30 |
| Cart centre-plane crossing | NO | NO |
| Complete downstream passage | NO | NO |
| Classification | CONTACT BEFORE BYPASS | OTHER STRUCTURAL CONTACT BEFORE CART |

The slower robot hits `/Root/Geo_M2_TrimDoor5/Geo_M2_TrimDoor/Geo_M2_TrimDoor`
with the right wheel: robot XY (3.781647, 12.065835), first reported contact
XYZ (3.773311, 12.329049, .158962) m, **7.143038 m from cart center**.
Reference first cart-contact robot XY is (-2.670712, 11.099033), contact XYZ
(-2.770960, 10.846304, .173048) m; its later structural hit is `Geo_M2_TrimDoor6`,
robot XY (-1.761368, 12.078021), first contact XYZ
(-1.829004, 12.334430, .159007) m. All reported points/spans remain exported.

This is not merely an insufficient 48-chunk *duration*: a different structural
contact prevents approach. No cart contact is therefore **not** successful
avoidance or evidence of delayed cart collision. The trajectory figure shows a
different corner approach, but these data do not isolate why it occurred.

### Observation opportunities and timing

| Quantity | Reference 1.5 | Reduced 1.2 |
|---|---:|---:|
| First VISIBLE | C11, 6.366667 s | C13, 7.433334 s |
| First CLEAR | C12, 6.866667 s | C19, 10.433334 s |
| Cart-center distance at first CLEAR [m] | 9.410797 | 8.358877 |
| CLEAR to first nonfloor contact [sim s] | 6.466667 | 1.400000 |
| Observations before first contact | 24 | 21 |
| Observations from first CLEAR through last pre-contact observation | 13 | 3 |
| CLEAR to first cart contact [sim s / travelled m] | 6.466667 / 9.686013 | N/A: never reaches cart |
| Action-call wall latency median / max [s] | .056448 / .118210 | .053239 / .172958 |
| Observation-to-application wall latency median / max [s] | .533462 / .583613 | .496977 / .665589 |
| Pending simulation time median / max [s] | .100000 / .166667 | .100000 / .216667 |
| Pending robot XY transport median / max [m] | .125059 / .250114 | .018911 / .260221 |

Latency/transport aggregates exclude bootstrap and include later stalled
observations; their medians are not speed-matched comparisons. All per-request
clocks, signed pending yaw change, native lookahead and actual commands are in
the observation tables. Bootstrap action latencies are 1.826505 / 1.898475 wall s,
application latencies 2.326355 / 2.390312 wall s, and pending simulation durations
1.866667 / 1.933333 s. C1 is t=0; C2 follows bootstrap at those simulation times;
C2 onward has measured gaps 0.500000026 s. Exact timestamps are exported rather
than assumed to be `C_index / 2`.

### FRESH geometry, before versus after contact

| Quantity | Reference 1.5 | Reduced 1.2 |
|---|---:|---:|
| Strictly pre-contact adjacent pairs | 23 | 20 |
| Pre-contact aligned RMSE median / max [m], all locations | .423018 / 1.186607 | .286075 / 1.158325 |
| Pre-contact max-absolute lateral revision median / max [m] | .086865 / .571270 | .088251 / .281055 |
| Pairs in common cart approach interval before contact | 6 | **0** |
| Post-contact pairs, excluded from primary interpretation | 24 | 27 |
| Post-contact RMSE median / max [m] | .108587 / 1.109488 | .217508 / .808136 |
| Pre-contact OLD-conflict -> FRESH cart-clear pairs | 0 | 0 |
| Valid FRESH avoidance pair | NO | **INSUFFICIENT** |

Whole-route medians combine different locations and are not evidence that slower
speed reduces relevant revision. In the fixed cart progress bin [-9,-8) m there
are 10 reference / 18 slower pairs (RMSE medians .445939 / .286075 m); [-8,-7)
has 2 / 2 (.524121 / .273776 m). Upstream progress is not one-to-one route arc
length while turning. **There is no slower pre-contact sample in the cart
approach interval [-5.657872, +1.157872] m**, so no matched near-cart comparison
or increase in useful near-cart replanning opportunities can be established.

The preserved primary reference inspection is **C18 -> C19**: RMSE .433020 m,
lateral mean/max .098864/.212837 m, near/far RMSE .386684/.461333 m, OLD/FRESH
cart clearance +2.532526/+1.749581 m. The slow run has **no eligible approach
inspection pair**. Its first-CLEAR C18 -> C19 is displayed only as explicitly
labelled upstream context: RMSE .347648 m, lateral mean/max .086924/.269426 m,
near/far RMSE .272156/.389939 m, OLD/FRESH cart clearance
+6.292366/+5.655790 m. Both predicted horizons stop short of the cart. Positive
clearance here is not an avoidance trajectory.

At that slower C19 the native lookahead index is 14 (zero-based array index),
point (1.0625, .23828125) m, command (v,w)=(1.2,.361412576), delta_v=0 and
delta_w=+.062599148 rad/s. Application is .100000005 sim s after observation,
with .120019 m measured pending translation. Reference C19 uses index 13,
(1.1640625,.07861328125) m, command (1.5,.159656400), delta_w=-.003850049.
These same-index observations occur at world XY (5.117640,11.107266) versus
(1.746539,11.233105), cart distances 8.358877 versus 5.005636 m. They are not a
matched physical comparison. Derived tangent mean/max differences are
40.153514/149.914996 versus 49.470497/175.096271 degrees; repeated/short native
segments make these much less interpretable than positional geometry.

**Decision: insufficient evidence for improved cart avoidance.** The three
intended cart outcomes (delayed collision only, revised-but-colliding FRESH,
safe FRESH bypass) cannot be distinguished in the slower run because it hits a
different structure first. Do not label it a safe bypass or a reconciliation
candidate. Next **single** recommended experiment, not executed here: the same
max_v=1.2, 48-prediction E16 lights-on run **without the inserted cart**, on the
current host, to check whether the upstream corner/trim failure also occurs in
the speed-matched no-cart control. Do not simultaneously extend the budget or
tune angular control.

## Figures and replay

Final derived directory: `outputs/e16-c26-speed-analysis-20261011-04`.
Each stem has PNG and PDF:

- `actual_world_xy`: full measured paths and contact points.
- `cart_contact_zoom`: cart plus the earlier upstream structural collision.
- `actual_speed_time`: measured XY/body speed, body yaw rate and yaw.
- `cart_distance_clearance_time`: distance, conservative clearance and progress.
- `precontact_old_fresh`: reference approach peak and explicitly upstream slower
  first-CLEAR context; full native-world curves remain separate from actual path.
- `observation_locations_clear`: all 48 observation markers per run and CLEAR.

CSV/JSON contains both runs' 47 adjacent pairs, all observations/visibility,
progress bins, full world curves, contact locations, dense executed path and
summary. The dense path directly references the unchanged `robot_state.csv`
in each source run. Intermediate analyses -01/-02/-03 are retained; fallback
labelling and plot layout were refined without changing primary selection or
numeric results. Six final PDFs are rendered and visually reviewed.

The existing saved-only viewer was verified with analysis -03 at
`outputs/e16-c26-speed-gui-20261011-02`: GUI_READY at t0/C1, all pose/visibility/
OLD/FRESH/application/contact navigation checks pass, physics stopped,
model calls 0, new semantic captures 0. That window was closed normally before
the last annotation-only layout adjustment. Reopen the final evidence with:

```bash
bash scripts/isaac6_python.sh research/view_reveal_window_search.py \
  --search-dir outputs/e16-c26-speed-analysis-20261011-04/viewer_data \
  --output-dir outputs/e16-c26-speed-gui-REVIEW --self-test
```

Cyan is the slower measured path, gold the reference cart run, grey the original
no-cart route, orange OLD and purple FRESH; the cart remains fixed. First-CLEAR
is context, not a qualified candidate. No contact-free route is invented.

## Validation and provenance

**13 focused tests PASS**. Independent checks reproduce all 96 commands/filter
states/lookahead indices, all 96 masks/classifications and exact observation
poses, native shape/frame/units and nominal overlaps. The model/checkpoint hashes
match both saved runtime manifests. All 1,527 new cart-state rows retain the
original exact world pose. Existing 700 protected files remain unchanged.

The receipt binds 108 code files and 601 source/authorization/weight files; all
were verified immediately after execution. Later saved-only analyzer changes
corrected GUI reference context, explicit missing-approach labelling and layout.
The additive config diff guard was also hardened to detect missing/extra null
keys, without changing this run's frozen config. Both pre-execution source
versions are archived under setup `executed-analysis-source/` and verified
against receipt hashes; no executed simulation/controller/model source changed.
The final analyzer's own hash and source evidence manifest are in derived
metadata. Final integrity checks and Git receipt are in the setup folder.

No upstream source modification, installation, driver change, model training,
cart preflight rerun or navigation retry. Old C26 report and raw evidence are
preserved. The original strict C26 analyzer guard remains intact; speed
comparison has its own additive module.

## Reproduction

The authorized run ID is consumed once; it cannot be rerun. Saved-only comparison:

```bash
MPLCONFIGDIR=/tmp/tic-vla-mpl python3 research/analyze_e16_c26_speed.py \
  --reference-run outputs/nova-e16-c26-cart-20261011-01 \
  --run-dir outputs/nova-e16-c26-cart-speed12-20261011-01 \
  --preflight-dir outputs/e16-c26-preflight-20261011-01 \
  --output-dir outputs/e16-c26-speed-analysis-REVIEW
```

All destinations must be fresh. Source run files are never written by analysis.
Focused tests cover the one-parameter guard, source/config/run-bound new scope,
destination preservation and contact/passage classification, alongside original
C26 and controller source-parity checks.

## Limits

One run per speed, 48 predictions each, different closed-loop/model histories,
unchanged failed bypass gate, conservative geometry proxies. The host was
rebooted between reference and new experiments: NVIDIA 595.84 -> 595.99.02 and
Linux 7.0.0-34 -> 7.0.0-38; the agent changes neither. This is an additional
confound, despite exact config parity except speed. No causal speed benefit,
official TIC-VLA performance, graph, correspondence or reconciliation validation
is established. More time before collision alone is not avoidance improvement.
