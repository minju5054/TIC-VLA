# Hospital obstacle-reveal pair acquisition: bounded preflight stop

**NO QUALIFYING REVEAL SCENARIO.** All 12 frozen placements passed the physical
and connected local-bypass checks, but none passed the required first-reveal
conditions. **No obstacle-conditioned inference, new navigation, or Graph solve
was executed.** There is no actual obstacle-response OLD/FRESH pair to classify
as successful avoidance or a reconciliation-needed hard case.

Started clean on `hardcase-probe` at local/freshly fetched origin
`b693ebffb84c19309170a869bf34037642dc94f9`; upstream/main
`9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Exact final commit/push verification
is in `outputs/obstacle-reveal-setup-20261011-01/git-finalization.json`.

## Route, scope and preregistration

Reference: `outputs/nova-e16-hospital-lights-baseline-20261010-01`, 48 finite
native `(30,2)` chunks, measured executed path, zero recorded non-floor contact.
The original successful no-cart route is reused, without resampling. Hospital
E16 starts at `(7.38,1.49,0.11)` m, yaw `pi/2`, with goal
`(-12.23,10.19,0.01)` m. The instruction remains:

> Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.

The loaded Hospital USD and all 2,161 Hospital collider records match the
previous audit. Lighting is the **existing validated E16 lights-on fixture
profile**: no new lights, native-light changes, exposure changes, occluders,
wall edits or camera changes in this task. This is not a claim that the prior
fixture-light additions were part of the unmodified native Hospital USD.

`configs/research/obstacle_reveal_acquisition.json` and the exclusive
`outputs/obstacle-reveal-qualification-20261011-01/protocol.json` fix the
candidate list, source/code hashes, method and stop rule before any query or
render. Candidate centres are saved C16/C17/C18/C20 observation XY plus
0/.4/.8 m along the left normal of the measured C(k−1)→C(k+1) chord. The cart's
long axis follows that chord. These cover four proximal stations around the
inner corner, with centreline and inward placements. No C19 or C26 placement
is repeated. Candidate IDs are not request IDs.

Exactly 12 candidates were qualified, with no placement refinement or
additional route search after the result. No alternative route was selected;
the previously tried E21 route has a failed useful-RGB baseline gate and was
not promoted to a valid reference. The current finite search is not evidence
that every Hospital corner is unsuitable. The explicit no-candidate stop
prevents forcing an E16 obstacle run despite a failed reveal gate.

## Obstacle, frames and physical checks

One independent candidate scene at a time references the existing
`/Root/SM_SupplyCart_01e_21` as `/World/E16Cart`, preserving its static triangle
collision, mesh and material. Asset: Hospital `Props/SM_SupplyCart_01e.usd`,
SHA256 `882bbdf100c171307303e920c4c48af19899f1f72a1abf747a16d46226ea5be6`.
Hospital USD SHA256:
`83e5d606fbb2de873314379e0ff74900930735eb3ae99b312c9bf17db0033383`.
The original asset source is unchanged. Bounds are
`(-.46186652104854886,-.2624729097826872,~0)` to
`(.6399123239536806,.2649667299173757,1.1127104701240995)` m.

World units are metres, +Z up, yaw CCW from +X. Full measured robot quaternions
are w,x,y,z. USD matrices act on row vectors: camera-to-world = camera-to-body
× body-to-world. Each saved camera matrix is checked against the full measured
body pose, including roll/pitch. The cart root transform compensates the
source pivot, so its specified XY is the footprint centre and its bottom Z
is the queried floor. All 12 exact XYZ/yaw/matrices are saved, not rounded in
runtime artifacts.

Cart support and its enclosing oriented box are checked against actual static
PhysX geometry. Nova's unchanged conservative disk radius is
`0.6069825421953869` m. Connected bypass proposals begin and end on the measured
baseline path outside the inflated cart, with a continuous side strip and
entry/exit connectors. Both sides and fixed .05… .50 m extra offsets are
tested in fixed order. Full polyline-to-cart clearance uses exact segment
geometry; floor/static queries sample at ≤.05 m with an additional .05 m
static margin. Every candidate has a passing connected bypass with **+.05 m**
cart clearance. These are conservative geometric feasibility proxies, not
executed steering feasibility or proof of successful future navigation.

The static query stage never plays the timeline or steps navigation physics.
The renderer authors each candidate before its C1 capture, holds its transform
unchanged for its entire chronological capture sequence, then removes that
independent candidate scene. This is model-free placement evaluation, not
obstacle spawning, hiding, teleporting or moving during an executed run.

## Nominal conflict and authoritative visibility

Native rows are cumulative forward/left offsets in their **own observation
body frame**, projected by `world = observation_XY + R(observation_yaw) native`.
No cumsum, native yaw, switch re-anchoring or spatial correspondence is used.
Timing is source-assigned nominal `t_observation + .1…3.0 s`, not a native
timestamp channel. Each of 12×47 = **564** adjacent-pair proxies clips OLD to
the FRESH observation time with interpolation inside its native domain only.
An opportunity requires OLD segment conflict, positive current clearance,
positive nominal conflict lead and saved baseline application before conflict,
all on the same pair. These use no-cart OLD predictions and saved application
times; they are not obstacle-conditioned predictions or a future latency forecast.

All 12 candidates pass some such temporal opportunity and receive actual
1920×1080 Hawk RGB plus uncolorized semantic segmentation, using the original
camera and an explicit render completion barrier with no timeline advance.
Thresholds are unchanged: **HIDDEN=0 pixels**; CLEAR requires fraction ≥.002,
width ≥20 px and height ≥80 px; other positive masks are MARGINAL. No ray
classification substitutes for rendered evidence.

Capture proceeds from C1 through the **first CLEAR**, inclusive. That complete
prefix establishes C1, all earlier visibility and the immediate three
predecessors. Later frames cannot repair a failed first reveal, so they are
not rendered and remain **UNMEASURED**, not HIDDEN or held CLEAR. There are
**39 actual RGB/semantic captures**, with full semantic arrays, label maps,
binary masks and camera/observation records. No fabricated HIDDEN frames.

## Results

| Candidate IDs | Station / offsets | C1 | First visible | First CLEAR | OLD clearance at first CLEAR |
|---|---|---|---|---|---|
| 1–3 | C16 / 0,.4,.8 m | CLEAR | C1 | C1 | N/A: no preceding OLD |
| 4–6 | C17 / 0,.4,.8 m | CLEAR | C1 | C1 | N/A: no preceding OLD |
| 7 | C18 / 0 m | MARGINAL | C1 | C3 | +7.932574 m |
| 8 | C18 / .4 m | MARGINAL | C1 | C3 | +7.633883 m |
| 9 | C18 / .8 m | MARGINAL | C1 | C4 | +6.564832 m |
| 10 | C20 / 0 m | MARGINAL (11 pixels) | C1 | C7 | +6.111253 m |
| 11 | C20 / .4 m | HIDDEN | C4 | C8 | +5.373616 m |
| 12 | C20 / .8 m | HIDDEN | C6 | C8 | +5.160831 m |

Funnel: **12 fixed → 12 physical/connected-bypass pass → 12 some-pair temporal
pass → 12 rendered → 2 start-HIDDEN → 0 strict-qualified**. Multi-label
rejections: C1 not HIDDEN 10; immediate OLD not HIDDEN 12; three prior not
HIDDEN 12; no OLD conflict at first CLEAR 12 (including six with no OLD at C1).
Selection is null. Original ranking, had any passed, is increasing positive
switch margin, decreasing first-CLEAR fraction, decreasing bypass clearance,
then lexicographic XYZ. No outcome-dependent weighted score was added.

Diagnostic candidate **12** is displayed because it is the last fixed ID,
not because it is a selected qualifying scenario. Footprint centre:
**(1.2160522143047907,10.425799277299056,−1.4901161193847656e−7) m**;
yaw **3.120893363243896 rad**. Pivot-compensated root translation is
`(1.3050818533449333,10.425203340950763,−1.4901161192351457e−7)` m.

Its C1–C5 are HIDDEN; C6/C7 are MARGINAL with 746/1,986 pixels; C8 is first
CLEAR with 4,630 pixels, fraction .002232831790123457, bbox 68×161 px.
Thus the preflight reveal inspection is baseline **C7→C8**, not a valid
HIDDEN→CLEAR pair. At C8, t=4.800000250 s, current cart clearance is +6.454250 m
and OLD clearance +5.160831 m. There is no nominal conflict time or lead.
Its later baseline C17→C18 proxy has −.016518 m OLD clearance and positive
1.323272 s application-to-conflict margin, but **cannot replace C7→C8**.

## Stop, absent results and interpretation

Stage A failed. Stages B/C—one selected scenario freeze and one real
obstacle-conditioned run—were **NOT EXECUTED**. No actual first-CLEAR request,
cart-conditioned OLD/FRESH identity, FRESH bypass/clearance, lateral or tangent
revision, switch pose, latency, RAW seam, controller jump, contact or local
passage result exists for this task. They are unavailable, not zero or success.
No CASE A–F is assigned to an unexecuted run.

This is negative evidence for these 12 physical reveal proposals, and
insufficient evidence about actual TIC-VLA obstacle response or reconciliation
need. The no-cart reference can pass through an inserted diagnostic cart in
replay because the viewer replays old measured poses without physics. It is
not a collision measurement. No Graph, rigid correction, correspondence
validation, speed/controller tuning, latency intervention or navigation
performance claim follows.

## Artifacts, GUI and verification

- `outputs/obstacle-reveal-qualification-20261011-01/`: frozen protocol;
  raw static-query traces and 39 RGB/semantic captures; all 564 temporal
  proxies; derived candidate decisions; null selected scenario; summary.
- `outputs/obstacle-reveal-review-20261011-02/`: candidate CSV/JSON, exact plot
  coordinates and five reviewed PNGs: original baseline/candidates, connected
  bypasses, visibility versus temporal opportunity, diagnostic RGB sequence,
  and first-CLEAR versus later no-cart conflict. Review -01 is preserved;
  -02 fixes layout only. There is no invented new-run trajectory/seam figure.
- `outputs/obstacle-reveal-setup-20261011-01/`: logs, tests, independent
  validation, pre-edit viewer/report archives and Git receipt.

Saved GUI `outputs/obstacle-reveal-gui-20261011-01/GUI_READY.json` passed
controls, exact saved-pose replay, fixed cart, archived masks and stopped
physics checks. It opens paused at t=0/C1. Jump first visible=C6, OLD=C7,
first CLEAR/FRESH=C8; all are clearly **PREFLIGHT ONLY / NO CART-CONDITIONED
RUN**. Original no-cart RGB is the default; the toggle shows archived cart
preflight overlays when available. C9–C48 have no cart captures and explicitly
show visibility NOT MEASURED while retaining the original no-cart RGB.
Native-world OLD/FRESH overlays remain anchored to their original observations.

Exact one-line launch from the repository root (new destination required):

```bash
bash scripts/isaac6_python.sh research/view_reveal_window_search.py --search-dir outputs/obstacle-reveal-review-20261011-02/viewer_data --output-dir outputs/obstacle-reveal-gui-REVIEW
```

**61 focused tests PASS**, no skips. They cover candidate cap/pose/pivot,
zero-pixel HIDDEN, chronological first-CLEAR without substitution, nominal
conflict/no extrapolation, finite FRESH passage with beyond-obstacle support,
source immutability, exact original observation/application poses, existing
seam geometry, and unmeasured GUI tails. Synthetic cases are unit tests only.
Independent validation recomputes all 39 semantic masks/bboxes/states, camera
matrices, original world projections and 564 continuous-polyline clearances;
all source file and directory inventories remain unchanged. The initial
independent verifier incorrectly required a semantic label for fully hidden
objects; its failed assertion is preserved. The corrected verifier admits an
absent label only for zero-pixel frames; no raw evidence or rule was changed.

Protected evidence: **1,545 files unchanged**. Of 115 frozen code files, 114
remain unchanged; the existing saved-only viewer has a small, documented
post-preflight change for an optional title and incomplete visibility prefixes.
Its original version is archived and hash-verified. All existing runtime,
model/controller, continuous execution and Graph code remain unchanged. The
preflight policy and numerical implementation were not changed after results.
Isaac 6.0.1 static querying/rendering only; no Isaac 5 SimulationApp or inference
service launch, installation, driver or checkpoint change.

Delivery commits only code, config, tests, this report and an append-only work
log. Raw/derived outputs and figures remain ignored. The finite candidate cap
was exhausted; no further scenario or experiment is implemented here.
