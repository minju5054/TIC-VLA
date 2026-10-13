# Current obstacle avoidance failure analysis

The measured failures have two immediate forms: TIC-VLA sometimes fails to
complete the prescribed left turn before reaching an added obstacle, and the
actual C26 cart run continues forward without producing a clear bypass. The
controller converts those native predictions into movement without checking
obstacle distance or braking for an obstructed path. These observations explain
how contact occurs. They do not isolate a single underlying cause in the model,
visual input, reasoning state or adapted execution conditions.

This analysis uses saved simulation records as of **2026-10-13, Asia/Seoul**.
It adds no navigation trial, model call, simulator step, parameter change or
reconciliation. The latest adapted positive control contains no added human;
its conditional human stage was not executed.

## Measured outcomes under different conditions

| Condition | Measured result | Implication |
|---|---|---|
| Original no-cart 2 Hz research baseline | Left corner and goal region entered; no nonfloor contact | The existing execution path can navigate this route |
| Six fresh repeats of the same 2 Hz configuration | 3/3 NO_HOOK and 3/3 ZERO_HOOK complete the corner, enter the goal region, and remain contact-free | Do not describe the original configuration as consistently failing |
| C26 cart, 1.5 m/s cap, 2 Hz | Cart contact at 13.333334 sim s; no complete bypass | An actual obstacle-conditioned navigation failure |
| C26 cart, 1.2 m/s cap, 2 Hz | Structure contact at 11.833334 sim s, 7.143038 m from cart centre | This trial fails before the cart; absence of cart contact is not avoidance |
| No-cart 1.0 m/s diagnostic, 2 Hz | Left corner not completed; structure contact at 21.016668 sim s | Lower speed alone did not restore the route in this realization |
| Latest paperlike 10 Hz simulation target, no added human | Left corner and goal not reached; first structure contact at 16.683334 sim s; 85 s timeout | Basic navigation prerequisite failed; human avoidance remains unmeasured |

The six repeat trials and the latest paperlike trial use materially different
execution conditions. Restoring nominal action cadence, slew/recovery and
official wheel-command constants changes several factors together; comparing
their outcomes does not identify the effect of any one factor.
The original research runs use their strict 2 m goal-region criterion; the
latest positive control uses an inclusive 1.5 m criterion. Neither equivalence
of these gates nor official benchmark stopping behavior is assumed.

Sources: [repeatability](E16_ZERO_DELAY_REPEATABILITY.md),
[C26 actual run](NOVA_E16_C26_CART.md#authorized-unchanged-c26-run),
[C26 speed diagnostic](NOVA_E16_C26_SPEED_ABLATION.md),
[no-cart speed diagnostic](NOVA_E16_NO_CART_SPEED10.md), and
[latest positive control](TICVLA_HUMAN_AVOIDANCE_POSITIVE_CONTROL.md).

## The latest failure starts with incorrect steering at the corner

Rechecking dense measured states against the existing world-Y corner bounds
gives the following sequence. Positive angular velocity turns left; negative
turns right. The prescribed hallway extends west after the northbound approach.

| Event in latest paperlike run | Sim time s | Measured world XY m | Applied angular command rad/s | Measured body yaw rate rad/s |
|---|---:|---|---:|---:|
| First crossing of trigger-south Y 8.707836 m | 5.766667 | (5.881681, 8.717815) | -0.249297 | -0.243848 |
| First crossing of corner-entry Y 9.457836 m | 6.350000 | (5.572794, 9.462668) | -0.302143 | -0.264967 |
| First crossing of corner north limit Y 11.741243 m | 8.066667 | (5.044616, 11.763584) | -0.422323 | -0.408532 |
| First recorded brightness-profile failure | 10.400001 | C105 RGB | — | — |
| First structural contact | 16.683334 | Outside the intended corner passage | — | — |

At corner entry the active prediction is C64. Its native lookahead left
component is negative, and the actual robot turns right. The successful
historical reference enters the same world-Y corner band with a positive
command **+0.285112 rad/s** and measured yaw rate **+0.298679 rad/s**. Its
entry XY is (5.516770, 9.468385) m, but yaw, image history, reasoning state,
cadence and other execution conditions differ. This is a comparison at a fixed
physical cross-section, not identical-input causal matching.

The latest robot therefore misses the instructed left corridor before its
first contact. At C61, sim t=6.0 s, reasoning text already claims the left turn
is completed although measured yaw is only 1.9506 rad, about 111.8 degrees
from world +X. C66 and later descriptions repeat that claim. This is evidence
of disagreement between generated descriptions and measured progress. It does
not establish that the text caused the native action: the action head also
uses the current image and robot state, and successful repeat runs sometimes
use similarly misleading wording.

Archived [C64 RGB](../outputs/paperlike-human-control-20261012-01-stage-a/raw/observations/rgb_000064.png)
shows the stair junction and left corridor with usable illumination. Its
Rec.709 luminance p95 is **195.1364/255**; C81 p95 is **199.2086/255**.
The first recorded full brightness-profile failure at C105 occurs after the
robot has crossed the corner's north bound. Thus poor illumination is not
established as the initial route-loss cause. Later
[C167 RGB](../outputs/paperlike-human-control-20261012-01-stage-a/raw/observations/rgb_000167.png)
is a dark close-wall view with little route context, plausibly making recovery
harder. No lighting intervention isolates that hypothesis.

## The C26 cart run lacks a safe response before contact

The actual cart becomes CLEAR at **C12, 6.866667 sim s**, with current proxy
clearance +8.193081 m. First cart contact is **13.333334 sim s**: a
**6.466667 s** interval, with 13 observations from C12 through last precontact
C24. This is not a measured last-instant reveal.

Among the 23 precontact adjacent pairs, 22 FRESH curves do not reach the
cart-centre plane and one crosses within the inflated cart span. None provides
a complete clear north/south bypass under the existing test, and none changes
OLD conflict to FRESH clear. A prediction stopping short of the centre plane
cannot be called a successful bypass.

Every one of the **24 precontact requests, including bootstrap, commands
1.5 m/s**. Last precontact C24 is observed at 12.866667 s and applied at
12.966667 s. Its lookahead is **(1.109375 forward, -0.053955 left) m** and
angular command **-0.064713 rad/s**. The nominal command ratio |v/w| is
**23.179335 m**, indicating weak turning at that instant; it is not a measured
path radius. First contact follows application by about 0.366667 sim s.

The [actual C24 RGB](../outputs/nova-e16-c26-cart-20261011-01/diagnostics/rgb_000024.png)
shows the stationary cart prominently in front of the camera. The returned
reasoning instead describes a blue bed on the right and proposes creeping and
stopping. Returned background text can describe earlier generation images;
it is not a guaranteed caption of C24's current image. That semantic emphasis
and the unchanged forward command suggest
insufficient obstacle-conditioned planning or grounding. They do not prove
that the network failed to detect the cart, reveal its attention, or identify
the consumed latent KV state.

## The controller preserves motion without an obstacle clearance check

Unchanged upstream [lookahead code](../DynaNav/behavior/nova_carter_test_ticvla.py)
and the additive [controller](../research/control.py) choose a native point near
1 m of polyline arc length. Speed depends on curvature:

```text
kappa = 2 * lookahead_left / lookahead_distance^2
v = min(max_v, max_w / (abs(kappa) + 0.001))
w = clip(0.5 * v * kappa + 0.8 * filtered_bearing, -max_w, max_w)
```

Except for a lookahead distance below 1 mm, this normal prediction-to-command
mapping does not infer stopping from a short horizon or from reasoning words
such as “slow” or “stop”. It has no obstacle-distance, swept-footprint or
time-to-contact input. The source separately provides stuck-triggered backup
and goal/lifecycle handling; those do not constitute an obstacle braking check
in this mapping. In the latest run the first backup starts at 24.9 sim s,
after structural contact at 16.683334 s.

This source observation provides a mechanism by which an unsafe or weakly
turning prediction becomes continued forward motion. The measured C26 commands
show that mechanism in operation. It does not by itself explain why the
network supplied those predictions.

Independent scalar reconstruction agrees with all **632 saved target commands**
and lookahead indices from the latest run (536), C26 run (48) and historical
baseline (48), with maximum numerical error **1.11e-16**. The latest run's
previous verification also checked all slew updates. Consequently, these
failures are not explained by an observed mismatch between the documented
equations and the saved target commands. This does not validate every aspect
of physical equivalence with official DynaNav.

## Passage geometry limits the C26 interpretation

C26's unchanged preflight found **0/10** passing robot-centre strips on each
side with the enclosing Nova disk radius **0.606982542 m** and additional
0.05 m structural margin. North strips intersect structural bounds; the nearest
south strip has only +0.039392 m structural-bound clearance, below the margin.
The actual run was separately authorized despite that failed gate.

There is therefore no demonstrated feasible local bypass for C26 under the
existing conservative checker. This is not proof that every continuous path
is impossible, but C26 cannot isolate failure to choose between two validated
safe alternatives. Its collision still shows that the implemented pipeline did
not stop before the obstruction.

Geometry cannot explain every cart failure. The earlier
[C19 cart](NOVA_E16_C19_CART.md) and
[C19 plus world Y](NOVA_E16_C19_PLUS_Y_CART.md) pass a south-side local bypass
check, yet their executions contact northern Hospital structure. Those results
show that geometric feasibility alone does not ensure correct navigation.

## Timing and checkpoint hypotheses remain separate

The usual 2 Hz continuous path retains the OLD command while action inference
is pending. C24 in the C26 run advances 0.1 simulation seconds before
application, with about 0.15 m of robot transport. Its action-call wall time,
total observation-to-application wall time, background reasoning age and
simulation transport are distinct quantities. The native curve remains
anchored at its observation pose.

The latest paperlike run freezes physics during every action/capture. All
536 observation/application pose and simulation-time pairs are identical,
yet navigation fails. Robot transport during action inference therefore cannot
be a common explanation for all these failures. The unchanged asynchronous
reasoning lifecycle, historical context and changed simulation-to-wall pacing
remain possible contributors; the exact consumed KV identity and worker-ready
time are unavailable.

The public checkpoint strictly loads and produces finite native predictions.
Its SHA256 is
`376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`;
the base weights SHA256 is
`a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`.
Identity with the paper's final RL evaluation artifact is unresolved. Lightning
epoch/validation metadata alone does not prove absence of RL or make this the
cause of failure. Strict-load PASS is not navigation-performance validation.

The platform also differs from the official Isaac 5 baseline: Isaac 6 physics,
adapted lifecycle/lighting and seed 36. The latest path uses official command
wheel radius .152 m on the unchanged .14 m audited asset; measured forward
speed near corner entry is about 1.382 m/s for a 1.5 m/s command. These are
documented differences, not individually isolated causes.

## What the evidence supports next

The immediate failures are **incorrect corner steering** and **insufficient
cart response followed by continued forward commands**. The cause of the
network's different outputs remains unresolved. Illumination after route loss,
reasoning/history/pacing, checkpoint parity and physical adaptations are
specific hypotheses, with different evidence strengths.

The first uncertainty to resolve is the public checkpoint's identity relative
to the final paper evaluation artifact. After that, a single-variable control
on an already feasible route can distinguish model/input behavior from command
execution. A blocked C26 bypass should not be used as that control. A saved
byte-identical-input repeatability check can examine output variability, but
cannot reconstruct unknown historical KV tensors or prove later navigation
causation.

There is no present basis to attribute these contacts to OLD/FRESH geometric
seams or to promise a reconciliation remedy. The prior
[smoothing audit](E16_CHUNK_SMOOTHING_AUDIT.md) measures nominal-time geometry
on successful no-cart runs, not avoidance recovery. Human avoidance and
official TIC-VLA navigation performance remain unvalidated in this adapted
negative control.

## Frames clocks and validation

World positions are metres, +Z up. Body native axes are +forward and +left;
yaw is radians CCW about world +Z from world +X. Saved quaternions use
**w,x,y,z**. Native `(30,2)` offsets share the observation body origin and
contain no yaw or timestamp channel. This audit introduces no world transform
or trajectory conversion. Prior cited curve analyses project body to world
with `p_world = xy_observation + R(yaw_observation) p_native`, never the
application pose, and assign the source's nominal +0.1…3.0 s times explicitly.

All event times quoted above are **simulation seconds**. Wall action start,
ready, poll/detection and command application are recorded separately. Returned
reasoning text and completion-poll steps do not provide exact worker completion
or consumed-KV identity. Applied targets and measured body velocities are kept
separate.

Saved-only evidence: **`outputs/obstacle-failure-analysis-20261013-02/`** contains
the scalar audit script, all reconstructed commands, selected original RGB and
reasoning references, exact dense crossing rows, summary, input SHA256 manifest
and validation receipt. **1305 input files were verified unchanged**;
632 native arrays and command reconstructions PASS. Existing raw/derived
outputs were preserved, and `DynaNav/` has no diff against upstream. No new
navigation evidence was generated.

Analysis starting local SHA:
`f99f6d2f29f24e93ff02e19161642353416f1748`, clean `hardcase-probe`;
upstream SHA: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`.
Executed navigation provenance is retained in each run's original metadata,
runtime configuration and dependency snapshots, bound by this audit manifest.
Analysis runtime is existing system Python **3.12.3**, NumPy **1.26.4**, Pillow
**10.2.0**, with no dependency installation or environment change. Exact command:

```bash
python3 /tmp/ticvla_obstacle_failure_audit_20261013.py
```

The archived `audit.py` rejects its existing output directory; reproducing this
analysis requires a new output ID. The final documentation commit is recorded
separately after creation to avoid a self-referential Git SHA.
The initial derived audit `...-01` is preserved. Final `...-02` reads exact
corner bounds from each frozen configuration, corrects the luminance-source
comment and records the inspected Pillow version. The initially rounded north
bound differs by approximately 1.53e-10 m and changes no selected tick or result.
Final audit runs while this new report is untracked; its receipt records that
dirty status. Original experiment provenance is unaffected.
