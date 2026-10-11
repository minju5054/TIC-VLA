# Hospital E16 late-handoff stress experiment

## Question and intervention

Does retaining the accepted OLD command after a real FRESH response is available degrade the original no-cart Hospital E16 left turn? A navigation failure would not, by itself, establish a recoverable boundary or validate SE(2) reconciliation.

The experiment retains the exact config shared by historical A (`outputs/nova-e16-hospital-lights-baseline-20261010-01`) and B (`outputs/nova-e16-no-cart-15-repro-20261011-01`). It uses the controlled Isaac 6.0.1 Hospital wrapper and separate Isaac 5 Python model service, not an unchanged official DynaNav reproduction. No Isaac 5 SimulationApp, actor addition, controller tuning or model change. Native `(30,2)` predictions remain forward/left metres at their own observation body frame. Yaw is CCW from world +X, world Z is up, saved quaternions are wxyz; world projection is `p_W = t_obs + R(yaw_obs) p_native`. Native points contain neither yaw nor timestamps. Nominal +0.1…3.0 s timing comes from the source-confirmed target convention.

The sole intervention is one **artificial post-response application hold**, not increased model inference latency. The normal pipeline captures, submits, executes OLD during inference, consumes/validates FRESH, and computes the original controller command. The opt-in hook then advances an integer number of normal physics ticks with OLD before applying that unchanged FRESH. It never captures or submits during the hold. Normal `max(observation + 0.5 s, application)` scheduling resumes afterwards. This can change subsequent observation times and prediction history, which limits attribution of later failures.

## Frozen preflight and trial rules

Plan: `outputs/e16-late-handoff-plan-20261011-01/receipt.json`; saved-only detail: `preflight.json`. Setup, startup and tests: `outputs/e16-late-handoff-setup-20261011-01/`.

Before any new model execution, both saved references were validated. The proposed 1.5 m approach band includes C12 and C13 in both references. C12's change in angular command has opposite signs across references. The inner half of that band selects C13 in both, where FRESH consistently increases the existing left command. This pre-model refinement is frozen; it is not selected from new trial outcomes.

The trigger is the **first non-bootstrap FRESH observation** satisfying all of:

- `2.5521036832844546 < world X <= 7.986982542195387 m` (east of original exit, west of start X plus original conservative radius).
- `8.707836118855571 <= world Y < 9.457836118855571 m` (within 0.75 m south of original corner-entry lower Y).
- `sin(yaw) > 0` and measured world `vy > 0`.
- No measured corner entry at or before that observation. Entry uses the original spatial gate.

Request ID and prediction content are not runtime trigger criteria. Entry occurring during inference does not invalidate an observation that was genuinely before entry.

| Saved reference | Selected FRESH | Observation XY [m], yaw [rad] | OLD w → FRESH w [rad/s] | Native lookahead [forward,left] m | South distance to entry [m] |
|---|---|---|---|---|---|
| A | C13 | (5.812160, 9.096426), 2.199121 | 0.213668 → 0.285112 | (1.125, 0.168945) | 0.361410 |
| B | C13 | (5.864472, 9.169891), 2.081357 | 0.144015 → 0.196955 | (1.093750, 0.108398) | 0.287945 |

Both OLD and FRESH already command left: this tests delayed strengthening of a left turn, not opposing route choices. Both saved observations occur at 7.30000038 s, before measured corner entry (A 7.61666706 s; B 7.53333373 s). Their normal observation-to-application interval is 0.100000005 s. Even with the maximum extra hold, about 1.90 s of nominal FRESH horizon remains; that is not proof of physical feasibility.

Prespecified holds: 0, 0.25, 0.50, 0.75, 1.00 s = 0, 15, 30, 45, 60 ticks at 60 Hz. Two independent fresh trials per level, ascending, maximum ten, no retries. Both zero controls must pass original left-turn, west-hallway and goal-region gates without nonfloor contact; otherwise stop. Stop larger delays after two clear corner failures if both trials at the preceding level passed. Mixed results stay mixed. Runtime/inference failure also stops the sweep. No range extension.

## Implementation and evidence

Only existing research source changed: `research/continuous.py` adds an optional `application_hook=None`, a post-tick observation callback, a pre-application callback, and optional `late_handoff` metadata. No upstream `DynaNav/` source changed. With no hook, the control/capture/inference order remains the existing order. A zero hold has no additional physics ticks; disk logging adds wall time, so bitwise stochastic replay is not claimed.

Additions: `research/late_handoff.py` implements the hold and a separate saved-data validator; `research/hospital_late_handoff.py` freezes authorization/provenance and runs one gated trial; `research/analyze_late_handoff.py` analyzes saved evidence and plots comparisons; `configs/research/e16_late_handoff_stress.json` fixes protocol and metric windows. Historical validators are not weakened to accept held ticks.

`raw/late_handoff/received.npy/json` archive the validated native response, original observation anchor, OLD command/wheels and original FRESH proposal before holding. `hold.json` records requested/realized ticks, every hold tick, start/end actual poses and clocks, transport, and application/stop status. Applied requests retain normal raw records. Every pending and hold tick is checked against the correct OLD source, command and wheel targets. Actual nonfloor environmental contact during a hold stops the trial safely with the received response unapplied; missing future predictions are not fabricated.

Receipts bind the user instruction, exact baseline config, source evidence, rules, preflight, startup/tests, runtime code and model weight hashes. New trials have fresh exclusive directories. Simulation/model versions, checkpoint identity, source Git SHA, dirty diff identity and invocation are recorded. No previous raw run is modified.

## Metrics and interpretation rules

Original left-motion and spatial-turn gates are reused. Corner passage is the original west-of-exit observation plus the next observation, measured at that next application's time; qualifying actual-left-motion intervals are reported separately. Goal-region entry is strict XY distance < 2 m, separately from corner passage. The fixed 48-prediction harness does not test stopping at the goal.

Outcomes: A contact-free corner passage; B passage with nonfloor contact; C no valid west-hallway passage; D structural contact before passage; E missing trigger/incomplete observation; F runtime/inference failure. A contact-terminated diagnostic can be runtime-valid but navigation-failed.

The corner metric window starts at the first dense measured crossing of the trigger band's south Y and ends at passage, first nonfloor contact, or run end, whichever occurs first. Northward overshoot is `max(0, max(world Y) - original center_y_max)`; signed excess and absolute max Y are also saved. Distance traveled, applied angular command, measured yaw rate, curvature `yaw_rate/XY_speed`, and reciprocal absolute curvature are reported in that window. Curvature/radius are undefined when speed or absolute yaw rate ≤ 1e-6; they are measured geometry, not native yaw or a minimum feasible turn radius.

Minimum structural clearance is the original circumscribed robot disk versus enabled structural world AABBs intersecting robot height, using the existing proxy. Negative proxy clearance is not proof of actual penetration. Actual contact is reported independently from PhysX records using the existing nonfloor convention, `abs(normal.z)<0.5`, excluding internal robot contacts.

At the actual switch B, boundary position compares measured XY to nominal FRESH interpolation anchored at observation. Heading seam compares measured robot yaw (and separately executed tangent) with a **derived geometric FRESH tangent**, not native TIC-VLA yaw. OLD/FRESH overlap uses same absolute nominal future times, linear interpolation, no spatial matching/extrapolation. Full and post-switch editable overlap are separate. Actual inference wall latency, genuine pending simulation time, additional hold simulation ticks and actual OLD transport remain distinct.

AABB clearance, remaining native horizon and positive left command do not establish dynamic recoverability. No last feasible turning point, collision-free corrected trajectory, graph method or recovery controller is implemented or validated.

## Results: stopped at the zero-delay stability gate

**INSUFFICIENT BASELINE STABILITY.** Two new zero-delay trials produced one contact failure and one success. Both are runtime-valid, with 48 finite predictions each and the spatial trigger selecting C13. The prespecified repeatability gate failed, so **no 0.25, 0.50, 0.75 or 1.00 s trial was run**. There is no measured positive-delay effect, no critical-delay estimate and no claim of “no failure up to 1 s.”

Fresh raw runs:

- `outputs/nova-e16-late-20261011-01-d000-r1`
- `outputs/nova-e16-late-20261011-01-d000-r2`

The setup `sweep-stop.json` records the enforced stop; eight planned IDs remain unused. Trial 1 used plan `...-01`; trial 2 used amendment `outputs/e16-late-handoff-plan-20261011-02/receipt.json`. Between trials a saved-only JSON serialization bug (`numpy.bool_`) was fixed and the gated runner's prior-result filename changed to `late_metrics_v2.json`. The partial v1 file is preserved. The amendment verifies that the runtime hook, continuous loop, trigger, delays, exact baseline config and model/controller code remained unchanged. No trial was retried. After stopping, only saved-only report fields/plot readability were extended; final analysis metadata records that source state separately.

Final derived comparison: `outputs/e16-late-handoff-analysis-20261011-02/`; the earlier `...-01` output is preserved. The final comparison includes exact first qualifying left-motion positions and remaining-FRESH structural clearance proxies. Per-run `derived/late_metrics_v2.json` preserves the metrics used for the online sweep gate.

All times below are simulation seconds. Passage is undefined on failure, not zero. Overshoot uses the fixed original footprint-clear north gate; positive overshoot alone is not a contact label.

| Run | Outcome | Requested / realized hold [s] | Extra OLD XY / yaw [m / deg] | Max corner Y / overshoot [m] | Passage [s] | First nonfloor contact [s] | Goal entry [s] |
|---|---|---|---|---|---|---|---|
| Historical A | A | 0.00 / 0.00 | 0.0 / 0.0 | 11.226068 / 0.000000 | 10.883334 | — | 18.566668 |
| Historical B | A | 0.00 / 0.00 | 0.0 / 0.0 | 12.055090 / 0.313846 | 12.883334 | — | 19.050001 |
| 0.00s trial 1 | D | 0.00 / 0.00 | 0.0 / 0.0 | 12.124000 / 0.382756 | — | 9.550000 | — |
| 0.00s trial 2 | A | 0.00 / 0.00 | 0.0 / 0.0 | 11.935427 / 0.194183 | 12.900001 | — | 18.966668 |

Trial 1 first contacts `/Root/Geo_M2_DoorWall2Corner2_656/Geo_M2_DoorWall2Corner/Geo_M2_DoorWall2Corner/Section1` with `/World/Robots/Nova_Carter/wheel_right/Cylinder_03` at 9.550000498 s (tick 573). Measured robot pose is `(4.095304, 12.124000, 2.065245 rad)`; reported contact point is `(4.219769, 12.343957, 0.140450) m`. It never passes the original hallway gate or enters the goal region during the 48 predictions. The contact is later than the zero-duration hold, so the specifically requested **during-hold** abort condition is not invoked; the original fixed-length baseline continues. Post-contact motion is visible in the path figures but excluded from the corner metric window.

Trial 2 passes the original actual-left-motion and hallway gates without nonfloor contact and enters the goal region. Trial 1 has no qualifying contact-free left-motion interval under the original definition; this does **not** mean its angular command or measured yaw never turns left.

| Run | First qualifying left interval starts [s] | Actual start XY [m] | Corner travel [m] | Minimum structural disk/AABB proxy [m] |
|---|---|---|---|---|
| Historical A | 7.800000 | (5.330324, 9.670548) | 5.826612 | 0.497043 |
| Historical B | 8.800000 | (4.498935, 10.949729) | 8.877647 | -0.331979 |
| 0.00s trial 1 | — | (—, —) | 3.841290 | -0.400889 |
| 0.00s trial 2 | 8.316667 | (4.903199, 10.336637) | 8.853050 | -0.212316 |

Negative AABB proxy clearance also occurs in successful B and trial 2, illustrating why it is not an actual collision oracle. Curvature, radius, command and yaw-rate samples are retained in `derived/run_XX_corner.csv`; near-contact curvature is descriptive response, not a recovery radius.

### Spatially selected handoffs

Every selection happens to be C12 → C13, but the rule and cross-run comparison use actual progress/poses, not matching request indices. For both new trials the extra hold is exactly **0 ticks, 0 s**, with **0 m, 0 deg** additional OLD transport. Genuine inference still advances physics for 0.100000005 s at C13, moving approximately 0.1501 m. These are different quantities.

| Run | Boundary position [m] | Robot / derived FRESH heading seam [deg] | Full future RMSE [m] | Mean / max absolute lateral revision [m] | OLD w → FRESH w [rad/s] | Δw [rad/s] |
|---|---|---|---|---|---|---|
| Historical A | 0.023403 | 0.823607 | 0.404870 | 0.109425 / 0.292575 | 0.213668 → 0.285112 | 0.071444 |
| Historical B | 0.026085 | 1.498320 | 0.380308 | 0.097731 / 0.339973 | 0.144015 → 0.196955 | 0.052940 |
| 0.00s trial 1 | 0.018778 | 2.524453 | 0.438907 | 0.048926 / 0.170761 | 0.103319 → 0.142540 | 0.039221 |
| 0.00s trial 2 | 0.020343 | 1.915448 | 0.408601 | 0.095818 / 0.253844 | 0.124048 → 0.188444 | 0.064396 |

All selected FRESHs retain positive native lookahead-left and positive angular command; Δv is zero. Each has about 2.90 s of nominal horizon at application. Trial 1's genuine action-call wall latency is 0.059978 s; trial 2's is 0.055973 s. Neither was artificially delayed. Original native/world curves, observation/detection/application poses and wall/monotonic clocks are archived in raw records and final `derived/run_XX.json`.

Saved post-selection evidence helps localize the failed behavior: trial 1 C14/C15 observe at 7.800000/8.300000 s, with native lookahead `(1.109375, -0.052246)` / `(1.125000, -0.103516)` m and original applied `w=-0.048176/-0.137362 rad/s` at 7.900000/8.400000 s. Measured yaw rate also becomes negative before contact. C16 is weakly left (`w=0.048792`), and C17 increases left to `0.475508 rad/s` at 9.400000 s, shortly before contact. In successful trial 2, the subsequent spatial corner approach retains positive w (0.187197, 0.237494, 0.360242 rad/s at observations Y=9.755373, 10.336637, 10.863919 m). These are each run's measured locations/times, not an assertion that equal C IDs are identical states. The failed execution follows later rightward proposals; why the model/history produced them remains unresolved. No artificial delay was involved.

At the selected switch the circumscribed-disk structural proxy is positive in all four runs; it cannot prove future reachability. Remaining-FRESH pointwise AABB proxy minima are recorded separately from actual contact and from the measured-path minimum. No trajectory is re-anchored at the switch.

## Research answers

1. **Does delaying FRESH worsen overshoot/turning?** Unanswered. Positive delays were gated off. Even with no extra hold, outcomes and overshoot vary.
2. **First observed failure delay / interval?** A failure occurs at the tested lower endpoint, 0 s, together with a success at 0 s. A success-to-failure delay interval is not identified; 0.25–1.00 s is unobserved due to the stability stop. This is not right-censored success through 1 s.
3. **Repeatable failure?** No: the two new zero controls disagree. Historical A/B are successful references, not interchangeable randomized replicates. Four runs do not estimate a reliable failure probability.
4. **Last physically recoverable turning point passed?** Unresolved. No delayed switch was executed, and no dynamic reachability calculation was performed. A positive current clearance or remaining horizon is insufficient proof.
5. **Original FRESH left intent survives?** Yes at both observed zero-delay switches by positive lookahead-left and original angular command. Survival after a positive hold was not tested. Trial 1's selected angular command is weaker than the successful examples, an association rather than a demonstrated cause.
6. **Longer OLD versus later prediction history?** Neither can explain this as an *artificial-delay effect*, since no extra OLD was executed. Later FRESHs in the failed run actually propose/apply right turns before contact; this is observed downstream behavior, not proof of why the model produced them. Variation in stochastic model/reasoning history, asynchronous wall timing and simulation execution remains unresolved. Zero-hook logging preserves physics ticks, not identical background reasoning timing.
7. **Can switch-only correction recover?** Unproven. The failed run has a small selected boundary seam (1.88 cm) and positive left intent, yet later collides; this is evidence against equating seam magnitude with navigation failure, not a validated reconciliation opportunity. Native prediction validity, later control evolution and physical feasibility need separate evaluation before graph work.

**Decision: stop the late-handoff sweep; insufficient baseline stability.** A real no-delay navigation failure has been preserved, but no delay-dependent hard case or reconciliation success has been established. The next single recommended experiment is a separately authorized, fixed-config **zero-delay repeatability study** that archives asynchronous reasoning lifecycle as well as the existing observation/action clocks. Do not tune the controller or add obstacles as part of that study; first distinguish model/reasoning-history variability from timing variation. This recommendation is not implemented here.

## Figures and validation

Eight PNGs in the final output's `figures/`:

- `world_xy.png`
- `corner_comparison.png`
- `delay_overshoot.png`
- `delay_passage.png`
- `delay_seam_transport.png`
- `angular_command_yaw_rate.png`
- `selected_old_fresh_world.png`
- `delay_contact_goal.png`

Untested positive delays are explicitly shaded/labeled; no positive-delay curve or interpolated threshold is fabricated. Measured paths, projected predictions, original structures/gates, trigger/detection/application poses and actual contact locations are distinct. Detection/application coincide for zero holds.

Simulator-only startup PASS. Final focused suite: **36 tests PASS**. Before the first model trial, **34 tests passed**; after fixing only result serialization/filename handling, **36 passed** before trial 2. Synthetic hold lifecycle tests cover integer ticks, zero compatibility, one hold, no extra capture/request, OLD command/wheels, switch order, unchanged native/anchor, missing trigger, contact stop, corruption failure, exclusivity, stopping gates and saved A/B deterministic trigger/config/immutability. Positive-duration physics holds are tested synthetically, **not validated in new real navigation runs** because the baseline gate failed.

Both new runs pass the **unmodified original Nova validator** as well as the additive hold validator, with 48 matched finite native chunks, exact observation anchors, all application ticks equal to detection ticks, recomputed controller commands and retained OLD targets. Both pre/post stage audits confirm the original Hospital collision inventory is unchanged and no actor/cart exists. Historical source files, frozen inputs and weights pass hash checks; final raw-run hashes are in analysis metadata. See setup `final-validation.json` and `sweep-stop.json`. Process exit code alone was not used to assign navigation success.

Starting local and origin SHA: `e7fdc23d99af09b62c2376e2529c1789ad97f642`; upstream/main: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Each run binds exact dirty source hashes. RTX 5060 Ti, driver 595.99.02, kernel 7.0.0-38-generic; no package, driver, binary-stack or global environment changes. Full runtime versions/checkpoint hashes remain in each run. Final commit/push verification is stored in setup `git-finalization.json` and reported to the user, avoiding a self-referential commit SHA.

Reproduction commands and exclusive run IDs are saved in setup `trial-00-process.json` / `trial-01-process.json`. Saved-only final analysis:

```bash
MPLCONFIGDIR=/tmp/ticvla-mpl python3 research/analyze_late_handoff.py --plan outputs/e16-late-handoff-plan-20261011-02 --output-dir outputs/e16-late-handoff-analysis-<NEW-ID>
```

Limitations: one scenario; two new zero controls; two historical successful references; no positive-delay trials; fixed 48-prediction length; asynchronous reasoning/history and timing variation; conservative clearance proxies; nominal waypoint timing and derived tangents; no stopping-at-goal validation; no feasible recovery construction, correspondence validation or graph reconciliation.
