# C10/C24 wheelchairs with a cart beside C24

2026-10-14 (Asia/Seoul). **Runtime PASS; FAIL_THIS_SCENARIO.**
One requested model trial, 90 predictions; no navigation retry or result-based tuning.
The requested validation is C10 passage, post-corner joint observation of the
C24 obstacles, left-side passage, and contact-free goal-region entry.
The robot passed C10, saw both C24 objects, then contacted the C24 wheelchair
and stalled before passing either C24 object. It did not reach the goal.

## Scenario and placement

Same stationary wheelchairs, original unobstructed-path reference, instruction,
controller max_v **0.8 m/s**, max_w **1.2 rad/s**, and 90-call budget as
[the successful two-wheelchair run](TWO_WHEELCHAIRS_V08.md).
One static Hospital supply cart is added on C24's world +Y side. No people.

- C10 wheelchair centre: [6.824887275695801, 7.096357822418213, 0] m.
- C24 wheelchair centre: [-1.7567275762557983, 10.988419532775879, 0] m.
- Additional cart bounds centre: [-1.7567275762557981, 11.834615672735879, 0.0] m.
- Cart centre world-Y offset from C24: **0.846196140 m**.
- Enclosing world-Y bounds gap: **0.050000 m**.

The initially suggested centre offset +0.4 m would overlap the wheelchair's
enclosing footprint. The user selected minimal nonoverlapping spacing.
The placement rule therefore fixes X and orientation, and uses a 5 cm gap
between enclosing world-Y bounds. This is a conservative small gap, not a
claim of the mathematical minimum mesh-to-mesh distance. No size reduction,
asset change, obstacle motion or hidden steering was used.

Cart source `/Root/SM_SupplyCart_01e_21`, Hospital `Props/SM_SupplyCart_01e.usd`,
one reference copy `/World/E16Cart`. Original mesh, material, triangle collision,
scale and source remain intact. Grounding uses bottom Z=0. Cart long axis
follows the original C23-to-C25 measured route tangent, same orientation rule
as C24 wheelchair. All three static world transforms are verified every tick.
They are static USD geometry measurements, not dynamic PhysX body poses.

Original instruction, verified in all 90 actual requests:

> Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.

World coordinates use the original Hospital origin, metres, +Z up; yaw CCW
from +X. Robot quaternions are wxyz. USD row-vector matrices map local geometry
to world. At C24 the original tangent is -2.986313828 rad;
forward=(cos yaw,sin yaw), left=(-sin yaw,cos yaw). Robot left is approximately
world -Y and right is +Y. Passage side is measured at the first forward crossing
of the C24 tangent-normal plane, with linear interpolation between physics rows.
Full passage additionally requires going beyond obstacle half-length + robot radius.
This describes measured motion, not an inference about route intent.

## Gates and measured result

Model-free preflight and actual pre-model gate: **39 checks PASS**.
Original camera/wheels/spawn/lighting, exact wheelchair centres, static transforms,
cart asset identity, floor support and no Hospital overlap were checked.
A sampled conservative left local bypass passes all three obstacles and Hospital
geometry. The tested right bypass intersects the new cart. One failed right
probe does not prove that every possible right route is blocked.

A saved-only geometric counterfactual places the new cart on the earlier 0.8 m/s
measured path: first enclosing-footprint conflict at 20.365527 sim s and minimum
proxy clearance -0.845304 m. This is static geometric analysis, not a rerun,
measured new collision, or penetration depth.

| Object | Fully passed sim s | First centre-plane side | Contact reports | First contact sim s |
| --- | ---: | --- | ---: | ---: |
| C10 | 10.316667 | left | 0 | Not observed |
| C24 | Not passed | No crossing | 3816 | 21.550001 |
| C24_cart | Not passed | No crossing | 0 | Not observed |

Scenario checks:

- goal_region_entered: **False**
- no_nonfloor_reports_through_goal_or_end: **False**
- all_obstacles_passed: **False**
- all_obstacles_fixed: **True**
- C24_passed_on_left: **False**
- both_C24_objects_clear_after_corner_before_crossing: **False**

The final combined check is false because no crossing occurred. Both objects
were actually CLEAR after the corner at **observation 29, 15.283334 sim s**:
21,817 wheelchair pixels and 15,536 cart pixels. This must not be read as a
failure to render or observe the two obstacles. Observation IDs in this new
run are distinct from the original baseline C10/C24 placement indices.

- First strict goal distance <2 m entry: **Not observed**.
- Closest goal distance: **10.886267 m**.
- Final goal distance: **10.886269 m**.
- Nonfloor collider-pair spans through goal/end: **22**.
- Total nonfloor spans: **22**, 3816 reports.
- First nonfloor report: **21.550001123920083 sim s**.
- Maximum impulse norm: **365.924534 N s**;
  minimum contact separation: **-0.0008953511714935303 m**.
- 2778 paired robot/cart rows and 5556 wheelchair rows;
  final simulation time 46.283336 s.
- All obstacle motion and transform errors are zero.

Persistent contact callbacks are included; reports/spans are not independent
impacts. Proxy distances are not exact physical surface clearance. Goal entry
is separate from stopping; the unchanged 90-call loop continues after entry.
Post-corner co-observation is explicitly defined as after C10 full passage,
heading within 45 degrees of the C24 reference tangent, with both objects meeting
the existing semantic visibility rule in the same actual RGB frame.

## Approach and contact timing

The first contact callback at 21.550001 s has zero impulse and positive separation.
The first nonzero contact impulse occurs at **21.566668 s**, followed by negative
separation at 21.600001 s. The robot chassis contacts the original C24 wheelchair
mesh at robot XY [-1.3448210954666138, 11.035462379455566] m. Every nonfloor
contact target in this trial is that wheelchair; no cart or Hospital contact
was recorded.

| Observation | Observation sim s | Application sim s | v m/s | w rad/s |
| --- | ---: | ---: | ---: | ---: |
| 39 | 20.283334 | 20.383334 | .800000 | .078719 |
| 40 | 20.783334 | 20.883334 | .800000 | .063571 |
| 41 | 21.283334 | 21.383334 | .800000 | .006985 |
| 42 | 21.783334 | 21.866668 | .113842 | -.893045 |
| 43 | 22.283334 | 22.383335 | .027335 | 1.114327 |

All observation 29–41 commands retain v=.8. The first substantial speed reduction
after joint observation is applied **.300000 s after the first nonzero impulse**.
The measured approach bends toward world -Y but does not create enough lateral
clearance to pass the wheelchair. It does not complete a left or a right passage.
Final target v is still .8 while measured body-forward velocity is .001055 m/s;
the robot is stalled against the obstacle, not stopped at the requested goal.

This establishes insufficient clearance and late command reduction in this
trial. It does not isolate model perception, native action generation, controller
mapping or reasoning timing as the sole cause. Seeing both objects in RGB does
not prove the model's internal semantic understanding. The repeated material
warnings' effect on contact response is unquantified.

## Validation and provenance

21 focused CPU tests PASS, including asymmetric asset transforms, grounding/gap,
westbound left/right sign, non-passage rejection and exclusive diagnostic logging.
Independent saved-evidence audit PASS: 1184 raw, 170 source,
388 reference hashes; 2807 protected previous files unchanged.
All 90 finite 30x2 native arrays and controller commands independently checked;
90 1920x1080 RGBs, 270 reconstructed object masks,
90-frame GIF (46.28 s), paired static measurements and speed cap verified.
Every non-actor config and both wheelchair assets exactly equal the previous v08 run.
Console: 19745 PhysX NULL-material warnings, 0 Python tracebacks.

Visual review caught a reversed direction in the automatic plot's decorative
left arrow; numerical side calculations were correct. The reviewed figure in
the audit fixes the arrow and adds contact/left-probe context. The final original
repository analyzer includes that annotation-only fix. TEST's executed bundle
remains frozen for receipt verification; all simulation/analysis source hashes
and original plots are preserved. This cosmetic difference is recorded in
`final-source-review.json`; no model rerun or outcome change.

The initial model-free preflight passed geometry but failed because the diagnostic
helper was invoked twice and tried to exclusively create `grounding_views.json`
twice. Source/failed raw records were retained. One combined three-target diagnostic
call fixed that logging error; p2 and actual run have the identical original placement.
No inference ran before the full p2 gate passed. No threshold or geometry was relaxed.

Additive `tools/c24_adjacent_cart/` and tests only; no upstream/model/controller/
package/driver/asset-installation change. Max_v .8, max_w1.2, lookahead1m,
angular gain.8, bearing filter.35 and epsilon.001 retain their prior values.
Native outputs are forward/left offsets with no yaw. 60Hz physics, target2Hz,
one IPC future, OLD retention and observation-anchored FRESH are unchanged.
Observation, inference start/ready, reasoning generation/ready/poll and command
application retain distinct clocks; simulation time is not wall time.

Latest supplied AGENTS requires `hardcase-probe`: clean branch was fast-forwarded
to existing local SHA `a6383275902ab368ebd5be1b10ab2b9cccb9ef46` before additions.
The prior `TIC-VLA-TEST` branch remains at that commit; no history rewritten or push.
Upstream SHA `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Dirty state, diff identity,
new-source hashes, exact commands and final local commit are in the audit directory.

Simulation Isaac6.0.1-rc.7 / Python3.12.13 / NumPy2.3.1 / RealTimePathTracing;
separate model-only Isaac5.0.0-rc.45 / Python3.11.13 / torch2.7.0+cu128 /
torchvision.22.0+cu128 / NumPy1.26.0 / transformers4.57.6; no second SimulationApp.
RTX5060Ti, driver595.99.02. Base weight SHA256
`a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`;
checkpoint SHA256
`376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`.
This is one modified Isaac6 harness trial, not unchanged Isaac5 reproduction,
a causal matched comparison, general avoidance qualification or real-world evidence.

## Commands and artifacts

These are already-used IDs. A rerun needs fresh run and session IDs.

```bash
cd /home/gpuadmin/Workspace/TIC-VLA-TEST
python3 -B tools/c24_adjacent_cart/runner.py prepare --run-id c24-adjacent-cart-20261014T040913Z-07666d-preflight-p2 --session /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-20261014T040913Z-07666d-preflight-p2-setup --preflight
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-20261014T040913Z-07666d-preflight-p2-setup/run.sh
python3 -B tools/c24_adjacent_cart/runner.py prepare --run-id c24-adjacent-cart-20261014T040913Z-07666d --session /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-20261014T040913Z-07666d-setup --prerequisite outputs/c24-adjacent-cart-20261014T040913Z-07666d-preflight-p2
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-20261014T040913Z-07666d-setup/run.sh
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-20261014T040913Z-07666d-setup/analyze.sh
```

Launch bound 900 wall seconds +30s termination grace. Raw and derived data separate,
all generated artifacts ignored/uncommitted.

- [Reviewed trajectory and C24 detail](/home/gpuadmin/Workspace/TIC-VLA/outputs/c24-adjacent-cart-preparation-20261014T040913Z-07666d/trajectory-reviewed.png)
- [Actual observation GIF](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-20261014T040913Z-07666d/derived/c24-adjacent-cart/rgb-observations.gif)
- [Actual approach/contact RGB](/home/gpuadmin/Workspace/TIC-VLA/outputs/c24-adjacent-cart-preparation-20261014T040913Z-07666d/encounter-rgb.png)
- [Summary](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-20261014T040913Z-07666d/derived/c24-adjacent-cart/summary.json)
- [Placement view](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-20261014T040913Z-07666d-preflight-p2/diagnostics/C24_body.png)
- [Console](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-20261014T040913Z-07666d-setup/console.log)
- [Audit](/home/gpuadmin/Workspace/TIC-VLA/outputs/c24-adjacent-cart-preparation-20261014T040913Z-07666d)
