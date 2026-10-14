# C24 adjacent cart with controller max_v 0.5 m/s

2026-10-14 (Asia/Seoul). **Runtime PASS; FAIL_THIS_SCENARIO.**
One requested trial, 144 finite native predictions, no navigation retry
or post-result tuning. This follows [the 0.8 m/s adjacent-cart trial](C24_ADJACENT_CART.md).
The robot passed C10 without contact, missed the left turn into the C24 corridor,
continued into the space under/beyond the staircase, turned back, and contacted
the staircase on its return. It never reached the C24 obstacle pair or the goal.

## Preserved scenario and requested speed change

Only two configuration values differ from the preceding actual run:

- Controller max_v **0.8 -> 0.5 m/s**.
- Prediction budget **90 -> 144** at target 2 Hz, preserving the nominal maximum
  distance budget (90*.8 = 144*.5); nominal duration 45 ->72 simulation seconds.

Max_w remains **1.2 rad/s**, lookahead1m, angular_gain.8, bearing_filter.35,
epsilon.001. The unchanged controller formula includes velocity in its curvature
term, so lowering the cap can also reduce that component of angular command;
angular control is not separately retuned. Model weights, instruction, camera,
start/goal, physics, lighting, visibility rules and all obstacle settings are identical.

Two stationary original Hospital wheelchairs at exact original unobstructed
baseline C10 and C24 centres, with one stationary supply cart beside C24:

- C10 [6.824887275695801, 7.096357822418213, 0] m.
- C24 [-1.7567275762557983, 10.988419532775879, 0] m.
- Cart [-1.7567275762557981, 11.834615672735879, 0.0] m, **C24 +0.846196140 world Y**.
- Conservative world-Y bounds gap **0.05 m**. All actual assets/poses and
  source hashes equal the preceding 0.8 run; no people or moving obstacles.

Original instruction, independently verified in every actual request:

> Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.

World original Hospital origin, XYZ metres, +Z up; yaw CCW from +X;
robot quaternion wxyz; USD row-vector transforms local->world. At C24, left
relative to original C23->C25 tangent is approximately world -Y. First passage
side uses that tangent-normal plane and interpolated measured robot poses;
full passage requires going beyond obstacle half-length plus enclosing robot radius.
This describes motion, not inferred route intent. Static obstacles are verified
through USD matrices every tick, not reported as dynamic PhysX measurements.

## Measured outcome

| Object | Full passage sim s | First centre-plane side | Contact reports | First report sim s |
| --- | ---: | --- | ---: | ---: |
| C10 | 15.566667 | left | 0 | Not observed |
| C24 | Not observed | No crossing | 0 | Not observed |
| C24_cart | Not observed | No crossing | 0 | Not observed |

First jointly CLEAR post-corner observation: **Not observed**.
Both objects must meet the existing semantic visibility rule in one frame,
after C10 passage and with heading within45deg of the original C24 tangent.
Observation numbers here are new-run samples, not the old C10/C24 placement indices.
A combined 'visible before crossing' check can be false when crossing never occurs;
the separately saved joint visibility list is authoritative for actual co-observation.

| Measurement | Previous 0.8 m/s | Current 0.5 m/s |
| --- | ---: | ---: |
| Goal first entry sim s | Not observed | Not observed |
| Closest goal distance m | 10.886267 | 17.860225 |
| Final goal distance m | 10.886269 | 19.816141 |
| Nonfloor spans | 22 | 9 |
| First nonfloor report sim s | 21.550001 | 58.283336 |

4431 nonfloor contact reports, maximum impulse norm
18.657437 N s, minimum separation -0.0002500787377357483 m.
Contact targets: /Root/Geo_M_Stairs_873/Geo_M_Stairs/Geo_M_Stairs/Section0.
Reports include persistent callbacks; spans are collider-pair intervals, not independent
impacts. Initial zero-impulse positive-separation callbacks alone do not prove impact.
Centre distances and enclosing footprints are not exact mesh-to-mesh clearance.

4409 paired robot/cart rows, 8818 wheelchair rows,
73.466670 recorded simulation seconds. All obstacles remain fixed.
Maximum applied forward command 0.500000 m/s; maximum measured
body-forward speed 0.502186 m/s (measurement can slightly
exceed a command cap). Goal entry is separate from stopping; the fixed144-call loop
continues after goal entry. No stop-controller logic is added.

A single asynchronous trial at each speed is descriptive evidence, not a controlled
causal proof that speed alone caused any outcome difference or a general success rate.
No reconciliation/correspondence/optimization method is introduced.

## Missed turn and later staircase contact

C10 full passage occurs at **15.566667 sim s**, with no C10 contact. The robot
then continues approximately world +Y instead of completing the left turn toward
the C24 corridor (approximately -X). Saved RGB shows the staircase ahead and the
open space beneath it; later frames show the robot turning back toward the lobby.

| Observation | Observation sim s | World XY m | Yaw rad | Command v m/s | Command w rad/s |
| --- | ---: | --- | ---: | ---: | ---: |
| 35 | 18.466668 | [5.70284, 9.30090] | 1.38147 | .5 | .147158 |
| 39 | 20.466668 | [5.88921, 10.28363] | 1.28273 | .5 | -.046955 |
| 43 | 22.466668 | [6.12734, 11.25313] | 1.51508 | .5 | .262063 |
| 47 | 24.466668 | [6.03633, 12.24904] | 1.67203 | .5 | -.073446 |
| 55 | 28.466668 | [6.17496, 14.23610] | 1.29556 | .5 | -.145919 |
| 85 | 43.466669 | [6.53004, 18.56292] | -.75898 | .5 | -.209820 |

The first nonfloor callback is **58.283336 s**. The first nonzero impulse is
**58.416670 s**, between the left wheel and
`/Root/Geo_M_Stairs_873/Geo_M_Stairs/Geo_M_Stairs/Section0`, at robot XY
[7.394272327423096, 11.313713073730469] m. Negative separation first appears at
58.433336 s. This is the only nonfloor target across all nine spans. The active
observation114 command is v=.5, w=.081137, applied58.066670s; subsequent commands
also keep v=.5. Final target v=.5 but measured body-forward velocity .003505 m/s
indicates the robot stalls against the staircase, not at the goal.

C24 wheelchair never has visible semantic pixels in the saved observations.
The cart first has some pixels at observation27 but never meets CLEAR visibility.
Neither object is contacted or passed; minimum robot-centre distance to C24 is
7.626047 m. Thus **left-versus-right avoidance of the pair was not exercised**.
This is an earlier route-following failure, rather than a successful avoidance
inferred from zero obstacle contact. Lowering the speed did not resolve navigation
in this trial; these data do not identify one causal model/controller component.

## Validation and provenance

Model-free preflight and actual pre-model gate each PASS **39 checks**,
including completed-render RGB without physics advance, unchanged camera/wheels,
spawn/lighting, grounded/no-overlap objects and a feasible sampled conservative
left bypass. Diagnostic probes never command the robot. Placement images reviewed.

21 existing focused CPU tests PASS; new-module global symbol checks PASS;
exact source diff reviewed. Saved-evidence audit PASS: 1886 raw,
170 source and 388 reference hashes;
4203 protected prior files unchanged. Every actual configuration
field matches the previous run except max_v and prediction budget. All finite144x30x2
native arrays, independently recomputed commands, exact instructions, original RGB,
432 masks, matched static ticks and144-frame GIF (73.46s)
verified. Console: 0 Python tracebacks and 11829
PhysX NULL-material warnings; their impact on contact response is unquantified.

Additive `tools/c24_adjacent_cart_v05/`; no new upstream/controller/model changes,
no packages/host/driver/installation changes. The previous bundles, receipts and
raw/derived outputs remain immutable. Native30x2 forward/left offsets contain no yaw.
60Hz physics, target2Hz, one IPC future, OLD retention and observation-anchored FRESH
remain unchanged. Observation, inference start/ready, reasoning generation/ready/poll
and command application clocks remain distinct; sim time differs from wall time.

Branch `hardcase-probe`, base local SHA `c50da4a52abeedac4cc6e520bd834b3bd5d55823`;
upstream SHA `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Dirty status, tracked-diff identity, all added
source hashes and commands are saved in the audit; focused final commit recorded
in `git-finalization.json`. No push; no generated experiment output committed.

Simulation 6.0.1-rc.7+release.42383.32955d8d.gl, Python3.12.13, NumPy2.3.1,
RealTimePathTracing. Separate model-only 5.0.0-rc.45+release.23960.184afb15.gl, Python3.11.13,
torch2.7.0+cu128, torchvision0.22.0+cu128, NumPy1.26.0,
transformers4.57.6; no second SimulationApp.
Base SHA256 `a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`;
checkpoint SHA256 `376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`.
This is a modified Isaac6 wrapper run, not unchanged Isaac5 reproduction or real-world evidence.

## Executed commands and artifacts

These IDs are consumed; any rerun needs fresh run and session IDs.

```bash
cd /home/gpuadmin/Workspace/TIC-VLA-TEST
python3 -B tools/c24_adjacent_cart_v05/runner.py prepare --run-id c24-adjacent-cart-v05-20261014T042243Z-1da200-preflight --session /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-v05-20261014T042243Z-1da200-preflight-setup --preflight
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-v05-20261014T042243Z-1da200-preflight-setup/run.sh
python3 -B tools/c24_adjacent_cart_v05/runner.py prepare --run-id c24-adjacent-cart-v05-20261014T042243Z-1da200 --session /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-v05-20261014T042243Z-1da200-setup --prerequisite outputs/c24-adjacent-cart-v05-20261014T042243Z-1da200-preflight
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-v05-20261014T042243Z-1da200-setup/run.sh
bash /home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-v05-20261014T042243Z-1da200-setup/analyze.sh
```

Bounded launch900 wall seconds +30s termination grace. Raw and derived artifacts separate.

- [Trajectory](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-v05-20261014T042243Z-1da200/derived/c24-adjacent-cart-v05/trajectory.png)
- [0.8 versus 0.5 m/s measured paths](/home/gpuadmin/Workspace/TIC-VLA/outputs/c24-cart-v05-preparation-20261014T042243Z-1da200/speed-comparison.png)
- [Missed-turn RGB](/home/gpuadmin/Workspace/TIC-VLA/outputs/c24-cart-v05-preparation-20261014T042243Z-1da200/corner-rgb.png)
- [Staircase contact RGB](/home/gpuadmin/Workspace/TIC-VLA/outputs/c24-cart-v05-preparation-20261014T042243Z-1da200/encounter-rgb.png)
- [Actual RGB GIF](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-v05-20261014T042243Z-1da200/derived/c24-adjacent-cart-v05/rgb-observations.gif)
- [Actual RGB overview](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-v05-20261014T042243Z-1da200/derived/c24-adjacent-cart-v05/rgb-contact-sheet.png)
- [Summary](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-v05-20261014T042243Z-1da200/derived/c24-adjacent-cart-v05/summary.json)
- [Console](/home/gpuadmin/Workspace/TIC-VLA-TEST/outputs/c24-adjacent-cart-v05-20261014T042243Z-1da200-setup/console.log)
- [Audit](/home/gpuadmin/Workspace/TIC-VLA/outputs/c24-cart-v05-preparation-20261014T042243Z-1da200)
