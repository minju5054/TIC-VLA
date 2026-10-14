# E16 C10 장애물 회피 중 OLD/FRESH application delay 검증

최종 판정: **`BASELINE_UNSTABLE`**. 고정한 네 대조군을 모두 보존했고,
PASS 실행은 [2], FAIL 실행은 [1, 3, 4]다. **실제 0.25초 지연 주입은 0회**이며
Stage C는 사전 gate에 의해 실행하지 않았다. 이번 결과에서 양의 지연에 의한
`POTENTIAL_RECONCILIATION_CASE`를 입증하지 못했다. Graph 구현·실행은 없다.

## 실행 범위와 설정 보존

기준 실제 성공 데이터는 `outputs/two-wheelchairs-v08-20261014T015420Z-a56068/`다.
새 suite는 `outputs/e16-delay-suite-20261014T095206Z-14e2d1/`이며 기준 raw/기존 derived 1,204개 파일은 보존했다.
실제 navigation은 **4회 / native prediction 360개**다.
별도의 simulator-only 시작 점검 1회는 로봇/모델을 실행하지 않았고 navigation 횟수에 포함하지 않는다.
각 navigation은 새 Isaac 6 simulator와 별도 새 Isaac 5 Python 모델 worker로 시작했다.
이전 inference/history state를 재사용하거나 실패 실행을 교체·재시도하지 않았다.

전체 navigation config 비교 결과는 `[]`이다. E16 원래 instruction, Nova Carter,
C10/C24 정적 휠체어, max_v=0.8 m/s, max_w=1.2 rad/s, lookahead=1 m,
angular gain=.8, filter gain=.35, 60 Hz physics, 목표 2 Hz inference,
90 predictions, seed=36, 카메라·체크포인트·renderer·wheel adapter를 유지했다.
Renderer는 기준과 같은 `RealTimePathTracing`, camera는 기존 front Hawk 1920×1080이다.
설치·driver·CUDA·Python·패키지 변경은 없다. 이는 Isaac 6 수정 harness 실험이며
변경 없는 Isaac 5 공식 reproduction이라고 부르지 않는다.

기존 `research/continuous.py`, controller, simulator, 모델 소스는 수정하지 않았다.
`NO_HOOK`은 기존 loop에 `application_hook=None`을 전달한다. 새 saved validator는
90 prediction과 application까지 pending OLD 유지, 선택된 hold 상태를 검사하며
과거 48 prediction 전용 `load_late`를 사용하지 않는다. 결과 분석 callback은
90개 명령 실행이 끝난 뒤 호출되므로 no-hook 주행 경로를 바꾸지 않는다.

## 사전 protocol과 공간 trigger

Protocol SHA-256: `ecfa3a8db28ee3a153ce508556ecb74666cbac7f7deda1e3b84e9c523eb3b0d0`.
새 모델 결과 또는 Stage A metric을 보기 전에 고정했다.
기준 C13 observation center는 `(6.358338833, 5.642079830)` world m이다.
직전 6 physics interval(tick 440→446)의 실제 XY 이동으로 구한 진행 yaw는
`2.045134372741` rad이다.
기준 body yaw는 `2.049091875239` rad다.
진행 방향 ±0.5 m, 횡방향 ±0.6 m, 기준 body yaw ±30°를 사용한다.
Body forward 및 기준 진행 방향 속도 >0.05 m/s, non-bootstrap,
이전 비바닥 callback 없음, C10 완전 통과 이전을 모두 요구했다.

현재 observation state와 과거 실제 passage/contact만 trigger에 사용했다.
FRESH prediction·미래 contact·성공 여부·새 request ID는 trigger 입력이 아니다.
첫 eligible observation 한 번만 선택하며 없으면 `TRIGGER_NOT_REACHED`다.
서로 같은 request 번호도 같은 물리 위치라는 의미가 아니다.

World 원점은 원래 Hospital, +Z up, XY는 m, yaw는 +X 기준 CCW rad, quaternion은 wxyz다.
Native `(30,2)`는 observation-body `[forward,left]` m이고 yaw/time이 없다.
`p_world = p_obs + R(yaw_obs) @ p_native`로 투영해 anchor를 고정했다.
Nominal `t_obs + 0.1…3.0 s`는 확인된 10 Hz source contract의 분석 convention이다.
공통 유효 support만 사용하며 외삽/원점 knot 추가를 하지 않는다.

## Stage A — 저장 데이터의 kinematic sensitivity

기준 C12 OLD `(v,w)`를 C13의 실제 response-detection pose에서 추가 유지하는
constant-command unicycle 근사다. 이후 실제 주행 pose를 OLD 연장으로 사용하지 않았다.
동일 C13 native FRESH와 observation anchor를 모든 지연 후보에 사용했다.
실제 PhysX·wheel dynamics·충돌 재현 결과가 아니다.

| 추가 지연 s | ticks | OLD 경로 cm | time-aligned e_p cm | delta e_p cm | 방향 오차 deg | longitudinal cm | lateral cm | fixed-reference e_p cm | 남은 horizon s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0.00 | 0 | 0.000 | 1.005 | 0.000 | 6.332 | 0.595 | -0.810 | 1.005 | 2.900 |
| 0.25 | 15 | 20.000 | 3.836 | 2.831 | 14.663 | 3.344 | -1.880 | 20.682 | 2.650 |
| 0.50 | 30 | 40.000 | 7.579 | 6.574 | 8.345 | 7.295 | -2.056 | 40.664 | 2.400 |
| 0.75 | 45 | 60.000 | 11.939 | 10.934 | 1.284 | 11.727 | -2.236 | 60.646 | 2.150 |

0.25초 근사에서 OLD 경로는 20 cm, time-aligned 위치 차이는 약 3.836 cm다.
하나의 고정 reference time에 대한 공간 차이는 약 20.682 cm로,
nominal FRESH 진행을 함께 반영한 3.836 cm와 구별된다.
0.50/0.75초는 이 model-free 계산에만 사용했다. 실제 추가 지연 수준을 확대하지 않았다.

## Stage B — 고정 순서의 실제 대조군 결과

| Run | 조건 | 추가 s / tick | 실제 prediction | Trigger ID | C10 passage s | C24 passage s | 목표 진입 s | 목표 전 비바닥 callback | Baseline |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | NO_HOOK | 0 / 0 | 90 | 12 | 10.300 | 미정/미관측 | 미정/미관측 | 2568 | FAIL |
| 2 | ZERO_HOOK | 0 / 0 | 90 | 12 | 10.283 | 23.283 | 32.500 | 0 | PASS |
| 3 | ZERO_HOOK | 0 / 0 | 90 | 12 | 10.233 | 미정/미관측 | 미정/미관측 | 1595 | FAIL |
| 4 | NO_HOOK | 0 / 0 | 90 | 12 | 10.350 | 미정/미관측 | 미정/미관측 | 5725 | FAIL |

목표는 strict XY <2 m 최초 진입이며 목표가 없으면 전체 실행을 접촉 평가 구간으로 삼았다.
Passage는 저장된 휠체어 방향의 중심면을 robot conservative radius와 최대 half-length만큼
완전히 지난 실제 tick이다. Goal entry는 목표에서 정지했다는 뜻이 아니다.
실행 1은 C10을 통과했지만 C24/목표에 도달하지 못했고 24.316668초부터 구조물 접촉을 기록했다.
네 실행 모두의 통과가 필요한 사전 조건은 이 실행만으로도 실패한다.
남은 0초 대조군은 사전 순서대로 완료했고 결과에 따라 대체하지 않았다.

### 실제 trigger 위치와 선택된 FRESH

| Run | 실제 request | observation s | B0 s | obs world X m | obs world Y m | obs yaw deg | FRESH 예측 side | 0초 local 진단 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 12 | 6.783334 | 6.883334 | 6.359358 | 5.221682 | 118.937 | LEFT | ROBUST_HANDOFF |
| 2 | 12 | 6.766667 | 6.866667 | 6.363841 | 5.227555 | 120.540 | LEFT | ROBUST_HANDOFF |
| 3 | 12 | 6.750000 | 6.850000 | 6.523037 | 5.281735 | 109.023 | LEFT | UNSAFE_OR_UNQUALIFIED_FRESH |
| 4 | 12 | 6.783334 | 6.883334 | 6.493896 | 5.261395 | 120.441 | LEFT | ROBUST_HANDOFF |

마지막 열은 선택된 **0초 handoff만의 국소 기하학 진단**이다.
`ROBUST_HANDOFF`가 나오더라도 전체 navigation 성공이나 실제 양의 지연 내성을 의미하지 않는다.
전체 suite 판정은 `BASELINE_UNSTABLE`이 우선한다.

### 같은 FRESH의 B0/B_delta

| Run | e_p B0 cm | e_p B_delta cm | delta e_p cm | e_theta B0 deg | e_theta B_delta deg | delta e_theta deg | 추가 OLD 경로 cm |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 0.984 | 0.984 | 0.000 | 4.466 | 4.466 | 0.000 | 0.000 |
| 2 | 0.904 | 0.904 | 0.000 | 2.417 | 2.417 | 0.000 | 0.000 |
| 3 | 1.545 | 1.545 | 0.000 | 5.342 | 5.342 | 0.000 | 0.000 |
| 4 | 1.210 | 1.210 | 0.000 | 5.760 | 5.760 | 0.000 | 0.000 |

모든 실제 추가 hold는 0 tick이므로 B0와 B_delta의 실제 pose는 동일하다.
명목 첫 knot 이전이거나 tangent가 짧으면 미정으로 처리하고, missing error를 0으로 채우지 않는 규칙이다.
이번 선택된 네 boundary 쌍은 모두 유효했다. 실제 15 tick 양의 hold 측정치는 없다.
각 run의 `analysis/result.json`에 longitudinal/lateral 오차와 fixed-reference 오차,
body yaw 오차, 남은 horizon, before/after proxy와 원본 anchor를 저장했다.

실제 실행 방향은 각 boundary 직전 6개 physics interval의 net XY displacement에서 구한다.
2 cm보다 짧으면 미정이다. FRESH tangent는 2 cm 이상의 초기/인접 segment와
45° 이하의 인접 heading 변화가 필요하며 observation 방향을 기준으로 역행을 검사한다.
B0는 response 검증 직후 hold-start pose, B_delta는 hold-end pose다.
NO_HOOK의 B0/B_delta는 각각 기존 response detection/application state다.
정확한 service-ready robot pose는 관측되지 않았고 생성하지 않았다.

### Timing 및 실제 OLD 이동

| Run | 선택 detect→apply ms | 그 외 detect→apply 중앙 ms | action latency 중앙 ms | obs→switch OLD cm | inference 내부 tick 경로 cm | 추가 ticks |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0.318 | 0.319 | 51.497 | 8.009 | 4.004 | 0 |
| 2 | 10.465 | 26.226 | 53.844 | 8.006 | 4.003 | 0 |
| 3 | 9.397 | 23.106 | 53.544 | 8.010 | 4.005 | 0 |
| 4 | 0.298 | 0.323 | 52.202 | 8.007 | 2.669 | 0 |

Action latency는 service inference start→ready의 monotonic wall 시간이다.
관측·ready·response detection·hold 시작·switch 시각을 분리했다.
Inference 내부 이동은 해당 wall interval에 완전히 포함된 physics tick만 합산했다.
`D_old_total`은 observation→switch 실제 누적 XY, `D_hold`는 B0→B_delta 누적 XY다.
Net displacement 및 yaw 변화도 별도로 저장했다.

Zero-hook은 추가 physics 진행이 없지만 파일 기록·검증의 wall overhead가 있다.
표의 detect→apply는 controller 계산·snapshot·hook·진단 저장을 포함한 실제 구간으로,
순수 hook 비용만을 분리한 실험은 아니다. `hook_total_wall_ns`는 hook 내부의 마지막
검증/hold JSON flush 직전까지의 계측치이므로 전체 overhead의 대용으로 단정하지 않는다.
현재 hook은 one-shot 선택이 끝난 뒤에도 매 응답에서 전체 contact history를 읽고 trigger를 검사한다.
선택되지 않은 호출의 내부 계측 중앙값은 실행 2에서 25.865 ms, 실행 3에서 22.743 ms다.
NO_HOOK의 전체 detect→apply 중앙값 약 0.32 ms와 비교하면 이 진단 비용은 작지 않다.
이 구현상의 한계를 그대로 기록했으며 결과를 보고 hook을 수정하거나 재실행하지 않았다.

Background reasoning은 wall time에서 진행하므로 0 tick만으로 모든 후속 모델 상태가
동일함을 보장할 수 없다. 서로 독립적인 두 번씩의 실행으로 hook의 navigation
인과 효과 또는 성공률을 추정하지 않는다. NO_HOOK 자체에서도 실패가 관측됐다.

### 선택된 handoff의 controller command

단위는 `(v m/s, w rad/s)`이며 실제 body velocity와 구분한다.

| Run | OLD (v,w) | FRESH (v,w) | FRESH − OLD |
| --- | --- | --- | --- |
| 1 | 0.800, 0.096586 | 0.800, 0.004708 | 0.000, -0.091878 |
| 2 | 0.800, 0.162883 | 0.800, 0.010446 | 0.000, -0.152436 |
| 3 | 0.800, -0.005977 | 0.800, 0.051021 | 0.000, 0.056998 |
| 4 | 0.800, 0.125604 | 0.800, 0.092229 | 0.000, -0.033374 |

### 실제 접촉과 motion

| Run | 첫 callback s | 전체 callback | 양의 impulse callback | 음수 separation callback | zero impulse/비침투 callback | 최대 impulse Ns | 최소 separation cm |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 24.317 | 2568 | 2319 | 1306 | 248 | 32.654 | -0.021 |
| 2 | 없음 | 0 | 0 | 0 | 0 | 0.000 | 미정/미관측 |
| 3 | 21.550 | 1595 | 1434 | 941 | 159 | 44.475 | -0.222 |
| 4 | 19.683 | 5725 | 1278 | 358 | 4441 | 25.181 | -0.747 |

위 횟수는 persistent PhysX callback 수이며 독립 충돌 횟수가 아니다.
양의 충격량과 음수 separation을 구분하고, zero-impulse proximity callback을 실제 충격이라고 부르지 않는다.
Hold 중이면 기존 비바닥 contact 기준(`abs(normal.z)<0.5`, robot 자기접촉 제외)에
해당하는 callback 하나에도 즉시 physics 진행을 중지하고 FRESH를 적용하지 않는 구현이다.
이번에는 실제 양의 hold가 없었으므로 `CONTACT_DURING_HOLD` 측정 사례는 없다.

실행 1의 첫 접촉은 오른쪽 wheel–Hospital 구조물이며 첫 callback부터 양의 impulse가 있다.
실행 3는 chassis–C24 휠체어 사이에서 21.550001초에 zero-impulse proximity가 먼저 기록됐고, 21.566668초에 첫 양의 impulse가 기록됐다.
실행 4는 chassis–C24 휠체어 사이에서 19.683334초에 zero-impulse proximity가 먼저 기록됐고, 19.700001초에 첫 양의 impulse가 기록됐다.
실행 1·3·4는 이후 음수 separation도 기록했다. 따라서 전체 실패를 proximity callback만의 결과로 설명할 수 없다.


| Run | 전체 실제 경로 m | 후진 적분 cm | 최소 goal 거리 m | 종료 goal 거리 m | 최대 body vx cm/s | 최대 abs yaw-rate deg/s |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 18.453 | 7.947 | 17.428 | 24.106 | 80.301 | 110.257 |
| 2 | 35.549 | 0.006 | 0.357 | 8.976 | 80.232 | 13.393 |
| 3 | 16.221 | 16.483 | 10.886 | 10.887 | 80.169 | 103.449 |
| 4 | 18.810 | 27.890 | 11.205 | 13.899 | 80.352 | 103.781 |

후진 적분은 `max(0,-measured body-forward velocity) × dt`의 합이며 route상의 전역 되돌아감과 동일하지 않다.
측정 velocity/yaw-rate와 controller command는 서로 다른 항목으로 저장했다.

## 기하학적 안전성과 해석 범위

저장된 휠체어 yaw/OBB와 해시 검증된 병원 구조물 828 AABB,
기존 robot disk radius 0.606982542 m를 사용했다. 정확한 전체 robot swept collision mesh는 없다.
0.01 m 이하 간격 sampling과 거리 하한 >0.02 m로 proxy clearance를 판정했다.
기준 성공 경로도 특히 C24에서 이 disk/AABB proxy와 겹쳤으므로 음수 proxy clearance를
실제 PhysX 충돌로 해석하지 않는다. OLD future/FRESH future/가상 RAW connector를 구별했다.
가상 connector는 실제 controller가 실행한 경로가 아니다.

| Run | FRESH B0 하한 cm | FRESH B_delta 하한 cm | connector B0 하한 cm | connector B_delta 하한 cm | B0 disk 하한 cm | B_delta disk 하한 cm |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 18.349 | 18.349 | 54.164 | 54.164 | 57.824 | 57.824 |
| 2 | 19.227 | 19.227 | 53.823 | 53.823 | 57.325 | 57.325 |
| 3 | -10.175 | -10.175 | 43.563 | 43.563 | 48.190 | 48.190 |
| 4 | 13.759 | 13.759 | 47.920 | 47.920 | 51.453 | 51.453 |

실행 3의 선택된 C10 FRESH는 보수적 proxy에서 미검증으로 분류됐다. 실제 C10 통과는 무접촉이었으며,
이 proxy 결과를 뒤에 발생한 C24 접촉의 원인이나 실제 trajectory 충돌로 확정하지 않는다.

FRESH가 안전한 후보이고 B0에서는 연결이 clear하지만 B_delta에서만 새로운 connector
간섭이 생기는 경우에만 potential case로 고려하도록 사전 규칙을 고정했다.
큰 position/direction error만으로 후보를 만들지 않는다.
Controller는 native timed queue를 그대로 추종하지 않고 lookahead의 단일 `(v,w)`를 실행한다.
따라서 offline native curvature나 nominal timing만으로 robot-level feasibility를 증명하지 않는다.
이번에는 실제 0.25초 trial이 없어 추가 OLD 실행에 의한 연결 위험과 controller 추종 가능성을
실제 환경에서 비교할 수 없다. 실패한 대조군 전체 주행을 reconciliation hard case로 선정하지 않는다.

## 연구 질문에 대한 답

1. **0초에서 성공이 재현됐는가?** PASS는 [2], FAIL은 [1, 3, 4]다. 네 번 모두 성공해야 하는 안정성 기준은 실패했다.
2. **Hook 자체의 영향은?** 추가 physics는 0 tick, FRESH와 anchor는 동일했다. Wall overhead는 관측됐지만 독립 실행의 navigation 차이를 hook의 인과 효과로 확정하지 못했다.
3. **0.25초에서 OLD 이동이 얼마나 증가했는가?** 실제 측정은 없다. Stage A 근사에서만 20 cm이며 PhysX 결과가 아니다.
4. **동일 FRESH B0/B_delta의 불일치 변화는?** 0초 actual controls는 동일 pose이며 유효 metric의 delta는 0이다. 양의 hold에 대한 실제 delta는 미검증이다.
5. **FRESH의 우회 방향 유지 여부는?** 0초 hold 전후에는 같은 FRESH/시각이므로 변하지 않았다. 각 구간의 predicted crossing을 표에 기록했다. 양의 지연 후 여부는 확인하지 못했다.
6. **실제 접합 위험이나 충돌이 있었는가?** 일부 대조군의 실제 구조물 및 C24 휠체어 contact가 있었지만 추가 hold를 주입하지 않았다. 지연 때문에 새 접합 위험이 생겼다는 증거가 아니다.
7. **SE(2) reconciliation으로 다룰 문제인가?** Editable FRESH에 연결 문제를 국한할 실제 양의 지연 비교를 확보하지 못했다. Committed OLD 영역의 충돌을 Graph로 되돌릴 수 있다고 주장하지 않는다.
8. **필요성의 증거는 충분한가?** 부족하다. 판정은 `BASELINE_UNSTABLE`; 다음 단계의 Graph 비교 근거는 확보되지 않았다.

## 사실 / 측정 / 해석 / 미검증 가설

- Repository 사실: 기준 config 전체 일치, runtime 소스 해시 보존, original loop의 opt-in hook 사용, 90-call 독립 validator와 동일 FRESH 보존 테스트, 모델/주행 코드 미수정.
- 실제 측정: 위 네 실행의 실제 prediction/state/contact, 0 tick hold와 wall overhead, passage/goal 결과. Stage A는 별도 kinematic approximation이다.
- 연구적 해석: 현재 조건에서 0초 baseline 안정성이 충분하지 않아 양의 지연의 인과 효과를 해석할 실험 전제가 실패했다.
- 미검증 가설: 비동기 background reasoning/렌더·inference timing의 변동이 navigation 차이에 기여할 수 있으나 원인 분리 실험은 하지 않았다. 더 큰 실제 지연이나 Graph 이점도 검증하지 않았다.

## 검증과 산출물

신규 테스트 **25 PASS**, 기존 handoff **30 PASS**, continuous **7 PASS**, 총 **62 PASS**.
테스트 명령은 `python3 -m unittest tests.test_e16_wheelchair_delay tests.test_e16_wheelchair_handoffs tests.test_continuous -v`다.
수학 fixture는 실제 결과에 포함하지 않았다. 테스트는 15 tick 변환, zero tick,
one-shot 공간 trigger, OLD target 유지, FRESH byte/anchor 보존, horizon 외삽 금지,
callback 중단과 FRESH 미적용, 90-call 허용/48-call 불완전 판정, state 정렬,
좌표/각도, 기존 파일 보호 및 post-goal 제외를 포함한다.

산출물은 `outputs/e16-delay-suite-20261014T095206Z-14e2d1/`에 있다.
`protocol.json`, `runtime-freeze.json`, `stage-a/result.json`, 각 `trial-N/analysis/result.json`,
`stage-b-gate.json`, `aggregate.json`, 테스트/시작 로그와 별도 실제 run 디렉터리를 보존했다.
`figures/figure1-world.png`, `figure2-handoffs.png`, `figure3-delay-sensitivity.png`,
`figure4-navigation.png`는 각각 전체 actual 경로, C10 100×100 cm 확대,
근사 delay sensitivity, actual navigation/motion을 표시한다.
Figure 2의 B0 원점 이동은 display-only이며 모든 native world anchor는 유지한다.
Stage C 미실행 패널은 그대로 미실행으로 표시한다. 네 PNG 모두 렌더링 후 직접 확인했다.

시작 Git SHA: `05f029f85693c658bc816decc992b54368879bc3`, upstream SHA:
`9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. 실제 실행은 새 wrapper의 dirty snapshot이며
파일별 해시와 diff identity를 저장했다. 최종 publication commit을 실행 당시 revision으로 바꾸지 않는다.
Base SHA-256 `a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`,
checkpoint SHA-256 `376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`.
해당 파일과 기존 Isaac installation/controller identity는 준비 단계에서 재검증했다.
실행별 Python/NumPy/torch/runtime 정보는 원본 `simulation-runtime.json`, `inference-runtime.json`에 보존했다.

실행 명령의 완전한 argv는 suite의 `trial-N/process-result.json`과
`trial-N/execution-start.json`에 저장했다. 사용한 진입점은 다음과 같다.
아래는 수행 내역이며, 기존 suite에서 다시 실행하면 overwrite 방지 검사로 거부한다.

```text
python3 -m tools.e16_wheelchair_delay.prepare --suite <fresh-suite>
python3 -m tools.e16_wheelchair_delay.analysis stage-a --suite <suite>
python3 -m tools.e16_wheelchair_delay.runner freeze --suite <suite>
python3 -m tools.e16_wheelchair_delay.runner launch --suite <suite> --index <1..4>
python3 -m tools.e16_wheelchair_delay.analysis trial --suite <suite> --index <same-index>
python3 -m tools.e16_wheelchair_delay.analysis gate --suite <suite>
python3 -m tools.e16_wheelchair_delay.plots --suite <suite>
```

각 launch 뒤 해당 trial의 saved 분석을 마친 다음 다음 index를 실행했다.
Launcher 내부는 `timeout --signal=INT --kill-after=30s 900s bash scripts/isaac6_python.sh`
를 거쳐 새 simulator process를 시작한다. Model worker 생성 코드는 기존 `research/loop.py`이며,
실제 executable·의존성 버전·checkpoint strict load는 각 `inference-runtime.json`에 기록했다.
준비·시작 점검·62개 테스트·runtime hash freeze를 모두 실제 navigation 전에 완료했다.
`trial-5` 및 `trial-6`은 생성되지 않았다.

Focused commit은 `TIC-VLA-TEST`에만 반영하고 `origin/TIC-VLA-TEST`로 일반 push한다.
코드·테스트·보고서·WORK_LOG만 포함하며 generated raw/derived/그림·모델은 제외한다.
