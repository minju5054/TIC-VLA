# E16 성공 주행의 Reconciliation 필요성 독립 검증

최종 판정: **`NO_RECONCILIATION_HARD_CASE_EVIDENCE`**.

실제 성공 주행의 90개 native prediction과 2,787개 로봇 상태를 검증하고, 연속 OLD/FRESH **89쌍 전부**를 계산했다. 목표 최초 진입 **32.583335 s 이전의 62쌍**이 주 분석 대상이며 이후 27쌍은 별도로 제외했다. FRESH 생성 중 실제 OLD 실행과 미래 경로의 수치 변화는 존재했지만, **안전성이 확인된 새로운 FRESH 우회 경로를 실제 switch 위치에 연결하면서 추가 위험이 생기는 구체적인 pair는 발견하지 못했다.**

이는 이 E16 성공 기록에서 reconciliation hard case를 입증하지 못했다는 결론이다. Graph가 일반적으로 불필요하다거나, 모든 FRESH 경로의 물리적 안전성이 증명됐다는 뜻은 아니다. 기존 Graph 결과·선정 pair·가중치·평가를 사용하지 않았으며 Graph 구현·실행, 새 VLA inference, Isaac navigation 재실행도 하지 않았다.

## 검증한 데이터와 성공의 범위

원본: `outputs/two-wheelchairs-v08-20261014T015420Z-a56068/`. 조건은 E16, 정지 C10/C24 휠체어, 최대 선속도 0.8 m/s와 기존 instruction이다. [기존 주행 기록](TWO_WHEELCHAIRS_V08.md)은 입력 식별과 측정된 주행 결과 확인에만 사용했다.

| 항목 | 독립 확인 결과 |
|---|---|
| Native prediction | C1–C90, 모두 유한 `(30,2)`, observation-local `[forward,left]`, m |
| 원본 해시 | 기존 raw manifest 1,092개 모두 일치; raw·기존 derived 등을 포함한 1,204개 입력 파일 보존 |
| State 정렬 | 60 Hz 연속 tick; observation/detection/application pose·quaternion·sim time과 CSV 일치 |
| OLD 실행 | 저장된 이전 `(v,w)` 및 wheel target을 다음 application까지 유지; waypoint 순차 실행 아님 |
| Controller 검증 | 90개 native chunk에서 명령과 lookahead index를 다시 계산해 저장값과 일치 |
| 휠체어 | 양쪽 2,787 tick씩, 총 5,574행; 위치·정적 USD matrix 변화 0 |
| 최초 목표 진입 | strict XY 거리 <2 m, tick 1955, 32.583335 sim s |
| 최소 / 종료 목표 거리 | 0.347256 / 8.984652 m |
| 실제 비바닥 접촉 보고 | 목표 이전 0, 이후 65; 이후 최대 impulse 0 N s, 최소 separation 0.005329 m |

성공은 휠체어 무접촉 통과와 목표 영역 진입을 뜻한다. 목표에서 정지하지 않고 계속 주행했으므로, 이후의 경로·접촉 보고는 navigation hard case 판정에 넣지 않았다. 이후 65 callback도 양의 separation과 0 impulse이므로 충격·침투의 증거로 보지 않는다.

## OLD와 FRESH 및 시각의 정의

`OLD=C_(k-1)`, `FRESH=C_k`로 두되, **OLD predicted future**, **실제 committed OLD history**, **FRESH predicted future**를 별도 보존했다. Committed history는 이전 application tick부터 이번 application tick까지의 실제 CSV 위치이며 prediction을 복사한 경로가 아니다. Pending OLD는 이번 observation부터 application까지의 부분 구간이다.

Hospital 원점, +Z up, XY는 m, yaw는 세계 +X에서 반시계 rad, 로봇 quaternion은 `wxyz`다. Observation body → world 변환은 `p_world=p_observation+R(yaw_observation) p_local`이다. Ready/switch pose로 FRESH를 재정렬하지 않는다. 휠체어 USD 기록의 row-vector local → world 행렬과 저장된 yaw/footprint를 구분해서 사용했다. Native output에는 yaw·timestamp가 없다.

소스 `ticvla/models/ticvla.py`의 10 Hz action contract와 `data/s01_batch_json_generation.py`의 `DST_HZ=10`을 확인해 nominal knot를 `t_obs+0.1…3.0 s`로 두었다. 이는 분석 convention이다. 시작 원점 knot를 추가하지 않고 공통 시간 구간에서만 선형 보간한다. 동일 nominal time RMSE는 공통 구간에서 piecewise-linear 차이의 제곱을 시간 적분한 값이다. Longitudinal/lateral 성분은 **FRESH observation body frame** 기준이다.

실제 boundary `B`는 application 시점의 저장 pose다. Boundary 위치 차이는 `B.xy`와 `FRESH(t_switch)` 사이 거리다. RAW connector는 **B에서 switch보다 엄격히 뒤에 있는 첫 native waypoint로 잇는 가상 선분**이며 실제 controller가 실행한 경로가 아니다. 첫 남은 FRESH segment는 nominal switch 보간점이 존재하면 그 점부터 다음 knot까지다.

Observation·inference start·ready·response detection·application의 monotonic clock 순서를 검증했다. 정확한 service-ready robot pose는 저장되지 않았다. Ready 이전에 완전히 끝난 tick과 inference start/ready 안에 완전히 포함된 physics tick의 이동만 각각 보고하며, 누락된 pose나 simulation timestamp를 만들어내지 않았다. Background reasoning의 생성·poll 시각은 action-call latency와 다른 항목이고 이 분석에서 하나로 합치지 않았다.

## 사전 고정 기준과 누락 처리

Pair 수치를 계산하기 전에 protocol을 저장했다. 동일 protocol SHA-256은 `457c9a294c3ddb27f5216c195a67d66cdde6e7c61fc4aa075b0f42af3af35b53`이며 구현 점검 과정에서도 기준을 바꾸지 않았다.

- Tangent는 첫 segment ≥0.02 m, 다음 segment도 충분히 길고 인접 heading 차이 ≤45°일 때만 유효하다. 초기 역행, 짧은 segment, 방향 반전을 별도 invalid로 남겼다. Heading을 보간하거나 만들어 채우지 않았다.
- 로봇 disk 반경은 기존 geometry 기록의 **0.606982542 m**다. 정확한 전체 swept collision mesh가 없어 이 보수적 proxy를 썼다. 휠체어는 실제 저장 yaw와 half-footprint `(0.552719161, 0.412630205)` m의 OBB, 병원 구조물은 해시 검증된 2,161 collider 중 로봇 높이와 겹치는 wall/doorframe/trim **828 AABB**다. 전체 동역학/모든 병원 소품의 안전성 증명은 아니다.
- Polyline sampling 최대 간격 0.01 m이며 거리 함수의 Lipschitz 성질로 `최소 sample clearance−최대 간격/2`를 보수적 하한으로 보고한다. FRESH가 proxy에서 clear하다는 판정은 그 하한 >0.02 m다. 음수는 proxy 겹침이며 실제 PhysX 충돌과 다르다.
- 의미 있는 우회 변화는 실제 예측 crossing의 좌우 전환, 또는 OLD의 proxy 겹침이 FRESH의 clear한 crossing으로 바뀌고 lateral RMS가 반경/2를 넘는 경우다. 단순 RMSE/heading 크기로 후보를 만들지 않는다.
- A 후보에는 위 변화, 유효한 nominal boundary/tangent, 실제 pending OLD 이동, clear한 FRESH와 B, **connector에서 새로 생긴 proxy 겹침**을 모두 요구한다. 큰 위치/각도 차이만의 효과는 표에 남기되 hard case로 승격하지 않는다. 이 엄격한 후보 규칙 밖의 모든 가능한 reconciliation 이점을 배제하는 것은 아니다.

Boundary 위치는 53/62쌍에서 계산 가능했다. FRESH request **8,12,26,32,34,38,46,48,50**은 switch가 첫 nominal knot보다 빨라 외삽하지 않았다. 초기 tangent는 52/62쌍에서 유효했다. Invalid FRESH ID는 **2,6,32,33,57,58,59,60,61,62**다. 전체 pair accounting에는 이 행들도 유지했다. 62쌍 모두 동일 nominal time의 유효 공통 구간은 있었다.

## 전체 handoff 측정값

아래 시각/이동 통계는 목표 이전 62쌍이다. 각도는 valid tangent 표본만, boundary 위치는 nominal interpolation이 가능한 53쌍만 집계했다. signed 성분의 부호와 전체 분포는 JSON/CSV에 보존했다.

| 측정값 | 유효 수 | 중앙값 | 최대 또는 최대 절댓값 |
|---|---:|---:|---:|
| Boundary 위치 차이 m | 53 | 0.009003 | 0.038597 |
| B yaw와 FRESH tangent 차이 deg | 52 | 2.769489 | 12.445157 |
| 실제 OLD 이동 방향과 FRESH tangent 차이 deg | 52 | 2.909805 | 11.964762 |
| Observation→switch sim s | 62 | 0.100000 | 0.200000 |
| Action inference wall s | 62 | 0.053830 | 0.156513 |
| Pending OLD 실제 이동 m | 62 | 0.080097 | 0.160178 |
| Inference에 완전히 포함된 tick의 이동 m | 62 | 0.040050 | 0.120132 |
| 동일 nominal time 위치 RMSE m | 62 | 0.224852 | 1.099910 |
| Longitudinal revision RMS m | 62 | 0.221759 | 1.092198 |
| Lateral revision RMS m | 62 | 0.027173 | 0.130024 |
| Controller delta_v m/s | 62 | 0.000000 | 0.000000 |
| Controller delta_w rad/s | 62 | -0.007262 | 0.062082 |

62쌍 모두 FRESH inference start/ready 안에 2–9개의 완전한 physics tick이 있었고 실제 OLD 이동이 관측됐다. Pending 동안 yaw 변화 최대 절댓값은 약 1.453°다. **OLD가 실행된다는 사실 자체는 확인됐지만 접합 실패를 뜻하지 않는다.**

60/62쌍은 longitudinal RMS가 lateral RMS보다 컸다. RMSE 최대는 C1→C2의 1.099910 m이며, 이때 첫 남은 segment가 0.014665 m여서 heading은 invalid다. Boundary 위치 최대는 C8→C9의 0.038597 m, 유효 heading 최대 절댓값은 약 12.445°다. 0.3 m급 future revision을 큰 switch 불연속으로 그대로 해석할 수 없다.

목표 이전의 모든 FRESH 명령 v는 0.8 m/s이며 delta_v는 0이다. `RMSE>반경/2`, `|delta_v|<0.05 m/s`, `|delta_w|<0.05 rad/s`인 설명용 표본은 FRESH ID **2,20,26,28,30,32,36,42,48**이다. 이는 가중합 점수나 후보 순위가 아니라 큰 prediction 차이가 명령 변화와 같지 않음을 보여주는 기록이다.

## 장애물 구간과 통과 방향

구간 시작은 실제 로봇 중심 거리가 `0.8×3 + robot_radius + norm(wheelchair_half)`인 **3.696738 m** 안으로 처음 들어온 tick이다. 끝은 reference tangent 방향으로 로봇 반경+휠체어 최대 반범위를 완전히 지난 tick이다. Pair의 observation–switch 구간이 이에 겹치면 포함했다. Visibility는 별도 관측 기록으로 표시했으며, 미래 prediction 결과로 구간을 고르지 않았다.

| 장애물 | 세계 중심 XY m | 고정 분석 구간 sim s | 실제 중심면 통과 sim s | 실제 방향 | 완전 통과 sim s | 포함 FRESH ID |
|---|---|---|---:|---|---:|---|
| C10 | (6.824887, 7.096358) | 4.483334–10.450001 | 8.988609 | LEFT | 10.450001 | 7–19 |
| C24 | (-1.756728, 10.988420) | 17.266668–23.433335 | 21.975753 | RIGHT | 23.433335 | 33–45 |

C10은 관측 C1부터 보였고 C24는 관측 C19에서 처음 픽셀이 기록됐다(C20에서 visibility 기준 충족). 이 관측 ID와 기준 경로의 장애물 이름 C10/C24는 서로 다른 의미다.

각 장애물 구간 13쌍 중 FRESH predicted crossing은 3쌍, `NO_PREDICTED_CROSSING`은 10쌍이었다. Crossing이 있는 FRESH는 C10에서 LEFT, C24에서 RIGHT이며 실제 통과 방향과 일치했다. OLD/FRESH 사이 좌우 crossing 전환은 0쌍이다. Horizon이 못 미치거나 이미 중심면 너머에서 시작한 경우 side를 강제로 지정하지 않았다. 좌우 부호는 clearance와 독립적이다. 따라서 C24의 오른쪽 통과가 disk proxy와 겹쳐도 실제 방향을 “중앙”으로 바꾸지 않는다.

### 대표 pair와 반증

C10/C24 대표는 각 고정 구간에서 FRESH predicted crossing과 유효 tangent가 처음 함께 존재하는 pair다. 결과 점수의 최댓값으로 선정하지 않았다. Control은 두 구간 밖에서 boundary gap≤반경/4, 유효 tangent, clear한 FRESH·connector가 확인되는 최초 pair다.

| 사례 | OLD→FRESH | observation sim s | switch sim s | Boundary 차이 m | B yaw 차이 deg | Pending 이동 m | 동일시각 RMSE m | FRESH clearance 하한 m | Connector clearance 하한 m |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| C10 | C12→C13 | 7.433334 | 7.533334 | 0.010049 | 6.103 | 0.080090 | 0.197797 | 0.014101 | 0.146500 |
| C24 | C38→C39 | 20.433334 | 20.533334 | 0.006606 | 2.282 | 0.080097 | 0.258488 | -0.233669 | -0.215176 |
| control | C2→C3 | 2.433333 | 2.533333 | 0.009536 | -8.776 | 0.080064 | 0.465618 | 2.019359 | 3.141375 |

- **C12→C13, C10:** 실제 OLD 이동 0.080090 m와 LEFT predicted crossing은 있다. OLD horizon은 아직 crossing하지 않아 좌우 전환 증거는 없다. FRESH clearance 하한 +0.014101 m는 양수지만 사전 0.02 m 기준에는 못 미쳐 D의 *unqualified*로 남겼다. Connector는 +0.146500 m이고 boundary gap은 0.010049 m다. 접합 위험의 증거가 없다.
- **C38→C39, C24:** FRESH는 RIGHT crossing을 제시하며 실제 RIGHT passage와 일치한다. FRESH 자체 proxy 하한 −0.233669 m이고 B에서도 구조물 AABB와 −0.210561 m 겹친다. Connector 하한 −0.215176 m를 새 접합 위험으로 볼 수 없다. 해당 FRESH에는 중심면을 재교차하는 작은 geometry 변화도 기록됐으며 전체 native array와 유효성 표를 보존했다.
- **C2→C3, control:** future RMSE 0.465618 m지만 longitudinal 성분이 0.463421 m다. Boundary gap은 0.009536 m, FRESH/connector 하한은 +2.019359/+3.141375 m다. 큰 future RMSE만으로 handoff 문제를 판정하면 이 정상 control도 잘못 선택하게 된다.

## FRESH geometry와 controller 제약

목표 이전 62쌍 중 FRESH remaining polyline이 선택한 proxy와 0.02 m 이상 여유를 확보한 것은 42쌍이다. 나머지 20쌍은 proxy 겹침 또는 여유 미확인으로 D에 분류했다. **실제 주행의 비바닥 접촉은 목표 이전 0건**이지만, 동일 proxy를 실제 성공 경로에 적용하면 C10에서 sample clearance −0.001689 m, C24에서 −0.220805 m, 병원 구조물에서 −0.253651 m가 나온다. Proxy 겹침은 실제 충돌·upstream failure의 확증이 아니며, 특히 C24에서 중요한 제한이다.

OLD/FRESH 전체 native polyline과 switch 이후 remaining polyline을 별도로 검사했다. 후보는 수정 가능한 FRESH future에서 평가했고 committed history를 이동시키거나 최적화하지 않았다. 현재 체크는 저장된 두 휠체어 OBB와 구조물 AABB에 한정된 기하학 진단이다.

현재 controller 제약은 `v≤0.8 m/s`, `|w|≤1.2 rad/s`다. Native knot를 0.1 s 간격으로 그대로 추종한다고 가정하면 모든 62쌍에 speed/turn-rate cap 초과 segment가 있고, 최대 nominal segment speed는 2.776342 m/s다. 유효 segment에서 유도한 최대 curvature는 약 23.802492 1/m다. 그러나 실제 controller는 이 timed queue를 추종하지 않고 1 m lookahead에서 단일 `(v,w)`를 만든다. 따라서 이 값으로 robot-level infeasibility나 실행 중 회전 불연속을 증명할 수 없다.

가상 connector의 초기 방향을 첫 future knot까지의 nominal 시간 안에 맞춘다고 가정한 각속도는 C12→C13에서 2.387313 rad/s, C38→C39에서 0.993295 rad/s다. 전자는 명령 상한을 넘지만 실제 실행 법칙이 아니고, FRESH 자체의 작은 segment 방향 변화도 nominal cap을 넘는다. 접합만의 문제로 국한할 근거가 아니다. 실제 명령은 각각 `(0.8,0.089786)→(0.8,0.041176)`, `(0.8,0.055823)→(0.8,0.043994)`였다.

## 전수 분류

분류 우선순위는 D(FRESH proxy 미확인), E(boundary/tangent/common time 부족), A(모든 후보 조건), B(우회 방향 변화지만 clear한 연결), C(그 밖의 수치 변화)다. D에 들어간 pair의 timestamp/tangent 누락도 accounting에 그대로 남겼다.

| 유형 | 목표 이전 pair 수 | 근거 |
|---|---:|---|
| A `POTENTIAL_RECONCILIATION_HARD_CASE` | 0 | 모든 조건을 만족하는 pair 없음 |
| B `INTENT_CHANGE_WITHOUT_HANDOFF_PROBLEM` | 0 | 고정 장애물 구간에서 좌우 crossing 전환 없음 |
| C `MISMATCH_WITHOUT_DEMONSTRATED_RISK` | 29 | 수치 변화는 있지만 새 접합 위험 근거 없음 |
| D `UNSAFE_OR_UNQUALIFIED_FRESH` | 20 | FRESH proxy 겹침/여유 미확인; 실제 충돌로 단정하지 않음 |
| E `INSUFFICIENT_EVIDENCE` | 13 | nominal boundary 또는 tangent 미확정 |

유형별 FRESH request ID는 다음과 같다. 각 ID k는 pair C(k−1)→Ck를 뜻한다.

- `MISMATCH_WITHOUT_DEMONSTRATED_RISK`: 3, 4, 5, 7, 9, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 27, 28, 29, 45, 47, 49, 51, 52, 53, 54, 55, 56, 63.
- `UNSAFE_OR_UNQUALIFIED_FRESH`: 10, 11, 12, 13, 14, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44.
- `INSUFFICIENT_EVIDENCE`: 2, 6, 8, 26, 46, 48, 50, 57, 58, 59, 60, 61, 62.

C10/C24 주변은 각각 13쌍을 전부 보존했다. C10의 전역 분류는 C=7, D=5, E=1이고 C24는 C=1, D=12다. 별도 obstacle 판정은 FRESH crossing이 없으면 `INSUFFICIENT_EVIDENCE / NO_PREDICTED_CROSSING`으로 기록한다. Post-goal C64–C90의 27쌍은 모두 `POST_GOAL_EXCLUDED`다. C1은 OLD가 없는 bootstrap이다.

## 연구 질문에 대한 답

**Q1. Geometric/temporal inconsistency가 관측되는가?** OLD/FRESH future 차이와 pending OLD 이동은 관측된다. 그러나 유효 boundary 차이는 중앙값 9.003 mm, 최대 38.597 mm로, 큰 future RMSE와 구별된다. 실제 로봇 위치의 순간 점프를 측정한 값은 아니다.

**Q2. 장애물 회피 intent 변화인가, 진행 정도·nominal timing 차이인가?** 60/62쌍에서 차이가 longitudinal 성분에 더 크고, 관측 가능한 predicted crossing 방향 전환은 없다. 같은 진행 방향에서 미래 진행량이 바뀐다는 해석이 더 잘 맞는다. 내부 intent나 차이의 원인을 nominal timing 하나로 확정하지 않는다.

**Q3. 실제 switch boundary와 FRESH future 사이에 문제가 있는가?** Clear한 B와 clear한 FRESH 사이에 connector가 새 proxy 겹침을 만드는 pair는 0쌍이다. 측정된 명령 변화도 `delta_v=0`, 최대 `|delta_w|=0.062082 rad/s`다. 이번 데이터에서 실증된 접합 위험은 없다. 9쌍의 nominal boundary와 10쌍의 tangent는 미확정 상태로 남긴다.

**Q4. FRESH 자체는 기하학적으로 안전한가?** 42쌍은 선택한 conservative proxy의 0.02 m 여유를 확보한다. 20쌍은 unsafe 또는 unqualified proxy 결과다. 정확한 swept mesh와 물리적 실행 가능성은 검증하지 못했으며, 실제 성공 경로에서도 proxy false-positive 겹침이 나온다. 무접촉 주행을 FRESH 안전성의 대리 증거로 쓰지 않는다.

**Q5. FRESH future를 수정하는 SE(2) reconciliation이 적절한가?** 이 기록에서는 수정할 접합 문제를 입증하지 못했다. 특히 FRESH 자체 또는 보수적 geometry 판정의 문제가 남은 상태에서 OLD에 FRESH를 강제로 맞출 근거는 없다. Future tangent는 파생 값이며 native yaw가 아니므로, 이를 graph 회전 제약으로 확정하는 것도 별도 검증이 필요하다.

**Q6. Graph 비교 가치가 있는 구체적 handoff pair가 있는가?** 사전 A 조건을 만족하는 pair는 없다. C12→C13·C38→C39는 장애물 진단 예시, C2→C3는 control이며 Graph 성능 평가용 hard case로 선정하지 않는다.

**Q7. 왜 이번 성공 실험은 reconciliation hard case로 입증되지 않았는가?** 실제 OLD transport는 있었지만 boundary 차이는 작고, 새로운 반대편 우회 intent가 확인되지 않았으며, connector만의 추가 위험도 없었다. 남은 큰 future 차이, 불안정한 tangent와 proxy 겹침은 접합 문제의 충분한 증거가 아니다. **이번 E16 데이터에서는 Graph 필요성이 입증되지 않았다.**

## 사실과 해석 및 미검증 가설

- **Repository에서 확인한 사실:** 90개 실제 prediction, 목표 이전 62쌍, OLD 명령 유지와 실제 이동, 저장된 시각·pose 일치, 위 수치·접촉 기록·분류. 성공 경로는 C10 LEFT/C24 RIGHT였고 goal region에 진입했다.
- **연구적 해석:** 확인된 차이는 주로 longitudinal future revision이며 현재 controller 하의 명확한 접합 위험으로 연결되지 않았다. 이 데이터로 Graph 우월성 실험을 시작할 근거가 부족하다.
- **미검증 가설:** 더 긴 실제 action latency, 명확한 우회 방향 변경, 정확한 swept geometry 조건에서는 reconciliation 문제가 생길 수 있다. 이번 작업에서는 이를 만들어내거나 검증하지 않았다. Graph가 전역적으로 불필요하다는 가설도 검증하지 않았다.

## 산출물과 재현

최종 분석 디렉터리: `outputs/e16-wheelchair-handoff-audit-20261014T073502Z-96bd3c/`. 아래 파일은 로컬 ignored output이며 GitHub에는 분석 코드·테스트·이 보고서만 반영한다.


| 파일 | 내용 |
|---|---|
| `pair-accounting.csv` | 89쌍 전수 포함/제외·validity·판정 |
| `pair-metrics.csv`, `pair-metrics.json` | Boundary, clocks, OLD transport, 명령, measured velocity, proxy·curvature 수치 |
| `c10-candidate-rejections.json`, `c24-candidate-rejections.json` | 각 고정 구간의 모든 후보/제외 근거 |
| `distributions.json`, `data-validation.json` | 유효 표본 수, 분포, 접촉·목표·원본 검증 |
| `protocol.json`, `windows-before-metrics.json` | 사전 기준과 결과 독립적인 obstacle 구간 |
| `source-before.json`, `analysis-provenance.json`, `final-verdict.json` | 전체 원본 해시, 코드·Git·환경 출처, 최종 판정 |
| `figures-reviewed/figure1-overview.png` | 실제 경로, 휠체어 footprint, passage, 최초 goal entry, 분석 구간 |
| `figures-reviewed/figure2-c10.png` | C12→C13의 committed OLD / OLD future / FRESH / observation / B / proxy |
| `figures-reviewed/figure3-c24.png` | C38→C39의 같은 구성 |
| `figures-reviewed/figure4-metrics.png` | 전수 metric timeline, invalid mask, visibility/passage, post-goal 제외 |
| `figures-reviewed/figure5-control.png` | C2→C3 control |

모든 trajectory 축은 world XY에서 X와 Y 길이 척도가 같으며, 세 pair 그림은 각각 동일한 8×8 m 범위다. 원본 그림은 남겨두고 라벨·범례를 정리한 판을 `figures-reviewed/`에 추가했다. 그림 5개를 직접 확인했다.

새 디렉터리를 사용해 동일 saved-only 분석을 재현할 수 있다. `--prepare-only`는 변경하지 않은 내장 protocol과 입력 해시를 먼저 고정하며 기존 디렉터리를 거부한다.

```bash
cd /home/gpuadmin/Workspace/TIC-VLA-TEST
SOURCE_RUN="outputs/two-wheelchairs-v08-20261014T015420Z-a56068"
AUDIT_RUN="outputs/e16-wheelchair-handoff-audit-$(date -u +%Y%m%dT%H%M%SZ)-$$"
python3 -B -m research.audit_e16_wheelchair_handoffs --source "$SOURCE_RUN" --output "$AUDIT_RUN" --prepare-only
MPLCONFIGDIR=/tmp/tic-vla-mpl python3 -B -m research.audit_e16_wheelchair_handoffs --source "$SOURCE_RUN" --output "$AUDIT_RUN"
```

## 검증과 출처

신규 수학·무결성 테스트 **30개 PASS**, 기존 continuous 테스트 **7개 PASS**. 추가 기존 Nova 회귀는 **10 PASS / 1 FAIL**이다. 실패는 `tests/test_nova.py:38`에서 보존된 `configs/research/nova_carter_hospital_episode16.yaml`의 runner SHA `e0f9aeb…`와 현재 `DynaNav/benchmark.py` SHA `9e9f432…`가 다른 기존 source identity 불일치다. 해당 test/config/source는 이번 diff에 없으며 수정하지 않았다. 실제 선택 run의 controller 및 loader source hash는 현재 검증한 함수와 일치했다.

초기 두 분석 시도는 quaternion 재계산의 지나치게 엄격한 1e-8 검사에서 멈췄다. 기록 당시 NumPy 2.3.1과 offline NumPy 1.26.4의 float32 norm 연산 차이를 확인해 그 산술 검사에만 `8×float32 epsilon`을 적용했다. Canonical float64 yaw와 기록 yaw의 최대 차이는 1.892e-7 rad, event/CSV pose 차이는 0이다. 초기 완료판에서 실제 RIGHT passage와 conservative corridor overlap을 같은 라벨로 취급한 표기를 발견해 두 필드로 분리했다. 통과 측면의 signed distance, threshold, pair 선정 기준은 바꾸지 않았다. 각 시도와 원본 결과는 별도 폴더에 남겼다.

실행된 분석기 snapshot과 최종 commit 대상의 **측정 함수 AST가 동일**함을 확인했다. 이후 변경은 그림 표시와 동일한 고정 protocol의 재현용 prepare CLI뿐이다. Required/legacy 테스트 로그와 code/hash 검증 기록을 audit에 보존한다. 새로운 추론·simulator·controller·optimizer를 실행하거나 수정하지 않았다. 단위 테스트의 수학 fixture는 실제 E16 결과에 포함하지 않았다.

분석 시작 HEAD/원격: `5be3384009ccadbfed41324fb2abe6689685379e`, branch `TIC-VLA-TEST`. 실제 주행 시작 SHA: `e8d1f0b7821af96eab2c236803e34e0166c30219`, upstream: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. 주행 당시 `tools/two_wheelchairs_v08/`는 미추적 상태였고 별도 provenance에 파일 해시가 보존됐다. 이 audit의 commit을 주행 당시 revision으로 재표기하지 않는다.

모델 기록은 strict checkpoint, 별도 Isaac 5 Python inference, native BF16 `(1,30,2)`다. Base SHA-256 `a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`, checkpoint SHA-256 `376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`. Simulation은 Isaac 6.0.1, 모델 worker는 Isaac 5.0.0이며 이는 수정 harness의 시뮬레이션 증거다. Offline 분석은 system Python 3.12 / NumPy 1.26.4를 사용했다. 설치·환경 변경은 없다.


## cm 단위 접합 확대도

사용자 요청에 따라 같은 대표 pair의 표시 단위만 cm로 바꾸고 확대했다.
새 산출물: `outputs/e16-handoff-cm-zoom-20261014T081148Z-4ea1bc/`. 이전 audit와 raw는 변경하지 않았다.

| 사례 | Pair | B와 FRESH nominal switch 위치 차이 cm | Pending OLD 실제 이동 cm |
|---|---|---:|---:|
| C10 | C12→C13 | 1.005 | 8.009 |
| C24 | C38→C39 | 0.661 | 8.010 |
| Control | C2→C3 | 0.954 | 8.006 |

`handoff-closeup-6cm.png`는 각 패널 가로·세로 6 cm(±3 cm),
`handoff-context-80cm.png`는 가로·세로 80 cm(±40 cm)다. SVG도 함께 저장했다.
실제 switch B를 표시 원점으로 두고 `q_cm = 100 × (p_world_m − B_world_m)`만 적용했다.
세 패널의 XY 축척은 동일하고 world 축 방향은 회전하지 않았다.
OLD/FRESH는 원래 observation pose에 투영한 값을 그대로 사용한다.
이는 표시 원점 이동이며 FRESH를 switch pose로 다시 anchor하는 연산이 아니다.
빨간 X는 실제 B, 초록 마름모는 기존 nominal timing으로 계산한 FRESH switch 위치다.
자주색 점선은 실행 궤적이 아닌 가상 connector이며 작은 창 밖의 곡선은 잘려 보인다.
원본 prediction, pair 선정, threshold, metric, 판정은 바뀌지 않았다.

전체 유효 boundary gap의 중앙값/최대는 **0.900 / 3.860 cm**다.
두 확대 그림을 직접 확인했으며 원본 1,204개 파일과 이전 audit 파일의 해시 보존을 확인했다.
표시 전용 변경이므로 신규 동역학 테스트나 navigation 실행은 하지 않았다.

```bash
cd /home/gpuadmin/Workspace/TIC-VLA-TEST
CM_OUT="outputs/e16-handoff-cm-zoom-$(date -u +%Y%m%dT%H%M%SZ)-$$"
MPLCONFIGDIR=/tmp/tic-vla-mpl python3 -B -m research.plot_e16_handoff_cm \
  --audit outputs/e16-wheelchair-handoff-audit-20261014T073502Z-96bd3c \
  --output "$CM_OUT"
```
