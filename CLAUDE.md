# 2026 HL FMA 자율주행 시뮬레이션 대회 — 제어기

VTD 시뮬레이터와 TCP/UDP/RTSP로 통신하며 지정 경로를 자율주행하는 제어기를 만든다.
이 코드는 **제어기 PC**에서 실행된다. VTD PC는 주최측이 준비하므로 수정 대상이 아니다.

- 대회일: 2026년 9월 12일
- 사전테스트: 2026년 9월 4일 **완료** (경로 2종 x 시나리오 6개 전부 completed)
- 제어기 PC: Windows / PowerShell / miniconda `hlfma` (Python 3.11, **표준 라이브러리만**)
- VTD PC: Ubuntu 24.04

## 현재 상태 요약

경로 추종은 완성됐고 실전에서 검증됐다. route CSV -> 주행 waypoint 자동 생성도 끝났다.
**남은 것은 규칙 대응 전부와 4단계(정지선·보호구역·신호등 좌표)다.**

| 영역 | 상태 |
|---|---|
| 통신·파싱·로깅 | 완료. 현장 VTD에서도 재현 확인 |
| 경로 추종 (Pure Pursuit) | 완료. 40km/h 825m 완주, 리스폰 0 |
| 구간 연결 | 완료. 9구간 이어붙이기 검증 |
| 도로망 (xodr 파서) | **1~3단계 완료.** 차로 연결 그래프 + 경로 탐색까지. 정지선·신호등 남음 |
| 규칙 대응 15개 | **미착수.** 점수는 여기서 갈린다 |

## 네트워크

| 포트 | 프로토콜 | 방향 | 용도 |
|---|---|---|---|
| 9910 | TCP | 제어기 <-> VTD (양방향, 20Hz) | 상태 수신 / 제어 송신 |
| 8554 | RTSP | 제어기 -> VTD (접속) | 전방 카메라 h264 1920x1080 20fps |
| 9912 | UDP | VTD -> 제어기 | LiDAR 포인트 클라우드 |

대회 당일 IP: 제어기 PC `192.168.50.10`, VTD PC `192.168.50.11`, 넷마스크 `255.255.255.0`.

## 9910 송신 — 제어 명령 (9바이트, 헤더 없음)

```python
CTRL = struct.Struct('<ffB')   # 9 bytes
# steering     float32  조향각 [rad]   양수=좌회전, 음수=우회전
# targetAccel  float32  목표 가속도 [m/s^2]  음수=감속
# turnSignal   uint8    0=OFF 1=LEFT 2=RIGHT
```

- 실용 범위: steering ±0.35 rad, targetAccel -6.0 ~ +3.0 m/s^2
- 20Hz로 끊김 없이 전송. 소켓에 `TCP_NODELAY` 설정 필수
- **연결이 끊기면 VTD 측에서 steering=0, accel=0으로 강제된다.** 재연결 로직 필수

## 9910 수신 — 상태 데이터 (1109 B 고정, 헤더/구분자 없음)

```
Ego (24B)
  egoX, egoY, egoZ               float32  [m]
  egoHeading, egoPitch, egoRoll  float32  [rad]

Object[30] (36B x 30 = 1080B)
  id                         uint32
  x, y, z                    float32  [m]
  heading                    float32  [rad]
  speed                      float32  [m/s]
  length, width, height      float32  [m]

TrafficLight (5B)   <- 배열이 아니라 정확히 1개
  id                         int32
  state                      uint8
```

신호등 state: `0`=NONE `1`=적색 `2`=황색 `3`=녹색 `4`=좌회전 `5`=녹색+좌회전 `6`=점멸

- Ego 제외, 수평거리 80m 이내 객체를 가까운 순으로 정렬해 최대 30개
- `id == 0` 슬롯은 무효. 배열 인덱스는 매 프레임 바뀌므로 추적에는 `id`를 쓴다
- **`state == 0`(NONE)은 "해당 신호등 없음"이다.** 교차로를 벗어나면 NONE이 되고
  다음 교차로에 접근하면 새 id가 할당된다. 정지 로직에서 반드시 예외 처리할 것
- `trafficLights.id`는 위치나 순서를 뜻하지 않는다. state만 쓴다

## 사전테스트 실측 (2026-09-04, 6개 세션)

### 객체 치수 — 분류 기준 확정

| 분류 | L x W x H | 관측 속도 | 등장 |
|---|---|---|---|
| 라바콘·소형 장애물 | 0.30 x 0.30 x 0.32~0.96 | 0 | 시나리오 1,2,3 |
| 보행자 | 0.60 x 0.70 x 1.80 | 최대 3.6 m/s | 시나리오 3 |
| 자전거·이륜차 추정 | 2.08 x 0.85 x 1.93 | 0 | 시나리오 2,3 |
| 승용차 A | 5.18 x 1.91 x 1.45 | 0 | 시나리오 2,3 |
| 승용차 B | 4.39 x 1.81 x 1.35 | **최대 16.7 m/s (60km/h)** | 시나리오 5,6 |

**`length`로는 구분되지 않는다** (자전거 2.08m > 보행자 0.60m). `width`와 `height`를 쓴다.

```python
if height < 1.2:      OBSTACLE     # 0.32, 0.96
elif width < 1.2:     PEDESTRIAN   # 0.70, 0.85 — 자전거도 여기 묶는 게 안전
else:                 VEHICLE      # 1.81, 1.91
```

### objects[].id 안정성

원칙적으로 유지된다 (6개 세션 중 5개에서 결손 0). **다만 시나리오 3에서
id 5가 한 프레임에 89 m/s 이동 — 삭제·재생성으로 인한 id 재사용이 실제로 발생했다.**

id를 그대로 믿지 말고 프레임 간 이동거리로 검증할 것.

```python
if dist(prev_pos, curr_pos) > 30.0 * 0.05:   # 108km/h 초과는 불가능
    # 같은 id지만 다른 객체다. 추적 상태를 리셋
```

### 신호등 거동

- 전이 패턴: `GREEN` -> `YELLOW` -> **정확히 3초 후** `RED`
- 교차로 통과 시 `NONE`(id 0), 다음 교차로 접근 시 새 id 할당
- **같은 경로라도 시나리오마다 신호 위상이 다르다.** 경로 2에서 시나리오 4는
  네 신호가 전부 RED였고 시나리오 5는 두 개가 GREEN이었다.
  운으로 통과할 수 없으므로 정지 로직은 필수다
- 6개 세션에서 **적색 신호를 총 15회 이상 통과**했다 (항목 7 중대 -6점씩)

### ego(0,0)은 시뮬레이션 종료 신호

완주 후 VTD가 `ego(0,0), heading 0, n_obj 0`을 보낸다.
**리스폰으로 오판하면 안 된다** — 현재 코드는 경로 인덱스를 재탐색하고
`accel=3.0`을 계속 송신한다. 종료로 판정해 안전 정지로 전환할 것.

### 수신 주기

주행 중 평균 50.0ms, 중앙 62ms, p95 63ms, 최대 79ms.
65.5ms 초과 프레임은 1.1~3.4% 수준으로 안정적이다.
(과거 로그의 13.9초 공백은 완주 후 대기 시간이었고 주행 중 문제가 아니었다)

### 경로 구조

| | 구간 | 길이 | 신호등 |
|---|---|---|---|
| 경로 1 | 7 | 825m | 3개 |
| 경로 2 | 9 | 737m | 4개 |

40km/h로 75~79초에 완주. 리스폰 0회. 조향·감속 모두 안정적이었다.

## 알려진 공백과 대응

**egoSpeed가 명세에 없다.** 평가 시스템은 VTD 내부 RDB의 실제 속도를 쓰지만
참가자에게는 오지 않는다. **추정치와 평가 기준이 어긋날 수 있으므로 안전 마진이 필요하다.**

추정 방식: 최근 5프레임 누적 이동거리 ÷ 경과시간. 경과시간은 측정값이 아니라
공칭값(구간수 × 0.05s)으로 스냅한다 — VTD는 20.0Hz 고정 스텝이고 TCP라 프레임이
유실되지 않으므로 구간 수를 정확히 알 수 있다. 한 프레임 차분 + EMA 방식은
수신 시각 지터가 속도로 비례 증폭되어 버렸다. `hlvtd_client.SpeedEstimator` 참고.

**정지선·보호구역 좌표가 오지 않는다.** 주최측은 "센서로 식별하라"고 했으나,
**xodr에 오브젝트로 들어 있다**(아래 참고). 파일 내 모든 정보 사용은 허가받았다.

**차선/도로 ID가 9910으로 오지 않는다.** xodr 파싱으로 해결한다.

## xodr에서 추출 가능한 것 (확인 완료)

`HL_FMA_VTD_LivingLab.xodr` 8.2MB, road 651 / junction 94,
geometry: line 1527 / arc 656 / spiral 1246 / poly3 571

| 요소 | 개수 | 용도 |
|---|---|---|
| `Rm_StopLine_300cm_JPN_01.flt` | 710 | **정지선** (항목 7) |
| `<signal>` | 646 | 신호등 위치. `type`/`subtype`, `<validity fromLane toLane>` 포함 |
| `roadmark_speed_30.flt` | 71 | **보호구역 30km/h 구간** (항목 2) |
| `RM_517_50.flt` | 69 | 50km/h 표시 |
| `Rm_Warning_Crosswalk_JPN_03.flt` | 400 | 횡단보도 (항목 12) |
| `RM_537_ST / LT / RT` | 458/144/134 | 차로별 진행방향 (직진/좌/우) |
| `RM_538_SRT / SLT` | 168/97 | 직진+우, 직진+좌 |

**차로 진행방향 표시가 있으므로** 대회 경로의 다음 기동에 맞는 차로를 고를 수 있다.
방향지시등(항목 13)에도 쓰인다.

**카메라 인식은 불필요하다.** 정지선·보호구역·신호등 위치가 전부 좌표로 나온다.

`<lane>`은 `type="driving"`만 뽑으면 되고, `<width>`는 대부분 `b=c=d=0`인 상수다.
`<link><predecessor/><successor/>`에 junction 연결이 명시돼 있어 세그먼트 그래프를
따로 만들 필요가 없다.

spiral은 Fresnel 적분 없이 **0.1m 간격 수치적분으로 충분하다** (`math`만 사용).

## xodr 3단계 — 차로 연결 그래프 (완료)

`src/xodr_graph.py`. route CSV(`seq,x,y`)를 받아 `pure_pursuit`이 그대로 먹는
주행 waypoint를 만든다. 노드는 `(road_id, section, lane_id)`, 엣지는 4종
(section / road-road / junction / 차선변경) 총 4,447개.

```
python xodr_graph.py --route ../data/route_X.csv --out ../data/route_path.csv
python xodr_graph.py --validate --dump-graph      # 진단 + 그래프 저장
python xodr_graph.py --load-graph --route ...     # xodr 재파싱 생략 (1.3s -> 0.03s)
```

전수 조사로 확정한 사실 (그래프 정확성의 근거)

- **junction 진입은 `<connection><laneLink>`가 유일한 정보원이다.** 진입도로의
  junction 방향 lane `<link>`는 1073건 전부 비어 있다. 한 차로가 최대 3개
  connecting road로 갈라지므로(1개:948 2개:121 3개:4) lane link로는 표현이 안 된다.
  **탈출은** 반대로 connecting road 자신의 road link + lane link로 완전히 기술된다
- connecting road 499개는 `<connection>`에 정확히 1회씩 등장하고
  **contactPoint가 499개 전부 `"start"`** — 항상 s=0에서 진입해 +s로 달린다
- 차로 부호 ↔ contactPoint 규칙(start→음수, end→양수) **671건 전수 일치, 위반 0**
- **`xodr_lanes.csv`의 heading은 reference line 접선이다.** 양수 차로의 실제 주행
  헤딩은 `heading + π`. 이 보정이 없으면 역주행 차로를 정주행으로 고른다
- **차선변경 엣지는 필수다.** 없으면 route_example은 탐색 자체가 실패하고
  route_pretest_2는 737m 경로가 3,812m 우회로 나온다
- 차선변경 비용에 구간 길이를 더하면 안 된다. 옆 차로로 옮겨도 세로 길이는 한 번만
  달리므로, 더하면 같은 구간을 두 번 세서 경로가 +32% 부풀었다
- 차선변경은 **한 점에서 3.2m 튀면 안 된다** (lookahead 7m 기준 조향 0.31 rad,
  한계 0.35에 붙는다). 20m에 걸쳐 두 차로 중심선을 smoothstep으로 섞는다
- 2단계 산출물의 **끝점 9개가 이상치**다 (road 429 / 2396, laneSection 끝에서
  헤딩이 88~107도 꺾인다). 3단계 로드 시 걸러낸다. 2단계는 건드리지 않는다

설계 결정

- 경유지 스냅은 "가장 가까운 차로"가 아니라 **"가장 가까운 도로 + 그 섹션의
  driving 차로 전부"**를 후보로 연다. 주최측이 "좌회전 구간에 3차로 좌표가
  주어질 수도 있다"고 했으므로 차로는 좌표가 아니라 다익스트라가 정한다
- 비용 = 거리 + 차선변경 페널티(25m). 경유지 좌표는 제약으로만 쓴다
- 차선변경 게이트는 roadMark `type`이 아니라 **`laneChange` 속성**을 본다
  (`both` 허용 / `none` 금지). 실선을 넘어야만 풀리면 폴백하고
  **"실선 차로변경 N회 포함 — 평가항목 6 경미 -3점 예상" 경고를 출력한다**
- 출력 CSV는 `waypoints_*.csv` 스키마 + `road_id,lane_id,section,s,width,lc_dir,lc_solid`.
  `pure_pursuit.load_path`는 `segment_id,x,y`만 읽고 DictReader가 여분 컬럼을
  무시하므로 **pure_pursuit 수정이 필요 없다.** `lc_dir`/`lc_solid`는 항목 13
  방향지시등이 쓴다

검증 결과 (`--validate`)

| | 탐색 | 녹화 | 오차 | 도로 일치 | 차로 일치 |
|---|---|---|---|---|---|
| route_example | 822.3 m | 802.8 m | +2.4% | 100% | 97.4% |
| route_pretest | 834.5 m | 824.2 m | +1.3% | 100% | 40.5% |
| route_pretest_2 | 750.2 m | 731.6 m | +2.6% | 100% | 96.5% |

- 녹화 궤적과의 횡오차 중앙값 0.12 / 3.03 / 0.16 m
- 연결 끊김 88건 중 **선택된 경로 위는 2건뿐이고 둘 다 차선변경으로 빠져나간다**
  (끝나는 차로라 원래 비켜야 한다). 나머지 78건은 경유지 50m 밖 주차장 진입로 등
- **route_pretest는 전 구간 한 차로씩 바깥으로 달린다**(차로 일치 40.5%).
  기하 오류가 아니라 비용 최소 규칙의 결과다 — 그 좌표가 실제로 바깥 차로에 가깝다.
  직진 구간에 우회전 전용 차로를 고를 위험이 있으므로 **4단계 RM_537
  (차로별 진행방향 표시)로 보정할 대상이다**
- route_pretest_2 p10은 선택 차로가 11.4m 떨어져 경고가 뜬다. 교차로에서 바깥
  차로로 진입한 뒤 안쪽으로 3번 옮기는 비용이 더 커서 그대로 둔 결과다

## 대회 경로 CSV

```
seq,x,y
```

- VTD 월드 직교좌표, m. egoX/egoY와 동일 좌표계
- 첫 지점=출발, 마지막=종료. 사이 지점은 두 개씩 짝을 이뤄 교차로 진입·진출
- **경유지는 대략적인 위치다.** 주최측: "임의로 피팅하여 제공하며 무조건 밟아야
  하는 것은 아니다. 좌회전 구간에 3차로 좌표가 주어질 수도 있으니 그럴 때는
  1차로로 주행하면 된다"
- 따라서 **체크포인트에 정밀 스냅하면 안 된다.** 가까운 도로를 찾고,
  그 도로의 driving 차로 중 다음 기동에 맞는 것을 골라야 한다
- 본 대회 경로는 사전테스트와 좌표가 다르다 (형식만 동일)

## 본 대회 진행 절차 (15분)

```
1. 제어기 PC 준비
2. 랜선 연결              <- 여기서 카운트다운 시작
3. 경로 USB로 전달받아 복사
4. 시뮬레이션 시작         <- 코드 수정은 여기까지만 가능
5. 제어기 작동 시작        <- 이후 PC 조작 시 즉시 종료·미완주 처리
```

시간이 남으면 재시도 가능하나 5번 이후 조작 불가는 동일하다.
**4번 전에 커버리지를 확인하고 판단할 시간이 있다.**

## 평가 규칙 (전략에 직결)

- 구간별 100점 시작. 경미 -3, 중대 -6. 동일 항목은 구간당 1회만 감점
- **완주 그룹은 미완주 그룹보다 항상 상위.** 완주가 최우선
- 완주 시간은 총점 동점일 때만 쓰는 2순위 -> **속도를 무리할 이유가 없다**
- 제한속도 이하 감속·저속 주행은 감점 없음 (주최측 확인)
- 예외: 녹색신호 정지선 30m 이내 이유 없이 5초 이상 정차 시 감점.
  단 전방에 차량·보행자가 있으면 면책
- 제한속도: **전 구간 50km/h, 붉은색 보호구역만 30km/h**
- 보호구역 판정은 Ego 후륜축 Position이 붉은 구역 위에 있을 때만
- 리스폰: 구간당 1회 무료, 이후 -6점. 도로 이탈 0.3초 / 경로 이탈 0.5초
- 리스폰 시 위치가 강제 이동되고 속도가 0이 된다
- NPC는 기본 법규는 지키나 **끼어들기·급정지 이벤트가 발생**하며,
  NPC 귀책이어도 충돌은 참가 차량 감점

**평가는 VTD 내부 속도로 하고 내 추정치와 다를 수 있으므로**
보호구역 27km/h, 일반 45km/h 정도로 마진을 두는 것이 안전하다.

## 개발 환경

- VTD PC(Ubuntu) 192.168.50.11 <-> 제어기 PC(Windows) 192.168.50.10, 랜선 직결
- **실행 순서: 랜선 -> `nmcli con up hlfma` -> `lmgrd` -> `vtdStart.sh`**
  (순서가 바뀌면 LiDAR 전송기가 소켓 생성에 실패해 시작되지 않는다)
- Setup은 반드시 **`00_HL_VTD`**. `Standard`에는 `libHLVTD.so`가 없어 9910이 열리지 않는다.
  판별법: mainRS / mainRS_Broadcast / mainRS_LiDAR 창 3개가 뜨면 `00_HL_VTD`
- `00_HL_VTD`는 Ego의 외부 제어를 전제하므로 **Path Shape 자율주행이 동작하지 않는다**
- **matplotlib과 numpy 모두 Windows 앱 제어 정책에 차단됨.**
  `import numpy`는 통과하지만 `numpy.linalg`에서 DLL 로드가 실패한다.
  얕은 import 테스트로는 안 걸러지니 주의. 시각화는 SVG 직접 생성
- PowerShell이라 힙독(`<<EOF`), `awk` 등 Unix 문법은 동작하지 않는다.
  임시 `.py` 파일을 만들어 실행할 것

## 작업 방침

- **표준 라이브러리만 쓴다.** 선호가 아니라 제약이다
- 제어 루프는 20Hz를 지킨다. 블로킹 연산을 루프 안에 넣지 않는다
- 예외로 루프가 죽으면 안 된다. 수신·연산 실패 시에도 마지막 유효 명령이나
  안전 감속을 계속 송신한다
- 모든 수신 프레임을 `logs/`에 기록해 사후 분석이 가능하게 한다
- `reference/`, `docs/`는 주최측 원본이다. 읽기만 한다 (gitignore 대상)
- 파일 삭제·이동, `git reset/clean`, 네트워크·conda 환경 변경은
  실행 전에 이유를 설명하고 확인받을 것
- `data/`의 waypoint CSV는 재수집이 어려우므로 절대 삭제하지 말 것

## 데이터 파일

| 파일 | 용도 |
|---|---|
| `data/xodr_reference_lines.csv` | **xodr 1단계 산출물.** road 651개 중심선, 1m 간격 34,223점 |
| `data/xodr_lanes.csv` | **xodr 2단계 산출물.** 주행차로 2,471개 중심선 95,306점 + roadMark |
| `data/xodr_graph_nodes.csv` / `_edges.csv` | **xodr 3단계 산출물.** 노드 2,471개 / 엣지 4,447개. `--load-graph`로 재파싱(1.3s)을 0.03s로 줄인다 |
| `data/route_path*.csv` | 3단계가 생성한 주행 waypoint 예시 (pretest 1·2, example) |
| `data/waypoints_v2.csv` | 예시 경로 P1~P8, 777m. **xodr 파서 검증용 정답지** |
| `data/waypoints_pretest1.csv` | 사전테스트 경로 1 (7구간 825m) |
| `data/waypoints_pretest2.csv` | 사전테스트 경로 2 (9구간 737m) |
| `data/route_pretest.csv` / `_2.csv` | 사전테스트 경로 원본 |
| `data/segment_info.csv` | 구간별 회전 방향·차로·제한속도 |
| `logs/pp_20260904_*` | **사전테스트 현장 로그 6개.** 객체·신호 실측 원본 |

v1 파일들은 첫 녹화 시도라 리스폰 오염이 있다. 참고용.
사전테스트 waypoint는 본 대회에 쓸 수 없다 (좌표가 다름). xodr 파서 검증용으로 남긴다.

## 제어기 구현 현황

| 파일 | 역할 |
|---|---|
| `src/xodr_parser.py` | **xodr 1단계.** planView -> reference line CSV, `--validate` 판정 리포트 |
| `src/xodr_lanes.py` | **xodr 2단계.** lane 구조 -> 주행차로 중심선 + roadMark CSV |
| `src/xodr_graph.py` | **xodr 3단계.** 차로 연결 그래프 + route CSV -> 주행 waypoint, `--validate` |
| `src/hlvtd_client.py` | 9910 파서·클라이언트, `SpeedEstimator`, 리스폰 감지. 단독 실행 시 모니터 |
| `src/pure_pursuit.py` | Pure Pursuit 경로 추종 |
| `src/drive_and_record.py` | WASD 수동 주행 + waypoint 녹화 |
| `src/route_check.py` | route CSV 커버리지 진단 |
| `src/preflight.py` | 현장 셋업 점검 |
| `src/session_log.py` | 세션 로그 (`logs/<run_id>/`) |
| `src/replay_server.py` | `frames.bin`을 9910으로 재생하는 가짜 VTD |
| `src/analyze_session.py` | 사후 분석 (id 안정성·신호 전이·리스폰·주기) |
| `src/plot_waypoints_svg.py` | waypoint SVG 시각화 |
| `scripts/pretest.ps1` | conda 활성화 + preflight |

`pure_pursuit.py` 설계 결정:
- `--segment`/`--segments` 필수. 쉼표로 여러 구간을 주면 이어붙여 연속 주행
- 시작·리스폰 시 최근접점 전역 탐색, 평소엔 전방 60점 창만 봐서 인덱스 역주행 방지
- lookahead = 0.9 × v, 4~10m. 조향식에는 명목값이 아니라 **실제 목표점 거리**를 쓴다
- 목표점 탐색은 **호길이로 lookahead만큼 앞선 점**부터. 경로 밖에서 수직으로
  파고들지 않고 접선 방향으로 복귀한다
- 경로 끝 근처(남은 거리 < lookahead)에서는 끝 접선 유지 조향으로 가중 전환.
  조향 포화를 막는다
- 목표속도 = `--speed`, 곡률 제한(횡가속도 2.5), 이탈 시 복귀 속도 제한,
  경로 끝 감속 중 최소값

## 남은 계획

| 시기 | 작업 |
|---|---|
| 9/5 | **xodr 1단계 완료** — planView -> reference line (`src/xodr_parser.py`) |
| 9/5 | **xodr 2단계 완료** — lane 중심선 + roadMark (`src/xodr_lanes.py`) |
| 9/5 | **xodr 3단계 완료** — 차로 연결 그래프 + 경로 탐색 (`src/xodr_graph.py`) |
| 9/6~9/8 | **xodr 4단계** — 정지선·보호구역·신호등 좌표 |
| 9/9~9/11 | **규칙 대응** — 신호등 정지 > 충돌 회피 > 보호구역 > 방향지시등 |
| 9/11 | 시나리오 시험, 코드 동결 |

규칙 대응 우선순위는 감점 크기와 발생 빈도 순이다.
사전테스트에서 적색 신호를 15회 이상 통과했으므로 **신호등 정지가 1순위**다.
