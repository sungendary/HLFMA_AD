# 2026 HL FMA 자율주행 시뮬레이션 대회 — 제어기

VTD 시뮬레이터와 TCP/UDP/RTSP로 통신하며 지정 경로를 자율주행하는 제어기를 만든다.
이 코드는 **제어기 PC**에서 실행된다. VTD PC는 주최측이 준비하므로 수정 대상이 아니다.

- 대회일: 2026년 9월 12일
- 사전테스트: 2026년 9월 3~4일
- 언어: Python 3 (Ubuntu 24.04)

## 네트워크

| 포트 | 프로토콜 | 방향 | 용도 |
|---|---|---|---|
| 9910 | TCP | 제어기 ↔ VTD (양방향, 20Hz) | 상태 수신 / 제어 송신 |
| 8554 | RTSP | 제어기 → VTD (접속) | 전방 카메라 h264 1920x1080 20fps |
| 9912 | UDP | VTD → 제어기 | LiDAR 포인트 클라우드 |

대회 당일 IP: 제어기 PC `192.168.50.10`, VTD PC `192.168.50.11`, 넷마스크 `255.255.255.0`.
연습 시에는 `127.0.0.1` 또는 로컬 VTD PC IP를 사용한다.

## 9910 송신 — 제어 명령 (9바이트, 헤더 없음)

```python
CTRL = struct.Struct('<ffB')   # 9 bytes
# steering     float32  조향각 [rad]   양수=좌회전, 음수=우회전
# targetAccel  float32  목표 가속도 [m/s^2]  음수=감속
# turnSignal   uint8    0=OFF 1=LEFT 2=RIGHT
```

- 실용 범위: steering ±0.35 rad, targetAccel -6.0 ~ +3.0 m/s^2
- 20Hz로 끊김 없이 전송. 소켓에 `TCP_NODELAY` 설정 필수.
- **연결이 끊기면 VTD 측에서 steering=0, accel=0으로 강제된다.** 재연결 로직 필수.
- - 조향 부호: 양수 = 좌회전, 음수 = 우회전. 수동 조작으로 검증 완료.

## 9910 수신 — 상태 데이터

고정 부분 1104바이트 + 신호등 배열. 정확한 프레이밍은 실측으로 확정한다.

```
Ego (24B)
  egoX, egoY, egoZ           float32  [m]
  egoHeading, egoPitch, egoRoll  float32  [rad]

Object[30] (36B x 30 = 1080B)
  id                         uint32
  x, y, z                    float32  [m]
  heading                    float32  [rad]
  speed                      float32  [m/s]
  length, width, height      float32  [m]

TrafficLight[N] (5B x N)
  id                         int32
  state                      uint8
```

신호등 state: `0`=미할당 `1`=적색 `2`=황색 `3`=녹색 `4`=좌회전 `5`=녹색+좌회전 `6`=점멸

객체 배열 규칙:
- Ego 제외, 수평거리(x,y) 80m 이내 객체를 가까운 순으로 정렬해 최대 30개
- `id == 0`인 슬롯은 무효. 배열은 항상 30개 고정 전송이며 미사용 슬롯은 전부 0
- 배열 인덱스는 매 프레임 바뀌므로 추적에는 반드시 `id`를 쓴다
- 주최측이 ID를 재매핑하므로 VTD 내부 ID와 다르다

## 알려진 공백과 대응

**egoSpeed가 명세에 없다.** 위치 차분으로 계산한다.
`v = hypot(x - x_prev, y - y_prev) / dt` (dt = 0.05s). 노이즈가 있으므로 저역통과 필터를 건다.

**객체 종류(type) 필드가 없다.** 치수로 분류한다.
보행자 ≈ 0.5 x 0.5 x 1.7m, 승용차 ≈ 4.8 x 1.9 x 1.5m.
`length < 1.0 and height < 2.0` 이면 보행자로 간주. 실측 로그로 임계값을 조정한다.

**정지선 위치가 오지 않는다.** 신호등은 `id`와 `state`만 온다.
지도는 대회 당일에도 동일하므로, 연습 주행에서 `trafficLights[].id`와 자기 위치를 함께 기록해
`data/traffic_light_map.json`에 **신호등 ID → 정지선 좌표** 매핑을 미리 구축한다.

**차선/도로 ID가 오지 않는다.** 차로 유지와 중앙선 침범은 경로 추종 정확도로만 방어한다.

## LiDAR UDP 패킷 (9912)

```python
HDR = struct.Struct('<IHIQdHHIIHBB')   # 42 bytes
# magic(0x4C485649="IVHL") version frame_id timestamp_ns sim_time
# packet_index packet_count point_count_total point_start_index
# point_count_in_pkt coordinate_mode point_stride
# 이후 point_count_in_pkt x float32(x, y, z)
```

- `coordinate_mode = 1` = **월드 좌표**. ego 좌표·경로 CSV와 동일 좌표계이므로 변환 불필요.
- 기본 프레임당 약 51200점을 512점씩 약 100패킷으로 분할, 20fps
- 대역폭 약 98Mbps → 기가비트 이더넷 필요
- UDP이므로 유실·순서 뒤바뀜을 가정하고 `frame_id` + `packet_index`로 재조립

## 경로 (route CSV)

```
seq,x,y
```

- VTD 월드 직교좌표, 단위 m. egoX/egoY와 동일 좌표계
- 첫 지점=출발, 마지막 지점=종료
- 사이 지점은 두 개씩 짝: seq 2·3 = 교차로1 진입·진출, 4·5 = 교차로2, 이후 동일
- 짝 사이 구간이 교차로 내부
- **CSV의 점만 직선으로 이으면 도로를 벗어난다.** 곡선 구간이 있으므로
  사전에 수동 주행으로 녹화한 조밀한 waypoint(`data/waypoints/`)에 스냅해 경로를 구성한다

## 평가 규칙 요약 (전략에 직결)

- 구간별 100점에서 시작. 경미 -3점, 중대 -6점. 동일 항목은 구간당 1회만 감점
- **완주 그룹은 미완주 그룹보다 항상 상위.** 총점이 낮아도 완주가 우선이다
- 완주 시간은 총점 동점일 때만 쓰는 2순위 → **속도를 무리할 이유가 없다**
- 예외: 녹색신호에서 정지선 30m 이내 이유 없이 5초 이상 정차하면 감점 (얼어붙지 말 것)
- 리스폰은 구간당 1회 무료, 이후 1회당 -6점
- 리스폰 시 위치가 강제 이동되고 **속도가 0이 된다.** 속도 급변을 감지해 상태를 초기화할 것
- 예시 경로 총 길이 약 777m, 4구간. 30km/h로도 약 95초 → 20분 안에 2회 시도 가능

## 코딩 방침

- 표준 라이브러리 우선. 의존성은 최소로 유지한다 (대회 당일 PC에서 확실히 돌아야 함)
- 제어 루프는 20Hz를 지킨다. 블로킹 연산을 루프 안에 넣지 않는다
- 예외로 루프가 죽으면 안 된다. 수신·연산 실패 시에도 마지막 유효 명령이나 안전 감속을 계속 송신
- 모든 수신 프레임을 `logs/`에 기록해 사후 분석이 가능하게 한다
- `reference/` 는 주최측 제공 원본이다. 읽기만 하고 수정하지 않는다

## 실측으로 확정된 사항 (2026-08-30)

- 수신 프레임 = **1109 B 고정**, 20.0 Hz. Ego 24B + Object 36B×30 + TrafficLight 5B
- TrafficLight는 **배열이 아니라 1개**. 내 차에 해당하는 신호등만 선별 전송됨
- 조향 부호: **양수 = 좌회전**, 음수 = 우회전 (수동 조작으로 검증)
- heading은 반시계 방향 양수. 실제 진행 방향과 0.1° 이내로 일치 (검증됨)
- egoSpeed는 전송되지 않음 → 위치 차분 + 저역통과로 추정
- LiDAR 실측: 34752 pts/frame, 68 pkt/frame, 약 7.7 fps, 25 Mbps
  (예상 20fps보다 낮음. GPU 부하로 추정)
- 교차로 회전 반경 실측 약 14 m → lookahead는 이보다 작게

## 개발 환경

- VTD PC(Ubuntu) 192.168.50.11 ↔ 제어기 PC(Windows) 192.168.50.10, 직결
- 실행 순서: 랜선 → `nmcli con up hlfma` → `lmgrd` → `vtdStart.sh`
  (순서가 바뀌면 LiDAR 전송기가 시작되지 않음)
- 제어기는 miniconda `hlfma` 환경 (Python 3.11, numpy 2.4.6)
- **matplotlib은 Windows 앱 제어 정책에 차단됨** → 시각화는 SVG 직접 생성
- 의존성을 최소로 유지할 것. 표준 라이브러리 우선
