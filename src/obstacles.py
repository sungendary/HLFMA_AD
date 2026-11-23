#!/usr/bin/env python3
"""충돌 회피 (평가항목 10 보행자 · 11 장애물 · 14 차량).

9910이 준 objects[]를 주행 경로에 투영해 매 프레임 속도 상한 하나를 낸다.
pure_pursuit은 그 값을 기존 v_cmd에 min()으로 섞기만 하면 된다.
stoplines.py와 같은 구조다 ― I/O도 소켓도 없고, hlvtd_client도 import하지
않는다(objects는 덕 타이핑). 순수 로직이라 로그만으로 재현·검증된다.

--- 사전테스트 6세션(2026-09-04) 실측이 이 파일의 상수를 정한다 ---

1. **length로는 종류가 갈리지 않는다.** 자전거 2.08m > 보행자 0.60m다.
   width와 height를 쓴다. hlvtd_client.Obj.is_pedestrian은 length를 보므로
   **거동에 쓰지 않는다** (그 속성은 objects.csv의 is_ped 컬럼 전용이다).

2. **id는 재사용된다.** 시나리오 3에서 id 5가 한 프레임에 89 m/s 이동했다.
   삭제·재생성으로 같은 id가 다른 객체에 붙은 것이다. 그래서 speed/heading을
   쓰는 예측은 **프레임 간 이동거리 검증을 통과한 track에만** 적용한다.
   통과 못 하면 보수적으로 앞차 속도를 0으로 보고, 보행자 예측 대역은 끈다.

3. **경로 기준으로 재야 한다.** 직선거리로 재면 커브에서 옆 차로 객체가
   전방으로, 전방 객체가 옆으로 찍힌다. 객체를 경로에 수직투영해
   (진행거리, 부호 횡거리)를 구한다.

4. **속도 상한만으로는 부족하다.** pure_pursuit의 가속도는 P 제어라
   sqrt 프로파일을 따라가면 정상상태 오차만큼 위로 달린다. 신호등 정지에서
   실제로 정지선을 0.8m 넘었다. _profile이 감속 피드포워드를 함께 낸다.
   stoplines._profile에 앞차 속도 항을 더한 일반화다 ― v_lead=0이면 같다.

5. **장애물(라바콘)에는 정지하지 않는다.** 차로 내 라바콘이면 속도 상한만
   저속으로 걸고 지나간다. 항목 11은 경미 -3이고 완주가 최우선이라, 교착이
   원리적으로 불가능한 쪽을 고른다. 대신 접근 중 최소 횡거리를 이벤트로
   남긴다 ― 실제로 차로 안이었는지가 OBST_LANE_M을 고칠 유일한 근거다.

6. **NPC는 끼어들기·급정지 이벤트를 낸다**고 주최측이 확인했다. 그래서
   정지 프로파일이 신호등(2.0)보다 강한 2.5다.
"""

from __future__ import annotations

import math

from stoplines import FRONT_OVERHANG, STOP_SPEED
from finish import FINISH_ZONE_M, gate_s

# 분류
OBSTACLE, PEDESTRIAN, VEHICLE = 'obstacle', 'pedestrian', 'vehicle'

LOOKAHEAD_M = 60.0          # m, 이 안의 객체만 본다 (수신 반경 80m 안쪽)
BEHIND_M = 5.0              # m, 이만큼 지난 객체는 버린다
PREFILTER_M = 20.0          # m, 직선거리 프리필터 여유. 커브에서 경로거리가
                            # 직선거리보다 길어지는 몫

DECEL_OBJ = 2.5             # m/s^2, 정지 프로파일. 신호(2.0)보다 강하게 —
                            # NPC 급정지 이벤트가 있다

# 차량 (항목 14)
MIN_GAP = 5.0               # m, 앞차 뒤범퍼와의 정지 간격
CREEP_GAP = 2.0             # m, 교착 해제 후 기어갈 때의 간격
HEADWAY_S = 1.2             # s, 차간시간. v_limit <= gap / HEADWAY_S
VEH_LANE_M = 2.0            # m, 이 안이면 같은 차로 (차로폭 3.2 기준)
VEH_BLOCK_S = 30.0          # s, 정지한 앞차는 신호 대기 중일 수 있어 길게

# 보행자 (항목 10)
PED_HARD_M = 3.0            # m, 즉시 정지 대역
PED_WATCH_M = 6.0           # m, 예측 감시 대역
PED_GAP = 4.0               # m, 보행자 앞 범퍼 정지 간격
REAR_OVERHANG_M = 1.0       # m, 후륜축 -> 뒷범퍼 (check_collision과 같은 가정).
                            # 보행자 예측에서 "횡단선을 완전히 지나는 시각"에 쓴다
# 횡단 완료 판정 (9/11). "대역을 벗어남"이 아니라 "건너기를 마침"이어야 한다.
# 실차에서 횡거리 +4.4~+5.2m에서 풀려 아직 도로 위인 보행자를 두고 출발했다
# (pp_20260911_133947 id 12/11/16). 항목 10은 중대 -6이고 저속·정차는 무감점이다
PED_CLEAR_M = 6.0           # m, 경로에서 이만큼 벗어나야 완료 후보 (정지 중 대역이기도 하다)
PED_OFFROAD_M = 2.0         # m, 이 반경 안에 driving 차로 중심점이 없으면 도로 밖.
                            # on_road 콜백(detour.LaneIndex)이 있을 때만 쓴다
PED_BEHIND_M = 2.0          # m, 후륜축 뒤로 이만큼 지나면 완료
PED_STILL_MS = 0.3          # m/s, 이 미만이면 서 있는 것 (detour.STOPPED_MS와 같게)
PED_STILL_S = 3.0           # s, 이만큼 서 있으면 횡단 의사가 없다
PED_LOST_S = 1.0            # s, 이만큼 안 보이면 시야 밖
PED_CLEAR_HOLD_S = 1.5      # s, 전원 완료 뒤 추가 대기. 멈췄다 다시 걷는 사람을 흡수
PED_CLEAR_TIMEOUT_S = 20.0  # s, 완료 조건이 영영 안 풀릴 때의 상한 (보도가 driving
                            # 차로 2m 안으로 잡히면 갇힌다). 경고를 남기고 푼다
PED_VLAT_MIN = 0.2          # m/s, 이보다 느린 횡방향 접근은 예측하지 않는다
PED_CORRIDOR_M = 1.5        # m, 우리 차체가 실제로 쓸어가는 반폭 (반폭 0.95 +
                            # 여유 0.55). 이 밖의 보행자는 교착 해제 시
                            # 서행으로 **지나갈 수 있다**. 안쪽이면 못 지나간다
MAX_BLOCK_S = 20.0          # s, 보행자 교착 방지

# 장애물 (항목 11) — 정지하지 않는다
OBST_LANE_M = 1.2           # m, 이 안이면 차로 내
OBST_CREEP = 2.8            # m/s (약 10km/h)

# 추적
ID_JUMP_M = 30.0 * 0.05     # m, 프레임 간 이 이상 움직이면 id 재사용이다
TRACK_GRACE = 3             # 프레임, 안 보여도 이만큼은 들고 있는다
TRUST_FRAMES = 2            # 연속 이만큼 봐야 speed/heading을 믿는다

BLOCKING_M = 25.0           # m, 항목 8 면책을 줄 전방 객체 거리

STOP_ACTS = ('stop', 'hold')


def classify(o) -> str:
    """치수로 종류를 가른다. 9910에는 종류 필드가 없다.

    라바콘 0.30x0.30x0.32~0.96 / 보행자 0.60x0.70x1.80 /
    자전거 2.08x0.85x1.93 / 승용차 4.39~5.18 x 1.81~1.91.
    자전거는 보행자로 묶는 게 안전하다 (더 크게 비켜 준다).
    """
    if o.height < 1.2:
        return OBSTACLE
    if o.width < 1.2:
        return PEDESTRIAN       # 자전거(w=0.85)도 여기 들어온다 — 의도한 것이다
    return VEHICLE


# ----------------------------------------------------------------------
# 기하

def _window(path, idx: int, back_m: float, fwd_m: float) -> tuple:
    """투영을 돌릴 경로 인덱스 구간 [lo, hi). hi는 선분 시작 인덱스의 상한이다.

    전역 탐색은 782점 x 30객체 x 20Hz = 47만 회/초라 20Hz 루프에서 못 쓴다.
    """
    lo = idx
    target = path.s[idx] - back_m
    while lo > 0 and path.s[lo] > target:
        lo -= 1
    hi = path._advance(idx, fwd_m)
    return lo, min(hi + 1, len(path.pts) - 1)


def _project(path, i_lo: int, i_hi: int, x: float, y: float) -> tuple:
    """(경로 누적거리, 부호 횡거리, 그 선분의 방위).

    정점이 아니라 **선분**에 수직투영한다. 부호는 진행방향 기준 좌측이 양수로
    route_check.nearest_on_route와 같은 규약이다.

    객체 직선거리로 선분을 더 쳐내는 가지치기도 재 봤지만, 선분마다 비교가
    하나 늘어 이득이 상쇄됐다 (30슬롯 최악에서 A/B 두 번 모두 유의차 없음).
    창을 좁게 잡는 것만으로 충분하다.
    """
    pts, sarr = path.pts, path.s
    best_d2 = None
    best = (sarr[i_lo], 0.0, 0.0)
    for i in range(i_lo, max(i_lo + 1, i_hi)):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        dx, dy = bx - ax, by - ay
        den = dx * dx + dy * dy
        if den < 1e-12:
            continue
        u = ((x - ax) * dx + (y - ay) * dy) / den
        u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
        qx, qy = ax + u * dx, ay + u * dy
        d2 = (x - qx) ** 2 + (y - qy) ** 2
        if best_d2 is None or d2 < best_d2:
            th = math.atan2(dy, dx)
            lat = -math.sin(th) * (x - qx) + math.cos(th) * (y - qy)
            best_d2 = d2
            best = (sarr[i] + u * (sarr[i + 1] - sarr[i]), lat, th)
    return best


def _profile(avail: float, v: float, v_lead: float = 0.0,
             decel: float = DECEL_OBJ) -> tuple:
    """avail m 안에서 v_lead까지 줄이기 위한 (속도 상한, 가속도 상한).

    stoplines._profile의 일반화다. v_lead=0이면 완전히 같은 수가 나오고,
    달리는 앞차에는 "그 속도까지만 줄이면 된다"가 된다. 둘 다 sqrt 형태라
    경로 끝 감속·신호 정지 프로파일과 min() 하나로 합쳐진다.

    가속도 상한은 P 제어의 정상상태 오차를 메우는 피드포워드다. 프로파일
    아래에 있을 때는 걸지 않는다 ― 안 그러면 멀리서부터 가속을 막는다.
    """
    if avail <= 0.3:
        v_limit = max(0.0, v_lead)
    else:
        v_limit = math.sqrt(v_lead * v_lead + 2.0 * decel * avail)
    if v <= v_limit:
        return v_limit, float('inf')
    return v_limit, -(v * v - v_lead * v_lead) / (2.0 * max(avail, 0.05))


# ----------------------------------------------------------------------

class Track:
    """id별 추적 상태. speed/heading을 믿어도 되는지가 핵심이다."""

    __slots__ = ('id', 'cls', 'x', 'y', 's', 'lat', 'th', 'speed', 'heading',
                 'n_seen', 'trusted', 'missing', 'min_lat', 'first_t',
                 'still_since')

    def __init__(self, oid, cls, x, y, t):
        self.id = oid
        self.cls = cls
        self.x, self.y = x, y
        self.s = self.lat = self.th = 0.0
        self.speed = self.heading = 0.0
        self.n_seen = 1
        self.trusted = False        # id 재사용 검증을 통과했나
        self.missing = 0
        self.min_lat = float('inf')  # 접근 중 최소 |횡거리| (부호 보존)
        self.first_t = t
        self.still_since = None     # speed < PED_STILL_MS 가 시작된 시각

    def __repr__(self) -> str:
        return (f'<Track {self.id} {self.cls} lat={self.lat:+.1f} '
                f'{"trusted" if self.trusted else "new"}>')


class _Held:
    """정지를 유발한 보행자 하나의 횡단 완료 대기 상태."""

    __slots__ = ('id', 'since', 'last_seen', 'done', 'why', 'code', 'stopped')

    def __init__(self, oid, t):
        self.id = oid
        self.since = t              # 처음 정지를 유발한 시각 (상한의 기준)
        self.last_seen = t          # 마지막으로 objects[]에 있던 시각
        self.done = False           # 횡단 완료 판정
        self.why = ''               # 완료 근거 (사람이 읽는 문장)
        self.code = ''              # lost / behind / still / lat / timeout
        # 이 보행자 때문에 실제로 멈춘 적이 있다. 그 뒤로는 해제까지 v_limit 0 --
        # TTC 등이 더 일찍 세웠으면 정지 프로파일이 PED_GAP까지 기어가게 두는데,
        # 건너는 사람 옆에서 2m를 기어가는 것은 "정차"가 아니다 (sim xw_gate)
        self.stopped = False


class ObstacleCmd:
    """한 프레임의 판정 결과. stoplines.SignalCmd와 같은 모양이다."""

    __slots__ = ('v_limit', 'accel_limit', 'action', 'kind', 'obj_id',
                 'dist', 'lat', 'reason', 'blocking', 'stop_avail',
                 'front_clear')

    def __init__(self, v_limit=float('inf'), action='none', kind='',
                 obj_id=-1, dist=float('nan'), lat=float('nan'), reason='',
                 accel_limit=float('inf'), blocking=False,
                 stop_avail=float('nan'), front_clear=float('inf')):
        self.v_limit = v_limit          # m/s. inf면 제약 없음
        self.accel_limit = accel_limit  # m/s^2 상한. inf면 제약 없음
        self.action = action            # none / follow / slow / stop / hold / creep
        self.kind = kind                # obstacle / pedestrian / vehicle
        self.obj_id = obj_id
        self.dist = dist                # 후륜축 -> 객체 경로거리 [m]
        self.lat = lat                  # 경로 기준 부호 횡거리 [m]
        self.reason = reason
        self.blocking = blocking        # 항목 8 면책 — 전방에 차량·보행자
        # 후륜축이 지금부터 몇 m 더 가서 설 작정인가. nan이면 정지 의사 없음
        # (장애물 감속처럼 서지 않는 판정, 또는 후보 없음).
        self.stop_avail = stop_avail
        # 앞범퍼에서 전방 차로 내 최근접 객체 뒷면까지의 여유 [m]. 후보가
        # 없으면 lookahead. **횡단보도 탈출(crosswalks.py)의 안전 게이트가
        # 쓰는 값이고, 이 값 없이는 탈출을 발동하지 않는다.**
        self.front_clear = front_clear

    @property
    def stopping(self) -> bool:
        return self.action in STOP_ACTS

    def __repr__(self) -> str:
        return (f'<ObstacleCmd {self.action} {self.kind} v<={self.v_limit:.1f} '
                f'd={self.dist:.1f} lat={self.lat:+.1f}>')


class ObstaclePolicy:
    """매 프레임 속도 상한 하나를 낸다.

    래치하지 않는다. 매 프레임 다시 판정하고, 히스테리시스는 "정지를 풀
    조건"에만 건다 ― 보행자가 대역 경계에서 떨릴 때 가다 서다 하는 것을 막는다.
    """

    def __init__(self, path, *, front_overhang: float = FRONT_OVERHANG,
                 lookahead: float = LOOKAHEAD_M, max_block: float = MAX_BLOCK_S,
                 finish_zone: float = FINISH_ZONE_M, on_event=None,
                 on_road=None):
        self.path = path
        self.front = front_overhang
        self.lookahead = lookahead
        self.max_block = max_block
        # 종료 구간. 종료선 위의 라바콘 2개는 장애물이 아니라 완주 표지다.
        # 이 s 이상의 obstacle 분류는 무시한다 (차량·보행자는 그대로)
        self.finish_zone = finish_zone
        self._gate_s = gate_s(path, finish_zone)
        self._on_event = on_event or (lambda kind, detail: None)
        # (x, y) -> bool. driving 차로 위인가. None이면 도로 밖 조건을 못 쓴다
        # (횡거리 PED_CLEAR_M만으로 완료). pure_pursuit이 detour.DetourPlanner
        # .on_road를 물린다 -- planner가 정책 뒤에 만들어져 생성자 인자가 아니다
        self.on_road = on_road
        self.reset()

    # -- 상태 -----------------------------------------------------------
    def reset(self) -> None:
        """리스폰 시 호출한다. 위치가 강제로 옮겨졌으므로 추적을 전부 버린다."""
        self._tracks: dict = {}
        self._announced = set()     # 이벤트 중복 억제 키
        self._hold: dict = {}       # 정지를 유발한 보행자 id -> _Held (횡단 완료 대기)
        self._all_done_t = None     # 전원 완료가 시작된 시각 (PED_CLEAR_HOLD_S 기준)
        self._hold_since = None     # 실제로 멈춘 시각
        self._creep_id = None       # 교착 해제로 크립 중인 객체 id

    def adopt(self, old) -> None:
        """우회 경로 교체. **트랙은 월드 좌표라 새 경로에서도 그대로 유효하다**
        -- s/lat/th는 update()가 매 프레임 다시 투영한다. 버리면 trusted
        (TRUST_FRAMES)를 다시 채우는 동안 보행자 예측·앞차 속도가 꺼지고
        detour의 최후 안전망 게이트(obs.obj_id)가 늦게 잡힌다 (9/11).

        **보행자 횡단 완료 대기(_hold)와 on_road도 넘긴다** -- 경로만 바뀐 것이지
        건너던 사람이 사라진 게 아니다. 횡거리 기준 판정은 새 경로에서 다시
        한다. 정지 시각·크립 id·이벤트 중복 억제는 옛 경로 기준 판정의
        산물이라 가져오지 않는다.
        """
        if old is not None:
            self._tracks = dict(old._tracks)
            self._hold = dict(old._hold)
            self._all_done_t = old._all_done_t
            if self.on_road is None:
                self.on_road = old.on_road

    def _announce(self, key, kind: str, detail: str) -> None:
        if key in self._announced:
            return
        self._announced.add(key)
        self._on_event(kind, detail)

    # -- 추적 -----------------------------------------------------------
    def _update_tracks(self, objects, t: float) -> None:
        """id별 추적을 갱신한다. 프레임 간 이동거리로 id 재사용을 걸러낸다."""
        seen = set()
        for o in objects:
            seen.add(o.id)
            tr = self._tracks.get(o.id)
            if tr is not None:
                jump = math.hypot(o.x - tr.x, o.y - tr.y)
                if jump > ID_JUMP_M:
                    # 108km/h 초과는 불가능하다. 같은 id지만 다른 객체다
                    self._announce(('reuse', o.id), 'obj_id_reuse',
                                   f'id={o.id}가 한 프레임에 {jump:.1f}m 이동 '
                                   f'(한계 {ID_JUMP_M:.2f}m) ― 다른 객체다. '
                                   f'추적을 리셋하고 속도 예측을 끈다')
                    self._hold.pop(o.id, None)
                    tr = None
            if tr is None:
                tr = Track(o.id, classify(o), o.x, o.y, t)
                self._tracks[o.id] = tr
            else:
                tr.x, tr.y = o.x, o.y
                tr.n_seen += 1
                tr.trusted = tr.n_seen >= TRUST_FRAMES
            tr.cls = classify(o)
            tr.speed, tr.heading = o.speed, o.heading
            tr.missing = 0
            # 정지 시작 시각. 횡단 완료 판정의 "3초 정지" 근거다
            if abs(o.speed) < PED_STILL_MS:
                if tr.still_since is None:
                    tr.still_since = t
            else:
                tr.still_since = None
            h = self._hold.get(o.id)
            if h is not None:
                h.last_seen = t

        for oid in [k for k, tr in self._tracks.items() if k not in seen]:
            tr = self._tracks[oid]
            tr.missing += 1
            # 횡단 완료를 기다리는 보행자의 트랙은 지우지 않는다 -- 마지막
            # 위치가 판정 근거이고 "시야 밖"은 last_seen으로 따로 잰다
            if tr.missing > TRACK_GRACE and oid not in self._hold:
                del self._tracks[oid]

    # -- 본체 -----------------------------------------------------------
    def update(self, idx: int, s_ego: float, v: float,
               ego_x: float, ego_y: float, ego_heading: float,
               objects, t: float) -> ObstacleCmd:
        self._update_tracks(objects, t)
        i_lo, i_hi = _window(self.path, idx, BEHIND_M + 2.0,
                             self.lookahead + 10.0)

        cands = []
        cand_ids = set()
        blocking = False
        ped_stopped = False
        # 앞범퍼에서 전방 차로 내 최근접 객체까지의 여유. 후보가 하나도
        # 없으면 "lookahead 안에 아무것도 없다"가 그 자체로 결론이다.
        front_clear = self.lookahead

        for o in objects:
            # 프리필터. 여기서 대부분이 걸러져 투영은 몇 개만 돈다
            if math.hypot(o.x - ego_x, o.y - ego_y) > self.lookahead + PREFILTER_M:
                continue
            s_obj, lat, th = _project(self.path, i_lo, i_hi, o.x, o.y)
            d = s_obj - s_ego
            cls = classify(o)

            tr = self._tracks.get(o.id)
            if tr is not None:
                tr.s, tr.lat, tr.th = s_obj, lat, th
                if abs(lat) < abs(tr.min_lat):
                    tr.min_lat = lat
                # 지나친 장애물의 최소 횡거리를 보고한다. 차로 안이었는지가
                # OBST_LANE_M을 고칠 유일한 근거다
                if (cls == OBSTACLE and d < -1.0
                        and ('slow', o.id) in self._announced):
                    self._announce(('cleared', o.id), 'obstacle_clear',
                                   f'id={o.id} 통과. 최소 횡거리 '
                                   f'{tr.min_lat:+.2f}m '
                                   f'({"차로 내" if abs(tr.min_lat) <= OBST_LANE_M else "차로 밖"})')

            if d < -BEHIND_M or d > self.lookahead:
                continue

            # 종료 구간의 장애물은 종료 라바콘이다. 감속도 front_clear 축소도
            # 하지 않는다. 진짜 라바콘이어도 obstacles는 피하지 않고 감속만
            # 하므로 충돌 결과는 같다 — 늦게 지나가는 시간만 아낀다
            if cls == OBSTACLE and s_obj >= self._gate_s:
                self._announce(('fin', o.id), 'finish_zone_ignore',
                               f'id={o.id} 장애물 lat={lat:+.2f}m d={d:.1f}m ― '
                               f'경로 끝 {self.finish_zone:.0f}m 안이라 무시한다 '
                               f'(종료 라바콘)')
                continue

            # 항목 8 면책 판단용. 전방 차로에 차량·보행자가 "있기만" 해도 면책이다
            if (cls in (PEDESTRIAN, VEHICLE) and abs(lat) <= VEH_LANE_M
                    and 0.0 < d <= BLOCKING_M):
                blocking = True

            # 전방 여유. 종류마다 "차로 안" 기준이 다르므로 각자의 기준을 쓴다.
            # 라바콘도 넣는다 — 들이받으면 항목 11이다
            band = OBST_LANE_M if cls == OBSTACLE else VEH_LANE_M
            if d > 0.0 and abs(lat) <= band:
                front_clear = min(front_clear,
                                  max(0.0, d - self.front - o.length * 0.5))

            if cls == VEHICLE:
                cmd = self._vehicle(o, tr, d, lat, th, v)
            elif cls == PEDESTRIAN:
                cmd = self._pedestrian(o, tr, d, lat, th, v, t)
                if cmd is not None:
                    ped_stopped = True
                    self._note_ped_cmd(o.id, tr, d, lat, t, v)
            else:
                cmd = self._obstacle(o, tr, d, lat)
            if cmd is not None:
                cands.append(cmd)
                cand_ids.add(o.id)

        self._release_pedestrians(ped_stopped, s_ego, t)
        # 횡단 완료를 기다리는 동안은 보행자가 대역 밖(또는 뒤)이라 명령이 안
        # 나와도 서 있어야 한다. 후보가 비면 v_limit=inf라 그냥 출발해 버린다
        hold = self._hold_cmd(ped_stopped, s_ego, v, t)
        if hold is not None:
            cands.append(hold)
            cand_ids.add(hold.obj_id)

        if not cands:
            self._hold_since = None
            self._creep_id = None
            return ObstacleCmd(blocking=blocking, front_clear=front_clear)

        best = min(cands, key=lambda c: (c.v_limit, c.dist))
        best.front_clear = front_clear
        # 실제로 차량·보행자 때문에 서 있다면 횡거리와 무관하게 면책이다.
        # VEH_LANE_M 밖(예: 횡거리 2.5m 보행자)에서 정지하는 경우가 있는데
        # 그때 면책이 없으면 항목 8 경고가 뜨고 MAX_HOLD_S가 돌아 버린다
        best.blocking = blocking or (best.stopping
                                     and best.kind in (PEDESTRIAN, VEHICLE))
        return self._deadlock(best, cand_ids, v, t)

    # -- 가지 -----------------------------------------------------------
    def _vehicle(self, o, tr, d, lat, th, v) -> ObstacleCmd | None:
        """항목 14. 같은 차로 앞차와 안전거리를 유지한다."""
        if d <= 0.0 or abs(lat) > VEH_LANE_M:
            return None

        # 앞차 속도는 경로 접선 성분이다. id 재사용 검증을 통과 못 했으면
        # 0으로 본다 — 서 있다고 가정하는 쪽이 보수적이다
        v_lead = 0.0
        if tr is not None and tr.trusted:
            v_lead = max(0.0, o.speed * math.cos(o.heading - th))

        gap = d - self.front - o.length * 0.5      # 앞차 뒤범퍼까지
        creeping = (self._creep_id == o.id)
        avail = gap - (CREEP_GAP if creeping else MIN_GAP)
        v_limit, accel_limit = _profile(avail, v, v_lead)
        # 차간시간 규칙. 넓은 간격에서는 아무 제약도 되지 않는다.
        #
        # **v_lead 아래로는 내리지 않는다.** 우리보다 빠른 차가 같은 차로에서
        # 추월해 나갈 때 gap이 잠깐 음수가 되는데(중심이 우리 뒤에서 앞으로
        # 넘어가는 동안), 그때 gap/HEADWAY_S를 그대로 쓰면 0이 되어 **멀어지는
        # 차 때문에 급제동한다.** 111906 t=30.1에서 실제로 그랬다 —
        # 50km/h로 추월하는 id=9에 v_limit=0을 냈다. 앞차가 우리보다 빠르면
        # 간격은 저절로 벌어지므로 그 속도까지는 내도 안전하다.
        v_limit = min(v_limit, max(v_lead, gap / HEADWAY_S))

        if creeping:
            # 교착 해제. 간격을 CREEP_GAP까지 좁히되 저속으로만 간다.
            # 속도 상한을 억지로 끌어올리지는 않는다 — 그러면 앞차를 들이받는다
            v_limit = min(v_limit, OBST_CREEP)
            action = 'creep'
            reason = f'앞차 간격 {gap:.1f}m ― 교착 해제 크립'
        elif v_limit < STOP_SPEED:
            action = 'hold' if v < STOP_SPEED else 'stop'
            self._announce((o.id, 'veh_stop'), 'veh_stop',
                           f'id={o.id} 전방 {d:.1f}m 간격 {gap:.1f}m '
                           f'앞차 {v_lead * 3.6:.0f}km/h ― 정지')
            reason = f'앞차 정지 간격 {gap:.1f}m'
        else:
            action = 'follow'
            self._announce((o.id, 'veh_follow'), 'veh_follow',
                           f'id={o.id} 전방 {d:.1f}m 간격 {gap:.1f}m '
                           f'앞차 {v_lead * 3.6:.0f}km/h ― 추종 '
                           f'상한 {v_limit * 3.6:.0f}km/h')
            reason = f'앞차 추종 간격 {gap:.1f}m'
        # 앞차가 달리고 있어도 그 자리를 정지 후보로 본다. 앞차가 서면
        # 우리가 서는 곳이라, 그 자리가 횡단보도 위인지 미리 봐야 한다.
        return ObstacleCmd(v_limit, action, VEHICLE, o.id, d, lat, reason,
                           accel_limit, stop_avail=avail)

    def _pedestrian(self, o, tr, d, lat, th, v, t) -> ObstacleCmd | None:
        """항목 10 (중대 -6). 정적 3m 대역만으로는 제동이 늦다.

        45km/h(12.5 m/s)에서 DECEL_OBJ 정지에 31m가 필요한데 3.6 m/s 보행자는
        3m 대역을 0.83초(우리는 10m 진행)에 지난다. 그래서 3~6m 대역에서는
        도달 시각을 예측해 미리 선다.

        **이미 이 보행자 때문에 서 있고 아직 횡단 완료가 아니면 대역이
        PED_CLEAR_M(6m)이다.** 완료 판정이 난 보행자는 원래 대역(3m)과 예측
        분기로만 다시 잡는다 -- 되돌아오면 재발동한다.
        """
        if d <= 0.0:
            return None
        alat = abs(lat)
        h = self._hold.get(o.id)
        band = PED_CLEAR_M if (h is not None and not h.done) else PED_HARD_M

        if alat <= band:
            ev, why = 'ped_stop_actual', f'횡거리 {alat:.1f}m ― 대역 안'
        elif (alat <= PED_WATCH_M and tr is not None and tr.trusted):
            # 경로 쪽으로 다가오는 횡방향 속도 성분 (양수 = 접근)
            vlat = -math.copysign(1.0, lat) * o.speed * math.sin(o.heading - th)
            if vlat < PED_VLAT_MIN:
                return None
            t_ped = (alat - PED_HARD_M) / vlat
            # 우리가 그 사람의 횡단선을 **차체 끝까지 완전히 지나는** 시각.
            # 예전엔 "정지점(범퍼 4m 앞)에 닿는 시각"이라 서 있는 차(v≈0)가
            # 정지점에 있으면 t_ego=0이 되어 되돌아오는 보행자를 대역(3m)에
            # 들어올 때까지 무시했다 -- 완료 대기 뒤 출발 직후 다시 서는 톱니
            t_ego = max(0.0, d + REAR_OVERHANG_M) / max(v, 0.1)
            if t_ped >= t_ego:
                return None
            ev = 'ped_stop_predicted'
            why = (f'예측 ― 횡거리 {alat:.1f}m 접근 {vlat:.1f}m/s, '
                   f'{t_ped:.1f}s 후 대역 진입 (우리 도착 {t_ego:.1f}s)')
        else:
            return None

        self._announce((o.id, ev), ev,
                       f'id={o.id} 전방 {d:.1f}m {why} ― 정지')
        if self._creep_id == o.id:
            # 교착 해제. 여기서 갈린다 — **차폭 밖이면 지나갈 수 있다.**
            # 20초를 기다려도 안 비키는 "보행자"는 건너는 중이 아니다.
            # 옆으로 3m 떨어져 서 있는 사람 앞에서 영영 멈춰 있으면 미완주다.
            # sim에서 실제로 그랬다 — lat=-2.96m 정지 보행자에 막혀 s=51m에서
            # 400초를 못 움직였다. 반대로 차폭 안이면 절대 지나가지 않는다.
            if abs(lat) > PED_CORRIDOR_M:
                v_limit, accel_limit = OBST_CREEP, float('inf')
                avail = float('nan')        # 서지 않고 지나간다
                why += f' (교착 해제 ― 차폭({PED_CORRIDOR_M:.1f}m) 밖이라 서행 통과)'
            else:
                avail = d - self.front - CREEP_GAP
                v_limit, accel_limit = _profile(avail, v)
                v_limit = min(v_limit, OBST_CREEP)
                why += ' (교착 해제 ― 차폭 안이라 간격까지만)'
            return ObstacleCmd(v_limit, 'creep', PEDESTRIAN, o.id, d, lat, why,
                               accel_limit, stop_avail=avail)

        avail = d - self.front - PED_GAP
        if h is not None and h.stopped:
            avail = 0.0             # 한 번 섰으면 해제까지 기어가지 않는다
        v_limit, accel_limit = _profile(avail, v)
        action = 'hold' if (v < STOP_SPEED and v_limit < STOP_SPEED) else 'stop'
        return ObstacleCmd(v_limit, action, PEDESTRIAN, o.id, d, lat, why,
                           accel_limit, stop_avail=avail)

    def _obstacle(self, o, tr, d, lat) -> ObstacleCmd | None:
        """항목 11 (경미 -3). **정지하지 않는다.**

        차로 내 라바콘이면 속도 상한만 저속으로 건다. 교착이 원리적으로
        불가능하고 최악이 -3점이라 완주 최우선 원칙에 맞는다.
        """
        if d <= 0.0 or abs(lat) > OBST_LANE_M:
            return None
        self._announce(('slow', o.id), 'obstacle_slow',
                       f'id={o.id} 전방 {d:.1f}m 횡거리 {lat:+.2f}m '
                       f'(h={o.height:.2f} w={o.width:.2f}) ― '
                       f'감속만 한다 (정지하지 않음)')
        # obstacles는 서지 않지만 **TTC가 이 라바콘 MIN_GAP 앞에 세운다.** 그
        # 자리를 예측 정지점으로 실어 crosswalks가 횡단보도 위 착지를 미리
        # 막게 한다 (실차 9/11 t=277: 라바콘 앞 TTC 정지가 횡단보도 4m 안이었다).
        # TTC warn은 t_cpa 4s라 서행 중에는 구간 진입 뒤에야 나온다
        return ObstacleCmd(OBST_CREEP, 'slow', OBSTACLE, o.id, d, lat,
                           f'차로 내 장애물 {abs(lat):.2f}m ― 감속만 '
                           f'(TTC 정지점 {d - self.front - MIN_GAP:.1f}m)',
                           stop_avail=d - self.front - MIN_GAP)

    # -- 횡단 완료 대기 -------------------------------------------------
    def _note_ped_cmd(self, oid: int, tr, d: float, lat: float, t: float,
                      v: float) -> None:
        """이번 프레임 이 보행자가 정지 명령을 냈다. 대기 목록에 넣거나,
        완료로 봤던 사람이 움직이며 다시 잡혔으면 완료를 취소한다."""
        h = self._hold.get(oid)
        if h is None:
            h = self._hold[oid] = _Held(oid, t)
        h.last_seen = t
        if v < STOP_SPEED:
            h.stopped = True
        # 상한으로 완료한 사람은 되돌리지 않는다 -- 되돌리면 다음 프레임에 또
        # 상한에 걸려 매 프레임 완료/취소가 반복된다
        if (h.done and h.code != 'timeout'
                and (tr is None or tr.still_since is None)):
            h.done, h.code, h.why = False, '', ''
            self._on_event('ped_resume',
                           f'id={oid} 전방 {d:.1f}m 횡거리 {lat:+.1f}m ― 완료로 '
                           f'봤던 보행자가 다시 접근한다. 횡단 완료 판정을 되돌린다')

    def _ped_done_reason(self, h: _Held, s_ego: float, t: float):
        """(code, why) 또는 None. 위에서부터 첫 일치."""
        tr = self._tracks.get(h.id)
        if tr is None or t - h.last_seen >= PED_LOST_S:
            return 'lost', f'시야 밖 {t - h.last_seen:.1f}s'
        d = tr.s - s_ego
        alat = abs(tr.lat)
        if d < -PED_BEHIND_M:
            return 'behind', f'후륜축 뒤 {-d:.1f}m 지나감'
        if alat > PED_HARD_M:
            # 대역 안(3m)에서는 "완료"가 의미가 없다 -- 어차피 정지 명령이 선다
            if tr.still_since is not None and t - tr.still_since >= PED_STILL_S:
                return 'still', (f'{t - tr.still_since:.1f}s 정지 (횡거리 '
                                 f'{tr.lat:+.1f}m) ― 횡단 의사 없음')
            if alat >= PED_CLEAR_M:
                if self.on_road is None:
                    return 'lat', f'경로에서 {alat:.1f}m (도로 여부 불명)'
                if not self.on_road(tr.x, tr.y):
                    return 'lat', f'경로에서 {alat:.1f}m, 도로 밖'
        if t - h.since >= PED_CLEAR_TIMEOUT_S:
            road = ('도로 위' if (self.on_road is not None
                                  and self.on_road(tr.x, tr.y)) else '도로 여부 불명')
            return 'timeout', (f'대기 상한 {PED_CLEAR_TIMEOUT_S:.0f}s (횡거리 '
                               f'{tr.lat:+.1f}m, {road}) ― 경고하고 푼다')
        return None

    def _release_pedestrians(self, ped_stopped: bool, s_ego: float,
                             t: float) -> None:
        """횡단이 끝났나. 보행자마다 완료 조건(시야 밖 / 뒤로 지나감 / 3초 정지 /
        6m 밖 + 도로 밖 / 상한)을 보고, **전원 완료 뒤 PED_CLEAR_HOLD_S를 더
        기다린 뒤** 푼다. 어느 조건으로 풀렸는지 이벤트에 남긴다."""
        if not self._hold:
            self._all_done_t = None
            return
        for h in self._hold.values():
            if h.done:
                continue
            r = self._ped_done_reason(h, s_ego, t)
            if r is None:
                continue
            h.done, (h.code, h.why) = True, r
            tr = self._tracks.get(h.id)
            where = (f'전방 {tr.s - s_ego:.1f}m 횡거리 {tr.lat:+.1f}m'
                     if tr is not None else '위치 불명')
            self._on_event('ped_done', f'id={h.id} {where} ― 횡단 완료: {h.why}')
        if ped_stopped or any(not h.done for h in self._hold.values()):
            self._all_done_t = None
            return
        # 전원 완료. 뒤로 지나간 사람뿐이면 기다릴 이유가 없다
        if not all(h.code == 'behind' for h in self._hold.values()):
            if self._all_done_t is None:
                self._all_done_t = t
            if t - self._all_done_t < PED_CLEAR_HOLD_S:
                return
        parts = [f'{h.id}({h.why})' for h in sorted(self._hold.values(),
                                                      key=lambda x: x.id)]
        timed = [h for h in self._hold.values() if h.code == 'timeout']
        if timed:
            self._on_event('ped_clear_timeout',
                           f'보행자 {", ".join(str(h.id) for h in timed)} 횡단 완료 '
                           f'조건이 {PED_CLEAR_TIMEOUT_S:.0f}s 안에 안 풀렸다 ― '
                           f'교착 방지로 해제한다 (도로 밖 판정을 의심할 것)')
        waited = 0.0 if self._all_done_t is None else t - self._all_done_t
        self._on_event('ped_clear',
                       f'보행자 {", ".join(parts)} ― 전원 횡단 완료, '
                       f'{waited:.1f}초 더 기다린 뒤 재출발')
        self._hold.clear()
        self._all_done_t = None
        self._hold_since = None
        self._creep_id = None

    def _hold_cmd(self, ped_stopped: bool, s_ego: float, v: float,
                  t: float) -> ObstacleCmd | None:
        """대기 중인데 이번 프레임 보행자 명령이 없으면 정지 명령을 합성한다.

        대상은 미완료 보행자 중 가장 가까운 것, 없으면(전원 완료 후 추가 대기)
        아무나. 크립 중인 id는 뺀다 -- 교착 해제된 크립을 되살리면 안 된다.
        **보행자 명령(크립 포함)이 하나라도 있으면 합성하지 않는다** -- 크립
        명령 옆에 정지 명령을 놓으면 min()이 정지를 골라 교착 해제가 죽는다.
        실제 정지 PEDESTRIAN 명령이므로 blocking(stoplines 동결), _deadlock
        (ped_timeout 크립), detour 최후 안전망의 obj_id 게이트가 그대로 산다.
        """
        if not self._hold or ped_stopped:
            return None
        pool = [h for h in self._hold.values() if h.id != self._creep_id]
        if not pool:
            return None
        pending = [h for h in pool if not h.done]

        def dist_of(h):
            tr = self._tracks.get(h.id)
            return tr.s - s_ego if tr is not None else float('inf')

        h = min(pending or pool, key=dist_of)
        tr = self._tracks.get(h.id)
        d = tr.s - s_ego if tr is not None else float('nan')
        lat = tr.lat if tr is not None else float('nan')
        avail = max(0.0, d - self.front - PED_GAP) if d == d else 0.0
        if h.stopped:
            avail = 0.0             # 한 번 섰으면 해제까지 기어가지 않는다
        v_limit, accel_limit = _profile(avail, v)
        action = 'hold' if (v < STOP_SPEED and v_limit < STOP_SPEED) else 'stop'
        if pending:
            why = (f'보행자 id={h.id} 횡단 대기 (대역 밖 {lat:+.1f}m, '
                   f'아직 완료 아님)')
            self._announce((h.id, 'hold'), 'ped_hold',
                           f'id={h.id} 전방 {d:.1f}m 횡거리 {lat:+.1f}m ― 대역 '
                           f'밖이지만 횡단 완료가 아니라 계속 선다')
        else:
            why = f'보행자 전원 완료 ― {PED_CLEAR_HOLD_S:.1f}초 추가 대기'
        return ObstacleCmd(v_limit, action, PEDESTRIAN, h.id, d, lat, why,
                           accel_limit, stop_avail=avail)

    def _deadlock(self, cmd: ObstacleCmd, cand_ids: set,
                  v: float, t: float) -> ObstacleCmd:
        """교착 방지. 완주가 최우선이라 감점을 먹더라도 움직인다.

        크립으로 전환하되 완전히 풀지는 않는다 ― 간격을 CREEP_GAP까지만
        좁히므로 정말 막혀 있으면 2m 앞에 서지, 들이받지는 않는다.

        **해제 조건은 "그 객체가 시야에서 사라졌을 때"다.** "명령이 정지가
        아니게 됐을 때"로 잡으면 안 된다 ― 크립이 간격을 완화해 스스로 정지가
        아니게 만들기 때문에 한 프레임 만에 되돌아간다. sim에서 실제로 그랬다:
        30초마다 크립 한 프레임 -> 해제 -> 다시 30초 대기를 반복하며
        s=442.8m에서 400초 내내 못 움직였다.
        """
        if self._creep_id is not None:
            if self._creep_id in cand_ids:
                return cmd          # 크립 간격은 _vehicle/_pedestrian이 이미 썼다
            self._creep_id = None   # 막던 객체가 사라졌다
            self._hold_since = None

        if not cmd.stopping:
            self._hold_since = None
            return cmd
        if v >= STOP_SPEED:
            self._hold_since = None
            return cmd

        limit = VEH_BLOCK_S if cmd.kind == VEHICLE else self.max_block
        if self._hold_since is None:
            self._hold_since = t
            return cmd
        if t - self._hold_since < limit:
            return cmd

        self._creep_id = cmd.obj_id
        passable = (cmd.kind == PEDESTRIAN and abs(cmd.lat) > PED_CORRIDOR_M)
        self._on_event(f'{"veh" if cmd.kind == VEHICLE else "ped"}_timeout',
                       f'id={cmd.obj_id} {cmd.kind} 횡거리 {cmd.lat:+.2f}m 앞에서 '
                       f'{limit:.0f}초 정지 ― 교착으로 보고 '
                       f'{OBST_CREEP * 3.6:.0f}km/h로 '
                       + ('서행 통과한다 (차폭 밖)' if passable
                          else f'간격 {CREEP_GAP:.1f}m까지 기어간다 (차폭 안)'))
        return cmd                  # 크립은 다음 프레임(50ms 뒤)부터 적용된다


def summary(policy: ObstaclePolicy) -> list:
    """시작 시 콘솔에 뿌릴 요약 줄."""
    return [
        f'  분류: height<1.2 장애물 / width<1.2 보행자 / 나머지 차량',
        f'  차량  차로폭 ±{VEH_LANE_M:.1f}m  최소간격 {MIN_GAP:.1f}m  '
        f'차간 {HEADWAY_S:.1f}s  교착 {VEH_BLOCK_S:.0f}s',
        f'  보행자 정지 ±{PED_HARD_M:.1f}m  예측 ±{PED_WATCH_M:.1f}m  '
        f'교착 {policy.max_block:.0f}s (차폭 ±{PED_CORRIDOR_M:.1f}m 밖이면 서행 통과)',
        f'  횡단 완료: 경로 {PED_CLEAR_M:.0f}m 밖'
        + ('+도로 밖' if policy.on_road is not None else '(도로 판정 없음)')
        + f' / 뒤 {PED_BEHIND_M:.0f}m / 정지 {PED_STILL_S:.0f}s / 시야 밖 '
          f'{PED_LOST_S:.0f}s, 이후 {PED_CLEAR_HOLD_S:.1f}s 대기, 상한 '
          f'{PED_CLEAR_TIMEOUT_S:.0f}s',
        f'  장애물 ±{OBST_LANE_M:.1f}m 이내면 {OBST_CREEP * 3.6:.0f}km/h '
        f'감속만 (정지 안 함). 경로 끝 {policy.finish_zone:.0f}m 안은 '
        f'종료 라바콘으로 보고 무시',
        f'  전방 {policy.lookahead:.0f}m  범퍼 보정 {policy.front:.3f}m',
    ]


# ----------------------------------------------------------------------
# 오프라인 재생 — sim 없이 로그만으로 반응을 본다

class _Obj:
    """objects.csv 한 행. hlvtd_client.Obj와 같은 속성만 갖는다."""
    __slots__ = ('id', 'x', 'y', 'z', 'heading', 'speed',
                 'length', 'width', 'height')

    def __init__(self, r):
        self.id = int(float(r['id']))
        self.x, self.y, self.z = float(r['x']), float(r['y']), float(r['z'])
        self.heading, self.speed = float(r['heading']), float(r['speed'])
        self.length = float(r['length'])
        self.width = float(r['width'])
        self.height = float(r['height'])


def _replay(session: str, waypoints: str, segments: list) -> None:
    import csv
    import io
    import os

    from pure_pursuit import load_path

    path, _bounds = load_path(waypoints, segments)

    by_n: dict = {}
    with io.open(os.path.join(session, 'objects.csv'), newline='',
                 encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            try:
                by_n.setdefault(int(r['n']), []).append(_Obj(r))
            except (ValueError, TypeError, KeyError):
                continue

    def show(kind: str, detail: str) -> None:
        print(f'  [{kind}] {detail}')

    policy = ObstaclePolicy(path, on_event=show)
    print(f'[obs] 경로 {len(path.pts)}점 {path.length:.1f}m')
    for line in summary(policy):
        print('[obs]' + line)
    print(f'[obs] {os.path.basename(os.path.normpath(session))} 재생\n')

    idx = None
    n_react = 0
    prev_act = 'none'
    with io.open(os.path.join(session, 'drive.csv'), newline='',
                 encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            try:
                n = int(r['n'])
                t, v = float(r['t']), float(r['speed'])
                x, y, h = float(r['ego_x']), float(r['ego_y']), float(r['heading'])
            except (ValueError, TypeError, KeyError):
                continue
            idx = path.nearest(x, y) if idx is None else path.nearest(x, y, start=idx)
            cmd = policy.update(idx, path.s[idx], v, x, y, h,
                                by_n.get(n, ()), t)
            if cmd.action != 'none':
                n_react += 1
            if cmd.action != prev_act:
                print(f'  t={t:6.1f}s v={v * 3.6:5.1f}km/h  '
                      f'{prev_act:>6s} -> {cmd.action:<6s} '
                      f'{cmd.kind:<10s} id={cmd.obj_id:<4d} '
                      f'd={cmd.dist:6.1f}m lat={cmd.lat:+5.1f}m '
                      f'v<={cmd.v_limit * 3.6:5.1f}km/h  {cmd.reason}')
                prev_act = cmd.action

    print(f'\n[obs] 반응 프레임 {n_react}개')


# ----------------------------------------------------------------------
# 자기검사 — 사전테스트 로그에 없는 상황을 합성해 확인한다
#
# 특히 **차로 내 라바콘은 6개 세션 어디에도 없다** (112119의 라바콘은 경로에서
# 4.9m 떨어져 있다). 로그로는 항목 11 분기를 한 번도 못 밟으므로 여기서 만든다.

class _Straight:
    """자기검사용 직선 경로. pure_pursuit.Path가 쓰는 것만 흉내낸다."""

    def __init__(self, length=300.0, step=1.0):
        n = int(length / step) + 1
        self.pts = [(i * step, 0.0) for i in range(n)]
        self.s = [i * step for i in range(n)]
        self.length = self.s[-1]

    def _advance(self, i, dist_m):
        target = self.s[i] + dist_m
        j = i
        while j < len(self.pts) - 1 and self.s[j] < target:
            j += 1
        return j


class _Fake:
    __slots__ = ('id', 'x', 'y', 'z', 'heading', 'speed',
                 'length', 'width', 'height')

    def __init__(self, oid, x, lat, speed=0.0, heading=0.0,
                 dims=(4.39, 1.81, 1.35)):
        self.id, self.x, self.y, self.z = oid, x, lat, 0.0
        self.heading, self.speed = heading, speed
        self.length, self.width, self.height = dims

CONE = (0.30, 0.30, 0.32)
PED = (0.60, 0.70, 1.80)
CAR = (4.39, 1.81, 1.35)


def _drive(policy, path, objs_at, v0=12.5, dt=0.05, n=400, decel=DECEL_OBJ):
    """정책을 물려 아주 단순한 종방향 적분을 돌린다. (기록, 최종 x)."""
    x, v, idx = 0.0, v0, 0
    log = []
    for k in range(n):
        t = k * dt
        while idx < len(path.pts) - 1 and path.s[idx + 1] <= x:
            idx += 1
        cmd = policy.update(idx, x, v, x, 0.0, 0.0, objs_at(t, x), t)
        v_cmd = min(v0, cmd.v_limit)
        a = max(-6.0, min(3.0, min(1.2 * (v_cmd - v), cmd.accel_limit)))
        if v_cmd <= 0.0:
            a = min(a, -1.5)
        x += v * dt
        v = max(0.0, v + a * dt)
        log.append((t, x, v, cmd.action, cmd.kind, cmd.obj_id, cmd.v_limit))
    return log, x


def _selftest() -> int:
    fails = []

    def check(name, ok, detail=''):
        print(f'  {"PASS" if ok else "FAIL"}  {name}' + (f'  ― {detail}' if detail else ''))
        if not ok:
            fails.append(name)

    # 1. 차로 내 라바콘 — 절대 정지하지 않고 저속으로 지나간다 (항목 11)
    path = _Straight()
    ev = []
    pol = ObstaclePolicy(path, on_event=lambda k, d: ev.append(k))
    cone = _Fake(1, 60.0, 0.4, dims=CONE)
    log, end = _drive(pol, path, lambda t, x: [cone], n=600)
    acts = {r[3] for r in log}
    check('라바콘: 정지 지령 없음', not (acts & {'stop', 'hold'}), f'관측 동작 {sorted(acts)}')
    check('라바콘: 지나간다', end > 80.0, f'{end:.1f}m 진행')
    check('라바콘: obstacle_slow 발화', 'obstacle_slow' in ev, ','.join(ev))
    slow = [r for r in log if r[3] == 'slow']
    check('라바콘: 상한이 크립 속도', all(abs(r[6] - OBST_CREEP) < 1e-6 for r in slow),
          f'{len(slow)}프레임')

    # 2. 차로 밖 라바콘 — 무시한다
    pol = ObstaclePolicy(_Straight())
    off = _Fake(1, 60.0, 2.5, dims=CONE)
    log, _ = _drive(pol, path, lambda t, x: [off], n=300)
    check('차로 밖 라바콘: 무반응', {r[3] for r in log} == {'none'})

    # 3. 정지한 앞차 — MIN_GAP 앞에 선다 (항목 14).
    #    교착 타임아웃(VEH_BLOCK_S) 전후를 따로 잰다. 타임아웃 뒤에는
    #    CREEP_GAP까지 좁히는 것이 설계된 거동이라 한 번에 재면 안 된다
    pol = ObstaclePolicy(_Straight())
    car = _Fake(2, 120.0, 0.0, dims=CAR)
    car_tail = car.x - car.length * 0.5
    log, end = _drive(pol, path, lambda t, x: [car], n=1400)
    halted = next((r for r in log if r[2] < STOP_SPEED), None)
    check('정지 앞차: 뒤에 선다', halted is not None)
    if halted is not None:
        gap0 = car_tail - (halted[1] + FRONT_OVERHANG)
        check('정지 앞차: 첫 정지 간격이 MIN_GAP',
              abs(gap0 - MIN_GAP) <= 0.5, f'간격 {gap0:.2f}m (목표 {MIN_GAP:.1f}m)')
    gap1 = car_tail - (end + FRONT_OVERHANG)
    check('정지 앞차: 교착 해제 뒤에도 CREEP_GAP은 지킨다',
          gap1 >= CREEP_GAP - 0.5, f'간격 {gap1:.2f}m (하한 {CREEP_GAP:.1f}m)')

    # 4. 우리보다 빠른 차가 같은 차로에서 추월 — 급제동하면 안 된다
    #    (111906 t=30.1 실측 상황. gap이 잠깐 음수가 된다)
    pol = ObstaclePolicy(_Straight())
    def overtake(t, x):
        return [_Fake(9, x - 3.0 + 13.9 * t, 0.2, speed=13.9, dims=CAR)]
    log, _ = _drive(pol, path, overtake, n=200)
    check('추월 차량: 정지 지령 없음',
          not ({r[3] for r in log} & {'stop', 'hold'}), f'{sorted({r[3] for r in log})}')
    check('추월 차량: 상한이 앞차 속도 이상',
          all(r[6] >= 13.8 for r in log if r[4] == VEHICLE))

    # 5. 횡단하는 보행자 — 대역에 들어오기 전에 예측으로 서고 (항목 10),
    #    **경로에서 PED_CLEAR_M 밖으로 나갈 때까지 서 있다가 1.5초 더 기다린 뒤**
    #    출발한다. 실차에서 +4.4~5.2m에서 풀려 도로 위 보행자를 두고 출발했다
    def crossing(t, x):
        return [_Fake(3, 70.0, -6.0 + 2.5 * t, speed=2.5,
                      heading=math.pi / 2, dims=PED)]

    def free_after(log):
        """(첫 정지 t, 그 뒤 처음으로 제약이 풀린 t)."""
        t0 = next((r[0] for r in log if r[3] in ('stop', 'hold')), None)
        if t0 is None:
            return None, None
        t1 = next((r[0] for r in log if r[0] > t0 and r[3] == 'none'), None)
        return t0, t1

    evd = []
    pol = ObstaclePolicy(_Straight(), on_event=lambda k, d: evd.append((k, d)))
    log, _ = _drive(pol, path, crossing, n=240)
    ev = [k for k, _d in evd]
    check('횡단 보행자: 예측 정지 발화', 'ped_stop_predicted' in ev, ','.join(ev))
    t0, t1 = free_after(log)
    check('횡단 보행자: 대역 진입 전에 반응', t0 is not None and t0 < 1.2,
          f'첫 정지 t={t0}')
    # 보행자가 lat +6.0에 닿는 t=4.8 + 1.5초 대기
    check('횡단 보행자: 6m 밖으로 나가고 1.5초 뒤에야 풀린다',
          t1 is not None and 6.2 <= t1 <= 7.0, f'해제 t={t1}')
    done = [d for k, d in evd if k == 'ped_done']
    clear = [d for k, d in evd if k == 'ped_clear']
    check('횡단 보행자: 해제 근거가 이벤트에 남는다',
          len(done) == 1 and '경로에서' in done[0] and len(clear) == 1
          and '더 기다린' in clear[0], f'{done} {clear}')

    # 5b. 넓은 도로 — 도로 밖(on_road)까지 기다린다. 인덱스가 없으면 6m만
    for road, lo, hi, why in ((lambda x, y: abs(y) < 8.0, 7.0, 7.8,
                               '3차로: lat 8m(도로 밖)까지'),
                              (None, 6.2, 7.0, '인덱스 없음: 6m')):
        pol = ObstaclePolicy(_Straight(), on_road=road)
        log, _ = _drive(pol, path, crossing, n=240)
        t0, t1 = free_after(log)
        check(f'횡단 완료 ({why})', t1 is not None and lo <= t1 <= hi,
              f'해제 t={t1}')

    # 5c. 보도로 물러나 서 있는 보행자 — 3초 정지면 횡단 의사 없음, 20초를 안 기다린다
    def to_sidewalk(t, x):
        u = min(t, 3.0)
        return [_Fake(4, 60.0, -2.0 - u, speed=(1.0 if t < 3.0 else 0.0),
                      heading=-math.pi / 2, dims=PED)]
    evd = []
    pol = ObstaclePolicy(_Straight(), on_road=lambda x, y: True,
                         on_event=lambda k, d: evd.append((k, d)))
    log, end = _drive(pol, path, to_sidewalk, n=240)
    t0, t1 = free_after(log)
    done = [d for k, d in evd if k == 'ped_done']
    check('보도 정지 보행자: 3초 정지 + 1.5초 뒤 출발 (t≈7.5)',
          t1 is not None and 7.4 <= t1 <= 8.2 and done and '정지' in done[0],
          f'해제 t={t1} {done}')
    check('보도 정지 보행자: 교착 타임아웃 없이 풀린다',
          'ped_timeout' not in [k for k, _d in evd] and end > 70.0,
          f'{end:.1f}m')

    # 5d. 우리 쪽으로 걸어와 옆을 지나간 보행자 — 뒤로 지나가면 바로 출발
    def walker(t, x):
        return [_Fake(6, 60.0 - 1.5 * t, 2.5, speed=1.5, heading=math.pi,
                      dims=PED)]
    evd = []
    pol = ObstaclePolicy(_Straight(), on_road=lambda x, y: True,
                         on_event=lambda k, d: evd.append((k, d)))
    log, end = _drive(pol, path, walker, n=600)
    done = [d for k, d in evd if k == 'ped_done']
    clear = [d for k, d in evd if k == 'ped_clear']
    check('지나간 보행자: 뒤로 지나가면 대기 없이 출발',
          done and '뒤' in done[0] and clear and '0.0초 더' in clear[0]
          and end > 100.0, f'{done} {clear} {end:.1f}m')

    # 5e~5f. 도로 위에서 6m 밖으로 걸어가는 보행자 — 명령이 안 나와도 정지를
    #        합성해 서 있고, 상한(PED_CLEAR_TIMEOUT_S)에 걸리면 경고하고 푼다
    global PED_CLEAR_TIMEOUT_S
    saved = PED_CLEAR_TIMEOUT_S
    PED_CLEAR_TIMEOUT_S = 12.0
    try:
        def wide(t, x):
            return [_Fake(7, 60.0, -2.0 + 1.0 * t, speed=1.0,
                          heading=math.pi / 2, dims=PED)]
        evd = []
        pol = ObstaclePolicy(_Straight(), on_road=lambda x, y: True,
                             on_event=lambda k, d: evd.append((k, d)))
        log, end = _drive(pol, path, wide, n=400)
        ev = [k for k, _d in evd]
        held = [r for r in log if 8.5 <= r[0] <= 11.5]
        check('넓은 도로 보행자: 6m 밖에서도 정지 명령을 합성한다 (hold)',
              held and all(r[3] in ('stop', 'hold') and r[4] == PEDESTRIAN
                           and r[5] == 7 for r in held) and 'ped_hold' in ev,
              f'{sorted({r[3] for r in held})} {",".join(ev)}')
        t0, t1 = free_after(log)
        check('넓은 도로 보행자: 상한에 걸리면 경고하고 푼다',
              'ped_clear_timeout' in ev and t1 is not None
              and 13.4 <= t1 <= 14.2 and end > 70.0,
              f'해제 t={t1} {end:.1f}m')
        check('넓은 도로 보행자: 완료/취소가 프레임마다 반복되지 않는다',
              ev.count('ped_done') == 1 and ev.count('ped_resume') == 0,
              f'ped_done {ev.count("ped_done")} ped_resume {ev.count("ped_resume")}')
    finally:
        PED_CLEAR_TIMEOUT_S = saved

    # 5g. 완료로 봤던 보행자가 되돌아오면 완료를 취소하고 계속 선다
    def back(t, x):
        if t < 3.0:
            return [_Fake(8, 60.0, -2.0 + (6.5 / 3.0) * t, speed=6.5 / 3.0,
                          heading=math.pi / 2, dims=PED)]
        if t < 7.0:
            return [_Fake(8, 60.0, 4.5, speed=0.0, heading=math.pi / 2, dims=PED)]
        return [_Fake(8, 60.0, 4.5 - 1.0 * (t - 7.0), speed=1.0,
                      heading=-math.pi / 2, dims=PED)]
    evd = []
    pol = ObstaclePolicy(_Straight(), on_road=lambda x, y: True,
                         on_event=lambda k, d: evd.append((k, d)))
    log, _ = _drive(pol, path, back, n=200)
    ev = [k for k, _d in evd]
    moving_free = [r for r in log if 7.5 <= r[0] <= 9.0 and r[3] == 'none']
    check('되돌아오는 보행자: ped_resume 로 완료를 취소하고 서 있는다',
          'ped_done' in ev and 'ped_resume' in ev and not moving_free,
          f'{",".join(ev)} 풀린 프레임 {len(moving_free)}')

    # 5h. 다른 이유(앞차)로 보행자보다 훨씬 앞에서 섰다가 앞차가 사라져도, 건너는
    #     사람 쪽으로 PED_GAP까지 기어가지 않는다 (sim xw_gate: TTC가 먼저 세운 뒤
    #     정지 프로파일이 2m를 2m/s로 기어갔다)
    def early_stop(t, x):
        objs = [_Fake(9, 70.0, -2.0 + 0.5 * t, speed=0.5, heading=math.pi / 2,
                      dims=PED)]
        if t < 12.0:
            objs.append(_Fake(10, 60.0, 0.0, dims=CAR))
        return objs
    evd = []
    pol = ObstaclePolicy(_Straight(), on_event=lambda k, d: evd.append((k, d)))
    log, end = _drive(pol, path, early_stop, n=440)
    t_stop = next((r[0] for r in log if r[2] < STOP_SPEED), None)
    t0, t1 = free_after(log)
    crept = [r for r in log if t_stop is not None and t_stop < r[0] < (t1 or 99.0)
             and r[2] > STOP_SPEED]
    check('일찍 선 뒤 보행자 쪽으로 기어가지 않는다',
          t_stop is not None and t_stop < 12.0 and not crept and t1 is not None
          and t1 > 16.0 and end > 60.0,
          f'정지 t={t_stop} 해제 t={t1} 기어간 프레임 {len(crept)} {end:.1f}m')

    # 6. 교착 해제 — 차폭 밖 정지 보행자는 결국 서행 통과한다
    ev = []
    pol = ObstaclePolicy(_Straight(), max_block=3.0,
                         on_event=lambda k, d: ev.append(k))
    stuck = _Fake(4, 40.0, -2.5, dims=PED)
    log, end = _drive(pol, path, lambda t, x: [stuck], n=1400)
    check('교착: ped_timeout 발화', 'ped_timeout' in ev, ','.join(ev))
    check('교착: 결국 지나간다', end > 60.0, f'{end:.1f}m 진행')
    check('교착: 완료 판정이 프레임마다 반복되지 않는다',
          ev.count('ped_done') <= 1, f'ped_done {ev.count("ped_done")}')

    # 7. 교착이어도 차폭 안 보행자는 들이받지 않는다
    pol = ObstaclePolicy(_Straight(), max_block=3.0)
    inpath = _Fake(5, 40.0, 0.2, dims=PED)
    log, end = _drive(pol, path, lambda t, x: [inpath], n=1400)
    stop_gap = (inpath.x - inpath.length * 0.5) - (end + FRONT_OVERHANG)
    check('교착: 차폭 안 보행자는 통과하지 않는다', stop_gap > 0.5,
          f'범퍼-보행자 {stop_gap:.2f}m')

    # 8. 종료 구간 — 경로 끝 30m 안의 라바콘은 종료 라바콘이라 무시한다.
    #    같은 라바콘이 구간 밖이면 예전처럼 감속한다
    ev = []
    pol = ObstaclePolicy(_Straight(), on_event=lambda k, d: ev.append(k))
    gate = [_Fake(7, 297.0, 1.0, dims=CONE), _Fake(8, 297.0, -1.0, dims=CONE)]
    log, end = _drive(pol, path, lambda t, x: gate, n=600)
    check('종료 라바콘: 감속 없음',
          'obstacle_slow' not in ev and all(r[3] == 'none' for r in log),
          f'{sorted({r[3] for r in log})} {",".join(ev)}')
    check('종료 라바콘: finish_zone_ignore 2건', ev.count('finish_zone_ignore') == 2,
          ','.join(ev))
    check('종료 라바콘: 지나간다', end > 300.0, f'{end:.1f}m 진행')
    ev = []
    pol = ObstaclePolicy(_Straight(), on_event=lambda k, d: ev.append(k))
    log, _ = _drive(pol, path, lambda t, x: [_Fake(9, 260.0, 0.4, dims=CONE)], n=600)
    check('구간 밖 라바콘: 여전히 감속', any(r[3] == 'slow' for r in log)
          and 'finish_zone_ignore' not in ev, ','.join(ev))
    ev = []
    pol = ObstaclePolicy(_Straight(), on_event=lambda k, d: ev.append(k))
    log, end = _drive(pol, path, lambda t, x: [_Fake(10, 290.0, 0.0, dims=CAR)], n=600)
    check('종료 구간 차량: 그대로 선다', any(r[3] in ('stop', 'hold') for r in log)
          and end < 290.0, f'{end:.1f}m')

    # 9. 우회 교체 -- 트랙(월드 좌표)은 새 정책이 이어받는다 (9/11)
    a = ObstaclePolicy(_Straight(), on_road=lambda x, y: True)
    _drive(a, a.path, lambda t, x: [_Fake(2, 120.0, 0.0, dims=CAR)], n=5)
    a._hold[3] = _Held(3, 0.0)
    b = ObstaclePolicy(_Straight())
    b.adopt(a)
    check('adopt: 트랙 승계 (trusted)', 2 in b._tracks and b._tracks[2].trusted
          and b._tracks[2].n_seen == a._tracks[2].n_seen,
          f'{b._tracks.get(2)}')
    check('adopt: 보행자 횡단 대기·on_road 승계',
          3 in b._hold and b.on_road is not None)
    b.adopt(None)
    check('adopt(None): 무해', 2 in b._tracks and b._hold_since is None)

    print()
    if fails:
        print(f'자기검사 {len(fails)}건 실패: {", ".join(fails)}')
        return 1
    print('자기검사 전부 통과')
    return 0


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(
        description='충돌 회피 정책의 오프라인 재생 / 자기검사 (항목 10·11·14)')
    ap.add_argument('--replay', default='', help='세션 디렉터리')
    ap.add_argument('--path', default='', help='waypoint CSV')
    ap.add_argument('--segments', default='', help='쉼표로 구분한 구간 이름')
    ap.add_argument('--selftest', action='store_true',
                    help='로그에 없는 상황을 합성해 확인한다 (차로 내 라바콘 등)')
    args = ap.parse_args()
    if args.selftest:
        raise SystemExit(_selftest())
    if not (args.replay and args.path and args.segments):
        ap.error('--replay/--path/--segments를 함께 주거나 --selftest를 쓸 것')
    _replay(args.replay, args.path, args.segments.split(','))


if __name__ == '__main__':
    main()
