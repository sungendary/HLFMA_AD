#!/usr/bin/env python3
"""TTC 충돌 예측 ― 상대 운동 기반 (평가항목 14 강화).

obstacles.py는 객체를 **내 경로에 투영한 지금 위치**로 판정한다. "지금 내 경로 위에
있는 것"만 보므로 다음이 구조적으로 안 잡힌다 ― 옆 차로에서 끼어드는 차, 교차로에서
직각으로 들어오는 차, 추월한 뒤 앞으로 들어오는 차. 전부 차로에 들어온 순간에야
보이고 45km/h에서는 이미 늦다. 주최측이 NPC의 끼어들기·급정지 이벤트를 확인했고
NPC 귀책이어도 충돌은 참가 차량 감점(항목 14 중대 -6)이다.

이 모듈은 **상대 운동**을 본다. 4초 안의 최근접점(CPA)을 찾아 최근접 거리가 차폭
안이고 최근접 시각이 가까우면 속도 상한을 낸다. stoplines/obstacles와 같은 순수
로직이고 pure_pursuit이 min()으로 섞는다.

--- 설계를 정한 것 ---

1. **ego는 경로를 따라간다.** 직선 외삽은 커브에서 오탐이 크다 ― 반경 100m에서
   40m 앞은 경로에서 8m 벗어난다. ego는 실제로 경로를 따라가므로 s_ego + v·τ의
   경로 점을 쓴다.

2. **객체는 경로 좌표계(s, lat)에서 외삽한다.** 월드 직선 외삽은 커브에서 옆 차로
   차량을 우리 차로로 밀어 넣는다 ― 반경 60m 안쪽 차로의 차가 20m 뒤에 3.3m 밖으로
   나온다고 계산된다. 실제 NPC는 도로를 따라 달리므로 along 속도와 lateral 속도로
   나눠 외삽하는 쪽이 맞다. 직선 도로에서는 해석식(t_cpa = -p·v/|v|²)과 같다.
   **헤딩이 경로와 45도 안이고 우리 쪽으로 오는 객체는 차로에서 멈춘다**(끼어들기).
   그 밖(교차 진입)은 그대로 통과한다. 그렇지 않으면 등속 외삽이 끼어드는 차를
   "스쳐 지나가는 궤적"으로 계산해 차로에 들어온 뒤에야 잡는다.

3. **현재 횡오프셋은 2초에 걸쳐 경로로 수렴시킨다.** 우회 교체 직후(블렌드 시작)나
   리스폰 복귀 중에는 ego가 경로에서 3m 옆에 있다. τ=0 예측이 실제 위치와 같아야
   하고, 그 뒤는 블렌드 거동 그대로 경로로 붙는다. 경로 교체 프레임에 잘못된 미래를
   계산하지 않는 장치다.

4. **오탐 방지가 가장 중요하다.** 불필요한 급제동은 시간 손실이고 뒤차 추돌
   위험이다. (a) speed/heading을 믿을 수 있는 트랙(id 재사용 검증 통과)만 보고,
   (b) 이미 뒤에 있는 객체는 버리고, (c) 제 차로로 마주오는 대향차는 지금 횡거리로
   걸러내고, (d) 진입 2프레임·이탈 1.5배 히스테리시스를 걸고, (e) 실제로 속도를
   줄일 때만 warn/brake/stop을 내고, (f) 정지 객체에는 obstacles의 차로폭(2.0m)을
   그대로 써서 obstacles가 무시하는 것을 여기서 세우지 않는다.

5. **단계는 활성 중 올라가기만 한다.** 제동하면 t_cpa가 늘어나 brake가 warn으로
   내려갔다 다시 올라오는 핑퐁이 난다. 해제는 "후보에서 빠졌거나 확실히 벗어났을
   때"뿐이다.

6. **정지(v_limit=0)는 1초 안의 임박 상황에만 낸다.** 그 밖에는 10km/h 하한을
   두어 교착을 막는다. 정지 상태에서 이 모듈은 아무것도 내지 않는다 ― 서 있는
   객체는 obstacles가 맡는다.
"""

from __future__ import annotations

import bisect
import math

import obstacles as OB
from stoplines import FRONT_OVERHANG
from finish import FINISH_ZONE_M, gate_s

D_CRIT = 2.5                # m, 최근접 거리 임계. ego 반폭 0.95 + 객체 반폭 + 여유
T_WARN = 4.0                # s, 이 안의 최근접만 본다
T_CRIT = 3.0                # s, 이 안이면 정지 프로파일
T_IMMINENT = 1.0            # s, 이 안이면 하한 없는 정지
TAU_STEP = 0.2              # s, 예측 샘플 간격 (45km/h에서 2.5m)
V_FLOOR = 10.0 / 3.6        # m/s, 임박이 아니면 이 아래로는 내리지 않는다
WARN_FRAC = 0.7             # warn 티어: 진입 시 속도의 이 비율

BEHIND_M = 2.0              # m, 헤딩 기준 이만큼 뒤면 버린다
MIN_GAP = OB.MIN_GAP        # m, 앞차 뒤범퍼와의 정지 간격 (obstacles와 같게)
DECEL = OB.DECEL_OBJ        # m/s^2, 정지 프로파일 (obstacles와 같게)
EGO_CENTER_M = 1.4          # m, 후륜축 -> 차체 중심 (앞 3.808 / 뒤 1.0 가정)
EGO_MIN_V = 1.0 / 3.6       # m/s, 이 아래면 계산하지 않는다
OFF_DECAY_S = 2.0           # s, 현재 횡오프셋이 경로로 수렴하는 시간
OFF_MAX_M = 8.0             # m, 경로에서 이보다 멀면 판정하지 않는다
PREFILTER_M = 30.0          # m, 직선거리 프리필터 여유
ONCOMING_COS = -0.5         # cos(헤딩차) < 이 값이면 대향 (120도 이상)
MERGE_RAD = math.radians(45.0)  # 경로와 이 안의 헤딩이면 차로에 정착(끼어들기)
STATIONARY_MS = 0.3         # m/s, 이 아래면 정지 객체 (obstacles 차로폭 적용)
LAT_MAX_M = 35.0            # m, 경로에서 이보다 옆이면 보지 않는다. 루프 경로에서
                            # 44m 옆 다른 도로의 차가 "옆으로 접근"으로 잡혔다.
                            # 교차로 측면 진입은 25~30m에서 시작한다

ENTER_FRAMES = 2            # 연속 이만큼 위험해야 활성
EXIT_RATIO = 1.5            # 해제 거리 = D_CRIT * 이 값
ACT_MARGIN_MS = 0.3         # m/s, 상한이 현재 속도보다 이만큼 낮아야 "실제 제한"
TTC_MAX_HOLD_S = 10.0       # s, stop을 이만큼 유지하면 교착으로 보고 완화
RELEASE_FRAMES = 30         # 프레임, 위험이 사라진 채 이만큼 지속돼야 해제 (1.5s).
                            # 0.5s로는 제동으로 풀린 교차 충돌에 다시 가속해 2.2m로 스쳤다
HOLD_EXTRA_M = 2.0          # m, 정지 유지: 앞차 간격이 MIN_GAP+이 값 안일 때만
HOLD_LOOK_M = 20.0          # m, 정지 유지 판정 창

N_SAMPLES = int(round(T_WARN / TAU_STEP))
EXTRAP_SAMPLES = 20         # 창 끝에서 이만큼(4s)까지 꼭짓점을 외삽한다
T_RELEASE = 2.0 * T_WARN    # s, t_cpa가 이보다 크면 위험이 사라진 것 (해제 유예로)
LEVELS = {'': 0, 'warn': 1, 'brake': 2, 'stop': 3}


class TtcCmd:
    """한 프레임의 판정. obstacles.ObstacleCmd와 같은 모양이다."""

    __slots__ = ('v_limit', 'accel_limit', 'action', 'kind', 'obj_id',
                 't_cpa', 'd_cpa', 'reason')

    def __init__(self, v_limit=float('inf'), action='none', kind='',
                 obj_id=-1, t_cpa=float('nan'), d_cpa=float('nan'),
                 reason='', accel_limit=float('inf')):
        self.v_limit = v_limit          # m/s. inf면 제약 없음
        self.accel_limit = accel_limit  # m/s^2 상한. inf면 제약 없음
        self.action = action            # none / warn / brake / stop / timeout
        self.kind = kind                # obstacle / pedestrian / vehicle
        self.obj_id = obj_id
        self.t_cpa = t_cpa              # s, 최근접까지 남은 시간
        self.d_cpa = d_cpa              # m, 최근접 거리
        self.reason = reason

    def __repr__(self) -> str:
        return (f'<TtcCmd {self.action} id={self.obj_id} v<={self.v_limit:.1f} '
                f't={self.t_cpa:.1f}s d={self.d_cpa:.1f}m>')


class _Tr:
    """최소 트랙. obstacles.Track과 같은 규칙으로 trusted를 낸다."""

    __slots__ = ('id', 'x', 'y', 'n_seen', 'missing', 'trusted')

    def __init__(self, oid, x, y):
        self.id = oid
        self.x, self.y = x, y
        self.n_seen = 1
        self.missing = 0
        self.trusted = False


class _Act:
    """활성(위험) 객체 하나의 래치 상태."""

    __slots__ = ('id', 'enter', 'level', 'v_entry', 'stop_since', 'timed_out',
                 'announced', 'miss', 'last', 'told')

    def __init__(self, oid):
        self.id = oid
        self.enter = 0              # 연속 위험 프레임 수
        self.level = ''             # 도달한 최고 단계 ('' / warn / brake / stop)
        self.v_entry = 0.0          # warn 진입 시 속도 (래치)
        self.stop_since = None      # stop이 실제로 걸리기 시작한 시각
        self.timed_out = False
        self.announced = False      # 이벤트를 한 번이라도 냈나 (clear 발화 조건)
        self.miss = 0               # 후보에서 빠진 연속 프레임 수 (해제 유예)
        self.last = None            # 유예 중 유지할 마지막 (v_lim, a_lim, t_cpa, d_cpa, kind)
        self.told = ''              # 이벤트로 알린 최고 단계

    @property
    def active(self) -> bool:
        return self.enter >= ENTER_FRAMES


def _point_at(path, s: float) -> tuple:
    """경로 누적거리 s의 점. 끝을 넘으면 마지막 선분으로 외삽한다."""
    sarr, pts = path.s, path.pts
    n = len(pts)
    i = bisect.bisect_right(sarr, s) - 1
    i = max(0, min(i, n - 2))
    seg = sarr[i + 1] - sarr[i]
    u = 0.0 if seg < 1e-9 else (s - sarr[i]) / seg
    u = max(0.0, u) if i == n - 2 else max(0.0, min(1.0, u))
    ax, ay = pts[i]
    bx, by = pts[i + 1]
    return ax + u * (bx - ax), ay + u * (by - ay)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _limit(avail: float, v: float, v_lead: float, floor: float) -> tuple:
    """(속도 상한, 가속도 상한). obstacles._profile에 하한을 얹은 것이다.

    하한을 **프로파일에 넣어서** 낸다. 하한을 적용한 뒤 별도로 accel을 내면
    v <= v_limit인데도 급제동 피드포워드가 남는 모순이 생긴다.

    **하한 자체도 남은 거리로 묶는다.** 안 그러면 정지 차량 앞에서 avail이 다
    떨어졌는데도 10km/h로 기어 들어간다. 하한은 "예측이 애매할 때 과제동을
    막는" 장치지 "실제 간격을 무시하는" 장치가 아니다.
    """
    floor = min(floor, math.sqrt(2.0 * DECEL * max(0.0, avail)))
    return OB._profile(avail, v, max(v_lead, floor), DECEL)


def _cpa(s_e: float, lat_e: float, v: float,
         s_o: float, lat_o: float, v_along: float, v_lat: float,
         settle: bool) -> tuple:
    """경로 좌표계 최근접점. (t_cpa, d_cpa, k). k==0이면 지금이 최근접이다.

    ego: (s_e + v·τ, lat_e·(1-τ/OFF_DECAY_S)). 객체: (s_o + v_along·τ,
    lat_o + v_lat·τ). settle이면 객체의 lat이 0을 넘지 않는다(차로에 정착).
    """
    best_k, best_d2 = 0, float('inf')
    d2s = []
    sign = 1.0 if lat_o >= 0.0 else -1.0
    for k in range(N_SAMPLES + 1):
        tau = k * TAU_STEP
        w = 1.0 - tau / OFF_DECAY_S
        el = lat_e * w if w > 0.0 else 0.0
        ol = lat_o + v_lat * tau
        if settle and ol * sign < 0.0:
            ol = 0.0
        ds = s_o + v_along * tau - (s_e + v * tau)
        dl = ol - el
        d2 = ds * ds + dl * dl
        d2s.append(d2)
        if d2 < best_d2:
            best_k, best_d2 = k, d2
    t_cpa = best_k * TAU_STEP
    n = len(d2s)
    if best_k > 0:
        # 이웃 세 점으로 포물선 보간. 직선 도로에서는 d²가 정확히 2차다.
        # 최근접이 창 끝(k==N)이면 진짜 최근접은 창 밖이다 ― 그대로 두면
        # "τ=4s의 거리"를 d_cpa로 잘못 보고하고 제동 중에 그 값이 흔들린다.
        # 꼭짓점을 창 밖으로 외삽해 t_cpa > T_WARN으로 정직하게 낸다
        kc = min(best_k, n - 2)
        a, b, c = d2s[kc - 1], d2s[kc], d2s[kc + 1]
        den = a - 2.0 * b + c
        if den > 1e-9:
            off = 0.5 * (a - c) / den
            off = max(-1.0, min(EXTRAP_SAMPLES, off))
            t_cpa = (kc + off) * TAU_STEP
            best_d2 = max(0.0, b - 0.25 * (a - c) * off)
    return t_cpa, math.sqrt(best_d2), best_k


class TtcPolicy:
    """매 프레임 속도 상한 하나를 낸다.

    트랙·래치는 월드 좌표라 경로와 무관하다. 경로가 바뀌면 rebind(path)만 한다.
    """

    def __init__(self, path, *, d_crit: float = D_CRIT,
                 front_overhang: float = FRONT_OVERHANG,
                 finish_zone: float = FINISH_ZONE_M, on_event=None):
        self.path = path
        self.d_crit = d_crit
        self.front = front_overhang
        self.finish_zone = finish_zone
        self._gate_s = gate_s(path, finish_zone)
        self._on_event = on_event or (lambda kind, detail: None)
        self.reset()

    # -- 상태 -----------------------------------------------------------
    def reset(self) -> None:
        """리스폰 시. 위치가 강제로 옮겨졌으므로 트랙·래치를 전부 버린다."""
        self._tracks: dict = {}
        self._acts: dict = {}       # id -> _Act

    def rebind(self, path) -> None:
        """경로 교체 후. 트랙과 래치는 월드 좌표라 그대로 둔다."""
        self.path = path
        self._gate_s = gate_s(path, self.finish_zone)

    # -- 추적 -----------------------------------------------------------
    def _update_tracks(self, objects) -> None:
        seen = set()
        for o in objects:
            seen.add(o.id)
            tr = self._tracks.get(o.id)
            if tr is not None and math.hypot(o.x - tr.x, o.y - tr.y) > OB.ID_JUMP_M:
                tr = None               # id 재사용. obstacles가 이벤트를 낸다
                self._acts.pop(o.id, None)
            if tr is None:
                self._tracks[o.id] = _Tr(o.id, o.x, o.y)
                continue
            tr.x, tr.y = o.x, o.y
            tr.n_seen += 1
            tr.missing = 0
            tr.trusted = tr.n_seen >= OB.TRUST_FRAMES
        for oid in [k for k in self._tracks if k not in seen]:
            tr = self._tracks[oid]
            tr.missing += 1
            if tr.missing > OB.TRACK_GRACE:
                del self._tracks[oid]

    # -- 본체 -----------------------------------------------------------
    def update(self, idx: int, s_ego: float, v: float,
               ego_x: float, ego_y: float, ego_heading: float,
               objects, t: float) -> TtcCmd:
        self._update_tracks(objects)
        cmd = TtcCmd()
        path = self.path
        s_e, lat_e, _th = OB._project(path, max(0, idx - 1),
                                      min(idx + 2, len(path.pts) - 1),
                                      ego_x, ego_y)
        if abs(lat_e) > OFF_MAX_M:
            self._acts.clear()
            cmd.reason = '경로 밖'
            return cmd
        if v < EGO_MIN_V:
            # 정지 상태. 예측할 상대 운동이 없다. stop 래치만 "그 객체가 아직
            # 코앞에 있는가"로 유지하고(안 그러면 서다 가다 톱니가 난다) 나머지는
            # 버린다. 서 있는 객체의 본 담당은 obstacles다
            return self._hold(cmd, idx, s_e, ego_heading, objects, t)
        s_c = s_e + EGO_CENTER_M            # 차체 중심

        ch, sh = math.cos(ego_heading), math.sin(ego_heading)
        horizon = v * T_WARN + PREFILTER_M
        i_lo, i_hi = OB._window(path, idx, BEHIND_M + 3.0, horizon)
        cands = {}          # id -> (o, cls, t_cpa, d_cpa, s_o, lat_o, v_lead, crit)
        best_log = None     # 로깅용 최소 t_cpa 후보 (활성 아니어도)
        for o in objects:
            dx, dy = o.x - ego_x, o.y - ego_y
            if math.hypot(dx, dy) > horizon + abs(o.speed) * T_WARN:
                continue
            tr = self._tracks.get(o.id)
            if tr is None or not tr.trusted:
                continue
            if dx * ch + dy * sh < -BEHIND_M:
                continue
            cls = OB.classify(o)
            s_o, lat_o, th_p = OB._project(path, i_lo, i_hi, o.x, o.y)
            if cls == OB.OBSTACLE and s_o >= self._gate_s:
                continue            # 종료 라바콘. obstacles와 같은 기준
            if abs(lat_o) > LAT_MAX_M:
                continue            # 다른 도로
            if s_o >= path.length - 1.0:
                continue            # 경로 끝 너머 -- 마지막 점에 붙어 유령이 된다
            crit = self.d_crit
            if abs(o.speed) < STATIONARY_MS and cls == OB.VEHICLE:
                crit = min(crit, OB.VEH_LANE_M)     # obstacles가 무시하면 우리도
            dh = _wrap(o.heading - th_p)
            cos_dh = math.cos(dh)
            # 가로지르는 차는 중심 거리로 재면 안 된다 -- 차체 길이(4.4m)가 옆으로
            # 놓여 있다. 헤딩 차만큼 반길이를 임계에 더한다 (직각이면 +2.2m).
            # sim에서 교차 차량이 0.38m 스쳐 지나갔다
            crit += 0.5 * o.length * abs(math.sin(dh))
            if cos_dh < ONCOMING_COS and abs(lat_o) >= crit:
                continue            # 제 차로로 마주오는 차
            v_along = o.speed * cos_dh
            v_lat = o.speed * math.sin(dh)
            settle = abs(dh) < MERGE_RAD and lat_o * v_lat < 0.0
            t_cpa, d_cpa, k = _cpa(s_c, lat_e, v, s_o, lat_o, v_along, v_lat,
                                   settle)
            if k == 0:
                continue            # 지금이 최근접 = 멀어지는 중
            cands[o.id] = (o, cls, t_cpa, d_cpa, s_o, lat_o, max(0.0, v_along),
                           crit)
            if best_log is None or t_cpa < best_log[2]:
                best_log = cands[o.id]

        # 래치 갱신. 해제는 "후보에서 빠졌거나 확실히 벗어난 채 RELEASE_FRAMES
        # 지속"이다 ― 제동으로 t_cpa가 늘어난 것이나 한 프레임 닫힘이 멈춘 것으로
        # 풀면 핑퐁이 난다
        for oid in list(self._acts):
            a = self._acts[oid]
            c = cands.get(oid)
            if c is None or c[3] >= c[7] * EXIT_RATIO or c[2] > T_RELEASE:
                if not a.active:
                    del self._acts[oid]     # 확인 중이었는데 빠졌다
                    continue
                a.miss += 1
                if a.miss > RELEASE_FRAMES:
                    self._release(oid)
                continue
            a.miss = 0
        for oid, c in cands.items():
            if c[3] < c[7] and 0.0 < c[2] <= T_WARN:
                a = self._acts.get(oid)
                if a is None:
                    a = self._acts[oid] = _Act(oid)
                a.enter += 1

        # 활성 객체마다 상한을 내고 min(). 유예 중(miss>0)인 것은 **마지막
        # 상한을 유지한다** ― 풀어 버리면 재가속 -> 다시 위험 -> 제동의 토글이 난다
        worst = None
        for oid, a in self._acts.items():
            if not a.active:
                continue
            if a.miss > 0:
                if a.last is not None:
                    v_lim, a_lim, t_cpa, d_cpa, cls = a.last
                    key = (v_lim, t_cpa)
                    if worst is None or key < worst[0]:
                        worst = (key, oid, cls, t_cpa, d_cpa, a.level, v_lim, a_lim)
                continue
            o, cls, t_cpa, d_cpa, s_o, lat_o, v_lead, _crit = cands[oid]
            dh = _wrap(o.heading - ego_heading)
            half_l = (0.5 * o.length * abs(math.cos(dh))
                      + 0.5 * o.width * abs(math.sin(dh)))
            avail_now = (s_o - half_l) - (s_e + self.front) - MIN_GAP
            avail_cpa = v * t_cpa - (self.front - EGO_CENTER_M) - half_l - MIN_GAP
            avail = min(avail_now, avail_cpa)
            if t_cpa <= T_IMMINENT:
                level = 'stop'
            elif t_cpa <= T_CRIT:
                level = 'brake'
            else:
                level = 'warn'
            if LEVELS[level] < LEVELS[a.level]:
                level = a.level         # 활성 중에는 내려가지 않는다
            if level == 'stop':
                v_lim, a_lim = _limit(avail, v, 0.0, 0.0)
            elif level == 'brake':
                v_lim, a_lim = _limit(avail, v, v_lead, V_FLOOR)
            else:
                if a.level != 'warn':
                    a.v_entry = v
                v_lim = max(WARN_FRAC * a.v_entry, V_FLOOR)
                a_lim = float('inf')
            effective = v_lim < v - ACT_MARGIN_MS
            if level == 'stop' and v_lim <= 0.01:
                if a.stop_since is None:
                    a.stop_since = t
                self._check_timeout(a, cls, t, t_cpa, d_cpa)
            if a.timed_out:
                v_lim, a_lim = _limit(avail, v, V_FLOOR, V_FLOOR)
                shown = 'timeout'
            else:
                shown = level
                if effective and LEVELS[level] > LEVELS[a.told]:
                    # 실제로 속도를 줄일 때, 알린 것보다 높은 단계만 알린다
                    self._on_event(
                        f'ttc_{level}',
                        f'id={oid} {cls} t_cpa={t_cpa:.1f}s d_cpa={d_cpa:.2f}m '
                        f'앞 {s_o - s_e:.1f}m lat={lat_o:+.1f}m '
                        f'v_obj={o.speed * 3.6:.0f}km/h '
                        f'v {v * 3.6:.1f} -> <={v_lim * 3.6:.1f}km/h')
                    a.announced = True
                    a.told = level
            a.level = level
            a.last = (v_lim, a_lim, t_cpa, d_cpa, cls)
            key = (v_lim, t_cpa)
            if worst is None or key < worst[0]:
                worst = (key, oid, cls, t_cpa, d_cpa, shown, v_lim, a_lim)

        if worst is not None:
            _k, oid, cls, t_cpa, d_cpa, shown, v_lim, a_lim = worst
            cmd.v_limit, cmd.accel_limit = v_lim, a_lim
            cmd.action, cmd.kind, cmd.obj_id = shown, cls, oid
            cmd.t_cpa, cmd.d_cpa = t_cpa, d_cpa
            cmd.reason = f'{cls} id={oid} t_cpa={t_cpa:.1f}s d_cpa={d_cpa:.1f}m'
        elif best_log is not None:
            o, cls, t_cpa, d_cpa = best_log[:4]
            cmd.kind, cmd.obj_id = cls, o.id
            cmd.t_cpa, cmd.d_cpa = t_cpa, d_cpa
        return cmd

    def _hold(self, cmd: TtcCmd, idx: int, s_e: float, ego_heading: float,
              objects, t: float) -> TtcCmd:
        """정지 상태. stop 래치를 "그 객체가 아직 코앞에 있는가"로만 유지한다."""
        by_id = {o.id: o for o in objects}
        i_lo, i_hi = OB._window(self.path, idx, BEHIND_M + 3.0, HOLD_LOOK_M)
        keep = set()
        for oid, a in self._acts.items():
            if not a.active or a.level not in ('brake', 'stop'):
                continue
            o = by_id.get(oid)
            if o is None:
                continue
            s_o, lat_o, _th = OB._project(self.path, i_lo, i_hi, o.x, o.y)
            dh = _wrap(o.heading - ego_heading)
            half_l = (0.5 * o.length * abs(math.cos(dh))
                      + 0.5 * o.width * abs(math.sin(dh)))
            gap = (s_o - half_l) - (s_e + self.front)
            if abs(lat_o) >= self.d_crit * EXIT_RATIO or gap > MIN_GAP + HOLD_EXTRA_M \
                    or gap < -half_l:
                continue
            keep.add(oid)
            if a.stop_since is None:
                a.stop_since = t
            self._check_timeout(a, OB.classify(o), t, 0.0, abs(lat_o))
            v_lim = V_FLOOR if a.timed_out else 0.0
            if v_lim < cmd.v_limit:
                cmd.v_limit = v_lim
                cmd.action = 'timeout' if a.timed_out else 'stop'
                cmd.kind, cmd.obj_id = OB.classify(o), oid
                cmd.t_cpa, cmd.d_cpa = 0.0, abs(lat_o)
                cmd.reason = f'정지 유지 (앞 {gap:.1f}m)'
        for oid in list(self._acts):
            if oid not in keep:
                self._release(oid)
        return cmd

    def _check_timeout(self, a: '_Act', cls: str, t: float,
                       t_cpa: float, d_cpa: float) -> None:
        if a.timed_out or a.stop_since is None:
            return
        if t - a.stop_since > TTC_MAX_HOLD_S:
            a.timed_out = True
            self._on_event(
                'ttc_timeout',
                f'id={a.id} {cls} 정지 상한 {TTC_MAX_HOLD_S:.0f}s 초과 ― '
                f'교착으로 보고 {V_FLOOR * 3.6:.0f}km/h로 완화 '
                f'(t_cpa={t_cpa:.1f}s d_cpa={d_cpa:.1f}m)')

    def _release(self, oid: int) -> None:
        a = self._acts.pop(oid, None)
        if a is not None and a.announced:
            self._on_event('ttc_clear', f'id={oid} 위험 해제')


def summary(policy: TtcPolicy) -> list:
    return [
        f'  예측: ego는 경로 위 s+v·τ, 객체는 경로 좌표계 (along, lat) 외삽. '
        f'τ 0~{T_WARN:.0f}s {TAU_STEP:.1f}s 간격, 횡오프셋 {OFF_DECAY_S:.0f}s 수렴, '
        f'경로 ±{math.degrees(MERGE_RAD):.0f}° 헤딩은 차로에 정착(끼어들기)',
        f'  위험: d_cpa<{policy.d_crit:.1f}m (정지 차량은 {OB.VEH_LANE_M:.1f}m), '
        f't_cpa<={T_WARN:.0f}s. warn>{T_CRIT:.0f}s ({WARN_FRAC:.0%}) / '
        f'brake<={T_CRIT:.0f}s (하한 {V_FLOOR * 3.6:.0f}km/h) / stop<={T_IMMINENT:.0f}s',
        f'  게이트: trusted 트랙만, 뒤 {BEHIND_M:.0f}m 제외, 제 차로 대향차 제외, '
        f'진입 {ENTER_FRAMES}프레임 / 해제 x{EXIT_RATIO:.1f}, '
        f'stop {TTC_MAX_HOLD_S:.0f}s 뒤 완화',
    ]


# ----------------------------------------------------------------------
# 오프라인 재생 ― sim 없이 로그만으로 반응을 본다

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
                by_n.setdefault(int(r['n']), []).append(OB._Obj(r))
            except (ValueError, TypeError, KeyError):
                continue

    def show(kind: str, detail: str) -> None:
        print(f'  [{kind}] {detail}')

    policy = TtcPolicy(path, on_event=show)
    print(f'[ttc] 경로 {len(path.pts)}점 {path.length:.1f}m')
    for line in summary(policy):
        print('[ttc]' + line)
    print(f'[ttc] {os.path.basename(os.path.normpath(session))} 재생\n')

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
            cmd = policy.update(idx, path.s[idx], v, x, y, h, by_n.get(n, ()), t)
            if cmd.action != 'none':
                n_react += 1
            if cmd.action != prev_act:
                print(f'  t={t:6.1f}s v={v * 3.6:5.1f}km/h  '
                      f'{prev_act:>7s} -> {cmd.action:<7s} {cmd.kind:<10s} '
                      f'id={cmd.obj_id:<4d} t_cpa={cmd.t_cpa:4.1f}s '
                      f'd_cpa={cmd.d_cpa:5.2f}m v<={cmd.v_limit * 3.6:5.1f}km/h')
                prev_act = cmd.action
    print(f'\n[ttc] 반응 프레임 {n_react}개')


# ----------------------------------------------------------------------
# 자기검사 ― 로그에 없는 상황을 합성한다

class _Arc:
    """반경 R 원호 (좌회전). obstacles._Straight와 같은 계약."""

    def __init__(self, radius=60.0, length=300.0, step=1.0):
        n = int(length / step) + 1
        self.pts = [(radius * math.sin(i * step / radius),
                     radius * (1.0 - math.cos(i * step / radius)))
                    for i in range(n)]
        self.s = [i * step for i in range(n)]
        self.length = self.s[-1]

    def _advance(self, i, dist_m):
        target = self.s[i] + dist_m
        j = i
        while j < len(self.pts) - 1 and self.s[j] < target:
            j += 1
        return j


def _drive_path(policy, path, objs_at, v0=12.5, dt=0.05, n=400,
                lat0=0.0, lat_decay_s=2.0):
    """경로를 따라 달리는 적분기. ego는 접선 헤딩, 처음 lat0만큼 옆에서 시작해
    lat_decay_s에 걸쳐 경로로 붙는다 (블렌드/리스폰 복귀 흉내). (기록, 최종 s)."""
    s, v, idx = 0.0, v0, 0
    log = []
    for k in range(n):
        t = k * dt
        while idx < len(path.pts) - 1 and path.s[idx + 1] <= s:
            idx += 1
        px, py = _point_at(path, s)
        qx, qy = _point_at(path, s + 1.0)
        th = math.atan2(qy - py, qx - px)
        lat = lat0 * max(0.0, 1.0 - t / lat_decay_s)
        ex, ey = px - math.sin(th) * lat, py + math.cos(th) * lat
        cmd = policy.update(idx, path.s[idx], v, ex, ey, th, objs_at(t, s), t)
        v_cmd = min(v0, cmd.v_limit)
        a = max(-6.0, min(3.0, min(1.2 * (v_cmd - v), cmd.accel_limit)))
        if v_cmd <= 0.0:
            a = min(a, -1.5)
        s += v * dt
        v = max(0.0, v + a * dt)
        log.append((t, s, v, cmd.action, cmd.kind, cmd.obj_id, cmd.v_limit,
                    cmd.t_cpa, cmd.d_cpa))
    return log, s


def _smooth(u: float) -> float:
    u = max(0.0, min(1.0, u))
    return u * u * (3.0 - 2.0 * u)


def _lane_changer(oid, x0, lat0, v_along, t_start, dur, v_end=None,
                  dims=(4.39, 1.81, 1.35)):
    """옆 차로에서 우리 차로로 들어오는 스크립트 객체. (t, x) -> [obj]."""
    st = {'x': x0, 'lat': lat0, 'v': v_along}
    v_end = v_along if v_end is None else v_end

    def at(t, x):
        u = _smooth((t - t_start) / dur)
        lat = lat0 * (1.0 - u)
        vv = v_along + (v_end - v_along) * u
        st['x'] += vv * 0.05
        dlat = (lat - st['lat']) / 0.05
        st['lat'] = lat
        return [OB._Fake(oid, st['x'], lat, speed=math.hypot(vv, dlat),
                         heading=math.atan2(dlat, vv), dims=dims)], lat
    return at


def _selftest() -> int:
    import time as _time
    fails = []

    def check(name, ok, detail=''):
        print(f'  {"PASS" if ok else "FAIL"}  {name}' + (f'  ― {detail}' if detail else ''))
        if not ok:
            fails.append(name)

    def acts(log):
        return {r[3] for r in log}

    def first(log, *levels):
        return next((r for r in log if r[3] in levels), None)

    Straight, Fake = OB._Straight, OB._Fake
    CAR, PED, CONE = OB.CAR, OB.PED, OB.CONE

    # 1. 같은 차로 앞차 정지 -> stop. obstacles 없이도 그 앞에서 거의 선다.
    #    선 뒤에는 이 모듈이 손을 떼므로(정지 객체는 obstacles 몫) 첫 정지 순간만 본다
    ev = []
    path = Straight()
    pol = TtcPolicy(path, on_event=lambda k, d: ev.append(k))
    car = Fake(1, 80.0, 0.0, dims=CAR)
    log, end = OB._drive(pol, path, lambda t, x: [car], n=300)   # 15s: 타임아웃(정지+10s) 전
    check('정지 앞차: brake/stop 발화', 'ttc_brake' in ev or 'ttc_stop' in ev, ','.join(ev))
    halted = next((r for r in log if r[2] < 0.5), None)
    gap = ((car.x - car.length * 0.5) - (halted[1] + FRONT_OVERHANG)
           if halted else float('nan'))
    check('정지 앞차: 그 앞에서 선다', halted is not None and 0.0 < gap < MIN_GAP + 6.0,
          f'간격 {gap:.2f}m')
    ups = [k for k in ev if k != 'ttc_clear']
    check('정지 앞차: 단계 핑퐁 없음', len(set(ups)) == len(ups), ','.join(ev))

    # 2. 대향 직진 (옆 차로, 상대속도 90km/h) -> none. **오탐 검사**
    ev = []
    pol = TtcPolicy(path, on_event=lambda k, d: ev.append(k))
    def oncoming(t, x):
        return [Fake(2, 100.0 - 12.5 * t, 3.5, speed=12.5, heading=math.pi, dims=CAR)]
    log, _ = OB._drive(pol, path, oncoming, n=160)
    check('대향차 옆 차로: 무반응', acts(log) == {'none'} and not ev,
          f'{sorted(acts(log))} {",".join(ev)}')
    # 2b. 제 차로에 들어온 대향차는 잡는다
    ev = []
    pol = TtcPolicy(path, on_event=lambda k, d: ev.append(k))
    def headon(t, x):
        return [Fake(2, 100.0 - 12.5 * t, 0.3, speed=12.5, heading=math.pi, dims=CAR)]
    log, _ = OB._drive(pol, path, headon, n=160)
    check('대향차 제 차로: brake/stop', bool(acts(log) & {'brake', 'stop'}),
          f'{sorted(acts(log))}')

    # 3. 우측에서 직각 진입, 교차 시각 일치 -> brake, 교차점 앞에서 감속
    ev = []
    pol = TtcPolicy(path, on_event=lambda k, d: ev.append(k))
    def crossing(t, x):
        # x=40에서 교차. ego 3.1초, 객체 10m/s로 31m 아래에서 출발
        return [Fake(3, 40.0, -31.0 + 10.0 * t, speed=10.0,
                     heading=math.pi / 2, dims=CAR)]
    log, end = OB._drive(pol, path, crossing, n=160)
    check('직각 진입: brake 발화', 'ttc_brake' in ev, ','.join(ev))
    f = first(log, 'brake', 'stop')
    check('직각 진입: 1초 안에 반응', f is not None and f[0] < 1.0,
          f'첫 반응 t={f[0] if f else None}')
    vmin = min(r[2] for r in log)
    check('직각 진입: 실제로 감속', vmin < 9.0, f'최저 {vmin * 3.6:.1f}km/h')
    # 3b. 교차점을 먼저 지나가 버리는 차 (우리 도착 전에 통과) -> none
    pol = TtcPolicy(path)
    def crossed(t, x):
        return [Fake(3, 40.0, -6.0 + 10.0 * t, speed=10.0,
                     heading=math.pi / 2, dims=CAR)]
    log, _ = OB._drive(pol, path, crossed, n=100)
    check('먼저 지나가는 교차차: 무반응', acts(log) == {'none'}, f'{sorted(acts(log))}')
    # 3c. 우리 도착 직전(0.3s 앞)에 교차점을 지나는 차 -- 중심 거리는 3m지만 차체가
    #     가로놓여 있어 스친다. 임계에 반길이를 더해 잡는다
    ev = []
    pol = TtcPolicy(path, on_event=lambda k, d: ev.append(k))
    def grazing(t, x):
        return [Fake(3, 40.0, -27.5 + 10.0 * t, speed=10.0,
                     heading=math.pi / 2, dims=CAR)]
    log, _ = OB._drive(pol, path, grazing, n=160)
    check('스치는 교차차: brake/stop', bool(acts(log) & {'brake', 'stop'}),
          f'{sorted(acts(log))}')

    # 4. 옆 차로 등속 평행 -> none
    pol = TtcPolicy(path)
    log, _ = OB._drive(pol, path, lambda t, x: [Fake(4, x + 10.0, 3.5, speed=12.5, dims=CAR)], n=200)
    check('옆 차로 평행: 무반응', acts(log) == {'none'}, f'{sorted(acts(log))}')
    # 4b. 옆 차로에 **느린** 차 (끼어들지 않음) -> none
    pol = TtcPolicy(path)
    log, _ = OB._drive(pol, path, lambda t, x: [Fake(4, 15.0 + 6.0 * t, 3.5, speed=6.0, dims=CAR)], n=100)
    check('옆 차로 느린 차: 무반응', acts(log) == {'none'}, f'{sorted(acts(log))}')

    # 5. 끼어들기 ― 옆 차로의 느린 차(6m/s, 15m 앞)가 우리 앞으로 들어온다.
    #    obstacles가 처음 보는 순간(|lat| < VEH_LANE_M)보다 먼저 반응해야 한다.
    ev = []
    pol = TtcPolicy(path, on_event=lambda k, d: ev.append(k))
    mover = _lane_changer(5, 15.0, 3.5, 6.0, t_start=0.5, dur=2.5)
    lat_trace = {}
    def cutin(t, x):
        objs, lat = mover(t, x)
        lat_trace[round(t, 2)] = lat
        return objs
    log, _ = OB._drive(pol, path, cutin, n=160)
    f = first(log, 'warn', 'brake', 'stop')
    lat_at = lat_trace.get(round(f[0], 2)) if f else None
    check('끼어들기: 반응', f is not None, f'{sorted(acts(log))}')
    check('끼어들기: 차로 진입(|lat|<2.0) 전에 반응',
          lat_at is not None and lat_at > OB.VEH_LANE_M,
          f'반응 t={f[0] if f else None} lat={lat_at}')
    check('끼어들기: brake 발화', 'ttc_brake' in ev or 'ttc_stop' in ev, ','.join(ev))
    check('끼어들기: brake 1회', ev.count('ttc_brake') <= 1, ','.join(ev))

    # 6. 뒤에서 추월해 지나가는 차 -> none
    pol = TtcPolicy(path)
    log, _ = OB._drive(pol, path, lambda t, x: [Fake(6, x - 8.0 + 8.0 * t, 3.5, speed=20.5, dims=CAR)], n=200)
    check('추월 통과: 무반응', acts(log) == {'none'}, f'{sorted(acts(log))}')

    # 7. 추월 뒤 앞으로 끼어들며 감속 (16.7 -> 8) -> brake
    ev = []
    pol = TtcPolicy(path, on_event=lambda k, d: ev.append(k))
    mover = _lane_changer(7, -8.0, 3.5, 16.7, t_start=1.5, dur=2.0, v_end=8.0)
    log, _ = OB._drive(pol, path, lambda t, x: mover(t, x)[0], n=200)
    check('추월 후 끼어들기: brake', 'ttc_brake' in ev or 'ttc_stop' in ev, ','.join(ev))
    # 1.5초 전(옆에서 달리는 동안)에는 반응하지 않는다
    early = [r for r in log if r[0] < 1.4 and r[3] != 'none']
    check('추월 후 끼어들기: 옆에서 달릴 때는 무반응', not early,
          f'{[(r[0], r[3]) for r in early[:3]]}')

    # 8. ego 정지 -> none
    pol = TtcPolicy(path)
    for k in range(2):
        cmd = pol.update(0, 0.0, 0.0, 0.0, 0.0, 0.0, [Fake(8, 3.0, 0.0, dims=CAR)], k * 0.05)
    check('ego 정지: 무반응', cmd.action == 'none' and cmd.v_limit == float('inf'))

    # 9. trusted 아닌 트랙(첫 프레임)은 제외, 2프레임부터 판정
    pol = TtcPolicy(path)
    c1 = pol.update(0, 0.0, 12.5, 0.0, 0.0, 0.0, [Fake(9, 15.0, 0.0, dims=CAR)], 0.0)
    c2 = pol.update(0, 0.0, 12.5, 0.0, 0.0, 0.0, [Fake(9, 15.0, 0.0, dims=CAR)], 0.05)
    c3 = pol.update(0, 0.0, 12.5, 0.0, 0.0, 0.0, [Fake(9, 15.0, 0.0, dims=CAR)], 0.10)
    check('첫 프레임: 후보 아님', c1.obj_id == -1, f'{c1}')
    check('2프레임: 후보, 3프레임: 활성', c2.obj_id == 9 and c3.action in ('brake', 'stop'),
          f'{c2} {c3}')

    # 10. 히스테리시스 ― d_cpa가 임계 근처에서 진동해도 채터링하지 않는다
    ev = []
    pol = TtcPolicy(path, on_event=lambda k, d: ev.append(k))
    def wobble(t, x):
        lat = 2.4 if int(t / 0.1) % 2 == 0 else 2.6
        return [Fake(10, x + 20.0, lat, speed=12.5 - 4.0, dims=CAR)]
    log, _ = OB._drive(pol, path, wobble, n=200)
    n_on = sum(1 for k in ev if k in ('ttc_warn', 'ttc_brake', 'ttc_stop'))
    check('히스테리시스: 점등 1회 이하', n_on <= 1, f'{ev}')

    # 11. warn 래치 ― 70%가 매 프레임 줄지 않는다
    pol = TtcPolicy(path)
    def slow_ahead(t, x):
        return [Fake(11, 60.0 + 4.0 * t, 0.0, speed=4.0, dims=CAR)]
    log, _ = OB._drive(pol, path, slow_ahead, n=80)
    warns = [r[6] for r in log if r[3] == 'warn']
    check('warn 래치: 상한 일정', warns and max(warns) - min(warns) < 0.3,
          f'{len(warns)}프레임 {min(warns) if warns else 0:.2f}~{max(warns) if warns else 0:.2f}')

    # 12. 타임아웃 ― stop을 10초 넘게 내면 완화한다
    ev = []
    pol = TtcPolicy(path, on_event=lambda k, d: ev.append(k))
    for k in range(260):
        cmd = pol.update(0, 0.0, 5.0, 0.0, 0.0, 0.0, [Fake(12, 4.0, 0.0, dims=CAR)], k * 0.05)
    check('타임아웃: ttc_timeout 발화', 'ttc_timeout' in ev, ','.join(ev))
    check('타임아웃: 하한으로 완화', cmd.action == 'timeout' and abs(cmd.v_limit - V_FLOOR) < 1e-6,
          f'{cmd}')

    # 13. **경로 밖 3.2m에서 시작** (우회 블렌드 / 리스폰 복귀). 옆 차로 정지 차량
    #     (새 경로 기준 lat +3.2 = 지금 ego 바로 앞)에 반응하지 않아야 한다 ―
    #     τ=0 예측은 실제 위치, 그 뒤는 경로로 수렴한다
    pol = TtcPolicy(path)
    log, _ = _drive_path(pol, path, lambda t, s: [Fake(13, 45.0, 3.2, dims=CAR)],
                         n=120, lat0=3.2, lat_decay_s=2.0)
    check('블렌드 시작: 옛 차로 정지 차량에 무반응', acts(log) == {'none'},
          f'{sorted(acts(log))}')
    # 13b. 같은 자리에서 새 경로 위 정지 차량은 잡는다
    pol = TtcPolicy(path)
    log, _ = _drive_path(pol, path, lambda t, s: [Fake(14, 45.0, 0.0, dims=CAR)],
                         n=120, lat0=3.2, lat_decay_s=2.0)
    check('블렌드 시작: 새 경로 정지 차량은 brake/stop', bool(acts(log) & {'brake', 'stop'}),
          f'{sorted(acts(log))}')

    # 14. 곡선 경로 ― 직선 외삽이면 틀렸을 세 경우
    arc = _Arc(radius=60.0)
    def off_curve(t, s):
        px, py = _point_at(arc, s)
        qx, qy = _point_at(arc, s + 1.0)
        th = math.atan2(qy - py, qx - px)
        return [Fake(15, px + 40.0 * math.cos(th), py + 40.0 * math.sin(th),
                     heading=th, dims=CAR)]
    pol = TtcPolicy(arc)
    log, _ = _drive_path(pol, arc, off_curve, n=100)
    check('곡선: 직선 연장선 위 주차 차량 무반응', acts(log) == {'none'},
          f'{sorted(acts(log))}')
    # 안쪽 옆 차로에서 같은 속도로 나란히 도는 차 (월드 직선 외삽이면 우리 차로로 들어온다)
    def inner_lane(t, s):
        so = s + 15.0
        px, py = _point_at(arc, so)
        qx, qy = _point_at(arc, so + 1.0)
        th = math.atan2(qy - py, qx - px)
        return [Fake(16, px - math.sin(th) * 3.5, py + math.cos(th) * 3.5,
                     speed=12.5, heading=th, dims=CAR)]
    pol = TtcPolicy(arc)
    log, _ = _drive_path(pol, arc, inner_lane, n=150)
    check('곡선: 안쪽 차로 나란히 도는 차 무반응', acts(log) == {'none'},
          f'{sorted(acts(log))}')
    # 40m 앞 **경로 위** 정지 차량 -> brake/stop (직선 외삽이면 13m 비켜가 놓친다)
    px, py = _point_at(arc, 40.0)
    qx, qy = _point_at(arc, 41.0)
    onpath = Fake(17, px, py, dims=CAR, heading=math.atan2(qy - py, qx - px))
    pol = TtcPolicy(arc)
    log, _ = _drive_path(pol, arc, lambda t, s: [onpath], n=200)
    check('곡선: 경로 위 정지 차량 brake/stop', bool(acts(log) & {'brake', 'stop'}),
          f'{sorted(acts(log))}')

    # 15. 종료 구간 라바콘은 후보가 아니다
    pol = TtcPolicy(path)
    gate = [Fake(18, 297.0, 0.3, dims=CONE), Fake(19, 297.0, -0.3, dims=CONE)]
    c = None
    for k in range(3):
        c = pol.update(270, 270.0, 12.5, 270.0, 0.0, 0.0, gate, k * 0.05)
    check('종료 라바콘: 후보 아님', c.obj_id == -1 and c.action == 'none', f'{c}')

    # 15b. 경로 끝 너머의 객체는 마지막 점에 투영돼 유령이 된다 -- 버린다
    pol = TtcPolicy(path)
    c = None
    for k in range(3):
        c = pol.update(295, 295.0, 5.0, 295.0, 0.0, 0.0,
                       [Fake(21, 340.0, 0.0, speed=8.0, dims=CAR)], k * 0.05)
    check('경로 끝 너머 앞차: 후보 아님', c.obj_id == -1, f'{c}')
    # 15c. 경로에서 35m 넘게 옆에 있는 차는 보지 않는다 (다른 도로)
    pol = TtcPolicy(path)
    log, _ = OB._drive(pol, path, lambda t, x: [Fake(22, x + 20.0, -44.0 + 3.0 * t,
                                                     speed=3.0, heading=math.pi / 2,
                                                     dims=CAR)], n=40)
    check('35m 밖 접근: 무반응', acts(log) == {'none'}, f'{sorted(acts(log))}')

    # 16. 정지 차량은 obstacles의 차로폭을 쓴다 ― lat 2.2 주차 차량에 서지 않는다
    pol = TtcPolicy(path)
    log, _ = OB._drive(pol, path, lambda t, x: [Fake(20, 40.0, 2.2, dims=CAR)], n=200)
    check('정지 차량 lat 2.2: 무반응 (obstacles 기준)', acts(log) == {'none'},
          f'{sorted(acts(log))}')

    # 17. 비용 ― 30객체 최악에서 프레임당 2ms 미만
    pol = TtcPolicy(path)
    many = [Fake(100 + k, 20.0 + 2.0 * k, (-1) ** k * 3.0, speed=5.0,
                 heading=0.3, dims=CAR) for k in range(30)]
    for k in range(3):
        pol.update(0, 0.0, 12.5, 0.0, 0.0, 0.0, many, k * 0.05)
    t0 = _time.perf_counter()
    for k in range(200):
        pol.update(0, 0.0, 12.5, 0.0, 0.0, 0.0, many, 1.0 + k * 0.05)
    per = (_time.perf_counter() - t0) / 200 * 1000.0
    check('비용: 30객체 프레임당 2ms 미만', per < 2.0, f'{per:.3f}ms')

    print()
    if fails:
        print(f'[ttc] 자기검사 {len(fails)}건 실패: {", ".join(fails)}')
        return 1
    print('[ttc] 자기검사 전부 통과')
    return 0


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(
        description='TTC 충돌 예측의 오프라인 재생 / 자기검사 (항목 14)')
    ap.add_argument('--replay', default='', help='세션 디렉터리')
    ap.add_argument('--path', default='', help='waypoint CSV')
    ap.add_argument('--segments', default='', help='쉼표로 구분한 구간 이름')
    ap.add_argument('--selftest', action='store_true',
                    help='로그에 없는 상황을 합성해 확인한다 (끼어들기·교차 진입 등)')
    args = ap.parse_args()
    if args.selftest:
        raise SystemExit(_selftest())
    if not (args.replay and args.path and args.segments):
        ap.error('--replay/--path/--segments를 함께 주거나 --selftest를 쓸 것')
    _replay(args.replay, args.path, args.segments.split(','))


if __name__ == '__main__':
    main()
