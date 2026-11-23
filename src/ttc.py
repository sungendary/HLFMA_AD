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

import ctrv
import obstacles as OB
from stoplines import FRONT_OVERHANG
from finish import FINISH_ZONE_M, gate_s

GAP_CRIT = 0.5              # m, 풋프린트 간격이 이 아래면 충돌 경로로 본다. 예전엔
                            # 중심 거리(D_CRIT 2.5)였는데 9/12 본대회에서 중심 거리
                            # 2.7~5m에 실제 접촉이 났다 ― 차체가 비스듬히 놓이면
                            # 중심 거리는 간격을 말해 주지 않는다
GAP_SLOPE = 0.25            # m/s, 교차·회전 상대의 판정 간격은 예측 시각에 비례해
                            # 넓힌다 (τ=4s에서 1.5m). 먼 미래일수록 예측이 틀린다
GAP_EXIT_ADD = 1.0          # m, 해제 = 전 표본이 판정 간격 + 이 값 밖
D_CRIT = GAP_CRIT           # 옛 이름 (--ttc-dcrit). 의미가 간격으로 바뀌었다
EGO_HALF_LEN = 2.4          # m, 차체 중심 -> 앞/뒤 끝 (앞 3.808 / 뒤 1.0)
EGO_HALF_W = 0.95           # m
REAR_M = 1.0                # m, 후륜축 -> 뒤범퍼
CORR_MARGIN_M = 0.5         # m, ego 주행 회랑 = 반폭 + 객체 횡 반폭 + 이 값
CROSS_MARGIN_M = 1.0        # m, 교차 상대의 회랑 진입점 앞에 앞범퍼를 이만큼 띄운다
ONCOMING_LAT_M = 2.5        # m, 마주오는 차가 이보다 옆이면 제 차로다
HOLD_LAT_M = 3.75           # m, 정지 유지(_hold)의 횡거리 한계
ROAD_YAW_TOL = 0.08         # rad/s, 요레이트가 도로 곡률과 이 안으로 맞으면 "도로를
ROAD_YAW_FRAC = 0.3         # 따라 도는 것"이다 (+ 곡률분의 이 비율)
PROJ_SLACK_M = OB.PROJ_SLACK_M
TRAJ_SPAN = 6               # 점, 등회전 표본을 직전 자리 둘레 이만큼만 투영한다
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
ACT_MARGIN_MS = 0.3         # m/s, 상한이 현재 속도보다 이만큼 낮아야 "실제 제한"
ACT_DECEL = 0.5             # m/s^2, 감속 피드포워드가 이보다 세도 "실제 제한"
REF_MARGIN_MS = 0.5         # m/s, 진입 속도가 지금보다 이만큼 높을 때만 다시 훑는다
TTC_MAX_HOLD_S = 10.0       # s, stop을 이만큼 유지하면 교착으로 보고 완화
RELEASE_FRAMES = 30         # 프레임, 위험이 사라진 채 이만큼 지속돼야 해제 (1.5s).
                            # 0.5s로는 제동으로 풀린 교차 충돌에 다시 가속해 2.2m로 스쳤다
HOLD_EXTRA_M = 2.0          # m, 정지 유지: 앞차 간격이 MIN_GAP+이 값 안일 때만
HOLD_LOOK_M = 20.0          # m, 정지 유지 판정 창

# 정지 중 판정 (9/12 본대회). 서 있을 때는 "지금 출발하면 부딪히는가"를 본다
DEPART_ACCEL = 1.5          # m/s^2, 출발 가속 가정
HOLD_CLEAR_S = 2.0          # s, 무위험이 이만큼 이어져야 출발
HOLD_MAX_S = 30.0           # s, 정지 유지 상한. 넘으면 틈에서 출발한다
HOLD_GAP_S = 2.0            # s, 상한 뒤 출발에 필요한 최소 틈 (가장 이른 위험까지)
V_PASS = 25.0 / 3.6         # m/s, 출발 뒤 통과 속도 가정 (junction_yield의 접근 상한과 같다)
HOLD_REARM_S = 5.0          # s, 상한으로 출발한 뒤 이만큼은 다시 잡지 않는다
V_DEPART = 3.0              # m/s, 이보다 느리면 "가속 중"으로 본다. 출발 직후
                            # 등속(v≈0) 예측은 우리가 교차점에 닿지 않는다고 계산한다

N_SAMPLES = int(round(T_WARN / TAU_STEP))
EXTRAP_SAMPLES = 20         # 창 끝에서 이만큼(4s)까지 꼭짓점을 외삽한다
T_RELEASE = 2.0 * T_WARN    # s, t_cpa가 이보다 크면 위험이 사라진 것 (해제 유예로)
LEVELS = {'': 0, 'warn': 1, 'brake': 2, 'stop': 3}


class TtcCmd:
    """한 프레임의 판정. obstacles.ObstacleCmd와 같은 모양이다."""

    __slots__ = ('v_limit', 'accel_limit', 'action', 'kind', 'obj_id',
                 't_cpa', 'd_cpa', 'reason', 'stop_avail')

    def __init__(self, v_limit=float('inf'), action='none', kind='',
                 obj_id=-1, t_cpa=float('nan'), d_cpa=float('nan'),
                 reason='', accel_limit=float('inf'),
                 stop_avail=float('nan')):
        self.v_limit = v_limit          # m/s. inf면 제약 없음
        self.accel_limit = accel_limit  # m/s^2 상한. inf면 제약 없음
        self.action = action            # none / warn / brake / stop / timeout
        self.kind = kind                # obstacle / pedestrian / vehicle
        self.obj_id = obj_id
        self.t_cpa = t_cpa              # s, 최근접까지 남은 시간
        self.d_cpa = d_cpa              # m, 최근접 거리
        self.reason = reason
        # 후륜축이 지금부터 몇 m 더 가서 설 작정인가 (obstacles.ObstacleCmd와
        # 같은 규약). brake/stop 단계, 그리고 **정지한 차로 내 객체의 warn**에
        # 싣는다 -- 그 경우 정지는 기하로 정해져 있어 crosswalks가 미리 봐도
        # 오탐이 아니다. 이동 객체의 warn·timeout은 nan
        self.stop_avail = stop_avail

    def __repr__(self) -> str:
        return (f'<TtcCmd {self.action} id={self.obj_id} v<={self.v_limit:.1f} '
                f't={self.t_cpa:.1f}s d={self.d_cpa:.1f}m>')


class _Tr:
    """최소 트랙. obstacles.Track과 같은 규칙으로 trusted를 낸다."""

    __slots__ = ('id', 'x', 'y', 'n_seen', 'missing', 'trusted', 'hist')

    def __init__(self, oid, x, y, heading=0.0):
        self.id = oid
        self.x, self.y = x, y
        self.n_seen = 1
        self.missing = 0
        self.trusted = False
        self.hist = ctrv.HeadingHist()      # 요레이트 추정용 (등회전 예측)
        self.hist.push(heading)


class _Act:
    """활성(위험) 객체 하나의 래치 상태."""

    __slots__ = ('id', 'enter', 'level', 'v_entry', 'stop_since', 'timed_out',
                 'announced', 'miss', 'last', 'told', 'v_ref')

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
        self.v_ref = 0.0            # 위험이던 동안의 최고 속도. 해제는 "이 속도로
                                    # 돌아가도 안전한가"로 본다

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


def _road_kappa(path, s: float, span: int = 3) -> float:
    """경로 s 지점의 부호 곡률 [1/m]. 좌회전이 양수."""
    pts, sarr = path.pts, path.s
    n = len(pts)
    i = max(span, min(n - 1 - span, bisect.bisect_right(sarr, s) - 1))
    a, b, c = pts[i - span], pts[i], pts[i + span]
    h1 = math.atan2(b[1] - a[1], b[0] - a[0])
    h2 = math.atan2(c[1] - b[1], c[0] - b[0])
    ds = 0.5 * (sarr[i + span] - sarr[i - span])
    return _wrap(h2 - h1) / ds if ds > 1e-6 else 0.0


_proj = OB.project_fix      # 선분 끝에 걸린 투영을 보정한 투영 (obstacles와 공용)


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


def _sweep(ego: list, obj: list, base: float, crossing: bool) -> tuple:
    """표본을 훑는다. ego[k] = (s, lat), obj[k] = (s, lat, 종 반폭, 횡 반폭).

    (첫 위반 k 또는 -1, 그 직전 여유, 그때 여유, 여유 최소 k, 최소 여유,
     그때 간격, 중심 거리 최소 k).
    """
    k_first, k_min, k_near = -1, 0, 0
    d2_min = m_min = m_prev = float('inf')
    m_before = m_first = g_at = float('inf')
    for k in range(len(ego)):
        es, el = ego[k]
        os_, ol, ext_lon, ext_lat = obj[k]
        ds, dl = os_ - es, ol - el
        d2 = ds * ds + dl * dl
        if d2 < d2_min:
            d2_min, k_near = d2, k
        g_lon = abs(ds) - ext_lon - EGO_HALF_LEN
        g_lat = abs(dl) - ext_lat - EGO_HALF_W
        g = g_lon if g_lon > g_lat else g_lat
        # 판정 간격은 T_WARN의 값(1.5m)에서 포화한다. 정지 중 hold는 8s까지 보는데
        # 거기까지 늘리면 2.5m라 옆 차로 교통이 걸린다
        m = g - (base + GAP_SLOPE * min(k, N_SAMPLES) * TAU_STEP
                 if crossing else base)
        if m < m_min:
            m_min, k_min, g_at = m, k, g
        if m < 0.0 and k_first < 0:
            k_first, m_before, m_first = k, m_prev, m
        m_prev = m
    return k_first, m_before, m_first, k_min, m_min, g_at, k_near


def gap_crit_at(base: float, tau: float, crossing: bool) -> float:
    """예측 시각 τ의 판정 간격. 교차·회전 상대는 먼 미래일수록 넓게 본다."""
    return base + GAP_SLOPE * tau if crossing else base


def footprint_gap(ds: float, dl: float, dh: float,
                  length: float, width: float) -> float:
    """경로 좌표계 풋프린트 간격 (분리축 근사). 음수면 겹친다.

    check_collision.gap_to와 같은 식이다. 거기는 ego 축(월드), 여기는 경로 축이라
    ds/dl이 곧 종/횡이다. ds·dl은 **차체 중심** 사이, dh는 객체 헤딩 - 경로 방위.
    """
    c, sn = abs(math.cos(dh)), abs(math.sin(dh))
    ext_lon = 0.5 * length * c + 0.5 * width * sn
    ext_lat = 0.5 * length * sn + 0.5 * width * c
    return max(abs(ds) - ext_lon - EGO_HALF_LEN, abs(dl) - ext_lat - EGO_HALF_W)


class _Cand:
    """한 프레임의 후보 객체 하나."""

    __slots__ = ('o', 'cls', 't', 'gap', 's_o', 'lat_o', 'v_lead', 'crossing',
                 'depart', 'danger', 'clear', 's_x', 't_x', 'a_dep', 'w', 'ext')

    def __init__(self, o, cls):
        self.o, self.cls = o, cls
        self.t = float('nan')       # 위험이면 첫 위반 시각, 아니면 여유 최소 시각
        self.gap = float('nan')     # 여유(간격 - 판정 간격)가 최소인 표본의 간격
        self.s_o = self.lat_o = 0.0
        self.v_lead = 0.0
        self.crossing = False       # 교차·회전 상대 (정지 목표·하한·판정 간격이 다르다)
        self.depart = False         # 출발 가속 가정의 대상
        self.danger = False         # 창 안에 위반이 있고 아직 다가오는 중
        self.clear = False          # 전 표본이 해제 간격 밖
        self.s_x = None             # 예측 궤적이 ego 회랑에 처음 드는 경로 s (객체 앞단)
        self.t_x = None             # 그 시각 [s]
        self.a_dep = 0.0
        self.w = 0.0                # 요레이트 [rad/s]
        self.ext = 0.0              # 객체의 경로 방향 반길이 (지금 헤딩)


class TtcPolicy:
    """매 프레임 속도 상한 하나를 낸다.

    트랙·래치는 월드 좌표라 경로와 무관하다. 경로가 바뀌면 rebind(path)만 한다.
    """

    def __init__(self, path, *, gap_crit: float = GAP_CRIT,
                 front_overhang: float = FRONT_OVERHANG,
                 finish_zone: float = FINISH_ZONE_M, on_event=None,
                 use_ctrv: bool = True):
        self.path = path
        self.gap_crit = gap_crit
        self.front = front_overhang
        self.finish_zone = finish_zone
        self.use_ctrv = use_ctrv
        self._gate_s = gate_s(path, finish_zone)
        self._on_event = on_event or (lambda kind, detail: None)
        self.reset()

    # -- 상태 -----------------------------------------------------------
    def reset(self) -> None:
        """리스폰 시. 위치가 강제로 옮겨졌으므로 트랙·래치를 전부 버린다."""
        self._tracks: dict = {}
        self._acts: dict = {}       # id -> _Act
        self._yielding = False
        self._hold_reset()
        self._hold_rearm = None     # 상한 출발 뒤 다시 잡기 시작할 시각
        self._last_t = None

    def _hold_reset(self) -> None:
        self._hold_on = False       # 정지 중 위험으로 출발을 막고 있다
        self._hold_enter = 0
        self._hold_since = None     # 상한 계산용 (양보 중에는 밀린다)
        self._hold_t0 = None        # 실제로 잡기 시작한 시각 (기록용)
        self._hold_clear = None     # 무위험이 시작된 시각
        self._hold_to = False       # 상한을 넘겼다
        self._hold_far = None       # hold 중 본 가장 먼 충돌 지점의 경로 s
        self._hold_best = 0.0       # hold 중 본 가장 긴 틈 [s] (가장 이른 위험까지)

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
                self._tracks[o.id] = _Tr(o.id, o.x, o.y, o.heading)
                continue
            tr.x, tr.y = o.x, o.y
            tr.n_seen += 1
            tr.missing = 0
            tr.trusted = tr.n_seen >= OB.TRUST_FRAMES
            tr.hist.push(o.heading)
        for oid in [k for k in self._tracks if k not in seen]:
            tr = self._tracks[oid]
            tr.missing += 1
            if tr.missing > OB.TRACK_GRACE:
                del self._tracks[oid]

    # -- 본체 -----------------------------------------------------------
    def update(self, idx: int, s_ego: float, v: float,
               ego_x: float, ego_y: float, ego_heading: float,
               objects, t: float, yielding: bool = False) -> TtcCmd:
        """yielding은 교차로 진입 양보 중(junction_yield)이다. 그동안은 정지 상한
        (TTC_MAX_HOLD_S / HOLD_MAX_S)을 세지 않는다 -- 교차 차량을 기다리는 중에
        완화하면 그 차들 사이로 기어 들어간다."""
        self._yielding = yielding
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        self._update_tracks(objects)
        cmd = TtcCmd()
        path = self.path
        s_e, lat_e, _th = OB._project(path, max(0, idx - 1),
                                      min(idx + 2, len(path.pts) - 1),
                                      ego_x, ego_y)
        if abs(lat_e) > OFF_MAX_M:
            self._acts.clear()
            self._hold_reset()
            cmd.reason = '경로 밖'
            return cmd
        if v < EGO_MIN_V:
            # 정지 상태. 서 있는 객체는 obstacles가 맡으므로 stop 래치만 "그 객체가
            # 아직 코앞에 있는가"로 유지한다. **가로지르거나 다가오는 이동 객체는
            # "지금 출발하면 부딪히는가"로 계속 본다** ― 9/12 본대회에서 정지할
            # 때마다 판정이 꺼져 교차 차량 행렬 속으로 재출발했다
            cmd = self._hold(cmd, idx, s_e, ego_heading, objects, t)
            return self._depart(cmd, idx, s_e, lat_e, ego_x, ego_y, ego_heading,
                                objects, t, dt)
        self._hold_reset()
        # 출발 직후(저속)에는 가로지르는 상대에 한해 가속을 가정한다. 상한으로
        # 출발한 직후(rearm)에는 넣지 않는다 -- 넣으면 다시 서서 교착이 된다
        if self._hold_rearm is not None and t >= self._hold_rearm:
            self._hold_rearm = None
        a_dep = (DEPART_ACCEL if v < V_DEPART and self._hold_rearm is None
                 else 0.0)
        cands, best_log = self._scan(idx, s_e, lat_e, v, a_dep, ego_x, ego_y,
                                     ego_heading, objects, False)
        return self._decide(cmd, cands, best_log, s_e, v, t)

    def _traj(self, o, w: float, s_o: float, i_lo: int, i_hi: int,
              n: int = N_SAMPLES) -> list:
        """등회전 예측 점열을 경로 좌표계로. [(s, lat, 헤딩 - 경로 방위)].

        표본마다 전 창을 투영하면 비싸다. 직전 표본의 자리 둘레만 본다 ―
        0.2초에 움직이는 거리는 몇 m다.
        """
        path = self.path
        pts, sarr = path.pts, path.s
        top = max(i_lo + 1, min(i_hi, len(pts) - 1))
        i = max(i_lo, min(top, bisect.bisect_right(sarr, s_o) - 1))
        out = []
        for k in range(n + 1):
            x, y, h = ctrv.point(o.x, o.y, o.heading, o.speed, w, k * TAU_STEP)
            best, bi = None, i
            for j in range(max(i_lo, i - TRAJ_SPAN), min(top, i + TRAJ_SPAN) + 1):
                px, py = pts[j]
                d2 = (px - x) * (px - x) + (py - y) * (py - y)
                if best is None or d2 < best:
                    best, bi = d2, j
            s, lat, th = _proj(path, max(i_lo, bi - 1), min(top, bi + 1), x, y)
            out.append((s, lat, _wrap(h - th)))
            i = bi
        return out

    def _scan(self, idx: int, s_e: float, lat_e: float, v: float, a_dep: float,
              ego_x: float, ego_y: float, ego_heading: float, objects,
              depart_only: bool, n: int = N_SAMPLES) -> tuple:
        """후보를 뽑는다. (id -> _Cand, 로깅용 가장 이른 후보).

        n은 표본 수(시간창 = n x TAU_STEP). 정지 중 hold만 T_WARN보다 길게 본다 ―
        그때 ego의 출발 가속은 V_PASS에서 포화시킨다.

        a_dep(출발 가속 가정)은 가로지르거나 다가오는 **이동** 객체에만 쓴다.
        depart_only면 정지 중이라 그런 객체만 본다 (정지 객체·같은 방향 앞차는
        _hold의 몫).
        """
        path = self.path
        s_c = s_e + EGO_CENTER_M            # 차체 중심
        ch, sh = math.cos(ego_heading), math.sin(ego_heading)
        t_win = n * TAU_STEP
        long_win = n > N_SAMPLES
        t_cap = (max(0.0, V_PASS - v) / a_dep) if (long_win and a_dep > 0.0) \
            else float('inf')

        def dep(tau):
            if tau <= t_cap:
                return v * tau + 0.5 * a_dep * tau * tau
            return (v * t_cap + 0.5 * a_dep * t_cap * t_cap
                    + max(v, V_PASS) * (tau - t_cap))

        horizon = v * t_win + PREFILTER_M
        if a_dep > 0.0:
            horizon = dep(t_win) + PREFILTER_M
        i_lo, i_hi = OB._window(path, idx, BEHIND_M + 3.0, horizon)
        # ego 예측. 등속 / 출발 가속 두 벌을 프레임당 한 번만 만든다
        ego_cv, ego_dep = [], []
        for k in range(n + 1):
            tau = k * TAU_STEP
            wd = 1.0 - tau / OFF_DECAY_S
            el = lat_e * wd if wd > 0.0 else 0.0
            ego_cv.append((s_c + v * tau, el))
            ego_dep.append((s_c + dep(tau), el))
        base = self.gap_crit
        cands = {}
        best_log = None
        for o in objects:
            dx, dy = o.x - ego_x, o.y - ego_y
            if math.hypot(dx, dy) > horizon + abs(o.speed) * t_win:
                continue
            tr = self._tracks.get(o.id)
            if tr is None or not tr.trusted:
                continue
            if dx * ch + dy * sh < -BEHIND_M:
                continue
            cls = OB.classify(o)
            s_o, lat_o, th_p = _proj(path, i_lo, i_hi, o.x, o.y)
            if cls == OB.OBSTACLE and s_o >= self._gate_s:
                continue            # 종료 라바콘. obstacles와 같은 기준
            if abs(lat_o) > LAT_MAX_M:
                continue            # 다른 도로
            if s_o >= path.length - 1.0:
                continue            # 경로 끝 너머 -- 마지막 점에 붙어 유령이 된다
            moving = abs(o.speed) >= STATIONARY_MS
            if not moving and cls == OB.VEHICLE and abs(lat_o) >= OB.VEH_LANE_M:
                continue            # obstacles가 무시하는 주차 차량은 우리도 무시
            dh = _wrap(o.heading - th_p)
            cos_dh = math.cos(dh)
            # 등회전은 차량에만 쓴다. 보행자의 헤딩은 걸음마다 흔들린다
            w = (tr.hist.rate(o.speed)
                 if (self.use_ctrv and moving and cls == OB.VEHICLE) else 0.0)
            v_along = o.speed * cos_dh
            v_lat = o.speed * math.sin(dh)
            if w != 0.0:
                # 굽은 도로를 따라 도는 차는 회전 차량이 아니다. 경로 좌표계 외삽이
                # 이미 곡률을 따라간다 ― 거기에 등회전까지 얹으면 굽은 길의 대향차가
                # 우리 차로로 꺾어 들어온다고 계산된다 (sim rg2_pt2_oncoming).
                # 같은 길을 가는 차의 요레이트는 (경로 방향 속도) x (경로 곡률)이다
                w_road = v_along * _road_kappa(path, s_o)
                if abs(w - w_road) <= ROAD_YAW_TOL + ROAD_YAW_FRAC * abs(w_road):
                    w = 0.0
            if cos_dh < ONCOMING_COS and w == 0.0 \
                    and abs(lat_o) >= ONCOMING_LAT_M:
                continue            # 제 차로로 마주오는 차
            # 같은 방향으로 나란히 가거나 멀어지는 것, 서 있는 것은 출발 가정의
            # 대상이 아니다
            depart = moving and (abs(dh) >= MERGE_RAD or w != 0.0
                                 or lat_o * v_lat < 0.0)
            if depart_only and not depart:
                continue
            # 교차·회전 상대. 마주오는 차(120도 이상)는 제 차로를 따라오는 것이라
            # 교차가 아니다
            crossing = moving and (w != 0.0 or (abs(dh) >= MERGE_RAD
                                                and cos_dh >= ONCOMING_COS))
            settle = abs(dh) < MERGE_RAD and lat_o * v_lat < 0.0
            ego = ego_dep if (depart and a_dep > 0.0) else ego_cv
            traj = self._traj(o, w, s_o, i_lo, i_hi, n) if w != 0.0 else None
            hl, hw = 0.5 * o.length, 0.5 * o.width
            c0, s0 = abs(cos_dh), abs(math.sin(dh))
            ext_now = hl * c0 + hw * s0
            # 객체 궤적 (경로 좌표계 위치 + 그 헤딩의 종/횡 반폭). ego 속도와 무관
            sign = 1.0 if lat_o >= 0.0 else -1.0
            obj = []
            s_x = t_x = None
            for k in range(n + 1):
                tau = k * TAU_STEP
                if traj is None:
                    os_ = s_o + v_along * tau
                    ol = lat_o + v_lat * tau
                    if settle and ol * sign < 0.0:
                        ol = 0.0
                    ext_lon, ext_lat = ext_now, hl * s0 + hw * c0
                else:
                    os_, ol, dhk = traj[k]
                    ck, sk = abs(math.cos(dhk)), abs(math.sin(dhk))
                    ext_lon = hl * ck + hw * sk
                    ext_lat = hl * sk + hw * ck
                obj.append((os_, ol, ext_lon, ext_lat))
                if crossing and s_x is None and os_ - ext_lon > s_e - REAR_M \
                        and abs(ol) < EGO_HALF_W + ext_lat + CORR_MARGIN_M:
                    s_x, t_x = os_ - ext_lon, tau
            (k_first, m_before, m_first, k_min, m_min, g_at,
             k_near) = _sweep(ego, obj, base, crossing)
            # 활성 래치는 **위험이던 때의 속도로 돌아가도 안전한가**로 해제를 본다.
            # 지금 속도만 보면 감속 -> 위반이 창 밖 -> 해제 -> 재가속 -> 다시 위험의
            # 톱니가 난다
            act = self._acts.get(o.id)
            ref_clear = True
            if act is not None and act.active and act.v_ref > v + REF_MARGIN_MS:
                ego_ref = [(s_c + act.v_ref * k * TAU_STEP, ego[k][1])
                           for k in range(n + 1)]
                r = _sweep(ego_ref, obj, base, crossing)
                ref_clear = r[4] >= GAP_EXIT_ADD or r[6] == 0
                if k_near == 0 and r[6] != 0:
                    k_near, k_first = r[6], -1      # 후보로 남기되 위험은 아니다
            if k_near == 0:
                continue            # 중심 거리가 지금 최소 = 멀어지는 중
            c = _Cand(o, cls)
            c.s_o, c.lat_o = s_o, lat_o
            c.crossing, c.depart, c.w = crossing, depart, w
            c.a_dep = a_dep if (depart and a_dep > 0.0) else 0.0
            c.v_lead = 0.0 if crossing else max(0.0, v_along)
            c.gap = g_at
            c.clear = m_min >= GAP_EXIT_ADD and ref_clear
            c.s_x, c.t_x = s_x, t_x
            c.ext = ext_now
            if k_first >= 0:
                c.danger = True
                if k_first == 0:
                    c.t = 0.0
                else:
                    # 표본 사이에서 여유가 0을 지나는 시각 (선형 보간)
                    den = m_before - m_first
                    frac = m_before / den if den > 1e-9 else 1.0
                    c.t = (k_first - 1 + max(0.0, min(1.0, frac))) * TAU_STEP
            else:
                c.t = k_min * TAU_STEP
            cands[o.id] = c
            if c.danger and (best_log is None or not best_log.danger
                             or c.t < best_log.t):
                best_log = c
            elif best_log is None or (not best_log.danger and c.gap < best_log.gap):
                best_log = c
        return cands, best_log

    def _avail(self, c: '_Cand', s_e: float, v: float) -> float:
        """후륜축이 더 갈 수 있는 거리.

        교차·회전 상대는 **예측 궤적이 ego 회랑에 처음 드는 자리** 앞에 선다.
        객체의 지금 종거리로 재면 옆에서 꺾여 들어오는 상대에 여유가 크게 잡혀
        그 궤적 안까지 들어간다 (9/12 본대회). 앞차 추종은 예전 그대로
        앞차 뒤범퍼 MIN_GAP 앞이다.
        """
        o = c.o
        if c.crossing and c.s_x is not None:
            return c.s_x - self.front - CROSS_MARGIN_M - s_e
        half_l = c.ext
        avail_now = (c.s_o - half_l) - (s_e + self.front) - MIN_GAP
        travel = v * c.t + 0.5 * c.a_dep * c.t * c.t
        avail_hit = travel + self.gap_crit - MIN_GAP
        return min(avail_now, avail_hit)

    def _decide(self, cmd: TtcCmd, cands: dict, best_log, s_e: float, v: float,
                t: float) -> TtcCmd:
        """주행 중. 후보로 래치를 갱신하고 상한을 낸다."""
        # 래치 갱신. 해제는 "후보에서 빠졌거나 확실히 벗어난 채 RELEASE_FRAMES
        # 지속"이다 ― 제동으로 위반이 창 밖으로 밀린 것이나 한 프레임 닫힘이 멈춘
        # 것으로 풀면 핑퐁이 난다
        for oid in list(self._acts):
            a = self._acts[oid]
            c = cands.get(oid)
            if c is None or c.clear:
                if not a.active:
                    del self._acts[oid]     # 확인 중이었는데 빠졌다
                    continue
                a.miss += 1
                if a.miss > RELEASE_FRAMES:
                    self._release(oid)
                continue
            a.miss = 0
        for oid, c in cands.items():
            if c.danger:
                a = self._acts.get(oid)
                if a is None:
                    a = self._acts[oid] = _Act(oid)
                a.enter += 1
                if v > a.v_ref:
                    a.v_ref = v

        # 활성 객체마다 상한을 내고 min(). 유예 중(miss>0)이거나 위반이 창 밖으로
        # 밀린 것은 **마지막 상한을 유지한다** ― 풀어 버리면 재가속 -> 다시 위험 ->
        # 제동의 토글이 난다
        worst = None
        for oid, a in self._acts.items():
            if not a.active:
                continue
            c = cands.get(oid)
            if a.miss > 0 or c is None or not c.danger:
                if a.last is None:
                    continue
                v_lim, a_lim, t_cpa, d_cpa, cls, sa = a.last
                if c is not None and a.miss == 0 and not a.timed_out \
                        and a.level in ('brake', 'stop'):
                    # 위반이 창 밖으로 밀렸지만 아직 벗어나지 않았다. 마지막 상한을
                    # 얼려 두면 앞차가 떠난 뒤에도 그 속도에 묶인다 -- 지금 기하로
                    # 다시 낸다 (앞차면 추종 법칙, 교차 상대면 회랑 진입점 앞 정지)
                    if c.crossing and c.s_x is not None:
                        avail = c.s_x - self.front - CROSS_MARGIN_M - s_e
                        v_lim, a_lim = _limit(avail, v, 0.0, 0.0)
                    elif not c.crossing:
                        avail = (c.s_o - c.ext) - (s_e + self.front) - MIN_GAP
                        v_lim, a_lim = _limit(
                            avail, v, c.v_lead,
                            0.0 if a.level == 'stop' else V_FLOOR)
                    sa = max(0.0, avail) if (c.crossing and c.s_x is not None) \
                        or not c.crossing else sa
                    a.last = (v_lim, a_lim, t_cpa, d_cpa, cls, sa)
                key = (v_lim, t_cpa)
                if worst is None or key < worst[0]:
                    worst = (key, oid, cls, t_cpa, d_cpa, a.level, v_lim,
                             a_lim, sa)
                continue
            o, cls, t_cpa, d_cpa = c.o, c.cls, c.t, c.gap
            avail = self._avail(c, s_e, v)
            if t_cpa <= T_IMMINENT:
                level = 'stop'
            elif t_cpa <= T_CRIT:
                level = 'brake'
            else:
                level = 'warn'
            if LEVELS[level] < LEVELS[a.level]:
                level = a.level         # 활성 중에는 내려가지 않는다
            # 교차·회전 상대에는 하한이 없다. 10km/h로 기어 들어가는 자리가 바로
            # 상대의 궤적이다. 앞차 추종의 하한은 그대로 둔다
            floor = 0.0 if c.crossing else V_FLOOR
            if level == 'stop':
                v_lim, a_lim = _limit(avail, v, 0.0, 0.0)
            elif level == 'brake':
                v_lim, a_lim = _limit(avail, v, c.v_lead, floor)
            else:
                if a.level != 'warn':
                    a.v_entry = v
                if c.crossing:
                    pv, pa = _limit(avail, v, 0.0, 0.0)
                    v_lim = min(WARN_FRAC * a.v_entry, pv)
                    a_lim = pa if pv <= WARN_FRAC * a.v_entry else float('inf')
                else:
                    v_lim = max(WARN_FRAC * a.v_entry, V_FLOOR)
                    a_lim = float('inf')
            # 상한이 속도 아래이거나 감속 피드포워드가 걸려 있으면 실제 제한이다.
            # 피드포워드가 속도를 프로파일에 붙여 끌고 내려오면 v_lim과 v의 차가
            # 끝까지 ACT_MARGIN_MS를 안 넘는다 (45 -> 16km/h로 줄이고도 이벤트 0)
            effective = v_lim < v - ACT_MARGIN_MS or a_lim < -ACT_DECEL
            if level == 'stop' and v_lim <= 0.01:
                if a.stop_since is None:
                    a.stop_since = t
                self._check_timeout(a, cls, t, t_cpa, d_cpa)
            # 정지 의사 (crosswalks 예방용). 정지한 차로 내 객체는 warn에서
            # 이미 정지점이 확정이다 -- 서행 중에는 warn(4s)이 너무 늦다
            sa = float('nan')
            if not a.timed_out and (
                    level in ('brake', 'stop')
                    or (abs(o.speed) < STATIONARY_MS
                        and abs(c.lat_o) <= OB.VEH_LANE_M)):
                sa = max(0.0, avail)
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
                        f'앞 {c.s_o - s_e:.1f}m lat={c.lat_o:+.1f}m '
                        f'v_obj={o.speed * 3.6:.0f}km/h '
                        + (f'회전 {c.w:+.2f}rad/s ' if c.w else '')
                        + (f'교차 정지여유 {avail:.1f}m ' if c.crossing else '')
                        + f'v {v * 3.6:.1f} -> <={v_lim * 3.6:.1f}km/h')
                    a.announced = True
                    a.told = level
            a.level = level
            a.last = (v_lim, a_lim, t_cpa, d_cpa, cls, sa)
            key = (v_lim, t_cpa)
            if worst is None or key < worst[0]:
                worst = (key, oid, cls, t_cpa, d_cpa, shown, v_lim, a_lim, sa)

        if worst is not None:
            _k, oid, cls, t_cpa, d_cpa, shown, v_lim, a_lim, sa = worst
            cmd.v_limit, cmd.accel_limit = v_lim, a_lim
            cmd.stop_avail = sa
            cmd.action, cmd.kind, cmd.obj_id = shown, cls, oid
            cmd.t_cpa, cmd.d_cpa = t_cpa, d_cpa
            cmd.reason = f'{cls} id={oid} t={t_cpa:.1f}s 간격={d_cpa:.1f}m'
        elif best_log is not None:
            cmd.kind, cmd.obj_id = best_log.cls, best_log.o.id
            cmd.t_cpa, cmd.d_cpa = best_log.t, best_log.gap
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
            s_o, lat_o, _th = _proj(self.path, i_lo, i_hi, o.x, o.y)
            dh = _wrap(o.heading - ego_heading)
            half_l = (0.5 * o.length * abs(math.cos(dh))
                      + 0.5 * o.width * abs(math.sin(dh)))
            gap = (s_o - half_l) - (s_e + self.front)
            # 정지 차량은 _scan과 같은 기준(obstacles의 차로폭)으로 놓는다. 우회로
            # 경로가 바뀌면 막던 차가 횡 3m 옆이 되는데, 넓은 기준으로 잡고 있으면
            # 우회 출발이 정지 상한(10s)까지 묶인다 (sim rgj_pt2_many6)
            lat_max = (OB.VEH_LANE_M
                       if (abs(o.speed) < STATIONARY_MS
                           and OB.classify(o) == OB.VEHICLE) else HOLD_LAT_M)
            if abs(lat_o) >= lat_max or gap > MIN_GAP + HOLD_EXTRA_M \
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
                cmd.stop_avail = (float('nan') if a.timed_out
                                  else max(0.0, gap - MIN_GAP))
        for oid in list(self._acts):
            if oid not in keep:
                self._release(oid)
        return cmd

    def _depart(self, cmd: TtcCmd, idx: int, s_e: float, lat_e: float,
                ego_x: float, ego_y: float, ego_heading: float, objects,
                t: float, dt: float) -> TtcCmd:
        """정지 중. 1.5m/s²로 출발한다고 보고 위험이면 출발을 막는다(hold).

        래치는 객체가 아니라 **정책에** 건다. 행렬이 지나가는 동안 대상 id가
        계속 바뀌는데 객체마다 걸면 차 사이마다 풀린다. 무위험이
        HOLD_CLEAR_S 이어져야 해제한다.

        **hold가 걸린 뒤의 시간창은 T_WARN(4s)이 아니라 ego가 충돌 지점을 빠져나가는
        시간 기준이다** (junction_yield와 같은 ctrv.horizon). 4s로는 4.9s 간격 행렬의
        차 사이마다 "무위험 2s"가 성립해 풀렸다 다시 걸렸다. 교차로를 모르므로
        통과 거리는 hold 중 본 가장 먼 충돌 지점(회랑 진입점 + 상대 차체)으로 잰다.
        진입 판정은 4s 그대로다.
        """
        n = N_SAMPLES
        tc = 0.0
        if self._hold_on and self._hold_far is not None:
            tc = ctrv.t_clear(0.0, max(0.0, self._hold_far + REAR_M - s_e),
                              V_PASS, DEPART_ACCEL)
            n = max(N_SAMPLES, int(round(ctrv.horizon(tc) / TAU_STEP)))
        cands, _log = self._scan(idx, s_e, lat_e, 0.0, DEPART_ACCEL, ego_x,
                                 ego_y, ego_heading, objects, True, n)
        danger = sorted(((c.t, oid) for oid, c in cands.items() if c.danger))
        if self._hold_on and self._hold_far is not None:
            # 걸린 뒤에는 junction_yield와 같은 잣대다: 시간창 안에 그 충돌 지점으로
            # 들어오는 교차 상대가 하나라도 있으면 위협이다. "우리가 지나간 뒤에
            # 온다"는 계산(간격 훑기)에 기대면 통과 시간의 여유(1.5s)가 없다
            seen = {oid for _t, oid in danger}
            danger = sorted(danger + [
                (c.t_x, oid) for oid, c in cands.items()
                if oid not in seen and c.crossing and c.s_x is not None
                and c.s_x <= self._hold_far])
        for _t, oid in danger:
            c = cands[oid]
            if not c.danger:
                continue
            far = (c.s_x if c.s_x is not None else c.s_o - c.ext) + 2.0 * c.ext
            if self._hold_far is None or far > self._hold_far:
                self._hold_far = far
        if danger and danger[0][0] > self._hold_best:
            self._hold_best = danger[0][0]
        # 상한 뒤 출발에 필요한 틈. 통과 시간만큼 비면 좋지만, 행렬 간격이 그보다
        # 짧으면 영영 못 간다 ― 그때는 본 것 중 가장 긴 틈의 80%로 낮춘다
        need_gap = max(HOLD_GAP_S, min(tc, 0.8 * self._hold_best))
        if self._hold_rearm is not None and t >= self._hold_rearm:
            self._hold_rearm = None
        if not self._hold_on:
            self._hold_enter = self._hold_enter + 1 if danger else 0
            if self._hold_enter < ENTER_FRAMES or self._hold_rearm is not None:
                return cmd
            self._hold_on = True
            self._hold_since = self._hold_t0 = t
            self._hold_clear = None
            self._hold_to = False
            c = cands[danger[0][1]]
            self._on_event(
                'ttc_hold',
                f'id={c.o.id} {c.cls} 정지 중 ― 지금 출발하면 t_cpa={c.t:.1f}s '
                f'd_cpa={c.gap:.2f}m (앞 {c.s_o - s_e:.1f}m lat={c.lat_o:+.1f}m '
                f'v_obj={c.o.speed * 3.6:.0f}km/h'
                + (f' 회전 {c.w:+.2f}rad/s' if c.w else '')
                + f', 위험 {len(danger)}대) ― 출발 보류')
        if danger:
            self._hold_clear = None
        else:
            if self._hold_clear is None:
                self._hold_clear = t
            if t - self._hold_clear >= HOLD_CLEAR_S:
                self._on_event('ttc_hold_clear',
                               f'시간창 {n * TAU_STEP:.1f}s 안 무위험 '
                               f'{HOLD_CLEAR_S:.0f}s ― 정지 '
                               f'{t - self._hold_t0:.1f}s 뒤 출발')
                self._hold_reset()
                return cmd
        if self._yielding:
            self._hold_since += dt      # 교차로 양보 중에는 상한을 세지 않는다
        elif t - self._hold_since > HOLD_MAX_S:
            if not self._hold_to:
                self._hold_to = True
                self._on_event(
                    'ttc_hold_timeout',
                    f'정지 유지 {HOLD_MAX_S:.0f}s 초과 ― 가장 이른 위험까지 '
                    f'{need_gap:.1f}s 이상 비면 출발한다')
            if danger and danger[0][0] >= need_gap:
                self._on_event('ttc_hold_clear',
                               f'상한 뒤 틈 {danger[0][0]:.1f}s ― 출발')
                self._hold_reset()
                self._hold_rearm = t + HOLD_REARM_S
                return cmd
        cmd.v_limit = 0.0
        cmd.action = 'hold'
        cmd.stop_avail = 0.0
        if danger:
            c = cands[danger[0][1]]
            cmd.kind, cmd.obj_id = c.cls, c.o.id
            cmd.t_cpa, cmd.d_cpa = danger[0][0], c.gap
            cmd.reason = f'출발 보류 ― {c.cls} id={c.o.id} t={danger[0][0]:.1f}s'
        else:
            cmd.reason = f'출발 보류 ― 무위험 {t - self._hold_clear:.1f}s'
        return cmd

    def _check_timeout(self, a: '_Act', cls: str, t: float,
                       t_cpa: float, d_cpa: float) -> None:
        if a.timed_out or a.stop_since is None:
            return
        if self._yielding:
            a.stop_since = t        # 양보 중에는 세지 않는다
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
        f'  예측: ego는 경로 위 s+v·τ (저속·정지 중 교차 상대에는 '
        f'{DEPART_ACCEL:.1f}m/s² 출발 가정), 객체는 경로 좌표계 (along, lat) 외삽, '
        f'요레이트 {ctrv.YAW_MIN:.2f}rad/s 이상이면 등회전'
        + ('' if policy.use_ctrv else ' (꺼짐)') + '. '
        f'τ 0~{T_WARN:.0f}s {TAU_STEP:.1f}s 간격',
        f'  위험: 풋프린트 간격<{policy.gap_crit:.1f}m (교차·회전 상대는 '
        f'+{GAP_SLOPE:.2f}·τ, 정지 차량은 횡 {OB.VEH_LANE_M:.1f}m 안만). '
        f'warn>{T_CRIT:.0f}s ({WARN_FRAC:.0%}) / '
        f'brake<={T_CRIT:.0f}s (앞차 추종 하한 {V_FLOOR * 3.6:.0f}km/h, 교차는 하한 '
        f'없음·정지점은 회랑 진입점 {CROSS_MARGIN_M:.0f}m 앞) / stop<={T_IMMINENT:.0f}s',
        f'  정지 중: 교차 상대에 출발 보류(hold), 무위험 {HOLD_CLEAR_S:.0f}s 뒤 해제, '
        f'상한 {HOLD_MAX_S:.0f}s',
        f'  게이트: trusted 트랙만, 뒤 {BEHIND_M:.0f}m 제외, 제 차로 대향차 제외, '
        f'진입 {ENTER_FRAMES}프레임 / 해제 +{GAP_EXIT_ADD:.1f}m, '
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

    # 8. ego 정지 + 정지 객체 -> none (서 있는 객체는 obstacles의 몫)
    pol = TtcPolicy(path)
    for k in range(4):
        cmd = pol.update(0, 0.0, 0.0, 0.0, 0.0, 0.0, [Fake(8, 3.0, 0.0, dims=CAR)], k * 0.05)
    check('ego 정지 + 정지 객체: 무반응',
          cmd.action == 'none' and cmd.v_limit == float('inf'))
    # 8b. ego 정지 + 같은 방향 앞차 출발 -> none
    pol = TtcPolicy(path)
    for k in range(40):
        cmd = pol.update(0, 0.0, 0.0, 0.0, 0.0, 0.0,
                         [Fake(8, 9.0 + 2.0 * k * 0.05, 0.0, speed=2.0, dims=CAR)],
                         k * 0.05)
    check('ego 정지 + 출발하는 앞차: 무반응', cmd.action == 'none', f'{cmd}')
    # 8c. ego 정지 + 앞을 가로지르는 차 -> hold, 지나간 뒤 2초에 해제
    ev = []
    pol = TtcPolicy(path, on_event=lambda k, d: ev.append(k))
    seq = []
    for k in range(200):
        t = k * 0.05
        y = -30.0 + 8.33 * t
        objs = ([Fake(8, 10.0, y, speed=8.33, heading=math.pi / 2, dims=CAR)]
                if y < 40.0 else [])
        cmd = pol.update(0, 0.0, 0.0, 0.0, 0.0, 0.0, objs, t)
        seq.append((t, cmd.action, cmd.v_limit))
    held = [r for r in seq if r[1] == 'hold']
    check('ego 정지 + 교차 차량: hold', bool(held) and all(r[2] == 0.0 for r in held)
          and 'ttc_hold' in ev, f'{len(held)}프레임 {",".join(ev)}')
    # 해제 = "지금 출발해도 안 부딪힌다"가 2초 이어진 시점. 그때 실제로 출발시켜
    # 풋프린트 간격을 잰다
    t_rel = held[-1][0] + 0.05 if held else 0.0
    gmin = float('inf')
    for k in range(120):
        tau = k * 0.05
        ex = 0.5 * DEPART_ACCEL * tau * tau + EGO_CENTER_M
        y = -30.0 + 8.33 * (t_rel + tau)
        gmin = min(gmin, max(abs(10.0 - ex) - 0.905 - 2.4, abs(y) - 2.195 - 0.95))
    check('ego 정지 + 교차 차량: 2s 이상 잡고, 해제 뒤 출발해도 간격 유지',
          held and t_rel - held[0][0] >= HOLD_CLEAR_S and gmin > 0.5
          and 'ttc_hold_clear' in ev,
          f'hold {held[0][0] if held else 0:.2f}~{t_rel:.2f}s, 출발 시 최소 간격 {gmin:.2f}m')
    # 8d. 행렬 5대 (3s 간격) -> id가 바뀌어도 hold가 끊기지 않는다
    ev = []
    pol = TtcPolicy(path, on_event=lambda k, d: ev.append(k))
    seq = []
    for k in range(500):
        t = k * 0.05
        objs = []
        for i in range(5):
            y = -30.0 + 8.33 * (t - 3.0 * i)
            if -30.0 <= y < 40.0:
                objs.append(Fake(80 + i, 10.0, y, speed=8.33,
                                 heading=math.pi / 2, dims=CAR))
        cmd = pol.update(0, 0.0, 0.0, 0.0, 0.0, 0.0, objs, t)
        seq.append((t, cmd.action, cmd.obj_id))
    on = [r[0] for r in seq if r[1] == 'hold']
    gaps = [b - a for a, b in zip(on, on[1:]) if b - a > 0.06]
    ids = {r[2] for r in seq if r[1] == 'hold' and r[2] >= 0}
    check('행렬 5대: hold가 id 교체에도 끊기지 않는다',
          on and not gaps and len(ids) >= 4 and ev.count('ttc_hold') == 1,
          f'hold {on[0] if on else 0:.1f}~{on[-1] if on else 0:.1f}s, 끊김 {len(gaps)}, '
          f'id {sorted(ids)}, {",".join(ev)}')
    # 8d2. 행렬 간격 4.9s (9/12 본대회) -> 차 사이에서 풀리지 않는다. 4s 고정
    #      시간창이면 차 사이마다 "무위험 2s"가 성립했다. 간격 10s면 틈에 풀린다
    for gap_s, want in ((4.9, 1), (10.0, 3)):
        ev = []
        pol = TtcPolicy(path, on_event=lambda k, d: ev.append(k))
        t_last = 0.0
        for k in range(1100):
            t = k * 0.05
            objs = []
            for i in range(3 if gap_s > 5.0 else 5):
                y = -30.0 + 8.33 * (t - gap_s * i)
                if -30.0 <= y < 40.0:
                    objs.append(Fake(80 + i, 10.0, y, speed=8.33,
                                     heading=math.pi / 2, dims=CAR))
                    if y < 5.0:
                        t_last = t
            pol.update(0, 0.0, 0.0, 0.0, 0.0, 0.0, objs, t)
        check(f'행렬 간격 {gap_s:.1f}s: ttc_hold_clear {want}회'
              + (' (마지막 차 뒤에만)' if want == 1 else ' (틈마다)'),
              ev.count('ttc_hold_clear') == want and ev.count('ttc_hold') == want,
              ','.join(ev))
    # 8e. 끝나지 않는 행렬 -> 상한 뒤 틈에서 출발
    ev = []
    pol = TtcPolicy(path, on_event=lambda k, d: ev.append(k))
    seq = []
    for k in range(900):
        t = k * 0.05
        objs = []
        for i in range(16):
            y = -30.0 + 8.33 * (t - 3.0 * i)
            if -30.0 <= y < 40.0:
                objs.append(Fake(80 + i, 10.0, y, speed=8.33,
                                 heading=math.pi / 2, dims=CAR))
        cmd = pol.update(0, 0.0, 0.0, 0.0, 0.0, 0.0, objs, t)
        seq.append((t, cmd.action))
        if cmd.action != 'hold' and t > HOLD_MAX_S:
            break
    check('끝나지 않는 행렬: ttc_hold_timeout 뒤 틈 출발',
          'ttc_hold_timeout' in ev and seq[-1][1] != 'hold'
          and HOLD_MAX_S < seq[-1][0] < HOLD_MAX_S + 6.0,
          f't={seq[-1][0]:.1f} {",".join(ev)}')
    # 8f. 교차로 양보 중에는 상한을 세지 않는다
    ev = []
    pol = TtcPolicy(path, on_event=lambda k, d: ev.append(k))
    for k in range(800):
        t = k * 0.05
        objs = []
        for i in range(16):
            y = -30.0 + 8.33 * (t - 3.0 * i)
            if -30.0 <= y < 40.0:
                objs.append(Fake(80 + i, 10.0, y, speed=8.33,
                                 heading=math.pi / 2, dims=CAR))
        cmd = pol.update(0, 0.0, 0.0, 0.0, 0.0, 0.0, objs, t, yielding=True)
    check('양보 중: hold 상한 동결', 'ttc_hold_timeout' not in ev
          and cmd.action == 'hold', f'{cmd.action} {",".join(ev)}')

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
        # 풋프린트 간격 0.45 / 0.65m ― 판정 간격(0.5) 양쪽을 오간다
        lat = 2.3 if int(t / 0.1) % 2 == 0 else 2.5
        return [Fake(10, 20.0 + 8.5 * t, lat, speed=8.5, dims=CAR)]
    log, _ = OB._drive(pol, path, wobble, n=400)
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

    # 16b. 정지 의사(stop_avail) ― crosswalks 예방용. 정지한 차로 내 객체는
    #      warn부터, 이동 객체는 brake/stop부터, 그 밖은 nan
    def run_sa(objs_at, n=200):
        pol = TtcPolicy(path)
        x, v, idx, out = 0.0, 12.5, 0, []
        for k in range(n):
            t = k * 0.05
            while idx < len(path.pts) - 1 and path.s[idx + 1] <= x:
                idx += 1
            cmd = pol.update(idx, x, v, x, 0.0, 0.0, objs_at(t, x), t)
            out.append((t, x, cmd.action, cmd.stop_avail))
            v_cmd = min(12.5, cmd.v_limit)
            a = max(-6.0, min(3.0, min(1.2 * (v_cmd - v), cmd.accel_limit)))
            if v_cmd <= 0.0:
                a = min(a, -1.5)
            x += v * 0.05
            v = max(0.0, v + a * 0.05)
        return out

    car = Fake(1, 80.0, 0.0, dims=CAR)
    sa = run_sa(lambda t, x: [car])
    w = next((r for r in sa if r[2] == 'warn'), None)
    want = (car.x - car.length * 0.5) - (w[1] + FRONT_OVERHANG) - MIN_GAP if w else 0.0
    check('stop_avail: 정지 앞차는 warn부터 정지점을 낸다',
          w is not None and w[3] == w[3] and abs(w[3] - want) < 0.5,
          f'{w} 기대 {want:.1f}')
    check('stop_avail: brake/stop 프레임은 전부 유한',
          all(r[3] == r[3] for r in sa if r[2] in ('brake', 'stop')))
    sa2 = run_sa(lambda t, x: [Fake(3, 40.0 + 6.0 * t, 0.0, speed=6.0, dims=CAR)])
    warns = [r for r in sa2 if r[2] == 'warn']
    check('stop_avail: 이동 객체(느린 앞차)의 warn은 nan',
          warns and all(r[3] != r[3] for r in warns), f'{len(warns)}프레임')

    # 18. 회전 차량 (0.4rad/s) ― 등회전 예측은 직선 외삽보다 간격을 작게 본다.
    #     9/12 본대회: 남하하던 차가 꺾어 우리 옆 대향 차로로 들어왔다
    def turner(t, y_end):
        w = -0.4
        r = 8.33 / abs(w)
        h = -math.pi / 2 + w * t
        # 남쪽을 보고 (40, y_end + r)에서 출발해 우회전, (40 - r, y_end)에서 서쪽을 본다
        cx, cy = 40.0 - r, y_end + r
        return Fake(18, cx + r * math.cos(h + math.pi / 2),
                    cy + r * math.sin(h + math.pi / 2), speed=8.33, heading=h,
                    dims=CAR)
    res = {}
    for name, use in (('ctrv', True), ('line', False)):
        pol = TtcPolicy(path, use_ctrv=use)
        for k in range(8):
            o = turner(k * 0.05, 2.0)
            pol._update_tracks([o])
        cands, _b = pol._scan(0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, [o], False)
        res[name] = cands.get(18)
    cc, cl = res['ctrv'], res['line']
    check('회전 차량: 요레이트 추정 -0.4rad/s',
          cc is not None and abs(cc.w + 0.4) < 0.03, f'{cc.w if cc else None}')
    check('회전 차량: 등회전 예측 간격 < 직선 외삽 간격',
          cc is not None and cc.danger
          and (cl is None or (not cl.danger and cl.gap > cc.gap + 3.0)),
          f'등회전 {cc.gap if cc else None:.2f}m 위험={cc.danger if cc else None}, '
          f'직선 {"후보 아님" if cl is None else f"{cl.gap:.2f}m 위험={cl.danger}"}')
    # 18b. 요레이트 튐 방어
    hh = ctrv.HeadingHist()
    for h in (0.0, 0.02, 0.04, 1.2):
        hh.push(h)
    check('요레이트: 헤딩 튐이면 이력을 버린다', hh.rate(8.0) == 0.0 and len(hh.h) == 1,
          f'{hh.rate(8.0)} {hh.h}')
    hh = ctrv.HeadingHist()
    for k in range(6):
        hh.push(0.5 + 0.001 * (-1) ** k)
    check('요레이트: 잡음 수준이면 직선', hh.rate(8.0) == 0.0, f'{hh.rate(8.0)}')
    hh = ctrv.HeadingHist()
    for k in range(6):
        hh.push(3.1 + 0.02 * k)         # pi를 넘어간다
    check('요레이트: wrap을 넘어도 연속', abs(hh.rate(8.0) - 0.4) < 1e-6,
          f'{hh.rate(8.0)}')

    # 19. 교차 상대의 정지 목표 ― 회랑 진입점 앞에 서고 하한(10km/h)이 없다
    ev = []
    pol = TtcPolicy(path, on_event=lambda k, d: ev.append((k, d)))
    trace = []
    def slow_cross(t, x):
        lat = -12.0 + 3.0 * t
        trace.append((t, x, lat))
        return [Fake(19, 40.0, lat, speed=3.0, heading=math.pi / 2, dims=CAR)]
    log, end = OB._drive(pol, path, slow_cross, n=400)
    band = 0.95 + 2.195 + 0.5
    inside = [r for r in trace if abs(r[2]) < band]
    bmax = max(r[1] + FRONT_OVERHANG for r in inside)
    check('교차 상대: 회랑 진입점 앞에 선다 (앞범퍼 <= 진입점)',
          bmax <= 40.0 - 0.905 + 0.3, f'차가 회랑 안일 때 앞범퍼 최대 {bmax:.2f}m '
          f'(진입점 {40.0 - 0.905:.2f})')
    vmin = min(r[2] for r in log)
    check('교차 상대: 하한 없음 (10km/h 아래로 내려간다)', vmin < V_FLOOR - 0.5,
          f'최저 {vmin * 3.6:.1f}km/h')
    check('교차 상대: 지나간 뒤 출발', end > 60.0, f'{end:.1f}m')
    # 앞차 추종의 하한은 그대로다
    pol = TtcPolicy(path)
    log, _ = OB._drive(pol, path,
                       lambda t, x: [Fake(19, 60.0 + 4.0 * t, 0.0, speed=4.0, dims=CAR)],
                       n=120)
    br = [r[6] for r in log if r[3] == 'brake']
    check('앞차 추종: brake 하한 유지', not br or min(br) >= min(V_FLOOR, 4.0) - 1e-6,
          f'{len(br)}프레임 최저 {min(br) * 3.6 if br else 0:.1f}km/h')

    # 20. 판정 간격은 예측 시각에 비례한다 (교차만). 간격 1.2m가 τ=4s면 위험,
    #     τ=0.4s면 아니다
    BIKE = (2.0, 0.8, 1.5)
    def probe(v_ego, x_obj, lat0, v_obj):
        pol = TtcPolicy(path)
        for k in range(3):
            o = Fake(20, x_obj, lat0 + v_obj * 0.05 * (k - 2), speed=v_obj,
                     heading=math.pi / 2, dims=BIKE)
            pol._update_tracks([o])
        cands, _b = pol._scan(0, 0.0, 0.0, v_ego, 0.0, 0.0, 0.0, 0.0, [o], False)
        return cands.get(20)
    far = probe(10.0, 1.4 + 40.0 + 2.4 + 0.4 + 1.2, -32.0, 8.0)
    near = probe(2.0, 1.4 + 0.8 + 2.4 + 0.4 + 1.2, -3.2, 8.0)
    check('판정 간격: τ=4s의 간격 1.2m는 위험',
          far is not None and far.danger and far.t > 3.5,
          f'{None if far is None else (far.danger, round(far.t, 2), round(far.gap, 2))}')
    check('판정 간격: τ=0.4s의 간격 1.2m는 위험 아님',
          near is None or not near.danger,
          f'{None if near is None else (near.danger, round(near.t, 2), round(near.gap, 2))}')
    # 21. 옆 차로 동행차 (간격 1.4m, 교차 아님) ― 전 구간 무반응
    pol = TtcPolicy(path)
    log, _ = OB._drive(pol, path,
                       lambda t, x: [Fake(21, x + 6.0, 3.3, speed=12.5, dims=CAR)], n=200)
    check('옆 차로 동행차(간격 1.4m): 무반응', acts(log) == {'none'},
          f'{sorted(acts(log))}')
    g = footprint_gap(10.0, 0.0, 0.0, 4.39, 1.81)
    check('풋프린트 간격: 앞 10m 승용차 = 10 - 2.195 - 2.4',
          abs(g - (10.0 - 2.195 - 2.4)) < 1e-9, f'{g:.3f}')

    # 21b. 굽은 도로의 대향차 ― 도로를 따라 도는 것이지 회전 차량이 아니다
    arc = _Arc(radius=60.0)
    def onc_arc(t, s):
        so = 110.0 - 12.5 * t
        px, py = _point_at(arc, so)
        qx, qy = _point_at(arc, so + 1.0)
        th = math.atan2(qy - py, qx - px)
        return [Fake(24, px - math.sin(th) * 3.5, py + math.cos(th) * 3.5,
                     speed=12.5, heading=th + math.pi, dims=CAR)]
    pol = TtcPolicy(arc)
    log, _ = _drive_path(pol, arc, onc_arc, n=160)
    check('곡선: 옆 차로 대향차 무반응 (요레이트가 도로 곡률과 같다)',
          acts(log) == {'none'}, f'{sorted(acts(log))}')
    check('도로 곡률: 반경 60m 좌회전 = +1/60',
          abs(_road_kappa(arc, 50.0) - 1.0 / 60.0) < 1e-3,
          f'{_road_kappa(arc, 50.0):.5f}')

    # 22. 선분 끝에 걸린 투영 ― 창 끝 너머 40m의 객체가 "코앞"으로 찍히면 안 된다
    s_p, lat_p, _t = _proj(path, 0, 10, 50.0, 1.0)
    check('끝에 걸린 투영: 실제 거리를 횡거리로', abs(lat_p) > 35.0,
          f's={s_p:.1f} lat={lat_p:.1f}')
    s_p, lat_p, _t = _proj(path, 0, 60, 50.0, 1.0)
    check('정상 투영: 그대로', abs(s_p - 50.0) < 1e-6 and abs(lat_p - 1.0) < 1e-6,
          f's={s_p:.1f} lat={lat_p:.1f}')
    # 23. 정지 유지 중 경로가 바뀌어 막던 차가 횡 3m 옆이 되면 곧바로 놓는다
    ev = []
    pol = TtcPolicy(path, on_event=lambda k, d: ev.append(k))
    car = Fake(23, 60.0, 0.0, dims=CAR)
    log, end = OB._drive(pol, path, lambda t, x: [car], n=200)
    held = pol.update(int(end), end, 0.0, end, 0.0, 0.0, [car], 10.0)
    side = Fake(23, 60.0, 3.0, dims=CAR)
    for k in range(3):
        moved = pol.update(int(end), end, 0.0, end, 0.0, 0.0, [side], 10.05 + 0.05 * k)
    check('정지 유지: 횡 3m로 비켜난 정지 차량은 놓는다',
          held.action == 'stop' and moved.action == 'none'
          and moved.v_limit == float('inf'), f'{held.action} -> {moved.action}')

    # 17. 비용 ― 30객체 최악에서 프레임당 2ms 미만
    pol = TtcPolicy(path)
    many = [Fake(100 + k, 20.0 + 2.0 * k, (-1) ** k * 3.0, speed=5.0,
                 heading=0.3, dims=CAR) for k in range(30)]
    for k in range(3):
        pol.update(0, 0.0, 12.5, 0.0, 0.0, 0.0, many, k * 0.05)
    per = float('inf')
    for _rep in range(5):           # 최솟값을 쓴다 -- 한 번 재면 잡음이 0.5ms다
        t0 = _time.perf_counter()
        for k in range(200):
            pol.update(0, 0.0, 12.5, 0.0, 0.0, 0.0, many, 1.0 + k * 0.05)
        per = min(per, (_time.perf_counter() - t0) / 200 * 1000.0)
    check('비용: 30객체 프레임당 3.5ms 미만', per < 3.5, f'{per:.3f}ms')
    # 17b. 전부 회전 중인 30객체 (등회전 투영이 붙는 최악)
    pol = TtcPolicy(path)
    def spin(k):
        return [Fake(100 + i, 20.0 + 2.0 * i, (-1) ** i * 6.0, speed=8.0,
                     heading=0.3 + 0.02 * k, dims=CAR) for i in range(30)]
    for k in range(6):
        pol.update(0, 0.0, 12.5, 0.0, 0.0, 0.0, spin(k), k * 0.05)
    frames = [spin(6 + k % 3) for k in range(3)]
    per = float('inf')
    for _rep in range(3):           # 최솟값 (위와 같은 이유)
        t0 = _time.perf_counter()
        for k in range(50):
            pol._scan(0, 0.0, 0.0, 12.5, 0.0, 0.0, 0.0, 0.0, frames[0], False)
        per = min(per, (_time.perf_counter() - t0) / 50 * 1000.0)
    check('비용: 회전 중 30객체 프레임당 10ms 미만 (20Hz 예산 50ms)', per < 10.0,
          f'{per:.3f}ms')
    # 17c. 정지 중 hold의 긴 시간창(8s, 표본 41개)으로 같은 30객체
    per = float('inf')
    for _rep in range(3):
        t0 = _time.perf_counter()
        for k in range(30):
            pol._scan(0, 0.0, 0.0, 0.0, DEPART_ACCEL, 0.0, 0.0, 0.0, frames[0],
                      True, 40)
        per = min(per, (_time.perf_counter() - t0) / 30 * 1000.0)
    check('비용: 정지 중 8s 시간창, 회전 30객체 프레임당 20ms 미만', per < 20.0,
          f'{per:.3f}ms')

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
