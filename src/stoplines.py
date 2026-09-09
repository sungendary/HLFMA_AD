#!/usr/bin/env python3
"""신호등 정지 정책 (평가항목 7·9).

route_path_*_events.csv의 정지선을 주행 경로에 붙이고, 매 프레임 9910이 준
tl_id/tl_state를 보고 속도 상한 하나를 돌려준다. pure_pursuit은 그 값을
기존 v_cmd에 min()으로 섞기만 하면 된다.

I/O도 소켓도 없다. 순수 로직이라 로그만으로 재현·검증된다.

--- 사전테스트 6세션(2026-09-04) 실측이 이 파일의 모든 상수를 정한다 ---

1. **VTD는 정지선에 닿는 순간 신호를 NONE으로 내린다.** 관측된 이탈 지점은
   후륜축 기준 -0.3 ~ +0.4m. 즉 교차로를 지나는 프레임의 tl_state는 이미 0이라
   "통과 시점 상태"로는 판정할 수 없다. 반대로 **정지선 앞에 서 있는 동안에는
   신호가 계속 보인다**(후륜축이 4.8m 뒤에 있으므로). 정지 후 녹색 대기가 성립한다.

2. **신호 획득 거리는 65 ~ 200m.** 최소가 65.4m(ctrl 25|26)인데 45km/h에서
   DECEL_SIGNAL 정지에 39m면 되므로 가장 짧은 접근에서도 여유가 있다.

3. **긴 접근에서는 신호가 여러 번 순환한다.** 064346의 s=488.8 접근은
   GREEN@200m -> ... -> RED@23m로 3주기를 돌았다. 첫 관측으로 래치하면 안 되고
   매 프레임 다시 판정해야 한다. 여기서 접근 중 래치를 두지 않는 이유다.

4. **딜레마존이 실재한다.** 111610의 s=605.1은 G+L@66m -> YELLOW@19m였다.
   황색은 3초 뒤 적색이 되므로 "설 수 있으면 서고, 못 서면 통과"로 가른다.
   판정은 **정지선까지의 거리**로 하고 DECEL_HARD를 쓴다. 여기에 정지 마진을
   빼고 재면 설 수 있는데도 통과로 기울어 적색에 걸린다 ― sim에서 실제로
   그랬다(포기하고 통과했는데 정지선 도착 속도가 4.4km/h였다).
   반대로 "황색이면 무조건 정지"도 나쁘다. 범퍼가 정지선을 넘긴 채 선다.

5. **관측된 state는 0/1/2/3/5뿐이다.** 4(좌회전 전용)와 6(점멸)은 한 번도 나온
   적이 없다. 본선에서 처음 겪을 수 있으므로 만나면 반드시 이벤트를 남긴다.
"""

from __future__ import annotations

import csv
import math
import os

# 9910 신호 상태 (hlvtd_client.TL_STATE와 같은 값)
NONE, RED, YELLOW, GREEN, LEFT, GREEN_LEFT, FLASHING = 0, 1, 2, 3, 4, 5, 6

FRONT_OVERHANG = 3.808      # m, 후륜축 -> 앞범퍼. 9910 ego 좌표는 후륜축이다
STOP_MARGIN = 1.0           # m, 범퍼를 정지선 이만큼 앞에 세운다

DECEL_SIGNAL = 2.0          # m/s^2, 정지 프로파일. 경로 끝(1.5)보다 강하게
DECEL_HARD = 4.5            # m/s^2, "정지할 수 있는가" 판정에만 쓴다.
                            # 지령 하한은 -6.0이라 추종 오차분 여유가 있다

STOP_SPEED = 0.3            # m/s, 이 아래면 정지로 본다
PASS_THROUGH_MS = 2.0       # m/s, 범퍼가 정지선을 넘었을 때 이보다 빠르면
                            # 교차로를 빠져나간다. 이보다 느리면 그대로 선다
TL_LOOKAHEAD = 130.0        # m, 이 안의 정지선만 대상 (실측 획득거리 65~200m)
BEHIND_M = 2.0              # m, 정지선을 이만큼 지날 때까지는 대상으로 남긴다
SNAP_MAX_M = 6.0            # m, 경로에서 이보다 멀면 이 경로의 정지선이 아니다

NONE_RELEASE_S = 2.0        # s, 정지 중 NONE이 이만큼 지속되면 출발 (교착 방지)
MAX_HOLD_S = 45.0           # s, 최종 교착 방지
GREEN_STUCK_S = 3.0         # s, 항목 8 감시 (감점 한계는 5초)
GREEN_STUCK_M = 30.0        # m, 항목 8은 정지선 30m 이내에서만 걸린다
PAUSE_HOLD_S = 0.6          # s, 항목 9 적색점멸 일시정지 (요건 0.5초)
PAUSE_ZONE_M = 2.0          # m, 항목 9 "정지선 2m 이내" — 범퍼 기준

MANEUVER_LOOKAHEAD = 40.0   # m, 정지선 이후 이만큼 앞의 헤딩과 비교
MANEUVER_TURN_RAD = math.radians(25.0)

# 그냥 통과해도 되는 상태
GO_STATES = frozenset((GREEN, GREEN_LEFT))
# 사전테스트에서 한 번도 관측되지 않은 상태. 만나면 이벤트를 남긴다
UNOBSERVED_STATES = frozenset((LEFT, FLASHING))

STATE_NAME = {NONE: 'NONE', RED: 'RED', YELLOW: 'YELLOW', GREEN: 'GREEN',
              LEFT: 'LEFT', GREEN_LEFT: 'GREEN+LEFT', FLASHING: 'FLASHING'}


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def default_events_path(waypoints: str) -> str:
    """waypoints CSV 옆의 _events.csv 경로. 없으면 빈 문자열."""
    base, ext = os.path.splitext(waypoints)
    cand = base + '_events' + ext
    return cand if os.path.exists(cand) else ''


class Stopline:
    """경로에 붙인 정지선 하나."""

    __slots__ = ('idx', 's', 'x', 'y', 'ctrl', 'maneuver', 'resid',
                 'road_id', 'lane_id', 's_along')

    def __init__(self, idx, s, x, y, ctrl, maneuver, resid,
                 road_id='', lane_id='', s_along=0.0):
        self.idx = idx              # 경로 인덱스
        self.s = s                  # 경로 누적거리 [m]
        self.x, self.y = x, y
        self.ctrl = ctrl            # frozenset[int]. 비면 비신호 정지선
        self.maneuver = maneuver    # 'left' / 'straight' / 'right'
        self.resid = resid          # 스냅 잔차 [m]
        self.road_id, self.lane_id = road_id, lane_id
        self.s_along = s_along      # 이벤트 CSV가 주장한 원래 값 (진단용)

    @property
    def signalized(self) -> bool:
        return bool(self.ctrl)

    def ctrl_str(self) -> str:
        return '|'.join(str(c) for c in sorted(self.ctrl)) if self.ctrl else '-'

    def __repr__(self) -> str:
        return (f'<Stopline s={self.s:.1f} ctrl={self.ctrl_str()} '
                f'{self.maneuver} resid={self.resid:.2f}>')


class SignalCmd:
    """한 프레임의 판정 결과."""

    __slots__ = ('v_limit', 'action', 'state', 'ctrl', 'dist', 'reason',
                 'accel_limit', 'stop_avail')

    def __init__(self, v_limit=float('inf'), action='none', state=NONE,
                 ctrl=0, dist=float('nan'), reason='',
                 accel_limit=float('inf'), stop_avail=float('nan')):
        self.v_limit = v_limit      # m/s. inf면 제약 없음
        self.accel_limit = accel_limit  # m/s^2 상한. inf면 제약 없음
        self.action = action        # none / go / stop / hold / pause
        self.state = state
        self.ctrl = ctrl            # 그때 받은 tl_id
        self.dist = dist            # 후륜축 -> 정지선 거리 [m]
        self.reason = reason
        # 후륜축이 지금부터 몇 m 더 가서 설 작정인가. nan이면 정지 의사 없음.
        # 횡단보도 정차 금지(crosswalks.py, 항목 12)가 "그 자리가 횡단보도
        # 위인가"를 판정하는 데 쓴다. 여기 말고 소비자가 없다.
        self.stop_avail = stop_avail

    @property
    def stopping(self) -> bool:
        return self.action in ('stop', 'hold', 'pause')

    def __repr__(self) -> str:
        return f'<SignalCmd {self.action} v<={self.v_limit:.1f} d={self.dist:.1f}>'


# ----------------------------------------------------------------------
# 로드 — 이벤트 CSV를 경로에 붙인다

def _path_heading(path, i: int, span_m: float = 3.0) -> float:
    """경로 인덱스 i에서의 진행 방위."""
    j = i
    target = path.s[i] + span_m
    while j < len(path.pts) - 1 and path.s[j] < target:
        j += 1
    if j == i:
        j = min(i + 1, len(path.pts) - 1)
        i = max(0, j - 1)
    return math.atan2(path.pts[j][1] - path.pts[i][1],
                      path.pts[j][0] - path.pts[i][0])


def _maneuver_at(path, i: int) -> str:
    """정지선 이후 MANEUVER_LOOKAHEAD 구간의 기동. state 4 판정에 쓴다."""
    j = i
    target = path.s[i] + MANEUVER_LOOKAHEAD
    while j < len(path.pts) - 1 and path.s[j] < target:
        j += 1
    d = wrap_pi(_path_heading(path, j) - _path_heading(path, i))
    if d > MANEUVER_TURN_RAD:
        return 'left'
    if d < -MANEUVER_TURN_RAD:
        return 'right'
    return 'straight'


def parse_ctrl(detail: str) -> frozenset:
    """'ctrl=11|12' -> frozenset({11, 12}). 빈 문자열이면 빈 집합.

    한 접근로에 신호 헤드가 둘 이상 걸린 곳이 있어 4단계가 후보를 하나로
    좁히지 않았다. 런타임은 "받은 tl_id가 이 집합에 있는가"로만 판정한다.
    """
    detail = (detail or '').strip()
    if not detail.startswith('ctrl='):
        return frozenset()
    out = set()
    for tok in detail[5:].split()[0].split('|'):
        tok = tok.strip()
        if tok.isdigit():
            out.add(int(tok))
    return frozenset(out)


def load_for_path(path, events_csv: str) -> tuple[list, list]:
    """이벤트 CSV의 정지선을 path에 스냅한다. (정지선 목록, 경고 목록).

    s_along이 아니라 **(x,y)로 스냅한다.** 이유가 셋이다.
      · pure_pursuit --segments가 부분집합이면 s_along의 원점이 어긋난다
      · 사전테스트 로그는 수동 녹화본을 달렸는데 이벤트 CSV는 생성 경로
        기준이다. 좌표여야 두 경로 모두에 같은 코드가 붙는다
      · route_path_pt1_events.csv처럼 경로 재생성에 뒤처진 파일이 있다
    """
    rows = []
    with open(events_csv, newline='', encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            if r.get('kind') == 'stopline':
                rows.append(r)

    out, warn = [], []
    for r in rows:
        x, y = float(r['x']), float(r['y'])
        i = path.nearest(x, y)
        resid = math.hypot(path.pts[i][0] - x, path.pts[i][1] - y)
        ctrl = parse_ctrl(r.get('detail', ''))
        s_along = float(r.get('s_along', 0.0) or 0.0)
        if resid > SNAP_MAX_M:
            warn.append(f'정지선 s_along={s_along:.1f} ctrl={"|".join(map(str, sorted(ctrl))) or "-"} '
                        f'를 버린다 ― 경로에서 {resid:.1f}m (한계 {SNAP_MAX_M:.0f}m)')
            continue
        if resid > 1.5:
            warn.append(f'정지선 s_along={s_along:.1f} 스냅 잔차 {resid:.1f}m '
                        f'― 이벤트 CSV가 이 경로보다 오래됐을 수 있다')
        out.append(Stopline(i, path.s[i], x, y, ctrl, _maneuver_at(path, i),
                            resid, r.get('road_id', ''), r.get('lane_id', ''),
                            s_along))

    out.sort(key=lambda sl: sl.s)
    # 같은 정지선이 t만 몇 cm 다르게 두 번 그려진 곳이 있다. 1.5m 안이면 하나로 본다
    merged = []
    for sl in out:
        if merged and sl.s - merged[-1].s < 1.5:
            prev = merged[-1]
            prev.ctrl = prev.ctrl | sl.ctrl
            continue
        merged.append(sl)
    return merged, warn


# ----------------------------------------------------------------------
# 정지 프로파일

def _profile(avail: float, v: float) -> tuple:
    """avail m 안에 세우기 위한 (속도 상한, 가속도 상한).

    속도 상한만으로는 부족하다. pure_pursuit의 가속도는 P 제어이고
    (accel = SPEED_KP * (v_cmd - v)), sqrt 프로파일을 따라가려면 일정한
    감속이 계속 필요하므로 정상상태 오차가 남는다. sim에서 실제로
    프로파일보다 9km/h 위로 달려 정지선을 0.8m 넘었다.

    그래서 **지금 속도로 avail 안에 서는 데 필요한 감속을 직접 준다.**
    v^2 = 2*a*avail 을 a에 대해 푼 값이다. 프로파일 아래에 있을 때는
    걸지 않는다 ― 안 그러면 멀리서부터 가속을 막아 버린다.
    """
    v_limit = 0.0 if avail <= 0.3 else math.sqrt(2.0 * DECEL_SIGNAL * avail)
    if v <= v_limit:
        return v_limit, float('inf')
    return v_limit, -(v * v) / (2.0 * max(avail, 0.05))


# ----------------------------------------------------------------------
# 정책

class SignalPolicy:
    """매 프레임 속도 상한 하나를 낸다.

    접근 중에는 래치하지 않는다 (모듈 docstring 3번). 래치는 "이 정지선은
    끝났다"는 판정에만 건다.
    """

    def __init__(self, path, stoplines, *, front_overhang=FRONT_OVERHANG,
                 max_hold=MAX_HOLD_S, on_event=None):
        self.path = path
        self.stoplines = list(stoplines)
        self.front = front_overhang
        self.max_hold = max_hold
        self._on_event = on_event or (lambda kind, detail: None)
        self.reset()

    # -- 상태 -----------------------------------------------------------
    def reset(self) -> None:
        """리스폰 시 호출한다. 지나갔다고 판정한 정지선을 되돌린다."""
        self._done = set()          # 끝난 정지선 (id())
        self._pause_done = set()    # 항목 9 일시정지를 마친 정지선
        self._stop_since = None     # 정지 결정 시각
        self._hold_since = None     # 실제로 멈춘 시각
        self._none_since = None     # 정지 중 NONE이 시작된 시각
        self._pause_since = None    # 일시정지 중 멈춘 시각
        self._green_stuck_since = None
        self._announced = set()     # 이벤트 중복 억제 키
        self._cur = None            # 현재 대상 정지선
        self._spent = set()         # 방금 놓아준 정지선의 ctrl id
        self._blocked = False       # 전방 객체에 막혀 있나 (obstacles.py가 준다)

    def _announce(self, key, kind: str, detail: str) -> None:
        if key in self._announced:
            return
        self._announced.add(key)
        self._on_event(kind, detail)

    # -- 대상 선택 -------------------------------------------------------
    def _candidates(self, s_ego: float):
        for sl in self.stoplines:
            if id(sl) in self._done:
                continue
            d = sl.s - s_ego
            if -BEHIND_M <= d <= TL_LOOKAHEAD:
                yield d, sl

    def _select(self, s_ego: float, tl_id: int):
        """받은 tl_id를 관장하는 정지선. 없으면 최근접으로 폴백."""
        cands = sorted(self._candidates(s_ego), key=lambda p: p[0])
        if not cands:
            return None, float('nan')
        for d, sl in cands:
            if tl_id in sl.ctrl:
                return sl, d
        # 체인은 두 경로 실측 전수 일치라 여기 오는 일은 거의 없다.
        # 폴백은 **신호 정지선으로만** 간다. 비신호 정지선은 애초에 신호가
        # 관장하지 않으므로 거기서 서면 근거 없는 정지가 된다. 실제로 sim에서
        # 이 실수가 나왔다 — 교차로를 막 빠져나온 뒤 아직 남아 있던 tl_id가
        # 100m 앞 비신호 정지선을 잡아 4km/h까지 제동했다.
        #
        # 그리고 방금 놓아준 신호가 몇 프레임 더 오는 것뿐이면 아무것도
        # 하지 않는다. 사전테스트 재생에서 정지선 통과 직후 남은 tl_id가
        # 90m 앞 다음 정지선을 잡고 제동한 적이 있다.
        if tl_id in self._spent:
            return None, float('nan')
        for d, sl in cands:
            if sl.signalized:
                self._announce(('fallback', tl_id, id(sl)), 'tl_fallback',
                               f'tl_id={tl_id}가 경로 정지선의 ctrl에 없다 ― '
                               f'최근접 신호 정지선(s={sl.s:.1f} '
                               f'ctrl={sl.ctrl_str()}) {d:.1f}m 사용')
                return sl, d
        # 앞에 신호 정지선이 없다. 방금 지나온 신호가 아직 남아 있는 것이다
        return None, float('nan')

    # -- 본체 -----------------------------------------------------------
    def update(self, s_ego: float, v: float, tl_id: int, tl_state: int,
               t: float, blocked: bool = False) -> SignalCmd:
        """blocked는 "전방에 차량·보행자가 있어 못 간다"는 뜻이다.

        충돌 회피(obstacles.py)가 넘긴다. 기본값이 False라 예전 호출은 그대로
        동작한다. 두 가지를 바꾼다.

          · 항목 8(녹색 정지선 30m 이내 5초 정차)은 전방 객체가 있으면 면책이다
          · MAX_HOLD_S를 세지 않는다. 안 그러면 보행자에 막혀 적색 앞에 45초
            서 있을 때 정지선을 놓아 버리고, 보행자가 비키는 순간 적색을 통과한다
        """
        self._blocked = blocked
        # 지나간 정지선을 정리한다. 범퍼가 넘어갔으면 끝이다.
        for sl in self.stoplines:
            if id(sl) not in self._done and sl.s - s_ego < -BEHIND_M:
                self._release(sl)

        if tl_state == NONE:
            return self._on_none(s_ego, v, t)

        sl, d = self._select(s_ego, tl_id)
        if sl is None:
            self._clear_timers()
            return SignalCmd(state=tl_state, ctrl=tl_id,
                             reason='대상 정지선 없음')

        self._none_since = None
        self._cur = sl
        avail = d - self.front - STOP_MARGIN     # 프로파일이 겨냥할 거리
        bumper = d - self.front                  # 범퍼-정지선 거리
        # 지금 속도로 정지에 필요한 거리. **정지선까지**로 잰다(마진 제외).
        # 마진까지 빼고 재면 설 수 있는데도 못 선다고 판정한다 — sim에서
        # 실제로 그랬다. 황색을 포기하고 통과했는데 정지선 도착 속도가
        # 4.4km/h였고 그때 신호는 이미 적색이었다.
        need = v * v / (2.0 * DECEL_HARD)

        if tl_state in GO_STATES:
            return self._go(sl, tl_state, tl_id, d, v, t, '녹색')

        if tl_state == RED:
            if bumper <= 0.0 and v > PASS_THROUGH_MS:
                # 이미 정지선을 넘었고 아직 달리고 있다. 교차로 한가운데서
                # 제동하면 더 나쁘다. 다만 **거의 멈춘 상태라면 풀지 않는다** —
                # 그러면 정지 직전에 제동이 풀려 적색으로 튀어나간다
                # (sim에서 실제로 이 실수가 났다).
                self._announce((id(sl), 'late'), 'tl_late',
                               f'ctrl={sl.ctrl_str()} 정지선을 {-bumper:.1f}m 넘은 뒤 '
                               f'적색 v={v * 3.6:.0f}km/h ― 교차로를 빠져나간다')
                self._release(sl)
                self._clear_timers()
                return SignalCmd(state=tl_state, ctrl=tl_id, dist=d,
                                 action='go', reason='정지선 통과 후 적색')
            return self._stop(sl, tl_state, tl_id, d, avail, v, t, '적색')

        if tl_state == YELLOW:
            if v > PASS_THROUGH_MS and (bumper <= 0.0 or bumper < need):
                # 딜레마존 (docstring 4번). 정지하면 정지선을 침범한다
                self._announce((id(sl), 'dilemma'), 'tl_yellow_go',
                               f'ctrl={sl.ctrl_str()} 황색 {d:.1f}m '
                               f'v={v * 3.6:.0f}km/h ― 정지에 {need:.1f}m 필요, '
                               f'정지선까지 {bumper:.1f}m. 통과한다')
                self._release(sl)
                self._clear_timers()
                return SignalCmd(state=tl_state, ctrl=tl_id, dist=d,
                                 action='go', reason='황색 ― 정지 불가, 통과')
            return self._stop(sl, tl_state, tl_id, d, avail, v, t, '황색')

        if tl_state == LEFT:
            self._announce((id(sl), LEFT), 'tl_unknown_state',
                           f'state=4(LEFT) 최초 관측 ― ctrl={sl.ctrl_str()} '
                           f'기동={sl.maneuver} {d:.1f}m. 사전테스트에 없던 상태다')
            if sl.maneuver == 'left':
                return self._go(sl, tl_state, tl_id, d, v, t, '좌회전 신호 + 좌회전 기동')
            if bumper <= 0.0 and v > PASS_THROUGH_MS:
                self._release(sl)
                return SignalCmd(state=tl_state, ctrl=tl_id, dist=d,
                                 action='go', reason='정지선 통과 후 좌회전 신호')
            return self._stop(sl, tl_state, tl_id, d, avail, v, t,
                              f'좌회전 전용인데 기동은 {sl.maneuver}')

        if tl_state == FLASHING:
            self._announce((id(sl), FLASHING), 'tl_unknown_state',
                           f'state=6(FLASHING) 최초 관측 ― ctrl={sl.ctrl_str()} '
                           f'{d:.1f}m. 적/황 점멸을 구분할 수 없어 일시정지한다')
            return self._flashing(sl, tl_state, tl_id, d, avail, bumper, v, t)

        # 명세에 없는 값. 정지가 안전하다
        self._announce((id(sl), tl_state), 'tl_unknown_state',
                       f'명세에 없는 state={tl_state} ― ctrl={sl.ctrl_str()} '
                       f'{d:.1f}m. 정지로 처리한다')
        if bumper <= 0.0 and v > PASS_THROUGH_MS:
            self._release(sl)
            return SignalCmd(state=tl_state, ctrl=tl_id, dist=d, action='go',
                             reason=f'미지 state={tl_state}, 정지선 통과 후')
        return self._stop(sl, tl_state, tl_id, d, avail, v, t,
                          f'미지 state={tl_state}')

    # -- 가지 -----------------------------------------------------------
    def _release(self, sl) -> None:
        """이 정지선은 끝났다. 그 ctrl이 몇 프레임 더 와도 무시한다.

        같은 ctrl이 경로 뒤쪽 정지선에도 붙어 있으면 넣지 않는다. 넣으면
        그 신호를 영영 무시하게 된다.
        """
        self._done.add(id(sl))
        later = set()
        for other in self.stoplines:
            if id(other) not in self._done:
                later |= set(other.ctrl)
        self._spent |= (set(sl.ctrl) - later)

    def _clear_timers(self) -> None:
        self._stop_since = self._hold_since = None
        self._pause_since = None

    def _on_none(self, s_ego, v, t) -> SignalCmd:
        """state 0 = 해당 신호 없음.

        정상적으로는 정지선을 지나며 뜬다. 그런데 정지선 앞에 서 있는 동안
        NONE이 지속되면 그대로 교착이므로(완주가 최우선) 풀어 준다.
        """
        if self._stop_since is not None and self._cur is not None:
            d = self._cur.s - s_ego
            if d - self.front > 0.0:
                if self._none_since is None:
                    self._none_since = t
                elif t - self._none_since >= NONE_RELEASE_S:
                    self._on_event('tl_timeout',
                                   f'ctrl={self._cur.ctrl_str()} 정지선 {d:.1f}m 앞에서 '
                                   f'신호가 {NONE_RELEASE_S:.0f}초간 NONE ― 출발한다')
                    self._release(self._cur)
                    self._clear_timers()
                    self._none_since = None
                    return SignalCmd(action='go', dist=d, reason='NONE 지속 ― 해제')
                return SignalCmd(v_limit=0.0, action='hold', dist=d,
                                 reason='신호 NONE ― 판정 대기')
        self._none_since = None
        self._clear_timers()
        self._green_stuck_since = None
        return SignalCmd(reason='신호 없음')

    def _go(self, sl, state, tl_id, d, v, t, reason) -> SignalCmd:
        if self._stop_since is not None:
            self._on_event('tl_go',
                           f'ctrl={sl.ctrl_str()} {STATE_NAME.get(state, state)} '
                           f'― 정지선 {d:.1f}m 앞에서 출발')
        self._clear_timers()

        # 항목 8: 녹색 + 정지선 30m 이내 + 5초 이상 정차는 감점이다.
        # 다만 전방에 차량·보행자가 있으면 면책이므로 감시하지 않는다.
        if self._blocked:
            self._green_stuck_since = None
        elif 0.0 < d <= GREEN_STUCK_M and v < STOP_SPEED:
            if self._green_stuck_since is None:
                self._green_stuck_since = t
            elif t - self._green_stuck_since >= GREEN_STUCK_S:
                self._announce((id(sl), 'stuck'), 'tl_green_stuck',
                               f'ctrl={sl.ctrl_str()} 녹색인데 정지선 {d:.1f}m 앞에서 '
                               f'{t - self._green_stuck_since:.1f}초 정차 ― 항목 8 위험')
        else:
            self._green_stuck_since = None
        return SignalCmd(state=state, ctrl=tl_id, dist=d, action='go',
                         reason=reason)

    def _stop(self, sl, state, tl_id, d, avail, v, t, reason) -> SignalCmd:
        self._green_stuck_since = None
        if self._stop_since is None:
            self._stop_since = t
            self._on_event('tl_stop',
                           f'ctrl={sl.ctrl_str()} {STATE_NAME.get(state, state)} '
                           f'{d:.1f}m v={v * 3.6:.0f}km/h ― 정지 결정 ({reason})')

        v_limit, accel_limit = _profile(avail, v)

        if v < STOP_SPEED and v_limit <= 0.0:
            if self._hold_since is None or self._blocked:
                # 전방 객체에 막혀 못 가는 시간은 신호 대기로 세지 않는다.
                # 세면 정지선을 놓아 버리고 객체가 비키는 순간 적색을 통과한다
                self._hold_since = t
            elif t - self._hold_since >= self.max_hold:
                # 완주가 최우선이다. -6점을 먹더라도 미완주는 피한다.
                self._on_event('tl_timeout',
                               f'ctrl={sl.ctrl_str()} {STATE_NAME.get(state, state)}로 '
                               f'{self.max_hold:.0f}초 대기 ― 교착 방지로 출발한다')
                self._release(sl)
                self._clear_timers()
                return SignalCmd(state=state, ctrl=tl_id, dist=d, action='go',
                                 reason='대기 시간 초과 ― 해제')
            return SignalCmd(0.0, 'hold', state, tl_id, d, reason,
                             stop_avail=0.0)
        return SignalCmd(v_limit, 'stop', state, tl_id, d, reason, accel_limit,
                         stop_avail=avail)

    def _flashing(self, sl, state, tl_id, d, avail, bumper, v, t) -> SignalCmd:
        """항목 9 ― 적색점멸은 정지선 2m 이내 0.5초 이상 정지가 요건이다.

        적/황 점멸을 9910으로는 구분할 수 없다. 황색점멸이었다면 불필요한
        정지지만 감점은 없고, 적색점멸에서 무정차 통과는 중대 -6점이다.
        """
        if id(sl) in self._pause_done:
            return self._go(sl, state, tl_id, d, v, t, '점멸 ― 일시정지 완료')
        if bumper <= 0.0 and v > PASS_THROUGH_MS:
            self._announce((id(sl), 'pause_late'), 'tl_pause',
                           f'ctrl={sl.ctrl_str()} 점멸 ― 정지선을 {-bumper:.1f}m 넘었다. '
                           f'일시정지 못 함 (항목 9 위험)')
            self._release(sl)
            self._clear_timers()
            return SignalCmd(state=state, ctrl=tl_id, dist=d, action='go',
                             reason='점멸 ― 정지선 통과 후')

        if self._stop_since is None:
            self._stop_since = t
            self._on_event('tl_stop',
                           f'ctrl={sl.ctrl_str()} FLASHING {d:.1f}m ― '
                           f'일시정지 (항목 9)')

        if v < STOP_SPEED and bumper <= PAUSE_ZONE_M:
            if self._pause_since is None:
                self._pause_since = t
            elif t - self._pause_since >= PAUSE_HOLD_S:
                self._pause_done.add(id(sl))
                self._on_event('tl_pause',
                               f'ctrl={sl.ctrl_str()} 범퍼 {bumper:.1f}m 앞에서 '
                               f'{t - self._pause_since:.1f}초 정지 ― 항목 9 충족')
                self._clear_timers()
                return SignalCmd(state=state, ctrl=tl_id, dist=d, action='go',
                                 reason='점멸 일시정지 완료')
            return SignalCmd(0.0, 'pause', state, tl_id, d, '점멸 일시정지 중',
                             stop_avail=0.0)

        v_limit, accel_limit = _profile(avail, v)
        return SignalCmd(v_limit, 'pause', state, tl_id, d, '점멸 ― 정지선 접근',
                         accel_limit, stop_avail=avail)


def summary(stoplines) -> list:
    """시작 시 콘솔에 뿌릴 요약 줄."""
    lines = []
    for sl in stoplines:
        kind = f'ctrl={sl.ctrl_str()}' if sl.signalized else '비신호'
        lines.append(f'  s={sl.s:7.1f}m  {kind:16s} {sl.maneuver:8s} '
                     f'스냅 {sl.resid:.2f}m  ({sl.x:.1f},{sl.y:.1f})')
    return lines
