#!/usr/bin/env python3
"""횡단보도 정차 금지 (평가항목 12).

기준은 "후륜축이 횡단보도 위에 있는 채 1km/h 이하로 3초 이상"이면 경미 -3점이다.

stoplines.py / obstacles.py와 같은 구조다 ― I/O 없는 순수 로직이고 매 프레임
결과 하나를 낸다. 다만 **이 모듈만 min()을 깨는 출구를 하나 가진다** (아래 2번).

--- 이 지도에서 무엇이 실제 위험인가 ---

경로 위 횡단보도와 정지선의 거리를 전수로 재 보면 **횡단보도가 정지선보다
41~65m 앞(mid-block)에 있다.** 경로 1은 41.0/82.0/47.1/41.9m, 경로 2는
65.1/56.0/44.5m다. 즉 "정지선 = 횡단보도 앞"이라는 통념은 이 지도에서 틀렸다.

그래서 **신호 정지는 위험하지 않다.** 정지선 앞에 서면 횡단보도는 이미 40m
넘게 지난 뒤다. 위험은 하나로 좁혀진다 ― **보행자·앞차 때문에 서는 경우.**
그리고 mid-block 횡단보도는 바로 보행자가 건너는 자리다.

--- 하는 일 두 가지 ---

1. **예방** (min()으로 합류). stoplines / obstacles가 겨냥한 후륜축 정지
   지점(stop_avail)을 받아, 그 자리가 횡단보도 위면 목표를 구간 **앞**으로
   당긴다. 더 앞에서 서는 것은 언제나 더 낮은 v_limit이므로 기존 min() 합류가
   그대로 성립한다. 다만 지금 속도로 DECEL_HARD에도 못 서는 거리면 포기한다 ―
   억지 급제동은 뒤차 추돌을 만들고 결국 횡단보도 위에 서게 된다.

2. **탈출** (min()을 깨는 유일한 지점). 이미 후륜축이 올라가 있으면 저속으로
   빠져나간다. 감점 우선순위가 **항목 7(-6) > 항목 14(-6) > 항목 12(-3)**이므로
   아래를 **전부** 만족할 때만 발동한다.

     · obstacles가 켜져 있어 front_clear를 받았을 것. 없으면 발동하지 않는다
       ― 전방을 모르는 채로 밀고 나가지 않는다
     · front_clear >= 빠져나갈 거리 + ESCAPE_GAP  (항목 14 우선)
     · 신호 정지 중이고 정지선이 앞에 있지 않을 것 (항목 7 우선)

   막혀 있으면 발동하지 않고 그대로 -3을 받는다. 그게 설계다.

--- 구간의 길이는 CSV에 없다 ---

route_path_*_events.csv의 crosswalk 행은 좌표 한 점뿐이고 detail도 비어 있다
(xodr_graph.py:742-746 ― ctrl_id가 있는 정지선만 detail을 받는다).
xodr_objects.py가 <object>의 length/width를 읽지 않기 때문이다.

원본 xodr을 전수로 확인하면 Rm_Warning_Crosswalk_JPN_03.flt **400개 전부**
length=5.0 / width=1.5이고, 형제 오브젝트(Rm_StopLine_300cm는 length=0.30 /
width=3.00)와 대조하면 **length가 도로 s 방향**이다. 그래서 구간은 좌표 ±2.5m다.
이 상수의 근거는 파일이 아니라 여기 주석에만 있다.
"""

from __future__ import annotations

import csv
import math

from obstacles import _profile
from stoplines import DECEL_HARD, FRONT_OVERHANG, SNAP_MAX_M

# 구간
HALF_M = 2.5                # m, xodr length=5.0의 절반 (위 docstring)
EDGE_MARGIN = 0.5           # m, 경계 여유. 넓게 잡는 쪽이 안전하다
SNAP_WARN_M = 1.5           # m, 이보다 크면 경고만 (stoplines와 같은 규약)
MERGE_M = 1.5               # m, 이 안의 같은 종류는 한 횡단보도로 본다

# 예방
PRE_MARGIN = 1.0            # m, 후륜축을 구간 시작 이만큼 앞에 세운다
DECEL_XW = 2.0              # m/s^2, 정지 프로파일. 신호(2.0)와 같게 —
                            # 이 정지는 급할 이유가 없다

# 탈출
DWELL_MS = 1.0 / 3.6        # m/s, 항목 12의 "1km/h 이하"
ESCAPE_AFTER_S = 1.5        # s, 요건 3초의 절반에서 움직이기 시작한다
ESCAPE_V = 1.5              # m/s (5.4km/h), 빠져나갈 때의 상한
ESCAPE_GAP = 2.0            # m, 빠져나간 뒤 앞 객체와 남길 여유
EXIT_M = 1.0                # m, 구간을 이만큼 더 지나야 벗어난 것으로 본다
MAX_ESCAPE_S = 10.0         # s, 탈출 래치 상한. 이보다 길면 뭔가 잘못된 것이다

ACT_STOP = ('guard',)


class Crosswalk:
    """경로에 붙인 횡단보도 하나."""

    __slots__ = ('idx', 's', 's0', 's1', 'x', 'y', 'resid',
                 'road_id', 'lane_id', 's_along')

    def __init__(self, idx, s, x, y, resid, road_id='', lane_id='',
                 s_along=0.0):
        self.idx = idx
        self.s = s                          # 경로 위 중심 [m]
        self.s0 = s - HALF_M - EDGE_MARGIN  # 구간 시작
        self.s1 = s + HALF_M + EDGE_MARGIN  # 구간 끝
        self.x, self.y = x, y
        self.resid = resid
        self.road_id = road_id
        self.lane_id = lane_id
        self.s_along = s_along

    def __repr__(self) -> str:
        return f'<Crosswalk s={self.s:.1f} [{self.s0:.1f},{self.s1:.1f}]>'


class CrosswalkCmd:
    """한 프레임의 판정 결과."""

    __slots__ = ('v_limit', 'accel_limit', 'action', 'dist', 'escape',
                 'escape_v', 'reason')

    def __init__(self, v_limit=float('inf'), action='none',
                 dist=float('nan'), reason='', accel_limit=float('inf'),
                 escape=False, escape_v=0.0):
        self.v_limit = v_limit          # m/s. inf면 제약 없음
        self.accel_limit = accel_limit  # m/s^2 상한. inf면 제약 없음
        self.action = action            # none / on / guard / dwell /
                                        # escape / blocked / error
        self.dist = dist                # 후륜축 -> 다음 구간 시작 [m]
        self.reason = reason
        # **여기만 min()을 깬다.** pure_pursuit이 v_cmd = max(v_cmd, escape_v)
        # 를 한다. escape가 False면 그 줄은 아무 일도 하지 않는다.
        self.escape = escape
        self.escape_v = escape_v

    @property
    def stopping(self) -> bool:
        return self.action in ACT_STOP

    def __repr__(self) -> str:
        return (f'<CrosswalkCmd {self.action} v<={self.v_limit:.1f} '
                f'd={self.dist:.1f}{" ESC" if self.escape else ""}>')


# ----------------------------------------------------------------------
# 로드

def load_for_path(path, events_csv: str) -> tuple[list, list]:
    """이벤트 CSV의 횡단보도를 path에 스냅한다. (목록, 경고 목록).

    stoplines.load_for_path와 같은 이유로 **(x,y)로 스냅한다.**
    """
    rows = []
    with open(events_csv, newline='', encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            if r.get('kind') == 'crosswalk':
                rows.append(r)

    out, warn = [], []
    for r in rows:
        x, y = float(r['x']), float(r['y'])
        i = path.nearest(x, y)
        resid = math.hypot(path.pts[i][0] - x, path.pts[i][1] - y)
        s_along = float(r.get('s_along', 0.0) or 0.0)
        if resid > SNAP_MAX_M:
            warn.append(f'횡단보도 s_along={s_along:.1f} 를 버린다 ― '
                        f'경로에서 {resid:.1f}m (한계 {SNAP_MAX_M:.0f}m)')
            continue
        if resid > SNAP_WARN_M:
            warn.append(f'횡단보도 s_along={s_along:.1f} 스냅 잔차 {resid:.1f}m '
                        f'― 이벤트 CSV가 이 경로보다 오래됐을 수 있다')
        out.append(Crosswalk(i, path.s[i], x, y, resid,
                             r.get('road_id', ''), r.get('lane_id', ''),
                             s_along))

    out.sort(key=lambda c: c.s)
    merged = []
    for c in out:
        if merged and c.s - merged[-1].s < MERGE_M:
            merged[-1].s1 = max(merged[-1].s1, c.s1)
            continue
        merged.append(c)
    return merged, warn


def summary(zones: list) -> list:
    """시작 시 콘솔에 뿌릴 요약 줄."""
    return [f'  s={c.s:7.1f}m  [{c.s0:7.1f},{c.s1:7.1f}]  '
            f'스냅 {c.resid:.2f}m  ({c.x:.1f},{c.y:.1f})' for c in zones]


# ----------------------------------------------------------------------
# 정책

class CrosswalkPolicy:
    """매 프레임 속도 상한 하나 + 탈출 요청 하나를 낸다."""

    def __init__(self, path, zones, *, front_overhang: float = FRONT_OVERHANG,
                 escape_after: float = ESCAPE_AFTER_S, on_event=None):
        self.path = path
        self.zones = list(zones)
        self.front = front_overhang
        self.escape_after = escape_after
        self._on_event = on_event or (lambda kind, detail: None)
        self.reset()

    def reset(self) -> None:
        """리스폰 시 호출한다. 위치가 강제로 옮겨졌으므로 타이머를 버린다."""
        self._dwell_since = None    # 구간 위에서 멈춘 시각
        self._escape_since = None   # 탈출을 시작한 시각
        self._escape_zone = None    # 탈출 중인 구간 (래치)
        self._on_zone = None        # 지금 올라가 있는 구간
        self._announced = set()

    def _announce(self, key, kind: str, detail: str) -> None:
        if key in self._announced:
            return
        self._announced.add(key)
        self._on_event(kind, detail)

    # -- 조회 -----------------------------------------------------------
    def _zone_on(self, s_ego: float):
        for z in self.zones:
            if z.s0 <= s_ego <= z.s1:
                return z
        return None

    def _next_dist(self, s_ego: float) -> float:
        for z in self.zones:
            if z.s1 >= s_ego:
                return z.s0 - s_ego
        return float('nan')

    # -- 본체 -----------------------------------------------------------
    def update(self, s_ego: float, v: float, t: float, *,
               stop_s: float = float('nan'), tl_stop_ahead: bool = False,
               front_clear=None) -> CrosswalkCmd:
        """stop_s는 sig/obs가 겨냥한 **후륜축 정지 지점**(경로 누적거리)이다.
        nan이면 아무도 정지를 지령하지 않았다는 뜻이다.

        front_clear는 obstacles가 낸 앞범퍼 기준 전방 여유다. **None이면
        obstacles가 꺼져 있다는 뜻이고 탈출을 발동하지 않는다.**
        """
        dist = self._next_dist(s_ego)
        on = self._zone_on(s_ego)

        # -- 구간 진입/이탈 --------------------------------------------
        if on is not self._on_zone:
            if self._on_zone is not None and self._dwell_since is not None:
                self._on_event('xw_clear',
                               f'횡단보도 s={self._on_zone.s:.1f}m 벗어남 ― '
                               f'정차 {t - self._dwell_since:.1f}초')
            self._on_zone = on
            self._dwell_since = None

        # 탈출 래치는 **구간을 다 지날 때까지** 유지한다. 여기서 "명령이
        # 정지가 아니게 됐을 때" 풀면 안 된다 — 탈출이 스스로를 취소한다.
        # 움직이는 순간 v가 DWELL_MS를 넘어 정차 감시가 풀리고, 다음 프레임에
        # 다시 서고, 1.5초 뒤 또 한 프레임 기어가는 것을 반복한다.
        # obstacles.py의 크립 교착 해제가 똑같은 실수를 했었다.
        if (self._escape_zone is not None
                and s_ego > self._escape_zone.s1 + EXIT_M):
            self._on_event('xw_clear',
                           f'횡단보도 s={self._escape_zone.s:.1f}m 탈출 완료 ― '
                           f'{t - self._escape_since:.1f}초')
            self._escape_zone = None
            self._escape_since = None

        # -- 정차 감시 --------------------------------------------------
        dwelled = False
        if on is not None and v < DWELL_MS:
            if self._dwell_since is None:
                self._dwell_since = t
                self._announce((id(on), 'dwell'), 'xw_dwell',
                               f'횡단보도 s={on.s:.1f}m 위에서 정차 시작 ― '
                               f'{self.escape_after:.1f}초 뒤 탈출을 검토한다 '
                               f'(항목 12는 3초)')
            dwelled = (t - self._dwell_since) >= self.escape_after
        else:
            self._dwell_since = None

        # -- 탈출 --------------------------------------------------------
        target = self._escape_zone or (on if dwelled else None)
        if target is not None:
            on = target
            need = on.s1 + EXIT_M - s_ego
            why = ''
            if front_clear is None:
                why = 'obstacles가 꺼져 있어 전방을 모른다'
            elif tl_stop_ahead:
                why = '적색·황색 정지선이 앞에 있다 (항목 7 우선)'
            elif front_clear < need + ESCAPE_GAP:
                why = (f'전방 여유 {front_clear:.1f}m < 필요 '
                       f'{need + ESCAPE_GAP:.1f}m (항목 14 우선)')
            elif (self._escape_since is not None
                  and t - self._escape_since > MAX_ESCAPE_S):
                why = f'탈출 {MAX_ESCAPE_S:.0f}초 초과 ― 포기한다'

            if not why:
                if self._escape_since is None:
                    self._escape_since = t
                    self._escape_zone = on
                    self._on_event(
                        'crosswalk_escape',
                        f'횡단보도 s={on.s:.1f}m 위 정차 '
                        f'{t - self._dwell_since:.1f}초 ― 저속 탈출한다. '
                        f'전방 여유 {front_clear:.1f}m / 필요 '
                        f'{need + ESCAPE_GAP:.1f}m, '
                        f'신호 정지선 {"앞에 있음" if tl_stop_ahead else "없음"}')
                return CrosswalkCmd(action='escape', dist=dist,
                                    reason=f'횡단보도 탈출 {need:.1f}m 남음',
                                    escape=True, escape_v=ESCAPE_V)

            # 게이트가 막았다. 래치를 풀어 다음 프레임에 다시 판정한다 —
            # 앞차가 비키면 그때 발동해야 한다.
            self._announce((id(on), 'blocked'), 'xw_escape_blocked',
                           f'횡단보도 s={on.s:.1f}m 위에서 못 벗어난다 ― {why}. '
                           f'항목 12 -3점을 받아들인다')
            self._escape_since = None
            self._escape_zone = None
            return CrosswalkCmd(action='blocked', dist=dist, reason=why)

        # -- 예방 --------------------------------------------------------
        # 아무도 정지를 지령하지 않으면 할 일이 없다. nan 비교로 판별한다.
        if stop_s == stop_s:
            for z in self.zones:
                if z.s1 < s_ego or not (z.s0 <= stop_s <= z.s1):
                    continue
                avail = (z.s0 - PRE_MARGIN) - s_ego
                if avail <= 0.0:
                    break               # 이미 붙었다. 예방은 늦었다
                if v * v > 2.0 * DECEL_HARD * avail:
                    self._announce(
                        (id(z), 'infeasible'), 'xw_guard_infeasible',
                        f'횡단보도 s={z.s:.1f}m 위에 설 판인데 그 앞 '
                        f'{avail:.1f}m에 못 선다 (v={v * 3.6:.0f}km/h, '
                        f'{DECEL_HARD:.1f}m/s^2 기준). 급제동하지 않는다 ― '
                        f'올라가면 탈출로 처리한다')
                    break
                v_limit, accel_limit = _profile(avail, v, 0.0, DECEL_XW)
                self._announce(
                    (id(z), 'guard'), 'xw_guard',
                    f'정지 목표가 횡단보도 s={z.s:.1f}m 위 '
                    f'({stop_s - z.s0:+.1f}m) ― 구간 앞 {avail:.1f}m로 당긴다')
                return CrosswalkCmd(v_limit, 'guard', dist,
                                    f'횡단보도 앞 정지 {avail:.1f}m',
                                    accel_limit)

        if on is not None:
            return CrosswalkCmd(action='on', dist=dist, reason='횡단보도 위')
        return CrosswalkCmd(dist=dist)


# ----------------------------------------------------------------------
# 오프라인 — 구간 표 / 자기검사

def _plan(csv_path: str, segments: list, events_csv: str) -> int:
    """경로의 횡단보도 구간과 정지선의 거리를 낸다."""
    import os
    import stoplines
    from pure_pursuit import load_path

    if not segments:
        segments = []
        with open(csv_path, newline='', encoding='utf-8') as fh:
            for r in csv.DictReader(fh):
                if r['segment_id'] not in segments:
                    segments.append(r['segment_id'])
    path, _b = load_path(csv_path, segments)
    zones, warn = load_for_path(path, events_csv)
    print(f'[cw] {os.path.basename(csv_path)}  {len(path.pts)}점 '
          f'{path.length:.1f}m')
    print(f'[cw] 횡단보도 {len(zones)}개 (구간 길이 '
          f'{2 * (HALF_M + EDGE_MARGIN):.1f}m)')
    for line in summary(zones):
        print('[cw]' + line)
    for w in warn:
        print(f'[cw]   *** {w} ***')

    sl_list, _w = stoplines.load_for_path(path, events_csv)
    print('[cw] 정지선과의 거리 ― 이 값이 크면 신호 정지는 위험하지 않다')
    for sl in sl_list:
        near = min(zones, key=lambda c: abs(c.s - sl.s)) if zones else None
        if near is None:
            continue
        print(f'[cw]   정지선 s={sl.s:7.1f}  가장 가까운 횡단보도 '
              f's={near.s:7.1f}  거리 {sl.s - near.s:+7.1f}m')
    return 0


class _Rec:
    def __init__(self):
        self.ev = []

    def __call__(self, kind, detail):
        self.ev.append((kind, detail))

    def kinds(self):
        return [k for k, _d in self.ev]


def _drive(policy, *, s0=0.0, v0=12.5, v_target=12.5, ext_stop_s=None,
           front_clear=None, tl_stop_ahead=False, ext_hold=False,
           dt=0.05, n=600):
    """pure_pursuit의 합류 산술을 그대로 흉내낸다.

    ext_stop_s는 stoplines/obstacles가 겨냥한 후륜축 정지 지점이다.
    ext_hold면 그 지점과 무관하게 외부가 v_limit=0을 계속 낸다 (앞차가
    영영 안 비키는 상황).
    """
    s, v, t = s0, v0, 0.0
    acts, hist = [], []
    for _ in range(n):
        stop_s = float('nan') if ext_stop_s is None else ext_stop_s
        cmd = policy.update(s, v, t, stop_s=stop_s,
                            tl_stop_ahead=tl_stop_ahead,
                            front_clear=front_clear)
        acts.append(cmd.action)

        v_cmd, a_lim = v_target, float('inf')
        if ext_hold:
            v_cmd = 0.0
        elif ext_stop_s is not None:
            ev, ea = _profile(ext_stop_s - s, v)     # obstacles의 정지 프로파일
            v_cmd, a_lim = min(v_cmd, ev), ea
        v_cmd = min(v_cmd, cmd.v_limit)
        if cmd.escape:
            v_cmd = max(v_cmd, cmd.escape_v)         # min()을 깨는 그 한 줄

        a = min(1.2 * (v_cmd - v), cmd.accel_limit, a_lim)
        if v_cmd <= 0.0:
            a = min(a, -1.5)
        a = max(-6.0, min(3.0, a))
        v = max(0.0, v + a * dt)
        s += v * dt
        t += dt
        hist.append((t, s, v, cmd.action))
    return s, v, acts, hist


def _selftest() -> int:
    ok = True

    def check(name, cond, detail=''):
        nonlocal ok
        ok = ok and bool(cond)
        print(f'  {"PASS" if cond else "FAIL"}  {name}'
              + (f'  - {detail}' if detail else ''))

    print('횡단보도 정차 금지 자기검사 (항목 12)')

    def zone(center):
        return Crosswalk(0, center, 0.0, 0.0, 0.0)

    z = zone(150.0)                     # 구간 [147.0, 153.0]

    # 1. 예방 — 정지 목표가 구간 위면 구간 앞에 선다
    rec = _Rec()
    pol = CrosswalkPolicy(None, [z], on_event=rec)
    s, v, acts, _h = _drive(pol, s0=60.0, ext_stop_s=150.0)
    check('예방: 횡단보도 앞에 선다', not (z.s0 <= s <= z.s1),
          f'정지 s={s:.2f} (구간 {z.s0:.1f}~{z.s1:.1f})')
    check('예방: 목표 지점 근처에 선다', abs(s - (z.s0 - PRE_MARGIN)) < 1.0,
          f's={s:.2f} 목표 {z.s0 - PRE_MARGIN:.1f}')
    check('예방: guard 발화', 'xw_guard' in rec.kinds(),
          ','.join(sorted(set(rec.kinds()))))

    # 2. 정지 지령이 없으면 아무 제약도 걸지 않는다
    pol = CrosswalkPolicy(None, [z])
    s, v, acts, _h = _drive(pol, s0=60.0, ext_stop_s=None)
    check('정지 지령이 없으면 무제약', 'guard' not in acts and s > 300.0,
          f's={s:.1f} v={v * 3.6:.0f}km/h')

    # 3. 예방 불가 — 급제동해야 하면 포기한다
    rec = _Rec()
    pol = CrosswalkPolicy(None, [z], on_event=rec)
    _s, _v, acts, _h = _drive(pol, s0=142.0, ext_stop_s=150.0)
    check('예방 불가: 급제동하지 않는다',
          'xw_guard_infeasible' in rec.kinds() and 'guard' not in acts,
          ','.join(sorted(set(rec.kinds()))))

    # 4. 탈출 — 전방이 비어 있으면 저속으로 빠져나간다
    rec = _Rec()
    pol = CrosswalkPolicy(None, [z], on_event=rec)
    s, _v, acts, _h = _drive(pol, s0=150.0, v0=0.0, ext_hold=True,
                             front_clear=30.0)
    check('탈출: 구간을 벗어난다', s > z.s1 + EXIT_M,
          f's={s:.2f} (필요 {z.s1 + EXIT_M:.1f})')
    check('탈출: crosswalk_escape 발화', 'crosswalk_escape' in rec.kinds(),
          ','.join(sorted(set(rec.kinds()))))
    esc_i = [i for i, a in enumerate(acts) if a == 'escape']
    check('탈출: 3초 요건 전에 움직이기 시작한다',
          esc_i and esc_i[0] * 0.05 < 3.0, f'{esc_i[0] * 0.05:.2f}초')

    # 5~7. 발동 금지 조건 — 하나라도 어긋나면 -3을 받아들인다
    for name, kw in (('전방 여유 부족 (항목 14)', dict(front_clear=3.0)),
                     ('적색 정지선이 앞 (항목 7)',
                      dict(front_clear=30.0, tl_stop_ahead=True)),
                     ('obstacles 비활성', dict(front_clear=None))):
        rec = _Rec()
        pol = CrosswalkPolicy(None, [z], on_event=rec)
        s, _v, acts, _h = _drive(pol, s0=150.0, v0=0.0, ext_hold=True, **kw)
        check(f'탈출 금지: {name}',
              'escape' not in acts and z.s0 <= s <= z.s1
              and 'xw_escape_blocked' in rec.kinds(),
              f's={s:.2f} {",".join(sorted(set(rec.kinds())))}')

    # 8. 통과 중에는 정차 감시만 하고 아무 제약도 없다
    pol = CrosswalkPolicy(None, [z])
    _s, _v, acts, _h = _drive(pol, s0=140.0, ext_stop_s=None)
    check('통과 중 제약 없음', set(acts) <= {'none', 'on'},
          ','.join(sorted(set(acts))))

    # 9. 실제 이벤트 CSV — 경로 2의 횡단보도 4개
    import os
    from pure_pursuit import load_path
    here = os.path.dirname(os.path.abspath(__file__))
    wp = os.path.normpath(os.path.join(here, '..', 'data',
                                       'route_path_pt2.csv'))
    ev = os.path.normpath(os.path.join(here, '..', 'data',
                                       'route_path_pt2_events.csv'))
    if os.path.exists(wp) and os.path.exists(ev):
        segs = []
        with open(wp, newline='', encoding='utf-8') as fh:
            for r in csv.DictReader(fh):
                if r['segment_id'] not in segs:
                    segs.append(r['segment_id'])
        path, _b = load_path(wp, segs)
        zs, ws = load_for_path(path, ev)
        check('경로 2 횡단보도 4개 로드', len(zs) == 4 and not ws,
              f'{len(zs)}개 ' + ' '.join(f'{c.s:.1f}' for c in zs))

    print('\n자기검사 ' + ('전부 통과' if ok else '실패 있음'))
    return 0 if ok else 1


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(
        description='횡단보도 구간 확인 / 자기검사 (항목 12)')
    ap.add_argument('--plan', action='store_true',
                    help='경로의 횡단보도 구간과 정지선 거리를 낸다')
    ap.add_argument('--path', default='', help='waypoint CSV')
    ap.add_argument('--segments', default='',
                    help='쉼표로 구분한 구간 이름. 생략하면 파일에 있는 전부')
    ap.add_argument('--events', default='', help='이벤트 CSV')
    ap.add_argument('--selftest', action='store_true',
                    help='로그에 없는 상황을 합성해 확인한다')
    args = ap.parse_args()

    if args.selftest:
        raise SystemExit(_selftest())
    if not args.plan or not args.path:
        ap.error('--plan --path를 함께 주거나 --selftest를 쓸 것')
    ev = args.events
    if not ev:
        import stoplines
        ev = stoplines.default_events_path(args.path)
    segs = [s.strip() for s in args.segments.split(',') if s.strip()]
    raise SystemExit(_plan(args.path, segs, ev))


if __name__ == '__main__':
    main()
