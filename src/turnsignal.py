#!/usr/bin/env python3
"""방향지시등 (평가항목 13).

기준은 "차로 변경 시 3초 내 미점등이면 경미 -3점"이다. **차로 변경만 채점
대상이다.** 교차로 회전 점등은 항목 어디에도 없다.

stoplines.py / obstacles.py와 같은 구조다 ― I/O가 없는 순수 로직이고, 매
프레임 결과 하나(TurnCmd)를 낸다. **다만 이 모듈만 v_limit을 내지 않는다.**
방향지시등은 속도에 아무 영향이 없으므로 min() 합류에 끼지 않고 CTRL의
3번째 바이트로만 나간다.

--- 점등 구간은 차로 변경뿐이다 ---

생성 경로 CSV의 lc_dir 컬럼. 다만 이것은 **한 행짜리 마커**다(xodr_graph.py,
노드를 떠나는 마지막 waypoint 한 행). 실제 기하는 그 행을 **중심으로 ±10m
블렌드**되어 있다(LC_BLEND=20.0). 그래서 구간은 마커 ± LC_BLEND_M/2다.
pure_pursuit.load_path는 segment_id/x/y만 읽고 lc_dir을 버리므로 이 모듈이
같은 CSV를 다시 열어 (x,y)로 스냅한다. 우회 탈출 블렌드(detour._mark_escape_blend)
도 같은 마커를 심으므로 같은 코드로 켜진다.

**9/11까지는 교차로 회전(경로 헤딩 누적)에도 켰다.** 실차에서 두 가지로 틀렸다 ―
도로 곡률(-37°/-25°)을 회전으로 잡아 직진 구간에서 우측을 켰고, 회전 점등이
차로 변경 점등과 겹쳐 상충 해소 장치(takeover)가 필요해졌다. 채점 대상이
아닌 점등이 채점 대상을 방해한 것이라 **회전 검출을 통째로 뺐다.** 상충
해소·인계 로직도 같이 사라졌다 ― 겹치는 차로 변경끼리는 먼저 시작하는
구간이 이긴다.

--- lc_dir 마커의 부호를 lane_id로 교차검증한다 ---

xodr_graph.py가 9/11까지 **양수 차로의 lc_dir을 반대로** 냈다(양수 차로는 -s로
달리므로 참조선에서 먼 쪽이 운전자의 우측인데 좌측으로 봤다). 모든 경로 CSV의
양수 차로 마커 5개가 전부 뒤집혀 있었다(lt 행 671 / pt1 699·717 / pt2 169 /
ex 554). 생성기는 고쳤지만 옛 CSV(pt1은 재현 불가)와 대회 당일의 방어를 위해
**로드 시 마커 행과 다음 행의 lane_id로 방향을 다시 유도해, 다르면 경고하고
정정한다.** 규칙은 부호와 무관하게 "|lane_id|가 커지면 우측"이다.

--- 래치 ---

lead가 속도에 비례하므로(3.5초 x v) 접근 중 감속하면 lead가 줄어 껐다 켰다
한다. **한 번 켜지면 구간이 끝날 때까지 래치해서** 그 채터링을 막는다.
리스폰 시 reset()으로 버린다.

--- 정차 중 소등 ---

실차에서 **정지한 채 램프가 11~35초 켜져 있었다** (9/9 저녁, drive.csv에서
turn!=0 이면서 speed<1km/h인 연속 구간). 멈춰 있는데 방향을 알릴 이유가 없고
잘못된 정보를 준다. 그래서 PAUSE_AFTER_S 이상 정지해 있으면 끈다.

**단 구간 시작이 RELIGHT_M 안이면 끄지 않는다.** 항목 13은 "구간 시작 직전의
연속 점등 3초"로 채점되므로, 구간 코앞에서 껐다가 재출발하면 그 3초를 잃는다.
20m는 지령 상한 3.0m/s^2에서도 3.65초가 걸려 재출발 뒤에 3초가 남는다.
소등/재점등은 turn_pause / turn_resume로 남긴다 ― 구간을 지나서 끄는
turn_off와 구분해야 사후에 헷갈리지 않는다.
"""

from __future__ import annotations

import bisect
import csv
import math
import os

from hlvtd_client import TURN_OFF, TURN_LEFT, TURN_RIGHT
from stoplines import SNAP_MAX_M, _path_heading

# 점등 시점
LEAD_S = 3.5                # s, 요건 3초에 대한 여유. 등속이면 정확히 이만큼 앞선다
LEAD_MIN_M = 15.0           # m, 저속에서의 하한 (3.5s x 4m/s = 14m)
OFF_EXTRA_M = 2.0           # m, 구간이 끝난 뒤 이만큼 더 켜 두고 끈다

# 차선변경 구간
LC_BLEND_M = 20.0           # m, xodr_graph.LC_BLEND. 마커를 중심으로 이만큼 섞인다
SNAP_WARN_M = 1.5           # m, 이보다 크면 경고만 (stoplines와 같은 규약)
LC_LANE_LOOK = 3            # 행, 마커 뒤 이만큼 안에서 새 차로 행을 찾는다

# 기하 보조 (detour.py의 selftest가 _lateral_ahead를 쓴다)
HEAD_SPAN_M = 3.0           # m, 접선을 뽑을 때 앞서 볼 거리

# 정차 중 소등 — "멈춰 있는데 방향을 알릴 이유가 없다"
STOP_MS = 1.0 / 3.6         # m/s, 1km/h. crosswalks.DWELL_MS와 같은 값
PAUSE_AFTER_S = 3.0         # s, 이만큼 정지해 있으면 끈다
RELIGHT_M = 20.0            # m, **구간 시작이 이 안이면 끄지 않는다.**
                            # 정지에서 20m는 지령 상한 3.0m/s^2로도 3.65초라
                            # 재출발 뒤에도 항목 13의 선행 3초가 남는다

SIGNAL_NAME = {TURN_OFF: 'OFF', TURN_LEFT: 'LEFT', TURN_RIGHT: 'RIGHT'}
SIGNAL_OF = {'left': TURN_LEFT, 'right': TURN_RIGHT}
KIND_NAME = {'lc': '차선변경'}


class TurnZone:
    """점등해야 하는 경로 구간 하나."""

    __slots__ = ('kind', 'dir', 'signal', 's_start', 's_end', 'x', 'y', 'note')

    def __init__(self, kind, direction, s_start, s_end, x, y, note=''):
        self.kind = kind            # 'lc' (차선변경) 하나뿐이다. 계수기 호환용
        self.dir = direction        # 'left' / 'right'
        self.signal = SIGNAL_OF[direction]
        self.s_start = s_start      # 기동이 시작되는 지점 [m]. lead는 여기서 뺀다
        self.s_end = s_end          # 기동이 끝나는 지점 [m]
        self.x, self.y = x, y
        self.note = note

    @property
    def length(self) -> float:
        return self.s_end - self.s_start

    def __repr__(self) -> str:
        return (f'<TurnZone {self.kind} {self.dir} '
                f'{self.s_start:.1f}~{self.s_end:.1f}m>')


class TurnCmd:
    """한 프레임의 판정 결과."""

    __slots__ = ('signal', 'action', 'src', 'dist', 'reason')

    def __init__(self, signal=TURN_OFF, action='none', src='',
                 dist=float('nan'), reason=''):
        self.signal = signal        # TURN_OFF / TURN_LEFT / TURN_RIGHT
        self.action = action        # none / on / pause / error
        self.src = src              # 'lc' / 'pause'
        self.dist = dist            # 구간 시작까지 [m]. 구간 안이면 음수
        self.reason = reason

    def __repr__(self) -> str:
        return (f'<TurnCmd {SIGNAL_NAME.get(self.signal, self.signal)} '
                f'{self.src} d={self.dist:.1f}>')


# ----------------------------------------------------------------------
# 로드

def lc_dir_from_lanes(a, b) -> str:
    """lane_id a에서 b로 옮길 때의 방향. 판정 불가면 ''.

    부호와 무관하게 **|id|가 커지면(참조선에서 멀어지면) 우측**이다. 음수 차로는
    +s로 달리고 왼쪽이 +t라 |id| 증가(-t 방향)가 우측이고, 양수 차로는 -s로
    달려 왼쪽이 -t라 |id| 증가(+t 방향)가 역시 우측이다. 부호가 다르거나
    같으면(대향 차로 / 같은 차로) 차로 변경이 아니다.
    """
    try:
        a, b = int(a), int(b)
    except (TypeError, ValueError):
        return ''
    if a == 0 or b == 0 or a == b or (a > 0) != (b > 0):
        return ''
    return 'right' if abs(b) > abs(a) else 'left'


def _lane_after(rows: list, i: int):
    """마커 행 i 뒤 LC_LANE_LOOK 행 안에서 같은 도로·섹션의 다른 차로 행."""
    r = rows[i]
    for j in range(i + 1, min(len(rows), i + 1 + LC_LANE_LOOK)):
        n = rows[j]
        if (n.get('segment_id') != r.get('segment_id')
                or n.get('road_id') != r.get('road_id')
                or n.get('section') != r.get('section')):
            return None
        if n.get('lane_id') != r.get('lane_id'):
            return n
    return None


def _zones_from_rows(path, rows: list) -> tuple[list, list]:
    """CSV 행 전부(dict) -> (구간 목록, 경고 목록). load_lane_changes의 본체."""
    out, warn = [], []
    for i, r in enumerate(rows):
        direction = (r.get('lc_dir') or '').strip()
        if direction not in ('left', 'right'):
            continue
        x, y = float(r['x']), float(r['y'])
        solid = (r.get('lc_solid') or '0').strip() == '1'

        # lane_id 교차검증. 다음 행이 같은 도로·섹션의 다른 차로면 그 둘로
        # 방향을 다시 유도한다. 우회 탈출 마커(차로가 안 바뀐다)는 조용히 통과
        note = ''
        nxt = _lane_after(rows, i)
        if nxt is not None:
            want = lc_dir_from_lanes(r.get('lane_id'), nxt.get('lane_id'))
            if want:
                note = f'{r.get("lane_id")}->{nxt.get("lane_id")}'
                if want != direction:
                    warn.append(
                        f'차선변경 마커 ({x:.1f},{y:.1f}) lc_dir={direction} 인데 '
                        f'road {r.get("road_id")} 차로 {note} '
                        f'({"바깥" if want == "right" else "안쪽"})이라 '
                        f'{want}로 정정한다 ― CSV가 오래됐다 (부호 버그)')
                    direction = want

        i_p = path.nearest(x, y)
        resid = math.hypot(path.pts[i_p][0] - x, path.pts[i_p][1] - y)
        if resid > SNAP_MAX_M:
            warn.append(f'차선변경 마커 ({x:.1f},{y:.1f}) {direction}를 버린다 ― '
                        f'경로에서 {resid:.1f}m (한계 {SNAP_MAX_M:.0f}m)')
            continue
        if resid > SNAP_WARN_M:
            warn.append(f'차선변경 마커 ({x:.1f},{y:.1f}) 스냅 잔차 {resid:.1f}m')
        s = path.s[i_p]
        half = LC_BLEND_M * 0.5
        if solid:
            note = (note + ' 실선(항목 6 위험)').strip()
        out.append(TurnZone('lc', direction,
                            max(0.0, s - half), min(path.length, s + half),
                            path.pts[i_p][0], path.pts[i_p][1], note))
    return out, warn


def load_lane_changes(path, waypoints_csv: str) -> tuple[list, list]:
    """경로 CSV의 lc_dir 마커를 구간으로 바꾼다. (구간 목록, 경고 목록).

    stoplines.load_for_path와 같은 이유로 **(x,y)로 스냅한다** ― --segments가
    부분집합이면 행 순서와 경로 인덱스가 어긋나기 때문이다.

    lc_dir 컬럼이 없는 CSV(수동 녹화본 waypoints_*.csv)면 조용히 빈 목록을
    낸다. 그 경로에는 차선변경 정보가 없다는 뜻이라 경고할 일이 아니다.
    """
    if not waypoints_csv or not os.path.exists(waypoints_csv):
        return [], []
    with open(waypoints_csv, newline='', encoding='utf-8') as fh:
        rdr = csv.DictReader(fh)
        if 'lc_dir' not in (rdr.fieldnames or []):
            return [], []
        rows = list(rdr)
    return _zones_from_rows(path, rows)


def _lateral_ahead(path, s_ego: float, span_m: float) -> float:
    """지금 헤딩 기준으로 span_m 앞 경로점의 부호 횡변위 [m]. **좌가 양수.**

    부호 규약은 obstacles._project / route_check.nearest_on_route와 같다.
    헤딩은 stoplines._path_heading을 그대로 쓴다 (길이 0 선분을 이미 방어한다).
    런타임 판정에는 더 이상 쓰지 않는다 ― --plan의 기하 힌트와 detour.py의
    탈출 블렌드 마커 부호 검사(selftest)가 쓴다.
    """
    n = len(path.pts)
    i0 = max(0, min(n - 1, bisect.bisect_right(path.s, s_ego) - 1))
    i1 = min(n - 1, bisect.bisect_left(path.s, path.s[i0] + span_m))
    if i1 <= i0:
        return 0.0
    th = _path_heading(path, i0, HEAD_SPAN_M)
    dx = path.pts[i1][0] - path.pts[i0][0]
    dy = path.pts[i1][1] - path.pts[i0][1]
    return -math.sin(th) * dx + math.cos(th) * dy


def _merge(zones: list) -> list:
    """겹치는 같은 방향 구간을 하나로 합친다.

    경로 1의 683.5m / 701.8m 연속 차선변경 2회가 여기서 한 구간이 된다.
    18m 떨어져 있어 블렌드가 겹치는데, 나눠 두면 중간에 한 프레임 꺼진다.
    """
    zones = sorted(zones, key=lambda z: z.s_start)
    out = []
    for z in zones:
        if out:
            p = out[-1]
            if p.kind == z.kind and p.dir == z.dir and z.s_start <= p.s_end:
                p.s_end = max(p.s_end, z.s_end)
                if z.note and z.note not in p.note:
                    p.note = (p.note + ' ' + z.note).strip()
                continue
        out.append(z)
    return out


def build_zones(path, waypoints_csv: str) -> tuple[list, list]:
    """점등 구간 전부 (차선변경만). (구간 목록, 경고 목록)."""
    zones, warn = load_lane_changes(path, waypoints_csv)
    zones = _merge(zones)
    zones.sort(key=lambda z: z.s_start)
    return zones, warn


def summary(zones: list) -> list:
    """시작 시 콘솔에 뿌릴 요약 줄."""
    lines = []
    for z in zones:
        lines.append(f'  s={z.s_start:7.1f}~{z.s_end:7.1f}m  '
                     f'{KIND_NAME[z.kind]:6s} {z.dir:5s}  '
                     f'({z.x:.1f},{z.y:.1f})'
                     + (f'  {z.note}' if z.note else ''))
    return lines


# ----------------------------------------------------------------------
# 정책

class TurnSignalPolicy:
    """매 프레임 turnSignal 바이트 하나를 낸다."""

    def __init__(self, path, zones, *, lead_s: float = LEAD_S,
                 lead_min: float = LEAD_MIN_M, v_ref: float = 0.0,
                 on_event=None):
        self.path = path
        self.zones = list(zones)
        self.lead_s = lead_s
        self.lead_min = lead_min
        # 지시속도 [m/s]. lead를 **현재 속도가 아니라 낼 수 있는 최고 속도**로
        # 잡기 위한 값이다. 현재 속도만 쓰면 저속에서 켜고 가속해 들어갈 때
        # 3초를 못 채운다 — v=5m/s에서 lead 17.5m를 켜고 12.5m/s로 가속하면
        # 2.0초 만에 도달한다. v_ref를 쓰면 이동거리가 lead_s*v_ref이고 속도가
        # v_ref를 크게 넘지 않으므로 시간이 lead_s 아래로 내려가지 않는다.
        self.v_ref = v_ref
        self._on_event = on_event or (lambda kind, detail: None)
        self.reset()

    def reset(self) -> None:
        """리스폰 시 호출한다. 래치를 버린다."""
        self._active = None
        self._carry = None          # 경로 교체 직전의 점등 (signal, 점등 시각)
        self._on_t = 0.0
        self._on_s = 0.0
        self._stop_t = None         # 정지가 시작된 시각. 움직이면 None
        self._paused = False        # 정차 소등 중인가

    def adopt(self, prev, t: float) -> None:
        """경로 교체(우회) 시 직전 정책의 점등 상태를 이어받는다.

        **램프는 실제로 꺼지지 않는다.** 새 정책이 점등 시각을 0으로 두면
        로그에만 소등/점등이 찍히고 항목 13이 요구하는 "연속 점등 시간"도
        끊겨 보인다. _switch의 same 분기와 같은 취지다. 구간(_active)은 옛
        경로의 s 기준이라 가져오지 않고 방향과 시각만 넘겨받는다.
        """
        if prev is None or prev._active is None or prev._paused:
            return                  # 정차 소등 중이면 램프는 실제로 꺼져 있다
        self._carry = (prev._active.signal, prev._on_t)

    def _finish(self, t: float, s_ego: float) -> None:
        if self._active is None:
            return
        z = self._active
        self._on_event('turn_off',
                       f'{KIND_NAME[z.kind]} {z.dir} 소등 ― '
                       f'{t - self._on_t:.1f}초 / {s_ego - self._on_s:.1f}m 점등')
        self._active = None

    def _switch(self, z, t: float, s_ego: float, v: float,
                lead: float) -> None:
        """활성 구간을 z로 바꾼다.

        **방향이 같으면 램프는 실제로 꺼지지 않는다.** 그때 소등/점등 이벤트를
        내면 로그에 없는 깜빡임이 있는 것처럼 보이고, 항목 13이 요구하는
        "연속 점등 시간"도 끊겨 보인다. 그래서 구간만 갈아 끼우고 점등 시각은
        유지한다.
        """
        same = (self._active is not None
                and self._active.signal == z.signal)
        if not same and self._carry is not None:
            if self._carry[0] == z.signal:      # 경로 교체 전과 같은 방향이다
                same = True
                self._on_t, self._on_s = self._carry[1], s_ego
            self._carry = None
        if not same:
            self._finish(t, s_ego)
            self._on_t, self._on_s = t, s_ego
        self._active = z
        self._on_event(
            'turn_on',
            f'{KIND_NAME[z.kind]} {z.dir} '
            + ('구간 인계' if same else '점등')
            + f' ― 구간 시작 {z.s_start - s_ego:.1f}m 앞 '
              f'(v={v * 3.6:.0f}km/h, lead {lead:.1f}m)')

    def _pause(self, best, s_ego: float, v: float, t: float) -> bool:
        """정차 중이면 램프를 끈다. 껐으면 True.

        실차에서 **정지한 채 램프가 11~35초 켜져 있었다.** 멈춰 있는데 방향을
        알릴 이유가 없고 잘못된 정보를 준다.

        **구간 시작이 RELIGHT_M 안이면 끄지 않는다.** 항목 13은 "구간 시작
        직전의 연속 점등 3초"로 채점되므로 구간 코앞에서 껐다가 재출발하면
        그 3초를 잃는다. 20m는 지령 상한 3.0m/s^2로도 3.65초라 재출발 뒤에
        3초가 남는다. 실제로 차로를 바꾸려고 서 있는 상태이기도 하다.
        """
        if v >= STOP_MS:
            self._stop_t = None
        elif self._stop_t is None:
            self._stop_t = t
        held = 0.0 if self._stop_t is None else t - self._stop_t
        want = (held >= PAUSE_AFTER_S
                and best.s_start - s_ego > RELIGHT_M)
        if want == self._paused:
            return self._paused
        gap = best.s_start - s_ego
        if want:
            self._on_event(
                'turn_pause',
                f'{KIND_NAME[best.kind]} {best.dir} 정차 소등 ― '
                f'{held:.1f}초 정지, 구간 시작 {gap:.1f}m 앞'
                f'(재점등 한계 {RELIGHT_M:.0f}m). '
                f'{t - self._on_t:.1f}초 점등했다')
        else:
            self._on_event(
                'turn_resume',
                f'{KIND_NAME[best.kind]} {best.dir} 재점등 ― '
                f'{held:.1f}초 정지 뒤 v={v * 3.6:.0f}km/h, '
                f'구간 시작 {gap:.1f}m 앞')
        self._paused = want
        return want

    def update(self, s_ego: float, v: float, t: float) -> TurnCmd:
        lead = max(self.lead_s * max(v, self.v_ref), self.lead_min)

        cands = []
        for z in self.zones:
            if s_ego > z.s_end + OFF_EXTRA_M:
                continue                        # 지나갔다
            # 래치: 한 번 켜진 구간은 lead가 줄어도 끝날 때까지 유지한다
            if z is self._active or s_ego >= z.s_start - lead:
                cands.append(z)

        if not cands:
            self._paused = False
            self._finish(t, s_ego)
            if self._carry is not None:
                self._on_event('turn_off',
                               f'경로 교체로 이어받은 점등 소등 ― '
                               f'{t - self._carry[1]:.1f}초 점등')
                self._carry = None
            return TurnCmd()

        # 먼저 시작하는 구간(지금 벌어지는 일)이 이긴다. 겹치는 반대 방향
        # 차선변경은 앞 구간이 끝나야 넘어간다
        cands.sort(key=lambda z: z.s_start)
        best = cands[0]

        if self._pause(best, s_ego, v, t):
            # 소등 중에도 활성 구간은 조용히 따라간다. 지나간 구간이 _active로
            # 남으면 후보 래치가 오염되고, 점등 시각을 현재로 밀어 두어야
            # 재점등 뒤의 연속 점등 시간이 그 순간부터 세진다.
            self._active = best
            self._on_t, self._on_s = t, s_ego
            # src를 'pause'로 둔다 -- drive.csv의 turn_src만 보고도 "왜 꺼져
            # 있나"가 바로 갈린다 (구간이 없어서 0인 것과 구분된다).
            return TurnCmd(TURN_OFF, 'pause', 'pause',
                           best.s_start - s_ego,
                           f'{KIND_NAME[best.kind]} {best.dir} 정차 소등')

        if best is not self._active:
            self._switch(best, t, s_ego, v, lead)

        return TurnCmd(best.signal, 'on', best.kind, best.s_start - s_ego,
                       f'{KIND_NAME[best.kind]} {best.dir}')


# ----------------------------------------------------------------------
# 오프라인 — 구간 표 / 자기검사

def _segments_of(csv_path: str) -> list:
    """파일에 나오는 순서대로 구간 이름 전부."""
    segs = []
    with open(csv_path, newline='', encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            if r['segment_id'] not in segs:
                segs.append(r['segment_id'])
    return segs


def _plan(csv_path: str, segments: list) -> int:
    """경로의 점등 구간 표를 낸다. 현장에서 4번 단계 전에 눈으로 확인할 용도.

    구간마다 lane_id(마커 행 -> 다음 행)와 **블렌드 구간의 실제 횡이동**
    (_lateral_ahead, 좌 양수)을 같이 낸다. 횡이동 부호가 방향과 다르면 `?`를
    붙인다 ― 도로 곡률이 섞여 애매할 수 있으므로 경고가 아니라 힌트다.
    """
    from pure_pursuit import load_path

    segments = segments or _segments_of(csv_path)
    path, _bounds = load_path(csv_path, segments)
    zones, warn = build_zones(path, csv_path)
    print(f'[ts] {os.path.basename(csv_path)}  {len(path.pts)}점 '
          f'{path.length:.1f}m  구간 {len(segments)}개')
    print(f'[ts] 점등 구간 {len(zones)}개 (차선변경만 ― 교차로 회전은 켜지 않는다)')
    for z in zones:
        lat = _lateral_ahead(path, z.s_start, LC_BLEND_M)
        agree = (lat > 0.0) == (z.dir == 'left')
        hint = '' if abs(lat) < 1.0 else ('' if agree else '  ? 기하 부호 반대')
        print(f'[ts]   s={z.s_start:7.1f}~{z.s_end:7.1f}m  {z.dir:5s}  '
              f'({z.x:.1f},{z.y:.1f})  차로 {z.note or "-":14s} '
              f'횡이동 {lat:+.1f}m{hint}')
    for w in warn:
        print(f'[ts]   *** {w} ***')
    return 0


class _Rec:
    """이벤트 수집기."""

    def __init__(self):
        self.ev = []

    def __call__(self, kind, detail):
        self.ev.append((kind, detail))

    def kinds(self):
        return [k for k, _d in self.ev]


def _straight(length=400.0, step=1.0):
    from pure_pursuit import Path
    n = int(length / step) + 1
    return Path([(i * step, 0.0) for i in range(n)])


def _run(policy, reach_at, v_of, length=400.0, dt=0.05):
    """v_of(t)로 달리며 (신호가 처음 켜진 t, 지점에 닿은 t, 궤적)를 낸다."""
    t, s = 0.0, 0.0
    first_on, hist = {}, []
    while s < length and t < 400.0:
        v = v_of(t)
        cmd = policy.update(s, v, t)
        hist.append((t, s, cmd.signal))
        if cmd.signal != TURN_OFF and cmd.signal not in first_on:
            first_on[cmd.signal] = t
        s += v * dt
        t += dt
    reach = {}
    for sig, s0 in reach_at.items():
        for tt, ss, _g in hist:
            if ss >= s0:
                reach[sig] = tt
                break
    return first_on, reach, hist


def _selftest() -> int:
    from pure_pursuit import Path

    ok = True

    def check(name, cond, detail=''):
        nonlocal ok
        ok = ok and bool(cond)
        print(f'  {"PASS" if cond else "FAIL"}  {name}'
              + (f'  - {detail}' if detail else ''))

    print('방향지시등 자기검사 (항목 13)')

    # 1~3. 차선변경 선행 점등이 3초 요건을 넘는가 (등속·감속·가속)
    path = _straight()
    zone = TurnZone('lc', 'left', 150.0, 170.0, 150.0, 0.0)
    for name, v_of in (('등속 45km/h', lambda t: 12.5),
                       ('감속 45->10km/h', lambda t: max(2.8, 12.5 - 0.5 * t)),
                       ('가속 10->45km/h', lambda t: min(12.5, 2.8 + 0.5 * t))):
        pol = TurnSignalPolicy(path, [zone], v_ref=12.5)
        on, reach, _h = _run(pol, {TURN_LEFT: 150.0}, v_of)
        lead_t = reach.get(TURN_LEFT, 0.0) - on.get(TURN_LEFT, 0.0)
        check(f'차선변경 선행 점등 3초 이상 ({name})', lead_t >= 3.0,
              f'{lead_t:.2f}초')

    # 4. 반대 방향 차선변경이 겹치면 먼저 시작하는 구간이 이긴다
    zl = [TurnZone('lc', 'right', 100.0, 120.0, 100.0, 0.0),
          TurnZone('lc', 'left', 130.0, 150.0, 130.0, 0.0)]
    rec = _Rec()
    pol = TurnSignalPolicy(path, zl, v_ref=12.5, on_event=rec)
    _on, _r, hist = _run(pol, {}, lambda t: 12.5)

    def at(s0):
        return next(g for _t, ss, g in hist if ss >= s0)

    check('우차선변경 중에는 우측 (좌 구간의 선행 창이 겹쳐도)',
          at(110.0) == TURN_RIGHT, SIGNAL_NAME[at(110.0)])
    check('우차선변경이 끝나면 좌측으로 전환', at(125.0) == TURN_LEFT,
          SIGNAL_NAME[at(125.0)])
    check('회전·상충 이벤트는 더 이상 없다',
          not ({'turn_conflict', 'turn_takeover'} & set(rec.kinds())),
          ','.join(sorted(set(rec.kinds()))))

    # 5. 같은 방향 연속 차선변경은 한 구간으로 합쳐진다 (경로 1의 683.5/701.8)
    merged = _merge([TurnZone('lc', 'right', 673.5, 693.5, 0, 0),
                     TurnZone('lc', 'right', 691.8, 711.8, 0, 0)])
    check('같은 방향 연속 차선변경 병합', len(merged) == 1,
          f'{len(merged)}개 {merged[0].s_start:.1f}~{merged[0].s_end:.1f}m')
    unmerged = _merge([TurnZone('lc', 'right', 100.0, 120.0, 0, 0),
                       TurnZone('lc', 'left', 110.0, 130.0, 0, 0)])
    check('방향이 다르면 병합하지 않는다', len(unmerged) == 2)

    # 6. 램프가 실제로 안 꺼지면 소등 이벤트를 내지 않는다
    rec2 = _Rec()
    pol2 = TurnSignalPolicy(path,
                            [TurnZone('lc', 'right', 100.0, 120.0, 0, 0),
                             TurnZone('lc', 'right', 140.0, 160.0, 0, 0)],
                            v_ref=12.5, on_event=rec2)
    _run(pol2, {}, lambda t: 12.5)
    offs = [d for k, d in rec2.ev if k == 'turn_off']
    check('같은 방향 인계에서 소등 이벤트 없음', len(offs) == 1,
          f'turn_off {len(offs)}회 (구간을 다 지난 뒤 1회만 정상)')

    # 7. lane_id -> 방향 규칙. 부호와 무관하게 |id| 증가가 우측이다
    table = ((-4, -5, 'right'), (-5, -4, 'left'), (2, 1, 'left'), (1, 2, 'right'),
             (2, 2, ''), (-1, 1, ''), ('', 3, ''), (None, None, ''))
    got = [lc_dir_from_lanes(a, b) for a, b, _w in table]
    check('lane_id 방향 규칙 (양수/음수 차로 모두 바깥=우측)',
          got == [w for _a, _b, w in table], ' '.join(g or "''" for g in got))

    # 8. 마커 교차검증 — 뒤집힌 양수 차로 마커를 정정하고 경고한다
    def rows_for(lane_a, lane_b, marker, x0=100.0):
        rows = []
        for i in range(0, 120):
            rows.append({'segment_id': 'p1', 'x': f'{i:.1f}', 'y': '0.0',
                         'road_id': '2467', 'section': '1',
                         'lane_id': str(lane_a if i <= x0 else lane_b),
                         'lc_dir': marker if i == int(x0) else '',
                         'lc_solid': '0'})
        return rows

    zs, ws = _zones_from_rows(path, rows_for(2, 1, 'right'))
    check('양수 차로 2->1 right 마커를 left로 정정', len(zs) == 1
          and zs[0].dir == 'left' and len(ws) == 1 and '정정' in ws[0],
          f'{zs} {ws}')
    zs, ws = _zones_from_rows(path, rows_for(-4, -5, 'right'))
    check('음수 차로 -4->-5 right 마커는 그대로', len(zs) == 1
          and zs[0].dir == 'right' and not ws, f'{zs} {ws}')
    zs, ws = _zones_from_rows(path, rows_for(2, 2, 'right'))
    check('차로가 안 바뀌는 마커(우회 탈출 블렌드)는 조용히 통과',
          len(zs) == 1 and zs[0].dir == 'right' and not ws, f'{zs} {ws}')

    # 9. lc_dir 컬럼이 없는 CSV는 조용히 빈 목록
    here = os.path.dirname(os.path.abspath(__file__))
    manual = os.path.normpath(
        os.path.join(here, '..', 'data', 'waypoints_pretest2.csv'))
    zs, ws = load_lane_changes(path, manual)
    check('lc_dir 없는 CSV는 경고 없이 빈 목록', not zs and not ws,
          f'{len(zs)}구간 {len(ws)}경고')

    # 10. 부호 규약 — 좌가 양수. obstacles._project / route_check와 같다
    def arc_path(sign=1.0, R=15.0):
        """직선 99m -> 90도 회전 -> 직선 60m. sign>0이 좌회전."""
        pts = [(i * 1.0, 0.0) for i in range(100)]
        for k in range(1, 40):
            a = k * (math.pi / 2) / 39
            pts.append((99.0 + R * math.sin(a), sign * R * (1.0 - math.cos(a))))
        for k in range(1, 61):
            pts.append((99.0 + R, sign * (R + k * 1.0)))
        return Path(pts)

    def sig_at(hist, s0):
        return next((g for _t, ss, g in hist if ss >= s0), TURN_OFF)

    left_p, right_p = arc_path(1.0), arc_path(-1.0)
    lat_l = _lateral_ahead(left_p, 100.0, 37.5)
    lat_r = _lateral_ahead(right_p, 100.0, 37.5)
    lat_s = _lateral_ahead(path, 100.0, 37.5)
    check('_lateral_ahead 부호 (좌회전 양수 / 우회전 음수 / 직선 0)',
          lat_l > 5.0 and lat_r < -5.0 and abs(lat_s) < 0.1,
          f'좌 {lat_l:+.1f}m / 우 {lat_r:+.1f}m / 직선 {lat_s:+.2f}m')

    # 11. 교차로 회전은 켜지 않는다 — 90도 좌회전 경로에 구간이 없다
    pol_t = TurnSignalPolicy(left_p, build_zones(left_p, '')[0], v_ref=12.5)
    _o, _r, hist_t = _run(pol_t, {}, lambda t: 12.5, length=175.0)
    check('교차로 회전에는 점등하지 않는다',
          all(g == TURN_OFF for _t, _s, g in hist_t))

    # 12~14. 실제 경로 회귀. 마커 정정 뒤의 방향으로 켜져야 한다
    from pure_pursuit import load_path
    for name, marks in (
            ('route_path_pt1.csv',
             ((690.0, TURN_LEFT, '좌측 차선변경 2회(옛 CSV right 마커 정정)'),
              (705.0, TURN_LEFT, '병합된 좌측 구간 안'),
              (744.3, TURN_RIGHT, '-2->-3 우측 차선변경 판정 시점'),
              (760.0, TURN_OFF, '구간 끝 +2m 뒤 소등'))),
            ('route_path_pt2.csv',
             ((160.0, TURN_LEFT, '3->2 좌측 차선변경 (정정)'),
              (600.0, TURN_OFF, '옛 회전 구간 ― 이제 꺼져 있다'),
              (670.0, TURN_LEFT, '-4->-3 좌측 차선변경'))),
            ('route_path_lt.csv',
             ((500.0, TURN_RIGHT, '-4->-5 우측 차선변경 선행'),
              (540.0, TURN_RIGHT, '우측 구간 안'),
              (600.0, TURN_LEFT, '2->1 좌측 차선변경 선행 (실차 결함 지점)'),
              (650.0, TURN_LEFT, '좌측 구간 안'),
              (720.0, TURN_OFF, '옛 좌회전 구간 ― 꺼져 있다'),
              (1550.0, TURN_OFF, '옛 -37도 곡률 오검출 ― 꺼져 있다'),
              (1660.0, TURN_OFF, '옛 -25도 곡률 오검출 ― 꺼져 있다'))),
    ):
        f = os.path.normpath(os.path.join(here, '..', 'data', name))
        if not os.path.exists(f):
            continue
        rp, _b = load_path(f, _segments_of(f))
        rz, _w = build_zones(rp, f)
        polr = TurnSignalPolicy(rp, rz, v_ref=12.5)
        _o, _r, histr = _run(polr, {}, lambda t: 12.5, length=rp.length - 1.0)
        for s0, want, why in marks:
            got = sig_at(histr, s0)
            check(f'{name} s={s0:.0f} {SIGNAL_NAME[want]} ({why})', got == want,
                  SIGNAL_NAME[got])
        with open(f, newline='', encoding='utf-8') as fh:
            rows = list(csv.DictReader(fh))
        raw = [(i, r) for i, r in enumerate(rows) if r.get('lc_dir')]
        bad = []
        for i, r in raw:
            nxt = _lane_after(rows, i)
            want = lc_dir_from_lanes(r['lane_id'], nxt['lane_id']) if nxt else ''
            if want and want != r['lc_dir']:
                bad.append(f'행{i} {r["lane_id"]}->{nxt["lane_id"]} {r["lc_dir"]}')
        check(f'{name} 로드된 구간 방향이 전부 lane_id 규칙과 일치',
              all(z.dir == lc_dir_from_lanes(*tok.split('->'))
                  for z in rz for tok in z.note.split() if '->' in tok),
              f'CSV 마커 뒤집힘 {len(bad)}개 (정정됨): {"; ".join(bad) or "없음"}')

    # 15. 경로 교체(우회) 시 점등 인계 — 램프는 실제로 꺼지지 않는다
    zr = TurnZone('lc', 'right', 100.0, 120.0, 100.0, 0.0)
    pol5 = TurnSignalPolicy(path, [zr], v_ref=12.5)
    for k in range(120):                    # s=0..75, 점등은 s=56.2부터
        pol5.update(k * 0.625, 12.5, k * 0.05)

    def after_rebind(carry):
        rec6 = _Rec()
        pol6 = TurnSignalPolicy(path, [TurnZone('lc', 'right', 100.0, 120.0,
                                                100.0, 0.0)],
                                v_ref=12.5, on_event=rec6)
        if carry:
            pol6.adopt(pol5, 6.0)
        for k in range(120, 260):
            pol6.update(k * 0.625, 12.5, k * 0.05)
        ons = [d for k, d in rec6.ev if k == 'turn_on']
        offs = [d for k, d in rec6.ev if k == 'turn_off']
        return ons, offs

    ons_c, offs_c = after_rebind(True)
    ons_n, offs_n = after_rebind(False)
    check('경로 교체에서 점등 시각을 이어받는다 (인계)',
          len(offs_c) == 1 and len(offs_n) == 1 and offs_c[0] != offs_n[0]
          and '구간 인계' in ons_c[0] and '구간 인계' not in ons_n[0],
          f'인계 {offs_c[0].split("― ")[-1]} / 미인계 '
          f'{offs_n[0].split("― ")[-1]}')

    # 16. 우회 탈출 블렌드 (detour._blend_start와 같은 기하) — 마커만으로 켜진다
    def blend_path(lat=3.0, blend=9.0, n=60):
        """ego가 lat만큼 왼쪽에 있는 상태에서 차로에 붙는 경로. lat>0이면 우측."""
        pts = []
        for i in range(n):
            u = min(1.0, i / blend)
            pts.append((float(i), lat * (1.0 - u * u * (3.0 - 2.0 * u))))
        return Path(pts)

    bp = blend_path()
    lcz = TurnZone('lc', 'right', 0.0, 14.5, 0.0, 0.0)
    rec7 = _Rec()
    pol7 = TurnSignalPolicy(bp, [lcz], v_ref=12.5, on_event=rec7)
    _o7, _r7, hist7 = _run(pol7, {}, lambda t: 12.5, length=40.0)
    lit = [sig_at(hist7, x) for x in (1.0, 4.0, 8.0, 12.0)]
    check('블렌드 내내 마커 방향(우)을 켠다',
          all(g == TURN_RIGHT for g in lit),
          ' '.join(SIGNAL_NAME[g] for g in lit))

    # ------------------------------------------------------------------
    # 17~20. 정차 중 소등 (실차에서 정지한 채 11~35초 켜져 있었다)

    def drive(pol, vs, dt=0.05):
        t, s0, hist = 0.0, 0.0, []
        for v in vs:
            cmd = pol.update(s0, v, t)
            hist.append((t, s0, cmd.signal))
            s0 += v * dt
            t += dt
        return hist

    def pause_case(cruise_steps, stop_steps, tail=None):
        z = TurnZone('lc', 'left', 150.0, 170.0, 150.0, 0.0)
        rec = _Rec()
        pol = TurnSignalPolicy(path, [z], v_ref=12.5, on_event=rec)
        vs = [12.5] * cruise_steps + [0.0] * stop_steps + (tail or [])
        return rec, drive(pol, vs)

    rec8, hist8 = pause_case(176, 120)          # s=110 (구간 40m 앞) 에서 6초 정지
    n_pause = len([1 for k, _d in rec8.ev if k == 'turn_pause'])
    check('구간 40m 앞에서 정지하면 소등한다',
          hist8[-1][2] == TURN_OFF and n_pause == 1,
          f'{SIGNAL_NAME[hist8[-1][2]]} / turn_pause {n_pause}회')
    check('소등 전까지는 켜져 있다 (PAUSE_AFTER_S 만큼 기다린다)',
          hist8[176 + 40][2] == TURN_LEFT,      # 정지 2.0초 시점
          SIGNAL_NAME[hist8[176 + 40][2]])

    rec9, hist9 = pause_case(224, 120)          # s=140 (구간 10m 앞) 에서 6초 정지
    check('구간 시작 20m 안이면 끄지 않는다 (항목 13 선행 보호)',
          hist9[-1][2] == TURN_LEFT
          and not [1 for k, _d in rec9.ev if k == 'turn_pause'],
          SIGNAL_NAME[hist9[-1][2]])

    # 재출발: 3.0m/s^2 (지령 상한) 로 가속해도 선행 3초가 남아야 한다
    ramp = [min(12.5, 3.0 * (k + 1) * 0.05) for k in range(600)]
    rec10, hist10 = pause_case(176, 200, ramp)
    hit = next(i for i, (_t, ss, _g) in enumerate(hist10) if ss >= 150.0)
    j = hit
    while j > 0 and hist10[j - 1][2] == TURN_LEFT:
        j -= 1
    lead_t = hist10[hit][0] - hist10[j][0]
    kinds10 = [k for k, _d in rec10.ev]
    check('재출발 뒤 선행 점등 3초 이상 (정차 소등이 갉아먹지 않는다)',
          hist10[hit][2] == TURN_LEFT and lead_t >= 3.0, f'{lead_t:.2f}초')
    check('turn_pause / turn_resume 가 한 번씩 난다',
          kinds10.count('turn_pause') == 1 and kinds10.count('turn_resume') == 1,
          ','.join(kinds10))

    # 미세 이동(0.9km/h)이 섞여도 채터링하지 않는다
    micro = [0.0 if k % 2 else 0.25 for k in range(240)]
    rec11, _h11 = pause_case(176, 0, micro)
    check('정지/미세 이동이 섞여도 소등은 한 번뿐',
          len([1 for k, _d in rec11.ev if k == 'turn_pause']) == 1,
          ','.join(k for k, _d in rec11.ev))

    print('\n자기검사 ' + ('전부 통과' if ok else '실패 있음'))
    return 0 if ok else 1


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(
        description='방향지시등 점등 구간 계획 / 자기검사 (항목 13)')
    ap.add_argument('--plan', action='store_true',
                    help='경로의 점등 구간 표를 낸다')
    ap.add_argument('--path', default='', help='waypoint CSV')
    ap.add_argument('--segments', default='',
                    help='쉼표로 구분한 구간 이름. 생략하면 파일에 있는 전부')
    ap.add_argument('--selftest', action='store_true',
                    help='로그에 없는 상황을 합성해 확인한다')
    args = ap.parse_args()

    if args.selftest:
        raise SystemExit(_selftest())
    if not args.plan or not args.path:
        ap.error('--plan --path를 함께 주거나 --selftest를 쓸 것')
    segs = [s.strip() for s in args.segments.split(',') if s.strip()]
    raise SystemExit(_plan(args.path, segs))


if __name__ == '__main__':
    main()
