#!/usr/bin/env python3
"""방향지시등 (평가항목 13).

기준은 "차로 변경 시 3초 내 미점등이면 경미 -3점"이다. 지금까지 제어 패킷의
3번째 바이트는 항상 0(OFF)이었고, 생성 경로에는 차선변경이 실제로 들어 있다
(경로 1에 3회, 경로 2에 2회). 그대로 나가면 확정 감점이라 이 파일이 생겼다.

stoplines.py / obstacles.py와 같은 구조다 — I/O가 없는 순수 로직이고, 매
프레임 결과 하나(TurnCmd)를 낸다. **다만 이 모듈만 v_limit을 내지 않는다.**
방향지시등은 속도에 아무 영향이 없으므로 min() 합류에 끼지 않고 CTRL의
3번째 바이트로만 나간다.

--- 점등 구간을 어디서 얻는가 ---

1. **차선변경** — 생성 경로 CSV의 lc_dir 컬럼. 다만 이것은 **한 행짜리
   마커**다(xodr_graph.py:623-636, 노드를 떠나는 마지막 waypoint 한 행).
   실제 기하는 그 행을 **중심으로 ±10m 블렌드**되어 있다
   (LC_BLEND=20.0, xodr_graph.py:53). 그래서 구간은 마커 ± LC_BLEND_M/2다.
   pure_pursuit.load_path는 segment_id/x/y만 읽고 lc_dir을 버리므로
   이 모듈이 같은 CSV를 다시 열어 (x,y)로 스냅한다.

2. **교차로 회전** — 경로 헤딩을 훑어 찾는다. 항목 13이 명시하는 것은 차로
   변경뿐이지만 회전에 켜는 쪽이 감점 위험이 없어 기본으로 켠다
   (--no-junction-signal로 끈다).

   **경로 CSV의 heading 컬럼은 쓰지 않는다.** 그것은 블렌드된 *차로* 헤딩이라
   차선변경 중 실제 접선과 최대 0.24 rad(14°) 어긋난다. 접선은
   stoplines._path_heading으로 좌표에서 직접 뽑는다 (길이 0 선분을 이미
   방어한다 — 경로 1에 13개, 경로 2에 15개 있고 그중 3개가 lc_dir 행과 겹친다).

   판정은 "헤딩 증분의 부호가 일관된 구간의 누적 변화"다. 이 조건이
   **차선변경 블렌드를 저절로 걸러낸다** — 블렌드는 +14° 뒤 -14°로 부호가
   뒤집혀 각 run의 누적이 25°에 못 미친다. 반대로 완만한 도로 커브는 누적이
   커질 수 있으므로 길이를 TURN_MAX_M으로 제한한다.

--- 겹칠 때 우선순위 ---

**차선변경이 교차로 회전을 이긴다.** 항목 13이 강제하는 것은 차로 변경뿐이고
회전 점등은 우리가 얹은 여유분이라, 양보시켜도 감점이 없다. 차선변경 구간이
끝나는 그 프레임에 즉시 회전 방향으로 전환한다.

  · 같은 방향(좌차선변경 -> 좌회전)이면 두 구간이 이어져 연속 점등이 된다
  · 반대 방향(우차선변경 -> 좌회전)이면 우측이 꺼지는 즉시 좌측이 켜진다.
    이때 회전 점등이 잘리므로 turn_conflict 이벤트를 남긴다.
    **경로 1에 실제로 있다** — 차선변경 right[734.3,754.3]이 좌회전
    [736.9,767.8] **안에** 들어 있어, 좌회전 앞 17m를 우측 점등으로 지난다.
    감점 항목이 아니라 그대로 두되 이벤트로 보이게 한다

--- 래치 ---

lead가 속도에 비례하므로(3.5초 x v) 접근 중 감속하면 lead가 줄어 껐다 켰다
한다. **한 번 켜지면 구간이 끝날 때까지 래치해서** 그 채터링을 막는다.
리스폰 시 reset()으로 버린다.
"""

from __future__ import annotations

import csv
import math
import os

from hlvtd_client import TURN_OFF, TURN_LEFT, TURN_RIGHT
from stoplines import MANEUVER_TURN_RAD, SNAP_MAX_M, _path_heading, wrap_pi

# 점등 시점
LEAD_S = 3.5                # s, 요건 3초에 대한 여유. 등속이면 정확히 이만큼 앞선다
LEAD_MIN_M = 15.0           # m, 저속에서의 하한 (3.5s x 4m/s = 14m)
OFF_EXTRA_M = 2.0           # m, 구간이 끝난 뒤 이만큼 더 켜 두고 끈다

# 차선변경 구간
LC_BLEND_M = 20.0           # m, xodr_graph.LC_BLEND. 마커를 중심으로 이만큼 섞인다
SNAP_WARN_M = 1.5           # m, 이보다 크면 경고만 (stoplines와 같은 규약)

# 교차로 회전 검출
HEAD_SPAN_M = 3.0           # m, 접선을 뽑을 때 앞서 볼 거리
TURN_RAD = MANEUVER_TURN_RAD        # 25°. stoplines의 기동 분류와 같은 임계
NOISE_RAD = 0.0015          # rad/step, 이보다 작은 증분은 방향이 없다고 본다
TURN_GAP_M = 8.0            # m, run 안에서 이만큼 무방향이 이어지면 끊는다
TURN_MAX_M = 70.0           # m, 이보다 긴 run은 교차로 회전이 아니라 도로 커브다.
                            # 연결로 실측이 23~32m이므로 넉넉하다

SIGNAL_NAME = {TURN_OFF: 'OFF', TURN_LEFT: 'LEFT', TURN_RIGHT: 'RIGHT'}
SIGNAL_OF = {'left': TURN_LEFT, 'right': TURN_RIGHT}
KIND_NAME = {'lc': '차선변경', 'turn': '교차로회전'}


class TurnZone:
    """점등해야 하는 경로 구간 하나."""

    __slots__ = ('kind', 'dir', 'signal', 's_start', 's_end', 'x', 'y', 'note')

    def __init__(self, kind, direction, s_start, s_end, x, y, note=''):
        self.kind = kind            # 'lc' (차선변경) / 'turn' (교차로 회전)
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
        self.action = action        # none / on / error
        self.src = src              # 'lc' / 'turn'
        self.dist = dist            # 구간 시작까지 [m]. 구간 안이면 음수
        self.reason = reason

    def __repr__(self) -> str:
        return (f'<TurnCmd {SIGNAL_NAME.get(self.signal, self.signal)} '
                f'{self.src} d={self.dist:.1f}>')


# ----------------------------------------------------------------------
# 로드

def load_lane_changes(path, waypoints_csv: str) -> tuple[list, list]:
    """경로 CSV의 lc_dir 마커를 구간으로 바꾼다. (구간 목록, 경고 목록).

    stoplines.load_for_path와 같은 이유로 **(x,y)로 스냅한다** — --segments가
    부분집합이면 행 순서와 경로 인덱스가 어긋나기 때문이다.

    lc_dir 컬럼이 없는 CSV(수동 녹화본 waypoints_*.csv)면 조용히 빈 목록을
    낸다. 그 경로에는 차선변경 정보가 없다는 뜻이라 경고할 일이 아니다.
    """
    out, warn = [], []
    if not waypoints_csv or not os.path.exists(waypoints_csv):
        return out, warn
    with open(waypoints_csv, newline='', encoding='utf-8') as fh:
        rdr = csv.DictReader(fh)
        if 'lc_dir' not in (rdr.fieldnames or []):
            return out, warn
        rows = [r for r in rdr
                if (r.get('lc_dir') or '').strip() in ('left', 'right')]

    for r in rows:
        x, y = float(r['x']), float(r['y'])
        direction = r['lc_dir'].strip()
        solid = (r.get('lc_solid') or '0').strip() == '1'
        i = path.nearest(x, y)
        resid = math.hypot(path.pts[i][0] - x, path.pts[i][1] - y)
        if resid > SNAP_MAX_M:
            warn.append(f'차선변경 마커 ({x:.1f},{y:.1f}) {direction}를 버린다 — '
                        f'경로에서 {resid:.1f}m (한계 {SNAP_MAX_M:.0f}m)')
            continue
        if resid > SNAP_WARN_M:
            warn.append(f'차선변경 마커 ({x:.1f},{y:.1f}) 스냅 잔차 {resid:.1f}m')
        s = path.s[i]
        half = LC_BLEND_M * 0.5
        note = '실선(항목 6 위험)' if solid else ''
        out.append(TurnZone('lc', direction,
                            max(0.0, s - half), min(path.length, s + half),
                            path.pts[i][0], path.pts[i][1], note))
    return out, warn


def find_turns(path) -> list:
    """경로 헤딩에서 교차로 회전 구간을 찾는다.

    부호가 일관된 헤딩 증분 run의 누적이 TURN_RAD 이상이면 회전으로 본다.
    길이가 TURN_MAX_M을 넘는 run은 도로 커브이므로, 끝쪽 TURN_MAX_M만 다시
    재서 그래도 임계를 넘으면 그 부분만 회전으로 잡는다 (접근 커브와 교차로
    회전이 같은 부호로 이어 붙는 경우).
    """
    n = len(path.pts)
    if n < 3:
        return []
    th = [_path_heading(path, i, HEAD_SPAN_M) for i in range(n)]

    runs = []
    cur = None                      # [i0, i1, total, sign]
    gap = 0.0
    for i in range(n - 1):
        d = wrap_pi(th[i + 1] - th[i])
        if abs(d) < NOISE_RAD:
            if cur is not None:
                gap += path.s[i + 1] - path.s[i]
                if gap > TURN_GAP_M:
                    runs.append(cur)
                    cur = None
            continue
        sgn = 1 if d > 0 else -1
        if cur is not None and cur[3] != sgn:
            runs.append(cur)
            cur = None
        if cur is None:
            cur = [i, i + 1, d, sgn]
        else:
            cur[1] = i + 1
            cur[2] += d
        gap = 0.0
    if cur is not None:
        runs.append(cur)

    out = []
    for i0, i1, total, _sgn in runs:
        if abs(total) < TURN_RAD:
            continue
        if path.s[i1] - path.s[i0] > TURN_MAX_M:
            j, acc = i1, 0.0
            while j > i0 and path.s[i1] - path.s[j - 1] <= TURN_MAX_M:
                acc += wrap_pi(th[j] - th[j - 1])
                j -= 1
            if abs(acc) < TURN_RAD:
                continue
            i0, total = j, acc
        out.append(TurnZone('turn', 'left' if total > 0 else 'right',
                            path.s[i0], path.s[i1],
                            path.pts[i0][0], path.pts[i0][1],
                            f'{math.degrees(total):+.0f}도'))
    return out


def _merge(zones: list) -> list:
    """겹치는 같은 방향·같은 종류 구간을 하나로 합친다.

    경로 1의 683.5m / 701.8m 연속 우측 차선변경 2회가 여기서 한 구간이 된다.
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


def build_zones(path, waypoints_csv: str, *,
                junction: bool = True) -> tuple[list, list]:
    """점등 구간 전부. (구간 목록, 경고 목록)."""
    zones, warn = load_lane_changes(path, waypoints_csv)
    zones = _merge(zones)
    if junction:
        zones = zones + _merge(find_turns(path))
    zones.sort(key=lambda z: (z.s_start, 0 if z.kind == 'lc' else 1))
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
        self._on_t = 0.0
        self._on_s = 0.0
        self._announced = set()

    def _announce(self, key, kind: str, detail: str) -> None:
        if key in self._announced:
            return
        self._announced.add(key)
        self._on_event(kind, detail)

    def _finish(self, t: float, s_ego: float) -> None:
        if self._active is None:
            return
        z = self._active
        self._on_event('turn_off',
                       f'{KIND_NAME[z.kind]} {z.dir} 소등 — '
                       f'{t - self._on_t:.1f}초 / {s_ego - self._on_s:.1f}m 점등')
        self._active = None

    def _switch(self, z, t: float, s_ego: float, v: float,
                lead: float) -> None:
        """활성 구간을 z로 바꾼다.

        **방향이 같으면 램프는 실제로 꺼지지 않는다.** 그때 소등/점등 이벤트를
        내면 로그에 없는 깜빡임이 있는 것처럼 보이고, 항목 13이 요구하는
        "연속 점등 시간"도 끊겨 보인다. 그래서 구간만 갈아 끼우고 점등 시각은
        유지한다 — 경로 1의 683.5m / 744.3m 우측 차선변경 두 구간이 이 경우다.
        """
        same = (self._active is not None
                and self._active.signal == z.signal)
        if not same:
            self._finish(t, s_ego)
            self._on_t, self._on_s = t, s_ego
        self._active = z
        self._on_event(
            'turn_on',
            f'{KIND_NAME[z.kind]} {z.dir} '
            + ('구간 인계' if same else '점등')
            + f' — 구간 시작 {z.s_start - s_ego:.1f}m 앞 '
              f'(v={v * 3.6:.0f}km/h, lead {lead:.1f}m)')

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
            self._finish(t, s_ego)
            return TurnCmd()

        # 차선변경이 교차로 회전을 이긴다. 같은 종류면 먼저 시작하는 구간
        # (지금 벌어지는 일)이 이긴다.
        cands.sort(key=lambda z: (0 if z.kind == 'lc' else 1, z.s_start))
        best = cands[0]

        if best.kind == 'lc':
            for other in cands[1:]:
                if other.kind == 'turn' and other.signal != best.signal:
                    # 회전이 뒤에 있으면 선행 점등이, 앞에 있으면 남은 점등이
                    # 잘린다. 어느 쪽이든 잃는 거리는 "지금부터 회전 구간이
                    # 끝날 때까지"다.
                    lost = other.s_end + OFF_EXTRA_M - s_ego
                    self._announce(
                        ('conflict', id(best), id(other)), 'turn_conflict',
                        f'차선변경 {best.dir}'
                        f'[{best.s_start:.1f},{best.s_end:.1f}]이 '
                        f'회전 {other.dir}'
                        f'[{other.s_start:.1f},{other.s_end:.1f}]을 밀어낸다 — '
                        f'항목 13은 차선변경만 강제한다. '
                        f'회전 점등 {max(0.0, lost):.1f}m 손실')
                    break

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


def _plan(csv_path: str, segments: list, events_csv: str,
          junction: bool) -> int:
    """경로의 점등 구간 표를 낸다. 현장에서 4번 단계 전에 눈으로 확인할 용도."""
    from pure_pursuit import load_path

    segments = segments or _segments_of(csv_path)
    path, _bounds = load_path(csv_path, segments)
    zones, warn = build_zones(path, csv_path, junction=junction)
    n_lc = sum(1 for z in zones if z.kind == 'lc')
    print(f'[ts] {os.path.basename(csv_path)}  {len(path.pts)}점 '
          f'{path.length:.1f}m  구간 {len(segments)}개')
    print(f'[ts] 점등 구간 {len(zones)}개 '
          f'(차선변경 {n_lc} / 회전 {len(zones) - n_lc})')
    for line in summary(zones):
        print('[ts]' + line)
    for w in warn:
        print(f'[ts]   *** {w} ***')

    # 방향이 어긋난 채 겹치는 구간. 항목 13은 차선변경만 강제하므로 그쪽이
    # 이기고, 회전 점등의 선행이 잘린다.
    for i, a in enumerate(zones):
        for b in zones[i + 1:]:
            if b.s_start > a.s_end:
                break
            if a.signal != b.signal:
                print(f'[ts]   *** 충돌: {KIND_NAME[a.kind]} {a.dir} '
                      f'[{a.s_start:.1f},{a.s_end:.1f}] vs '
                      f'{KIND_NAME[b.kind]} {b.dir} '
                      f'[{b.s_start:.1f},{b.s_end:.1f}] — '
                      f'{KIND_NAME[a.kind]}이 이긴다 ***')

    if events_csv and os.path.exists(events_csv):
        import stoplines
        sl_list, _w = stoplines.load_for_path(path, events_csv)
        print('[ts] 정지선 기동과의 대조 (회전 검출이 맞는지)')
        for sl in sl_list:
            near = [z for z in zones
                    if z.kind == 'turn' and z.s_end > sl.s > z.s_start - 60.0]
            got = near[0].dir if near else 'straight'
            mark = 'OK' if got == sl.maneuver else '??'
            print(f'[ts]   {mark} s={sl.s:7.1f}  정지선 {sl.maneuver:8s} '
                  f'/ 검출 {got}')
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

    # 4. 우차선변경 직후 좌회전 — 방향이 반대인 경우
    zl = [TurnZone('lc', 'right', 100.0, 120.0, 100.0, 0.0),
          TurnZone('turn', 'left', 125.0, 160.0, 125.0, 0.0)]
    rec = _Rec()
    pol = TurnSignalPolicy(path, zl, v_ref=12.5, on_event=rec)
    _on, _r, hist = _run(pol, {}, lambda t: 12.5)

    def at(s0):
        return next(g for _t, ss, g in hist if ss >= s0)

    check('우차선변경 중에는 우측', at(110.0) == TURN_RIGHT,
          SIGNAL_NAME[at(110.0)])
    check('차선변경이 끝나면 좌측으로 전환', at(130.0) == TURN_LEFT,
          SIGNAL_NAME[at(130.0)])
    check('turn_conflict 발화', 'turn_conflict' in rec.kinds(),
          ','.join(sorted(set(rec.kinds()))))
    dark = [1 for _t, ss, g in hist
            if 118.0 <= ss <= 128.0 and g == TURN_OFF]
    check('전환 중 소등 구간이 짧다', len(dark) * 12.5 * 0.05 < 5.0,
          f'소등 {len(dark) * 12.5 * 0.05:.1f}m')

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

    # 7. 차선변경 블렌드는 회전으로 검출되지 않는다
    pts = []
    for i in range(401):
        x = i * 1.0
        u = (x - 190.0) / 20.0
        u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
        w = u * u * (3.0 - 2.0 * u)             # smoothstep, xodr_graph와 동일
        pts.append((x, 3.2 * w))
    blend = find_turns(Path(pts))
    check('차선변경 블렌드는 회전이 아니다', not blend, f'{len(blend)}개 검출')

    # 8. 90도 회전은 검출된다
    pts = [(i * 1.0, 0.0) for i in range(100)]
    R = 15.0
    for k in range(1, 40):
        a = k * (math.pi / 2) / 39
        pts.append((99.0 + R * math.sin(a), R * (1.0 - math.cos(a))))
    for k in range(1, 60):
        pts.append((99.0 + R, R + k * 1.0))
    turns = find_turns(Path(pts))
    check('90도 좌회전 검출', len(turns) == 1 and turns[0].dir == 'left',
          repr(turns))

    # 9. 길이 0 선분이 섞여도 죽지 않는다
    dup = [(i * 1.0, 0.0) for i in range(50)]
    dup.insert(20, dup[20])
    dup.insert(35, dup[35])
    find_turns(Path(dup))
    check('길이 0 선분에서 죽지 않는다', True)

    # 10. lc_dir 컬럼이 없는 CSV는 조용히 빈 목록
    here = os.path.dirname(os.path.abspath(__file__))
    manual = os.path.normpath(
        os.path.join(here, '..', 'data', 'waypoints_pretest2.csv'))
    zs, ws = load_lane_changes(path, manual)
    check('lc_dir 없는 CSV는 경고 없이 빈 목록', not zs and not ws,
          f'{len(zs)}구간 {len(ws)}경고')

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
    ap.add_argument('--events', default='',
                    help='정지선 이벤트 CSV. 주면 기동 분류와 대조한다')
    ap.add_argument('--no-junction-signal', action='store_true',
                    help='교차로 회전 점등을 빼고 차선변경만 본다')
    ap.add_argument('--selftest', action='store_true',
                    help='로그에 없는 상황을 합성해 확인한다')
    args = ap.parse_args()

    if args.selftest:
        raise SystemExit(_selftest())
    if not args.plan or not args.path:
        ap.error('--plan --path를 함께 주거나 --selftest를 쓸 것')
    segs = [s.strip() for s in args.segments.split(',') if s.strip()]
    ev = args.events
    if not ev:
        import stoplines
        ev = stoplines.default_events_path(args.path)
    raise SystemExit(_plan(args.path, segs, ev, not args.no_junction_signal))


if __name__ == '__main__':
    main()
