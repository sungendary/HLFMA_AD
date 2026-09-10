#!/usr/bin/env python3
"""종료선 통과 보장 (완주 판정).

주최측 공지: 본 대회 경로의 종료 지점은 **정지선 위에 놓인 라바콘 2개**로 표시되고,
**차량의 후륜축이 두 라바콘 사이의 종료선을 통과해야** 완주로 인정된다.
9910의 ego 좌표는 후륜축이다.

--- 실측 (2026-09-10, pt1/pt2/ex 세 경로 전수) ---

1. **마지막 정지선이 경로 끝점 그 자리다.** 이벤트 CSV의 마지막 stopline을 (x,y)로
   스냅하면 셋 다 정확히 path.length에 떨어지고, 끝 접선 기준 종방향 오차는 -0.15m다
   (pt1은 옛 CSV라 횡으로 2.85m 어긋나 있지만 종방향은 같다). 즉 기존 경로 끝 감속
   (rem-1.0에 정지)은 후륜축을 종료선 **0~1.5m 앞**에 세운다 -- 못 넘긴다.

2. **경로를 그대로 두고 "끝을 지나친 거리"를 따로 계산하는 방식은 안 된다.**
   최근접 인덱스가 마지막 점에 포화되면 s_ego가 path.length에 붙어서
     · 종점 정지선의 d가 영원히 0이라 stoplines가 거리로 놓아주지 못하고
       (sim 신호원은 0<=d라 NONE도 안 된다 -- 지나친 뒤 적색이면 2m/s 이하에서 45초 hold),
     · 모든 객체가 s_obj<=length로 투영돼 지나치는 동안 충돌 회피가 눈을 감고,
     · 목표점이 뒤로 가며 OFFSET_SLOW_M 복귀 제한이 걸리고,
     · check_signals.project가 잔차>6m에서 전역 재탐색해 꼬리 프레임이 튄다.
   그래서 **경로 자체를 끝 접선 방향으로 overrun+1m 가상 연장한다.** 기존 끝 감속·끝
   조향 블렌드·finished 래치·sim 종료 판정·계수기가 전부 손대지 않고 그대로 맞는다.
   종료선은 경로 **내부** 점이 된다. +1m는 프로파일이 rem=1에서 0이 되기 때문이다.

3. **ego(0,0)은 시뮬레이션 종료 신호다.** 완주 후 VTD가 ego(0,0), heading 0, 객체 0을
   보낸다. 리스폰(5m 점프)으로 오판하면 전역 재탐색 뒤 accel=3.0을 낸다. 이 모듈이
   "종료선을 지났거나 경로 끝 FINISH_ZONE_M 안"이라는 게이트를 주고 pure_pursuit이
   그때만 종료로 처리한다. 그 밖의 원점 프레임은 정지 유지로만 대응한다.

4. **종료 라바콘은 장애물이 아니다.** obstacles(차로 내 라바콘 10km/h 감속)와
   detour(회랑 1.5m 안 정지 객체 8초 확인 -> 우회)가 그것을 잡으면 안 된다. 경로 끝
   FINISH_ZONE_M 안의 obstacle 분류는 둘 다 후보에서 뺀다 (gate_s). 차량·보행자는
   그대로 대응하고, 그 안의 막힘은 detour_impossible로 보내 최후 안전망만 남긴다 --
   우회로 종료선을 벗어나면 완주가 아니다.

I/O도 소켓도 없다. stoplines.load_for_path를 그대로 재사용한다.
"""

from __future__ import annotations

import math
import os

import stoplines as SL

FINISH_OVERRUN_M = 8.0      # m, 경로 끝(=종료선)을 이만큼 지나쳐 선다. --finish-overrun
FINISH_ZONE_M = 30.0        # m, 경로 끝 이 안은 종료 구간. obstacle 무시·우회 금지·
                            # ego(0,0) 게이트에 공통. --finish-zone
END_STEP_M = 1.0            # m, 가상 연장 점 간격 (경로 격자와 같게)
TANGENT_BACK_M = 3.0        # m, 끝 접선을 잴 때 뒤로 볼 거리 (Path.end_heading과 같은 규칙.
                            # 이음매의 0.0m 중복점에 흔들리지 않는다)
LINE_WARN_M = 3.0           # m, 종료선 CSV 좌표가 경로 접선 기준 이보다 어긋나면 경고
LINE_LAT_M = 6.0            # m, 통과 판정을 인정할 횡거리 (check_redlight.LAT_MAX와 같게)
DONE_M = 1.5                # m, pure_pursuit의 finished 래치와 같은 값
STOP_SPEED = SL.STOP_SPEED


# ----------------------------------------------------------------------
# 기하 -- 가상 연장

def end_tangent(pts: list, back_m: float = TANGENT_BACK_M) -> float:
    """마지막 점에서의 진행 방위. 끝에서 back_m 이상 뒤의 점과 잇는다."""
    ex, ey = pts[-1]
    i = len(pts) - 1
    acc = 0.0
    while i > 0 and acc < back_m:
        acc += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        i -= 1
    if i == len(pts) - 1:
        return 0.0
    return math.atan2(ey - pts[i][1], ex - pts[i][0])


def extend_points(pts: list, ext_m: float, step: float = END_STEP_M,
                  back_m: float = TANGENT_BACK_M) -> list:
    """끝 접선 방향으로 ext_m 이상을 step 간격으로 붙일 가상 점 목록."""
    if ext_m <= 0.0 or len(pts) < 2:
        return []
    h = end_tangent(pts, back_m)
    n = int(math.ceil(ext_m / step - 1e-9))
    ex, ey = pts[-1]
    return [(ex + k * step * math.cos(h), ey + k * step * math.sin(h))
            for k in range(1, n + 1)]


def finish_index(path) -> int:
    """실제 경로의 마지막 인덱스. 연장이 없는 path(자기검사 스텁)도 받는다."""
    return getattr(path, 'finish_idx', len(path.pts) - 1)


def finish_s(path) -> float:
    return path.s[finish_index(path)]


def gate_s(path, zone_m: float = FINISH_ZONE_M) -> float:
    """이 s 이상은 종료 구간. obstacles/detour가 obstacle 분류를 무시하는 경계."""
    return finish_s(path) - zone_m


# ----------------------------------------------------------------------
# 종료선

class FinishLine:
    """경로에 붙인 종료선 하나. 위치는 **경로 위 점**이고 방위는 그 접선이다."""

    __slots__ = ('idx', 's', 'x', 'y', 'heading', 'src', 'ctrl',
                 'lon_off', 'lat_off', 'resid')

    def __init__(self, idx, s, x, y, heading, src, ctrl=frozenset(),
                 lon_off=0.0, lat_off=0.0, resid=0.0):
        self.idx, self.s = idx, s
        self.x, self.y, self.heading = x, y, heading
        self.src = src                  # 'stopline' / 'path_end'
        self.ctrl = ctrl                # 신호 ctrl 집합 (정지선일 때)
        self.lon_off, self.lat_off = lon_off, lat_off   # CSV 좌표의 접선 기준 오차
        self.resid = resid

    def ctrl_str(self) -> str:
        return '|'.join(str(c) for c in sorted(self.ctrl)) if self.ctrl else '-'

    def label(self) -> str:
        return (f'{self.src}' + (f' ctrl={self.ctrl_str()}'
                                 if self.src == 'stopline' else ''))

    def lon_of(self, x: float, y: float) -> float:
        """접선 기준 종방향 부호 거리. 양수 = 선을 넘었다."""
        return ((x - self.x) * math.cos(self.heading)
                + (y - self.y) * math.sin(self.heading))

    def lat_of(self, x: float, y: float) -> float:
        return (-(x - self.x) * math.sin(self.heading)
                + (y - self.y) * math.cos(self.heading))

    def __repr__(self) -> str:
        return f'<FinishLine s={self.s:.1f} {self.label()} lon_off={self.lon_off:+.2f}>'


def load_for_path(path, events_csv: str,
                  zone_m: float = FINISH_ZONE_M) -> tuple:
    """종료선을 정한다. (FinishLine, 경고 목록).

    이벤트 CSV의 **s가 가장 큰 정지선**이 경로 끝 zone_m 안에 있으면 그것이고,
    아니면 경로 끝점이다. 위치는 CSV 좌표가 아니라 스냅된 경로 점을 쓴다 --
    pt1처럼 CSV가 횡으로 2.85m 빠져 있어도 접선 기준 통과 판정은 같아야 한다.
    """
    warns = []
    fi = finish_index(path)
    fs = path.s[fi]
    sl = None
    if events_csv and os.path.exists(events_csv):
        sls, _w = SL.load_for_path(path, events_csv)
        if sls:
            cand = max(sls, key=lambda z: z.s)
            if fs - cand.s <= zone_m:
                sl = cand
            else:
                warns.append(f'마지막 정지선 s={cand.s:.1f}이 경로 끝 {fs:.1f}보다 '
                             f'{fs - cand.s:.1f}m 앞 (한계 {zone_m:.0f}m) '
                             f'― 종료선을 경로 끝으로 본다')
        else:
            warns.append('이벤트 CSV에 정지선이 없다 ― 종료선을 경로 끝으로 본다')
    else:
        warns.append('이벤트 CSV 없음 ― 종료선을 경로 끝으로 본다')

    if sl is None:
        h = SL._path_heading(path, fi)
        return FinishLine(fi, fs, path.pts[fi][0], path.pts[fi][1], h,
                          'path_end'), warns

    idx = sl.idx
    h = SL._path_heading(path, idx)
    px, py = path.pts[idx]
    line = FinishLine(idx, path.s[idx], px, py, h, 'stopline', sl.ctrl,
                      resid=sl.resid)
    line.lon_off = line.lon_of(sl.x, sl.y)
    line.lat_off = line.lat_of(sl.x, sl.y)
    if abs(line.lon_off) > LINE_WARN_M:
        warns.append(f'종료선 CSV 좌표가 경로 점에서 종방향 {line.lon_off:+.2f}m '
                     f'(한계 {LINE_WARN_M:.0f}m) ― 이벤트 CSV가 오래됐을 수 있다')
    if path.s[idx] > fs + 0.5:
        warns.append(f'종료선 s={path.s[idx]:.1f}이 경로 끝 {fs:.1f}보다 뒤다 '
                     f'― 가상 연장 위에 스냅됐다. overrun이 이를 덮는지 확인할 것')
    return line, warns


# ----------------------------------------------------------------------
# 매 프레임

class FinishCmd:
    """한 프레임의 판정. lon은 후륜축의 종료선 기준 종방향 부호 거리(+ = 넘음)."""

    __slots__ = ('lon', 'dist', 'passed', 'done', 'near', 'action')

    def __init__(self, lon=float('nan'), dist=float('nan'), passed=False,
                 done=False, near=False, action='none'):
        self.lon, self.dist = lon, dist
        self.passed, self.done, self.near = passed, done, near
        self.action = action


class FinishPolicy:
    """종료선 통과를 감시하고 완주 래치 조건을 낸다. 속도 상한은 내지 않는다.

    감속·정지는 연장된 경로의 끝 감속이 맡는다. 이 클래스는 "넘었는가"와
    "다 왔는가"만 판정하고 이벤트를 남긴다.
    """

    def __init__(self, path, line: FinishLine, *,
                 zone_m: float = FINISH_ZONE_M, on_event=None):
        self.path = path
        self.line = line
        self.zone_m = zone_m
        self.finish_s = finish_s(path)
        self._on_event = on_event or (lambda kind, detail: None)
        self.reset()

    def reset(self) -> None:
        """리스폰 시. 위치가 강제로 옮겨졌으므로 통과 래치를 버린다."""
        self.passed = False
        self.near = False
        self.max_lon = float('-inf')
        self._short = False

    def update(self, idx: int, s_ego: float, ego_x: float, ego_y: float,
               v: float, t: float) -> FinishCmd:
        lon = self.line.lon_of(ego_x, ego_y)
        lat = self.line.lat_of(ego_x, ego_y)
        self.near = (self.finish_s - s_ego) <= self.zone_m
        dist = self.line.s - s_ego

        # 통과 판정은 **종료선 근처에서만** 인정한다. 경로가 자기교차하면 출발
        # 직후에도 접선 반평면의 양쪽에 있을 수 있다.
        if (not self.passed and lon > 0.0 and self.near
                and abs(lat) <= LINE_LAT_M):
            self.passed = True
            self._on_event(
                'finish_line_passed',
                f'종료선 s={self.line.s:.1f} ({self.line.label()}) ― '
                f'후륜축 s={self.line.s + lon:.1f} 여유 {lon:+.2f}m '
                f'횡 {lat:+.2f}m v={v * 3.6:.1f}km/h')
        if self.passed and lon > self.max_lon:
            self.max_lon = lon

        rem = self.path.remaining(idx)
        done = rem < DONE_M and v < STOP_SPEED
        if done and not self.passed and not self._short:
            self._short = True
            self._on_event(
                'finish_short',
                f'경로 끝에 섰는데 종료선을 못 넘었다 ― lon={lon:+.2f}m '
                f'횡 {lat:+.2f}m (종료선 s={self.line.s:.1f}, 경로 끝 {self.finish_s:.1f})')

        action = ('done' if done else 'passed' if self.passed
                  else 'near' if self.near else 'none')
        return FinishCmd(lon, dist, self.passed, done, self.near, action)


def summary(policy: FinishPolicy, overrun_m: float) -> list:
    ln = policy.line
    out = [f'  종료선 s={ln.s:.1f} ({ln.label()}) 경로 끝 {policy.finish_s:.1f}m '
           f'― 끝 +{overrun_m:.0f}m 지나 정지, 종료 구간 {policy.zone_m:.0f}m']
    if ln.src == 'stopline':
        out.append(f'  CSV 좌표 오차 종 {ln.lon_off:+.2f}m 횡 {ln.lat_off:+.2f}m '
                   f'(스냅 잔차 {ln.resid:.2f}m)')
    return out


# ----------------------------------------------------------------------
# 진단 -- 대회 당일 4단계 전에 눈으로 볼 표

def _segments_of(csv_path: str) -> list:
    import csv
    seen = []
    with open(csv_path, newline='', encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            if r['segment_id'] not in seen:
                seen.append(r['segment_id'])
    return seen


def _plan(args) -> int:
    from pure_pursuit import load_path

    segs = ([s.strip() for s in args.segments.split(',') if s.strip()]
            if args.segments else _segments_of(args.path))
    ext = args.overrun + 1.0
    path, _b = load_path(args.path, segs, extend_m=ext)
    ev = args.events or SL.default_events_path(args.path)
    line, warns = load_for_path(path, ev, zone_m=args.zone)
    n_real = getattr(path, 'n_real', len(path.pts))
    print(f'[fin] 경로 {os.path.basename(args.path)}: 실제 {n_real}점 '
          f'{path.finish_s:.2f}m + 연장 {len(path.pts) - n_real}점 '
          f'{path.length - path.finish_s:.1f}m (끝 접선 '
          f'{math.degrees(path.end_heading()):+.1f}°)')
    print(f'[fin] 이벤트 CSV: {ev or "(없음)"}')
    print(f'[fin] 종료선: s={line.s:.2f} ({line.label()}) '
          f'경로점 ({line.x:.3f},{line.y:.3f}) 접선 {math.degrees(line.heading):+.1f}°')
    if line.src == 'stopline':
        print(f'[fin]   CSV 좌표 오차 종 {line.lon_off:+.2f}m 횡 {line.lat_off:+.2f}m '
              f'(스냅 잔차 {line.resid:.2f}m), 경로 끝과 {path.finish_s - line.s:+.2f}m')
    stop_s = path.length - 1.0
    print(f'[fin] 정지 목표: s={stop_s:.1f} = 종료선 +{stop_s - line.s:.1f}m '
          f'(overrun {args.overrun:.0f}m)')
    print(f'[fin] 종료 구간: s>={gate_s(path, args.zone):.1f} ― obstacle 무시·우회 금지')
    for w in warns:
        print(f'[fin] *** {w} ***')
    if line.src != 'stopline':
        print('[fin] *** 종료선이 정지선이 아니다. 라바콘 위치를 현장에서 확인하고 '
              'overrun이 그 차이를 덮는지 볼 것 ***')
    return 0


# ----------------------------------------------------------------------
# 자기검사

class _Rec:
    def __init__(self):
        self.log = []

    def __call__(self, kind, detail):
        self.log.append((kind, detail))

    def kinds(self):
        return [k for k, _d in self.log]


def _selftest() -> int:
    import tempfile

    from pure_pursuit import Path, load_path

    fails = []

    def check(name, ok, detail=''):
        if not ok:
            fails.append(f'{name}: {detail}')
        print(f'  {"OK  " if ok else "FAIL"} {name}'
              + (f'   {detail}' if detail and not ok else ''))

    # 1. 가상 연장 -- 직선 / 곡선 / 끝 중복점
    line_pts = [(float(i), 0.0) for i in range(101)]
    ext = extend_points(line_pts, 9.0)
    check('연장: 9m -> 9점', len(ext) == 9, f'{len(ext)}')
    check('연장: 접선 방향', abs(ext[-1][0] - 109.0) < 1e-9 and abs(ext[-1][1]) < 1e-9,
          f'{ext[-1]}')
    # 반지름 50m 원호. 파라미터 k*0.02가 곧 접선 방위다
    arc = [(50.0 * math.sin(k * 0.02), 50.0 - 50.0 * math.cos(k * 0.02))
           for k in range(80)]
    h_true = 79 * 0.02
    ext = extend_points(arc, 5.0)
    h_got = math.atan2(ext[-1][1] - ext[0][1], ext[-1][0] - ext[0][0])
    check('연장: 곡선 끝 접선 (3m 후퇴)', abs(h_got - h_true) < math.radians(3.0),
          f'{math.degrees(h_got):.1f} vs {math.degrees(h_true):.1f}')
    dup = line_pts + [line_pts[-1], line_pts[-1]]
    ext = extend_points(dup, 4.0)
    check('연장: 끝 중복점에 흔들리지 않는다',
          len(ext) == 4 and abs(ext[-1][1]) < 1e-9 and abs(ext[-1][0] - 104.0) < 1e-9,
          f'{ext}')
    check('연장: 0이면 비어 있다', extend_points(line_pts, 0.0) == [])

    # 2. Path 메타 -- n_real / finish_s
    p = Path(line_pts + extend_points(line_pts, 9.0), n_real=len(line_pts))
    check('Path: finish_s는 실제 끝', abs(p.finish_s - 100.0) < 1e-9, f'{p.finish_s}')
    check('Path: length는 연장 포함', abs(p.length - 109.0) < 1e-9, f'{p.length}')
    check('Path: 연장 없으면 finish_s == length',
          Path(line_pts).finish_s == Path(line_pts).length)

    # 3. 종료선 = 경로 끝 정지선. 통과 래치·이벤트 1회·정지 위치
    with tempfile.TemporaryDirectory() as td:
        wp = os.path.join(td, 'w.csv')
        ev = os.path.join(td, 'w_events.csv')
        with open(wp, 'w', encoding='utf-8') as fh:
            fh.write('segment_id,seq,x,y\n')
            for i, (x, y) in enumerate(line_pts):
                fh.write(f'a,{i},{x},{y}\n')
        with open(ev, 'w', encoding='utf-8') as fh:
            fh.write('kind,s_along,x,y,road_id,lane_id,detail\n')
            fh.write('stopline,40.0,40.0,0.0,1,-1,ctrl=7\n')
            fh.write('stopline,100.0,99.85,-2.85,1,-1,ctrl=15|16\n')
        path, _b = load_path(wp, ['a'], extend_m=9.0)
        line, warns = load_for_path(path, ev)
        check('종료선: 마지막 정지선', line.src == 'stopline' and abs(line.s - 100.0) < 1e-9,
              repr(line))
        check('종료선: 위치는 경로 점', abs(line.x - 100.0) < 1e-9 and abs(line.y) < 1e-9,
              f'({line.x},{line.y})')
        check('종료선: CSV 오차 종 -0.15 횡 -2.85',
              abs(line.lon_off + 0.15) < 1e-6 and abs(line.lat_off + 2.85) < 1e-6,
              f'{line.lon_off:+.2f}/{line.lat_off:+.2f}')
        check('종료선: ctrl 유지', line.ctrl == frozenset({15, 16}), line.ctrl_str())
        check('종료선: 경고 없음', not warns, ' / '.join(warns))

        rec = _Rec()
        pol = FinishPolicy(path, line, on_event=rec)
        # 등감속 정지 프로파일로 끝까지 달린다 (pure_pursuit과 같은 식)
        s, v, dt, hist = 0.0, 12.5, 0.05, []
        idx = 0
        for _k in range(2000):
            while idx < len(path.pts) - 1 and path.s[idx + 1] <= s:
                idx += 1
            cmd = pol.update(idx, path.s[idx], s, 0.0, v, _k * dt)
            hist.append((s, v, cmd))
            if cmd.done:
                break
            rem = path.remaining(idx)
            v_cmd = min(12.5, math.sqrt(2.0 * 1.5 * max(0.0, rem - 1.0)))
            a = max(-6.0, min(3.0, 1.2 * (v_cmd - v)))
            if v_cmd <= 0.0:
                a = min(a, -1.5)
            s += v * dt
            v = max(0.0, v + a * dt)
        check('통과: finish_line_passed 1회',
              rec.kinds().count('finish_line_passed') == 1, ','.join(rec.kinds()))
        first = next((h for h in hist if h[2].passed), None)
        check('통과: 래치 시점 lon>0이고 작다',
              first is not None and 0.0 < first[2].lon < 1.0,
              f'{first[2].lon if first else None}')
        check('통과: 통과 속도가 걷는 속도보다 빠르다 (끝에서 서지 않는다)',
              first is not None and first[1] > 3.0, f'{first[1] if first else None} m/s')
        check('정지: done 래치', hist[-1][2].done)
        # P 제어(accel=1.2*(v_cmd-v))라 sqrt 프로파일보다 2m쯤 더 나가 선다.
        # pure_pursuit이 같은 법칙이라 실차도 overrun+2m 근처다 — 선 너머라 무해
        check('정지: 종료선 +6~11m (프로파일 +8, P제어 오버슈트 +2)',
              6.0 <= hist[-1][0] - line.s <= 11.0,
              f'{hist[-1][0] - line.s:+.2f}m')
        check('정지: finish_short 없음', 'finish_short' not in rec.kinds())
        check('근접: near는 30m 안에서만',
              all(h[2].near == (path.finish_s - h[0] <= FINISH_ZONE_M) for h in hist))
        check('통과 후 max_lon 누적', pol.max_lon > 5.0, f'{pol.max_lon:.2f}')

        # 4. 적색 대기 -- 선 4.8m 앞 정지 상태에서 done도 passed도 아니다
        pol.reset()
        cmd = pol.update(path.nearest(95.2, 0.0), 95.2, 95.2, 0.0, 0.0, 0.0)
        check('적색 대기: passed 아님', not cmd.passed and cmd.lon < 0.0, f'{cmd.lon:+.2f}')
        check('적색 대기: done 아님', not cmd.done)
        check('적색 대기: near', cmd.near)

        # 5. 자기교차 방어 -- 멀리서 접선 반평면 양쪽에 있어도 통과가 아니다
        pol.reset()
        cmd = pol.update(0, 0.0, 130.0, 0.0, 12.5, 0.0)      # s=0인데 x=130
        check('멀리서 lon>0: 통과 인정 안 함', not cmd.passed and cmd.lon > 0.0)
        cmd = pol.update(path.nearest(99.0, 0.0), 99.0, 101.0, 8.0, 3.0, 1.0)
        check('횡 8m: 통과 인정 안 함', not cmd.passed)

        # 6. 폴백 -- 정지선 없음 / CSV 없음 / 30m 넘게 앞
        with open(ev, 'w', encoding='utf-8') as fh:
            fh.write('kind,s_along,x,y,road_id,lane_id,detail\n')
            fh.write('stopline,40.0,40.0,0.0,1,-1,ctrl=7\n')
        line2, w2 = load_for_path(path, ev)
        check('폴백: 정지선이 멀면 경로 끝', line2.src == 'path_end' and abs(line2.s - 100.0) < 1e-9,
              repr(line2))
        check('폴백: 경고를 낸다', bool(w2), ' / '.join(w2))
        line3, w3 = load_for_path(path, '')
        check('폴백: CSV 없음', line3.src == 'path_end' and bool(w3))
        check('폴백: 접선은 끝 접선', abs(line3.heading - path.end_heading()) < 1e-6)
        rec = _Rec()
        pol = FinishPolicy(path, line3, on_event=rec)
        cmd = pol.update(path.nearest(100.5, 0.0), 100.5, 100.5, 0.0, 4.0, 0.0)
        check('폴백: 경로 끝 통과 판정', cmd.passed and 'finish_line_passed' in rec.kinds())

        # 7. reset 뒤 재발화 / finish_short
        rec = _Rec()
        pol = FinishPolicy(path, line, on_event=rec)
        pol.update(path.nearest(100.5, 0.0), 100.5, 100.5, 0.0, 4.0, 0.0)
        pol.reset()
        pol.update(path.nearest(100.5, 0.0), 100.5, 100.5, 0.0, 4.0, 1.0)
        check('reset: 이벤트 재발화', rec.kinds().count('finish_line_passed') == 2,
              ','.join(rec.kinds()))
        rec = _Rec()
        short_path = Path(line_pts[:95] + extend_points(line_pts[:95], 1.0), n_real=95)
        pol = FinishPolicy(short_path, line, on_event=rec)
        cmd = pol.update(len(short_path.pts) - 1, short_path.length, 94.5, 0.0, 0.0, 0.0)
        check('finish_short: 선을 못 넘고 서면 경고', cmd.done and 'finish_short' in rec.kinds(),
              ','.join(rec.kinds()))

    # 8. 실제 경로 -- 셋 다 마지막 정지선이 경로 끝이고 종방향 오차 -0.15m
    data = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         '..', 'data'))
    for name in ('route_path_pt1.csv', 'route_path_pt2.csv', 'route_path_ex.csv'):
        f = os.path.join(data, name)
        if not os.path.exists(f):
            continue
        path, _b = load_path(f, _segments_of(f))
        line, warns = load_for_path(path, SL.default_events_path(f))
        check(f'실경로 {name}: 종료선 = 경로 끝 정지선',
              line.src == 'stopline' and abs(line.s - path.finish_s) < 1e-6,
              f'{line!r} finish_s={path.finish_s:.2f}')
        check(f'실경로 {name}: 종방향 오차 |lon| < 0.5m', abs(line.lon_off) < 0.5,
              f'{line.lon_off:+.2f}')
        check(f'실경로 {name}: 연장 9점', len(path.pts) - path.n_real == 9,
              f'{len(path.pts) - path.n_real}')

    print()
    if fails:
        print(f'자기검사 {len(fails)}건 실패:')
        for f in fails:
            print(f'  - {f}')
        return 1
    print('자기검사 전부 통과')
    return 0


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description='종료선 통과 보장 ― 진단 / 자기검사')
    ap.add_argument('--plan', action='store_true', help='종료선 표를 낸다')
    ap.add_argument('--path', default='', help='waypoint CSV')
    ap.add_argument('--events', default='', help='이벤트 CSV. 생략하면 --path 옆')
    ap.add_argument('--segments', default='', help='쉼표 구분 구간. 생략하면 전부')
    ap.add_argument('--overrun', type=float, default=FINISH_OVERRUN_M)
    ap.add_argument('--zone', type=float, default=FINISH_ZONE_M)
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args()
    if args.selftest:
        raise SystemExit(_selftest())
    if args.plan and args.path:
        raise SystemExit(_plan(args))
    ap.error('--plan --path ... 또는 --selftest')


if __name__ == '__main__':
    main()
