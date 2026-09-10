#!/usr/bin/env python3
"""종료선 통과 보장 (완주 판정).

주최측 공지: 본 대회 경로의 종료 지점은 **정지선 위에 놓인 라바콘 2개**로 표시되고,
**차량의 후륜축이 두 라바콘 사이의 종료선을 통과해야** 완주로 인정된다.
9910의 ego 좌표는 후륜축이다.

--- 9/10 실차가 가르쳐 준 것 (logs/pp_20260910_181704, route_path_lt) ---

**마지막 경유지는 종료선보다 앞에 찍힌다.** 주최측 말대로 경유지는 대략적 위치다.
경로 끝점에서 종료 라바콘 중점까지 15.3m였고, "경로 끝 = 정지선" 전제로 끝 +8m에 선
1차 구현은 **선을 6m 못 넘고 완주 처리**됐다 -- 실제로는 미완주다. 세 겹의 원인이다.
  1. 종료선 출처가 이벤트 CSV의 정지선뿐인데, route_events가 마지막 waypoint 너머의
     정지선(같은 차로, 15.9m 뒤)을 버려서 path_end 폴백이 됐다.
  2. 라바콘 2개가 objects[]에 분명히 보였는데 아무도 종료선으로 쓰지 않았다.
  3. done이 passed와 무관해서 선을 못 넘고도 finished를 세웠다.

--- 설계 ---

종료선은 3단 우선순위다. 어느 경우든 경로 꼬리를 종료선 + overrun 너머까지 붙이고,
정지 목표는 path.length가 아니라 **stop_s = 종료선 s + overrun + 1**이며, **선을 넘기
전에는 절대 done을 내지 않는다.**

  1. cones     런타임. 좌우 대칭으로 놓인 정지 객체 쌍 -- 기하·운동만 본다. 치수는
                "차량 길이 초과 제외"와 "둘이 같은 크기"뿐이다 (라바콘 모델은 대회마다
                다를 수 있다). 한 번 잡으면 래치.
  2. stopline  로드 시. 이벤트 CSV 정지선. 경로 끝 **앞 FINISH_ZONE_M(30) ~ 뒤
                FINISH_SEARCH_M(150)**. 앞쪽을 넓히면 직전 교차로 정지선(pt1/pt2/ex/lt에서
                끝 108~215m 앞)을 종료선으로 잡아 그 앞에 서 버린다 -- 비대칭이 맞다.
                끝 너머 행은 xodr_graph.route_events가 detail=beyond_end로 싣는다.
  3. path_end  둘 다 없음. overrun을 FINISH_FALLBACK_OVERRUN_M(25)로 늘린다 --
                지나쳐서 잃는 것은 없고 못 넘으면 미완주라 비대칭이 극명하다.

경로는 **제자리에서 꼬리만 갈아 끼운다** (Path.replace_tail). 모든 정책이 같은 Path
객체를 참조하므로 자동으로 따라오고, 꼬리는 절대 짧아지지 않아 꼬리 위 인덱스를 들고
있는 정책(정지선 스냅)이 범위를 벗어나지 않는다. 정지 위치는 stop_s가 따로 정한다.

경로가 소진됐는데 못 넘었으면 끝 접선 방향으로 5km/h 직진(creep)한다. FINISH_CREEP_M
을 넘으면 포기하고 선다 -- 무한 직진은 도로 이탈 리스폰이다.

I/O도 소켓도 없다. stoplines.load_for_path / _path_heading을 그대로 재사용한다.
"""

from __future__ import annotations

import csv
import math
import os

import stoplines as SL

FINISH_OVERRUN_M = 8.0      # m, 종료선을 이만큼 지나쳐 선다. --finish-overrun
FINISH_ZONE_M = 30.0        # m, 경로 끝 이 안은 종료 구간 (obstacle 무시·우회 금지).
                            # 정지선 폴백의 **앞쪽** 탐색 한계이기도 하다. --finish-zone
FINISH_SEARCH_M = 150.0     # m, 정지선 폴백의 **뒤쪽**(경로 끝 너머) 탐색 한계. --finish-search
FINISH_FALLBACK_OVERRUN_M = 25.0    # m, path_end 폴백일 때의 overrun. --finish-fallback-overrun
END_STEP_M = 1.0            # m, 꼬리 점 간격 (경로 격자와 같게)
TANGENT_BACK_M = 3.0        # m, 끝 접선을 잴 때 뒤로 볼 거리 (Path.end_heading과 같은 규칙.
                            # 이음매의 0.0m 중복점에 흔들리지 않는다)
LINE_WARN_M = 3.0           # m, 종료선 CSV 좌표가 경로 접선 기준 이보다 어긋나면 경고
LINE_LAT_M = 6.0            # m, 통과 판정을 인정할 횡거리 (check_redlight.LAT_MAX와 같게)
DONE_M = 1.5                # m, pure_pursuit의 finished 래치와 같은 값
STOP_SPEED = SL.STOP_SPEED

# 라바콘 쌍 판정 -- 기하·운동. 치수는 CONE_MAX_LEN(절대)·CONE_SAME_SIZE(상대)뿐
CONE_SEARCH_M = 50.0        # m, 경로 끝 ± 또는 현재 종료선 ± 이 안의 정지 객체만
CONE_LAT_TOL = 1.5          # m, |lat_a| 와 |lat_b| 의 차 (좌우 대칭)
CONE_LON_TOL = 5.0          # m, 종방향 차 (한 선 위)
CONE_GAP_MIN = 1.8          # m, 간격 하한. 차폭 1.9가 물리적 하한 (2.0이면 lat ±1.0 배치가
                            # 부동소수 1.9999로 탈락한다)
CONE_GAP_MAX = 8.0          # m, 간격 상한. 사전테스트 라바콘(lat ±4.8~6)은 걸러진다
CONE_MAX_LEN = 3.0          # m, 이보다 긴 객체는 후보에서 뺀다 (주차 차량 쌍 오탐 방지)
CONE_SAME_SIZE = 0.2        # 두 객체 height·length 상대차 허용. "같은 크기 쌍"
CONE_STOPPED_MS = 0.3       # m/s, detour.STOPPED_MS와 같게
CONE_CONFIRM_FRAMES = 3     # 같은 id 쌍이 연속 이만큼 보이면 확정 (0.15초)
CONE_OPEN_M = 80.0          # m, 9910 수신 반경. 종료선 이 + CONE_SEARCH_M 안에서 탐색

FINISH_CREEP_V = 5.0 / 3.6  # m/s, 경로 소진 뒤 직진 속도
FINISH_CREEP_M = 30.0       # m, 이만큼 직진해도 못 넘으면 포기

_SRC_LABEL = {'cones': '라바콘 쌍', 'stopline': '정지선', 'path_end': '경로 끝'}


# ----------------------------------------------------------------------
# 기하 -- 꼬리

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


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _hermite(p0, h0, p1, h1, step: float = END_STEP_M) -> list:
    """p0(접선 h0) -> p1(접선 h1) 3차 Hermite. p0는 빼고 p1까지 ~step 간격."""
    d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    if d < 1e-6:
        return []
    n = max(1, int(round(d / step)))
    t0 = (d * math.cos(h0), d * math.sin(h0))
    t1 = (d * math.cos(h1), d * math.sin(h1))
    out = []
    for k in range(1, n + 1):
        u = k / n
        a = 2 * u ** 3 - 3 * u ** 2 + 1
        b = u ** 3 - 2 * u ** 2 + u
        c = -2 * u ** 3 + 3 * u ** 2
        e = u ** 3 - u ** 2
        out.append((a * p0[0] + b * t0[0] + c * p1[0] + e * t1[0],
                    a * p0[1] + b * t0[1] + c * p1[1] + e * t1[1]))
    return out


def plan_tail(real_pts: list, target, target_heading: float, overrun: float,
              min_len_m: float = 0.0, min_pts: int = 0,
              step: float = END_STEP_M) -> list:
    """실제 경로 끝 뒤에 붙일 꼬리. 목표점을 지나 overrun+1m 더 간다.

    목표가 끝과 같거나 앞(끝 접선 기준 lon <= 0.5)이면 끝 접선 직선만 (1차 구현과
    동일). 뒤면 끝점 -> 목표점 Hermite(시작 접선 = 끝 접선, 끝 접선 = 목표 헤딩) 뒤에
    목표 헤딩 직선. 총 길이·점 수가 min_len_m·min_pts에 못 미치면 마지막 방향으로
    직선을 더 붙인다 -- 꼬리 위 인덱스를 들고 있는 정책이 범위를 벗어나지 않게.
    """
    ex, ey = real_pts[-1]
    h0 = end_tangent(real_pts)
    lon_t = (target[0] - ex) * math.cos(h0) + (target[1] - ey) * math.sin(h0)
    need = overrun + 1.0
    if lon_t <= 0.5:
        tail = extend_points(real_pts, max(need + max(0.0, lon_t), 0.0), step)
        h_last = h0
    else:
        tail = _hermite((ex, ey), h0, tuple(target), target_heading, step)
        n = int(math.ceil(need / step - 1e-9))
        tail += [(target[0] + k * step * math.cos(target_heading),
                  target[1] + k * step * math.sin(target_heading))
                 for k in range(1, n + 1)]
        h_last = target_heading

    def _len(t):
        acc, px, py = 0.0, ex, ey
        for x, y in t:
            acc += math.hypot(x - px, y - py)
            px, py = x, y
        return acc

    while tail and (_len(tail) < min_len_m - 1e-9 or len(tail) < min_pts):
        lx, ly = tail[-1]
        tail.append((lx + step * math.cos(h_last), ly + step * math.sin(h_last)))
    return tail


def tail_stats(real_pts: list, tail: list) -> tuple:
    """(길이 m, 끝 접선 기준 최대 |lat| m)."""
    if not tail:
        return 0.0, 0.0
    ex, ey = real_pts[-1]
    h0 = end_tangent(real_pts)
    acc, px, py, worst = 0.0, ex, ey, 0.0
    for x, y in tail:
        acc += math.hypot(x - px, y - py)
        px, py = x, y
        lat = -(x - ex) * math.sin(h0) + (y - ey) * math.cos(h0)
        worst = max(worst, abs(lat))
    return acc, worst


def finish_index(path) -> int:
    """실제 경로의 마지막 인덱스. 연장이 없는 path(자기검사 스텁)도 받는다."""
    return getattr(path, 'finish_idx', len(path.pts) - 1)


def finish_s(path) -> float:
    return path.s[finish_index(path)]


def gate_s(path, zone_m: float = FINISH_ZONE_M) -> float:
    """이 s 이상은 종료 구간. obstacles/detour가 obstacle 분류를 무시하는 경계.

    실제 경로 끝 기준이다 -- 끝 너머(꼬리)는 전부 종료 구간이다.
    """
    return finish_s(path) - zone_m


def _frame_of(path, x: float, y: float, lo: int) -> tuple:
    """경로 끝 근처 객체의 (실제 끝 기준 lon, 부호 lat, 최근접 idx).

    [lo, len) 창의 최근접 점과 그 접선으로 잰다. 최근접이 마지막 점이면 그 접선으로
    외삽한다 (꼬리 밖 객체). lon은 실제 경로 끝(finish_s)을 0으로 한다.
    """
    n = len(path.pts)
    lo = max(0, min(lo, n - 1))
    i = min(range(lo, n), key=lambda k: math.hypot(path.pts[k][0] - x,
                                                   path.pts[k][1] - y))
    h = SL._path_heading(path, i)
    px, py = path.pts[i]
    along = (x - px) * math.cos(h) + (y - py) * math.sin(h)
    lat = -(x - px) * math.sin(h) + (y - py) * math.cos(h)
    if i < n - 1:
        along = max(-1.0, min(1.0, along))     # 점 격자 안에서만 보정
    return path.s[i] - finish_s(path) + along, lat, i


# ----------------------------------------------------------------------
# 종료선

class FinishLine:
    """경로에 붙인 종료선 하나. 위치와 방위는 출처에 따라 다르다.

    stopline(끝 30m 안): 경로 위 점 + 그 접선. stopline(끝 너머)·cones: CSV/중점 좌표 +
    그 헤딩. path_end: 경로 끝점 + 끝 접선. bind(path)로 꼬리 위 최근접 점 idx/s를 확정한다.
    """

    __slots__ = ('idx', 's', 'x', 'y', 'heading', 'src', 'ctrl',
                 'lon_off', 'lat_off', 'resid', 'ahead', 'cone_ids', 'end_lon')

    def __init__(self, idx, s, x, y, heading, src, ctrl=frozenset(),
                 lon_off=0.0, lat_off=0.0, resid=0.0, ahead=False,
                 cone_ids=(), end_lon=0.0):
        self.idx, self.s = idx, s
        self.x, self.y, self.heading = x, y, heading
        self.src = src                  # 'cones' / 'stopline' / 'path_end'
        self.ctrl = ctrl                # 신호 ctrl 집합 (정지선일 때)
        self.lon_off, self.lat_off = lon_off, lat_off   # CSV 좌표의 접선 기준 오차
        self.resid = resid
        self.ahead = ahead              # 실제 경로 끝보다 뒤에 있다 (꼬리 위)
        self.cone_ids = tuple(cone_ids)
        self.end_lon = end_lon          # 실제 경로 끝 접선 기준 lon

    def ctrl_str(self) -> str:
        return '|'.join(str(c) for c in sorted(self.ctrl)) if self.ctrl else '-'

    def label(self) -> str:
        if self.src == 'stopline':
            return f'stopline ctrl={self.ctrl_str()}' + (' 끝 너머' if self.ahead else '')
        if self.src == 'cones':
            return 'cones id=' + '/'.join(str(i) for i in self.cone_ids)
        return self.src

    def lon_of(self, x: float, y: float) -> float:
        """접선 기준 종방향 부호 거리. 양수 = 선을 넘었다."""
        return ((x - self.x) * math.cos(self.heading)
                + (y - self.y) * math.sin(self.heading))

    def lat_of(self, x: float, y: float) -> float:
        return (-(x - self.x) * math.sin(self.heading)
                + (y - self.y) * math.cos(self.heading))

    def bind(self, path) -> None:
        """꼬리를 붙인 뒤 경로 위 최근접 점의 idx/s를 확정한다."""
        fi = finish_index(path)
        lo = max(0, fi - 70)
        n = len(path.pts)
        self.idx = min(range(lo, n), key=lambda k: math.hypot(
            path.pts[k][0] - self.x, path.pts[k][1] - self.y))
        self.s = path.s[self.idx] + max(-1.0, min(1.0, (
            (self.x - path.pts[self.idx][0]) * math.cos(self.heading)
            + (self.y - path.pts[self.idx][1]) * math.sin(self.heading))))

    def __repr__(self) -> str:
        return f'<FinishLine s={self.s:.1f} {self.label()} lon_off={self.lon_off:+.2f}>'


def _read_stoplines(events_csv: str) -> list:
    rows = []
    with open(events_csv, newline='', encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            if r.get('kind') == 'stopline':
                rows.append(r)
    return rows


def _target_heading(h0: float, ex: float, ey: float, tx: float, ty: float) -> float:
    """끝 접선과 끝점->목표 chord의 거리 가중 평균. 멀수록 chord를 믿는다."""
    d = math.hypot(tx - ex, ty - ey)
    if d < 1.0:
        return h0
    chord = math.atan2(ty - ey, tx - ex)
    w = min(1.0, d / 40.0)
    return h0 + w * _wrap(chord - h0)


def load_for_path(path, events_csv: str, zone_m: float = FINISH_ZONE_M,
                  search_m: float = FINISH_SEARCH_M,
                  overrun: float = FINISH_OVERRUN_M,
                  fallback_overrun: float = FINISH_FALLBACK_OVERRUN_M) -> tuple:
    """종료선을 정한다. (FinishLine, 꼬리 점 목록, 경고 목록).

    호출자가 path.replace_tail(꼬리)를 한 뒤 line.bind(path)를 불러야 한다.
      (a) 스냅된 정지선 중 끝 zone_m 안의 최대 s -- 단 스냅 점이 끝보다 뒤거나 CSV
          좌표가 끝 너머(lon_off > 0.5)면 (b)로 보낸다 (스냅 점이 아니라 CSV 좌표가 목표다)
      (b) detail에 beyond_end가 있는 행 중 끝 접선 프레임 0 < lon <= search_m,
          |lat| <= LINE_LAT_M 인 최소 lon. 위치는 CSV 좌표
      (c) 없으면 경로 끝점, overrun = fallback_overrun
    """
    warns = []
    fi = finish_index(path)
    fs = path.s[fi]
    real = path.pts[:fi + 1]
    ex, ey = real[-1]
    h0 = end_tangent(real)

    def _lonlat(x, y):
        return ((x - ex) * math.cos(h0) + (y - ey) * math.sin(h0),
                -(x - ex) * math.sin(h0) + (y - ey) * math.cos(h0))

    rows = []
    if events_csv and os.path.exists(events_csv):
        rows = _read_stoplines(events_csv)
    else:
        warns.append('이벤트 CSV 없음')

    # (a) 끝 30m 안에 스냅된 정지선
    sl = None
    if rows:
        sls, _w = SL.load_for_path(path, events_csv)
        if sls:
            cand = max(sls, key=lambda z: z.s)
            lon_off, _lat = _lonlat(cand.x, cand.y)
            if fs - cand.s > zone_m:
                warns.append(f'마지막 스냅 정지선 s={cand.s:.1f}이 경로 끝 {fs:.1f}보다 '
                             f'{fs - cand.s:.1f}m 앞 (한계 {zone_m:.0f}m) ― 종료선 아님')
            elif cand.idx > fi or lon_off > 0.5:
                pass                # 끝 너머 -- (b)가 CSV 좌표로 잡는다
            else:
                sl = cand
    if sl is not None:
        idx = sl.idx
        h = SL._path_heading(path, idx)
        px, py = path.pts[idx]
        line = FinishLine(idx, path.s[idx], px, py, h, 'stopline', sl.ctrl,
                          resid=sl.resid, end_lon=path.s[idx] - fs)
        line.lon_off = line.lon_of(sl.x, sl.y)
        line.lat_off = line.lat_of(sl.x, sl.y)
        if abs(line.lon_off) > LINE_WARN_M:
            warns.append(f'종료선 CSV 좌표가 경로 점에서 종방향 {line.lon_off:+.2f}m '
                         f'(한계 {LINE_WARN_M:.0f}m) ― 이벤트 CSV가 오래됐을 수 있다')
        tail = plan_tail(real, (px, py), h, overrun)
        return line, tail, warns

    # (b) 경로 끝 너머 정지선 (route_events의 beyond_end 행)
    best = None
    for r in rows:
        x, y = float(r['x']), float(r['y'])
        lon, lat = _lonlat(x, y)
        det = r.get('detail', '') or ''
        if 'beyond_end' not in det:
            continue
        if not (0.0 < lon <= search_m) or abs(lat) > LINE_LAT_M:
            warns.append(f'끝 너머 정지선 s_along={float(r.get("s_along") or 0):.1f} '
                         f'lon={lon:+.1f} lat={lat:+.1f} ― 범위 밖 (뒤 {search_m:.0f}m, '
                         f'횡 {LINE_LAT_M:.0f}m)')
            continue
        if best is None or lon < best[0]:
            best = (lon, lat, x, y, SL.parse_ctrl(det), float(r.get('s_along') or 0))
    if best is not None:
        lon, lat, x, y, ctrl, s_along = best
        h = _target_heading(h0, ex, ey, x, y)
        line = FinishLine(fi, fs + lon, x, y, h, 'stopline', ctrl, ahead=True,
                          end_lon=lon, lat_off=lat)
        if abs((s_along - fs) - lon) > 10.0 and s_along > 0:
            warns.append(f'끝 너머 정지선 s_along {s_along:.1f}과 접선 lon {lon:+.1f}이 '
                         f'{abs(s_along - fs - lon):.0f}m 다르다 ― 이벤트 CSV와 경로가 '
                         f'다른 세대일 수 있다')
        tail = plan_tail(real, (x, y), h, overrun)
        return line, tail, warns

    # (c) 경로 끝
    warns.append(f'종료선을 경로 끝으로 본다 ― overrun {fallback_overrun:.0f}m로 늘린다 '
                 f'(정지선이 앞 {zone_m:.0f}m·뒤 {search_m:.0f}m 안에 없다)')
    line = FinishLine(fi, fs, ex, ey, h0, 'path_end')
    tail = plan_tail(real, (ex, ey), h0, fallback_overrun)
    return line, tail, warns


# ----------------------------------------------------------------------
# 라바콘 쌍

class ConeFinder:
    """objects[]에서 좌우 대칭 정지 객체 쌍을 찾는다. 치수는 거의 보지 않는다.

    분류(obstacles.classify)·finish_zone_ignore와 무관하게 원본 objects를 본다.
    """

    def __init__(self, path):
        self.path = path
        self._last_pair = None
        self._count = 0

    def reset(self) -> None:
        self._last_pair, self._count = None, 0

    def _candidates(self, objects, line_xy, ref_lon: float) -> list:
        fi = finish_index(self.path)
        ex, ey = self.path.pts[fi]
        lo = fi - 70
        out = []
        for o in objects:
            if o.speed >= CONE_STOPPED_MS or o.length > CONE_MAX_LEN:
                continue
            d_end = math.hypot(o.x - ex, o.y - ey)
            d_line = math.hypot(o.x - line_xy[0], o.y - line_xy[1])
            if min(d_end, d_line) > CONE_SEARCH_M + 10.0:
                continue
            lon, lat, _i = _frame_of(self.path, o.x, o.y, lo)
            if abs(lat) > CONE_GAP_MAX:
                continue
            if not (-CONE_SEARCH_M <= lon <= max(CONE_SEARCH_M, ref_lon + CONE_SEARCH_M)):
                continue
            out.append((o, lon, lat))
        return out

    @staticmethod
    def _same_size(a, b) -> bool:
        for va, vb in ((a.height, b.height), (a.length, b.length)):
            m = max(va, vb, 1e-6)
            if abs(va - vb) > CONE_SAME_SIZE * m:
                return False
        return True

    def pair(self, objects, line_xy, ref_lon: float):
        """조건을 만족하는 쌍 중 현재 종료선(ref_lon)에 가장 가까운 것. 없으면 None.

        반환: ((a, lon_a, lat_a), (b, lon_b, lat_b)) -- a가 좌(lat>0).
        """
        cands = self._candidates(objects, line_xy, ref_lon)
        best, best_d = None, float('inf')
        for i in range(len(cands)):
            oa, la, ta = cands[i]
            for j in range(i + 1, len(cands)):
                ob, lb, tb = cands[j]
                if ta * tb >= 0.0:
                    continue
                if abs(abs(ta) - abs(tb)) > CONE_LAT_TOL:
                    continue
                if abs(la - lb) > CONE_LON_TOL:
                    continue
                gap = math.hypot(oa.x - ob.x, oa.y - ob.y)
                if not (CONE_GAP_MIN <= gap <= CONE_GAP_MAX):
                    continue
                if not self._same_size(oa, ob):
                    continue
                d = abs(0.5 * (la + lb) - ref_lon)
                if d < best_d:
                    best_d = d
                    best = (((oa, la, ta), (ob, lb, tb)) if ta > 0
                            else ((ob, lb, tb), (oa, la, ta)))
        return best

    def update(self, objects, line_xy, ref_lon: float):
        """연속 CONE_CONFIRM_FRAMES 프레임 같은 id 쌍이면 그 쌍을, 아니면 None."""
        p = self.pair(objects, line_xy, ref_lon)
        if p is None:
            self._last_pair, self._count = None, 0
            return None
        ids = (p[0][0].id, p[1][0].id)
        if ids == self._last_pair:
            self._count += 1
        else:
            self._last_pair, self._count = ids, 1
        if self._count >= CONE_CONFIRM_FRAMES:
            return p
        return None


def line_from_cones(path, a, b, overrun: float) -> tuple:
    """두 라바콘 -> (FinishLine(cones), 꼬리). 중점이 종료선, 방위는 두 점을 잇는 선의
    법선 중 끝 접선과 같은 방향. 꼬리는 지금 꼬리보다 짧아지지 않는다."""
    fi = finish_index(path)
    real = path.pts[:fi + 1]
    h0 = end_tangent(real)
    mx, my = 0.5 * (a.x + b.x), 0.5 * (a.y + b.y)
    h = math.atan2(b.y - a.y, b.x - a.x) + math.pi / 2.0
    if abs(_wrap(h - h0)) > math.pi / 2.0:
        h += math.pi
    h = _wrap(h)
    ex, ey = real[-1]
    lon = (mx - ex) * math.cos(h0) + (my - ey) * math.sin(h0)
    lat = -(mx - ex) * math.sin(h0) + (my - ey) * math.cos(h0)
    line = FinishLine(fi, path.s[fi] + lon, mx, my, h, 'cones',
                      cone_ids=(a.id, b.id), end_lon=lon, lat_off=lat)
    old_tail = path.pts[fi + 1:]
    old_len = path.s[-1] - path.s[fi]
    tail = plan_tail(real, (mx, my), h, overrun,
                     min_len_m=old_len, min_pts=len(old_tail))
    return line, tail


# ----------------------------------------------------------------------
# 매 프레임

class FinishCmd:
    """한 프레임의 판정. lon은 후륜축의 종료선 기준 종방향 부호 거리(+ = 넘음).

    rem은 정지 목표(stop_s)까지 남은 거리 -- pure_pursuit의 끝 감속·끝 조향 블렌드가
    이것을 쓴다. v_floor는 creep 중에만 > 0.
    """

    __slots__ = ('lon', 'dist', 'rem', 'passed', 'done', 'near', 'creep',
                 'v_floor', 'retargeted', 'action')

    def __init__(self, lon=float('nan'), dist=float('nan'), rem=float('inf'),
                 passed=False, done=False, near=False, creep=False,
                 v_floor=0.0, retargeted=False, action='none'):
        self.lon, self.dist, self.rem = lon, dist, rem
        self.passed, self.done, self.near = passed, done, near
        self.creep, self.v_floor, self.retargeted = creep, v_floor, retargeted
        self.action = action


class FinishPolicy:
    """종료선 통과를 감시하고 정지 목표·완주 래치를 낸다. 속도 상한은 내지 않는다.

    감속·정지는 꼬리 위 끝 감속이 맡는다 (rem = stop_s - s_ego). 이 클래스는
    "넘었는가"·"어디에 설 것인가"·"라바콘 쌍이 보이는가"를 판정하고 이벤트를 남긴다.
    """

    def __init__(self, path, line: FinishLine, *, overrun: float = FINISH_OVERRUN_M,
                 zone_m: float = FINISH_ZONE_M, on_event=None):
        self.path = path
        self.line = line
        self.overrun = overrun
        self.zone_m = zone_m
        self.finish_s = finish_s(path)
        self._on_event = on_event or (lambda kind, detail: None)
        self._cones = ConeFinder(path)
        self.locked = line.src == 'cones'
        self._set_stop()
        self.reset()

    def _set_stop(self) -> None:
        # path_end 폴백은 꼬리 전체가 overrun이다 (plan_tail이 fallback_overrun으로 냈다)
        if self.line.src == 'path_end':
            self.stop_s = self.path.length
        else:
            self.stop_s = min(self.path.length, self.line.s + self.overrun + 1.0)

    def reset(self) -> None:
        """리스폰 시. 위치가 강제로 옮겨졌으므로 통과 래치와 creep를 버린다.
        라바콘 래치(locked)는 유지한다 -- 종료선 위치는 리스폰과 무관하다."""
        self.passed = False
        self.near = False
        self.max_lon = float('-inf')
        self._short = False
        self._creep_from = None
        self._creep_m = 0.0
        self._creep_gave_up = False
        self._prev_xy = None
        self._cones.reset()

    def adopt(self, old: 'FinishPolicy') -> None:
        """우회 교체 -- 옛 정책이 라바콘을 래치했으면 새 경로에도 같은 종료선을 건다."""
        if old is None or old.line.src != 'cones' or self.locked:
            return
        ln = old.line

        class _P:
            __slots__ = ('id', 'x', 'y')

            def __init__(self, i, x, y):
                self.id, self.x, self.y = i, x, y

        # 중점과 방위만 옮긴다 (원본 라바콘 좌표는 line에 없다). 법선 방향 ±1m
        nx, ny = -math.sin(ln.heading), math.cos(ln.heading)
        a = _P(ln.cone_ids[0] if ln.cone_ids else 0, ln.x + nx, ln.y + ny)
        b = _P(ln.cone_ids[1] if len(ln.cone_ids) > 1 else 0, ln.x - nx, ln.y - ny)
        line, tail = line_from_cones(self.path, a, b, self.overrun)
        self.path.replace_tail(tail)
        line.bind(self.path)
        self.line = line
        self.locked = True
        self._set_stop()
        self.passed = old.passed
        self.max_lon = old.max_lon

    def _retarget(self, pair, t: float) -> None:
        (a, la, ta), (b, lb, tb) = pair
        prev = self.line
        line, tail = line_from_cones(self.path, a, b, self.overrun)
        self.path.replace_tail(tail)
        line.bind(self.path)
        gap = math.hypot(a.x - b.x, a.y - b.y)
        d_prev = line.lon_of(prev.x, prev.y)
        self.line = line
        self.locked = True
        self._set_stop()
        self._on_event(
            'finish_line_from_cones',
            f'id={a.id} ({a.x:.2f},{a.y:.2f}) lat={ta:+.2f} / '
            f'id={b.id} ({b.x:.2f},{b.y:.2f}) lat={tb:+.2f} ― 중점 ({line.x:.2f},{line.y:.2f}) '
            f'간격 {gap:.2f}m, 경로 끝 기준 lon {line.end_lon:+.1f}m lat {line.lat_off:+.2f}m, '
            f'종료선 s={line.s:.1f} (이전 {_SRC_LABEL.get(prev.src, prev.src)} s={prev.s:.1f}, '
            f'{-d_prev:+.1f}m 차이), 정지 목표 s={self.stop_s:.1f}, '
            f'꼬리 {self.path.length - self.finish_s:.0f}m')

    def update(self, idx: int, s_ego: float, ego_x: float, ego_y: float,
               ego_heading: float, v: float, objects, t: float) -> FinishCmd:
        retargeted = False
        # 라바콘 쌍 -- 종료선 근처에서만, 확정 전까지만
        if not self.locked and objects and (
                self.line.s - s_ego <= CONE_OPEN_M + CONE_SEARCH_M):
            pair = self._cones.update(objects, (self.line.x, self.line.y),
                                      self.line.s - self.finish_s)
            if pair is not None:
                self._retarget(pair, t)
                retargeted = True

        lon = self.line.lon_of(ego_x, ego_y)
        lat = self.line.lat_of(ego_x, ego_y)
        self.near = (self.line.s - s_ego) <= self.zone_m
        dist = self.line.s - s_ego
        rem = self.stop_s - s_ego

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

        # **선을 넘기 전에는 절대 done이 아니다.** 경로(정지 목표)가 소진됐는데 못
        # 넘었으면 끝 접선 방향으로 저속 직진한다. 조향은 pure_pursuit의 끝 블렌드가
        # rem->0에서 끝 접선 hold 100%라 그대로 직진이다.
        creep, v_floor = False, 0.0
        exhausted = rem < DONE_M
        if self.passed:
            done = exhausted and v < STOP_SPEED
        else:
            done = False
            if exhausted and not self._creep_gave_up:
                if self._creep_from is None:
                    self._creep_from = (ego_x, ego_y)
                    self._creep_m = 0.0
                    self._on_event(
                        'finish_creep',
                        f'정지 목표 s={self.stop_s:.1f}에 닿았는데 종료선을 못 넘었다 '
                        f'(lon={lon:+.2f}m 횡 {lat:+.2f}m, 종료선 s={self.line.s:.1f} '
                        f'{self.line.label()}) ― 끝 접선으로 {FINISH_CREEP_V * 3.6:.0f}km/h '
                        f'직진, 최대 {FINISH_CREEP_M:.0f}m')
                if self._prev_xy is not None:
                    step = math.hypot(ego_x - self._prev_xy[0], ego_y - self._prev_xy[1])
                    if step < 10.0:
                        self._creep_m += step
                if self._creep_m >= FINISH_CREEP_M:
                    self._creep_gave_up = True
                    self._on_event(
                        'finish_creep_giveup',
                        f'{self._creep_m:.1f}m 직진해도 종료선을 못 넘었다 '
                        f'(lon={lon:+.2f}m 횡 {lat:+.2f}m) ― 정지한다')
                else:
                    creep, v_floor = True, FINISH_CREEP_V
            if self._creep_gave_up and exhausted:
                done = v < STOP_SPEED
                if done and not self._short:
                    self._short = True
                    self._on_event(
                        'finish_short',
                        f'종료선을 못 넘고 섰다 ― lon={lon:+.2f}m 횡 {lat:+.2f}m '
                        f'(종료선 s={self.line.s:.1f}, 정지 목표 {self.stop_s:.1f})')
        self._prev_xy = (ego_x, ego_y)

        action = ('done' if done else 'creep' if creep else 'passed' if self.passed
                  else 'near' if self.near else 'none')
        return FinishCmd(lon, dist, rem, self.passed, done, self.near, creep,
                         v_floor, retargeted, action)


def apply_meta_tail(path, meta: dict) -> bool:
    """세션 meta.json의 finish_tail(런타임 최종 꼬리)을 경로에 다시 붙인다.
    계수기(check_*)가 라바콘 런의 꼬리 프레임을 제대로 투영하게 한다."""
    tail = (meta or {}).get('finish_tail')
    if not tail or not hasattr(path, 'replace_tail'):
        return False
    try:
        pts = [(float(p[0]), float(p[1])) for p in tail]
    except (TypeError, ValueError, IndexError):
        return False
    if not pts:
        return False
    path.replace_tail(pts)
    return True


def summary(policy: FinishPolicy, overrun: float) -> list:
    ln = policy.line
    fi = finish_index(policy.path)
    tail_m, tail_lat = tail_stats(policy.path.pts[:fi + 1], policy.path.pts[fi + 1:])
    out = [f'  종료선 s={ln.s:.1f} ({ln.label()}) ― 경로 끝 {policy.finish_s:.1f}m 기준 '
           f'{ln.end_lon:+.1f}m, 정지 목표 s={policy.stop_s:.1f} '
           f'(종료선 +{policy.stop_s - ln.s:.1f}m), 꼬리 {tail_m:.0f}m (최대 횡 {tail_lat:.1f}m), '
           f'종료 구간 {policy.zone_m:.0f}m'
           + (f' + 끝 너머 {ln.end_lon:.0f}m' if ln.end_lon > 0.5 else '')]
    if ln.src == 'stopline' and not ln.ahead:
        out.append(f'  CSV 좌표 오차 종 {ln.lon_off:+.2f}m 횡 {ln.lat_off:+.2f}m '
                   f'(스냅 잔차 {ln.resid:.2f}m)')
    if ln.src == 'path_end':
        out.append('  *** 정지선도 라바콘도 없다 ― 라바콘 쌍이 보이면 런타임에 잡는다. '
                   f'못 보면 꼬리 끝(끝 +{policy.stop_s - policy.finish_s:.0f}m)에 선다 ***')
    return out


# ----------------------------------------------------------------------
# 진단 -- 대회 당일 4단계 전에 눈으로 볼 표

def _segments_of(csv_path: str) -> list:
    seen = []
    with open(csv_path, newline='', encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            if r['segment_id'] not in seen:
                seen.append(r['segment_id'])
    return seen


class _Obj:
    __slots__ = ('id', 'x', 'y', 'speed', 'length', 'width', 'height')

    def __init__(self, i, x, y, speed=0.0, length=0.3, width=0.3, height=0.5):
        self.id, self.x, self.y, self.speed = i, x, y, speed
        self.length, self.width, self.height = length, width, height


def _plan(args) -> int:
    from pure_pursuit import load_path

    segs = ([s.strip() for s in args.segments.split(',') if s.strip()]
            if args.segments else _segments_of(args.path))
    path, _b = load_path(args.path, segs, extend_m=args.overrun + 1.0)
    ev = args.events or SL.default_events_path(args.path)
    line, tail, warns = load_for_path(path, ev, zone_m=args.zone, search_m=args.search,
                                      overrun=args.overrun,
                                      fallback_overrun=args.fallback_overrun)
    path.replace_tail(tail)
    line.bind(path)
    pol = FinishPolicy(path, line, overrun=args.overrun, zone_m=args.zone)
    n_real = path.n_real
    print(f'[fin] 경로 {os.path.basename(args.path)}: 실제 {n_real}점 '
          f'{path.finish_s:.2f}m, 끝 ({path.pts[path.finish_idx][0]:.3f},'
          f'{path.pts[path.finish_idx][1]:.3f}) 접선 '
          f'{math.degrees(end_tangent(path.pts[:n_real])):+.1f}°')
    print(f'[fin] 이벤트 CSV: {ev or "(없음)"}')
    print(f'[fin] 종료선: {_SRC_LABEL.get(line.src, line.src)} s={line.s:.2f} ({line.label()}) '
          f'({line.x:.3f},{line.y:.3f}) 방위 {math.degrees(line.heading):+.1f}° '
          f'경로 끝 기준 lon {line.end_lon:+.2f}m lat {line.lat_off:+.2f}m')
    for s in summary(pol, args.overrun):
        print('[fin]' + s)
    print(f'[fin] 종료 구간: s>={gate_s(path, args.zone):.1f} ― obstacle 무시·우회 금지')
    for w in warns:
        print(f'[fin] *** {w} ***')
    if args.cones:
        v = [float(x) for x in args.cones.split(',')]
        objs = [_Obj(901, v[0], v[1]), _Obj(902, v[2], v[3])]
        ev_log = []
        pol = FinishPolicy(path, line, overrun=args.overrun, zone_m=args.zone,
                           on_event=lambda k, d: ev_log.append((k, d)))
        i0 = max(0, path.finish_idx - 60)
        x0, y0 = path.pts[i0]
        cmd = None
        for k in range(CONE_CONFIRM_FRAMES):
            cmd = pol.update(i0, path.s[i0], x0, y0, 0.0, 10.0, objs, k * 0.05)
        print('[fin] --cones 시험: ' + ('확정 ― ' + ev_log[-1][1] if cmd.retargeted
                                      else '*** 쌍으로 인정되지 않았다 ***'))
    if line.src == 'path_end':
        print('[fin] *** 종료선이 정지선도 라바콘도 아니다. 라바콘 위치를 현장에서 확인하고 '
              f'꼬리 {path.length - path.finish_s:.0f}m가 그 차이를 덮는지 볼 것 ***')
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


def _drive(path, pol, s0=0.0, v0=12.5, n_max=4000, objects=()):
    """등감속 정지 프로파일로 끝까지 달린다 (pure_pursuit과 같은 식). (hist, cmd).
    x축 직선 경로 전용 (ego_x = s)."""
    s, v, dt, hist = s0, v0, 0.05, []
    idx = 0
    cmd = None
    for _k in range(n_max):
        while idx < len(path.pts) - 1 and path.s[idx + 1] <= s:
            idx += 1
        idx = min(idx, len(path.pts) - 1)
        cmd = pol.update(idx, path.s[idx], s, 0.0, 0.0, v, objects, _k * dt)
        hist.append((s, v, cmd))
        if cmd.done:
            break
        v_cmd = min(12.5, math.sqrt(2.0 * 1.5 * max(0.0, cmd.rem - 1.0)))
        if cmd.v_floor > v_cmd:
            v_cmd = cmd.v_floor
        a = max(-6.0, min(3.0, 1.2 * (v_cmd - v)))
        if v_cmd <= 0.0:
            a = min(a, -1.5)
        s += v * dt
        v = max(0.0, v + a * dt)
    return hist, cmd


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

    # 2. Path 메타 / replace_tail
    p = Path(line_pts + extend_points(line_pts, 9.0), n_real=len(line_pts))
    check('Path: finish_s는 실제 끝', abs(p.finish_s - 100.0) < 1e-9, f'{p.finish_s}')
    check('Path: length는 연장 포함', abs(p.length - 109.0) < 1e-9, f'{p.length}')
    check('Path: 연장 없으면 finish_s == length',
          Path(line_pts).finish_s == Path(line_pts).length)
    p.replace_tail(extend_points(line_pts, 30.0))
    check('replace_tail: length 갱신·finish_s 불변',
          abs(p.length - 130.0) < 1e-9 and abs(p.finish_s - 100.0) < 1e-9
          and p.n_real == 101 and len(p.pts) == 131,
          f'{p.length} {p.finish_s} {len(p.pts)}')
    check('replace_tail: s 단조', all(p.s[i] < p.s[i + 1] for i in range(len(p.s) - 1)))

    # 3. plan_tail -- 직선 / Hermite / 패딩
    t = plan_tail(line_pts, (100.0, 0.0), 0.0, 8.0)
    check('plan_tail: 목표=끝이면 9m 직선', len(t) == 9 and abs(t[-1][0] - 109.0) < 1e-9, f'{t[-1]}')
    t = plan_tail(line_pts, (115.0, 3.0), 0.0, 8.0)
    steps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip([line_pts[-1]] + t, t)]
    check('plan_tail: 15m 뒤 횡 3m -- 점 간격 0.8~1.25m',
          all(0.8 <= d <= 1.25 for d in steps), f'{min(steps):.2f}~{max(steps):.2f}')
    check('plan_tail: 목표점을 지난다',
          any(abs(x - 115.0) < 0.6 and abs(y - 3.0) < 0.3 for x, y in t),
          f'{[q for q in t if abs(q[0] - 115) < 1.5]}')
    check('plan_tail: 시작 접선 = 끝 접선', abs(t[0][1]) < 0.1, f'{t[0]}')
    check('plan_tail: 끝 헤딩 = 목표 헤딩',
          abs(t[-1][1] - 3.0) < 1e-6 and abs(t[-1][0] - 124.0) < 1e-6, f'{t[-1]}')
    worst = 0.0
    pts3 = [line_pts[-1]] + t
    for a, b, c in zip(pts3, pts3[1:], pts3[2:]):
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        den = (math.hypot(b[0] - a[0], b[1] - a[1]) * math.hypot(c[0] - b[0], c[1] - b[1])
               * math.hypot(c[0] - a[0], c[1] - a[1]))
        if den > 1e-9:
            worst = max(worst, abs(2 * cross / den))
    # 15m에 횡 3m는 반경 ~15m. 조향 한계 0.35rad(휠베이스 2.7)의 최소 반경 7.6m에 여유
    check('plan_tail: 최대 곡률 < 1/12 (반경 12m 이상)', worst < 1.0 / 12.0, f'{worst:.4f}')
    t2 = plan_tail(line_pts, (115.0, 3.0), 0.0, 8.0, min_len_m=60.0, min_pts=70)
    tl, _ = tail_stats(line_pts, t2)
    check('plan_tail: min_len/min_pts 패딩', tl >= 60.0 and len(t2) >= 70, f'{tl:.1f} {len(t2)}')
    t3 = plan_tail(line_pts, (95.0, 0.0), 0.0, 8.0)
    check('plan_tail: 목표가 끝보다 앞이면 직선 9m', len(t3) == 9, f'{len(t3)}')

    # 4. 종료선 = 경로 끝 정지선. 통과 래치·이벤트 1회·정지 위치 (1차 구현과 동일)
    with tempfile.TemporaryDirectory() as td:
        wp = os.path.join(td, 'w.csv')
        ev = os.path.join(td, 'w_events.csv')
        with open(wp, 'w', encoding='utf-8') as fh:
            fh.write('segment_id,seq,x,y\n')
            for i, (x, y) in enumerate(line_pts):
                fh.write(f'a,{i},{x},{y}\n')

        def write_ev(rows):
            with open(ev, 'w', encoding='utf-8') as fh:
                fh.write('kind,s_along,x,y,road_id,lane_id,detail\n')
                for r in rows:
                    fh.write(r + '\n')

        def build(ev_path, **kw):
            path, _b = load_path(wp, ['a'], extend_m=9.0)
            line, tail, warns = load_for_path(path, ev_path, **kw)
            path.replace_tail(tail)
            line.bind(path)
            return path, line, warns

        write_ev(['stopline,40.0,40.0,0.0,1,-1,ctrl=7',
                  'stopline,100.0,99.85,-2.85,1,-1,ctrl=15|16'])
        path, line, warns = build(ev)
        check('종료선: 마지막 정지선', line.src == 'stopline' and abs(line.s - 100.0) < 1e-9,
              repr(line))
        check('종료선: 위치는 경로 점', abs(line.x - 100.0) < 1e-9 and abs(line.y) < 1e-9,
              f'({line.x},{line.y})')
        check('종료선: CSV 오차 종 -0.15 횡 -2.85',
              abs(line.lon_off + 0.15) < 1e-6 and abs(line.lat_off + 2.85) < 1e-6,
              f'{line.lon_off:+.2f}/{line.lat_off:+.2f}')
        check('종료선: ctrl 유지', line.ctrl == frozenset({15, 16}), line.ctrl_str())
        check('종료선: 경고 없음', not warns, ' / '.join(warns))
        check('종료선: 꼬리 9m', abs(path.length - 109.0) < 1e-9, f'{path.length}')

        rec = _Rec()
        pol = FinishPolicy(path, line, on_event=rec)
        check('stop_s == length (재조준 없음)', abs(pol.stop_s - path.length) < 1e-9)
        hist, cmd = _drive(path, pol)
        check('통과: finish_line_passed 1회',
              rec.kinds().count('finish_line_passed') == 1, ','.join(rec.kinds()))
        first = next((h for h in hist if h[2].passed), None)
        check('통과: 래치 시점 lon>0이고 작다',
              first is not None and 0.0 < first[2].lon < 1.0,
              f'{first[2].lon if first else None}')
        check('통과: 통과 속도가 걷는 속도보다 빠르다 (끝에서 서지 않는다)',
              first is not None and first[1] > 3.0, f'{first[1] if first else None} m/s')
        check('정지: done 래치', hist[-1][2].done)
        check('정지: 종료선 +6~11m (프로파일 +8, P제어 오버슈트 +2)',
              6.0 <= hist[-1][0] - line.s <= 11.0, f'{hist[-1][0] - line.s:+.2f}m')
        check('정지: finish_short/creep 없음',
              not {'finish_short', 'finish_creep'} & set(rec.kinds()), ','.join(rec.kinds()))
        check('근접: near는 종료선 30m 안에서만',
              all(h[2].near == (line.s - h[0] <= FINISH_ZONE_M) for h in hist))
        check('통과 후 max_lon 누적', pol.max_lon > 5.0, f'{pol.max_lon:.2f}')

        # 5. 적색 대기 -- 선 4.8m 앞 정지 상태에서 done도 passed도 아니다
        pol.reset()
        cmd = pol.update(path.nearest(95.2, 0.0), 95.2, 95.2, 0.0, 0.0, 0.0, [], 0.0)
        check('적색 대기: passed 아님', not cmd.passed and cmd.lon < 0.0, f'{cmd.lon:+.2f}')
        check('적색 대기: done 아님', not cmd.done)
        check('적색 대기: near', cmd.near)

        # 6. 자기교차 방어
        pol.reset()
        cmd = pol.update(0, 0.0, 130.0, 0.0, 0.0, 12.5, [], 0.0)
        check('멀리서 lon>0: 통과 인정 안 함', not cmd.passed and cmd.lon > 0.0)
        cmd = pol.update(path.nearest(99.0, 0.0), 99.0, 101.0, 8.0, 0.0, 3.0, [], 1.0)
        check('횡 8m: 통과 인정 안 함', not cmd.passed)

        # 7. 폴백 (c) -- 정지선이 멀면 경로 끝 + 26m 꼬리
        write_ev(['stopline,40.0,40.0,0.0,1,-1,ctrl=7'])
        path2, line2, w2 = build(ev)
        check('폴백: 정지선이 멀면 경로 끝', line2.src == 'path_end' and abs(line2.s - 100.0) < 1e-9,
              repr(line2))
        check('폴백: 경고를 낸다', bool(w2), ' / '.join(w2))
        check('폴백: 꼬리 26m', abs(path2.length - 126.0) < 1e-9, f'{path2.length}')
        pol2 = FinishPolicy(path2, line2)
        check('폴백: stop_s = 끝 +26', abs(pol2.stop_s - 126.0) < 1e-9, f'{pol2.stop_s}')
        hist, cmd = _drive(path2, pol2)
        check('폴백: 끝 +23~28m에 선다', 23.0 <= hist[-1][0] - 100.0 <= 28.0,
              f'{hist[-1][0] - 100.0:+.1f}')
        path3, line3, w3 = build('')
        check('폴백: CSV 없음', line3.src == 'path_end' and bool(w3))
        check('폴백: 접선은 끝 접선', abs(line3.heading - end_tangent(line_pts)) < 1e-6)
        rec = _Rec()
        pol = FinishPolicy(path3, line3, on_event=rec)
        cmd = pol.update(path3.nearest(100.5, 0.0), 100.5, 100.5, 0.0, 0.0, 4.0, [], 0.0)
        check('폴백: 경로 끝 통과 판정', cmd.passed and 'finish_line_passed' in rec.kinds())

        # 8. 폴백 (b) -- 끝 너머 정지선 (beyond_end 행)
        write_ev(['stopline,40.0,40.0,0.0,1,-1,ctrl=7',
                  'stopline,180.7,180.5,1.0,1,-1,beyond_end'])
        path4, line4, w4 = build(ev)
        check('끝 너머 정지선: stopline ahead', line4.src == 'stopline' and line4.ahead,
              repr(line4))
        check('끝 너머 정지선: s ≈ 180.5', abs(line4.s - 180.5) < 0.6, f'{line4.s:.2f}')
        check('끝 너머 정지선: 꼬리 ≈ 80.5+9', abs(path4.length - 189.5) < 1.0, f'{path4.length:.1f}')
        pol4 = FinishPolicy(path4, line4)
        check('끝 너머 정지선: stop_s = 선 +9', abs(pol4.stop_s - (line4.s + 9.0)) < 1e-6,
              f'{pol4.stop_s:.2f} vs {line4.s + 9:.2f}')
        rec = _Rec()
        pol4 = FinishPolicy(path4, line4, on_event=rec)
        hist, cmd = _drive(path4, pol4)
        check('끝 너머 정지선: 통과 뒤 +6~11m 정지',
              'finish_line_passed' in rec.kinds() and 6.0 <= hist[-1][0] - line4.s <= 11.0,
              f'{hist[-1][0] - line4.s:+.2f} {rec.kinds()}')
        write_ev(['stopline,180.7,180.5,0.0,1,-1,ctrl=15|16 beyond_end'])
        _p, line5, _w = build(ev)
        check('끝 너머 신호 정지선: ctrl 보존', line5.ctrl == frozenset({15, 16}), line5.ctrl_str())
        for lon_t in (7.0, 12.0):
            write_ev([f'stopline,{100 + lon_t:.1f},{100 + lon_t:.1f},0.0,1,-1,beyond_end'])
            _p, l6, _w = build(ev)
            check(f'끝 +{lon_t:.0f}m 정지선: CSV 좌표가 목표',
                  l6.src == 'stopline' and abs(l6.s - (100 + lon_t)) < 0.3 and l6.ahead,
                  repr(l6))
        write_ev(['stopline,300.0,300.0,0.0,1,-1,beyond_end'])
        _p, l7, _w = build(ev)
        check('끝 너머 200m: 범위 밖 -> path_end', l7.src == 'path_end')
        write_ev(['stopline,120.0,120.0,8.0,1,-1,beyond_end'])
        _p, l8, _w = build(ev)
        check('끝 너머 횡 8m: 범위 밖 -> path_end', l8.src == 'path_end')
        write_ev(['stopline,120.0,120.0,0.0,1,-1,'])
        _p, l9, _w = build(ev)
        check('beyond_end 표시 없는 끝 너머 행은 무시', l9.src == 'path_end')

        # 9. done은 passed 없이는 거짓 / creep / giveup
        write_ev(['stopline,40.0,40.0,0.0,1,-1,ctrl=7',
                  'stopline,100.0,100.0,0.0,1,-1,ctrl=15|16'])
        pathc, _lc, _w = build(ev)
        # 종료선을 꼬리 끝 너머로 옮겨 "경로 소진 + 미통과"를 만든다
        linec = FinishLine(pathc.finish_idx, 140.0, 140.0, 0.0, 0.0, 'stopline',
                           frozenset(), ahead=True, end_lon=40.0)
        rec = _Rec()
        polc = FinishPolicy(pathc, linec, on_event=rec)
        polc.stop_s = pathc.length                  # 109
        hist, cmd = _drive(pathc, polc, n_max=3000)
        ks = rec.kinds()
        check('creep: finish_creep 발화', 'finish_creep' in ks, ','.join(ks))
        cr = [h for h in hist if h[2].creep]
        check('creep: v_floor 5km/h',
              bool(cr) and all(abs(h[2].v_floor - FINISH_CREEP_V) < 1e-9 for h in cr))
        check('creep: 도중 done 없음', all(not h[2].done for h in cr))
        check('creep: 30m 뒤 giveup 후 정지', 'finish_creep_giveup' in ks and hist[-1][2].done
              and 'finish_short' in ks, f'{ks} s={hist[-1][0]:.1f}')
        check('creep: 진행 거리 30~35m', 137.0 <= hist[-1][0] <= 145.0, f'{hist[-1][0]:.1f}')
        lined = FinishLine(pathc.finish_idx, 105.0, 105.0, 0.0, 0.0, 'stopline',
                           frozenset(), ahead=True, end_lon=5.0)
        pathc.replace_tail(plan_tail(line_pts, (105.0, 0.0), 0.0, 8.0))
        rec = _Rec()
        pold = FinishPolicy(pathc, lined, on_event=rec)
        hist, cmd = _drive(pathc, pold)
        check('꼬리 안 종료선: creep 없이 통과·정지',
              'finish_creep' not in rec.kinds() and 'finish_line_passed' in rec.kinds()
              and hist[-1][2].done and 6.0 <= hist[-1][0] - 105.0 <= 11.0,
              f'{rec.kinds()} {hist[-1][0] - 105.0:+.1f}')

        # 10. reset 뒤 재발화 / 경로 끝 미통과는 done이 아니다
        rec = _Rec()
        pol = FinishPolicy(path, line, on_event=rec)
        pol.update(path.nearest(100.5, 0.0), 100.5, 100.5, 0.0, 0.0, 4.0, [], 0.0)
        pol.reset()
        pol.update(path.nearest(100.5, 0.0), 100.5, 100.5, 0.0, 0.0, 4.0, [], 1.0)
        check('reset: 이벤트 재발화', rec.kinds().count('finish_line_passed') == 2,
              ','.join(rec.kinds()))
        rec = _Rec()
        short_path = Path(line_pts[:95] + extend_points(line_pts[:95], 1.0), n_real=95)
        pol = FinishPolicy(short_path, line, on_event=rec)
        cmd = pol.update(len(short_path.pts) - 1, short_path.length, 94.5, 0.0, 0.0, 0.0, [], 0.0)
        check('경로 끝에 섰는데 미통과: done 아님·creep', not cmd.done and cmd.creep
              and 'finish_creep' in rec.kinds(), ','.join(rec.kinds()))

        # 11. 라바콘 쌍 -- lt 실측 좌표를 직선 경로 프레임으로 옮긴 것과 같다
        path, line, _w = build(ev)          # 끝 100 정지선, 꼬리 9m
        cones = [_Obj(10, 115.2, 1.45, 0.0, 0.08, 0.08, 1.05),
                 _Obj(11, 115.35, -1.41, 0.0, 0.08, 0.08, 1.05)]
        cf = ConeFinder(path)
        p = cf.pair(cones, (line.x, line.y), 0.0)
        check('쌍: lt 실측 (lon +15.2, lat ±1.4, 간격 2.86)', p is not None
              and p[0][0].id == 10 and p[1][0].id == 11, f'{p}')
        big = [_Obj(10, 115.2, 1.45, 0.0, 0.3, 0.3, 0.8),
               _Obj(11, 115.35, -1.41, 0.0, 0.3, 0.3, 0.8)]
        check('쌍: 치수 0.3x0.3x0.8도 같다', cf.pair(big, (line.x, line.y), 0.0) is not None)
        near = [_Obj(1, 100.0, 1.0, 0.0, 0.3, 0.3, 0.5), _Obj(2, 100.0, -1.0, 0.0, 0.3, 0.3, 0.5)]
        p = cf.pair(near, (line.x, line.y), 0.0)
        check('쌍: pt2 배치 (끝 s, lat ±1.0, 간격 2.0)', p is not None, f'{p}')
        near15 = [_Obj(1, 100.0, 1.5, 0.0, 0.3, 0.3, 0.5), _Obj(2, 100.0, -1.5, 0.0, 0.3, 0.3, 0.5)]
        check('쌍: lat ±1.5', cf.pair(near15, (line.x, line.y), 0.0) is not None)
        before = [_Obj(1, 60.0, 1.5, 0.0, 0.3, 0.3, 0.5), _Obj(2, 60.0, -1.5, 0.0, 0.3, 0.3, 0.5)]
        check('쌍: 끝 40m 앞도 인정 (경유지가 정지선을 지나친 경우)',
              cf.pair(before, (line.x, line.y), 0.0) is not None)
        rej = {
            '차량 쌍 (길이 4.39)': [_Obj(1, 110.0, 2.5, 0.0, 4.39, 1.81, 1.35),
                                   _Obj(2, 110.0, -2.5, 0.0, 4.39, 1.81, 1.35)],
            '한쪽만': [_Obj(1, 110.0, 1.5, 0.0, 0.3, 0.3, 0.5)],
            '간격 1.5m': [_Obj(1, 110.0, 0.75, 0.0, 0.3, 0.3, 0.5),
                         _Obj(2, 110.0, -0.75, 0.0, 0.3, 0.3, 0.5)],
            '간격 9m': [_Obj(1, 110.0, 4.5, 0.0, 0.3, 0.3, 0.5),
                       _Obj(2, 110.0, -4.5, 0.0, 0.3, 0.3, 0.5)],
            '한쪽 speed 1.0': [_Obj(1, 110.0, 1.5, 1.0, 0.3, 0.3, 0.5),
                              _Obj(2, 110.0, -1.5, 0.0, 0.3, 0.3, 0.5)],
            '비대칭 lat +1/-3': [_Obj(1, 110.0, 1.0, 0.0, 0.3, 0.3, 0.5),
                                _Obj(2, 110.0, -3.0, 0.0, 0.3, 0.3, 0.5)],
            'lon 차 6m': [_Obj(1, 110.0, 1.5, 0.0, 0.3, 0.3, 0.5),
                         _Obj(2, 116.0, -1.5, 0.0, 0.3, 0.3, 0.5)],
            '같은 쪽': [_Obj(1, 110.0, 1.5, 0.0, 0.3, 0.3, 0.5),
                       _Obj(2, 110.0, 3.5, 0.0, 0.3, 0.3, 0.5)],
            '끝 60m 앞': [_Obj(1, 40.0, 1.5, 0.0, 0.3, 0.3, 0.5),
                         _Obj(2, 40.0, -1.5, 0.0, 0.3, 0.3, 0.5)],
            '보행자+자전거 (크기 다름)': [_Obj(1, 110.0, 1.5, 0.0, 0.6, 0.7, 1.8),
                                          _Obj(2, 110.0, -1.5, 0.0, 2.08, 0.85, 1.93)],
        }
        for name, objs in rej.items():
            check(f'오탐 거부: {name}', cf.pair(objs, (line.x, line.y), 0.0) is None)
        many = cones + [_Obj(20, 130.0, 1.5, 0.0, 0.3, 0.3, 0.5),
                        _Obj(21, 130.0, -1.5, 0.0, 0.3, 0.3, 0.5)]
        p = cf.pair(many, (line.x, line.y), 0.0)
        check('쌍 여럿: 종료선에 가장 가까운 것',
              p is not None and {p[0][0].id, p[1][0].id} == {10, 11})
        cf.reset()
        got = [cf.update(cones, (line.x, line.y), 0.0) is not None for _ in range(3)]
        check('확인: 3프레임째 확정', got == [False, False, True], f'{got}')
        cf.reset()
        cf.update(cones, (line.x, line.y), 0.0)
        cf.update(cones, (line.x, line.y), 0.0)
        other = [_Obj(30, 115.2, 1.45, 0.0, 0.08, 0.08, 1.05),
                 _Obj(31, 115.35, -1.41, 0.0, 0.08, 0.08, 1.05)]
        check('확인: id가 바뀌면 다시 센다', cf.update(other, (line.x, line.y), 0.0) is None)

        # 12. 런타임 재조준 -- 정책 안에서 꼬리 교체·stop_s·이벤트·통과·정지
        rec = _Rec()
        pol = FinishPolicy(path, line, on_event=rec)
        old_len = path.length
        hist, cmd = _drive(path, pol, objects=cones)
        ks = rec.kinds()
        check('재조준: finish_line_from_cones 1회', ks.count('finish_line_from_cones') == 1,
              ','.join(ks))
        check('재조준: 종료선 = 중점 (115.3, 0.02)', pol.line.src == 'cones'
              and abs(pol.line.x - 115.275) < 0.05 and abs(pol.line.y - 0.02) < 0.05,
              f'({pol.line.x:.2f},{pol.line.y:.2f})')
        check('재조준: 꼬리가 길어졌다 (>= 이전)',
              path.length >= old_len and path.length >= 115.3 + 9.0 - 0.5, f'{path.length:.1f}')
        check('재조준: stop_s = 중점 +9', abs(pol.stop_s - (pol.line.s + 9.0)) < 0.5,
              f'{pol.stop_s:.2f} {pol.line.s:.2f}')
        check('재조준: 통과 뒤 +6~11m 정지', 'finish_line_passed' in ks and hist[-1][2].done
              and 6.0 <= hist[-1][0] - pol.line.s <= 11.0, f'{hist[-1][0] - pol.line.s:+.2f}')
        rt = [h for h in hist if h[2].retargeted]
        check('재조준: retargeted 플래그 1프레임', len(rt) == 1, f'{len(rt)}')
        check('재조준: 래치 (locked)', pol.locked)
        cmd = pol.update(path.nearest(110.0, 0.0), 110.0, 110.0, 0.0, 0.0, 5.0, [], 99.0)
        check('재조준: 객체가 사라져도 종료선 유지', pol.line.src == 'cones')
        path, line, _w = build(ev)
        rec = _Rec()
        pol = FinishPolicy(path, line, on_event=rec)
        cmd = None
        for k in range(3):
            cmd = pol.update(path.nearest(107.0, 0.0), 107.0, 117.0, 0.0, 0.0, 3.0, cones, k * 0.05)
        check('늦은 탐지: 확정 프레임에 passed', cmd.retargeted and cmd.passed, f'{cmd.action}')
        path_n, line_n, _w = build(ev)
        pol_n = FinishPolicy(path_n, line_n)
        pol_n.adopt(pol)
        check('adopt: 새 경로에 라바콘 종료선 이식', pol_n.locked and pol_n.line.src == 'cones'
              and abs(pol_n.line.s - pol.line.s) < 0.5 and path_n.length >= 124.0,
              f'{pol_n.line!r} {path_n.length:.1f}')

    # 13. 실제 경로
    data = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         '..', 'data'))
    for name in ('route_path_pt1.csv', 'route_path_pt2.csv', 'route_path_ex.csv'):
        f = os.path.join(data, name)
        if not os.path.exists(f):
            continue
        path, _b = load_path(f, _segments_of(f))
        line, tail, warns = load_for_path(path, SL.default_events_path(f))
        path.replace_tail(tail)
        line.bind(path)
        check(f'실경로 {name}: 종료선 = 경로 끝 정지선',
              line.src == 'stopline' and not line.ahead and abs(line.s - path.finish_s) < 1e-6,
              f'{line!r} finish_s={path.finish_s:.2f}')
        check(f'실경로 {name}: 종방향 오차 |lon| < 0.5m', abs(line.lon_off) < 0.5,
              f'{line.lon_off:+.2f}')
        check(f'실경로 {name}: 꼬리 9점, stop_s == length',
              len(path.pts) - path.n_real == 9
              and abs(FinishPolicy(path, line).stop_s - path.length) < 1e-9,
              f'{len(path.pts) - path.n_real}')
    f = os.path.join(data, 'route_path_lt.csv')
    if os.path.exists(f) and os.path.exists(SL.default_events_path(f)):
        path, _b = load_path(f, _segments_of(f))
        line, tail, warns = load_for_path(path, SL.default_events_path(f))
        path.replace_tail(tail)
        line.bind(path)
        check('실경로 lt: 끝 너머 정지선 lon +15.9',
              line.src == 'stopline' and line.ahead and abs(line.end_lon - 15.9) < 0.3,
              f'{line!r} end_lon={line.end_lon:+.2f}')
        pol = FinishPolicy(path, line)
        check('실경로 lt: stop_s ≈ 1777.1 + 15.9 + 9', abs(pol.stop_s - (1777.14 + 15.9 + 9.0)) < 0.5,
              f'{pol.stop_s:.1f}')
        cones = [_Obj(10, 1092.881, -43.621, 0.0, 0.08, 0.08, 1.05),
                 _Obj(11, 1094.749, -45.786, 0.0, 0.08, 0.08, 1.05)]
        rec = _Rec()
        pol = FinishPolicy(path, line, on_event=rec)
        i0 = path.finish_idx - 60
        cmd = None
        for k in range(3):
            cmd = pol.update(i0, path.s[i0], path.pts[i0][0], path.pts[i0][1], 0.0, 10.0,
                             cones, k * 0.05)
        check('실경로 lt: 실측 라바콘 쌍 확정', cmd.retargeted and pol.line.src == 'cones',
              ','.join(rec.kinds()))
        check('실경로 lt: 중점이 정지선과 1m 안', pol.line.src == 'cones'
              and math.hypot(pol.line.x - 1094.380, pol.line.y + 44.488) < 1.0,
              f'({pol.line.x:.2f},{pol.line.y:.2f})')
        logf = os.path.join(data, '..', 'logs', 'pp_20260910_181704', 'objects.csv')
        drv = os.path.join(data, '..', 'logs', 'pp_20260910_181704', 'drive.csv')
        if os.path.exists(logf) and os.path.exists(drv):
            by_n = {}
            with open(logf, newline='', encoding='utf-8') as fh:
                for r in csv.DictReader(fh):
                    by_n.setdefault(int(r['n']), []).append(_Obj(
                        int(r['id']), float(r['x']), float(r['y']), float(r['speed']),
                        float(r['length']), float(r['width']), float(r['height'])))
            path, _b = load_path(f, _segments_of(f))
            line, tail, warns = load_for_path(path, SL.default_events_path(f))
            path.replace_tail(tail)
            line.bind(path)
            rec = _Rec()
            pol = FinishPolicy(path, line, on_event=rec)
            hit_n = None
            with open(drv, newline='', encoding='utf-8') as fh:
                idx = None
                for r in csv.DictReader(fh):
                    n = int(r['n'])
                    if n < 4800:
                        continue
                    x, y = float(r['ego_x']), float(r['ego_y'])
                    idx = (path.nearest(x, y) if idx is None
                           else path.nearest(x, y, start=max(0, idx - 5)))
                    cmd = pol.update(idx, path.s[idx], x, y, float(r['heading']),
                                     float(r['speed']), by_n.get(n, []), float(r['t']))
                    if cmd.retargeted and hit_n is None:
                        hit_n = n
                    if n >= 5022:
                        break
            check('실차 로그 재생: 첫 관측(n=4846)+3프레임에 확정', hit_n == 4848, f'{hit_n}')
            check('실차 로그 재생: 종료선 미통과 (실제로 6m 못 갔다)', not pol.passed,
                  f'max_lon={pol.max_lon}')

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
    ap.add_argument('--search', type=float, default=FINISH_SEARCH_M)
    ap.add_argument('--fallback-overrun', type=float, default=FINISH_FALLBACK_OVERRUN_M)
    ap.add_argument('--cones', default='', metavar='x1,y1,x2,y2',
                    help='라바콘 두 좌표를 넣어 런타임 쌍 판정을 미리 본다')
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args()
    if args.selftest:
        raise SystemExit(_selftest())
    if args.plan and args.path:
        raise SystemExit(_plan(args))
    ap.error('--plan --path ... 또는 --selftest')


if __name__ == '__main__':
    main()
