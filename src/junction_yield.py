#!/usr/bin/env python3
"""교차로 진입 양보 게이트 (평가항목 14) ― 순수 로직.

9/12 본대회(pp_20260912_154040)에서 신호 없는 교차로 입구에 좌회전하려고 선 자리가
우측에서 좌회전해 들어오는 NPC 행렬 5대의 회전 궤적 안이었고 4대와 충돌했다.
ttc는 상대를 하나씩 보고 그때그때 서지만 **어디에 서야 하는가**는 모른다. 여기서는
교차로에 들어가기 **전에** 들어가도 되는지를 판단하고, 안 되면 입구 밖에 선다.

  · 판단은 경로가 교차로 내부(connecting road)로 들어가는 지점 s_j0 앞에서만 한다.
    정지점은 후륜축 s_stop = s_j0 - FRONT_OVERHANG - 1m. 이미 들어갔으면 관여하지
    않는다 (그 뒤는 ttc의 몫이다)
  · 대상은 (a) 비신호 교차로 전부, (b) 신호 교차로의 비보호 좌회전
    (기동 좌회전 + tl_state GREEN(3). 5 GREEN+LEFT / 4 LEFT는 보호 좌회전이라 제외)
  · 위협 = trusted 이동 차량의 등회전(CTRV) 예측이 시간창 안에 판정 영역에 드는 것.
    영역은 (a) 교차로 전체(같은 junction의 connecting road 점열 + 버퍼),
    (b) ego 통과 회랑(경로 s_j0~s_j1 ± (반폭 + 2m)). (b)를 좁힌 것은 항목 8
    (녹색 30m 5초 정차) 때문이다 ― 겹치지 않는 차량에 서면 이유 없는 정차가 된다
  · 시간창은 고정값이 아니라 **우리가 교차로를 빠져나가는 데 걸리는 시간**이다.
    T = max(4, t_clear + 1.5). 정지 출발로 30m 교차로를 지나는 데 7초가 넘는데
    4초만 보면 5초 간격 행렬의 틈으로 출발해 다음 차에 받힌다
  · 해제는 위협 0대가 CLEAR_S 연속. 래치는 "위협 집합이 빈 시간"에 건다 ―
    대상 id가 바뀌어도 끊기지 않는다
  · 비신호 교차로에는 접근 속도 상한(25km/h)을 건다. 위협이 늦게 보여도
    DECEL_HARD 안에 s_stop에 설 수 있게 하려는 것이다 (저속은 무감점)
  · 교착 상한 YIELD_MAX_S 뒤에는 대기 중 본 가장 긴 틈의 80%가 다시 오면 출발한다

stoplines.py와 같은 구조다. I/O 없이 매 프레임 속도 상한 하나를 내고
pure_pursuit이 min()으로 섞는다. yielding이면 stoplines(blocked)·detour 최후
안전망·ttc_timeout·crosswalks 탈출이 그동안 얼어 있는다.

    python junction_yield.py --build                      # data/xodr_junction_roads.csv
    python junction_yield.py --plan --path ../data/race.csv
    python junction_yield.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ctrv
import obstacles as OB
import stoplines as SL
from stoplines import FRONT_OVERHANG

GREEN = SL.GREEN

REAR_OVERHANG = 1.0         # m, 후륜축 -> 뒤범퍼 (check_collision과 같은 가정)
EGO_HALF_W = 0.95           # m
EGO_LEN = FRONT_OVERHANG + REAR_OVERHANG

STOP_MARGIN = 1.0           # m, 앞범퍼를 s_j0 이만큼 앞에 세운다
PAST_M = 1.5                # m, 후륜축이 s_stop을 이만큼 넘으면 이미 들어간 것이다
APPROACH_LOOK_M = 150.0     # m, 이 안의 교차로만 본다

CELL = 1.0                  # m, 영역 격자
AREA_BUF_M = 2.0            # m, connecting road 중심선 버퍼 (비신호)
CORR_BUF_M = EGO_HALF_W + 2.0   # m, ego 통과 회랑 반폭 (신호 비보호 좌회전)
NEAR_FALLBACK_M = 45.0      # m, junction id를 모를 때 같은 교차로로 볼 거리

MOVING_MS = 1.5             # m/s, 이보다 느린 차량은 위협으로 보지 않는다
OBJ_RANGE_M = 80.0          # m, 9910 객체 수신 한계
TAU_STEP = 0.2              # s
T_HORIZON_MIN = ctrv.T_HORIZON_MIN     # s (ttc의 정지 중 hold와 공용 ― ctrv.py)
T_HORIZON_MAX = ctrv.T_HORIZON_MAX     # s
T_MARGIN = ctrv.T_MARGIN               # s, 통과 시간에 얹는 여유
DEPART_ACCEL = ctrv.DEPART_ACCEL       # m/s^2, 출발 가속 가정

SAME_DIR_RAD = math.radians(45.0)
APPROACH_LAT_M = 8.0        # m, 같은 접근로로 볼 횡거리 (뒤차·옆 차로 동행차)
LEAD_LAT_M = 2.0            # m, 경로 위 앞차 (obstacles.VEH_LANE_M과 같다)
LEAVE_MARGIN_M = 0.5        # m, 빠져나가는 차: ego 회랑 = 반폭 + 객체 횡 반폭 + 이 값
LEAVE_APPROACH_M = 0.3      # m, 예측 끝의 횡거리가 지금보다 이만큼 넘게 줄면 다가오는 것

JY_APPROACH_KMH = 25.0      # km/h, 비신호 교차로 접근 속도 상한
JY_APPROACH_M = 40.0        # m, s_stop 앞 이 거리부터 상한 속도
DECEL_APPROACH = 2.0        # m/s^2, 상한까지 내려가는 프로파일
DECEL_HARD = SL.DECEL_HARD  # m/s^2, "설 수 있는가" 판정 전용

CLEAR_S = 2.0               # s, 위협 0대가 이만큼 이어져야 출발
YIELD_MAX_S = 45.0          # s, 교착 상한
GAP_MIN_S = 2.0             # s, 상한 뒤에도 이보다 짧은 틈에는 출발하지 않는다
GAP_FRAC = 0.8              # 상한 뒤 출발 기준 = 관측한 최대 틈 x 이 값

SIGNAL_NEAR_M = 15.0        # m, s_j0 앞 이 안에 신호 정지선이 있으면 신호 교차로
TURN_RAD = math.radians(30.0)

JROADS_CSV = 'xodr_junction_roads.csv'


# ----------------------------------------------------------------------
# 로드

def build_junction_roads(xodr_path: str, out_csv: str) -> int:
    """xodr의 <road id= junction=> 만 읽어 CSV로 낸다. 본문은 파싱하지 않는다."""
    pat = re.compile(r'<road\b([^>]*)>')
    rows = []
    with open(xodr_path, encoding='utf-8', errors='replace') as fh:
        for line in fh:
            if '<road' not in line:
                continue
            for m in pat.finditer(line):
                attrs = dict(re.findall(r'(\w+)="([^"]*)"', m.group(1)))
                jid = attrs.get('junction', '-1')
                if 'id' in attrs and jid not in ('-1', ''):
                    rows.append((attrs['id'], jid))
    with open(out_csv, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['road_id', 'junction_id'])
        w.writerows(rows)
    return len(rows)


def load_junction_roads(path: str) -> dict:
    """road_id -> junction_id. 파일이 없으면 빈 dict."""
    if not path or not os.path.exists(path):
        return {}
    with open(path, newline='', encoding='utf-8') as fh:
        return {r['road_id']: r['junction_id'] for r in csv.DictReader(fh)}


def fallback_junction_roads(edges_csv: str) -> dict:
    """junction id를 모를 때. 엣지 CSV의 kind=junction 도착 도로 -> '?'.

    같은 교차로인지는 거리(NEAR_FALLBACK_M)로만 가른다.
    """
    out = {}
    if not edges_csv or not os.path.exists(edges_csv):
        return out
    with open(edges_csv, newline='', encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            if r.get('kind') == 'junction':
                out[r['to_road']] = '?'
    return out


def load_area_points(lanes_csv: str, jroads: dict) -> dict:
    """xodr_lanes.csv에서 교차로 내부 차로 점만 뽑는다. junction_id -> [(x, y)]."""
    out: dict = {}
    if not jroads or not lanes_csv or not os.path.exists(lanes_csv):
        return out
    with open(lanes_csv, newline='', encoding='utf-8-sig') as fh:
        for r in csv.DictReader(fh):
            jid = jroads.get(r['road_id'])
            if jid is not None:
                out.setdefault(jid, []).append((float(r['x']), float(r['y'])))
    return out


class _Row:
    __slots__ = ('road_id',)

    def __init__(self, road_id):
        self.road_id = road_id


def load_rows(waypoints_csv: str, segments: list) -> list:
    """pure_pursuit.load_path와 같은 순서로 road_id만 읽는다. 컬럼이 없으면 []."""
    by_seg: dict = {}
    with open(waypoints_csv, newline='', encoding='utf-8') as fh:
        rdr = csv.DictReader(fh)
        if 'road_id' not in (rdr.fieldnames or []):
            return []
        for r in rdr:
            by_seg.setdefault(r['segment_id'], []).append(_Row(r['road_id']))
    out = []
    for name in segments:
        out.extend(by_seg.get(name, ()))
    return out


def _mark(cells: set, x: float, y: float, r: float) -> None:
    n = int(math.ceil(r / CELL))
    cx, cy = math.floor(x / CELL), math.floor(y / CELL)
    lim = r + 0.71 * CELL
    for dx in range(-n, n + 1):
        for dy in range(-n, n + 1):
            if math.hypot(dx, dy) * CELL <= lim:
                cells.add((cx + dx, cy + dy))


def _in(cells, x: float, y: float) -> bool:
    return (math.floor(x / CELL), math.floor(y / CELL)) in cells


class JZone:
    """경로가 지나는 교차로 하나."""

    __slots__ = ('i0', 'i1', 's_j0', 's_j1', 'jid', 'maneuver', 'signalized',
                 'area', 'corr', 'cx', 'cy', 'radius', 'x0', 'y0', 'n_area_pts',
                 's_line')

    @property
    def cells(self):
        return self.corr if self.signalized else self.area

    @property
    def kind(self) -> str:
        return 'corridor' if self.signalized else 'area'

    def __repr__(self) -> str:
        return (f'<JZone j={self.jid} s={self.s_j0:.1f}~{self.s_j1:.1f} '
                f'{self.maneuver} {"신호" if self.signalized else "비신호"}>')


def make_zone(path, i0: int, i1: int, jid: str, signalized: bool,
              area_xy, maneuver: str = '', s_line=None) -> JZone:
    z = JZone()
    z.s_line = s_line           # 신호 정지선의 경로 s (없으면 None)
    z.i0, z.i1 = i0, i1
    z.s_j0, z.s_j1 = path.s[i0], path.s[i1]
    z.jid = jid
    z.signalized = signalized
    z.x0, z.y0 = path.pts[i0]
    if not maneuver:
        ia = max(0, i0 - 3)
        d = ctrv.wrap(SL._path_heading(path, i1) - SL._path_heading(path, ia))
        maneuver = 'lt' if d > TURN_RAD else ('rt' if d < -TURN_RAD else 'st')
    z.maneuver = maneuver
    # ego 통과 회랑. 경로 점이 1m 간격이라 사이를 0.5m로 메운다
    z.corr = set()
    for i in range(i0, i1 + 1):
        x, y = path.pts[i]
        _mark(z.corr, x, y, CORR_BUF_M)
        if i < i1:
            bx, by = path.pts[i + 1]
            _mark(z.corr, 0.5 * (x + bx), 0.5 * (y + by), CORR_BUF_M)
    ego_xy = [path.pts[i] for i in range(i0, i1 + 1)]
    pts = list(area_xy) or ego_xy
    z.area = set()
    for x, y in pts:
        _mark(z.area, x, y, AREA_BUF_M)
    for x, y in ego_xy:             # 제 연결로는 항상 영역 안이다
        _mark(z.area, x, y, AREA_BUF_M)
    z.n_area_pts = len(pts)
    z.cx = sum(p[0] for p in pts) / len(pts)
    z.cy = sum(p[1] for p in pts) / len(pts)
    z.radius = max(math.hypot(p[0] - z.cx, p[1] - z.cy) for p in pts) + AREA_BUF_M
    return z


def build_zones(path, rows: list, jroads: dict, area_pts: dict,
                stoplines=()) -> list:
    """경로 위 교차로 구간 목록 (s 오름차순)."""
    n = min(len(rows), getattr(path, 'n_real', len(path.pts)))
    out = []
    i = 0
    while i < n:
        jid = jroads.get(rows[i].road_id)
        if jid is None:
            i += 1
            continue
        j = i
        while j + 1 < n and jroads.get(rows[j + 1].road_id) == jid:
            j += 1
        if i > 0:                   # 교차로 안에서 출발하면 판단할 접근이 없다
            pts = area_pts.get(jid, ())
            if jid == '?':
                mx, my = path.pts[(i + j) // 2]
                pts = [p for p in pts
                       if math.hypot(p[0] - mx, p[1] - my) <= NEAR_FALLBACK_M]
            s0 = path.s[i]
            lines = [sl.s for sl in stoplines
                     if sl.signalized and -3.0 <= s0 - sl.s <= SIGNAL_NEAR_M]
            out.append(make_zone(path, i, j, jid, bool(lines), pts,
                                 s_line=min(lines) if lines else None))
        i = j + 1
    return out


def load_for_path(path, waypoints_csv: str, segments: list, jroads: dict,
                  area_pts: dict, stoplines=()) -> tuple:
    """(교차로 목록, 경고 목록)."""
    warn = []
    rows = load_rows(waypoints_csv, segments)
    if not rows:
        return [], ['경로 CSV에 road_id 컬럼이 없다 ― 교차로 양보를 쓸 수 없다']
    n_real = getattr(path, 'n_real', len(path.pts))
    if len(rows) != n_real:
        return [], [f'행 수 불일치 ({len(rows)}행 vs 경로 {n_real}점) ― '
                    '교차로 양보를 쓸 수 없다']
    if not jroads:
        return [], ['교차로 소속 정보가 없다 (xodr_junction_roads.csv) ― '
                    '교차로 양보를 쓸 수 없다']
    return build_zones(path, rows, jroads, area_pts, stoplines), warn


# ----------------------------------------------------------------------
# 정책

class YieldCmd:
    __slots__ = ('v_limit', 'accel_limit', 'action', 'yielding', 'dist', 'n',
                 'stop_avail', 'horizon', 'reason')

    def __init__(self, action='none'):
        self.v_limit = float('inf')
        self.accel_limit = float('inf')
        self.action = action        # none / approach / yield / hold / commit
        self.yielding = False       # 양보 정지 중 (다른 정책의 교착 타이머를 얼린다)
        self.dist = float('nan')    # 후륜축 -> 정지점 [m]
        self.n = 0                  # 위협 차량 수
        self.stop_avail = float('nan')
        self.horizon = float('nan')
        self.reason = ''

    def __repr__(self) -> str:
        return f'<YieldCmd {self.action} v<={self.v_limit:.1f} d={self.dist:.1f} n={self.n}>'


class _Tr:
    __slots__ = ('id', 'x', 'y', 'n_seen', 'missing', 'trusted', 'hist')

    def __init__(self, oid, x, y, heading):
        self.id = oid
        self.x, self.y = x, y
        self.n_seen = 1
        self.missing = 0
        self.trusted = False
        self.hist = ctrv.HeadingHist()
        self.hist.push(heading)


t_clear = ctrv.t_clear


class JunctionYieldPolicy:
    """매 프레임 속도 상한 하나를 낸다."""

    def __init__(self, path, zones, *, front_overhang: float = FRONT_OVERHANG,
                 max_hold: float = YIELD_MAX_S,
                 approach_kmh: float = JY_APPROACH_KMH, on_event=None):
        self.path = path
        self.zones = sorted(zones, key=lambda z: z.s_j0)
        self.front = front_overhang
        self.max_hold = max_hold
        self.v_app = approach_kmh / 3.6
        self._on_event = on_event or (lambda kind, detail: None)
        self.reset()

    # -- 상태 -----------------------------------------------------------
    def reset(self) -> None:
        self._tracks: dict = {}
        self._yz = None             # 양보 중인 교차로
        self._yield_since = None
        self._clear_since = None
        self._best_gap = 0.0        # 대기 중 본 가장 긴 틈 [s]
        self._timed_out = False
        self._forced = set()        # 상한으로 출발한 교차로 (다시 잡지 않는다)
        self._announced = set()
        self._wait_total = 0.0
        self._left = set()          # 이번 양보에서 "빠져나가는 중"으로 뺀 차량 id

    def adopt(self, old: 'JunctionYieldPolicy') -> None:
        """우회로 경로가 바뀌었다. 트랙과 양보 래치를 넘겨받는다."""
        if old is None:
            return
        self._tracks = old._tracks
        self._wait_total = old._wait_total

        def match(z):
            for nz in self.zones:
                if nz.jid == z.jid and math.hypot(nz.x0 - z.x0, nz.y0 - z.y0) < 5.0:
                    return nz
            return None

        if old._yz is not None:
            nz = match(old._yz)
            if nz is not None:
                self._yz = nz
                self._yield_since = old._yield_since
                self._clear_since = old._clear_since
                self._best_gap = old._best_gap
                self._timed_out = old._timed_out
        for z in old.zones:
            if id(z) in old._forced:
                nz = match(z)
                if nz is not None:
                    self._forced.add(id(nz))
            for key in old._announced:
                if key[1] == id(z):
                    nz = match(z)
                    if nz is not None:
                        self._announced.add((key[0], id(nz)))

    def _announce(self, key, kind: str, detail: str) -> None:
        if key in self._announced:
            return
        self._announced.add(key)
        self._on_event(kind, detail)

    # -- 추적 -----------------------------------------------------------
    def _update_tracks(self, objects) -> None:
        seen = set()
        for o in objects:
            seen.add(o.id)
            tr = self._tracks.get(o.id)
            if tr is not None and math.hypot(o.x - tr.x, o.y - tr.y) > OB.ID_JUMP_M:
                tr = None
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

    # -- 대상 -----------------------------------------------------------
    def _s_stop(self, z: JZone) -> float:
        """후륜축 정지점. 신호 교차로에서는 stoplines의 정지점보다 앞서지 않는다 ―
        정지선이 s_j0보다 조금 앞(0.15m)이면 stoplines의 여유가 음수가 되어, 양보하다
        신호가 바뀌었을 때 두 정책이 서로 다른 자리를 겨냥한다."""
        s = z.s_j0 - self.front - STOP_MARGIN
        if z.s_line is not None:
            s = min(s, z.s_line - self.front - SL.STOP_MARGIN)
        return s

    def _zone(self, s_ego: float):
        for z in self.zones:
            s_stop = self._s_stop(z)
            if s_ego > s_stop + PAST_M:
                continue
            if s_stop - s_ego > APPROACH_LOOK_M:
                return None
            return z
        return None

    def _threats(self, z: JZone, idx: int, objects, horizon: float) -> list:
        """[(id, eta, 회랑 겹침, 교차로 중심 거리, 속도)]. eta 0은 지금 영역 안."""
        path = self.path
        out = []
        win = None
        n = int(round(horizon / TAU_STEP))
        cells = z.cells
        for o in objects:
            if abs(o.speed) <= MOVING_MS:
                continue
            tr = self._tracks.get(o.id)
            if tr is None or not tr.trusted:
                continue
            if OB.classify(o) != OB.VEHICLE:
                continue
            dc = math.hypot(o.x - z.cx, o.y - z.cy)
            if dc > min(OBJ_RANGE_M, abs(o.speed) * horizon + z.radius):
                continue
            w = tr.hist.rate(o.speed)
            eta = None
            in_corr = False
            for k in range(n + 1):
                px, py, _h = ctrv.point(o.x, o.y, o.heading, o.speed, w,
                                        k * TAU_STEP)
                if eta is None and _in(cells, px, py):
                    eta = k * TAU_STEP
                if not in_corr and _in(z.corr, px, py):
                    in_corr = True
                if eta is not None and in_corr:
                    break
            if eta is None:
                continue
            # 제외 판정은 위협으로 잡힌 것에만 한다 (투영이 비싸다)
            if win is None:
                win = OB._window(path, idx, 40.0,
                                 max(0.0, z.s_j1 - path.s[idx]) + 30.0)
            s_o, lat_o, th_p = self._project(win, o.x, o.y)
            if abs(ctrv.wrap(o.heading - th_p)) < SAME_DIR_RAD:
                if abs(lat_o) < LEAD_LAT_M:
                    continue            # 경로 위 앞차·뒤차 ― obstacles의 몫
                if s_o < z.s_j0 and abs(lat_o) < APPROACH_LAT_M:
                    continue            # 같은 접근로의 옆 차로 동행차
            if eta == 0.0 and self._leaving(z, win, idx, o, w, n, lat_o):
                self._left.add(o.id)
                continue                # 경로를 이미 지나 빠져나가는 중
            out.append((o.id, eta, in_corr, dc, abs(o.speed)))
        return out

    def _leaving(self, z: JZone, win: tuple, idx: int, o, w: float, n: int,
                 lat_now: float) -> bool:
        """영역 안의 차량이 ego 통과 회랑에서 멀어지는 중인가.

        영역은 교차로 전체라 경로를 가로지른 차가 출구로 빠져나가는 동안에도
        (9/12 재현에서 2.3s) 위협으로 남는다. 예측 궤적이 앞으로 ego가 쓸고 갈
        자리(뒤범퍼 ~ 교차로 출구 + 차체)에 들지 않고, 경로를 가로지르지 않고,
        다가오지도 않으면 위협이 아니다. 셀이 아니라 경로 투영 횡거리로 잰다 ―
        빠져나가는 차는 ego 바로 옆 대향 차로(횡 2.7~2.9m)를 지나간다.
        """
        s_lo = self.path.s[idx] - REAR_OVERHANG
        s_hi = z.s_j1 + EGO_LEN
        hl, hw = 0.5 * o.length, 0.5 * o.width
        sign = 0.0
        lat_k = lat_now
        for k in range(n + 1):
            px, py, h = ctrv.point(o.x, o.y, o.heading, o.speed, w, k * TAU_STEP)
            s_k, lat_k, th_k = self._project(win, px, py)
            if not s_lo <= s_k <= s_hi:
                continue
            dh = h - th_k
            need = (EGO_HALF_W + hl * abs(math.sin(dh)) + hw * abs(math.cos(dh))
                    + LEAVE_MARGIN_M)
            if abs(lat_k) < need:
                return False
            sg = 1.0 if lat_k > 0.0 else -1.0
            if sign and sg != sign:
                return False
            sign = sg
        return abs(lat_k) >= abs(lat_now) - LEAVE_APPROACH_M

    def _project(self, win: tuple, x: float, y: float) -> tuple:
        """5점 간격으로 최근접을 찾고 그 둘레만 선분 투영한다."""
        pts = self.path.pts
        lo, hi = win
        best, bi = None, lo
        for i in range(lo, hi + 1, 5):
            d2 = (pts[i][0] - x) ** 2 + (pts[i][1] - y) ** 2
            if best is None or d2 < best:
                best, bi = d2, i
        return OB.project_fix(self.path, max(lo, bi - 6),
                              min(hi, bi + 6, len(pts) - 1), x, y)

    # -- 본체 -----------------------------------------------------------
    def update(self, idx: int, s_ego: float, v: float, ego_x: float,
               ego_y: float, ego_heading: float, objects, tl_state: int,
               t: float) -> YieldCmd:
        self._update_tracks(objects)
        cmd = YieldCmd()
        z = self._zone(s_ego)
        if self._yz is not None and z is not self._yz:
            self._end(t, '정지점을 지났다')
        if z is None:
            return cmd
        s_stop = self._s_stop(z)
        d = s_stop - s_ego
        cmd.dist = d

        # 비신호 교차로 접근 상한. 위협이 없어도 건다
        if not z.signalized:
            v_app = math.sqrt(self.v_app * self.v_app + 2.0 * DECEL_APPROACH
                              * max(0.0, d - JY_APPROACH_M))
            cmd.v_limit = v_app
            if v > v_app:
                cmd.accel_limit = -(v * v - self.v_app * self.v_app) \
                    / (2.0 * max(d - JY_APPROACH_M, 0.5))
            if d <= JY_APPROACH_M or v > v_app - 0.5:
                cmd.action = 'approach'
                self._announce(('approach', id(z)), 'junction_approach',
                               f'jid={z.jid} 비신호 교차로 {d + self.front + STOP_MARGIN:.0f}m 앞 ― '
                               f'{self.v_app * 3.6:.0f}km/h로 접근 '
                               f'(기동 {z.maneuver}, 정지점 s={s_stop:.1f})')

        # 대상 여부는 **새 양보를 시작할 때만** 본다. 양보 중에 신호가 GREEN(3)이
        # 아니게 바뀌어도 위협이 남아 있으면 계속 선다 ― 거기서 빠지면 정지선을
        # 넘은 것으로 읽힌 stoplines가 놓아 버릴 수 있고, 교차 차량은 그대로 있다.
        # 적색 정지는 stoplines가 같이 내고 min()이 더 보수적인 쪽을 고른다
        applicable = (not z.signalized) or (z.maneuver == 'lt' and tl_state == GREEN)
        if id(z) in self._forced:
            applicable = False
            if self._yz is z:
                self._end(t, '상한으로 출발한 교차로')
        self._tl = tl_state
        if not applicable and self._yz is not z:
            return cmd

        pass_len = (z.s_j1 + EGO_LEN) - s_stop
        tc = t_clear(v, pass_len, min(self.v_app, max(v, self.v_app)))
        horizon = ctrv.horizon(tc)
        cmd.horizon = horizon
        th = self._threats(z, idx, objects, horizon)
        cmd.n = len(th)
        yielding = self._yz is z

        if th and not yielding:
            need = v * v / (2.0 * DECEL_HARD)
            if need > max(d, 0.0) + 0.5:
                cmd.action = 'commit'
                cmd.reason = f'못 선다 (필요 {need:.1f}m, 남은 {d:.1f}m)'
                self._announce(('commit', id(z)), 'junction_commit',
                               f'jid={z.jid} 위협 {len(th)}대 [{self._fmt(th)}] 인데 '
                               f'정지점까지 {d:.1f}m, {v * 3.6:.1f}km/h에서 '
                               f'{DECEL_HARD:.1f}m/s²로 {need:.1f}m 필요 ― 양보하지 않는다')
                return cmd
            self._yz = z
            self._yield_since = t
            self._clear_since = None
            self._best_gap = 0.0
            self._timed_out = False
            self._left = set()
            yielding = True
            bx = ego_x + self.front * math.cos(ego_heading)
            by = ego_y + self.front * math.sin(ego_heading)
            self._on_event(
                'junction_yield',
                f'jid={z.jid} {"신호 GREEN 좌회전" if z.signalized else "비신호"} '
                f'{z.kind} 위협 {len(th)}대 [{self._fmt(th)}] T={horizon:.1f}s '
                f'정지점 s={s_stop:.1f} ({d:.1f}m 앞) '
                f'앞범퍼 ({bx:.1f},{by:.1f}) v={v * 3.6:.1f}km/h')
        if not yielding:
            return cmd

        if th:
            self._clear_since = None
            if all(e[1] > 0.0 for e in th):
                gap = min(e[1] for e in th)
                if gap > self._best_gap:
                    self._best_gap = gap
        else:
            if self._clear_since is None:
                self._clear_since = t
            if t - self._clear_since >= CLEAR_S:
                self._end(t, f'위협 0대 {CLEAR_S:.0f}s 연속')
                return self._after(cmd)

        if not self._timed_out and t - self._yield_since > self.max_hold:
            self._timed_out = True
            self._on_event(
                'junction_timeout',
                f'jid={z.jid} 양보 {self.max_hold:.0f}s 초과 ― 행렬이 끝나지 않는다. '
                f'관측한 최대 틈 {self._best_gap:.1f}s의 {GAP_FRAC:.0%}'
                f'(최소 {GAP_MIN_S:.0f}s) 이상 비면 출발한다')
        if self._timed_out and th:
            need_gap = max(GAP_MIN_S, GAP_FRAC * self._best_gap)
            if all(e[1] > 0.0 for e in th) and min(e[1] for e in th) >= need_gap:
                self._forced.add(id(z))
                self._end(t, f'상한 뒤 틈 {min(e[1] for e in th):.1f}s '
                             f'(기준 {need_gap:.1f}s)')
                return self._after(cmd)

        avail = max(0.0, d)
        v_lim, a_lim = SL._profile(avail, v)
        cmd.v_limit = min(cmd.v_limit, v_lim)
        cmd.accel_limit = min(cmd.accel_limit, a_lim)
        if v_lim <= 0.0:
            cmd.accel_limit = min(cmd.accel_limit, -1.5)
        cmd.yielding = True
        cmd.stop_avail = avail
        cmd.action = 'hold' if v < SL.STOP_SPEED else 'yield'
        cmd.reason = (f'위협 {len(th)}대 [{self._fmt(th)}]' if th
                      else f'해제 대기 {t - self._clear_since:.1f}s')
        return cmd

    def _after(self, cmd: YieldCmd) -> YieldCmd:
        """해제한 프레임. 접근 상한만 남긴다."""
        cmd.n = 0
        return cmd

    def _end(self, t: float, why: str) -> None:
        z = self._yz
        if z is None:
            return
        waited = t - self._yield_since if self._yield_since is not None else 0.0
        self._wait_total += waited
        tl = getattr(self, '_tl', 0)
        self._on_event('junction_go',
                       f'jid={z.jid} {why} ― 양보 {waited:.1f}s 뒤 출발'
                       + (f' (빠져나가는 차량 제외 {len(self._left)}대: '
                          f'{",".join(str(i) for i in sorted(self._left))})'
                          if self._left else '')
                       + (f' [신호 {SL.STATE_NAME.get(tl, tl)}]'
                          if z.signalized else ''))
        self._yz = None
        self._yield_since = self._clear_since = None
        self._timed_out = False

    @staticmethod
    def _fmt(th: list) -> str:
        return ', '.join(f'id={e[0]} eta={e[1]:.1f}s 회랑={"Y" if e[2] else "N"} '
                         f'거리={e[3]:.0f}m {e[4] * 3.6:.0f}km/h'
                         for e in sorted(th, key=lambda e: e[1])[:6])


def summary(policy: JunctionYieldPolicy) -> list:
    out = [f'  시간창 max({T_HORIZON_MIN:.0f}, 통과시간 + {T_MARGIN:.1f})s '
           f'(상한 {T_HORIZON_MAX:.0f}s), 해제 {CLEAR_S:.0f}s, 교착 상한 '
           f'{policy.max_hold:.0f}s, 비신호 접근 {policy.v_app * 3.6:.0f}km/h '
           f'({JY_APPROACH_M:.0f}m 앞부터)']
    for z in policy.zones:
        out.append(
            f'  s={z.s_j0:7.1f}~{z.s_j1:7.1f}  정지점 {policy._s_stop(z):7.1f}  '
            + (f'정지선 {z.s_line:7.1f}  ' if z.s_line is not None else '')
            +
            f'j={z.jid:>4s}  {z.maneuver}  '
            f'{"신호(GREEN 좌회전만, 회랑)" if z.signalized else "비신호(전체 영역)"}  '
            f'영역 점 {z.n_area_pts} 반경 {z.radius:.0f}m')
    return out


def zones_meta(policy: JunctionYieldPolicy) -> list:
    return [{'s_j0': round(z.s_j0, 1), 's_j1': round(z.s_j1, 1),
             's_stop': round(policy._s_stop(z), 1), 'jid': z.jid,
             'maneuver': z.maneuver, 'signalized': z.signalized,
             'radius_m': round(z.radius, 1)} for z in policy.zones]


# ----------------------------------------------------------------------
# 자기검사

class _Path(OB._Straight):
    def __init__(self, length=400.0):
        super().__init__(length)
        self.n_real = len(self.pts)


def _cross_area(x0=115.0, half=15.0):
    """x=x0에서 y 방향으로 가로지르는 연결로 + ego 연결로(100~130)."""
    pts = [(x0, -half + k) for k in range(int(2 * half) + 1)]
    pts += [(100.0 + k, 0.0) for k in range(31)]
    return pts


def _sim(policy, path, objs_at, *, v0=12.5, x0=0.0, n=600, tl=0, v_init=None):
    """종방향 적분. (기록, 이벤트용 없음). 기록: (t, x, v, action, n, v_limit, yielding)."""
    x = x0
    v = v0 if v_init is None else v_init
    idx = 0
    dt = 0.05
    log = []
    for k in range(n):
        t = k * dt
        while idx < len(path.pts) - 1 and path.s[idx + 1] <= x:
            idx += 1
        tls = tl(t) if callable(tl) else tl
        cmd = policy.update(idx, x, v, x, 0.0, 0.0, objs_at(t, x), tls, t)
        v_cmd = min(v0, cmd.v_limit)
        a = max(-6.0, min(3.0, min(1.2 * (v_cmd - v), cmd.accel_limit)))
        if v_cmd <= 0.0:
            a = min(a, -1.5)
        x += v * dt
        v = max(0.0, v + a * dt)
        log.append((t, x, v, cmd.action, cmd.n, cmd.v_limit, cmd.yielding))
    return log


def _crosser(oid, t0, *, x=115.0, y0=-60.0, v=8.33, hdg=math.pi / 2):
    """t0에 (x, y0)에서 나타나 hdg 방향으로 v로 달린다. 영역을 40m 지나면 사라진다."""
    def at(t):
        if t < t0:
            return None
        d = v * (t - t0)
        px, py = x + d * math.cos(hdg), y0 + d * math.sin(hdg)
        if math.hypot(px - 115.0, py) > 75.0:
            return None
        return OB._Fake(oid, px, py, speed=v, heading=hdg)
    return at


def _selftest() -> int:
    fails = []

    def check(name, ok, detail=''):
        print(f'  {"PASS" if ok else "FAIL"}  {name}'
              + (f'  ― {detail}' if detail else ''))
        if not ok:
            fails.append(name)

    def setup(signalized=False, maneuver='st', **kw):
        path = _Path()
        z = make_zone(path, 100, 130, '1', signalized, _cross_area(),
                      maneuver=maneuver)
        ev = []
        pol = JunctionYieldPolicy(path, [z], on_event=lambda k, d: ev.append((k, d)),
                                  **kw)
        return path, z, pol, ev

    def kinds(ev):
        return [k for k, _d in ev]

    def objs(*fns):
        def at(t, x):
            return [o for o in (f(t) for f in fns) if o is not None]
        return at

    s_stop = 100.0 - FRONT_OVERHANG - STOP_MARGIN

    # 1. 비신호 교차 차량 -> 입구 앞 정지
    path, z, pol, ev = setup()
    log = _sim(pol, path, objs(_crosser(1, 3.0, y0=-70.0, v=5.0)), n=800)
    k = kinds(ev)
    check('비신호 교차 차량: junction_yield', 'junction_yield' in k, ','.join(k))
    held = [r for r in log if r[6]]
    xmax = max((r[1] for r in held), default=0.0)
    check('비신호 교차 차량: 정지점을 넘지 않는다', held and xmax <= s_stop + 0.6,
          f'양보 중 최대 s={xmax:.2f} (정지점 {s_stop:.2f})')
    check('비신호 교차 차량: 앞범퍼가 교차 궤적 밖',
          xmax + FRONT_OVERHANG < 115.0 - AREA_BUF_M - 0.9,
          f'앞범퍼 x={xmax + FRONT_OVERHANG:.2f}')
    check('비신호 교차 차량: 지나간 뒤 출발', 'junction_go' in k and log[-1][1] > 135.0,
          f'끝 s={log[-1][1]:.1f}')
    check('비신호 교차 차량: commit 없음', 'junction_commit' not in k)

    # 2. 행렬 5대 (3s 간격) -> 끊기지 않고, 마지막 차 뒤에 출발
    path, z, pol, ev = setup()
    fns = [_crosser(10 + i, 2.0 + 3.0 * i, y0=-60.0) for i in range(5)]
    log = _sim(pol, path, objs(*fns), n=900)
    k = kinds(ev)
    check('행렬 5대: yield 1회·go 1회',
          k.count('junction_yield') == 1 and k.count('junction_go') == 1, ','.join(k))
    t_go = next((r[0] for i, r in enumerate(log)
                 if i and log[i - 1][6] and not r[6]), None)
    # 마지막 차(t0=14)가 경로를 가로질러 ego 회랑을 벗어나는 시각. 직각으로 지나므로
    # 횡 반폭은 차 길이의 절반이다. 영역을 벗어나는 것(y=+17)은 그보다 1.6s 뒤다
    t_last = 14.0 + (60.0 + EGO_HALF_W + 0.5 * 4.39 + LEAVE_MARGIN_M) / 8.33
    t_area = 14.0 + (60.0 + 15.0 + AREA_BUF_M) / 8.33
    check('행렬 5대: 마지막 차가 회랑을 벗어난 뒤 2s에 출발 (영역 이탈을 기다리지 않는다)',
          t_go is not None and t_last + CLEAR_S - 0.3 <= t_go <= t_last + CLEAR_S + 1.0
          and t_go < t_area + CLEAR_S - 1.0,
          f'출발 t={t_go} (회랑 이탈 {t_last:.1f}s, 영역 이탈 {t_area:.1f}s)')
    go = [d for kk, d in ev if kk == 'junction_go']
    check('행렬 5대: junction_go에 제외한 차량 기록',
          bool(go) and '빠져나가는 차량 제외' in go[0], go[0] if go else '')
    check('행렬 5대: 충돌 없이 통과', log[-1][1] > 135.0, f'끝 s={log[-1][1]:.1f}')

    # 3. 교차 차량 없음 -> 무발화
    path, z, pol, ev = setup()
    log = _sim(pol, path, objs(), n=500)
    k = kinds(ev)
    check('교차 차량 없음: yield/commit 없음',
          'junction_yield' not in k and 'junction_commit' not in k, ','.join(k))
    check('교차 차량 없음: 통과', log[-1][1] > 135.0, f'끝 s={log[-1][1]:.1f}')

    # 4. 제외: 뒤차 / 옆 차로 동행차 / 경로 위 앞차
    def along(oid, x0, lat, v):
        return lambda t: OB._Fake(oid, x0 + v * t, lat, speed=v, heading=0.0)
    for name, fn in (('뒤차', along(1, -15.0, 0.0, 6.9)),
                     ('옆 차로 동행차', along(2, 5.0, -3.3, 6.9)),
                     ('경로 위 앞차', along(3, 25.0, 0.0, 6.9))):
        path, z, pol, ev = setup()
        _sim(pol, path, objs(fn), n=400)
        k = kinds(ev)
        check(f'제외 ― {name}: yield 없음', 'junction_yield' not in k, ','.join(k))

    # 5. 신호 교차로
    onc = lambda oid, t0: _crosser(oid, t0, x=180.0, y0=1.5, v=8.33, hdg=math.pi)
    path, z, pol, ev = setup(signalized=True, maneuver='lt')
    _sim(pol, path, objs(onc(1, 3.0)), n=400, tl=SL.GREEN)
    check('신호 GREEN 좌회전: 회랑을 지나는 대향 직진에 발동',
          'junction_yield' in kinds(ev), ','.join(kinds(ev)))
    path, z, pol, ev = setup(signalized=True, maneuver='lt')
    _sim(pol, path, objs(onc(1, 3.0)), n=400, tl=SL.GREEN_LEFT)
    check('신호 GREEN+LEFT(보호 좌회전): 무발화', not kinds(ev), ','.join(kinds(ev)))
    path, z, pol, ev = setup(signalized=True, maneuver='st')
    _sim(pol, path, objs(onc(1, 3.0)), n=400, tl=SL.GREEN)
    check('신호 GREEN 직진: 무발화', not kinds(ev), ','.join(kinds(ev)))
    # 영역 안·회랑 밖: y=-12에서 x 방향으로 가로지르는 차 (우회전해 멀리 도는 차)
    far = lambda oid, t0: _crosser(oid, t0, x=60.0, y0=-12.0, v=8.33, hdg=0.0)
    az = _cross_area() + [(100.0 + i, -12.0) for i in range(31)]
    path = _Path()
    ev = []
    pol = JunctionYieldPolicy(path, [make_zone(path, 100, 130, '1', True, az, 'lt')],
                              on_event=lambda k, d: ev.append((k, d)))
    _sim(pol, path, objs(far(1, 3.0)), n=400, tl=SL.GREEN)
    check('신호 GREEN 좌회전: 회랑 밖 차량에 무발화', not kinds(ev), ','.join(kinds(ev)))
    ev = []
    pol = JunctionYieldPolicy(path, [make_zone(path, 100, 130, '1', False, az)],
                              on_event=lambda k, d: ev.append((k, d)))
    _sim(pol, path, objs(far(1, 3.0)), n=400)
    yd = [d for k, d in ev if k == 'junction_yield']
    check('같은 차량이 비신호에서는 발동 (회랑=N 기록)',
          bool(yd) and '회랑=N' in yd[0], yd[0][:80] if yd else ','.join(kinds(ev)))

    # 6. 못 서는 거리 -> commit (신호 교차로에는 접근 상한이 없다)
    path, z, pol, ev = setup(signalized=True, maneuver='lt')
    late = lambda t: (OB._Fake(1, 115.0, -20.0 + 8.33 * (t - 6.9), speed=8.33,
                               heading=math.pi / 2) if 6.9 <= t < 12.0 else None)
    log = _sim(pol, path, objs(late), n=400, tl=SL.GREEN)
    k = kinds(ev)
    check('못 서는 거리: commit, yield 없음',
          'junction_commit' in k and 'junction_yield' not in k, ','.join(k))
    check('신호 교차로: 접근 상한 없음',
          all(r[3] != 'approach' for r in log) and max(r[2] for r in log) > 12.0)

    # 7. 끝나지 않는 행렬 (9s 간격 ― 틈 4.9s가 시간창보다 짧아 해제가 안 된다)
    #    -> 상한 뒤 틈 출발
    path, z, pol, ev = setup(max_hold=20.0)
    fns = [_crosser(100 + i, 2.0 + 9.0 * i, y0=-60.0) for i in range(30)]
    log = _sim(pol, path, objs(*fns), n=1400)
    k = kinds(ev)
    check('끝나지 않는 행렬: junction_timeout 뒤 출발',
          'junction_timeout' in k and k.index('junction_timeout') < len(k) - 1
          and k[-1] == 'junction_go' and log[-1][1] > 110.0,
          f'{",".join(k)} 끝 s={log[-1][1]:.1f}')
    t_to = next((i for i, kk in enumerate(k) if kk == 'junction_timeout'), None)
    check('끝나지 않는 행렬: 상한 전에는 go 없음',
          t_to is not None and 'junction_go' not in k[:t_to], ','.join(k))

    # 8. adopt 뒤 래치 유지
    path, z, pol, ev = setup()
    fns = [_crosser(10 + i, 2.0 + 3.0 * i, y0=-60.0) for i in range(5)]
    o_at = objs(*fns)
    x, v, idx = 0.0, 12.5, 0
    swapped = False
    n_yield_frames = 0
    broke = False
    for kf in range(700):
        t = kf * 0.05
        while idx < len(path.pts) - 1 and path.s[idx + 1] <= x:
            idx += 1
        if not swapped and pol._yz is not None and v < 0.3:
            ev2 = ev
            p2 = _Path()
            new = JunctionYieldPolicy(
                p2, [make_zone(p2, 100, 130, '1', False, _cross_area())],
                on_event=lambda k, d: ev2.append((k, d)))
            new.adopt(pol)
            pol, path, swapped = new, p2, True
            was = True
        cmd = pol.update(idx, x, v, x, 0.0, 0.0, o_at(t, x), 0, t)
        if swapped and t < 20.0:
            n_yield_frames += 1
            if not cmd.yielding:
                broke = True
        v_cmd = min(12.5, cmd.v_limit)
        a = max(-6.0, min(3.0, min(1.2 * (v_cmd - v), cmd.accel_limit)))
        if v_cmd <= 0.0:
            a = min(a, -1.5)
        x += v * 0.05
        v = max(0.0, v + a * 0.05)
    k = kinds(ev)
    check('adopt: 경로 교체 뒤에도 양보 유지',
          swapped and not broke and k.count('junction_yield') == 1,
          f'교체 {swapped}, 끊김 {broke}, {",".join(k)}')

    # 9. 행렬 간격 5s -> 틈에 go 없음 / 간격 12s -> 틈에 go
    path, z, pol, ev = setup()
    fns = [_crosser(10 + i, 2.0 + 5.0 * i, y0=-60.0) for i in range(4)]
    log = _sim(pol, path, objs(*fns), n=900)
    k = kinds(ev)
    t_last = 17.0 + (60.0 + 15.0 + AREA_BUF_M) / 8.33
    t_go = next((r[0] for i, r in enumerate(log)
                 if i and log[i - 1][6] and not r[6]), None)
    check('간격 5s 행렬: 차 사이에서 go를 내지 않는다',
          k.count('junction_go') == 1 and t_go is not None and t_go > t_last,
          f'go {k.count("junction_go")}회, 출발 t={t_go} (마지막 차 이탈 {t_last:.1f}s)')
    path, z, pol, ev = setup()
    fns = [_crosser(10 + i, 2.0 + 14.0 * i, y0=-60.0) for i in range(2)]
    log = _sim(pol, path, objs(*fns), n=500)
    k = kinds(ev)
    t_go = next((r[0] for i, r in enumerate(log)
                 if i and log[i - 1][6] and not r[6]), None)
    check('간격 14s 행렬: 첫 차 뒤 틈에 출발',
          t_go is not None and t_go < 16.0, f'출발 t={t_go}, {",".join(k)}')
    # 시간창이 4s 고정이었다면 5s 간격에서 틈이 보인다
    path, z, pol, ev = setup()
    pol.update(0, s_stop, 0.0, s_stop, 0.0, 0.0, [], 0, 0.0)
    cmdh = YieldCmd()
    tc = t_clear(0.0, (z.s_j1 + EGO_LEN) - s_stop, pol.v_app)
    check('정지 출발 시간창이 5s 간격보다 길다',
          max(T_HORIZON_MIN, min(T_HORIZON_MAX, tc + T_MARGIN)) > 5.0 + CLEAR_S - 0.01
          or tc + T_MARGIN > 5.0, f't_clear {tc:.1f}s')

    # 10. 50km/h 차량이 교차로 중심 70m에서 접근 -> yield. 10km/h는 무발화
    path, z, pol, ev = setup()
    fast = _crosser(1, 0.0, y0=-70.0, v=50.0 / 3.6)
    log = _sim(pol, path, objs(fast), n=40, x0=s_stop - 30.0, v_init=6.9, v0=6.9)
    yd = [d for kk, d in ev if kk == 'junction_yield']
    m = re.search(r'거리=(\d+)m', yd[0]) if yd else None
    check('50km/h 차량 70m 접근: yield (반경 50m 밖에서)',
          m is not None and int(m.group(1)) > 55, yd[0][:90] if yd else '무발화')
    path, z, pol, ev = setup()
    slow = _crosser(1, 0.0, y0=-70.0, v=10.0 / 3.6)
    _sim(pol, path, objs(slow), n=40, x0=s_stop - 30.0, v_init=6.9, v0=6.9)
    check('10km/h 차량 70m: 무발화', 'junction_yield' not in kinds(ev),
          ','.join(kinds(ev)))

    # 11. 비신호 접근 상한
    path, z, pol, ev = setup()
    log = _sim(pol, path, objs(), n=400)
    at40 = [r for r in log if s_stop - JY_APPROACH_M - 1.0 <= r[1] <= s_stop]
    vmax = max(r[2] for r in at40)
    check('비신호 접근: 40m 앞부터 25km/h', vmax * 3.6 <= JY_APPROACH_KMH + 1.5,
          f'구간 내 최대 {vmax * 3.6:.1f}km/h')
    check('비신호 접근: junction_approach', 'junction_approach' in kinds(ev))
    path, z, pol, ev = setup()
    # ego가 정지점 20m 앞일 때 나타나는 교차 차량
    appear = {'t': None}

    def late2(t, x):
        if appear['t'] is None:
            if x < s_stop - 20.0:
                return []
            appear['t'] = t
        d = 8.33 * (t - appear['t'])
        if d > 80.0:
            return []
        return [OB._Fake(1, 115.0, -30.0 + d, speed=8.33, heading=math.pi / 2)]
    log = _sim(pol, path, late2, n=600)
    k = kinds(ev)
    held = [r for r in log if r[6]]
    check('비신호: 20m 앞에서 위협이 나타나도 commit 0·정지',
          'junction_commit' not in k and 'junction_yield' in k
          and max(r[1] for r in held) <= s_stop + 0.6, ','.join(k))

    # 12. 요레이트 ― 회전 차량의 예측이 영역에 든다 (직선이면 빗나간다)
    path, z, pol, ev = setup()
    # 영역 남서쪽에서 동쪽으로 달리다 좌회전(북쪽)해 x=115 연결로로 드는 차
    w = 0.4

    def turner(t):
        h = w * t
        r = 8.33 / w
        return OB._Fake(1, 94.0 + r * math.sin(h), -45.0 + r * (1 - math.cos(h)),
                        speed=8.33, heading=h)
    for kf in range(8):
        pol.update(60, 60.0, 5.0, 60.0, 0.0, 0.0, [turner(kf * 0.05)], 0, kf * 0.05)
    o = turner(0.35)
    wr = pol._tracks[1].hist.rate(o.speed)
    check('요레이트 추정 0.4rad/s', abs(wr - 0.4) < 0.03, f'{wr:.3f}')
    th = pol._threats(z, 60, [o], 6.0)
    pol._tracks[1].hist.clear()
    th0 = pol._threats(z, 60, [o], 6.0)
    check('회전 차량: CTRV 예측은 영역에 들고 직선 외삽은 빗나간다',
          bool(th) and not th0, f'CTRV {len(th)}대, 직선 {len(th0)}대')

    # 14. 빠져나가는 차량 ― 영역 안이어도 ego 회랑에서 멀어지면 위협이 아니다
    az = _cross_area() + [(100.0 + i, 3.0) for i in range(31)]
    for name, lat, hdg, want in (
            ('대향 차로(횡 3.0m)로 빠져나간다', 3.0, math.pi, False),
            ('횡 2.0m로 스친다', 2.0, math.pi, True),
            ('횡 3.0m에서 회랑으로 꺾어 든다', 3.0, -0.75 * math.pi, True)):
        path = _Path()
        z = make_zone(path, 100, 130, '1', False, az)
        pol = JunctionYieldPolicy(path, [z])
        o = OB._Fake(1, 125.0, lat, speed=8.33, heading=hdg)
        for kf in range(4):
            pol.update(60, 60.0, 0.0, 60.0, 0.0, 0.0, [o], 0, kf * 0.05)
        th = pol._threats(z, 60, [o], 8.0)
        check(f'영역 안 차량 ― {name}: {"위협" if want else "위협 아님"}',
              bool(th) == want and _in(z.area, o.x, o.y), f'{th}')

    # 15. 정지점 통일 ― 신호 교차로는 stoplines의 정지점보다 앞서지 않는다
    path = _Path()
    z_sig = make_zone(path, 100, 130, '1', True, _cross_area(), 'lt', s_line=99.85)
    pol = JunctionYieldPolicy(path, [z_sig])
    check('신호 교차로 정지점 = 정지선 - 범퍼 - 1.0 (stoplines와 같다)',
          abs(pol._s_stop(z_sig) - (99.85 - FRONT_OVERHANG - SL.STOP_MARGIN)) < 1e-9,
          f'{pol._s_stop(z_sig):.3f}')
    path, z, pol, ev = setup()
    check('비신호 정지점 불변', abs(pol._s_stop(z) - s_stop) < 1e-9)

    # 16. 양보 중 신호 전환 3 -> 2 -> 1. 위협이 남은 동안 계속 서고, stoplines가
    #     정지선을 놓지 않으며, 위협이 사라진 뒤에는 적색이 세운다
    def tl_at(t):
        if t < 14.0:
            return SL.GREEN
        if t < 17.0:
            return SL.YELLOW
        return SL.RED if t < 45.0 else SL.GREEN_LEFT
    path = _Path()
    z = make_zone(path, 100, 130, '1', True, _cross_area(), 'lt', s_line=99.85)
    ev = []
    pol = JunctionYieldPolicy(path, [z], on_event=lambda k, d: ev.append((k, d)))
    line = SL.Stopline(100, 99.85, 99.85, 0.0, frozenset({7}), 'left', 0.0)
    sig = SL.SignalPolicy(path, [line], on_event=lambda k, d: ev.append((k, d)))
    o_at = objs(*[onc(20 + i, 3.0 + 4.0 * i) for i in range(6)])
    x, v, idx = 0.0, 12.5, 0
    rec = []
    for kf in range(1200):
        t = kf * 0.05
        while idx < len(path.pts) - 1 and path.s[idx + 1] <= x:
            idx += 1
        tls = tl_at(t) if x <= 99.85 else 0
        jc = pol.update(idx, x, v, x, 0.0, 0.0, o_at(t, x), tls, t)
        sc = sig.update(x, v, 7 if tls else 0, tls, t,
                        blocked=jc.yielding, yielding=jc.yielding)
        v_cmd = min(12.5, jc.v_limit, sc.v_limit)
        a = max(-6.0, min(3.0, min(1.2 * (v_cmd - v), jc.accel_limit,
                                   sc.accel_limit)))
        if v_cmd <= 0.0:
            a = min(a, -1.5)
        x += v * 0.05
        v = max(0.0, v + a * 0.05)
        rec.append((t, x, v, jc.yielding, jc.n, tls))
    k = kinds(ev)
    t_go = next((d for kk, d in ev if kk == 'junction_go'), '')
    last_th = max((r[0] for r in rec if r[4] > 0), default=0.0)
    check('신호 전환: 양보가 3->2->1을 넘어 이어진다',
          all(r[3] for r in rec if 14.0 <= r[0] <= min(last_th, 20.0))
          and last_th > 17.0, f'마지막 위협 t={last_th:.1f}s')
    check('신호 전환: 황색 통과·정지선 통과 판정 없음',
          'tl_yellow_go' not in k and 'tl_late' not in k, ','.join(k))
    red = [r for r in rec if 14.0 <= r[0] < 45.0]
    check('신호 전환: 황색·적색 동안 범퍼가 정지선을 넘지 않는다',
          max(r[1] for r in red) + FRONT_OVERHANG <= 99.85 + 1e-6
          and max(r[2] for r in red if r[0] > 15.0) < SL.STOP_SPEED,
          f'범퍼 최대 {max(r[1] for r in red) + FRONT_OVERHANG:.2f} '
          f'v 최대 {max(r[2] for r in red if r[0] > 15.0):.2f}')
    check('신호 전환: 위협이 사라지면 junction_go, 그 뒤는 적색이 세운다',
          k.count('junction_go') == 1 and 'RED' in t_go and 'tl_stop' in k, t_go)
    check('신호 전환: 다음 녹색에 출발해 통과', rec[-1][1] > 135.0,
          f'끝 s={rec[-1][1]:.1f}')
    # stoplines 단독: 양보 중에는 딜레마 통과를 내지 않는다
    for yl, want in ((True, 'stop'), (False, 'go')):
        ev2 = []
        sp = SL.SignalPolicy(path, [line], on_event=lambda kk, d: ev2.append(kk))
        d0 = 99.85 - FRONT_OVERHANG - 0.8
        c = sp.update(d0, 3.0, 7, SL.YELLOW, 0.0, yielding=yl)
        check(f'stoplines: 황색·범퍼 0.8m 앞·3m/s, yielding={yl} -> {want}',
              (c.action == 'go') == (want == 'go')
              and ('tl_yellow_go' in ev2) == (want == 'go'),
              f'{c.action} {",".join(ev2)}')

    # 13. 비용
    import time
    path, z, pol, ev = setup()
    many = [OB._Fake(i + 1, 115.0 + (i % 5), -40.0 + i, speed=8.0,
                     heading=math.pi / 2) for i in range(30)]
    for kf in range(3):
        pol.update(50, 50.0, 6.0, 50.0, 0.0, 0.0, many, 0, kf * 0.05)
    t0 = time.perf_counter()
    for kf in range(200):
        pol.update(50, 50.0, 6.0, 50.0, 0.0, 0.0, many, 0, 1.0 + kf * 0.05)
    ms = (time.perf_counter() - t0) / 200 * 1000.0
    check('비용: 이동 차량 30대 < 3ms/프레임', ms < 3.0, f'{ms:.2f}ms')

    print(f'\n{"전부 통과" if not fails else f"실패 {len(fails)}건: " + ", ".join(fails)}')
    return 1 if fails else 0


# ----------------------------------------------------------------------

def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    data = os.path.normpath(os.path.join(here, '..', 'data'))
    ap = argparse.ArgumentParser(description='교차로 진입 양보 게이트')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--build', action='store_true',
                    help='xodr에서 road -> junction id CSV를 만든다')
    ap.add_argument('--xodr', default=os.path.join(data, 'HL_FMA_VTD_LivingLab.xodr'))
    ap.add_argument('--out', default=os.path.join(data, JROADS_CSV))
    ap.add_argument('--plan', action='store_true', help='경로 위 교차로 표')
    ap.add_argument('--path', default='')
    ap.add_argument('--segments', default='')
    ap.add_argument('--events', default='')
    ap.add_argument('--lanes', default=os.path.join(data, 'xodr_lanes.csv'))
    args = ap.parse_args()

    if args.selftest:
        return _selftest()
    if args.build:
        n = build_junction_roads(args.xodr, args.out)
        print(f'{args.out}: 교차로 내부 도로 {n}개')
        return 0
    if args.plan:
        if not args.path:
            ap.error('--plan에는 --path가 필요하다')
        from pure_pursuit import load_path
        segs = [s.strip() for s in args.segments.split(',') if s.strip()]
        if not segs:
            with open(args.path, newline='', encoding='utf-8') as fh:
                for r in csv.DictReader(fh):
                    if r['segment_id'] not in segs:
                        segs.append(r['segment_id'])
        path, _b = load_path(args.path, segs)
        evp = args.events or SL.default_events_path(args.path)
        sls = SL.load_for_path(path, evp)[0] if evp else []
        jroads = load_junction_roads(args.out)
        if not jroads:
            print(f'*** {args.out} 가 없다 ― 엣지 CSV 폴백 (거리 {NEAR_FALLBACK_M:.0f}m) ***')
            jroads = fallback_junction_roads(os.path.join(data, 'xodr_graph_edges.csv'))
        zones, warn = load_for_path(path, args.path, segs, jroads,
                                    load_area_points(args.lanes, jroads), sls)
        for w in warn:
            print(f'*** {w} ***')
        pol = JunctionYieldPolicy(path, zones)
        print(f'경로 {path.finish_s:.1f}m, 교차로 {len(zones)}곳')
        for line in summary(pol):
            print(line)
        return 0
    ap.print_help()
    return 0


if __name__ == '__main__':
    sys.exit(main())
