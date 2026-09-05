#!/usr/bin/env python3
"""xodr의 planView를 읽어 도로 중심선(reference line)을 점열로 만든다. 1단계.

    python xodr_parser.py                    # CSV 생성 + 요약
    python xodr_parser.py --validate         # + 녹화 궤적 3종 대조 리포트
    python xodr_parser.py --road 429 --step 0.5 --out ../logs/road429.csv

본 대회 경로는 사전테스트와 좌표가 다르다. 녹화해 둔 waypoint를 그대로 쓸 수
없으므로 도로망을 xodr에서 직접 만든다. 2~4단계(lane / junction / 정지선·신호등)가
전부 이 위에 얹히므로, 1단계는 **road의 s -> 월드 (x, y, heading) 변환이 정확한가**
하나만 확실히 못박는 것이 목적이다.

설계 근거
- 각 geometry는 자기 시작점(x, y, hdg)에서 다시 출발한다. geometry 간 오차가
  누적되지 않으므로 도로 전체를 이어서 적분할 필요가 없다
- spiral은 Fresnel 적분 없이 0.1m 중점법 수치적분으로 충분하다. 4000개 전수
  연속성 검증에서 최대 2.2mm였다
- poly3의 length는 u가 아니라 **호길이**다. ds = sqrt(1 + (dv/du)^2) du 를
  적분해 목표 호길이에 해당하는 u를 찾는다
- 이 지도는 편도 5차로까지 있고 <laneOffset>이 276개 있다. 따라서 녹화 궤적과
  reference line의 **절대거리는 파싱 정확성의 판정 기준이 될 수 없다**.
  판정은 헤딩 일치와 도로별 오프셋 일관성으로 한다 (--validate 참고)

표준 라이브러리만 사용한다.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import time
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from dataclasses import dataclass, field

KINDS = ('line', 'arc', 'spiral', 'poly3')

STEP_INT = 0.1              # spiral / poly3 수치적분 스텝 [m]
EPS = 1e-12


# ----------------------------------------------------------------------
# geometry
# ----------------------------------------------------------------------

@dataclass
class Geometry:
    """planView의 <geometry> 하나. s는 도로 기준, ds는 이 geometry 내부 거리."""

    s: float
    x: float
    y: float
    hdg: float
    length: float
    kind: str
    p: dict[str, float]
    _tab: list[tuple[float, float, float]] | None = field(default=None, repr=False)

    @property
    def s_end(self) -> float:
        return self.s + self.length

    # -- 수치적분 테이블 (spiral / poly3 전용) -------------------------
    def _table(self) -> list[tuple[float, float, float]]:
        """호길이 STEP_INT 간격의 (x, y, heading) 로컬 테이블. 최초 1회만 만든다.

        로컬 좌표계는 시작점이 원점이고 hdg 방향이 +x다. 월드 변환은 호출부에서 한다.
        """
        if self._tab is not None:
            return self._tab

        n = max(1, int(math.ceil(self.length / STEP_INT)))
        ds = self.length / n
        tab = [(0.0, 0.0, 0.0)]

        if self.kind == 'spiral':
            c0 = self.p['curvStart']
            rate = ((self.p['curvEnd'] - c0) / self.length
                    if self.length > EPS else 0.0)
            x = y = th = 0.0
            for i in range(n):
                k = c0 + rate * ((i + 0.5) * ds)
                th_mid = th + k * ds * 0.5          # 중점법
                x += math.cos(th_mid) * ds
                y += math.sin(th_mid) * ds
                th += k * ds
                tab.append((x, y, th))
        else:                                       # poly3
            a, b, c, d = (self.p['a'], self.p['b'], self.p['c'], self.p['d'])
            # u를 STEP_INT 간격으로 전진시키며 호길이를 누적하고,
            # 호길이가 ds의 배수를 넘을 때마다 선형 보간해 표본을 찍는다
            u = 0.0
            arc = 0.0
            target = ds
            u_step = STEP_INT
            while len(tab) <= n:
                dv = b + 2 * c * u + 3 * d * u * u
                dl = math.hypot(1.0, dv) * u_step
                if dl <= EPS:
                    break
                if arc + dl >= target:
                    uu = u + u_step * (target - arc) / dl
                    v = a + uu * (b + uu * (c + uu * d))
                    dvv = b + 2 * c * uu + 3 * d * uu * uu
                    tab.append((uu, v, math.atan(dvv)))
                    target += ds
                    continue
                arc += dl
                u += u_step
            while len(tab) <= n:                    # 수치 오차로 모자라면 마지막 반복
                tab.append(tab[-1])

        self._tab = tab
        return tab

    # -- 공개 API ------------------------------------------------------
    def pose_at(self, ds: float) -> tuple[float, float, float]:
        """geometry 시작에서 호길이 ds 지점의 월드 (x, y, heading)."""
        ds = min(max(ds, 0.0), self.length)
        hdg = self.hdg

        if self.kind == 'line':
            return (self.x + math.cos(hdg) * ds,
                    self.y + math.sin(hdg) * ds, hdg)

        if self.kind == 'arc':
            k = self.p['curvature']
            if abs(k) < EPS:
                return (self.x + math.cos(hdg) * ds,
                        self.y + math.sin(hdg) * ds, hdg)
            th = hdg + k * ds
            return (self.x + (math.sin(th) - math.sin(hdg)) / k,
                    self.y + (math.cos(hdg) - math.cos(th)) / k, th)

        # spiral / poly3 ― 테이블 선형 보간
        tab = self._table()
        n = len(tab) - 1
        if n <= 0 or self.length <= EPS:
            lx, ly, lth = 0.0, 0.0, 0.0
        else:
            f = ds / self.length * n
            i = min(int(f), n - 1)
            t = f - i
            (x0, y0, t0), (x1, y1, t1) = tab[i], tab[i + 1]
            lx, ly, lth = (x0 + (x1 - x0) * t,
                           y0 + (y1 - y0) * t,
                           t0 + (t1 - t0) * t)
        ca, sa = math.cos(hdg), math.sin(hdg)
        return (self.x + lx * ca - ly * sa,
                self.y + lx * sa + ly * ca,
                hdg + lth)

    def curvature_at(self, ds: float) -> float:
        """해당 지점의 곡률. lane0 헤딩 보정에만 쓴다."""
        if self.kind == 'line':
            return 0.0
        if self.kind == 'arc':
            return self.p['curvature']
        if self.kind == 'spiral':
            if self.length <= EPS:
                return self.p['curvStart']
            c0, c1 = self.p['curvStart'], self.p['curvEnd']
            return c0 + (c1 - c0) * min(max(ds, 0.0), self.length) / self.length
        # poly3: k = v'' / (1 + v'^2)^1.5. u는 테이블에서 가져온다
        tab = self._table()
        n = len(tab) - 1
        u = tab[min(max(int(round(ds / self.length * n)), 0), n)][0] \
            if n > 0 and self.length > EPS else 0.0
        b, c, d = self.p['b'], self.p['c'], self.p['d']
        dv = b + 2 * c * u + 3 * d * u * u
        ddv = 2 * c + 6 * d * u
        return ddv / (1.0 + dv * dv) ** 1.5


# ----------------------------------------------------------------------
# road
# ----------------------------------------------------------------------

@dataclass
class Sample:
    s: float
    x: float
    y: float
    heading: float
    x0: float
    y0: float
    heading0: float
    lane_offset: float


@dataclass
class Road:
    id: str
    length: float
    junction: str
    geoms: list[Geometry]
    offsets: list[tuple[float, float, float, float, float]]   # (s, a, b, c, d)

    @property
    def segment_id(self) -> str:
        return f'road_{self.id}'

    def _geom_at(self, s: float) -> Geometry:
        g = self.geoms[0]
        for cand in self.geoms:
            if s + 1e-9 < cand.s:
                break
            g = cand
        return g

    def pose_at(self, s: float) -> tuple[float, float, float]:
        """도로 s 위치의 reference line 월드 (x, y, heading)."""
        g = self._geom_at(s)
        return g.pose_at(s - g.s)

    def lane_offset_at(self, s: float) -> tuple[float, float]:
        """<laneOffset>이 주는 (t, dt/ds). 없으면 (0, 0)."""
        rec = None
        for r in self.offsets:
            if s + 1e-9 < r[0]:
                break
            rec = r
        if rec is None:
            return 0.0, 0.0
        s0, a, b, c, d = rec
        u = s - s0
        return (a + u * (b + u * (c + u * d)),
                b + 2 * c * u + 3 * d * u * u)

    def lane0_at(self, s: float) -> tuple[float, float, float]:
        """laneOffset을 적용한 lane 0 중심선 (x, y, heading).

        헤딩을 reference line 것으로 쓰면 안 된다. 관측된 b가 0.32까지 나와
        확폭 구간에서 18도까지 벌어진다. 오프셋 곡선의 접선으로 계산한다.
        """
        g = self._geom_at(s)
        ds = s - g.s
        x, y, th = g.pose_at(ds)
        t, dt = self.lane_offset_at(s)
        if t == 0.0 and dt == 0.0:
            return x, y, th
        k = g.curvature_at(ds)
        return (x - math.sin(th) * t,
                y + math.cos(th) * t,
                th + math.atan2(dt, 1.0 - k * t))

    def point_at(self, s: float, t: float) -> tuple[float, float]:
        """도로 좌표 (s, t)를 월드로. 4단계 정지선·신호등이 이걸 쓴다."""
        x, y, th = self.pose_at(s)
        return x - math.sin(th) * t, y + math.cos(th) * t

    def sample(self, step: float = 1.0) -> list[Sample]:
        """s = 0, step, 2*step, ... 과 도로 끝점."""
        out: list[Sample] = []
        n = max(1, int(self.length // step))
        ss = [i * step for i in range(n + 1)]
        if ss[-1] < self.length - 1e-6:
            ss.append(self.length)
        for s in ss:
            x, y, th = self.pose_at(s)
            x0, y0, th0 = self.lane0_at(s)
            t, _ = self.lane_offset_at(s)
            out.append(Sample(s, x, y, th, x0, y0, th0, t))
        return out


# ----------------------------------------------------------------------
# 파싱
# ----------------------------------------------------------------------

def parse_roads(path: str) -> list[Road]:
    """xodr의 모든 <road>를 planView + laneOffset만 읽어 Road로 만든다.

    laneSection / lane 기하, junction 연결, elevation, object/signal은
    이 단계에서 다루지 않는다.
    """
    root = ET.parse(path).getroot()
    roads: list[Road] = []

    for r in root.findall('road'):
        geoms: list[Geometry] = []
        pv = r.find('planView')
        for ge in (pv.findall('geometry') if pv is not None else ()):
            prim = next((ch for ch in ge if ch.tag in KINDS), None)
            if prim is None:
                continue
            geoms.append(Geometry(
                s=float(ge.get('s')), x=float(ge.get('x')), y=float(ge.get('y')),
                hdg=float(ge.get('hdg')), length=float(ge.get('length')),
                kind=prim.tag,
                p={k: float(v) for k, v in prim.attrib.items()}))
        if not geoms:
            continue
        geoms.sort(key=lambda g: g.s)

        offsets = []
        lanes = r.find('lanes')
        if lanes is not None:
            for lo in lanes.findall('laneOffset'):
                offsets.append(tuple(float(lo.get(k))
                                     for k in ('s', 'a', 'b', 'c', 'd')))
        offsets.sort()

        roads.append(Road(id=r.get('id'), length=float(r.get('length')),
                          junction=r.get('junction', '-1'),
                          geoms=geoms, offsets=offsets))
    return roads


# ----------------------------------------------------------------------
# CSV
# ----------------------------------------------------------------------

CSV_HEADER = ['segment_id', 'seq', 's', 'x', 'y', 'heading',
              'x0', 'y0', 'heading0', 'lane_offset']


def write_csv(path: str, roads: list[Road], step: float = 1.0) -> int:
    """x,y,heading 은 reference line, x0,y0,heading0 은 lane0 중심선.

    plot_waypoints_svg / route_check / pure_pursuit 세 소비자 모두
    csv.DictReader로 이름 참조라 뒤 열이 늘어도 무해하다. 다만
    pure_pursuit.load_path 가 encoding='utf-8'(utf-8-sig 아님)로 열기 때문에
    BOM 없이 쓴다.
    """
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    total = 0
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(CSV_HEADER)
        for road in roads:
            for i, p in enumerate(road.sample(step)):
                w.writerow([road.segment_id, i, f'{p.s:.3f}',
                            f'{p.x:.3f}', f'{p.y:.3f}', f'{p.heading:.5f}',
                            f'{p.x0:.3f}', f'{p.y0:.3f}', f'{p.heading0:.5f}',
                            f'{p.lane_offset:.3f}'])
                total += 1
    return total


# ----------------------------------------------------------------------
# 검증
# ----------------------------------------------------------------------

CELL = 5.0                  # 격자 셀 크기 [m]
HDG_LIMIT = 0.02            # 헤딩차 중앙값 임계 [rad] ― 넘으면 파싱 오류
SD_LIMIT = 0.5              # 도로별 부호 오프셋 표준편차 임계 [m]
OVER_LIMIT = 0.20           # 설명한계 초과 비율 임계
MIN_N = 10                  # 이보다 적게 배정된 도로는 통계에서 뺀다


def driving_halfwidth(xodr_path: str) -> dict[str, float]:
    """도로별 '주행차로가 닿는 최대 횡거리'. 검증 전용 스칼라 상한이다.

    <lane type="driving">의 <width a>만 안쪽부터 누적한다. 차로 기하를
    만들지 않으므로 2단계 침범이 아니다. 절대거리가 설명 가능한 범위인지
    판단하는 데만 쓴다.
    """
    root = ET.parse(xodr_path).getroot()
    out: dict[str, float] = {}
    for r in root.findall('road'):
        best = 0.0
        lanes = r.find('lanes')
        for ls in (lanes.findall('laneSection') if lanes is not None else ()):
            for side in ('left', 'right'):
                sd = ls.find(side)
                if sd is None:
                    continue
                acc = 0.0
                reach = 0.0
                for lane in sd.findall('lane'):
                    w = lane.find('width')
                    acc += float(w.get('a')) if w is not None else 0.0
                    if lane.get('type') == 'driving':
                        reach = acc
                best = max(best, reach)
        out[f'road_{r.get("id")}'] = best
    return out


class Index:
    """5m 셀 균등 격자. 전수 대조(1496 x 34000)를 피하기 위한 것이다."""

    def __init__(self, lines: dict[str, list[tuple[float, float, float, float]]]):
        self.lines = lines
        self.grid: dict[tuple[int, int], list[tuple[str, int]]] = defaultdict(list)
        for name, pts in lines.items():
            for i in range(len(pts) - 1):
                ax, ay = pts[i][0], pts[i][1]
                bx, by = pts[i + 1][0], pts[i + 1][1]
                for cx in range(int(math.floor(min(ax, bx) / CELL)),
                                int(math.floor(max(ax, bx) / CELL)) + 1):
                    for cy in range(int(math.floor(min(ay, by) / CELL)),
                                    int(math.floor(max(ay, by) / CELL)) + 1):
                        self.grid[(cx, cy)].append((name, i))

    def nearest(self, px: float, py: float):
        """(거리, segment_id, 부호 횡오프셋, 도로 s, 그 지점 heading)."""
        best = None
        rad = 1
        cx0, cy0 = int(math.floor(px / CELL)), int(math.floor(py / CELL))
        while True:
            cand: list[tuple[str, int]] = []
            for i in range(cx0 - rad, cx0 + rad + 1):
                for j in range(cy0 - rad, cy0 + rad + 1):
                    cand.extend(self.grid.get((i, j), ()))
            for name, i in cand:
                pts = self.lines[name]
                ax, ay, ah, as_ = pts[i]
                bx, by, _bh, bs = pts[i + 1]
                d, t, qx, qy = point_to_segment(px, py, ax, ay, bx, by)
                if best is None or d < best[0]:
                    th = math.atan2(by - ay, bx - ax)
                    lat = -math.sin(th) * (px - qx) + math.cos(th) * (py - qy)
                    best = (d, name, lat, as_ + (bs - as_) * t, ah)
            if best is not None or rad >= 16:
                return best
            rad *= 2


def point_to_segment(px, py, ax, ay, bx, by):
    """route_check.point_to_segment 와 같은 식. import 실패 시 폴백용 사본."""
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    if den < 1e-12:
        return math.hypot(px - ax, py - ay), 0.0, ax, ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / den))
    qx, qy = ax + t * dx, ay + t * dy
    return math.hypot(px - qx, py - qy), t, qx, qy


try:                                    # 같은 식이 두 벌 돌아다니지 않게 한다
    from route_check import point_to_segment as _p2s   # noqa: E402
    point_to_segment = _p2s
except ImportError:
    pass


def pct(vals: list[float], q: float) -> float:
    v = sorted(vals)
    return v[min(len(v) - 1, int(len(v) * q))]


def build_lines(roads: list[Road], step: float, lane0: bool):
    """segment_id -> [(x, y, heading, s)]"""
    out = {}
    for r in roads:
        pts = []
        for p in r.sample(step):
            if lane0:
                pts.append((p.x0, p.y0, p.heading0, p.s))
            else:
                pts.append((p.x, p.y, p.heading, p.s))
        out[r.segment_id] = pts
    return out


def validate(roads: list[Road], xodr_path: str, targets: list[str],
             step: float) -> None:
    ref_lines = build_lines(roads, step, lane0=False)
    l0_lines = build_lines(roads, step, lane0=True)
    ref_idx, l0_idx = Index(ref_lines), Index(l0_lines)
    halfw = driving_halfwidth(xodr_path)
    by_id = {r.segment_id: r for r in roads}
    kinds = {r.segment_id: Counter(g.kind for g in r.geoms) for r in roads}

    print()
    print('=' * 78)
    print('검증 ― 녹화 궤적 대조')
    print('=' * 78)
    print('reference line은 도로 중심선이고 이 지도는 편도 5차로까지 있다.')
    print('<laneOffset>도 276개 있어 바깥 차로를 달리면 13m 떨어지는 것이 정상이다.')
    print('따라서 **절대거리는 판정 기준이 아니다**. 아래 A는 참고용이고')
    print('파싱 정확성 판정은 B(헤딩 / 오프셋 일관성 / 설명한계)로 한다.')

    for path in targets:
        if not os.path.exists(path):
            print(f'\n[건너뜀] {path} 없음')
            continue
        with open(path, encoding='utf-8-sig', newline='') as fh:
            rows = list(csv.DictReader(fh))
        if not rows:
            print(f'\n[건너뜀] {path} 비어 있음')
            continue
        pts = [(float(r['x']), float(r['y']),
                float(r.get('heading', 0) or 0)) for r in rows]

        t0 = time.time()
        print('\n' + '-' * 78)
        print(f'{os.path.basename(path)}   n={len(pts)}')
        print('-' * 78)

        # ---- A. 절대거리 (참고용) ----------------------------------
        print('A. 절대거리 [참고용 ― 판정 아님]')
        print(f'   {"기준":<12} {"평균":>7} {"중앙":>7} {"p95":>7} {"최대":>7}'
              f' {">5m":>7} {">10m":>7}')
        stash = {}
        for label, idx in (('reference', ref_idx), ('lane0', l0_idx)):
            ds = [idx.nearest(x, y)[0] for x, y, _ in pts]
            stash[label] = ds
            n = len(ds)
            print(f'   {label:<12} {sum(ds)/n:7.2f} {pct(ds,.5):7.2f} '
                  f'{pct(ds,.95):7.2f} {max(ds):7.2f} '
                  f'{100*sum(1 for d in ds if d>5)/n:6.1f}% '
                  f'{100*sum(1 for d in ds if d>10)/n:6.1f}%')
        hist = Counter(min(int(d), 15) for d in stash['reference'])
        print('   히스토그램(reference, 1m 버킷, 15=15m이상)')
        print('     ' + ' '.join(f'{k}:{hist[k]}' for k in sorted(hist)))

        # ---- B. 판정 ------------------------------------------------
        dh_all: list[float] = []
        per_lat: dict[str, list[float]] = defaultdict(list)
        per_over: dict[str, list[int]] = defaultdict(list)
        per_hdg: dict[str, list[float]] = defaultdict(list)
        n_over = 0
        for px, py, ph in pts:
            d, name, lat, s, rh = ref_idx.nearest(px, py)
            # 도로 reference 방향과 주행 방향이 반대일 수 있으므로 mod pi
            e = abs((ph - rh + math.pi / 2) % math.pi - math.pi / 2)
            dh_all.append(e)
            per_hdg[name].append(e)
            per_lat[name].append(lat)
            t, _ = by_id[name].lane_offset_at(s)
            over = 1 if d > halfw.get(name, 0.0) + abs(t) + 2.0 else 0
            per_over[name].append(over)
            n_over += over

        print(f'B. 판정   ({time.time() - t0:.2f}s)')
        med_h = pct(dh_all, .5)
        ok_h = med_h < HDG_LIMIT
        print(f'   헤딩차(mod pi)   중앙 {med_h:.4f}  p95 {pct(dh_all,.95):.4f}  '
              f'최대 {max(dh_all):.4f} rad   [{"합격" if ok_h else "불합격"}'
              f' 기준 중앙 < {HDG_LIMIT}]')
        r_over = n_over / len(pts)
        print(f'   설명한계 초과     {n_over}점 ({100*r_over:.1f}%)   '
              f'[{"합격" if r_over < 0.05 else "확인"} 기준 < 5%]'
              f'   한계 = Σ주행차로폭 + |laneOffset| + 2m')

        sd_map: dict[str, float] = {}
        small = 0
        for name, v in per_lat.items():
            if len(v) < MIN_N:
                small += 1
                continue
            m = sum(v) / len(v)
            sd_map[name] = math.sqrt(sum((z - m) ** 2 for z in v) / len(v))
        n_sd = sum(1 for s in sd_map.values() if s > SD_LIMIT)
        print(f'   오프셋 편차       도로 {len(sd_map)}개 중 sd > {SD_LIMIT}m 가 '
              f'{n_sd}개   (n<{MIN_N} 라 제외한 도로 {small}개)')
        print(f'                     sd 경고는 실제 차선변경일 수 있다. '
              f'SVG로 눈으로 가른다')

        # ---- C. 불합격 road 상위 10 --------------------------------
        rank = []
        for name in per_lat:
            hs = per_hdg[name]
            mh = pct(hs, .5)
            sd = sd_map.get(name)
            ov = sum(per_over[name]) / len(per_over[name])
            flags = []
            score = 0.0
            if mh > HDG_LIMIT:
                flags.append(f'헤딩 {mh:.3f}')
                score += 1000 + mh
            if ov > OVER_LIMIT:
                flags.append(f'설명한계 {100*ov:.0f}%')
                score += 100 + ov
            if sd is not None and sd > SD_LIMIT:
                flags.append(f'편차 sd={sd:.2f}')
                score += 10 + sd
            if flags:
                rank.append((score, name, flags, len(per_lat[name])))
        rank.sort(reverse=True)

        print(f'C. 기준 미달 road ({len(rank)}개, 상위 10)')
        if not rank:
            print('   없음')
        else:
            print(f'   {"road":<12} {"n":>5}  {"geometry":<30} {"LO":>3}  걸린 지표')
            for _sc, name, flags, n in rank[:10]:
                kc = dict(kinds[name])
                print(f'   {name:<12} {n:>5}  {str(kc):<30} '
                      f'{len(by_id[name].offsets):>3}  {", ".join(flags)}')


# ----------------------------------------------------------------------

def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    data = os.path.normpath(os.path.join(here, '..', 'data'))

    ap = argparse.ArgumentParser(description='xodr planView -> reference line CSV')
    ap.add_argument('--xodr', default=os.path.join(data, 'HL_FMA_VTD_LivingLab.xodr'))
    ap.add_argument('--out', default=os.path.join(data, 'xodr_reference_lines.csv'))
    ap.add_argument('--step', type=float, default=1.0, help='점 간격 [m]')
    ap.add_argument('--road', help='이 road id 하나만 뽑는다 (디버그)')
    ap.add_argument('--validate', action='store_true',
                    help='녹화 궤적 3종과 대조해 판정 리포트를 낸다')
    ap.add_argument('--targets', nargs='*', default=None,
                    help='대조할 waypoint CSV. 기본은 v2 / pretest1 / pretest2')
    args = ap.parse_args()

    if not os.path.exists(args.xodr):
        raise SystemExit(f'{args.xodr}: 없다')

    t0 = time.time()
    roads = parse_roads(args.xodr)
    t_parse = time.time() - t0

    if args.road:
        roads = [r for r in roads if r.id == args.road]
        if not roads:
            raise SystemExit(f'road {args.road}: 없다')

    ngeom = sum(len(r.geoms) for r in roads)
    kc = Counter(g.kind for r in roads for g in r.geoms)
    nlo = sum(len(r.offsets) for r in roads)
    njun = sum(1 for r in roads if r.junction != '-1')
    total_len = sum(r.length for r in roads)

    print(f'[xodr] {os.path.basename(args.xodr)}  파싱 {t_parse:.2f}s')
    print(f'[xodr] road {len(roads)}개 (junction 소속 {njun}), '
          f'geometry {ngeom}개, laneOffset {nlo}개')
    print(f'[xodr] {"  ".join(f"{k} {kc[k]}" for k in KINDS)}')
    print(f'[xodr] 총 도로 길이 {total_len:.0f} m')

    t0 = time.time()
    n = write_csv(args.out, roads, args.step)
    print(f'[xodr] {os.path.normpath(args.out)}: {n}점 '
          f'(간격 {args.step}m, {time.time() - t0:.2f}s)')

    if args.validate:
        targets = args.targets if args.targets is not None else [
            os.path.join(data, 'waypoints_v2.csv'),
            os.path.join(data, 'waypoints_pretest1.csv'),
            os.path.join(data, 'waypoints_pretest2.csv'),
        ]
        validate(roads, args.xodr, targets, args.step)


if __name__ == '__main__':
    main()
