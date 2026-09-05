#!/usr/bin/env python3
"""xodr의 lane 구조를 읽어 주행차로 중심선을 만든다. 2단계.

    python xodr_lanes.py                     # CSV 생성 + 요약
    python xodr_lanes.py --validate          # + 녹화 궤적 3종 대조 리포트
    python xodr_lanes.py --road 429          # 단일 도로 디버그

1단계 산출물은 도로 중심선이라 녹화 궤적과 항상 반차로(1.4~1.9m) 어긋난다.
주행에 쓰려면 실제 차로 중심선이 필요하고, 3단계 junction 연결과 4단계
정지선·신호등도 전부 "어느 차로인가"를 전제로 한다.

설계 근거
- **type="driving" 이면서 id 0 인 lane 이 802개 있다.** 전부 <width> 가 없는
  center lane 이다. type 만 보고 뽑으면 폭 0짜리 가짜 차로가 생긴다.
  id != 0 조건이 반드시 필요하다
- 폭 누적에는 border / none / sidewalk 도 전부 포함해야 한다. driving 만 더하면
  안쪽에 none(w=0.5) 이 낀 도로가 전부 어긋난다
- **lane id 는 laneSection 경계에서 재번호된다.** road 429 는 s=92 에서
  +6+5+4+3+2 가 +3+2+1 이 된다. 같은 물리 차로가 id 를 바꾸므로
  섹션을 가로질러 이어붙이려면 <lane><link><successor> 를 따라가야 한다
- <width> / <roadMark> 정렬도 1단계 laneOffset 과 같은 이유로 sOffset 만으로
  안정 정렬한다. 튜플 전체로 정렬하면 같은 sOffset 일 때 파일 순서가 뒤집힌다

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

import xodr_parser as X

SAMPLE_STEP = 1.0           # 점 간격 [m]
MIN_WIDTH = 0.1             # 이보다 좁으면 그 지점에 차로가 없다고 본다
DRIVABLE_W = 0.5            # 검증에서 "주행 가능"으로 칠 최소 폭
DIFF_DS = 0.05              # 헤딩용 중앙차분 폭 [m]


def _sorted_by_s(recs: list[tuple]) -> list[tuple]:
    """첫 원소(s/sOffset)만으로 안정 정렬. 같은 s면 파일 순서를 지킨다."""
    return sorted(recs, key=lambda r: round(r[0], 6))


def _poly(a: float, b: float, c: float, d: float, ds: float) -> float:
    return a + ds * (b + ds * (c + ds * d))


# ----------------------------------------------------------------------
# 자료구조
# ----------------------------------------------------------------------

@dataclass
class Lane:
    id: int
    type: str
    widths: list[tuple[float, float, float, float, float]]      # (sOffset,a,b,c,d)
    marks: list[tuple[float, str, str, str]]                    # (sOffset,type,color,change)
    successor: int | None
    predecessor: int | None
    order: int                  # 안쪽에서 바깥쪽 순서 (abs(id) 오름차순, 0부터)

    def width_at(self, ds: float) -> float:
        """laneSection 시작 기준 ds 에서의 폭."""
        if not self.widths:
            return 0.0
        rec = self.widths[0]
        for cand in self.widths:
            if ds + 1e-9 >= cand[0]:
                rec = cand
        return max(0.0, _poly(rec[1], rec[2], rec[3], rec[4], ds - rec[0]))

    def mark_at(self, ds: float) -> tuple[str, str, str]:
        """(type, color, laneChange). roadMark 가 없으면 빈 값."""
        if not self.marks:
            return ('', '', '')
        rec = self.marks[0]
        for cand in self.marks:
            if ds + 1e-9 >= cand[0]:
                rec = cand
        return (rec[1], rec[2], rec[3])


@dataclass
class LaneSection:
    s: float
    s_end: float
    lanes: dict[int, Lane] = field(default_factory=dict)

    def lane_t(self, road: X.Road, s: float, lane_id: int) -> tuple[float, float]:
        """(횡오프셋 t, 그 차로 폭). t는 왼쪽이 양수다.

        같은 쪽에서 자기보다 안쪽인 차로의 폭을 전부 더하고 자기 폭의 절반을 더한다.
        타입은 가리지 않는다 -- border / none / sidewalk 도 실제로 자리를 차지한다.
        """
        me = self.lanes.get(lane_id)
        if me is None:
            return 0.0, 0.0
        sign = 1 if lane_id > 0 else -1
        ds = s - self.s
        inner = 0.0
        for lid, lane in self.lanes.items():
            if lid * sign <= 0 or lane.order > me.order:
                continue
            if lane.order < me.order:
                inner += lane.width_at(ds)
        w = me.width_at(ds)
        return road.lane_offset_at(s)[0] + sign * (inner + w * 0.5), w

    def inner_mark(self, lane_id: int, ds: float) -> tuple[str, str, str]:
        """안쪽(중앙선 방향) 경계의 roadMark.

        OpenDRIVE 에서 lane 의 <roadMark> 는 그 차로의 바깥쪽 경계다.
        따라서 안쪽 경계는 abs(id)-1 차로의 것이고, abs(id)==1 이면 center lane(0) 것이다.
        """
        sign = 1 if lane_id > 0 else -1
        inner_id = 0 if abs(lane_id) == 1 else sign * (abs(lane_id) - 1)
        lane = self.lanes.get(inner_id)
        return lane.mark_at(ds) if lane is not None else ('', '', '')


# ----------------------------------------------------------------------
# 파싱
# ----------------------------------------------------------------------

def parse_lanes(path: str) -> dict[str, list[LaneSection]]:
    """road id -> laneSection 목록 (s 오름차순)."""
    root = ET.parse(path).getroot()
    out: dict[str, list[LaneSection]] = {}

    for r in root.findall('road'):
        rid = r.get('id')
        length = float(r.get('length'))
        lanes_el = r.find('lanes')
        if lanes_el is None:
            continue
        lss = sorted(lanes_el.findall('laneSection'),
                     key=lambda e: float(e.get('s')))
        secs: list[LaneSection] = []
        for i, ls in enumerate(lss):
            s0 = float(ls.get('s'))
            s1 = float(lss[i + 1].get('s')) if i + 1 < len(lss) else length
            sec = LaneSection(s=s0, s_end=s1)
            for side in ('left', 'center', 'right'):
                sd = ls.find(side)
                if sd is None:
                    continue
                ordered = sorted(sd.findall('lane'),
                                 key=lambda e: abs(int(e.get('id'))))
                for order, el in enumerate(ordered):
                    lid = int(el.get('id'))
                    widths = _sorted_by_s([
                        tuple(float(w.get(k))
                              for k in ('sOffset', 'a', 'b', 'c', 'd'))
                        for w in el.findall('width')])
                    marks = _sorted_by_s([
                        (float(m.get('sOffset')), m.get('type', ''),
                         m.get('color', ''), m.get('laneChange', ''))
                        for m in el.findall('roadMark')])
                    link = el.find('link')
                    succ = pred = None
                    if link is not None:
                        if link.find('successor') is not None:
                            succ = int(link.find('successor').get('id'))
                        if link.find('predecessor') is not None:
                            pred = int(link.find('predecessor').get('id'))
                    sec.lanes[lid] = Lane(id=lid, type=el.get('type', ''),
                                          widths=widths, marks=marks,
                                          successor=succ, predecessor=pred,
                                          order=order)
            secs.append(sec)
        out[rid] = secs
    return out


def driving_lane_ids(sec: LaneSection) -> list[int]:
    """그 섹션의 주행차로 id. center(0)는 폭이 없으므로 제외한다."""
    return sorted((lid for lid, ln in sec.lanes.items()
                   if ln.type == 'driving' and lid != 0 and ln.widths),
                  key=lambda v: (-v))


# ----------------------------------------------------------------------
# 샘플링
# ----------------------------------------------------------------------

@dataclass
class LanePoint:
    s: float
    x: float
    y: float
    heading: float
    width: float
    mark_in: tuple[str, str, str]
    mark_out: tuple[str, str, str]


def sample_lane(road: X.Road, sec: LaneSection, lane_id: int,
                step: float = SAMPLE_STEP) -> list[LanePoint]:
    """한 laneSection 안에서 그 차로의 중심선을 step 간격으로 뽑는다."""
    pts: list[LanePoint] = []
    span = sec.s_end - sec.s
    n = max(1, int(span // step))
    ss = [sec.s + i * step for i in range(n + 1)]
    if ss[-1] < sec.s_end - 1e-6:
        ss.append(sec.s_end)

    for s in ss:
        s = min(max(s, sec.s), sec.s_end)
        t, w = sec.lane_t(road, s, lane_id)
        if w < MIN_WIDTH:
            continue
        x, y = road.point_at(s, t)

        # 헤딩은 오프셋 곡선의 접선. t가 laneOffset과 폭 다항식의 합이라
        # 해석 미분이 번거로워 중앙차분으로 dt/ds 를 구한다.
        sa = max(sec.s, s - DIFF_DS)
        sb = min(sec.s_end, s + DIFF_DS)
        if sb - sa > 1e-9:
            ta, _ = sec.lane_t(road, sa, lane_id)
            tb, _ = sec.lane_t(road, sb, lane_id)
            dtds = (tb - ta) / (sb - sa)
        else:
            dtds = 0.0
        g = road._geom_at(s)
        k = g.curvature_at(s - g.s)
        _, _, th = road.pose_at(s)
        heading = th + math.atan2(dtds, 1.0 - k * t)

        ds = s - sec.s
        pts.append(LanePoint(s=s, x=x, y=y, heading=heading, width=w,
                             mark_in=sec.inner_mark(lane_id, ds),
                             mark_out=sec.lanes[lane_id].mark_at(ds)))
    return pts


def build_lanes(roads: dict[str, X.Road], sections: dict[str, list[LaneSection]],
                step: float = SAMPLE_STEP):
    """segment_id -> (road_id, lane_id, section_idx, [LanePoint])."""
    out: dict[str, tuple[str, int, int, list[LanePoint]]] = {}
    for rid, secs in sections.items():
        road = roads.get(rid)
        if road is None:
            continue
        multi = len(secs) > 1
        for si, sec in enumerate(secs):
            for lid in driving_lane_ids(sec):
                pts = sample_lane(road, sec, lid, step)
                if len(pts) < 2:
                    continue
                name = f'road_{rid}_lane_{lid}'
                if multi:
                    name += f'_s{si}'
                out[name] = (rid, lid, si, pts)
    return out


# ----------------------------------------------------------------------
# CSV
# ----------------------------------------------------------------------

CSV_HEADER = ['segment_id', 'seq', 'road_id', 'lane_id', 'section', 's',
              'x', 'y', 'heading', 'width',
              'mark_in_type', 'mark_in_color', 'mark_in_change',
              'mark_out_type', 'mark_out_color', 'mark_out_change']


def write_csv(path: str, lanes) -> int:
    """x,y,heading 을 1단계와 같은 이름으로 둬서 기존 소비자에 그대로 들어간다.

    pure_pursuit.load_path 가 encoding='utf-8'(utf-8-sig 아님)로 열기 때문에
    BOM 없이 쓴다.
    """
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    total = 0
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(CSV_HEADER)
        for name in sorted(lanes):
            rid, lid, si, pts = lanes[name]
            for i, p in enumerate(pts):
                w.writerow([name, i, rid, lid, si, f'{p.s:.3f}',
                            f'{p.x:.3f}', f'{p.y:.3f}', f'{p.heading:.5f}',
                            f'{p.width:.3f}',
                            p.mark_in[0], p.mark_in[1], p.mark_in[2],
                            p.mark_out[0], p.mark_out[1], p.mark_out[2]])
                total += 1
    return total


# ----------------------------------------------------------------------
# 검증
# ----------------------------------------------------------------------

MED_LIMIT = 0.2             # 최근접 거리 중앙값 임계 [m]
FAR = 0.5                   # 이 위는 설명이 필요한 점
GAP_LIMIT = 0.5             # laneSection 경계 끊김 임계 [m]
EDGE_M = 3.0                # 섹션 경계로 칠 s 여유
RUN_MIN = 3                 # 차선변경으로 칠 최소 연속 점 수


def pct(vals: list[float], q: float) -> float:
    v = sorted(vals)
    return v[min(len(v) - 1, int(len(v) * q))]


def nearest_two(idx: X.Index, px: float, py: float):
    """(segment_id, 거리) 상위 2개. 차선변경 걸침 판정에 차순위가 필요하다."""
    cx0, cy0 = int(math.floor(px / X.CELL)), int(math.floor(py / X.CELL))
    cand = set()
    for i in range(cx0 - 2, cx0 + 3):
        for j in range(cy0 - 2, cy0 + 3):
            cand.update(idx.grid.get((i, j), ()))
    per: dict[str, tuple[float, float]] = {}
    for name, i in cand:
        pts = idx.lines[name]
        ax, ay, _ah, as_ = pts[i]
        bx, by, _bh, bs = pts[i + 1]
        d, t, _qx, _qy = X.point_to_segment(px, py, ax, ay, bx, by)
        if name not in per or d < per[name][0]:
            per[name] = (d, as_ + (bs - as_) * t)
    ranked = sorted(per.items(), key=lambda kv: kv[1][0])[:2]
    return [(n, v[0], v[1]) for n, v in ranked]


def check_boundaries(roads, sections) -> None:
    """laneSection 경계에서 <successor> 로 이어지는 차로가 실제로 붙는지."""
    print('\n' + '=' * 78)
    print('E. laneSection 경계 연속성')
    print('=' * 78)
    gaps = []
    ended = 0
    nolink = 0
    for rid, secs in sections.items():
        road = roads.get(rid)
        if road is None or len(secs) < 2:
            continue
        for i in range(len(secs) - 1):
            a, b = secs[i], secs[i + 1]
            sb = a.s_end
            for lid in driving_lane_ids(a):
                ta, wa = a.lane_t(road, sb - 1e-6, lid)
                if wa < DRIVABLE_W:
                    continue
                succ = a.lanes[lid].successor
                if succ is None:
                    nolink += 1
                    continue
                nxt = b.lanes.get(succ)
                if nxt is None or nxt.type != 'driving' or not nxt.widths:
                    ended += 1
                    continue
                tb, wb = b.lane_t(road, sb + 1e-6, succ)
                if wb < DRIVABLE_W:
                    ended += 1
                    continue
                xa, ya = road.point_at(sb - 1e-6, ta)
                xb, yb = road.point_at(sb + 1e-6, tb)
                gaps.append((math.hypot(xa - xb, ya - yb), rid, lid, succ, sb))
    if not gaps:
        print('   경계가 없다')
        return
    gaps.sort(reverse=True)
    v = sorted(g[0] for g in gaps)
    n = len(v)
    over = [g for g in gaps if g[0] > GAP_LIMIT]
    print(f'   이어지는 경계 {n}건  '
          f'(successor 없음=차로 종료 {nolink}건 / 주행차로로 안 이어짐 {ended}건)')
    print(f'   끝점-시작점 거리  중앙 {v[n//2]:.6f}  p95 {v[int(n*.95)]:.6f}  '
          f'최대 {v[-1]:.6f} m')
    print(f'   {GAP_LIMIT}m 초과: {len(over)}건'
          f'   [{"합격" if not over else "불합격"}]')
    for d, rid, a, b, sb in over[:10]:
        print(f'      road {rid:<6} lane {a:+d} -> {b:+d}  s={sb:.2f}  {d:.3f} m')


def validate(roads, sections, lanes, targets: list[str]) -> None:
    # 검증에는 주행 가능한 폭의 점만 쓴다. 폭 0으로 사라지는 차로의 꼬리가
    # 이웃 차로와 겹쳐 최근접 판정을 흔들기 때문이다.
    lines: dict[str, list[tuple[float, float, float, float]]] = {}
    meta: dict[str, tuple[str, int, int]] = {}
    for name, (rid, lid, si, pts) in lanes.items():
        keep = [(p.x, p.y, p.heading, p.s) for p in pts if p.width >= DRIVABLE_W]
        if len(keep) >= 2:
            lines[name] = keep
            meta[name] = (rid, lid, si)
    idx = X.Index(lines)

    print('\n' + '=' * 78)
    print('검증 -- 녹화 궤적 대조')
    print('=' * 78)
    print('1단계는 도로 중심선이라 반차로(1.4~1.9m) 어긋났다. 2단계는 차로 중심선이므로')
    print(f'중앙값이 {MED_LIMIT}m 아래로 떨어져야 한다. {FAR}m 를 넘는 점은 전부')
    print('차선변경 / laneSection 경계 / junction 중 하나로 설명돼야 한다.')

    for path in targets:
        if not os.path.exists(path):
            print(f'\n[건너뜀] {path} 없음')
            continue
        with open(path, encoding='utf-8-sig', newline='') as fh:
            rows = list(csv.DictReader(fh))
        if not rows:
            continue

        t0 = time.time()
        hits = []
        for r in rows:
            px, py = float(r['x']), float(r['y'])
            top = nearest_two(idx, px, py)
            hits.append((px, py, top))
        ds = [h[2][0][1] for h in hits if h[2]]
        n = len(ds)

        print('\n' + '-' * 78)
        print(f'{os.path.basename(path)}   n={n}   ({time.time() - t0:.2f}s)')
        print('-' * 78)
        med = pct(ds, .5)
        print(f'A. 최근접 거리   평균 {sum(ds)/n:.3f}  중앙 {med:.3f}  '
              f'p95 {pct(ds,.95):.3f}  최대 {max(ds):.3f} m')
        print(f'   >{FAR}m {sum(1 for d in ds if d > FAR)}점 '
              f'({100*sum(1 for d in ds if d > FAR)/n:.1f}%)'
              f'   [{"합격" if med < MED_LIMIT else "불합격"} 기준 중앙 < {MED_LIMIT}]')

        # ---- B. >FAR 점 전수 분류 --------------------------------------
        far_idx = [i for i, h in enumerate(hits) if h[2] and h[2][0][1] > FAR]
        runs = []
        cur = []
        for i in far_idx:
            if cur and i == cur[-1] + 1:
                cur.append(i)
            else:
                if len(cur) >= RUN_MIN:
                    runs.append(cur)
                cur = [i]
        if len(cur) >= RUN_MIN:
            runs.append(cur)
        in_run = {i for run in runs for i in run}

        cls = Counter()
        unexplained = []
        for i in far_idx:
            px, py, top = hits[i]
            name, d, s = top[0]
            rid, lid, si = meta[name]
            sec = sections[rid][si]
            # 이웃한 두 차로의 중심 간격은 (w1+w2)/2 다. 두 차로 사이에 있으면
            # 두 거리의 합이 그 값과 같아진다. 확폭 구간은 좌우 폭이 다르므로
            # 한쪽 폭으로 비교하면 안 된다.
            straddle = False
            if len(top) > 1 and i in in_run:
                n2, d2, s2 = top[1]
                r2, l2, i2 = meta[n2]
                w1 = sec.lane_t(roads[rid], s, lid)[1]
                w2 = sections[r2][i2].lane_t(roads[r2], s2, l2)[1]
                span = (w1 + w2) / 2 or 3.0
                straddle = abs((d + d2) - span) <= 0.25 * span
            if straddle:
                cls['차선변경 걸침'] += 1
            elif min(abs(s - sec.s), abs(s - sec.s_end)) < EDGE_M:
                cls['laneSection 경계'] += 1
            elif roads[rid].junction != '-1':
                cls['junction 내부'] += 1
            else:
                cls['미분류'] += 1
                unexplained.append((i, px, py, name, d))
        print(f'B. >{FAR}m 점 분류')
        for k in ('차선변경 걸침', 'laneSection 경계', 'junction 내부', '미분류'):
            if cls[k]:
                print(f'   {k:<16} {cls[k]:>5}점')
        print(f'   [{"합격" if not cls["미분류"] else "불합격"} 미분류 0점이어야 한다]')
        for i, px, py, name, d in unexplained[:10]:
            print(f'      idx {i}  ({px:.1f},{py:.1f})  {name}  {d:.2f} m')

        # ---- C. 차선변경 구간 목록 -------------------------------------
        print(f'C. 차선변경 구간 {len(runs)}개')
        for run in runs:
            a, b = run[0], run[-1]
            na = hits[a][2][0][0]
            nb = hits[b][2][0][0]
            ra, la, _ = meta[na]
            rb, lb, _ = meta[nb]
            wsum = sum(hits[i][2][0][1] + hits[i][2][1][1]
                       for i in run if len(hits[i][2]) > 1) / max(1, len(run))
            dmax = max(hits[i][2][0][1] for i in run)
            arrow = (f'road {ra} lane {la:+d} -> lane {lb:+d}' if ra == rb
                     else f'road {ra} lane {la:+d} -> road {rb} lane {lb:+d}')
            print(f'   idx {a:>4}~{b:<4} ({len(run):>3}점)  '
                  f'({hits[a][0]:.0f},{hits[a][1]:.0f})  {arrow}  '
                  f'폭합 {wsum:.2f}m  최대 {dmax:.2f}m')

        # ---- D. lane 배정 일관성 ---------------------------------------
        # 차선변경으로 분류된 구간의 lane 이동은 정상이므로 세지 않는다.
        assign = [meta[h[2][0][0]] if h[2] else None for h in hits]
        seq = []
        for a in assign:
            if a is None:
                continue
            if not seq or seq[-1][0] != a:
                seq.append([a, 1])
            else:
                seq[-1][1] += 1
        jumps = []
        for i in range(1, len(assign)):
            a, b = assign[i - 1], assign[i]
            if a is None or b is None or a == b:
                continue
            if i in in_run or (i - 1) in in_run:
                continue                        # 차선변경 중
            (ra, la, sa), (rb, lb, sb) = a, b
            if ra != rb:
                continue                        # 도로가 바뀌는 건 3단계 소관
            # 왼쪽 차로(id>0)는 reference 반대 방향으로 달리므로 섹션 번호가
            # 줄어든다. 그때는 successor 가 아니라 predecessor 를 따라가야 한다.
            if sb == sa + 1:
                lane = sections[ra][sa].lanes.get(la)
                if lane is not None and lane.successor == lb:
                    continue
            if sb == sa - 1:
                lane = sections[ra][sa].lanes.get(la)
                if lane is not None and lane.predecessor == lb:
                    continue
            if la == lb:
                continue
            # 같은 부호에서 |id| 가 1 차이면 옆 차로다. 차선변경으로 설명된다.
            if la * lb > 0 and abs(abs(la) - abs(lb)) == 1:
                cls['인접 차로 이동'] += 1
                continue
            kind = '중앙선 넘음' if la * lb < 0 else '건너뜀'
            jumps.append((i, ra, la, sa, lb, sb, kind))
        print(f'D. lane 배정   구간 {len(seq)}개, '
              f'인접 차로 이동 {cls["인접 차로 이동"]}회, '
              f'설명 안 되는 이동 {len(jumps)}회'
              f'   [{"합격" if not jumps else "확인"}]')
        for i, ra, la, sa, lb, sb, kind in jumps[:6]:
            print(f'      idx {i}  road {ra}  lane {la:+d}(s{sa}) -> {lb:+d}(s{sb})'
                  f'  {kind}')
        head = ' -> '.join(f'{r}:{l:+d}' for (r, l, _s), _c in seq[:8])
        print(f'   앞부분: {head}{" ..." if len(seq) > 8 else ""}')

    check_boundaries(roads, sections)


# ----------------------------------------------------------------------

def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    data = os.path.normpath(os.path.join(here, '..', 'data'))

    ap = argparse.ArgumentParser(description='xodr lane 구조 -> 차로 중심선 CSV')
    ap.add_argument('--xodr', default=os.path.join(data, 'HL_FMA_VTD_LivingLab.xodr'))
    ap.add_argument('--out', default=os.path.join(data, 'xodr_lanes.csv'))
    ap.add_argument('--step', type=float, default=SAMPLE_STEP, help='점 간격 [m]')
    ap.add_argument('--road', help='이 road id 하나만 뽑는다 (디버그)')
    ap.add_argument('--validate', action='store_true',
                    help='녹화 궤적 3종과 대조해 판정 리포트를 낸다')
    ap.add_argument('--targets', nargs='*', default=None)
    args = ap.parse_args()

    if not os.path.exists(args.xodr):
        raise SystemExit(f'{args.xodr}: 없다')

    t0 = time.time()
    roads = {r.id: r for r in X.parse_roads(args.xodr)}
    sections = parse_lanes(args.xodr)
    t_parse = time.time() - t0

    if args.road:
        if args.road not in sections:
            raise SystemExit(f'road {args.road}: 없다')
        roads = {args.road: roads[args.road]}
        sections = {args.road: sections[args.road]}

    t0 = time.time()
    lanes = build_lanes(roads, sections, args.step)
    t_build = time.time() - t0

    n_sec = sum(len(v) for v in sections.values())
    types = Counter(ln.type for secs in sections.values()
                    for sec in secs for ln in sec.lanes.values())
    n_pts = sum(len(v[3]) for v in lanes.values())
    marks = Counter()
    for _rid, _lid, _si, pts in lanes.values():
        for p in pts:
            marks[(p.mark_in[0], p.mark_in[1])] += 1

    print(f'[lane] {os.path.basename(args.xodr)}  파싱 {t_parse:.2f}s')
    print(f'[lane] road {len(sections)}개, laneSection {n_sec}개, '
          f'lane {sum(types.values())}개')
    print(f'[lane] lane type  '
          f'{"  ".join(f"{k} {v}" for k, v in types.most_common())}')
    print(f'[lane] 주행차로 중심선 {len(lanes)}개, {n_pts}점 ({t_build:.2f}s)')
    top = marks.most_common(4)
    desc = '  '.join('{}/{}:{}'.format(t or '없음', c or '없음', n)
                     for (t, c), n in top)
    print(f'[lane] 안쪽 경계선 상위  {desc}')

    n = write_csv(args.out, lanes)
    print(f'[lane] {os.path.normpath(args.out)}: {n}점')

    if args.validate:
        targets = args.targets if args.targets is not None else [
            os.path.join(data, 'waypoints_v2.csv'),
            os.path.join(data, 'waypoints_pretest1.csv'),
            os.path.join(data, 'waypoints_pretest2.csv'),
        ]
        validate(roads, sections, lanes, targets)


if __name__ == '__main__':
    main()
