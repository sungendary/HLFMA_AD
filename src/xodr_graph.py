#!/usr/bin/env python3
"""차로 단위 연결 그래프를 만들고 route CSV -> 주행 waypoint를 생성한다. 3단계.

    python xodr_graph.py --route ../data/route_pretest.csv --out ../data/route_path.csv
    python xodr_graph.py --validate --dump-graph      # 전체 진단 + 그래프 덤프
    python xodr_graph.py --load-graph --route ...     # 덤프 재사용 (xodr 파싱 생략)

2단계까지로 주행차로 중심선 2,471개가 생겼지만 **서로 이어져 있지 않다.**
본 대회 경로는 사전테스트와 좌표가 달라 녹화 waypoint를 쓸 수 없으므로,
당일 받은 route CSV(seq,x,y 8~10점)에서 연속 waypoint를 자동으로 만들어야 한다.

설계 근거 (xodr 전수 조사로 확인한 값)
- **junction 진입은 <connection><laneLink> 가 유일한 정보원이다.** 진입도로의
  junction 방향 lane <link> 는 1073건 전부 비어 있다. 진입 차로 하나가 최대 3개
  connecting road 로 갈라지므로(1개:948 2개:121 3개:4) lane link 하나로는
  표현 자체가 불가능하다. 반대로 **탈출은** connecting road 자신의 road link +
  lane link 로 완전히 기술된다
- connecting road 499개는 <connection> 에 정확히 1회씩만 등장하고
  **contactPoint 가 499개 전부 "start"** 다. 즉 항상 s=0 에서 진입해 +s 로 달린다
- 주행 방향은 차로 부호에 내재한다. lane_id<0 이면 +s, >0 이면 -s 다.
  진행 방향 끝의 링크(successor / predecessor)와 contactPoint 로 정해지는
  다음 차로 부호가 **671건 전수 일치**했다 (start->음수, end->양수). 위반 0건
- **xodr_lanes.csv 의 heading 은 reference line 접선 방향이다.** 양수 차로는
  실제 주행 헤딩이 heading+pi 다. 이 보정이 없으면 역주행 차로를 정주행으로
  오판한다 (프로토타입에서 실제로 발생했다)
- **차선변경 엣지가 없으면 탐색이 실패한다.** route_example 은 아예 실패하고
  route_pretest_2 는 737m 경로가 3,812m 우회로 나온다. 필수다

표준 라이브러리만 사용한다.
"""

from __future__ import annotations

import argparse
import csv
import heapq
import math
import os
import time
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from dataclasses import dataclass, field

import xodr_objects as O
import xodr_parser as X

SNAP_R = 25.0               # 경유지 스냅 반경 [m]
SNAP_ROADS = 3              # 경유지마다 후보로 열 도로 수
HDG_LIMIT = math.radians(75)    # 주행 방향 필터 [rad]
LC_PENALTY = 25.0           # 차선변경 비용 가산 [m 상당]
LC_SOLID_PENALTY = 200.0    # 실선 차선변경 폴백 비용 [m 상당]
LC_MIN_LEN = 8.0            # 이보다 짧은 섹션에서는 차선변경하지 않는다 [m]
LC_BLEND = 20.0             # 차선변경을 이 거리에 걸쳐 비스듬히 섞는다 [m]
SNAP_WARN = 10.0            # 경유지-선택차로 횡거리 경고 임계 [m]
GAP_NEAR = 50.0             # 끊김이 경로에 영향을 준다고 볼 거리 [m]
LEN_TOL = 0.10              # 녹화 대비 길이 허용 오차
ROAD_HIT = 0.90             # 도로 일치율 하한

ARROW_PENALTY = 0.0         # 노면 방향표시와 어긋나는 기동의 비용 가산 [m 상당]
                            # **기본 0 = 끔.** 검증된 3단계 경로를 바꾸지 않는다
STRAIGHT_LIMIT = math.radians(30)    # 이 안이면 직진으로 본다
UTURN_LIMIT = math.radians(150)      # 이 밖이면 유턴으로 본다
ZONE_LEAD = 30.0            # 보호구역 진입을 이만큼 앞당긴다 [m]
ZONE_TAIL = 30.0            # 50 표시로 닫히지 않은 보호구역의 꼬리 [m]

Node = tuple[str, int, int]     # (road_id, section, lane_id)


# ----------------------------------------------------------------------
# 파싱 -- junction 과 road link 만. lane 구조는 2단계 CSV 를 재사용한다
# ----------------------------------------------------------------------

def parse_junctions(root: ET.Element) -> dict[tuple[str, str, int], list[tuple[str, int, str]]]:
    """(junction_id, incoming_road, from_lane) -> [(connecting_road, to_lane, contactPoint)].

    한 진입 차로가 여러 connecting road 로 갈라지므로 값이 리스트다.
    이것이 교차로에서 직진/좌회전/우회전이 갈리는 지점이다.
    """
    out: dict[tuple[str, str, int], list[tuple[str, int, str]]] = defaultdict(list)
    for j in root.findall('junction'):
        jid = j.get('id')
        for c in j.findall('connection'):
            inc = c.get('incomingRoad')
            con = c.get('connectingRoad')
            cp = c.get('contactPoint', 'start')
            for lk in c.findall('laneLink'):
                out[(jid, inc, int(lk.get('from')))].append(
                    (con, int(lk.get('to')), cp))
    return dict(out)


def parse_road_links(root: ET.Element) -> dict[str, dict[str, tuple[str, str, str | None]]]:
    """road_id -> {'predecessor'|'successor': (elementType, elementId, contactPoint)}.

    elementType 이 junction 이면 contactPoint 는 없다(None).
    """
    out: dict[str, dict[str, tuple[str, str, str | None]]] = {}
    for r in root.findall('road'):
        d: dict[str, tuple[str, str, str | None]] = {}
        link = r.find('link')
        if link is not None:
            for tag in ('predecessor', 'successor'):
                e = link.find(tag)
                if e is not None:
                    d[tag] = (e.get('elementType'), e.get('elementId'),
                              e.get('contactPoint'))
        out[r.get('id')] = d
    return out


# ----------------------------------------------------------------------
# 2단계 산출물 로드
# ----------------------------------------------------------------------

@dataclass
class LanePt:
    s: float
    x: float
    y: float
    heading: float          # reference line 접선. 주행 헤딩은 travel_heading() 참고
    width: float
    mark_in: str            # laneChange 속성 (both / none / '')
    mark_in_type: str
    mark_out: str
    mark_out_type: str


BAD_HDG = math.radians(30)      # 이만큼 꺾이면 끝점 이상치로 본다


def load_lane_points(path: str) -> tuple[dict[Node, list[LanePt]], int]:
    """xodr_lanes.csv -> 노드별 점열 (s 오름차순). 끝점 이상치는 버린다.

    2단계는 laneSection 끝(s_end)을 표본에 한 번 더 추가하는데, 바로 그 지점에서
    road._geom_at(s) 가 다음 geometry 를 집어 좌표와 헤딩이 튀는 노드가
    2,408개 중 9개 있다 (road 429 / 2396, 헤딩이 88~107도 꺾인다).
    1m 간격에서 30도는 정상 곡률로 나올 수 없으므로 안전하게 걸러낼 수 있다.
    2단계 산출물은 검증이 끝났으므로 건드리지 않고 여기서 흡수한다.
    """
    raw: dict[Node, list[LanePt]] = defaultdict(list)
    with open(path, encoding='utf-8-sig', newline='') as fh:
        for r in csv.DictReader(fh):
            n = (r['road_id'], int(r['section']), int(r['lane_id']))
            raw[n].append(LanePt(
                s=float(r['s']), x=float(r['x']), y=float(r['y']),
                heading=float(r['heading']), width=float(r['width']),
                mark_in=r['mark_in_change'], mark_in_type=r['mark_in_type'],
                mark_out=r['mark_out_change'], mark_out_type=r['mark_out_type']))
    dropped = 0
    for n, pl in raw.items():
        while len(pl) >= 3 and abs((pl[-1].heading - pl[-2].heading + math.pi)
                                   % (2 * math.pi) - math.pi) > BAD_HDG:
            pl.pop()
            dropped += 1
    return dict(raw), dropped


def travel_heading(lane_id: int, heading: float) -> float:
    """실제 주행 방향. 양수 차로는 reference line 역방향으로 달린다."""
    return heading if lane_id < 0 else heading + math.pi


def span(pts: list[LanePt]) -> float:
    return abs(pts[-1].s - pts[0].s)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _exit_pt(n: Node, pts: dict[Node, list[LanePt]]) -> LanePt:
    """그 노드의 진행 방향 끝점. 양수 차로는 s가 작은 쪽이 끝이다."""
    pl = pts[n]
    return pl[-1] if n[2] < 0 else pl[0]


def maneuver(g: Graph, frm: Node, to: Node) -> str:
    """junction 엣지의 기동. 'st' / 'lt' / 'rt' / 'ut'.

    진입 차로의 진행 방향 끝 헤딩과 connecting road 의 진출 헤딩 차이로 본다.
    connecting road 자체의 진입-진출 차이만 보면 짧은 연결로에서 회전량이
    과소평가되므로 접근로 기준으로 잰다.
    """
    if frm not in g.pts or to not in g.pts:
        return 'st'
    a = travel_heading(frm[2], _exit_pt(frm, g.pts).heading)
    b = travel_heading(to[2], _exit_pt(to, g.pts).heading)
    d = _wrap(b - a)
    if abs(d) >= UTURN_LIMIT:
        return 'ut'
    if abs(d) <= STRAIGHT_LIMIT:
        return 'st'
    return 'lt' if d > 0 else 'rt'


def load_arrows(path: str) -> dict[Node, set[str]]:
    """4단계 CSV -> 노드별 허용 기동. 주행 방향 마지막 화살표만 쓴다."""
    return O.exit_arrows(O.load_objects(path))


def s_behind(node: Node, s: float, pts: list[LanePt]) -> float:
    """그 노드 안에서 s 지점까지 이미 달린 거리 (진행 방향 기준)."""
    a, b = pts[0].s, pts[-1].s
    return (s - a) if node[2] < 0 else (b - s)


# ----------------------------------------------------------------------
# 그래프
# ----------------------------------------------------------------------

@dataclass
class Edge:
    to: Node
    cost: float
    kind: str               # section / road / junction / lanechange
    solid: bool = False     # 실선을 넘는 차선변경인가


@dataclass
class Graph:
    pts: dict[Node, list[LanePt]]
    edges: dict[Node, list[Edge]] = field(default_factory=dict)
    stat: Counter = field(default_factory=Counter)
    dead: list[tuple] = field(default_factory=list)   # (kind, node, 사유)

    def out(self, n: Node) -> list[Edge]:
        return self.edges.get(n, ())


def _entry_section(rid: str, cp: str | None, nsec: dict[str, int]) -> int:
    """contactPoint 로 정해지는 진입 laneSection.

    start 면 s=0 쪽이니 0번, end 면 s=length 쪽이니 마지막 섹션이다.
    """
    return 0 if (cp or 'start') == 'start' else nsec.get(rid, 1) - 1


def build_graph(pts: dict[Node, list[LanePt]],
                sections, road_links, junctions,
                lc_penalty: float = LC_PENALTY,
                allow_solid: bool = False,
                arrows: dict[Node, set[str]] | None = None,
                arrow_penalty: float = ARROW_PENALTY) -> Graph:
    """엣지 4종을 만든다.

    방향 판정은 전부 한 규칙으로 통일한다 -- d = +1(음수 차로) / -1(양수 차로).
    진행 방향 끝의 road link 와 그 차로의 같은 방향 lane link 를 본다.

    arrow_penalty > 0 이면 노면 방향표시(4단계)와 어긋나는 junction 기동에
    비용을 더한다. **소프트 가산이지 차단이 아니다** -- 표시가 틀렸거나 없어도
    탐색이 실패하면 안 되기 때문이다. 화살표가 없는 노드는 가산 0이다.
    """
    g = Graph(pts=pts)
    nsec = {rid: len(secs) for rid, secs in sections.items()}
    E: dict[Node, list[Edge]] = defaultdict(list)

    for n in pts:
        rid, si, lid = n
        secs = sections.get(rid)
        if secs is None or si >= len(secs):
            continue
        lane = secs[si].lanes.get(lid)
        if lane is None:
            continue
        d = 1 if lid < 0 else -1
        w = span(pts[n])

        # -- 1. laneSection 경계 (도로 내부) ---------------------------
        ni = si + d
        if 0 <= ni < len(secs):
            tgt = lane.successor if d == 1 else lane.predecessor
            if tgt is not None and (rid, ni, tgt) in pts:
                E[n].append(Edge((rid, ni, tgt), w, 'section'))
                g.stat['section'] += 1
            else:
                g.stat['section 끊김'] += 1
                g.dead.append(('section', n, f'lane link {tgt}'))
        else:
            # -- 2/3. 도로 끝 -----------------------------------------
            lk = road_links.get(rid, {}).get('successor' if d == 1 else 'predecessor')
            if lk is None:
                g.stat['도로 종단'] += 1
            elif lk[0] == 'road':
                tgt = lane.successor if d == 1 else lane.predecessor
                to = (lk[1], _entry_section(lk[1], lk[2], nsec), tgt) if tgt is not None else None
                if to is not None and to in pts:
                    E[n].append(Edge(to, w, 'road'))
                    g.stat['road-road'] += 1
                else:
                    g.stat['road-road 끊김'] += 1
                    g.dead.append(('road-road', n, f'-> road {lk[1]} lane {tgt}'))
            else:
                got = 0
                moves = arrows.get(n) if (arrows and arrow_penalty > 0) else None
                for con, tl, cp in junctions.get((lk[1], rid, lid), ()):
                    to = (con, _entry_section(con, cp, nsec), tl)
                    if to in pts:
                        # w 에는 절대 섞지 않는다. w 는 407행의 목표 노드 꼬리
                        # 차감(span - s_behind)과 짝을 이루므로 페널티가 들어가면
                        # 총거리가 어긋난다. 엣지 비용에만 더한다.
                        pen = 0.0
                        if moves and maneuver(g, n, to) not in moves:
                            pen = arrow_penalty
                            g.stat['화살표 불일치'] += 1
                        E[n].append(Edge(to, w + pen, 'junction'))
                        got += 1
                if got:
                    g.stat['junction'] += 1
                else:
                    g.stat['junction 끊김'] += 1
                    g.dead.append(('junction', n, f'junction {lk[1]}'))

        # -- 4. 차선변경 ------------------------------------------------
        # 비용에 span 을 넣지 않는다. 옆 차로로 옮겨도 그 섹션의 세로 길이는
        # 한 번만 달리기 때문이다. span 을 더하면 같은 구간을 두 번 세서
        # 40m 섹션이 80m 로 계산되고, 실제로 route_example 이 +32% 부풀었다.
        if w < LC_MIN_LEN:
            continue
        for adj in (lid - 1, lid + 1):
            if adj == 0 or adj * lid < 0 or (rid, si, adj) not in pts:
                continue
            inner = n if abs(lid) < abs(adj) else (rid, si, adj)
            # 두 차로 사이 경계 = 안쪽 차로의 mark_out (= 바깥 차로의 mark_in)
            mp = pts[inner][0]
            change = mp.mark_out
            if change == 'none':
                if not allow_solid:
                    g.stat['차선변경 금지(실선)'] += 1
                    continue
                E[n].append(Edge((rid, si, adj), LC_SOLID_PENALTY,
                                 'lanechange', solid=True))
                g.stat['차선변경(실선 폴백)'] += 1
            else:
                E[n].append(Edge((rid, si, adj), lc_penalty, 'lanechange'))
                g.stat['차선변경' if change == 'both' else '차선변경(마크없음)'] += 1

    g.edges = dict(E)
    return g


# ----------------------------------------------------------------------
# 공간 인덱스와 경유지 스냅
# ----------------------------------------------------------------------

class LaneIndex:
    """차로점 5m 격자. X.Index 는 4-튜플 전용이라 노드/폭을 함께 다루려면 얇은 래퍼가 필요하다."""

    def __init__(self, pts: dict[Node, list[LanePt]]):
        self.pts = pts
        self.grid: dict[tuple[int, int], list[tuple[Node, int]]] = defaultdict(list)
        for n, pl in pts.items():
            for i, p in enumerate(pl):
                self.grid[(int(math.floor(p.x / X.CELL)),
                           int(math.floor(p.y / X.CELL)))].append((n, i))

    def near(self, px: float, py: float, radius: float):
        """반경 안에서 노드별 최근접 (거리, s, heading). 노드당 1개."""
        rc = int(math.ceil(radius / X.CELL))
        cx = int(math.floor(px / X.CELL))
        cy = int(math.floor(py / X.CELL))
        best: dict[Node, tuple[float, float, float]] = {}
        for i in range(cx - rc, cx + rc + 1):
            for j in range(cy - rc, cy + rc + 1):
                for n, k in self.grid.get((i, j), ()):
                    p = self.pts[n][k]
                    dd = math.hypot(p.x - px, p.y - py)
                    if dd <= radius and (n not in best or dd < best[n][0]):
                        best[n] = (dd, p.s, p.heading)
        return best


def snap_candidates(idx: LaneIndex, px: float, py: float, hdg: float,
                    n_roads: int = SNAP_ROADS, radius: float = SNAP_R
                    ) -> dict[Node, tuple[float, float]]:
    """경유지 -> 후보 노드 {node: (횡거리, s)}.

    **가장 가까운 차로 하나가 아니라 가장 가까운 "도로+섹션"의 driving 차로 전부**를
    연다. 주최측이 "좌회전 구간에 3차로 좌표가 주어질 수도 있으니 1차로로 주행하면
    된다"고 했으므로, 어느 차로를 쓸지는 좌표가 아니라 다익스트라가 정해야 한다.
    교차로 부근에서는 connecting road 가 더 가까울 수 있어 도로를 여러 개 연다.
    """
    near = idx.near(px, py, radius)
    if not near:
        return {}

    # 도로+섹션 단위로 묶어 가까운 순으로 상위 n_roads 개만 남긴다
    per: dict[tuple[str, int], float] = {}
    for (rid, si, _lid), (dd, _s, _h) in near.items():
        k = (rid, si)
        if k not in per or dd < per[k]:
            per[k] = dd
    keep = {k for k, _ in sorted(per.items(), key=lambda kv: kv[1])[:n_roads]}

    out: dict[Node, tuple[float, float]] = {}
    for n, (dd, s, h) in near.items():
        if (n[0], n[1]) not in keep:
            continue
        err = abs((travel_heading(n[2], h) - hdg + math.pi) % (2 * math.pi) - math.pi)
        if err > HDG_LIMIT:
            continue                    # 역주행 차로
        out[n] = (dd, s)
    return out


def leg_heading(cps: list[tuple[float, float]], k: int) -> float:
    """k번 경유지에서의 진행 방향. 앞뒤 경유지를 이어 추정한다."""
    a = max(k - 1, 0)
    b = min(k + 1, len(cps) - 1)
    if a == b:
        return 0.0
    return math.atan2(cps[b][1] - cps[a][1], cps[b][0] - cps[a][0])


# ----------------------------------------------------------------------
# 탐색
# ----------------------------------------------------------------------

@dataclass
class RouteResult:
    nodes: list[Node]
    cost: float
    solid_lc: int
    snap: list[tuple[int, Node, float]]      # (경유지 index, 선택 노드, 횡거리)
    start_s: float
    goal_s: float


def find_route(g: Graph, idx: LaneIndex, cps: list[tuple[float, float]],
               n_roads: int = SNAP_ROADS,
               pin: dict[int, set] | None = None) -> tuple[RouteResult | None, int]:
    """계층 다익스트라. 상태 = (node, k), k = 만족한 경유지 수.

    경유지 좌표는 비용에 넣지 않고 "그 도로/섹션을 지난다"는 제약으로만 쓴다.
    두 번째 반환값은 탐색이 막힌 경유지 index (성공이면 -1).

    pin은 특정 경유지의 후보를 주어진 노드 집합으로 좁힌다. 기본값 None이면
    예전과 완전히 같다. 런타임 우회 재탐색(detour.py)이 **출발 차로를 ego가
    실제로 있는 차로(와 그 옆 차로)로 묶는 데** 쓴다 ― 안 그러면 다익스트라가
    차선변경 비용을 물지 않고 출발 후보 중 가장 싼 차로를 골라, 경로가 첫
    점에서 몇 차로씩 옆으로 튄다. 좁힌 결과가 비면 제약을 버린다. 탐색을
    실패시키는 것보다 낫다.
    """
    C = [snap_candidates(idx, x, y, leg_heading(cps, k), n_roads)
         for k, (x, y) in enumerate(cps)]
    for k, allow in (pin or {}).items():
        if 0 <= k < len(C):
            keep = {n: v for n, v in C[k].items() if n in allow}
            if keep:
                C[k] = keep
    for k, c in enumerate(C):
        if not c:
            return None, k
    K = len(cps)

    dist: dict[tuple[Node, int], float] = {}
    prev: dict[tuple[Node, int], tuple[Node, int]] = {}
    pq: list[tuple[float, tuple[Node, int]]] = []
    for n, (dd, s) in C[0].items():
        # 출발 노드는 이미 지나온 부분을 빼고 남은 거리만 센다
        c0 = -s_behind(n, s, g.pts[n])
        st = (n, 1)
        if c0 < dist.get(st, math.inf):
            dist[st] = c0
            heapq.heappush(pq, (c0, st))

    reached = 1
    while pq:
        d, state = heapq.heappop(pq)
        if d > dist.get(state, math.inf) + 1e-9:
            continue
        u, k = state
        reached = max(reached, k)
        if k == K:
            # 도착 노드는 경유지 s 까지만 센다
            gs = C[K - 1][u][1]
            total = d - (span(g.pts[u]) - s_behind(u, gs, g.pts[u]))
            nodes = []
            cur = state
            while cur in prev:
                nodes.append(cur[0])
                cur = prev[cur]
            nodes.append(cur[0])
            nodes.reverse()
            solid = _count_solid(g, nodes)
            snap = _snap_report(C, nodes)
            return RouteResult(nodes, total, solid, snap,
                               C[0].get(nodes[0], (0.0, 0.0))[1], gs), -1
        for e in g.out(u):
            nk = k + 1 if (k < K and e.to in C[k]) else k
            nd = d + e.cost
            ns = (e.to, nk)
            if nd < dist.get(ns, math.inf):
                dist[ns] = nd
                prev[ns] = state
                heapq.heappush(pq, (nd, ns))
    return None, reached


def _count_solid(g: Graph, nodes: list[Node]) -> int:
    n = 0
    for a, b in zip(nodes, nodes[1:]):
        for e in g.out(a):
            if e.to == b and e.kind == 'lanechange' and e.solid:
                n += 1
                break
    return n


def _snap_report(C, nodes: list[Node]) -> list[tuple[int, Node, float]]:
    """경유지마다 실제로 선택된 노드와 그 횡거리."""
    out = []
    k = 0
    for n in nodes:
        while k < len(C) and n in C[k]:
            out.append((k, n, C[k][n][0]))
            k += 1
    return out


# ----------------------------------------------------------------------
# waypoint 생성
# ----------------------------------------------------------------------

@dataclass
class Waypoint:
    seg: str
    x: float
    y: float
    heading: float
    road_id: str
    lane_id: int
    section: int
    s: float
    width: float
    lc_dir: str = ''
    lc_solid: int = 0


def _is_lc(a: Node, b: Node) -> bool:
    """두 노드가 같은 laneSection 안의 옆 차로인가 (= 차선변경 엣지)."""
    return a[0] == b[0] and a[1] == b[1] and a[2] != b[2]


def _by_s(pl: list[LanePt]) -> dict[float, LanePt]:
    return {round(p.s, 3): p for p in pl}


def _u(lane_id: int, s: float) -> float:
    """진행 방향으로 단조 증가하는 좌표. 양수 차로는 s가 줄어드는 쪽으로 달린다."""
    return s if lane_id < 0 else -s


def path_to_waypoints(g: Graph, res: RouteResult,
                      cps: list[tuple[float, float]]) -> tuple[list[Waypoint], list[tuple]]:
    """노드 열 -> 연속 waypoint. 두 번째 반환값은 차선변경 메타데이터.

    핵심은 **차선변경 구간을 쪼개는 것**이다. 차선변경 엣지는 같은 laneSection
    안에서 옆 차로로 옮기므로, 두 노드의 점열을 그대로 이어붙이면 앞 차로를
    끝까지 달린 뒤 옆 차로 시작점으로 되돌아가는 역주행 경로가 나온다.
    남은 구간의 중간에서 넘어가도록 s 범위를 나눠 단조 증가를 보장한다.

    진행 방향은 _u() 로 단조 증가 좌표로 바꿔 부호를 한 번만 신경 쓴다.
    """
    nodes = res.nodes
    seg_at: dict[Node, int] = {}
    for k, n, _d in res.snap:
        seg_at.setdefault(n, k)

    # 각 노드에서 달릴 u 구간을 먼저 정한다
    ranges: list[tuple[float, float]] = []
    u_from = _u(nodes[0][2], res.start_s)
    for i, n in enumerate(nodes):
        lid = n[2]
        pl = g.pts[n]
        u_lo = min(_u(lid, pl[0].s), _u(lid, pl[-1].s))
        u_hi = max(_u(lid, pl[0].s), _u(lid, pl[-1].s))
        if i == 0:
            u_from = min(max(u_from, u_lo), u_hi)
        elif _is_lc(nodes[i - 1], n):
            u_from = min(max(u_from, u_lo), u_hi)   # 차선변경: 넘어간 지점부터
        else:
            u_from = u_lo               # 도로/섹션이 바뀌면 s가 새로 시작한다
        if i == len(nodes) - 1:
            u_to = min(max(_u(lid, res.goal_s), u_lo), u_hi)
        elif _is_lc(n, nodes[i + 1]):
            u_to = u_from + (u_hi - u_from) * 0.5      # 차선변경: 남은 구간 절반에서
        else:
            u_to = u_hi
        ranges.append((u_from, max(u_to, u_from)))
        u_from = ranges[-1][1]      # 다음 노드는 넘어간 지점부터

    lc_meta: list[tuple] = []
    wps: list[Waypoint] = []
    seg_i = 0
    for i, n in enumerate(nodes):
        rid, si, lid = n
        lo, hi = ranges[i]
        pl = [p for p in g.pts[n] if lo - 1e-6 <= _u(lid, p.s) <= hi + 1e-6]
        pl.sort(key=lambda p: _u(lid, p.s))

        if n in seg_at and seg_at[n] > seg_i:
            seg_i = seg_at[n]
        j = min(seg_i, len(cps) - 2)
        seg = f'p{j + 1}_to_p{j + 2}'

        # 차선변경은 한 점에서 옆으로 3.2m 튀면 안 된다. lookahead 7m 기준
        # 조향이 0.31 rad 까지 올라가 한계(0.35)에 붙는다. 앞뒤 노드의 중심선을
        # LC_BLEND 구간에 걸쳐 섞어 비스듬히 넘어가게 한다.
        blend: list[tuple[float, dict[float, LanePt], float]] = []
        if i + 1 < len(nodes) and _is_lc(n, nodes[i + 1]):
            blend.append((ranges[i][1], _by_s(g.pts[nodes[i + 1]]), +1.0))
        if i > 0 and _is_lc(nodes[i - 1], n):
            blend.append((ranges[i][0], _by_s(g.pts[nodes[i - 1]]), -1.0))

        for p in pl:
            x, y, h = p.x, p.y, travel_heading(lid, p.heading)
            for u_mid, other, side in blend:
                half = LC_BLEND * 0.5
                w = (_u(lid, p.s) - (u_mid - half)) / LC_BLEND
                w = min(1.0, max(0.0, w))
                w = w * w * (3.0 - 2.0 * w)             # smoothstep
                if side < 0:
                    w = 1.0 - w
                q = other.get(round(p.s, 3))
                if q is None or w <= 0.0:
                    continue
                # side>0: 이 노드 -> 다음 노드로 넘어간다. side<0: 그 반대
                x += (q.x - p.x) * w
                y += (q.y - p.y) * w
                h += ((travel_heading(lid, q.heading) - h + math.pi)
                      % (2 * math.pi) - math.pi) * w
            wps.append(Waypoint(
                seg=seg, x=x, y=y, heading=h,
                road_id=rid, lane_id=lid, section=si, s=p.s, width=p.width))

        # 차선변경 지점 기록 (항목 13 방향지시등이 좌표·방향·실선을 그대로 쓴다)
        if i + 1 < len(nodes):
            nx = nodes[i + 1]
            if _is_lc(n, nx):
                solid = any(e.to == nx and e.kind == 'lanechange' and e.solid
                            for e in g.out(n))
                # 바깥(|id| 증가)으로 나가면 음수 차로는 우측, 양수 차로는 좌측이다
                outward = abs(nx[2]) > abs(lid)
                direction = ('right' if outward else 'left') if lid < 0 else                             ('left' if outward else 'right')
                if wps:
                    lc_meta.append((wps[-1].x, wps[-1].y, rid, lid, nx[2],
                                    direction, solid))
                    wps[-1].lc_dir = direction
                    wps[-1].lc_solid = 1 if solid else 0
    return wps, lc_meta


WP_HEADER = ['segment_id', 'seq', 'x', 'y', 'heading', 'road_id', 'lane_id',
             'section', 's', 'width', 'lc_dir', 'lc_solid']


def write_waypoints(path: str, wps: list[Waypoint]) -> int:
    """waypoints_*.csv 스키마 + 4단계용 컬럼.

    pure_pursuit.load_path 는 segment_id / x / y 만 읽고 DictReader 가 여분
    컬럼을 무시하므로 pure_pursuit 수정 없이 그대로 들어간다.
    BOM 없이 쓴다 (load_path 가 encoding='utf-8' 로 연다).
    """
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    seq = Counter()
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(WP_HEADER)
        for p in wps:
            i = seq[p.seg]
            seq[p.seg] += 1
            w.writerow([p.seg, i, f'{p.x:.3f}', f'{p.y:.3f}', f'{p.heading:.5f}',
                        p.road_id, p.lane_id, p.section, f'{p.s:.3f}',
                        f'{p.width:.3f}', p.lc_dir, p.lc_solid])
    return len(wps)


# ----------------------------------------------------------------------
# 경로 위 이벤트 (4단계 산출물 투영)
# ----------------------------------------------------------------------

WP_S_TOL = 2.0              # waypoint 간격이 1m 라 이 안이면 그 지점을 지났다고 본다
EV_DEDUP = 1.5              # 이 안의 같은 종류는 한 개로 본다 [m]
EV_HEADER = ['kind', 's_along', 'x', 'y', 'road_id', 'lane_id', 'detail']

EV_KINDS = ('stopline', 'crosswalk')    # 그대로 흘리는 것
ZONE_KINDS = ('speed30', 'speed50')     # 구간으로 묶는 것


def _at(track: list[tuple], c: float) -> tuple[float, float, str, int]:
    """누적거리 c 지점의 (x, y, road_id, lane_id).

    보호구역 진입·이탈은 표시 위치가 아니라 앞당기고 늦춘 지점이므로,
    그 지점이 실제로 어느 도로·차로인지는 경로에서 다시 찾아야 한다.
    """
    if c <= track[0][0]:
        return track[0][1], track[0][2], track[0][3], track[0][4]
    if c >= track[-1][0]:
        return track[-1][1], track[-1][2], track[-1][3], track[-1][4]
    lo, hi = 0, len(track) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if track[mid][0] <= c:
            lo = mid
        else:
            hi = mid
    c0, x0, y0, rid, lid = track[lo]
    c1, x1, y1, _r, _l = track[hi]
    r = (c - c0) / (c1 - c0) if c1 > c0 else 0.0
    return x0 + (x1 - x0) * r, y0 + (y1 - y0) * r, rid, lid


def route_events(objs, wps: list[Waypoint],
                 zone_lead: float = ZONE_LEAD,
                 zone_tail: float = ZONE_TAIL) -> list[tuple]:
    """경로 위 정지선·횡단보도·보호구역을 along-path 거리와 함께 뽑는다.

    경로가 실제로 지나는 (road, section, lane) 노드에 배정된 오브젝트만 본다.
    반대 차로와 주차장 진입로의 정지선은 여기서 저절로 걸러진다.
    차선변경으로 노드의 일부만 달린 경우도 s 허용오차로 걸러진다.
    """
    cum = 0.0
    table: dict[Node, list[tuple[float, float]]] = defaultdict(list)
    track: list[tuple] = []
    for i, w in enumerate(wps):
        if i:
            cum += math.hypot(w.x - wps[i - 1].x, w.y - wps[i - 1].y)
        table[(w.road_id, w.section, w.lane_id)].append((w.s, cum))
        track.append((cum, w.x, w.y, w.road_id, w.lane_id))
    if not track:
        return []

    # **마지막 노드는 waypoint 끝 너머도 본다 (정지선만).** 주최측 경유지는 대략적
    # 위치라 마지막 경유지가 종료 정지선 앞에 찍힌다 (9/10 실차: 끝 15.9m 뒤에
    # 정지선과 종료 라바콘). 그 정지선은 같은 차로 위인데 위 WP_S_TOL 검사가
    # 버려서 finish.py가 종료선을 못 잡았다. s_along = 경로 길이 + 지나친 거리로
    # 싣고 detail에 beyond_end를 붙인다. 다른 kind는 넣지 않는다.
    w_last = wps[-1]
    n_last = (w_last.road_id, w_last.section, w_last.lane_id)
    c_last = track[-1][0]

    hits: list[tuple[float, object]] = []
    beyond: list[tuple[float, object]] = []
    for o in objs:
        n = o.node
        if n is None or n not in table:
            continue
        s_wp, c = min(table[n], key=lambda r: abs(r[0] - o.s))
        if abs(s_wp - o.s) > WP_S_TOL:
            if (o.kind == 'stopline' and n == n_last
                    and _u(w_last.lane_id, o.s) > _u(w_last.lane_id, w_last.s)):
                beyond.append((c_last + abs(o.s - w_last.s), o))
            continue            # 그 노드에서 달리지 않은 구간에 있다
        hits.append((c, o))
    hits.sort(key=lambda r: r[0])
    beyond.sort(key=lambda r: r[0])
    beyond_ids = {id(o) for _c, o in beyond}
    hits.extend(beyond)

    # 지도에 같은 정지선이 t 만 몇 cm 다르게 두 번 그려진 곳이 있다
    # (road 420 lane -1 s=56.218 에 t=-1.6 / -1.5 두 개). 하나로 본다.
    dedup: list[tuple[float, object]] = []
    last: dict[str, float] = {}
    for c, o in hits:
        if o.kind in last and c - last[o.kind] < EV_DEDUP:
            continue
        last[o.kind] = c
        dedup.append((c, o))
    hits = dedup

    out: list[tuple] = []
    for c, o in hits:
        if o.kind in EV_KINDS:
            detail = f'ctrl={o.ctrl_id}' if o.ctrl_id else ''
            if id(o) in beyond_ids:
                # 공백 구분 -- stoplines.parse_ctrl이 split()[0]을 쓴다
                detail = (detail + ' beyond_end').strip()
            out.append((o.kind, c, o.x, o.y, o.road_id, o.lane_id, detail))

    # -- 보호구역: 표시 하나하나가 아니라 구간으로 묶는다 ------------------
    # 노면 표시는 구역 안에 주기적으로 그려져 있고 RM_517_50 이 50km/h 복귀
    # 지점이다. 붉은 구역 폴리곤이 아니므로 경계는 **보수적으로** 잡는다 --
    # 진입은 앞당기고 이탈은 늦춘다. 제한속도 이하 저속 주행은 감점이 없어서
    # 그 방향 오차는 안전하지만 반대 방향은 항목 2 감점이다.
    zone = [(c, o) for c, o in hits if o.kind in ZONE_KINDS]
    open_c = None
    marks = 0
    last30 = 0.0
    for c, o in zone:
        if o.kind == 'speed30':
            if open_c is None:
                open_c = max(0.0, c - zone_lead)
                marks = 0
            marks += 1
            last30 = c
        elif open_c is not None:
            x, y, rid, lid = _at(track, open_c)
            out.append(('speed30_enter', open_c, x, y, rid, lid,
                        f'marks={marks} lead={zone_lead:.0f}'))
            x, y, rid, lid = _at(track, c)
            out.append(('speed30_exit', c, x, y, rid, lid,
                        'closed_by=RM_517_50'))
            open_c = None
    if open_c is not None:
        end = min(track[-1][0], last30 + zone_tail)
        x, y, rid, lid = _at(track, open_c)
        out.append(('speed30_enter', open_c, x, y, rid, lid,
                    f'marks={marks} lead={zone_lead:.0f}'))
        x, y, rid, lid = _at(track, end)
        out.append(('speed30_exit', end, x, y, rid, lid,
                    f'closed_by=tail tail={zone_tail:.0f}'))

    out.sort(key=lambda r: (r[1], r[0]))
    return out


def write_route_events(path: str, evs: list[tuple]) -> int:
    """route_path.csv 스키마는 pure_pursuit 가 읽으므로 건드리지 않고 따로 낸다."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(EV_HEADER)
        for kind, c, x, y, rid, lid, detail in evs:
            w.writerow([kind, f'{c:.1f}', f'{x:.3f}', f'{y:.3f}',
                        rid, lid, detail])
    return len(evs)


# ----------------------------------------------------------------------
# 그래프 덤프 / 로드
# ----------------------------------------------------------------------

NODE_HEADER = ['road_id', 'section', 'lane_id', 'length', 'x0', 'y0', 'x1', 'y1']
EDGE_HEADER = ['from_road', 'from_section', 'from_lane',
               'to_road', 'to_section', 'to_lane', 'kind', 'cost', 'solid']


def write_graph(node_path: str, edge_path: str, g: Graph) -> tuple[int, int]:
    os.makedirs(os.path.dirname(os.path.abspath(node_path)), exist_ok=True)
    with open(node_path, 'w', encoding='utf-8', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(NODE_HEADER)
        for (rid, si, lid), pl in sorted(g.pts.items()):
            w.writerow([rid, si, lid, f'{span(pl):.3f}',
                        f'{pl[0].x:.3f}', f'{pl[0].y:.3f}',
                        f'{pl[-1].x:.3f}', f'{pl[-1].y:.3f}'])
    n_e = 0
    with open(edge_path, 'w', encoding='utf-8', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(EDGE_HEADER)
        for n in sorted(g.edges):
            for e in g.edges[n]:
                w.writerow([n[0], n[1], n[2], e.to[0], e.to[1], e.to[2],
                            e.kind, f'{e.cost:.3f}', 1 if e.solid else 0])
                n_e += 1
    return len(g.pts), n_e


def load_graph(edge_path: str, pts: dict[Node, list[LanePt]]) -> Graph:
    """덤프한 엣지를 읽어 그래프를 복원한다. xodr 파싱을 건너뛴다."""
    g = Graph(pts=pts)
    E: dict[Node, list[Edge]] = defaultdict(list)
    with open(edge_path, encoding='utf-8-sig', newline='') as fh:
        for r in csv.DictReader(fh):
            a = (r['from_road'], int(r['from_section']), int(r['from_lane']))
            b = (r['to_road'], int(r['to_section']), int(r['to_lane']))
            E[a].append(Edge(b, float(r['cost']), r['kind'], r['solid'] == '1'))
            g.stat[r['kind']] += 1
    g.edges = dict(E)
    return g


# ----------------------------------------------------------------------
# route CSV
# ----------------------------------------------------------------------

def load_route(path: str) -> list[tuple[float, float]]:
    with open(path, encoding='utf-8-sig', newline='') as fh:
        rows = list(csv.DictReader(fh))
    rows.sort(key=lambda r: int(r['seq']))
    return [(float(r['x']), float(r['y'])) for r in rows]


def run_route(g: Graph, idx: LaneIndex, route_path: str, out_path: str | None,
              n_roads: int, quiet: bool = False, objs=None,
              events_out: str | None = None):
    """route CSV -> waypoint. 실선 폴백은 호출부(main)에서 그래프를 바꿔 재시도한다."""
    cps = load_route(route_path)
    res, stuck = find_route(g, idx, cps, n_roads)
    if res is None:
        return None, cps, stuck
    wps, lc = path_to_waypoints(g, res, cps)
    if out_path:
        write_waypoints(out_path, wps)
    if not quiet:
        _report_route(route_path, out_path, cps, res, wps, lc)
    if events_out and objs:
        evs = route_events(objs, wps)
        write_route_events(events_out, evs)
        if not quiet:
            _report_events(events_out, evs)
    return res, cps, -1


def _report_events(path: str, evs: list[tuple]) -> None:
    kinds = Counter(e[0] for e in evs)
    n_beyond = sum(1 for e in evs if 'beyond_end' in e[6])
    print(f'[graph] {os.path.normpath(path)}: 이벤트 {len(evs)}개  ' +
          '  '.join(f'{k} {v}' for k, v in sorted(kinds.items()))
          + (f'  (경로 끝 너머 정지선 {n_beyond}개 ― 종료선 후보)' if n_beyond else
             '  (경로 끝 너머 정지선 없음 ― finish.py --plan 확인)'))
    for kind, c, x, y, rid, lid, detail in evs:
        if kind == 'crosswalk':
            continue                # 개수가 많아 요약에서는 뺀다
        print(f'[graph]   {c:7.1f} m  {kind:<14} road {rid:<6} lane {lid:+d}  '
              f'({x:.1f},{y:.1f})  {detail}')


def _report_route(route_path, out_path, cps, res: RouteResult, wps, lc) -> None:
    dist = sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(wps, wps[1:]))
    print(f'[graph] {os.path.basename(route_path)}: 경유지 {len(cps)}개  '
          f'노드 {len(res.nodes)}개  waypoint {len(wps)}점  실거리 {dist:.1f} m')
    if out_path:
        print(f'[graph] {os.path.normpath(out_path)}')

    # 경유지 스냅 품질 -- 도로를 잘못 잡으면 여기서 드러난다
    bad = [(k, n, d) for k, n, d in res.snap if d > SNAP_WARN]
    print(f'[graph] 경유지 스냅  ' + '  '.join(
        f'p{k+1}:{d:.1f}m' for k, _n, d in res.snap))
    for k, n, d in bad:
        print(f'[graph] 경고: p{k+1} 선택 차로가 {d:.1f} m 떨어졌다 '
              f'(road {n[0]} lane {n[2]}) -- 도로를 잘못 잡았을 수 있다')

    if lc:
        print(f'[graph] 차선변경 {len(lc)}회')
        for x, y, rid, a, b, direction, solid in lc:
            print(f'[graph]   ({x:.1f},{y:.1f}) road {rid:<6} lane {a:+d} -> {b:+d}  '
                  f'{direction:<5} {"solid" if solid else "broken"}')
    if res.solid_lc:
        print(f'[graph] 경고: 실선 차로변경 {res.solid_lc}회 포함 -- '
              f'평가항목 6 경미 -3점 예상')


# ----------------------------------------------------------------------
# 검증
# ----------------------------------------------------------------------

def _traj_length(rows) -> tuple[float, dict[str, float]]:
    """녹화 궤적의 길이. **구간별로 재서 더한다.**

    waypoints_v2.csv 는 파일 안에서 구간 순서가 섞여 있다(p4_to_p5 가 p3_to_p4 보다
    먼저 나온다). 전체를 연속으로 이으면 구간 사이 점프가 끼어 802.8m 가
    1596.8m 로 두 배 가까이 부풀어 오른다.
    """
    seg: dict[str, list[tuple[float, float]]] = defaultdict(list)
    for r in rows:
        seg[r.get('segment_id', '')].append((float(r['x']), float(r['y'])))
    per = {k: sum(math.dist(a, b) for a, b in zip(v, v[1:])) for k, v in seg.items()}
    return sum(per.values()), per


ROUTES = ('route_example.csv', 'route_pretest.csv', 'route_pretest_2.csv')
TRAJ = {'route_example.csv': 'waypoints_v2.csv',
        'route_pretest.csv': 'waypoints_pretest1.csv',
        'route_pretest_2.csv': 'waypoints_pretest2.csv'}


def _hit_rates(idx: LaneIndex, rows, res: RouteResult) -> tuple[float, float]:
    """녹화 궤적 대비 (도로 일치율, 차로 일치율)."""
    used = {n[0] for n in res.nodes}
    used_lane = {(n[0], n[2]) for n in res.nodes}
    hit = lane_hit = 0
    for r in rows:
        near_ = idx.near(float(r['x']), float(r['y']), 6.0)
        if any(n[0] in used for n in near_):
            hit += 1
        if near_:
            nb = min(near_.items(), key=lambda kv: kv[1][0])[0]
            if (nb[0], nb[2]) in used_lane:
                lane_hit += 1
    return hit / len(rows), lane_hit / len(rows)


def _n_lc(res: RouteResult) -> int:
    return sum(1 for a, b in zip(res.nodes, res.nodes[1:]) if _is_lc(a, b))


def compare_arrows(pts, sections, road_links, junctions, idx: LaneIndex,
                   data: str, n_roads: int, lc_penalty: float,
                   arrows: dict[Node, set[str]], penalty: float) -> None:
    """방향표시 페널티 OFF/ON 을 나란히 잰다.

    **차로 일치율만 보면 안 된다.** 일치율이 올라도 거리가 늘거나 차선변경이
    늘면 득실을 따져야 한다 -- 차선변경은 항목 6(실선 -3점)과 항목 13
    (방향지시등) 위험을, 거리는 완주 시간을 늘린다.
    """
    print('\n' + '=' * 78)
    print(f'E. 방향표시 페널티 A/B (OFF vs ON={penalty:.0f})')
    print('=' * 78)

    cols = {}
    for tag, pen in (('OFF', 0.0), ('ON', penalty)):
        gg = build_graph(pts, sections, road_links, junctions, lc_penalty,
                         arrows=arrows, arrow_penalty=pen)
        gs = build_graph(pts, sections, road_links, junctions, lc_penalty,
                         allow_solid=True, arrows=arrows, arrow_penalty=pen)
        per = {}
        for f in ROUTES:
            rp = os.path.join(data, f)
            tp = os.path.join(data, TRAJ[f])
            if not (os.path.exists(rp) and os.path.exists(tp)):
                continue
            cps = load_route(rp)
            res, _stuck = find_route(gg, idx, cps, n_roads)
            use = gg
            if res is None:
                res, _stuck = find_route(gs, idx, cps, n_roads)
                use = gs
            if res is None:
                per[f] = None
                continue
            wps, _lc = path_to_waypoints(use, res, cps)
            dist = sum(math.hypot(b.x - a.x, b.y - a.y)
                       for a, b in zip(wps, wps[1:]))
            with open(tp, encoding='utf-8-sig', newline='') as fh:
                rows = list(csv.DictReader(fh))
            road_r, lane_r = _hit_rates(idx, rows, res)
            per[f] = (dist, _n_lc(res), res.solid_lc, road_r, lane_r)
        cols[tag] = per
        if pen > 0:
            print(f'   페널티 적용 엣지 {gg.stat.get("화살표 불일치", 0)}개')

    print(f'\n   {"":<22}{"페널티 OFF":^34}{"페널티 ON":^34}')
    print(f'   {"":<22}' + 2 * f'{"거리":>10}{"LC":>5}{"실선":>5}{"도로":>7}{"차로":>7}')
    for f in ROUTES:
        a, b = cols['OFF'].get(f), cols['ON'].get(f)
        if a is None or b is None:
            print(f'   {f:<22} 탐색 실패')
            continue
        line = f'   {f:<22}'
        for v in (a, b):
            line += (f'{v[0]:>9.1f}m{v[1]:>5}{v[2]:>5}'
                     f'{v[3]*100:>6.1f}%{v[4]*100:>6.1f}%')
        print(line)
    print('\n   LC = 차선변경 횟수, 실선 = 그중 실선 변경(항목 6 -3점)')
    print('   차로 일치율이 올라도 거리·LC 가 늘면 득실을 따져야 한다.')
    print('   **기본값은 OFF 다.** 이 표를 보고 켤지 정한다.')


def validate(g: Graph, idx: LaneIndex, roads, data: str,
             n_roads: int, g_solid: Graph | None = None) -> None:
    routes = [os.path.join(data, f) for f in ROUTES]
    traj = TRAJ

    all_cps = []
    for p in routes:
        if os.path.exists(p):
            all_cps.extend(load_route(p))

    # ---- B. route 전 구간 탐색 ---------------------------------------
    print('\n' + '=' * 78)
    print('B. route 전 구간 탐색')
    print('=' * 78)
    results = {}
    ok_b = True
    for p in routes:
        name = os.path.basename(p)
        if not os.path.exists(p):
            print(f'   [건너뜀] {name} 없음')
            continue
        cps = load_route(p)
        t0 = time.time()
        res, stuck = find_route(g, idx, cps, n_roads)
        used_solid = False
        if res is None and g_solid is not None:
            res, stuck = find_route(g_solid, idx, cps, n_roads)
            used_solid = res is not None
        if res is None:
            ok_b = False
            print(f'   {name:<22} 실패 -- 경유지 p{stuck + 1} 에서 막혔다  '
                  f'({cps[stuck][0]:.1f},{cps[stuck][1]:.1f})')
            cand = snap_candidates(idx, *cps[stuck], leg_heading(cps, stuck), n_roads)
            for n, (dd, _s) in sorted(cand.items(), key=lambda kv: kv[1][0])[:5]:
                print(f'      후보 road {n[0]} lane {n[2]} s{n[1]}  {dd:.1f} m')
        else:
            gg = g_solid if used_solid else g
            wps, _lc = path_to_waypoints(gg, res, cps)
            dist = sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(wps, wps[1:]))
            results[name] = (res, wps, dist, gg)
            note = '  (실선 폴백)' if used_solid else ''
            print(f'   {name:<22} {dist:8.1f} m  노드 {len(res.nodes):>3}개  '
                  f'{time.time() - t0:.2f}s{note}')
    print(f'   [{"합격" if ok_b and len(results) == len(routes) else "불합격"}]')

    # ---- A. 끊김 목록 -------------------------------------------------
    # B 를 먼저 돌린 뒤에 본다. 끊긴 노드가 실제로 선택된 경로 위에 있는지가
    # "이 끊김이 문제인가"의 진짜 판정 기준이기 때문이다.
    print('\n' + '=' * 78)
    print('A. 연결 끊김')
    print('=' * 78)
    on_path = {n for res, _w, _d, _g in results.values() for n in res.nodes}
    # 경로 위 끊김이라도 그 직후 차선변경으로 빠져나갔다면 정상이다.
    # 합류차로처럼 끝나는 차로는 원래 옆으로 비켜야 한다.
    escaped = set()
    for res, _w, _d, _g in results.values():
        for a, b in zip(res.nodes, res.nodes[1:]):
            if _is_lc(a, b):
                escaped.add(a)
    used_roads = {n[0] for n in on_path}
    near, far, on = [], 0, []
    for kind, n, why in g.dead:
        pl = g.pts[n]
        ex = pl[-1] if n[2] < 0 else pl[0]      # 진행 방향 끝점
        d = min((math.hypot(ex.x - cx, ex.y - cy) for cx, cy in all_cps),
                default=math.inf)
        if n in on_path:
            on.append((d, kind, n, why, ex.x, ex.y))
        elif d <= GAP_NEAR:
            near.append((d, kind, n, why, ex.x, ex.y))
        else:
            far += 1
    near.sort()
    on.sort()
    print(f'   총 {len(g.dead)}건')
    stuck_on = [r for r in on if r[2] not in escaped]
    print(f'   - 선택된 경로 위          {len(on):>3}건  '
          f'(차선변경으로 빠져나감 {len(on) - len(stuck_on)}건 / '
          f'막힘 {len(stuck_on)}건)')
    print(f'   - 경유지 {GAP_NEAR:.0f}m 이내(경로 밖) {len(near):>3}건   '
          f'같은 도로의 다른 차로라 우회로 흡수된다')
    print(f'   - 그 밖                   {far:>3}건   주차장 진입로 등 경로와 무관')
    for d, kind, n, why, x, y in on:
        tag = '차선변경 회피' if n in escaped else '막힘'
        print(f'      [경로 위/{tag}] {kind:<9} road {n[0]:<6} lane {n[2]:+d} s{n[1]}  '
              f'({x:.1f},{y:.1f})   {why}')
    for d, kind, n, why, x, y in near[:10]:
        tag = '경로 도로' if n[0] in used_roads else '무관 도로'
        print(f'      {kind:<10} road {n[0]:<6} lane {n[2]:+d} s{n[1]}  '
              f'({x:.1f},{y:.1f})  경유지 {d:5.1f}m  {tag}   {why}')
    print(f'   [{"합격" if not stuck_on else "불합격"} '
          f'경로 위에서 막힌 끊김 0건이어야 한다]')

    # ---- C. 녹화 궤적 대조 --------------------------------------------
    print('\n' + '=' * 78)
    print('C. 녹화 궤적 대조')
    print('=' * 78)
    ok_c = True
    for name, (res, wps, dist, _gg) in results.items():
        tp = os.path.join(data, traj[name])
        if not os.path.exists(tp):
            print(f'   [건너뜀] {traj[name]} 없음')
            continue
        with open(tp, encoding='utf-8-sig', newline='') as fh:
            rows = list(csv.DictReader(fh))
        rec, _per = _traj_length(rows)
        err = (dist - rec) / rec

        rate, lane_rate = _hit_rates(idx, rows, res)
        good = abs(err) <= LEN_TOL and rate >= ROAD_HIT
        ok_c &= good
        print(f'   {name:<22} 탐색 {dist:7.1f} m / 녹화 {rec:7.1f} m  '
              f'오차 {err*100:+6.1f}%   도로 {rate*100:5.1f}%  '
              f'차로 {lane_rate*100:5.1f}%  [{"합격" if good else "불합격"}]')
    print(f'   [기준 길이 오차 ±{LEN_TOL*100:.0f}% 이내, 도로 일치율 {ROAD_HIT*100:.0f}% 이상]')
    print('   차로 일치율은 판정 기준이 아니다. 비용 최소 규칙상 경유지 좌표가')
    print('   가리키는 도로만 맞추고 차로는 다익스트라가 고르기 때문이다.')
    print('   낮으면 4단계 RM_537 차로별 진행방향 표시로 보정할 대상이라는 뜻이다.')

    # ---- D. 경유지 스냅 품질 ------------------------------------------
    print('\n' + '=' * 78)
    print('D. 경유지 스냅 품질')
    print('=' * 78)
    ok_d = True
    for name, (res, _wps, _dist, _gg) in results.items():
        ds = [d for _k, _n, d in res.snap]
        bad = [(k, n, d) for k, n, d in res.snap if d > SNAP_WARN]
        ok_d &= not bad
        print(f'   {name:<22} 최대 {max(ds):5.1f} m  평균 {sum(ds)/len(ds):5.1f} m  '
              f'>{SNAP_WARN:.0f}m {len(bad)}건  [{"합격" if not bad else "확인"}]')
        for k, n, d in bad:
            print(f'      p{k+1}  road {n[0]} lane {n[2]}  {d:.1f} m')

    print('\n' + '=' * 78)
    print(f'종합  A {"합격" if not stuck_on else "불합격"}  '
          f'B {"합격" if ok_b else "불합격"}  '
          f'C {"합격" if ok_c else "불합격"}  D {"합격" if ok_d else "확인"}')
    print('=' * 78)


# ----------------------------------------------------------------------

def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    data = os.path.normpath(os.path.join(here, '..', 'data'))

    ap = argparse.ArgumentParser(description='xodr 차로 연결 그래프 -> 경로 탐색')
    ap.add_argument('--xodr', default=os.path.join(data, 'HL_FMA_VTD_LivingLab.xodr'))
    ap.add_argument('--lanes', default=os.path.join(data, 'xodr_lanes.csv'),
                    help='2단계 산출물')
    ap.add_argument('--route', help='경유지 CSV (seq,x,y)')
    ap.add_argument('--out', help='생성할 waypoint CSV')
    ap.add_argument('--validate', action='store_true', help='전체 진단 리포트')
    ap.add_argument('--dump-graph', action='store_true', help='그래프를 CSV로 저장')
    ap.add_argument('--load-graph', action='store_true',
                    help='덤프한 그래프를 읽어 xodr 파싱을 건너뛴다')
    ap.add_argument('--nodes-csv', default=os.path.join(data, 'xodr_graph_nodes.csv'))
    ap.add_argument('--edges-csv', default=os.path.join(data, 'xodr_graph_edges.csv'))
    ap.add_argument('--lc-penalty', type=float, default=LC_PENALTY,
                    help='차선변경 비용 가산 [m 상당]')
    ap.add_argument('--snap-roads', type=int, default=SNAP_ROADS,
                    help='경유지마다 후보로 열 도로 수')
    ap.add_argument('--objects', default=os.path.join(data, 'xodr_objects.csv'),
                    help='4단계 산출물 (방향표시 / 경로 이벤트)')
    ap.add_argument('--arrow-penalty', type=float, default=ARROW_PENALTY,
                    help='노면 방향표시와 어긋나는 기동의 비용 가산 [m 상당]. '
                         '기본 0 = 끔')
    ap.add_argument('--events-out', help='경로 위 이벤트 CSV (정지선·보호구역·횡단보도)')
    args = ap.parse_args()

    if not os.path.exists(args.lanes):
        raise SystemExit(f'{args.lanes}: 없다. 먼저 xodr_lanes.py 를 돌려라')

    objs, arrows = None, None
    if os.path.exists(args.objects):
        objs = O.load_objects(args.objects)
        arrows = O.exit_arrows(objs)
        print(f'[graph] {os.path.basename(args.objects)}  오브젝트 {len(objs)}개  '
              f'방향표시 노드 {len(arrows)}개')
    elif args.arrow_penalty > 0 or args.events_out:
        raise SystemExit(f'{args.objects}: 없다. 먼저 xodr_objects.py 를 돌려라')

    t0 = time.time()
    pts, dropped = load_lane_points(args.lanes)
    t_lanes = time.time() - t0
    print(f'[graph] {os.path.basename(args.lanes)}  노드 {len(pts)}개 '
          f'({t_lanes:.2f}s)  끝점 이상치 {dropped}점 제거')

    g_solid = None
    if args.load_graph:
        if not os.path.exists(args.edges_csv):
            raise SystemExit(f'{args.edges_csv}: 없다. --dump-graph 를 먼저 돌려라')
        t0 = time.time()
        g = load_graph(args.edges_csv, pts)
        print(f'[graph] 덤프 로드 {time.time() - t0:.2f}s  '
              f'엣지 {sum(len(v) for v in g.edges.values())}개')
        roads = None
    else:
        if not os.path.exists(args.xodr):
            raise SystemExit(f'{args.xodr}: 없다')
        t0 = time.time()
        import xodr_lanes as L
        root = ET.parse(args.xodr).getroot()
        sections = L.parse_lanes(args.xodr)
        roads = {r.id: r for r in X.parse_roads(args.xodr)}
        junctions = parse_junctions(root)
        road_links = parse_road_links(root)
        t_parse = time.time() - t0
        print(f'[graph] xodr 파싱 {t_parse:.2f}s  '
              f'junction laneLink {sum(len(v) for v in junctions.values())}개')

        t0 = time.time()
        g = build_graph(pts, sections, road_links, junctions, args.lc_penalty,
                        arrows=arrows, arrow_penalty=args.arrow_penalty)
        g_solid = build_graph(pts, sections, road_links, junctions,
                              args.lc_penalty, allow_solid=True,
                              arrows=arrows, arrow_penalty=args.arrow_penalty)
        print(f'[graph] 엣지 {sum(len(v) for v in g.edges.values())}개 '
              f'({time.time() - t0:.2f}s)')
        for k, v in g.stat.most_common():
            print(f'[graph]   {k:<22} {v:>6}')

    t0 = time.time()
    idx = LaneIndex(pts)
    print(f'[graph] 공간 인덱스 {time.time() - t0:.2f}s')

    if args.dump_graph and not args.load_graph:
        n_n, n_e = write_graph(args.nodes_csv, args.edges_csv, g)
        print(f'[graph] {os.path.normpath(args.nodes_csv)}: 노드 {n_n}개')
        print(f'[graph] {os.path.normpath(args.edges_csv)}: 엣지 {n_e}개')

    if args.route:
        if not os.path.exists(args.route):
            raise SystemExit(f'{args.route}: 없다')
        res, cps, stuck = run_route(g, idx, args.route, args.out,
                                    args.snap_roads, objs=objs,
                                    events_out=args.events_out)
        if res is None and g_solid is not None:
            print(f'[graph] 경유지 p{stuck + 1} 에서 막혔다. 실선 차로변경을 열어 재시도한다')
            res, cps, stuck = run_route(g_solid, idx, args.route, args.out,
                                        args.snap_roads, objs=objs,
                                        events_out=args.events_out)
        if res is None:
            raise SystemExit(f'[graph] 탐색 실패: 경유지 p{stuck + 1} '
                             f'({cps[stuck][0]:.1f},{cps[stuck][1]:.1f})')

    if args.validate:
        if roads is None:
            print('[graph] --load-graph 상태에서는 끊김 진단(A)이 제한된다')
        validate(g, idx, roads, data, args.snap_roads, g_solid)
        if args.arrow_penalty > 0 and arrows and not args.load_graph:
            compare_arrows(pts, sections, road_links, junctions, idx, data,
                           args.snap_roads, args.lc_penalty, arrows,
                           args.arrow_penalty)
        elif args.arrow_penalty <= 0:
            print('\n[graph] 방향표시 A/B 비교: --arrow-penalty 80 처럼 값을 주면 낸다')


if __name__ == '__main__':
    main()
