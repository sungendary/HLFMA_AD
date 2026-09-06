#!/usr/bin/env python3
"""xodr의 <object>/<signal>을 뽑아 차로에 배정한다. 4단계.

    python xodr_objects.py                    # CSV 생성 + 요약
    python xodr_objects.py --validate         # + 사전테스트 로그 대조 리포트

정지선·보호구역·횡단보도·차로별 진행방향이 전부 xodr에 노면 오브젝트로 들어 있다.
주최측은 "센서로 식별하라"고 했으나 파일 내 모든 정보 사용은 허가받았으므로
카메라 인식 없이 좌표로 푼다. 신호등 정지(항목 7)는 사전테스트에서 적색 신호를
15회 이상 통과해 구간당 -6점이 걸려 있는 가장 큰 구멍이다.

설계 근거 (xodr 전수 조사로 확인한 값)
- **.flt 파일명은 name= 속성에만 있다.** type= 은 무의미하다(none 2720개).
  orientation 은 6,344개 **전부 "none"** 이라 방향 판정에 쓸 수 없다. hdg 를 쓴다
- **대상 오브젝트에는 <repeat> 가 하나도 없다.** 168개 <repeat> 는 전부 나무·펜스·
  건물이다. 즉 개수가 곧 실제 인스턴스 수이고 전개가 필요 없다
- **런타임 tl_id 는 <signal> id 가 아니라 <controller> id 다.** 로그에 찍힌
  tl_id {7,11,15,25,150,152,155,159,312} 중 _Sg7 / _Sg11 / _Sg25 는 파일에
  존재하지 않고 ctrl007 / ctrl011 / ctrl025 는 전부 있다. 체인은
  tl_id -> <controller> -> <control signalId> -> <signal> -> road+s+t 다.
  controller 당 신호 3개(210개) 또는 4개(4개) = 한 접근로의 신호등 헤드 묶음이라
  교차로 진입 방향과 1:1 대응한다. <junction><controller id=/> 231건이 교차로와 잇는다
- **<signal> 의 <positionRoad> 는 다른 도로를 가리킬 수 있다.** 신호 501 은 감싸는
  도로가 2817 인데 positionRoad roadId="2815" 다. 그건 지주(mast arm)의 물리
  위치이므로 **차로 귀속에는 감싸는 <road> 의 s/t 를 쓰고 positionRoad 는 버린다**
- **방향표시는 한 도로에서 여러 번 반복된다.** road 2817 의 ST 화살표는 s = 4.0 /
  68.7 / 97.8 / 143.8 / 146.6 / 157.6 에 있다. 교차로 기동을 규정하는 것은
  **주행 방향 기준 마지막 화살표**다 (exit_arrows)
- object 의 t 와 xodr_lanes.LaneSection.lane_t 의 반환 t 는 **둘 다 reference line
  기준이고 lane_t 는 laneOffset 을 이미 포함한다.** 그대로 비교하면 된다.
  laneOffset 을 두 번 더하지 말 것

표준 라이브러리만 사용한다.
"""

from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import re
import time
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from dataclasses import dataclass

import xodr_lanes as L
import xodr_parser as X

# .flt 이름 -> 정규화 kind. 여기 없는 오브젝트는 버린다(나무·건물·가로등 등)
KIND_BY_NAME = {
    'Rm_StopLine_300cm_JPN_01.flt':    'stopline',
    'roadmark_speed_30.flt':           'speed30',
    'RM_517_50.flt':                   'speed50',    # 보호구역 이탈 판정에 쓴다
    'Rm_Warning_Crosswalk_JPN_03.flt': 'crosswalk',
    'RM_537_ST.flt':                   'arrow_st',
    'RM_537_LT.flt':                   'arrow_lt',
    'RM_537_RT.flt':                   'arrow_rt',
    'RM_538_SLT.flt':                  'arrow_slt',
    'RM_538_SRT.flt':                  'arrow_srt',
    'RM_539_LUT.flt':                  'arrow_lut',
    'RM_539_UT.flt':                   'arrow_ut',
}

# 화살표 kind -> 허용 기동 집합. LUT 는 좌회전 및 유턴이다
ARROW_MOVES = {
    'arrow_st':  {'st'},
    'arrow_lt':  {'lt'},
    'arrow_rt':  {'rt'},
    'arrow_slt': {'st', 'lt'},
    'arrow_srt': {'st', 'rt'},
    'arrow_lut': {'lt', 'ut'},
    'arrow_ut':  {'ut'},
}

LANE_EPS = 0.20         # 차로 경계 판정 여유 [m]
ZONE_LINK = 100.0       # 보호구역 군집 단일연결 임계 [m]
NEAR_STOPLINE = 50.0    # 신호 이탈 지점 - 정지선 합격 기준 [m]
CHAIN_HIT = 30.0        # controller 체인 - 관측 궤적 합격 기준 [m]

Node = tuple[str, int, int]     # (road_id, section, lane_id) -- 3단계와 같은 키


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def travel_u(lane_id: int, s: float) -> float:
    """진행 방향으로 단조 증가하는 좌표. xodr_graph._u 와 같은 규칙이다.

    양수 차로는 reference line 을 거슬러(-s 방향으로) 달린다.
    """
    return s if lane_id < 0 else -s


# ----------------------------------------------------------------------
# 파싱
# ----------------------------------------------------------------------

@dataclass
class RawObject:
    road_id: str
    obj_id: str
    name: str
    raw_type: str
    s: float
    t: float
    hdg: float


@dataclass
class RawSignal:
    road_id: str
    sig_id: str
    s: float
    t: float
    h_offset: float
    sig_type: str
    subtype: str
    validity: list[tuple[int, int]]


def parse_objects(root: ET.Element) -> tuple[list[RawObject], Counter]:
    """KIND_BY_NAME 에 있는 <object> 만. 두 번째 반환값은 전체 이름 census."""
    out: list[RawObject] = []
    census: Counter = Counter()
    for r in root.findall('road'):
        rid = r.get('id')
        objs = r.find('objects')
        if objs is None:
            continue
        for o in objs.findall('object'):
            name = o.get('name', '')
            census[name] += 1
            if name not in KIND_BY_NAME:
                continue
            out.append(RawObject(
                road_id=rid, obj_id=o.get('id', ''), name=name,
                raw_type=o.get('type', ''),
                s=float(o.get('s', 0.0)), t=float(o.get('t', 0.0)),
                hdg=float(o.get('hdg', 0.0))))
    return out, census


def parse_signals(root: ET.Element) -> list[RawSignal]:
    """<signal> 전부. <positionRoad> 는 지주 위치라 쓰지 않는다."""
    out: list[RawSignal] = []
    for r in root.findall('road'):
        rid = r.get('id')
        sigs = r.find('signals')
        if sigs is None:
            continue
        for sg in sigs.findall('signal'):
            val = []
            for v in sg.findall('validity'):
                a, b = int(v.get('fromLane')), int(v.get('toLane'))
                val.append((min(a, b), max(a, b)))
            out.append(RawSignal(
                road_id=rid, sig_id=sg.get('id', ''),
                s=float(sg.get('s', 0.0)), t=float(sg.get('t', 0.0)),
                h_offset=float(sg.get('hOffset', 0.0)),
                sig_type=sg.get('type', ''), subtype=sg.get('subtype', ''),
                validity=val))
    return out


def parse_controllers(root: ET.Element) -> dict[str, list[str]]:
    """ctrl_id -> [signal_id]. **9910이 주는 tl_id 가 바로 이 ctrl_id 다.**"""
    out: dict[str, list[str]] = {}
    for c in root.findall('controller'):
        out[c.get('id')] = [ct.get('signalId')
                            for ct in c.findall('control')]
    return out


def parse_junction_controllers(root: ET.Element) -> dict[str, str]:
    """ctrl_id -> junction_id. <junction> 안의 <controller id=/> 참조."""
    out: dict[str, str] = {}
    for j in root.findall('junction'):
        for c in j.findall('controller'):
            out[c.get('id')] = j.get('id')
    return out


# ----------------------------------------------------------------------
# 배치 -- (s, t) -> (section, lane_id, x, y, heading)
# ----------------------------------------------------------------------

def section_index(secs: list[L.LaneSection], s: float) -> int:
    """s 를 포함하는 laneSection index. secs 는 s 오름차순이다."""
    i = 0
    for k, sec in enumerate(secs):
        if s + 1e-9 >= sec.s:
            i = k
        else:
            break
    return i


def lane_at_t(road: X.Road, sec: L.LaneSection, s: float, t: float
              ) -> tuple[int | None, float]:
    """t 가 들어앉는 driving 차로. 없으면 (None, 최근접 차로 경계까지 거리).

    lane_t 의 t 는 laneOffset 을 이미 포함하므로 object 의 t 와 직접 비교한다.
    """
    gap_best = math.inf
    for lid in L.driving_lane_ids(sec):
        c, w = sec.lane_t(road, s, lid)
        d = abs(t - c)
        if d <= w * 0.5 + LANE_EPS:
            return lid, d
        gap_best = min(gap_best, d - w * 0.5)
    return None, gap_best


@dataclass
class Placed:
    kind: str
    road_id: str
    section: int
    lane_id: int | None
    s: float
    t: float
    x: float
    y: float
    heading: float
    name: str
    raw_type: str
    sig_id: str = ''
    ctrl_id: str = ''
    junction_id: str = ''
    lane_src: str = ''

    @property
    def node(self) -> Node | None:
        if self.lane_id is None:
            return None
        return (self.road_id, self.section, self.lane_id)


def place_objects(objs: list[RawObject], roads: dict[str, X.Road],
                  sections: dict[str, list[L.LaneSection]]) -> list[Placed]:
    out: list[Placed] = []
    for o in objs:
        road = roads.get(o.road_id)
        secs = sections.get(o.road_id)
        if road is None or not secs:
            continue
        si = section_index(secs, o.s)
        lid, _gap = lane_at_t(road, secs[si], o.s, o.t)
        x, y = road.point_at(o.s, o.t)
        out.append(Placed(
            kind=KIND_BY_NAME[o.name], road_id=o.road_id, section=si,
            lane_id=lid, s=o.s, t=o.t, x=x, y=y,
            heading=_wrap(road.pose_at(o.s)[2] + o.hdg),
            name=o.name, raw_type=o.raw_type,
            lane_src='t' if lid is not None else 'none'))
    return out


def place_signals(sigs: list[RawSignal], roads, sections,
                  sig_to_ctrl: dict[str, str],
                  ctrl_to_junction: dict[str, str]) -> list[Placed]:
    """<validity> 가 있으면 차로마다 한 행, 없으면 t 로 한 행.

    646개 중 validity 가 있는 건 148개뿐이다. 나머지는 t 로 추정하되
    신호등은 차로 경계 위에 걸리는 일이 흔하므로(마스트암) lane_id 를
    맹신하지 말고 road + ctrl_id 로 쓰는 것이 맞다.
    """
    out: list[Placed] = []
    for sg in sigs:
        road = roads.get(sg.road_id)
        secs = sections.get(sg.road_id)
        if road is None or not secs:
            continue
        si = section_index(secs, sg.s)
        x, y = road.point_at(sg.s, sg.t)
        head = _wrap(road.pose_at(sg.s)[2] + sg.h_offset)
        ctrl = sig_to_ctrl.get(sg.sig_id, '')
        base = dict(kind='signal', road_id=sg.road_id, section=si,
                    s=sg.s, t=sg.t, x=x, y=y, heading=head,
                    name=f'_Sg{sg.sig_id}', raw_type=f'{sg.sig_type}/{sg.subtype}',
                    sig_id=sg.sig_id, ctrl_id=ctrl,
                    junction_id=ctrl_to_junction.get(ctrl, ''))
        lanes: list[int] = []
        for a, b in sg.validity:
            lanes.extend(range(a, b + 1))
        lanes = [v for v in dict.fromkeys(lanes) if v != 0]
        if lanes:
            for lid in lanes:
                out.append(Placed(lane_id=lid, lane_src='validity', **base))
        else:
            lid, _gap = lane_at_t(road, secs[si], sg.s, sg.t)
            out.append(Placed(lane_id=lid,
                              lane_src='t' if lid is not None else 'none',
                              **base))
    return out


LINK_R = 45.0           # 정지선-신호등을 같은 접근로로 볼 최대 거리 [m]
LINK_HDG = math.radians(30)     # 같은 접근로로 볼 최대 헤딩 차이 [rad]


def link_stopline_ctrl(placed: list[Placed]) -> tuple[int, int]:
    """정지선에 그 접근로를 관장하는 ctrl_id 를 채운다.

    9910이 주는 tl_id 가 ctrl_id 이므로, 이게 있어야 런타임에 "지금 이 신호가
    내 정지선의 신호인가"를 판정할 수 있다.

    **같은 road 로 묶으면 안 된다.** 신호는 정지선과 다른 도로에 붙어 있을 수
    있다 -- ctrl 312 의 신호 3개는 road 791 에 있는데 실제로 관장하는 정지선은
    road 190 위에 있다(24.7m 떨어져 있고 헤딩은 0.015 rad 차이). 그래서
    **거리 + 헤딩**으로 잇는다. 정지선의 hdg 와 신호의 hOffset 은 둘 다 그
    접근로의 주행 방향이므로 마주 오는 방향은 헤딩 필터에서 걸러진다.

    후보를 하나로 좁히지 않고 전부 남긴다. 한 접근로에 신호등 헤드가 둘 이상
    걸린 경우가 있고(road 2817 은 ctrl 159 와 160 의 신호가 전부 s=0 에 몰려
    있다), 9910 은 신호등을 하나만 주므로 VTD 가 둘 중 무엇을 고를지 파일만
    보고는 정할 수 없다. 런타임은 "받은 tl_id 가 이 집합에 있는가"로 판정한다.
    """
    sigs = [q for q in placed if q.kind == 'signal' and q.ctrl_id]
    # 5m 격자로 좁힌다 (X.CELL 과 같은 규칙)
    grid: dict[tuple[int, int], list[Placed]] = defaultdict(list)
    for q in sigs:
        grid[(int(q.x // X.CELL), int(q.y // X.CELL))].append(q)
    rc = int(math.ceil(LINK_R / X.CELL))

    n = multi = 0
    for p in placed:
        if p.kind != 'stopline' or p.lane_id is None:
            continue
        cx, cy = int(p.x // X.CELL), int(p.y // X.CELL)
        cand = []
        for i in range(cx - rc, cx + rc + 1):
            for j in range(cy - rc, cy + rc + 1):
                for q in grid.get((i, j), ()):
                    if math.hypot(q.x - p.x, q.y - p.y) > LINK_R:
                        continue
                    if abs(_wrap(q.heading - p.heading)) > LINK_HDG:
                        continue
                    cand.append(q)
        if not cand:
            continue
        ctrls = sorted({q.ctrl_id for q in cand}, key=int)
        p.ctrl_id = '|'.join(ctrls)
        p.junction_id = '|'.join(sorted({q.junction_id for q in cand if q.junction_id}))
        n += 1
        if len(ctrls) > 1:
            multi += 1
    return n, multi


# ----------------------------------------------------------------------
# 화살표 -> 노드
# ----------------------------------------------------------------------

def exit_arrows(placed: list[Placed]) -> dict[Node, set[str]]:
    """노드별 허용 기동. **주행 방향 마지막 화살표만** 남긴다.

    한 도로에 같은 화살표가 여러 번 반복되지만(road 2817 의 ST 는 6곳),
    교차로 기동을 규정하는 것은 진행 방향 끝에 가장 가까운 것이다.
    """
    best: dict[Node, tuple[float, set[str]]] = {}
    for p in placed:
        if p.kind not in ARROW_MOVES or p.lane_id is None:
            continue
        n = (p.road_id, p.section, p.lane_id)
        u = travel_u(p.lane_id, p.s)
        cur = best.get(n)
        if cur is None or u > cur[0] + 1e-6:
            best[n] = (u, set(ARROW_MOVES[p.kind]))
        elif abs(u - cur[0]) <= 1e-6:
            cur[1].update(ARROW_MOVES[p.kind])     # 같은 지점에 겹쳐 그린 경우
    return {n: mv for n, (_u, mv) in best.items()}


# ----------------------------------------------------------------------
# CSV
# ----------------------------------------------------------------------

CSV_HEADER = ['kind', 'road_id', 'section', 'lane_id', 's', 't', 'x', 'y',
              'heading', 'name', 'raw_type', 'sig_id', 'ctrl_id',
              'junction_id', 'lane_src']


def write_csv(path: str, placed: list[Placed]) -> int:
    """BOM 없이 쓴다. 1~3단계 산출물과 같은 규칙이다(읽을 때는 utf-8-sig)."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    rows = sorted(placed, key=lambda p: (p.kind, p.road_id, p.s, p.t))
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(CSV_HEADER)
        for p in rows:
            w.writerow([p.kind, p.road_id, p.section,
                        '' if p.lane_id is None else p.lane_id,
                        f'{p.s:.3f}', f'{p.t:.3f}', f'{p.x:.3f}', f'{p.y:.3f}',
                        f'{p.heading:.5f}', p.name, p.raw_type,
                        p.sig_id, p.ctrl_id, p.junction_id, p.lane_src])
    return len(rows)


def load_objects(path: str) -> list[Placed]:
    """4단계 산출물을 읽는다. 3단계가 재파싱 없이 화살표를 쓰기 위한 진입점."""
    out: list[Placed] = []
    with open(path, encoding='utf-8-sig', newline='') as fh:
        for r in csv.DictReader(fh):
            lid = r['lane_id']
            out.append(Placed(
                kind=r['kind'], road_id=r['road_id'], section=int(r['section']),
                lane_id=int(lid) if lid != '' else None,
                s=float(r['s']), t=float(r['t']),
                x=float(r['x']), y=float(r['y']), heading=float(r['heading']),
                name=r['name'], raw_type=r['raw_type'], sig_id=r['sig_id'],
                ctrl_id=r['ctrl_id'], junction_id=r['junction_id'],
                lane_src=r['lane_src']))
    return out


# ----------------------------------------------------------------------
# 검증
# ----------------------------------------------------------------------

TL_RE = re.compile(r'\(id (\d+)->(\d+)\)')


def _pct(vals: list[float], q: float) -> float:
    if not vals:
        return float('nan')
    v = sorted(vals)
    return v[min(len(v) - 1, max(0, int(q * len(v))))]


def _read_events(logs: str) -> list[dict]:
    out = []
    for d in sorted(glob.glob(os.path.join(logs, 'pp_20260904_*'))):
        p = os.path.join(d, 'events.csv')
        if not os.path.exists(p):
            continue
        with open(p, encoding='utf-8-sig', newline='') as fh:
            for r in csv.DictReader(fh):
                r['_run'] = os.path.basename(d)
                out.append(r)
    return out


def _nearest(pts: list[tuple[float, float]], x: float, y: float) -> float:
    return min((math.hypot(px - x, py - y) for px, py in pts), default=math.inf)


def _validate_a(placed: list[Placed], logs: str) -> bool:
    print('\n' + '=' * 78)
    print('A. 신호 전이 지점 vs 정지선 / controller 체인')
    print('=' * 78)

    stop = [(p.x, p.y) for p in placed
            if p.kind == 'stopline' and p.lane_id is not None]
    print(f'   차로에 배정된 정지선 {len(stop)}개로 대조한다')

    events = _read_events(logs)
    tl = [e for e in events if e['kind'] == 'tl_change'
          and e['ego_x'] not in ('', 'nan')]
    if not tl:
        print('   [건너뜀] tl_change 이벤트가 없다')
        return True

    enter, leave = [], []
    for e in tl:
        m = TL_RE.search(e['detail'])
        if not m:
            continue
        a, b = m.group(1), m.group(2)
        d = _nearest(stop, float(e['ego_x']), float(e['ego_y']))
        if a == '0' and b != '0':
            enter.append((d, e))
        elif b == '0' and a != '0':
            leave.append((d, e))

    for label, rows, limit in (('신규 할당 (NONE->X)', enter, None),
                               ('이탈      (X->NONE)', leave, NEAR_STOPLINE)):
        ds = [d for d, _e in rows]
        if not ds:
            continue
        over = [r for r in rows if limit is not None and r[0] > limit]
        print(f'   {label}  {len(ds):>3}건   '
              f'중앙 {_pct(ds, 0.5):5.1f} m  p95 {_pct(ds, 0.95):5.1f} m  '
              f'최대 {max(ds):5.1f} m')
        for d, e in sorted(over, reverse=True)[:5]:
            print(f'      초과 {d:6.1f} m  {e["_run"]}  {e["detail"]}  '
                  f'({float(e["ego_x"]):.1f},{float(e["ego_y"]):.1f})')
    ok_near = all(d <= NEAR_STOPLINE for d, _e in leave)
    print(f'   [{"합격" if ok_near else "불합격"} '
          f'이탈 지점의 최근접 정지선이 {NEAR_STOPLINE:.0f} m 이내여야 한다]')

    # ---- controller 체인 --------------------------------------------
    print()
    sig_xy: dict[str, list[tuple[float, float]]] = defaultdict(list)
    for p in placed:
        if p.kind == 'signal' and p.ctrl_id:
            sig_xy[p.ctrl_id].append((p.x, p.y))

    seen: dict[str, list[tuple[float, float]]] = defaultdict(list)
    for d in sorted(glob.glob(os.path.join(logs, 'pp_20260904_*'))):
        p = os.path.join(d, 'drive.csv')
        if not os.path.exists(p):
            continue
        with open(p, encoding='utf-8-sig', newline='') as fh:
            for r in csv.DictReader(fh):
                tid = r.get('tl_id', '')
                if tid and tid != '0':
                    seen[tid].append((float(r['ego_x']), float(r['ego_y'])))

    ok_chain = True
    print(f'   관측된 tl_id {len(seen)}종을 controller 체인으로 해석한다')
    for tid in sorted(seen, key=int):
        pos = sig_xy.get(tid, [])
        if not pos:
            ok_chain = False
            print(f'      tl_id {tid:>4}  해석 실패 -- ctrl {tid} 에 신호가 없다')
            continue
        d = min(_nearest(pos, x, y) for x, y in seen[tid])
        # **판정은 정지선 거리로 한다.** 궤적 거리는 판정 기준이 될 수 없다 --
        # VTD 는 다가오는 교차로의 신호를 멀리서부터 주므로, 그 교차로까지
        # 실제로 가지 않고 끝난 세션에서는 궤적이 멀 수밖에 없다.
        # 체인이 맞았는지는 "그 신호가 정지선 옆에 있는가"로 봐야 한다.
        stop_d = min(_nearest(stop, sx, sy) for sx, sy in pos)
        good = stop_d <= CHAIN_HIT
        ok_chain &= good
        print(f'      tl_id {tid:>4}  신호 {len(pos)}개  관측 {len(seen[tid]):>6}프레임  '
              f'정지선까지 {stop_d:5.1f} m [{"합격" if good else "불합격"}]  '
              f'(궤적까지 {d:5.1f} m)')
    print(f'   [{"합격" if ok_chain else "불합격"} '
          f'tl_id 전부 해석되고 신호가 정지선 {CHAIN_HIT:.0f} m 이내여야 한다]')
    return ok_near and ok_chain


def _validate_b(placed: list[Placed]) -> bool:
    print('\n' + '=' * 78)
    print('B. 보호구역(30km/h) 군집')
    print('=' * 78)
    z30 = [p for p in placed if p.kind == 'speed30']
    z50 = [p for p in placed if p.kind == 'speed50']
    print(f'   speed30 {len(z30)}개  speed50 {len(z50)}개')
    if not z30:
        return True

    # 최근접 이웃 거리 분포를 먼저 보여 임계값이 데이터에서 나오게 한다
    nn = []
    for i, a in enumerate(z30):
        nn.append(min(math.hypot(a.x - b.x, a.y - b.y)
                      for j, b in enumerate(z30) if i != j))
    print(f'   표시 간 최근접 거리  중앙 {_pct(nn, 0.5):6.1f} m  '
          f'p90 {_pct(nn, 0.9):6.1f} m  최대 {max(nn):6.1f} m  '
          f'-> 단일연결 임계 {ZONE_LINK:.0f} m')

    # 단일연결 군집 (union-find)
    parent = list(range(len(z30)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i, a in enumerate(z30):
        for j in range(i + 1, len(z30)):
            b = z30[j]
            if math.hypot(a.x - b.x, a.y - b.y) <= ZONE_LINK:
                ra, rb = find(i), find(j)
                if ra != rb:
                    parent[ra] = rb

    comp: dict[int, list[Placed]] = defaultdict(list)
    for i, p in enumerate(z30):
        comp[find(i)].append(p)

    print(f'   군집 {len(comp)}개')
    for k in sorted(comp, key=lambda k: -len(comp[k])):
        mem = comp[k]
        roads = sorted({p.road_id for p in mem}, key=int)
        cx = sum(p.x for p in mem) / len(mem)
        cy = sum(p.y for p in mem) / len(mem)
        ext = max(math.hypot(a.x - b.x, a.y - b.y) for a in mem for b in mem)
        # 군집 중심이 아니라 **구성원 기준**으로 잰다. 폭 400m 짜리 군집에서
        # 중심까지의 거리는 아무 뜻이 없다.
        near50 = min((math.hypot(a.x - b.x, a.y - b.y) for a in mem for b in z50),
                     default=float('inf'))
        print(f'      표시 {len(mem):>2}개  도로 {len(roads):>2}개  '
              f'중심 ({cx:7.1f},{cy:7.1f})  최대폭 {ext:6.1f} m  '
              f'최근접 speed50 {near50:6.1f} m')
        tail = ' ...' if len(roads) > 12 else ''
        print(f'         road {",".join(roads[:12])}{tail}')
    print('   [참고] 붉은 구역 폴리곤이 아니라 노면 표시 위치다. 경로 투영 시')
    print('          진입은 앞당기고 이탈은 늦춰 보수적으로 잡는다')
    return True


def _validate_c(placed: list[Placed], lanes_csv: str, xodr: str) -> bool:
    print('\n' + '=' * 78)
    print('C. 방향표시 vs 실제 junction 기동')
    print('=' * 78)
    if not os.path.exists(lanes_csv):
        print(f'   [건너뜀] {lanes_csv} 없음')
        return True

    import xodr_graph as G       # 순환 import 회피 -- graph 가 objects 를 쓴다

    pts, _dropped = G.load_lane_points(lanes_csv)
    root = ET.parse(xodr).getroot()
    sections = L.parse_lanes(xodr)
    g = G.build_graph(pts, sections, G.parse_road_links(root),
                      G.parse_junctions(root))

    arrows = exit_arrows(placed)
    print(f'   화살표가 붙은 노드 {len(arrows)}개')

    pattern: Counter = Counter()
    match = mismatch = no_junction = 0
    bad: list[tuple] = []
    for n, moves in sorted(arrows.items()):
        if n not in pts:
            continue
        js = [e for e in g.out(n) if e.kind == 'junction']
        if not js:
            no_junction += 1
            continue
        actual = {G.maneuver(g, n, e.to) for e in js}
        if actual & moves:
            match += 1
        else:
            mismatch += 1
            pattern[('|'.join(sorted(moves)), '|'.join(sorted(actual)))] += 1
            if len(bad) < 8:
                bad.append((n, sorted(moves), sorted(actual)))

    total = match + mismatch
    print(f'   junction 엣지가 있는 노드 {total}개 '
          f'(교차로에 닿지 않는 노드 {no_junction}개는 제외)')
    if total:
        print(f'   표시와 실제 기동이 겹치는 노드 {match}개 '
              f'({match / total * 100:.1f}%)  불일치 {mismatch}개')
    # 한 차로가 connecting road 3개로 갈라질 수 있어 실제 기동은 집합이다.
    # 교차표를 그리면 "선택지"가 오차처럼 보이므로 불일치 패턴만 센다.
    print('   불일치 패턴 (표시 -> 실제)')
    for (mv, ac), c_ in pattern.most_common(10):
        print(f'      {mv:<8} -> {ac:<8} {c_:>4}건')
    for n, moves, actual in bad:
        print(f'      예: road {n[0]:<6} lane {n[2]:+d} s{n[1]}  '
              f'표시 {"|".join(moves):<7} 실제 {"|".join(actual)}')
    ok = total > 0 and match / total >= 0.90
    print(f'   [{"합격" if ok else "확인"} 겹침 90% 이상이면 페널티 도입이 안전하다]')
    return ok


def _validate_d(placed: list[Placed], roads: dict[str, X.Road],
                road_links: dict) -> bool:
    print('\n' + '=' * 78)
    print('D. 정지선이 junction 진입부에 있는가')
    print('=' * 78)
    ds, on_junction = [], 0
    for p in placed:
        if p.kind != 'stopline' or p.lane_id is None:
            continue
        road = roads.get(p.road_id)
        if road is None:
            continue
        # 진행 방향 끝단까지의 거리. 음수 차로는 s=length 쪽, 양수 차로는 s=0 쪽
        d = (road.length - p.s) if p.lane_id < 0 else p.s
        ds.append(d)
        lk = road_links.get(p.road_id, {}).get(
            'successor' if p.lane_id < 0 else 'predecessor')
        if lk is not None and lk[0] == 'junction':
            on_junction += 1
    if not ds:
        print('   [건너뜀] 차로에 배정된 정지선이 없다')
        return False
    print(f'   차로 배정 정지선 {len(ds)}개')
    print(f'   진행 방향 끝단까지  중앙 {_pct(ds, 0.5):6.1f} m  '
          f'p90 {_pct(ds, 0.9):6.1f} m  최대 {max(ds):7.1f} m')
    for lim in (1.0, 5.0, 20.0):
        n = sum(1 for d in ds if d <= lim)
        print(f'      {lim:>5.0f} m 이내  {n:>4}개 ({n / len(ds) * 100:5.1f}%)')
    print(f'   그 방향 road link 가 junction  {on_junction}개 '
          f'({on_junction / len(ds) * 100:.1f}%)')
    return True


def validate(placed: list[Placed], roads, road_links,
             logs: str, lanes_csv: str, xodr: str, census: Counter) -> None:
    a = _validate_a(placed, logs)
    b = _validate_b(placed)
    c = _validate_c(placed, lanes_csv, xodr)
    d = _validate_d(placed, roads, road_links)

    print('\n' + '=' * 78)
    print('E. 미분류 노면표시 census (대상에서 빠진 것이 없는지)')
    print('=' * 78)
    rest = [(n, c_) for n, c_ in census.most_common()
            if n not in KIND_BY_NAME
            and (n.startswith('RM_') or n.startswith('Rm_')
                 or n.startswith('roadmark'))]
    for n, c_ in rest[:15]:
        print(f'      {c_:>5}  {n}')
    if not rest:
        print('      없음')

    print('\n' + '=' * 78)
    print(f'종합  A {"합격" if a else "불합격"}  B 참고  '
          f'C {"합격" if c else "확인"}  D {"합격" if d else "불합격"}')
    print('=' * 78)


# ----------------------------------------------------------------------

def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    data = os.path.normpath(os.path.join(here, '..', 'data'))
    logs = os.path.normpath(os.path.join(here, '..', 'logs'))

    ap = argparse.ArgumentParser(
        description='xodr object/signal -> 정지선·보호구역·횡단보도·방향표시·신호등')
    ap.add_argument('--xodr', default=os.path.join(data, 'HL_FMA_VTD_LivingLab.xodr'))
    ap.add_argument('--out', default=os.path.join(data, 'xodr_objects.csv'))
    ap.add_argument('--lanes', default=os.path.join(data, 'xodr_lanes.csv'),
                    help='2단계 산출물 (검증 C 에서 그래프를 만들 때 쓴다)')
    ap.add_argument('--logs', default=logs, help='사전테스트 세션 로그 디렉터리')
    ap.add_argument('--validate', action='store_true', help='전체 진단 리포트')
    args = ap.parse_args()

    if not os.path.exists(args.xodr):
        raise SystemExit(f'{args.xodr}: 없다')

    t0 = time.time()
    root = ET.parse(args.xodr).getroot()
    roads = {r.id: r for r in X.parse_roads(args.xodr)}
    sections = L.parse_lanes(args.xodr)
    road_links = None
    print(f'[obj] xodr 파싱 {time.time() - t0:.2f}s  road {len(roads)}개')

    t0 = time.time()
    raw_obj, census = parse_objects(root)
    raw_sig = parse_signals(root)
    ctrl = parse_controllers(root)
    ctrl_junction = parse_junction_controllers(root)
    sig_to_ctrl = {sid: cid for cid, sids in ctrl.items() for sid in sids}
    print(f'[obj] object {sum(census.values())}개 중 대상 {len(raw_obj)}개  '
          f'signal {len(raw_sig)}개  controller {len(ctrl)}개  '
          f'junction 연결 {len(ctrl_junction)}개')

    placed = place_objects(raw_obj, roads, sections)
    placed += place_signals(raw_sig, roads, sections, sig_to_ctrl, ctrl_junction)
    n_link, n_multi = link_stopline_ctrl(placed)
    print(f'[obj] 배치 {time.time() - t0:.2f}s  '
          f'정지선-controller 연결 {n_link}개 (후보 2개 이상 {n_multi}개)')

    kinds = Counter(p.kind for p in placed)
    unassigned = Counter(p.kind for p in placed if p.lane_id is None)
    for k, v in sorted(kinds.items()):
        u = unassigned.get(k, 0)
        print(f'[obj]   {k:<12} {v:>5}개   차로 미배정 {u:>4}개')

    # hdg 교차 검증 -- hdg~pi 면 양수 차로, hdg~0 이면 음수 차로여야 한다
    bad_side = sum(1 for p in placed
                   if p.lane_id is not None and p.kind != 'signal'
                   and (abs(_wrap(p.heading - X_pose_heading(roads, p))) > 1.0)
                   != (p.lane_id > 0))
    print(f'[obj] hdg-차로부호 교차검증 불일치 {bad_side}건')

    n = write_csv(args.out, placed)
    print(f'[obj] {os.path.normpath(args.out)}: {n}행')

    if args.validate:
        road_links = __import__('xodr_graph').parse_road_links(root)
        validate(placed, roads, road_links, args.logs, args.lanes,
                 args.xodr, census)


def X_pose_heading(roads: dict[str, X.Road], p: Placed) -> float:
    return roads[p.road_id].pose_at(p.s)[2]


if __name__ == '__main__':
    main()
