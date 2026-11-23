#!/usr/bin/env python3
"""영구 정지 차량·장애물 우회 (완주 보장).

obstacles.py는 앞차가 안 비키면 크립으로 CREEP_GAP(2m)까지 접근한 뒤 **거기서
영원히 선다.** 차량·보행자는 어떤 경우에도 통과하지 않도록 설계돼 있다.
주최측이 "정지 차량들을 우회해서 다시 좌회전 차선으로 들어와야 한다"고 했으므로
이 시나리오는 본선에 나오고, 지금 구조로는 미완주다. 완주 그룹은 미완주 그룹보다
항상 상위이므로 이것이 어떤 감점보다 나쁘다.

두 층으로 푼다.

1. **인접 차로 재탐색.** xodr_graph의 탐색을 그대로 재사용해 "현재 위치 ->
   남은 체크포인트"를 다시 푼다. 막힌 (road, section, lane) 노드에 큰 비용을
   더해 옆 차로를 고르게 만든다. 경로를 새로 만들지 않는다.
2. **최후 안전망.** 우회가 불가능하거나 실패했는데도 상대가 안 비키면
   LAST_RESORT_S 뒤에 저속으로 밀고 통과한다. 미완주보다 -6점이 낫다.

--- 계획 단계에서 직접 잰 것이 이 파일의 설계를 정한다 ---

1. **재탐색은 빠르다.** find_route 2~4ms, path_to_waypoints 0.2~1.6ms.
   기하 생성에 xodr 파싱이 필요 없다 -- path_to_waypoints는 g.pts(=
   xodr_lanes.csv)만 읽는다. 그래도 우회 실패 시 탐색이 25ms까지 가므로
   **재탐색은 워커 스레드에서 돌린다.** 20Hz 루프는 절대 막지 않는다.

2. **우회 가능성은 경로마다 극단이다.** 차선변경 엣지 유무로 재면
   route_path_pt1은 우회 불가가 4.0m(0.5%)인데 route_path_pt2는
   457.3m(60.9%)다. 경로 2에는 s 222.8~539.0의 **316.1m 연속 우회 불가
   구간**이 있다(단일 차로 road 420/190 + 교차로 내부 3곳).
   **여기서 막히면 그래프로는 방법이 없다** -- 최후 안전망이 있는 이유다.

3. **--load-graph 덤프에는 실선 차선변경 엣지가 없다.** build_graph가
   allow_solid=False일 때 버리기 때문이다. 시작 시 합성해 붙이면 경로 2의
   우회 불가가 60.9% -> 47.4%로 준다(road 429의 101.4m가 살아난다).
   항목 6 경미 -3점은 미완주보다 낫다. 폴백으로만 쓰고 경고를 남긴다.

4. **제자리 차선변경은 조향이 포화된다.** 정지 차량 2m 뒤에서는 LC_BLEND(20m)
   여유가 없어 경로 시작부에 횡방향 점프가 남는다. v=0이면 lookahead가 4m라
   3.2m 오프셋에서 조향이 한계 0.35 rad에 붙는다. 그래서 (a) 정지를 기다리지
   않고 접근 중에 먼저 발동하고, (b) 그래도 촉박하면 lookahead를 일시 상향한다.

5. **제한은 "총 횟수"가 아니라 "같은 지점의 반복"에 걸어야 한다.** 총 3회로
   묶었더니 네 번째 정지 차량에서 대응하지 못하고 영구 정지했다. 전체는 20회
   (실질 무제한)로 두고 **같은 객체 id에 3회**로 좁힌다(_Hist).

6. **녹색인데 못 가면 그것은 신호 대기가 아니라 객체 교착이다.** 정지선 35m
   앞에서 차량에 막혔는데 sig.stopping이 참이라 확인 타이머와 안전망 누적이
   둘 다 멈췄다. GO_STATES(3/5)일 때는 얼리지 않는다.

--- 이 모듈이 min() 합류에 끼지 않는 이유 ---

우회는 속도 정책이 아니다. v_limit을 내지 않는다. 대신 두 가지를 낸다.
  * v_floor  -- 최후 안전망. crosswalks.escape와 같은 자리에서 max()로 들어간다
  * ld_min   -- lookahead 하한. 조향 안정용
경로 교체 자체는 pure_pursuit이 리스폰 처리를 그대로 재사용해 수행한다.
"""

from __future__ import annotations

import bisect
import csv
import math
import os
import queue
import threading

import obstacles as OB
import finish as FN

# 트리거 -- 확인 시간은 맥락으로 3단계다. 8초 하나로 두면 "신호 대기열 vs 영구
# 정차"를 시간으로만 가르는 셈이라 앞차 뒤에 선 뒤 총 14초를 잃는다 (실측: 45km/h에서
# 객체를 보고 5.9초 만에 앞차 뒤에 선다). 아래 _classify 참고.
DETOUR_CONFIRM_FAST_S = 1.5     # s, 맥락상 명백한 장애물 (녹색인데 정지, 신호 없는 단독)
DETOUR_CONFIRM_S = 4.0          # s, 판단 애매 (정지선·교차로 근처, 연쇄, 보행자)
DETOUR_CONFIRM_QUEUE_S = 12.0   # s, 대기열로 보이나 계속 안 움직임
BLOCK_CONFIRM_S = DETOUR_CONFIRM_S      # 옛 이름
TIER_FAST, TIER_AMBIG, TIER_QUEUE, TIER_FREEZE = 'fast', 'ambig', 'queue', 'freeze'
TIER_RANK = {TIER_FAST: 0, TIER_AMBIG: 1, TIER_QUEUE: 2, TIER_FREEZE: 3}
CONFIRM_BY_TIER = {TIER_FAST: DETOUR_CONFIRM_FAST_S, TIER_AMBIG: DETOUR_CONFIRM_S,
                   TIER_QUEUE: DETOUR_CONFIRM_QUEUE_S, TIER_FREEZE: float('inf')}
FREEZE_MAX_S = 60.0         # s, 적색 대기열 동결 상한. 적색은 25초고 stoplines의
                            # MAX_HOLD_S(45s)는 blocked면 얼어 있어 강제 적색 + 정지
                            # 차량이면 둘이 서로 기다린다(sim 500초 미완주). 그 뒤는
                            # 대기열(12s)로 넘겨 완주를 지킨다
QUEUE_SL_M = 60.0           # m, 객체 앞 이 안에 정지선이 있으면 대기열일 수 있다
CHAIN_GAP_M = 15.0          # m, 정지 차량이 이 간격 이하로 이어지면 연쇄(대기열)
JUNCTION_NEAR_M = 30.0      # m, 객체 앞 이 안에 교차로 진입이 있으면 대기열일 수 있다
BLOCK_CORRIDOR_M = 1.5      # m, 차체가 실제로 쓸어가는 반폭. obstacles.
                            # PED_CORRIDOR_M과 같은 값이다. OBST_LANE_M(1.2)은
                            # "감속할지"의 기준이라 여기 쓰지 않는다
STOPPED_MS = 0.3            # m/s, 이 아래면 정지. 움직이는 객체는 대상 아니다 --
                            # 횡단 중 보행자를 우회하면 항목 10 중대 -6점이다
DETOUR_LOOKAHEAD_M = 60.0   # m, 이 안의 객체만 본다 (obstacles.LOOKAHEAD_M과 같게)
# **옆 차로의 정지 객체도 추적한다** (9/11 실차 sc_jam). 회랑 안 객체만 보면 다익스트라가
# "가장 싼 옆 차로"를 고르는데 그 차로에도 정지 차량이 있으면 거기로 들어가 다시 막힌다.
# 3차로 도로에서 2·3차로가 막혔는데 3->2->(다시 3) 우회를 6번 실패하고 안전망이 두 대를
# 관통했다. 이 안의 정지 객체는 전부 (1) 차로 노드 페널티, (2) 같은 군집이면 여유 검증에 쓴다.
DETOUR_CLEAR_AHEAD_M = DETOUR_LOOKAHEAD_M   # m, 옆 차로 정지 객체를 추적·페널티하는 전방 거리
WIDE_LAT_M = 10.0           # m, 옆 차로 추적 횡거리 (3차로 ≈ 6.6m + 여유). 대향 차로는
                            # 차로 부호로 거른다 (DetourPlanner._lane_node)
LANE_SNAP_R = 2.5           # m, 옆 차로 객체를 차로 노드에 붙일 반경 (반폭 1.6 + 표본 간격)
# **제한의 대상은 "총 몇 번 우회했나"가 아니라 "같은 지점에서 같은 답을 반복해서
# 받나"다.** 처음에 총 3회로 묶었더니 네 번째 정지 차량에서 대응하지 못하고 영구
# 정지했다 -- 완주 그룹은 미완주 그룹보다 항상 상위라 그쪽이 훨씬 나쁘다.
MAX_DETOURS = 20            # 실질 무제한. 차폭 경계에서 진동하는 객체가 리셋을
                            # 반복하는 병리적 경우의 유일한 상한으로만 남긴다
MAX_TRIES_PER_OBJ = 3       # 같은 객체 id에 대한 재탐색 요청 횟수
TRIES_RESET_S = 3.0         # s, 이만큼 후보에서 사라지면 그 객체 기록을 버린다.
                            # "정상 주행 복귀"의 정의다 -- 아래 _Hist 참고
RETRY_MIN_S = 5.0           # s, 재시도 사이 최소 간격. 같은 입력으로 곧바로
                            # 다시 풀어 봐야 같은 답이 나온다

# 탐색
BLOCK_PENALTY = 5000.0      # 경로 최대 길이(~1000m)보다 확실히 크게. 차단이
                            # 아니라 가산이다 -- --arrow-penalty와 같은 원칙.
                            # **객체 1대당 가산** -- 정지 차량이 적은 차로가 이긴다
# 새 경로가 막은 객체에서 얼마나 떨어져야 "우회했다"고 볼 것인가.
# **obstacles가 그 객체를 어떻게 보느냐로 정해야 한다.** 처음에 우리 차폭(1.5m)
# 으로 잡았더니, 20m 블렌드가 장애물 지점에서 1.93m를 내고 "성공"으로 통과했는데
# obstacles의 VEH_LANE_M(2.0) 안이라 새 경로에서도 그대로 veh_stop이 걸렸다.
# sim이 s=691.9m에 300초 갇혔다. 기준은 "정지·크립을 유발하는 대역 밖"이다.
CLEAR_MARGIN_M = 0.3
# 블렌드 중에는 차체가 비스듬해서 **쓸고 가는 폭이 넓어진다.** 중심선 거리만
# 재면 통과인데 실제로는 스친다 -- 16m 블렌드로 중심선 2.2m를 두고 지났는데
# check_collision이 풋프린트 간격 -0.18m(충돌)를 냈다. 필요 여유에
# HALF_LEN*|sin(요각)|을 더한다.
EGO_HALF_LEN_M = 2.4        # m, 차체 중심에서 앞뒤 끝까지 (앞 3.808 / 뒤 1.0)
CLEAR_BY_KIND = {
    OB.VEHICLE: OB.VEH_LANE_M + CLEAR_MARGIN_M,       # 2.3 -- 이 안이면 영영 선다
    OB.PEDESTRIAN: OB.PED_CORRIDOR_M + CLEAR_MARGIN_M,  # 1.8 -- 밖이면 서행 통과한다
    OB.OBSTACLE: OB.OBST_LANE_M + CLEAR_MARGIN_M,     # 1.5 -- 밖이면 감속도 없다
}
CP_MIN_AHEAD_M = 5.0        # m, 이보다 가까운 체크포인트는 이미 지난 것으로 본다

# 조향 여유
LC_ROOM_M = 12.0            # m, 이보다 짧은 블렌드는 촉박한 교체다
# 블렌드는 **비켜 갈 수 있는 한 길게** 잡는다. 짧을수록 조향이 가팔라진다 --
# 3.36m를 8m에 붙였더니 조향이 한계 0.350 rad에 그대로 포화됐다. smoothstep이라
# 장애물 지점에서 절반만 넘어가 있어도 1.5m는 확보되므로 길게 잡을 여지가 크다.
BLEND_TRIES = (20.0, 16.0, 12.0, 9.0, 7.0, 5.0)
# 달리는 중에 발동하면(맥락 판단으로 1.5초 만에 걸린다) 여유가 훨씬 많다. 긴
# 블렌드부터 넣는다 -- _clears가 여유를 그대로 검사하므로 가까우면 저절로 내려간다
BLEND_TRIES_MOVING = (35.0, 30.0, 25.0) + BLEND_TRIES
MOVING_MS = 3.0             # m/s, 이 이상이면 "달리는 중" 사다리
DETOUR_LD_M = 8.0           # m, 그때 쓸 lookahead 하한 (pure_pursuit.LD_MAX=10 안)
LD_RESTORE_M = 1.0          # m, 경로 이탈이 이 아래면 복귀로 본다
LD_RESTORE_N = 5            # 프레임, 연속 이만큼 유지돼야 복원
LD_BOOST_MAX_S = 10.0       # s, 상한

# 최후 안전망
LAST_RESORT_S = 10.0        # s, 이만큼 정지가 지속되면 밀고 나간다. 20초는
                            # 길었다 -- 15분 예산에서 여러 번 발동하면 손실이 크다
LAST_RESORT_V = 5.0 / 3.6   # m/s, 5km/h
LR_EXIT_M = 3.0             # m, 객체를 이만큼 지나면 해제


# ----------------------------------------------------------------------
# 명령
# ----------------------------------------------------------------------

class DetourCmd:
    """한 프레임의 판정. v_limit이 없는 유일한 정책이다."""

    __slots__ = ('action', 'reason', 'v_floor', 'ld_min', 'dist',
                 'obj_id', 'kind')

    def __init__(self, action='none', reason='', v_floor=0.0, ld_min=0.0,
                 dist=float('nan'), obj_id=-1, kind=''):
        self.action = action    # none/watch/search/swapped/impossible/last_resort
        self.reason = reason
        self.v_floor = v_floor  # m/s. 0이면 제약 없음. max()로 들어간다
        self.ld_min = ld_min    # m. lookahead 하한. 0이면 제약 없음
        self.dist = dist        # 후륜축 -> 막은 객체 경로거리 [m]
        self.obj_id = obj_id
        self.kind = kind

    @property
    def forcing(self) -> bool:
        """최후 안전망이 발동 중인가. obs.accel_limit를 빼는 판단에 쓴다."""
        return self.v_floor > 0.0

    def __repr__(self) -> str:
        return (f'<DetourCmd {self.action} id={self.obj_id} '
                f'd={self.dist:.1f} floor={self.v_floor * 3.6:.1f}km/h>')


# ----------------------------------------------------------------------
# 경로 메타데이터 -- waypoint CSV 를 다시 읽는다
# ----------------------------------------------------------------------

class Row:
    """경로 인덱스 하나의 차로 귀속. pure_pursuit.Path는 (x,y)만 들고 있다."""

    __slots__ = ('x', 'y', 'road_id', 'section', 'lane_id', 'lc_dir')

    def __init__(self, x, y, road_id, section, lane_id, lc_dir=''):
        self.x, self.y = x, y
        self.road_id, self.section, self.lane_id = road_id, section, lane_id
        self.lc_dir = lc_dir

    @property
    def node(self) -> tuple:
        return (self.road_id, self.section, self.lane_id)


def load_rows(waypoints_csv: str, segments: list) -> list:
    """pure_pursuit.load_path와 **같은 순서로** 차로 메타데이터를 읽는다.

    load_path는 segment_id/x/y만 읽고 road_id·lane_id를 버린다. turnsignal이
    lc_dir 때문에 같은 CSV를 다시 여는 것과 같은 선례다 (load_path는 손대지 않는다).
    """
    by_seg: dict = {}
    with open(waypoints_csv, newline='', encoding='utf-8') as fh:
        rdr = csv.DictReader(fh)
        need = ('road_id', 'lane_id', 'section')
        if not all(c in (rdr.fieldnames or []) for c in need):
            return []                   # 수동 녹화본 등 -- 차로 정보가 없다
        for r in rdr:
            by_seg.setdefault(r['segment_id'], []).append(Row(
                float(r['x']), float(r['y']), r['road_id'],
                int(r['section']), int(r['lane_id']),
                (r.get('lc_dir') or '').strip()))
    out = []
    for name in segments:
        out.extend(by_seg.get(name, ()))
    return out


def check_rows(path, rows: list) -> str:
    """rows가 path.pts와 1:1로 맞는가. 어긋나면 사유 문자열, 맞으면 ''."""
    if not rows:
        return '경로 CSV에 road_id/lane_id/section 컬럼이 없다'
    # path.pts는 끝에 가상 연장(finish.extend_points)이 붙어 있다. rows는
    # CSV 행이라 실제 점(n_real)과만 1:1이다
    n_real = getattr(path, 'n_real', len(path.pts))
    if len(rows) != n_real:
        return f'행 수 불일치 ({len(rows)}행 vs 경로 {n_real}점)'
    worst = 0.0
    for r, (x, y) in zip(rows, path.pts):
        d = math.hypot(r.x - x, r.y - y)
        if d > worst:
            worst = d
    if worst > 0.05:
        return f'좌표 불일치 최대 {worst:.3f}m'
    return ''


# ----------------------------------------------------------------------
# 그래프 보조 -- 우회 가능성 판정
# ----------------------------------------------------------------------

def lanechange_nodes(g) -> set:
    return {n for n, es in g.edges.items()
            if any(e.kind == 'lanechange' for e in es)}


def junction_nodes(g) -> set:
    """kind='junction' 엣지의 도착 노드 = connecting road. 엣지 CSV만으로 난다."""
    out = set()
    for es in g.edges.values():
        for e in es:
            if e.kind == 'junction':
                out.add(e.to)
    return out


def same_dir_lanes(g, node) -> int:
    """같은 진행 방향(같은 부호) driving 차로 수."""
    rid, sec, lid = node
    sgn = 1 if lid > 0 else -1
    return sum(1 for k in range(1, 9) if (rid, sec, sgn * k) in g.pts)


def add_solid_lanechanges(g) -> int:
    """덤프에 없는 실선 차선변경 엣지를 합성해 붙인다.

    build_graph의 차선변경 게이트를 그대로 옮긴 것이다. allow_solid=False로
    덤프된 그래프에는 실선 쌍이 아예 빠져 있어 폴백 탐색을 할 수 없다.
    """
    import xodr_graph as G
    added = 0
    for n, pl in g.pts.items():
        rid, si, lid = n
        if G.span(pl) < G.LC_MIN_LEN:
            continue
        have = {e.to for e in g.out(n) if e.kind == 'lanechange'}
        for adj in (lid - 1, lid + 1):
            to = (rid, si, adj)
            if adj == 0 or adj * lid < 0 or to not in g.pts or to in have:
                continue
            inner = n if abs(lid) < abs(adj) else to
            if g.pts[inner][0].mark_out == 'none':
                g.edges.setdefault(n, []).append(
                    G.Edge(to, G.LC_SOLID_PENALTY, 'lanechange', solid=True))
                added += 1
    return added


class Zone:
    """차선변경 엣지가 없어 그래프로는 우회할 수 없는 연속 구간."""

    __slots__ = ('i0', 'i1', 's0', 's1', 'nodes', 'why')

    def __init__(self, i0, i1, s0, s1, nodes, why):
        self.i0, self.i1 = i0, i1
        self.s0, self.s1 = s0, s1
        self.nodes = nodes          # [(road,sec,lane), ...] 등장 순
        self.why = why              # {node: 사유 문자열}

    @property
    def length(self) -> float:
        return self.s1 - self.s0

    def contains(self, s: float) -> bool:
        return self.s0 <= s <= self.s1

    def label(self) -> str:
        seen, parts = set(), []
        for n in self.nodes:
            key = (n[0], self.why[n])
            if key in seen:
                continue
            seen.add(key)
            parts.append(f'road {n[0]} {self.why[n]}')
        return ' / '.join(parts)

    def node_at(self, rows: list, i: int) -> tuple:
        i = max(self.i0, min(self.i1, i))
        return rows[i].node


def scan_undetourable(path, rows: list, g, has_lc: set = None,
                      injunc: set = None) -> list:
    """차선변경 엣지가 없는 노드의 연속 구간을 묶는다.

    대회 당일 4번 단계(시뮬레이션 시작) 전에 눈으로 확인해야 하는 정보다.
    """
    if has_lc is None:
        has_lc = lanechange_nodes(g)
    if injunc is None:
        injunc = junction_nodes(g)

    def why(n):
        if n in injunc:
            return '교차로 내부'
        c = same_dir_lanes(g, n)
        if c <= 1:
            return '단일 차로'
        return f'{c}차로지만 차선변경 엣지 없음'

    zones, run = [], None
    for i, r in enumerate(rows):
        n = r.node
        if n not in has_lc:
            if run is None:
                run = [i, i, [], {}]
            run[1] = i
            if n not in run[3]:
                run[2].append(n)
                run[3][n] = why(n)
        elif run is not None:
            zones.append(Zone(run[0], run[1], path.s[run[0]], path.s[run[1]],
                              run[2], run[3]))
            run = None
    if run is not None:
        zones.append(Zone(run[0], run[1], path.s[run[0]], path.s[run[1]],
                          run[2], run[3]))
    return zones


def zone_report(path, zones: list, solid_m: float = None) -> list:
    """시작 시 콘솔에 낼 줄들. meta.json에도 같은 내용이 들어간다."""
    tot = sum(z.length for z in zones)
    plen = FN.finish_s(path)            # 실제 길이 (끝 가상 연장 제외)
    pct = 100.0 * tot / plen if plen > 0 else 0.0
    out = [f' 우회 불가 구간 {len(zones)}개 / 총 {tot:.1f}m ― 경로의 {pct:.1f}%']
    for k, z in enumerate(zones, 1):
        out.append(f'   {k}. s {z.s0:7.1f}~{z.s1:7.1f} {z.length:6.1f}m  '
                   f'{z.label()}')
    if solid_m is not None and solid_m < tot - 0.05:
        sp = 100.0 * solid_m / plen if plen > 0 else 0.0
        out.append(f'   실선 차선변경 허용 시 {solid_m:.1f}m ({sp:.1f}%)로 줄어든다')
    if not zones:
        out.append('   경로 전 구간에서 옆 차로로 우회할 수 있다')
    return out


def zones_meta(zones: list) -> list:
    return [{'s0': round(z.s0, 1), 's1': round(z.s1, 1),
             'len_m': round(z.length, 1), 'why': z.label()} for z in zones]


# ----------------------------------------------------------------------
# 재탐색 -- 워커 스레드
# ----------------------------------------------------------------------

class Plan:
    """워커가 낸 결과. ok면 pure_pursuit이 경로를 통째로 갈아 끼운다."""

    __slots__ = ('ok', 'reason', 'bundle', 'rows', 'zones', 'csv_path',
                 'ev_path', 'segments', 'tight', 'first_lc_m', 'solid_lc',
                 'n_lc', 'cost', 'blocked', 'obj_ids', 'start_lat', 'blend_m',
                 'blend_dir', 'n_wide')

    def __init__(self, ok, reason='', **kw):
        self.ok, self.reason = ok, reason
        self.bundle = kw.get('bundle')
        self.rows = kw.get('rows') or []
        self.zones = kw.get('zones') or []
        self.csv_path = kw.get('csv_path', '')
        self.ev_path = kw.get('ev_path', '')
        self.segments = kw.get('segments') or []
        self.tight = kw.get('tight', False)
        self.first_lc_m = kw.get('first_lc_m', float('nan'))
        self.solid_lc = kw.get('solid_lc', 0)
        self.n_lc = kw.get('n_lc', 0)
        self.cost = kw.get('cost', 0.0)
        self.blocked = kw.get('blocked') or []
        self.obj_ids = kw.get('obj_ids') or []
        self.start_lat = kw.get('start_lat', 0.0)
        self.blend_m = kw.get('blend_m', 0.0)
        self.blend_dir = kw.get('blend_dir', '')
        self.n_wide = kw.get('n_wide', 0)     # 차로 노드가 풀린 옆 차로 정지 객체 수


def _snapshot(wps: list) -> list:
    return [(w.x, w.y) for w in wps]


def _restore(wps: list, snap: list) -> None:
    for w, (x, y) in zip(wps, snap):
        w.x, w.y = x, y


def _blend_start(wps: list, ego_x: float, ego_y: float,
                 blend_m: float) -> float:
    """새 경로가 ego에서 옆으로 떨어져 시작하면 그 간격을 blend_m에 걸쳐 없앤다.

    출발 후보를 옆 차로까지 열어 두는 것은 **의도한 것이다** -- 정지 차량 바로
    뒤에서는 원래 차로를 계속 달릴 여지가 없고, path_to_waypoints의 차선변경
    핸드오프는 남은 구간의 절반 지점이라 장애물보다 뒤에 잡힌다. 대신 그대로
    두면 경로 첫 점에서 차로폭만큼 횡방향 점프가 남으므로, 여기서 ego 위치부터
    비스듬히 붙인다. smoothstep은 path_to_waypoints의 차선변경 블렌드와 같다.

    반환값은 처음 횡간격 [m] (좌가 양수).
    """
    if len(wps) < 2 or blend_m <= 0.1:
        return 0.0
    th = math.atan2(wps[1].y - wps[0].y, wps[1].x - wps[0].x)
    lat = (-math.sin(th) * (ego_x - wps[0].x)
           + math.cos(th) * (ego_y - wps[0].y))
    if abs(lat) < 0.05:
        return 0.0
    nx, ny = -math.sin(th), math.cos(th)
    c, px, py = 0.0, wps[0].x, wps[0].y
    for w in wps:
        c += math.hypot(w.x - px, w.y - py)
        px, py = w.x, w.y
        u = min(1.0, max(0.0, c / blend_m))
        k = 1.0 - u * u * (3.0 - 2.0 * u)       # smoothstep의 여집합
        if k <= 0.0:
            break
        w.x += nx * lat * k
        w.y += ny * lat * k
    return lat


LC_MARK_MIN_M = 0.8         # m, 이보다 작은 블렌드는 차로 변경이 아니라 미세 보정


def _mark_escape_blend(wps: list, lat: float, blend_m: float) -> str:
    """탈출 블렌드에 lc_dir 마커를 심는다. 심은 방향, 안 심었으면 ''.

    _blend_start가 붙이는 구간은 **실제 차로 변경이다.** 마커가 없으면
    turnsignal이 그 기동을 모른다 -- 실차에서 좌회전 점등을 켠 채 우측으로
    블렌드해 나갔다 (pp_20260909_162539의 detour_4, 횡간격 +3.06m).

    **부호에 주의한다.** lat은 _blend_start가 낸 값이고 **새 경로 기준 ego의
    횡위치**(좌가 양수)다. lat>0이면 새 차로가 오른쪽에 있다는 뜻이라 차는
    **우측으로** 움직인다. 실측 3건이 전부 그렇다 (detour_1 +2.95 -> 우,
    detour_2 -3.01 -> 좌, detour_4 +3.06 -> 우). --selftest가 이 불변식을
    turnsignal._lateral_ahead와 대조한다.

    마커는 블렌드 한가운데에 둔다. turnsignal이 마커 ±LC_BLEND_M/2(10m)를
    구간으로 잡으므로 9~20m 블렌드 전체가 덮인다.
    """
    if len(wps) < 2 or blend_m <= 0.1 or abs(lat) < LC_MARK_MIN_M:
        return ''
    want = blend_m * 0.5
    c, px, py = 0.0, wps[0].x, wps[0].y
    best_i, best_d = 0, float('inf')
    for i, w in enumerate(wps):
        c += math.hypot(w.x - px, w.y - py)
        px, py = w.x, w.y
        d = abs(c - want)
        if d < best_d:
            best_i, best_d = i, d
        if c > want + 1.0:
            break
    if wps[best_i].lc_dir:              # 그래프가 준 마커를 덮지 않는다
        return ''
    direction = 'right' if lat > 0.0 else 'left'
    wps[best_i].lc_dir = direction
    wps[best_i].lc_solid = 0            # 실선인지 모른다. 항목 6 경고를
    return direction                    # 거짓으로 올리지 않는다


def _seg_index(name: str) -> int:
    """'p7_to_p8' -> 7. 형식이 아니면 0."""
    try:
        a, b = name.split('_to_p')
        return int(a[1:]) if b else 0
    except Exception:
        return 0


def _renumber(wps: list, offset: int) -> list:
    """path_to_waypoints는 p1_to_p2부터 다시 센다. 원래 번호를 잇는다."""
    if offset <= 0:
        return sorted({w.seg for w in wps}, key=_seg_index)
    order = []
    for w in wps:
        k = _seg_index(w.seg)
        if k:
            w.seg = f'p{k + offset}_to_p{k + offset + 1}'
        if w.seg not in order:
            order.append(w.seg)
    return order


class DetourPlanner:
    """그래프 자원을 소유하고 재탐색을 워커 스레드에서 돌린다.

    request()는 즉시 돌아오고 poll()은 결과가 있을 때만 준다.
    20Hz 루프는 어떤 경우에도 여기서 블로킹되지 않는다.
    """

    def __init__(self, data_dir: str, out_dir: str, build_policies,
                 on_event=None, lanes_csv: str = '', edges_csv: str = '',
                 objects_csv: str = ''):
        import xodr_graph as G
        self.G = G
        self.out_dir = out_dir
        self.build_policies = build_policies
        self._on_event = on_event or (lambda kind, detail: None)

        self.lanes_csv = lanes_csv or os.path.join(data_dir, 'xodr_lanes.csv')
        self.edges_csv = edges_csv or os.path.join(data_dir,
                                                   'xodr_graph_edges.csv')
        self.objects_csv = objects_csv or os.path.join(data_dir,
                                                       'xodr_objects.csv')

        # 시작 시 한 번. 실측 0.54s -- 연결 전이라 루프에 영향이 없다.
        pts, _dropped = G.load_lane_points(self.lanes_csv)
        self.graph = G.load_graph(self.edges_csv, pts)
        self.index = G.LaneIndex(pts)
        self.has_lc = lanechange_nodes(self.graph)
        self.injunc = junction_nodes(self.graph)

        # 실선 폴백용 그래프는 원본을 복사해 따로 만든다. 기본 탐색이 실선을
        # 쓰지 않도록 원본은 손대지 않는다.
        self.solid_graph = G.Graph(pts=pts,
                                   edges={n: list(es) for n, es
                                          in self.graph.edges.items()})
        self.n_solid = add_solid_lanechanges(self.solid_graph)
        self.has_lc_solid = lanechange_nodes(self.solid_graph)

        self.objs = []
        if os.path.exists(self.objects_csv):
            import xodr_objects as O
            self.objs = O.load_objects(self.objects_csv)

        self._q_in = queue.Queue()
        self._q_out = queue.Queue()
        self._busy = False
        self._n = 0
        self._th = threading.Thread(target=self._loop, daemon=True,
                                    name='detour')
        self._th.start()

    def on_road(self, x: float, y: float) -> bool:
        """이 점이 driving 차로 위인가 (obstacles의 보행자 횡단 완료 판정용).

        LaneIndex는 driving 차로 중심점만 담고 있으므로 PED_OFFROAD_M 안에
        중심점이 하나도 없으면 보도·중앙분리대다. 5m 격자라 싸고 읽기만 한다
        (워커 스레드와 공유해도 안전하다).
        """
        return bool(self.index.near(x, y, OB.PED_OFFROAD_M))

    # -- 외부 인터페이스 ------------------------------------------------
    @property
    def busy(self) -> bool:
        return self._busy

    def request(self, job: dict) -> None:
        self._busy = True
        self._n += 1
        job['n'] = self._n
        self._q_in.put(job)

    def poll(self):
        try:
            plan = self._q_out.get_nowait()
        except queue.Empty:
            return None
        self._busy = False
        return plan

    # -- 워커 ------------------------------------------------------------
    def _loop(self) -> None:
        while True:
            job = self._q_in.get()
            try:
                plan = self._solve(job)
            except Exception as exc:            # 워커가 죽으면 안 된다
                plan = Plan(False, f'재탐색 예외 {exc!r}',
                            obj_ids=job.get('obj_ids') or [])
            self._q_out.put(plan)

    @staticmethod
    def _penalized(g, blocked: dict):
        """막힌 노드로 들어가는 엣지와 거기서 나가는 직진 엣지에 비용을 더한다.

        차단이 아니라 가산이다. 대안이 없으면 페널티를 먹고 원래대로 가고,
        그 경우는 아래 결과 검증에서 걸린다. 탐색이 실패하지 않는 것이 중요하다.

        blocked는 {노드: 정지 객체 수}. **객체 1대당 가산**이라 정지 차량이 적은
        차로가 이긴다 (3차로에 2·2·1대면 1대짜리로 간다). 차선변경으로 빠져나가는
        엣지는 그대로 무료다.
        """
        import xodr_graph as G
        edges = {}
        for n, es in g.edges.items():
            out = []
            for e in es:
                c = e.cost + BLOCK_PENALTY * blocked.get(e.to, 0)
                if e.kind != 'lanechange':
                    c += BLOCK_PENALTY * blocked.get(n, 0)
                out.append(G.Edge(e.to, c, e.kind, e.solid))
            edges[n] = out
        return G.Graph(pts=g.pts, edges=edges)

    @staticmethod
    def _lane_node(index, x: float, y: float, ref_node):
        """옆 차로 객체 (x,y)가 놓인 차로 노드. 없으면 None.

        같은 도로(ref_node의 road id)의 **같은 부호** 차로 중 최근접만 받는다 --
        대향 차로(부호 반대)·다른 도로·교차로 연결로에 선 차는 우리 경로 후보가
        아니거나 판단할 근거가 없으므로 페널티를 주지 않는다. section은 거르지
        않는다 (같은 s의 경로 행보다 한 섹션 앞뒤일 수 있다).
        """
        rid, _sec, lid = ref_node
        best, best_d = None, float('inf')
        for n, (dd, _s, _h) in index.near(x, y, LANE_SNAP_R).items():
            if n[0] != rid or n[2] * lid <= 0:
                continue
            if dd < best_d:
                best, best_d = n, dd
        return best

    @staticmethod
    def _clears(wps: list, obstacles_xy: list) -> tuple:
        """새 경로가 막은 객체들을 실제로 비켜 가는가. (성공 여부, 최소 여유 비율)

        **이것이 성공 판정이다.** 비용이나 노드 집합으로는 판정할 수 없다 --
        ego가 막힌 노드 안에 있다가 장애물 앞에서 빠져나가는 정상 해도 그
        노드를 지나기 때문이다. 기하로 직접 재는 것이 유일하게 확실하다.

        필요 여유는 종류마다 다르다 (CLEAR_BY_KIND 참고).
        """
        worst, ok = float('inf'), True
        for ox, oy, need, ohdg in obstacles_xy:
            best, need_here = float('inf'), need
            for a, b in zip(wps, wps[1:]):
                ax, ay, bx, by = a.x, a.y, b.x, b.y
                dx, dy = bx - ax, by - ay
                den = dx * dx + dy * dy
                if den < 1e-12:
                    continue
                u = ((ox - ax) * dx + (oy - ay) * dy) / den
                u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
                d = math.hypot(ox - (ax + u * dx), oy - (ay + u * dy))
                if d < best:
                    best = d
                    yaw = (math.atan2(dy, dx) - ohdg + math.pi) % (2 * math.pi) - math.pi
                    need_here = need + EGO_HALF_LEN_M * abs(math.sin(yaw))
            if best - (need_here - need) < worst:
                worst = best - (need_here - need)    # 요각을 뺀 실효 여유
            if best < need_here:
                ok = False
        return ok, worst

    def _search(self, g, cps, blocked: dict, pin=None):
        G = self.G
        g2 = self._penalized(g, blocked)
        res, stuck = G.find_route(g2, self.index, cps, pin=pin)
        if res is None:
            return None, None, f'탐색 실패 (경유지 {stuck}에서 막힘)'
        wps, _lc = G.path_to_waypoints(g2, res, cps)
        if len(wps) < 2:
            return None, None, '생성된 waypoint가 2점 미만'
        return res, wps, ''

    def _neighbours(self, node) -> set:
        """같은 섹션의 좌우 인접 차로 (같은 진행 방향)."""
        rid, sec, lid = node
        out = {node}
        for adj in (lid - 1, lid + 1):
            if adj and adj * lid > 0 and (rid, sec, adj) in self.graph.pts:
                out.add((rid, sec, adj))
        return out

    def _solve(self, job: dict) -> Plan:
        G = self.G
        cps = job['cps']
        blocked = dict(job['blocked'])
        obs_xy = list(job['obs_xy'])
        ids = job['obj_ids']
        ex, ey = job['ego_xy']
        # 옆 차로의 정지 객체 -- 차로 노드를 풀어 페널티에 더하고, 회랑 그룹과
        # 같은 군집(hard)이면 여유 검증 대상에도 넣는다 (9/11 sc_jam)
        n_wide = 0
        for w in job.get('wide_obs', ()):
            n_at = self._lane_node(self.index, w['x'], w['y'], w['ref_node'])
            if n_at is None:
                continue
            blocked[n_at] = blocked.get(n_at, 0) + 1
            n_wide += 1
            if w.get('hard'):
                obs_xy.append((w['x'], w['y'], w['need'], w['ohdg']))

        # **출발 차로를 묶는다.** 안 묶으면 다익스트라가 차선변경 비용을 물지
        # 않고 출발 후보 중 가장 싼 차로를 골라, 실측에서 3차로 건너뛴 경로가
        # 나왔다(road 418 lane -4 -> -1, 끝점이 10.2m 어긋났다). 도착 차로도
        # 원래 경로의 것으로 묶어 "우회한 뒤 다시 제 차선으로 들어오게" 한다.
        pin_full = {0: self._neighbours(job['ego_node']),
                    len(cps) - 1: {job['goal_node']}}
        pin_start = {0: pin_full[0]}

        best, last_err = None, '탐색 실패'
        for label, g, pin in (('파선', self.graph, pin_full),
                              ('실선 폴백', self.solid_graph, pin_full),
                              ('도착 차로 제약 해제', self.graph, pin_start)):
            res, wps, err = self._search(g, cps, blocked, pin)
            if res is None:
                last_err = err
                continue
            snap = _snapshot(wps)
            tries = (BLEND_TRIES_MOVING if job.get('ego_v', 0.0) >= MOVING_MS
                     else BLEND_TRIES)
            for blend_m in tries:           # 긴 것부터 -- 조향이 가장 완만하다
                _restore(wps, snap)
                lat = _blend_start(wps, ex, ey, blend_m)
                ok, gap = self._clears(wps, obs_xy)
                if ok:
                    best = (label, res, wps, lat, blend_m)
                    break
                need = max(n for _x, _y, n, _h in obs_xy)
                last_err = (f'우회로가 막은 객체를 {gap:.2f}m까지밖에 못 '
                            f'벗어난다 (필요 {need:.1f}m, '
                            f'블렌드 {blend_m:.0f}m)')
                if abs(lat) < 0.05:
                    break               # 블렌드와 무관한 실패다. 더 줄여도 같다
            if best is not None:
                break
        if best is None:
            return Plan(False, last_err, blocked=sorted(blocked), obj_ids=ids,
                        n_wide=n_wide)

        label, res, wps, lat, blend_m = best
        solid = res.solid_lc
        if label == '실선 폴백' and solid:
            self._on_event('detour_solid_lc',
                           f'실선 차선변경 {solid}회를 포함한 우회로를 쓴다 ― '
                           f'항목 6 경미 -3점 예상 (파선 우회로가 없다)')
        elif label == '도착 차로 제약 해제':
            self._on_event('detour_solid_lc',
                           '원래 도착 차로로 돌아오는 우회로가 없어 도착 차로 '
                           '제약을 풀었다 ― 마지막 구간의 차로가 달라진다')

        segments = _renumber(wps, job.get('seg_offset', 0))

        # **마커를 심기 전에 센다.** 합성 마커까지 세면 first_lc_m이 늘
        # blend_m/2가 되어 tight가 항상 참이 되고 lookahead 상향이 달라진다.
        first, acc = float('nan'), 0.0
        for a, b in zip(wps, wps[1:]):
            if a.lc_dir:
                first = acc
                break
            acc += math.hypot(b.x - a.x, b.y - a.y)
        n_lc = sum(1 for w in wps if w.lc_dir)
        tight = ((abs(lat) > 0.5 and blend_m < LC_ROOM_M)
                 or ((first == first) and first < LC_ROOM_M))

        blend_dir = _mark_escape_blend(wps, lat, blend_m)

        base = os.path.join(self.out_dir, f'detour_{job["n"]}')
        csv_path, ev_path = base + '.csv', base + '_events.csv'
        G.write_waypoints(csv_path, wps)
        if self.objs:
            G.write_route_events(ev_path, G.route_events(self.objs, wps))
        else:
            ev_path = ''

        rows = load_rows(csv_path, segments)
        bundle = self.build_policies(csv_path, ev_path, segments)
        # rows가 새 경로와 1:1이 아니면 다음 재탐색이 엉뚱한 노드를 막는다.
        # 여기서 걸러 내고 기존 거동을 유지하는 편이 안전하다.
        bad = check_rows(bundle['path'], rows)
        if bad:
            return Plan(False, f'우회로 메타데이터 불일치: {bad}',
                        blocked=sorted(blocked), obj_ids=ids, n_wide=n_wide)
        zones = scan_undetourable(bundle['path'], rows, self.graph,
                                  self.has_lc, self.injunc)

        return Plan(True, '', bundle=bundle, rows=rows, zones=zones,
                    csv_path=csv_path, ev_path=ev_path, segments=segments,
                    tight=tight, first_lc_m=first, solid_lc=solid,
                    n_lc=n_lc, cost=res.cost, blocked=sorted(blocked),
                    obj_ids=ids, start_lat=lat, blend_m=blend_m,
                    blend_dir=blend_dir, n_wide=n_wide)


# ----------------------------------------------------------------------
# 매 프레임 판정
# ----------------------------------------------------------------------

LR_NEAR_M = 15.0            # m, 이 안에 있는 확정 막힘만 최후 안전망 대상


class _B:
    """추적 중인 정지 객체 하나. 확인 시간(acc)이 핵심이다.

    **경로 교체(rebind)를 넘어 산다.** x/y는 월드 좌표라 그대로 유효하고 s/lat/d/
    corridor는 다음 프레임 _track이 새 경로로 재투영한다. 예전엔 rebind가 통째로
    버려서 옆 차로에서 우회해 들어간 뒤 그 차로의 정지 차량을 0초부터 다시 셌다
    (9/11 sc_jam). 회랑 밖(옆 차로) 객체도 여기 들어오지만 corridor=False라
    발동(_ready) 대상은 아니다 -- 페널티·군집 검증·acc 선적립에만 쓴다.
    """

    __slots__ = ('id', 'x', 'y', 's', 'lat', 'd', 'kind', 'acc', 'missing',
                 'tier', 'confirm', 'why', 'frozen', 'corridor', 'told')

    def __init__(self, oid, x, y, s, lat, d, kind, t):
        self.id = oid
        self.x, self.y = x, y
        self.s, self.lat, self.d = s, lat, d
        self.kind = kind
        self.acc = 0.0          # 조건을 만족한 누적 시간 [s]
        self.missing = 0        # 안 보인 연속 프레임 수
        self.tier = TIER_AMBIG  # 맥락 판단 (매 프레임 다시 정한다)
        self.confirm = DETOUR_CONFIRM_S
        self.why = ''
        self.frozen = 0.0       # 적색 대기열로 동결돼 있은 누적 시간 [s]
        self.corridor = False   # 이번 프레임 회랑(|lat| < BLOCK_CORRIDOR_M) 안인가
        self.told = False       # detour_context를 한 번이라도 냈나 (회랑 안에서만 낸다)


class _Hist:
    """객체별 기록. **경로 교체(rebind)를 넘어 산다.**

    시도 횟수를 _B에 두면 그 객체가 잠깐 후보에서 빠지는 순간(드롭아웃 grace
    초과, 옆 차로로 나갔다 들어옴) 0으로 돌아간다. MAX_DETOURS를 20으로 올린
    지금 그대로 두면 같은 객체가 계속 차폭 안에 남을 때 20회까지 무한 루프를 돈다.
    (9/11까지는 rebind가 _B를 통째로 버리는 것이 이 분리의 첫 이유였다.)

    **리셋은 "그 객체가 후보에서 사라졌을 때"다** (TRIES_RESET_S). 우회에
    성공하면 그 객체는 필요 여유(2.3/1.8/1.5m) 밖이라 즉시 후보에서 빠진다.
    "움직이기 시작했을 때"로 잡으면 obstacles의 크립 교착, crosswalks의 탈출
    자기취소와 똑같은 실수가 된다.
    """

    __slots__ = ('tries', 'last_try', 'gone_s')

    def __init__(self):
        self.tries = 0
        self.last_try = -1e9    # 마지막 재탐색 요청 시각
        self.gone_s = 0.0       # 후보에서 사라져 있은 시간 [s]


class DetourPolicy:
    """막힘을 확인해 재탐색을 걸고, 안 되면 최후 안전망을 낸다.

    v_limit을 내지 않는다. 경로 교체는 pure_pursuit이 리스폰 처리를 재사용해
    수행하고, 이 클래스는 언제 무엇을 바꿀지만 정한다.
    """

    def __init__(self, path, rows: list, zones: list, bounds: list, *,
                 planner=None,
                 confirm_s=None,
                 last_resort_s: float = LAST_RESORT_S,
                 lookahead: float = DETOUR_LOOKAHEAD_M,
                 use_last_resort: bool = True,
                 finish_zone: float = FN.FINISH_ZONE_M, on_event=None,
                 stoplines=(), injunc=None):
        """confirm_s=None이면 맥락 3단계(1.5/4/12s). 값을 주면 **세 단계 전부**
        그 값이다 (옛 --detour-confirm 명령과 호환). 적색 대기열 동결은 어느
        쪽이든 그대로다.

        stoplines는 stoplines.Stopline 목록(.s/.signalized), injunc는 교차로
        내부 노드 집합(detour.junction_nodes). 둘 다 맥락 판단에만 쓴다.
        """
        self.path = path
        self.rows = rows
        self.zones = zones
        self.bounds = bounds
        self.planner = planner
        self.confirm_s = confirm_s
        self.stoplines = list(stoplines)
        if injunc is None:
            injunc = getattr(planner, 'injunc', None) or set()
        self.injunc = set(injunc)
        self.last_resort_s = last_resort_s
        self.lookahead = lookahead
        self.use_last_resort = use_last_resort
        # 종료 구간. 여기서 우회하면 종료선을 벗어나 완주가 아니다. 라바콘은
        # 후보에서 빼고(종료 라바콘), 차량·보행자 막힘은 우회 불가로 보내
        # 최후 안전망만 남긴다
        self.finish_zone = finish_zone
        self._gate_s = FN.gate_s(path, finish_zone)
        self._on_event = on_event or (lambda kind, detail: None)
        self.n_detours = 0
        self.reset()

    # -- 상태 -----------------------------------------------------------
    def reset(self) -> None:
        """리스폰 시. 위치가 강제로 옮겨졌으므로 관측을 전부 버린다."""
        self._blocks: dict = {}
        self._hist: dict = {}           # id -> _Hist. rebind를 넘어 산다
        self._last_t = None
        self._pending: tuple = ()
        self._failed: set = set()       # 재탐색이 실패한 객체
        self._impossible: set = set()   # 우회 불가 구간에서 막은 객체
        self._retry_stopped: set = set()  # 정지 후에 다시 시도할 객체
        self._announced: set = set()
        self._lr_acc = 0.0
        self._lr_id = -1
        self._ld_on = False
        self._ld_until = 0.0
        self._ld_ok = 0

    def _persist(self, oid: int) -> '_Hist':
        """영속 기록을 연다.

        **_failed/_impossible에 넣을 때는 반드시 이걸 거친다.** 정리 루프가
        _hist를 훑으므로, 항목 없이 집합에만 넣으면 그 id는 영영 지워지지 않고
        (우회 불가로 한 번 찍힌 객체를 지나쳐도) 다시 시도할 수 없게 된다.
        """
        h = self._hist.get(oid)
        if h is None:
            h = self._hist[oid] = _Hist()
        return h

    def _announce(self, key, kind: str, detail: str) -> None:
        if key in self._announced:
            return
        self._announced.add(key)
        self._on_event(kind, detail)

    def rebind(self, path, rows: list, zones: list, bounds: list,
               stoplines=None) -> None:
        """경로 교체 후. **경로만 바뀐 것이지 주변 상황이 바뀐 것이 아니다.**

        reset()을 부르지 않는다. 객체 추적(_blocks)은 월드 좌표(x,y)라 그대로
        유효하고, 경로 좌표계 값(s/lat/d/corridor)은 다음 프레임 _track이 새
        경로로 재투영한다 -- update() 안에서 _track이 _classify/_last_resort보다
        먼저 돌고, poll()->rebind()와 update() 사이에 _B.s/lat/d를 읽는 곳이
        없어 경합이 없다. 새 경로에서 안 잡히는 객체는 missing/TRACK_GRACE로
        자연히 빠진다.

        예전엔 여기서 reset()을 불러 _blocks를 비웠다. 그래서 옆 차로로 우회해
        들어간 뒤 그 차로의 정지 차량을 acc 0부터 다시 셌고(AMBIG 4초), 실차에서
        재우회가 4배 느리거나 안 걸렸다 (9/11 sc_jam). 확인 타이머·맥락(tier)·
        동결 시간·시도 횟수(_hist)·실패/불가 표시·재시도 게이트·이벤트 중복
        억제·lookahead 상향·최후 안전망 래치 전부 승계한다.

        최후 안전망 래치(_lr_id)는 예전엔 _blocks가 비어 첫 프레임에 자동
        해제됐는데 이제 "그 객체를 지났을 때"(b.d < -LR_EXIT_M)로 해제된다 --
        설계 의도 그대로다.
        """
        self.path, self.rows, self.zones = path, rows, zones
        self.bounds = bounds
        if stoplines is not None:
            self.stoplines = list(stoplines)
        self._gate_s = FN.gate_s(path, self.finish_zone)
        self._pending = ()      # 이 교체를 만든 요청은 끝났다 (poll이 이미 비웠다)

    @property
    def ld_floor(self) -> float:
        """lookahead 하한. update 전에 읽어도 되도록 상태에서 바로 낸다."""
        return DETOUR_LD_M if self._ld_on else 0.0

    def zone_at(self, s: float):
        for z in self.zones:
            if z.contains(s):
                return z
        return None

    # -- 매 프레임 ------------------------------------------------------
    def update(self, idx: int, s_ego: float, v: float, ego_x: float,
               ego_y: float, objects, t: float, *, sig_stopping: bool = False,
               sig_green: bool = False, obs=None,
               finished: bool = False, sig_state: int = 0,
               sig_dist: float = float('nan')) -> DetourCmd:
        """sig_green은 **stoplines.GO_STATES = {3 녹색, 5 녹색+좌회전}**이다.
        sig_state는 받은 신호 상태 원값, sig_dist는 stoplines가 겨냥한 정지선까지의
        후륜축 거리(정지 명령 중이 아니면 nan). 둘 다 맥락 판단(_classify)에만 쓴다.

        4(좌회전 전용)는 여기 들어가지 않는다. 직진 차로에서 4는 실질적으로
        적색이고 그 판단은 stoplines의 기동 분류가 이미 한다 -- 직진이면
        _stop으로 가 sig_stopping이 참이 되고, 좌회전 기동이면 _go라 거짓이다.
        그래서 4를 위한 분기가 따로 필요 없다.

        **녹색인데 못 가고 있으면 그것은 신호 대기가 아니라 객체 교착이다.**
        정지선 35m 앞에서 차량에 막혔는데 sig.stopping이 참이라 확인 타이머와
        안전망 누적이 둘 다 멈춘 적이 있다. tl_go(GREEN+LEFT)가 난 뒤에도 차가
        움직이지 않은 것이 결정적 증거다.
        """
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        sig_wait = sig_stopping and not sig_green

        cands, moving, wide = self._candidates(idx, s_ego, ego_x, ego_y, objects)
        self._track(cands, moving, wide, dt, t)
        ctx = (s_ego, sig_wait, sig_green, sig_state, sig_dist)
        for b in self._blocks.values():
            self._classify(b, cands, wide, ctx)
            if b.id not in wide:
                continue
            # 옆 차로에 있는 동안에도 쌓는다 -- 우회해 들어간 차로의 정지 차량은
            # 이미 정지해 있던 것이고, 회랑에 들어오는 순간 발동 대상이 된다
            if b.tier == TIER_FREEZE:
                b.frozen += dt
            else:
                b.acc += dt

        cmd = DetourCmd()
        self._last_resort(v, dt, sig_wait, obs, finished, cmd)
        if self._ld_on:
            cmd.ld_min = DETOUR_LD_M
            self._ld_check(idx, ego_x, ego_y, t)

        if cmd.action == 'last_resort':
            return cmd
        if self._pending:
            cmd.action, cmd.reason = 'search', '재탐색 중'
            b = self._blocks.get(self._pending[0])
            if b is not None:
                cmd.obj_id, cmd.dist, cmd.kind = b.id, b.d, b.kind
            return cmd

        ready = self._ready(v, t)
        if ready is not None:
            self._trigger(ready, s_ego, idx, ego_x, ego_y, v, t, cmd)
        else:
            inside = [x for x in self._blocks.values() if x.corridor]
            if inside:
                b = max(inside, key=lambda x: x.acc)
                cmd.action, cmd.obj_id, cmd.dist, cmd.kind = (
                    'watch', b.id, b.d, b.kind)
                cmd.reason = (f'막힘 확인 {b.acc:.1f}/{b.confirm:.1f}s '
                              f'({b.tier}: {b.why})')
        return cmd

    # -- 후보 추출 ------------------------------------------------------
    def _candidates(self, idx: int, s_ego: float, ego_x: float,
                    ego_y: float, objects) -> tuple:
        """(회랑 안 후보, 그 자리에서 움직이는 객체 id, 옆 차로까지 넓힌 정지 객체).

        obstacles의 투영을 재사용한다. 두 번째 집합은 "후보였을 자리인데 지금
        움직이고 있는" 객체다 -- 그 프레임에 확인 시간을 버린다.

        셋째(wide)는 |lat| < WIDE_LAT_M 안의 정지 객체 전부고 첫째(cands)는 그중
        |lat| < BLOCK_CORRIDOR_M인 부분집합이다. 발동은 cands만, 페널티·군집
        검증·acc 선적립은 wide로 한다 (9/11 sc_jam).
        """
        out, moving, wide = {}, set(), {}
        if not objects:
            return out, moving, wide
        ahead = min(self.lookahead, DETOUR_CLEAR_AHEAD_M)
        i_lo, i_hi = OB._window(self.path, idx, 2.0, ahead + 10.0)
        for o in objects:
            if math.hypot(o.x - ego_x, o.y - ego_y) > ahead + OB.PREFILTER_M:
                continue
            s_obj, lat, _th = OB._project(self.path, i_lo, i_hi, o.x, o.y)
            d = s_obj - s_ego
            if d <= 0.0 or d > ahead:
                continue
            if abs(lat) >= WIDE_LAT_M:
                continue
            if abs(o.speed) >= STOPPED_MS:
                moving.add(o.id)    # 움직이는 객체는 대상이 아니다
                continue
            cls = OB.classify(o)
            if cls == OB.OBSTACLE and s_obj >= self._gate_s:
                continue            # 종료 라바콘. obstacles와 같은 기준으로 뺀다
            wide[o.id] = (o.x, o.y, s_obj, lat, d, cls)
            if abs(lat) < BLOCK_CORRIDOR_M:
                out[o.id] = wide[o.id]
        return out, moving, wide

    def _track(self, cands: dict, moving: set, wide: dict, dt: float,
               t: float) -> None:
        for oid in list(self._blocks):
            if oid in wide:
                continue
            b = self._blocks[oid]
            if oid in moving:
                # **움직이면 그 프레임에 버린다.** 조금이라도 움직였으면 곧
                # 지나갈 상대지 장애물이 아니다 (특히 횡단 중 멈췄던 보행자).
                # 아래 grace는 검출 드롭아웃용이지 이동용이 아니다.
                # 옆 차로 객체는 조용히 버린다 -- 알린 적이 없는 대상이다
                if b.corridor:
                    self._on_event('detour_context',
                                   f'{b.kind} id={oid} 이동 감지 (확인 {b.acc:.1f}s '
                                   f'중) ― 우회 판단을 처음부터 다시 한다')
                del self._blocks[oid]
                continue
            b.missing += 1
            if b.missing <= OB.TRACK_GRACE:
                continue        # 한 프레임 튀었다고 확인 시간을 버리지 않는다
            del self._blocks[oid]

        # 객체별 기록은 _blocks보다 오래 산다. **경로를 갈아 끼우면 _blocks는
        # 통째로 비지만 _hist는 남으므로 여기서만 정리한다** -- 위 드롭아웃
        # 루프는 TRACK_GRACE 프레임짜리라 교체 뒤에는 아예 돌지 않는다.
        for oid, h in list(self._hist.items()):
            if oid in cands:
                h.gone_s = 0.0
                continue
            h.gone_s += dt
            if h.gone_s < TRIES_RESET_S:
                continue
            del self._hist[oid]         # 지나갔다 = 정상 주행 복귀
            self._failed.discard(oid)
            self._impossible.discard(oid)
            self._retry_stopped.discard(oid)
        for oid, (x, y, s, lat, d, kind) in wide.items():
            b = self._blocks.get(oid)
            if b is None:
                b = self._blocks[oid] = _B(oid, x, y, s, lat, d, kind, t)
                b.corridor = oid in cands
                continue
            if math.hypot(x - b.x, y - b.y) > OB.ID_JUMP_M:
                # 같은 id지만 다른 객체다. 확인 시간을 처음부터 다시 센다.
                self._announce(('reuse', oid), 'obj_id_reuse',
                               f'id={oid} 점프 ― 우회 확인 시간을 리셋한다')
                b = self._blocks[oid] = _B(oid, x, y, s, lat, d, kind, t)
                b.corridor = oid in cands
                continue
            b.x, b.y, b.s, b.lat, b.d, b.kind = x, y, s, lat, d, kind
            b.corridor = oid in cands
            b.missing = 0

    # -- 맥락 판단 ------------------------------------------------------
    def _next_stopline(self, s: float):
        """s 이후(2m 뒤까지) 첫 정지선. 없으면 None."""
        best = None
        for sl in self.stoplines:
            if sl.s >= s - 2.0 and (best is None or sl.s < best.s):
                best = sl
        return best

    def _chain(self, b: '_B', cands: dict) -> int:
        """b가 속한 정지 차량 군집의 크기 (앞뒤로 CHAIN_GAP_M 이하 간격, b 포함).

        앞쪽만 세면 대기열의 **맨 앞 차**가 단독으로 보여 4초 만에 우회를 건다.
        """
        ss = sorted(s for _x, _y, s, _lat, _d, kind in cands.values()
                    if kind == OB.VEHICLE)
        if not ss:
            return 0
        i0 = min(range(len(ss)), key=lambda i: abs(ss[i] - b.s))
        lo = hi = i0
        while lo > 0 and ss[lo] - ss[lo - 1] <= CHAIN_GAP_M:
            lo -= 1
        while hi < len(ss) - 1 and ss[hi + 1] - ss[hi] <= CHAIN_GAP_M:
            hi += 1
        return hi - lo + 1

    def _near_junction(self, s: float) -> bool:
        """s가 교차로 내부이거나 앞 JUNCTION_NEAR_M 안에 교차로 진입이 있나."""
        if not self.injunc or not self.rows:
            return False
        j = min(max(bisect.bisect_left(self.path.s, s), 0), len(self.rows) - 1)
        k1 = min(self.path._advance(j, JUNCTION_NEAR_M), len(self.rows) - 1)
        for k in range(j, k1 + 1):
            if self.rows[k].node in self.injunc:
                return True
        return False

    def _classify(self, b: '_B', cands: dict, wide: dict, ctx: tuple) -> None:
        """막힘 후보의 티어(확인 시간)를 맥락으로 정한다. 매 프레임 다시 본다.

        FREEZE  적색·황색 정지 명령 중이고 객체가 그 정지선 앞 -- 적색 대기열.
                타이머를 얼린다 (옛 동작 그대로). 녹색이 되면 아래 FAST로 간다
        FAST    녹색인데 안 움직인다 / 신호·정지선·교차로 없는 단독 정지
        QUEUE   신호 정지선 앞인데 신호가 NONE(미획득) / 교차로 근접 연쇄
        AMBIG   그 밖 -- 정지선·교차로 근처, 연쇄, 보행자

        옆 차로 객체도 티어를 매긴다(회랑에 들어오는 순간 확인 시간이 맞아야
        한다). 연쇄는 **그 객체의 차로대** 안에서 센다. 다만 detour_context는
        회랑 안에서만 낸다 -- check_ttc의 지연 측정이 회랑 진입 시점부터다.
        """
        s_ego, sig_wait, sig_green, sig_state, sig_dist = ctx
        sl = self._next_stopline(b.s)
        near_sl = sl is not None and sl.s - b.s <= QUEUE_SL_M
        if b.corridor:
            pool = cands
        else:
            pool = {k: v for k, v in wide.items()
                    if abs(v[3] - b.lat) < BLOCK_CORRIDOR_M}
        chain = self._chain(b, pool)
        junc = self._near_junction(b.s)
        sl_txt = (f'{"신호" if sl.signalized else "비신호"} 정지선 {sl.s - b.s:.0f}m 앞'
                  if near_sl else '정지선 없음')
        if (sig_wait and (sig_dist != sig_dist or b.s <= s_ego + sig_dist + 2.0)
                and b.frozen < FREEZE_MAX_S):
            tier, why = TIER_FREEZE, f'적색 대기열 ({sl_txt}, 앞차 {chain}대 연쇄, 대기)'
        elif sig_wait and b.frozen >= FREEZE_MAX_S:
            tier, why = TIER_QUEUE, (f'적색 대기열 동결 {FREEZE_MAX_S:.0f}s 초과 '
                                     f'({sl_txt}, 앞차 {chain}대 연쇄)')
        elif sig_green:
            tier, why = TIER_FAST, f'녹색인데 정지 ({sl_txt}, 앞차 {chain}대)'
        elif near_sl and sl.signalized and sig_state == 0:
            tier, why = TIER_QUEUE, f'신호 미획득 ({sl_txt}, 앞차 {chain}대 연쇄)'
        elif junc and chain >= 2:
            tier, why = TIER_QUEUE, f'교차로 근접 연쇄 (앞차 {chain}대)'
        elif near_sl or junc or chain >= 2:
            tier, why = TIER_AMBIG, (f'{sl_txt}, 앞차 {chain}대'
                                     + (', 교차로 근접' if junc else ''))
        else:
            tier, why = TIER_FAST, f'신호 없음, 앞차 단독, {b.d:.0f}m 전방'
        if b.kind == OB.PEDESTRIAN and TIER_RANK[tier] < TIER_RANK[TIER_AMBIG]:
            tier, why = TIER_AMBIG, f'정지 보행자 ({why})'
        confirm = CONFIRM_BY_TIER[tier]
        if self.confirm_s is not None and tier != TIER_FREEZE:
            confirm = self.confirm_s
        if b.corridor and (tier != b.tier or not b.told):
            label = {TIER_FREEZE: '대기열 판정', TIER_QUEUE: '대기열 판정',
                     TIER_AMBIG: '판단 애매', TIER_FAST: '장애물 판정'}[tier]
            self._announce(('ctx', b.id, tier), 'detour_context',
                           f'{label} ― {b.kind} id={b.id} {why}'
                           + (f' (확인 {confirm:.1f}s)' if confirm != float('inf')
                              else '')
                           + (f' [옆 차로에서 {b.acc:.1f}s 선적립]'
                              if not b.told and b.acc > 0.0 else ''))
            b.told = True
        b.tier, b.confirm, b.why = tier, confirm, why

    def _ready(self, v: float, t: float):
        """재탐색을 걸 대상. 없으면 None."""
        if self.planner is None or self.planner.busy:
            return None
        if self.n_detours >= MAX_DETOURS:
            return None
        best = None
        for b in self._blocks.values():
            if not b.corridor or b.acc < b.confirm:
                continue        # 옆 차로 객체는 발동 대상이 아니다 (추적만)
            h = self._hist.get(b.id)
            # 같은 객체를 여기까지 시도했으면 답이 바뀌지 않는다. **경로를
            # 갈아 끼워도 이 횟수는 남는다** (_Hist 참고).
            if h is not None and h.tries >= MAX_TRIES_PER_OBJ:
                continue
            if b.id in self._impossible:
                continue
            # 실패 후 재시도는 **정지한 뒤에만** 한다. 접근 중 같은 탐색을
            # 반복해 봐야 같은 답이 나오고, 정지 후에는 조건이 달라진다.
            if b.id in self._retry_stopped and v >= STOPPED_MS:
                continue
            if h is not None and h.tries and t - h.last_try < RETRY_MIN_S:
                continue
            if best is None or b.d < best.d:
                best = b
        return best

    def _trigger(self, b: _B, s_ego: float, idx: int, ego_x: float,
                 ego_y: float, v: float, t: float, cmd: DetourCmd) -> None:
        cmd.obj_id, cmd.dist, cmd.kind = b.id, b.d, b.kind
        if b.tier == TIER_QUEUE:
            self._announce(('ctx-late', b.id), 'detour_context',
                           f'대기열이었으나 {b.acc:.1f}초 경과 ― 우회 시도 '
                           f'({b.kind} id={b.id} {b.why})')
        z = self.zone_at(b.s)
        if b.s >= self._gate_s:
            # 종료 구간. 우회하면 종료선을 벗어나 완주가 아니다. 우회 불가
            # 구간과 같은 경로를 타서 최후 안전망만 남긴다
            self._persist(b.id)
            self._impossible.add(b.id)
            cmd.action = 'impossible'
            cmd.reason = '종료 구간'
            self._announce(
                ('imp', b.id), 'detour_impossible',
                f'종료 구간에서 막힘 ― 우회하면 종료선을 벗어난다. 크립 유지 '
                f'(경로 끝 {self.finish_zone:.0f}m 이내, s>={self._gate_s:.1f}, '
                f'obj id={b.id} {b.kind} d={b.d:.1f}m lat={b.lat:+.1f}m)')
            return
        if z is not None:
            self._persist(b.id)
            self._impossible.add(b.id)
            cmd.action = 'impossible'
            cmd.reason = '우회 불가 구간'
            self._announce(
                ('imp', b.id), 'detour_impossible',
                f'우회 불가 구간에서 막힘 ― 크립 유지 '
                f'(s {z.s0:.1f}~{z.s1:.1f} {z.length:.1f}m, {z.label()}, '
                f'obj id={b.id} {b.kind} d={b.d:.1f}m lat={b.lat:+.1f}m)')
            return

        h = self._persist(b.id)
        h.tries += 1
        h.last_try = t
        h.gone_s = 0.0
        self._retry_stopped.discard(b.id)
        # 같은 노드를 막은 객체는 함께 피해야 한다 (줄지어 선 정지 차량).
        # **노드당 객체 수**를 센다 -- 페널티가 객체 1대당 가산이라 정지 차량이
        # 적은 차로가 이긴다
        group = [x for x in self._blocks.values()
                 if x.corridor and (x.acc >= x.confirm or x.id == b.id)]
        blocked: dict = {}
        obs_xy = []
        for x in group:
            n_at, ohdg = self._at_s(x.s)
            blocked[n_at] = blocked.get(n_at, 0) + 1
            obs_xy.append((x.x, x.y,
                           CLEAR_BY_KIND.get(x.kind, OB.VEH_LANE_M + 0.3),
                           ohdg))

        # **옆 차로의 정지 객체.** 회랑 그룹과 같은 군집(s 간격 CHAIN_GAP_M 이하로
        # 이어짐)이면 새 경로가 그것도 비켜 가야 한다(hard). 군집 밖은 페널티만 --
        # 60m 앞 주차 차량 하나 때문에 모든 해가 실패하면 안 되고, 그 차는
        # 회랑에 들어오는 순간 (acc가 이미 차 있어) 곧바로 재우회된다.
        # 차로 노드는 워커가 LaneIndex로 푼다 (다른 도로·대향 차로면 페널티 없음)
        rest = [x for x in self._blocks.values() if not x.corridor]
        cluster = [x.s for x in group]
        hard = set()
        grew = True
        while grew:
            grew = False
            for x in rest:
                if x.id in hard:
                    continue
                if any(abs(x.s - s0) <= CHAIN_GAP_M for s0 in cluster):
                    hard.add(x.id)
                    cluster.append(x.s)
                    grew = True
        wide_obs = []
        for x in rest:
            n_at, ohdg = self._at_s(x.s)
            wide_obs.append({'id': x.id, 'x': x.x, 'y': x.y, 's': x.s,
                             'lat': x.lat, 'kind': x.kind, 'ref_node': n_at,
                             'ohdg': ohdg,
                             'need': CLEAR_BY_KIND.get(x.kind, OB.VEH_LANE_M + 0.3),
                             'hard': x.id in hard})

        cps, seg_off = self._checkpoints(s_ego, idx)
        if len(cps) < 2:
            cmd.action, cmd.reason = 'impossible', '남은 체크포인트가 없다'
            return
        self._pending = tuple(x.id for x in group)
        cmd.action, cmd.reason = 'search', '재탐색 요청'
        # check_ttc가 첫 `id=`, `v=..km/h`, `({tier}:`를 정규식으로 읽는다
        self._on_event('detour_start',
                       f'{b.kind} id={b.id} 정지 {b.acc:.1f}s/{b.confirm:.1f}s '
                       f'({b.tier}: {b.why}; d={b.d:.1f}m lat={b.lat:+.1f}m, '
                       f'대상 {len(group)}개, v={v * 3.6:.1f}km/h) '
                       f'― {dict(sorted(blocked.items()))} 우회 재탐색'
                       + (f', 옆 차로 정지 {len(wide_obs)}대'
                          f'(군집 {len(hard)}대 검증)' if wide_obs else '')
                       + f' [{self.n_detours + 1}/{MAX_DETOURS}, '
                       f'이 객체 {h.tries}/{MAX_TRIES_PER_OBJ}회]')
        self.planner.request({
            'cps': cps, 'blocked': blocked, 'obs_xy': obs_xy,
            'wide_obs': wide_obs,
            'seg_offset': seg_off, 'obj_ids': list(self._pending),
            'ego_xy': (ego_x, ego_y), 'ego_v': v,
            'ego_node': self.rows[min(idx, len(self.rows) - 1)].node,
            'goal_node': self.rows[-1].node})

    def _at_s(self, s: float) -> tuple:
        """경로 s의 (노드, 접선 헤딩). 객체 헤딩은 그 자리의 경로 접선으로 본다
        (차로 안의 차량이다 -- 옆 차로도 같은 도로라 같은 접선이다)."""
        k = min(max(bisect.bisect_left(self.path.s, s), 0), len(self.rows) - 1)
        j0 = max(0, min(k, len(self.path.pts) - 2))
        ohdg = math.atan2(self.path.pts[j0 + 1][1] - self.path.pts[j0][1],
                          self.path.pts[j0 + 1][0] - self.path.pts[j0][0])
        return self.rows[k].node, ohdg

    def _checkpoints(self, s_ego: float, idx: int) -> tuple:
        """[현재 위치] + [남은 구간 경계] + [경로 끝].

        구간 경계점이 곧 원래 route CSV의 경유지다. 차로 중심선 위의 점이라
        스냅이 정확하고, snap_candidates가 "가장 가까운 도로+섹션의 driving
        차로 전부"를 열므로 옆 차로 해가 막히지 않는다. --route가 없어도 되고
        --segments가 부분집합이어도 그대로 성립한다.
        """
        pts = self.path.pts
        cps = [pts[idx]]
        seg_off = 0
        for name, start in self.bounds:
            if self.path.s[start] > s_ego + CP_MIN_AHEAD_M:
                cps.append(pts[start])
            elif self.path.s[start] <= s_ego:
                seg_off = max(seg_off, _seg_index(name) - 1)
        # 끝점은 **실제** 끝이다. 가상 연장 점은 차로 밖에 있을 수 있어
        # 경유지로 넣으면 스냅이 어긋난다
        fi = FN.finish_index(self.path)
        if self.path.s[fi] - s_ego > CP_MIN_AHEAD_M:
            cps.append(pts[fi])
        return cps, seg_off

    # -- 결과 수령 ------------------------------------------------------
    def poll(self):
        """워커 결과를 받는다. 경로를 갈아 끼울 Plan이거나 None."""
        if self.planner is None:
            return None
        plan = self.planner.poll()
        if plan is None:
            return None
        self._pending = ()
        if not plan.ok:
            for oid in plan.obj_ids:
                self._persist(oid)   # 그룹 동승 객체도 정리 대상에 올린다
                self._failed.add(oid)
                self._retry_stopped.add(oid)
            self._on_event('detour_failed',
                           f'{plan.reason} ― 기존 거동 유지 '
                           f'(obj {plan.obj_ids})')
            return None
        self.n_detours += 1
        self._on_event(
            'detour_done',
            f'우회로 {len(plan.rows)}점 {plan.bundle["path"].length:.1f}m, '
            f'차선변경 {plan.n_lc}회'
            + (f' (실선 {plan.solid_lc}회)' if plan.solid_lc else '')
            + (f', 출발 횡간격 {plan.start_lat:+.2f}m를 {plan.blend_m:.1f}m에 '
               f'걸쳐 붙인다'
               + (f'({plan.blend_dir} 점등)' if plan.blend_dir else '')
               if abs(plan.start_lat) > 0.05 else '')
            + (f', 첫 차선변경 마커 {plan.first_lc_m:.1f}m 앞'
               if plan.first_lc_m == plan.first_lc_m else '')
            + (f', 옆 차로 정지 {plan.n_wide}대 반영' if plan.n_wide else '')
            + f' -> {os.path.basename(plan.csv_path)}')
        if plan.tight:
            self._ld_on = True
            self._ld_until = (self._last_t or 0.0) + LD_BOOST_MAX_S
            self._ld_ok = 0
            self._on_event(
                'detour_tight',
                f'비켜 나갈 여유가 {plan.blend_m:.1f}m뿐이다 '
                f'(기준 {LC_ROOM_M:.0f}m) ― 조향 포화를 막으려고 lookahead '
                f'하한을 {DETOUR_LD_M:.0f}m로 올린다')
            self._on_event('detour_ld_boost', f'lookahead >= {DETOUR_LD_M:.0f}m')
        return plan

    def _ld_check(self, idx: int, ego_x: float, ego_y: float,
                  t: float) -> None:
        """해제는 **경로에 복귀했을 때**다. "움직이기 시작했을 때"로 잡으면
        obstacles의 크립 교착·crosswalks의 탈출 자기취소와 같은 실수가 된다."""
        off = math.hypot(self.path.pts[idx][0] - ego_x,
                         self.path.pts[idx][1] - ego_y)
        self._ld_ok = self._ld_ok + 1 if off < LD_RESTORE_M else 0
        if self._ld_ok >= LD_RESTORE_N or t > self._ld_until:
            self._ld_on = False
            self._on_event('detour_ld_restore',
                           f'경로 복귀 (이탈 {off:.2f}m) ― lookahead 원복'
                           if self._ld_ok >= LD_RESTORE_N
                           else f'{LD_BOOST_MAX_S:.0f}s 상한 ― lookahead 원복')

    # -- 최후 안전망 ----------------------------------------------------
    def _last_resort(self, v: float, dt: float, sig_wait: bool, obs,
                     finished: bool, cmd: DetourCmd) -> None:
        """우회로 못 푸는 교착에서 저속으로 밀고 통과한다.

        해제는 **그 객체를 지났을 때**다. 속도가 붙었는지로 잡으면 스스로를
        취소한다 (obstacles 크립 교착 / crosswalks 탈출 자기취소와 같은 실수).
        """
        if not self.use_last_resort:
            return
        if self._lr_id >= 0:
            b = self._blocks.get(self._lr_id)
            gone = b is None
            if not gone and b.d < -LR_EXIT_M:
                gone = True
            if gone or finished:
                self._on_event('last_resort_clear',
                               f'id={self._lr_id} 통과 ― 정상 복귀')
                self._lr_id, self._lr_acc = -1, 0.0
                return
            cmd.action = 'last_resort'
            cmd.v_floor = LAST_RESORT_V
            cmd.obj_id, cmd.dist, cmd.kind = b.id, b.d, b.kind
            cmd.reason = f'{LAST_RESORT_V * 3.6:.0f}km/h로 통과 중'
            return

        stuck = None
        for b in self._blocks.values():
            if b.id not in self._failed and b.id not in self._impossible:
                continue
            if b.d > LR_NEAR_M:
                continue
            if obs is not None and obs.obj_id != b.id:
                continue        # 우리를 세우고 있는 것이 이 객체인가
            if stuck is None or b.d < stuck.d:
                stuck = b
        if stuck is None or v >= STOPPED_MS or finished:
            self._lr_acc = 0.0
            return
        # 적색 앞 정상 대기를 막힘으로 오판하면 항목 7 중대 -6점이다.
        # **리셋이 아니라 동결이다.** 적색 25초/녹색 25초가 번갈면 리셋으로는
        # 10초를 채우기 전에 매번 처음으로 돌아가 영영 발동하지 않는다.
        # 발동 조건은 매 프레임 다시 보므로(그 프레임이 신호 대기면 아래에서
        # 돌아간다) 얼려 둔 누적값이 적색 통과를 만들지는 않는다.
        if sig_wait:
            return
        self._lr_acc += dt
        if self._lr_acc < self.last_resort_s:
            return
        self._lr_id = stuck.id
        cmd.action = 'last_resort'
        cmd.v_floor = LAST_RESORT_V
        cmd.obj_id, cmd.dist, cmd.kind = stuck.id, stuck.d, stuck.kind
        cmd.reason = '최후 안전망'
        self._on_event(
            'last_resort_pass',
            f'{stuck.kind} id={stuck.id} 앞 {stuck.d:.1f}m에서 '
            f'{self._lr_acc:.0f}s 정지 ― 미완주를 막으려고 '
            f'{LAST_RESORT_V * 3.6:.0f}km/h로 밀고 통과한다 '
            f'({"우회 불가 구간" if stuck.id in self._impossible else "우회 실패"}, '
            f'신호 대기 아님)')


def summary(policy: DetourPolicy) -> list:
    conf = (f'확인 {policy.confirm_s:.1f}s (균일)' if policy.confirm_s is not None
            else f'확인 {DETOUR_CONFIRM_FAST_S:.1f}/{DETOUR_CONFIRM_S:.1f}/'
                 f'{DETOUR_CONFIRM_QUEUE_S:.1f}s (장애물/애매/대기열, 적색 동결)')
    out = [f' 재탐색 {"활성" if policy.planner else "비활성"}, {conf}, '
           f'정지선 {len(policy.stoplines)}개·교차로 노드 {len(policy.injunc)}개 참조, '
           f'최후 안전망 '
           + (f'{policy.last_resort_s:.0f}s' if policy.use_last_resort else '꺼짐')
           + f', 종료 구간 {policy.finish_zone:.0f}m (우회 금지·라바콘 무시)']
    out.extend(zone_report(policy.path, policy.zones))
    return out


# ----------------------------------------------------------------------
# 오프라인 -- 경로 진단
# ----------------------------------------------------------------------

def _data_dir() -> str:
    return os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'data'))


def _plan(args) -> int:
    """--plan: 경로의 우회 가능성을 진단한다.

    **대회 당일 경로 CSV를 받은 직후, 4번 단계(시뮬레이션 시작) 전에 돌린다.**
    """
    import pure_pursuit as PP
    import xodr_graph as G

    data = _data_dir()
    segs = ([s.strip() for s in args.segments.split(',')] if args.segments
            else None)
    if segs is None:
        with open(args.path, newline='', encoding='utf-8') as fh:
            seen = []
            for r in csv.DictReader(fh):
                if r['segment_id'] not in seen:
                    seen.append(r['segment_id'])
            segs = seen
    path, bounds = PP.load_path(args.path, segs)
    rows = load_rows(args.path, segs)
    bad = check_rows(path, rows)
    if bad:
        print(f'[det] 우회 비활성: {bad}')
        return 1

    pts, dropped = G.load_lane_points(args.lanes or
                                      os.path.join(data, 'xodr_lanes.csv'))
    g = G.load_graph(args.graph_edges or
                     os.path.join(data, 'xodr_graph_edges.csv'), pts)
    has_lc, injunc = lanechange_nodes(g), junction_nodes(g)
    zones = scan_undetourable(path, rows, g, has_lc, injunc)

    gs = G.Graph(pts=pts, edges={n: list(es) for n, es in g.edges.items()})
    n_solid = add_solid_lanechanges(gs)
    z_solid = scan_undetourable(path, rows, gs, lanechange_nodes(gs), injunc)
    solid_m = sum(z.length for z in z_solid)

    print(f'[det] {os.path.basename(args.path)}  {path.n_real}점 '
          f'{path.finish_s:.1f}m  구간 {len(segs)}개')
    print(f'[det] 그래프 노드 {len(pts)} (끝점 이상치 {dropped}점 제거), '
          f'실선 차선변경 엣지 {n_solid}개 합성 가능')
    for line in zone_report(path, zones, solid_m):
        print('[det]' + line)
    if zones:
        print('[det] 위 구간에서 정지 차량에 막히면 최후 안전망'
              f'(정지 {LAST_RESORT_S:.0f}초 뒤 {LAST_RESORT_V * 3.6:.0f}km/h '
              '통과)만이 완주 수단이다')
    return 0


# ----------------------------------------------------------------------
# 자기검사 -- 로그에 없는 상황을 합성한다
# ----------------------------------------------------------------------

class _Straight:
    """300m 직선. obstacles._Straight와 같은 계약(pts/s/length/_advance)."""

    def __init__(self, n=1000, step=0.3, y0=0.0):
        self.pts = [(i * step, y0) for i in range(n)]      # y0: 옆 차로 경로
        self.s = [i * step for i in range(n)]
        self.length = self.s[-1]

    def _advance(self, i, dist_m):
        target = self.s[i] + dist_m
        j = i
        while j < len(self.pts) - 1 and self.s[j] < target:
            j += 1
        return j

    def nearest(self, x, y, start=None):
        return min(range(len(self.pts)),
                   key=lambda i: math.hypot(self.pts[i][0] - x,
                                            self.pts[i][1] - y))


class _Fake:
    """hlvtd_client.Obj와 같은 __slots__. obstacles._Fake와 같은 형태."""

    __slots__ = ('id', 'x', 'y', 'z', 'heading', 'speed',
                 'length', 'width', 'height')

    def __init__(self, oid, x, y, speed=0.0, dims=(4.39, 1.81, 1.35)):
        self.id, self.x, self.y = oid, x, y
        self.z = self.heading = 0.0
        self.speed = speed
        self.length, self.width, self.height = dims


CAR = (4.39, 1.81, 1.35)
PED = (0.60, 0.70, 1.80)
CONE = (0.30, 0.30, 0.32)


class _Rec:
    """이벤트 기록기. crosswalks._Rec와 같은 형태."""

    def __init__(self):
        self.log = []

    def __call__(self, kind, detail):
        self.log.append((kind, detail))

    def kinds(self):
        return [k for k, _d in self.log]


class _StubPlanner:
    """워커 없이 결과를 즉시 돌려준다. 트리거만 검사할 때 쓴다."""

    def __init__(self, ok=False, reason='합성 실패'):
        self.ok, self.reason = ok, reason
        self.jobs = []
        self._out = []

    @property
    def busy(self):
        return False

    def request(self, job):
        self.jobs.append(job)
        self._out.append(Plan(self.ok, self.reason,
                              obj_ids=job.get('obj_ids') or []))

    def poll(self):
        return self._out.pop(0) if self._out else None


class _SL:
    """stoplines.Stopline 중 맥락 판단이 보는 것만."""

    __slots__ = ('s', 'signalized')

    def __init__(self, s, signalized=True):
        self.s, self.signalized = s, signalized


def _mk(zones=(), planner=None, stoplines=(), junc_s=None, **kw):
    """junc_s=(s0, s1)이면 그 구간을 교차로 내부 노드(road '9')로 만든다."""
    path = _Straight()
    rows = [Row(x, y, '1', 0, -1) for x, y in path.pts]
    injunc = None
    if junc_s is not None:
        for i, (x, y) in enumerate(path.pts):
            if junc_s[0] <= x <= junc_s[1]:
                rows[i] = Row(x, y, '9', 0, -1)
        injunc = {('9', 0, -1)}
    rec = _Rec()
    pol = DetourPolicy(path, rows, list(zones), [('p1_to_p2', 0)],
                       planner=planner, on_event=rec, stoplines=stoplines,
                       injunc=injunc, **kw)
    return path, pol, rec


def _run(pol, objs_at, n=400, dt=0.05, v=0.0, s0=0.0, t0=0.0, **kw):
    """등속(또는 정지) 주행을 흉내 낸다. (cmd 로그, 마지막 s, 마지막 t)."""
    log, s, t = [], s0, t0
    for _k in range(n):
        pol.poll()              # pure_pursuit 루프가 매 프레임 하는 일
        idx = min(int(s / 0.3), len(pol.path.pts) - 2)
        cmd = pol.update(idx, s, v, s, 0.0, objs_at(t, s), t, **kw)
        log.append((round(t, 2), round(s, 2), cmd.action, cmd.obj_id,
                    cmd.v_floor, cmd.ld_min))
        s += max(v, cmd.v_floor) * dt
        t += dt
    return log, s, t


def _selftest() -> int:
    fails = []

    def check(name, ok, detail=''):
        if not ok:
            fails.append(f'{name}: {detail}')
        print(f'  {"OK  " if ok else "FAIL"} {name}'
              + (f'   {detail}' if detail and not ok else ''))

    # 1. 확인 시간 -- 신호·정지선·교차로 없는 단독 정지 차량은 FAST(1.5s)
    _p, pol, rec = _mk(planner=_StubPlanner())
    car = _Fake(1, 40.0, 0.0)
    _run(pol, lambda t, s: [car], n=20)           # 1.0초
    check('확인 미달: 발동 안 함', not pol.planner.jobs,
          f'{len(pol.planner.jobs)}회 요청')
    check('장애물 판정 이벤트', any(k == 'detour_context' and '장애물 판정' in d
                                for k, d in rec.log), ','.join(rec.kinds()))
    _run(pol, lambda t, s: [car], n=20)           # 누적 2.0초
    check('확인 초과: 발동', len(pol.planner.jobs) == 1,
          f'{len(pol.planner.jobs)}회 요청')
    check('확인 초과: detour_start 발화', 'detour_start' in rec.kinds(),
          ','.join(rec.kinds()))
    check('detour_start에 속도 기록', any(k == 'detour_start' and 'v=' in d
                                      for k, d in rec.log))

    # 2. 신호 정지 중에는 타이머가 얼어붙는다 (적색 대기열 오탐 방지)
    _p, pol, rec = _mk(planner=_StubPlanner())
    _run(pol, lambda t, s: [_Fake(1, 40.0, 0.0)], n=400, sig_stopping=True)
    check('신호 정지 중: 발동 안 함', not pol.planner.jobs,
          f'{len(pol.planner.jobs)}회 요청 (20초 관측)')

    # 2b. **녹색인데 못 가면 신호 대기가 아니라 객체 교착이다.**
    #     정지선 35m 앞에서 차량에 막혔는데 sig.stopping이 참이라 확인 타이머가
    #     얼어붙어 영영 발동하지 못한 적이 있다.
    _p, pol, rec = _mk(planner=_StubPlanner())
    _run(pol, lambda t, s: [_Fake(1, 40.0, 0.0)], n=400,
         sig_stopping=True, sig_green=True)
    check('녹색인데 정지: 발동', bool(pol.planner.jobs),
          f'{len(pol.planner.jobs)}회 요청')
    # 4(좌회전 전용) 직진 차로는 stoplines가 _stop으로 보내므로 여기 계약은
    # sig_stopping=True / sig_green=False다 -- 위 2번과 같은 조합이다.

    # 3. 움직이는 객체는 대상이 아니다 (항목 10 중대 -6점 방지)
    _p, pol, rec = _mk(planner=_StubPlanner())
    _run(pol, lambda t, s: [_Fake(1, 40.0, 0.0, speed=3.6, dims=PED)], n=400)
    check('움직이는 보행자: 발동 안 함', not pol.planner.jobs)

    # 4. 차폭 경계
    for lat, want in ((1.4, True), (1.6, False)):
        _p, pol, _r = _mk(planner=_StubPlanner())
        _run(pol, lambda t, s: [_Fake(1, 40.0, lat)], n=400)
        got = bool(pol.planner.jobs)
        check(f'횡거리 {lat}m: {"발동" if want else "미발동"}', got == want)

    # 5. 종류 무관 -- 라바콘도 우회 대상이다
    for name, dims in (('라바콘', CONE), ('보행자', PED), ('차량', CAR)):
        _p, pol, _r = _mk(planner=_StubPlanner())
        _run(pol, lambda t, s: [_Fake(1, 40.0, 0.0, dims=dims)], n=400)
        check(f'{name} 정지: 발동', bool(pol.planner.jobs))

    # 6. id 재사용 -- 같은 id가 튀면 확인 시간을 다시 센다
    _p, pol, rec = _mk(planner=_StubPlanner())
    def jumpy(t, s):
        return [_Fake(1, 40.0 if t < 1.0 else 25.0, 0.0)]
    _run(pol, jumpy, n=40)                        # 2.0초 (1.0 + 1.0 < 1.5)
    check('id 재사용: 확인 리셋', not pol.planner.jobs,
          f'{len(pol.planner.jobs)}회 요청')
    check('id 재사용: obj_id_reuse 발화', 'obj_id_reuse' in rec.kinds())

    # 7. 우회 불가 구간이면 탐색조차 걸지 않고 사유를 남긴다
    z = Zone(0, 300, 0.0, 90.0, [('420', 0, -1)], {('420', 0, -1): '단일 차로'})
    _p, pol, rec = _mk(zones=[z], planner=_StubPlanner())
    _run(pol, lambda t, s: [_Fake(1, 40.0, 0.0)], n=400)
    check('우회 불가: 탐색 안 함', not pol.planner.jobs)
    check('우회 불가: detour_impossible 발화',
          'detour_impossible' in rec.kinds(), ','.join(rec.kinds()))
    det = [d for k, d in rec.log if k == 'detour_impossible']
    check('우회 불가: 구간 정보를 남긴다',
          det and '단일 차로' in det[0] and 's 0.0~90.0' in det[0],
          det[0] if det else '없음')
    # 지나치고 나면 그 낙인도 지운다. _hist 항목 없이 집합에만 넣으면 정리
    # 루프가 닿지 못해 영영 남고, 같은 id가 다시 막아도 시도조차 못 한다.
    check('우회 불가: 영속 기록을 함께 연다', 1 in pol._hist, f'{list(pol._hist)}')
    _run(pol, lambda t, s: [], n=int(TRIES_RESET_S / 0.05) + 4)
    check('우회 불가: 지나치면 낙인을 지운다',
          not pol._impossible and not pol._hist,
          f'imp={sorted(pol._impossible)} hist={list(pol._hist)}')

    # 7b. 종료 구간 -- 라바콘은 후보조차 아니고, 차량은 우회 대신 우회 불가로
    #     보내 최후 안전망만 남긴다. 구간 밖이면 예전처럼 우회를 건다
    _p, pol, rec = _mk(planner=_StubPlanner())
    cone = _Fake(1, 290.0, 0.0, dims=CONE)
    log, _s, _t = _run(pol, lambda t, s: [cone], n=400, s0=280.0)
    check('종료 구간 라바콘: 후보 아님',
          all(r[2] == 'none' for r in log) and not pol.planner.jobs,
          f'{sorted({r[2] for r in log})}')
    _p, pol, rec = _mk(planner=_StubPlanner())
    car = _Fake(2, 290.0, 0.0)
    obs = OB.ObstacleCmd(action='stop', obj_id=2)
    log, _s, _t = _run(pol, lambda t, s: [car], n=800, s0=280.0, obs=obs)
    check('종료 구간 차량: 탐색 안 함', not pol.planner.jobs,
          f'{len(pol.planner.jobs)}회 요청')
    check('종료 구간 차량: detour_impossible(종료 구간)',
          any(k == 'detour_impossible' and '종료 구간' in d for k, d in rec.log),
          ','.join(rec.kinds()))
    check('종료 구간 차량: 최후 안전망 발동', 'last_resort_pass' in rec.kinds(),
          ','.join(rec.kinds()))
    _p, pol, rec = _mk(planner=_StubPlanner())
    _run(pol, lambda t, s: [_Fake(3, 240.0, 0.0)], n=400, s0=230.0)
    check('종료 구간 밖 차량: 우회 시도', bool(pol.planner.jobs),
          ','.join(rec.kinds()))

    # 8. 최후 안전망 -- 우회 실패 + 20초 정지
    _p, pol, rec = _mk(planner=_StubPlanner(ok=False))
    obs = OB.ObstacleCmd(action='stop', obj_id=1)
    log, _s, _t = _run(pol, lambda t, s: [_Fake(1, 8.0, 0.0)], n=800, obs=obs)
    check('최후 안전망: 발동', 'last_resort_pass' in rec.kinds(),
          ','.join(sorted(set(rec.kinds()))))
    check('최후 안전망: v_floor 5km/h',
          any(abs(r[4] - LAST_RESORT_V) < 1e-6 for r in log))
    fire = [r[0] for r in log if r[2] == 'last_resort']
    check(f'최후 안전망: {LAST_RESORT_S:.0f}초 전에는 안 나온다',
          fire and fire[0] >= LAST_RESORT_S,
          f'첫 발동 t={fire[0] if fire else float("nan")}')

    # 8c. 적색 정지 중 누적은 **리셋이 아니라 동결이다.** 적색/녹색이 번갈면
    #     리셋으로는 LAST_RESORT_S를 채우기 전에 매번 처음으로 돌아간다.
    _p, pol, rec = _mk(planner=_StubPlanner(ok=False))
    o = lambda t, s: [_Fake(1, 8.0, 0.0)]
    _l, _s, t1 = _run(pol, o, n=120, obs=obs)                   # 6초: 확인 1.5 + 누적 4.5
    hold = pol._lr_acc
    _l, _s, t2 = _run(pol, o, n=400, obs=obs, t0=t1,            # 20초 적색 대기
                      sig_stopping=True)
    check('적색 대기: 누적 동결', abs(pol._lr_acc - hold) < 1e-9,
          f'{hold:.2f} -> {pol._lr_acc:.2f}')
    check('적색 대기: 발동 안 함', 'last_resort_pass' not in rec.kinds())
    _run(pol, o, n=int((LAST_RESORT_S - hold) / 0.05) + 4, obs=obs, t0=t2)
    check('적색 해제 후: 이어서 발동', 'last_resort_pass' in rec.kinds(),
          f'누적 {pol._lr_acc:.1f}s')

    # 8b. 게이트를 하나씩 깨면 발동하지 않는다
    for name, kw, ctor in (
            ('신호 정지 중', {'sig_stopping': True, 'obs': obs}, {}),
            ('완주 처리 중', {'finished': True, 'obs': obs}, {}),
            ('--no-last-resort', {'obs': obs}, {'use_last_resort': False}),
            ('obstacles가 다른 객체를 보고 있음',
             {'obs': OB.ObstacleCmd(action='stop', obj_id=99)}, {})):
        _p, pol, rec = _mk(planner=_StubPlanner(ok=False), **ctor)
        _run(pol, lambda t, s: [_Fake(1, 8.0, 0.0)], n=800, **kw)
        check(f'최후 안전망 미발동: {name}',
              'last_resort_pass' not in rec.kinds())

    # 9. 해제는 "객체를 지났을 때"다 -- 움직였다고 풀리면 안 된다
    #    (obstacles 크립 교착 / crosswalks 탈출 자기취소와 같은 실수)
    _p, pol, rec = _mk(planner=_StubPlanner(ok=False))
    log, end, _t = _run(pol, lambda t, s: [_Fake(1, 8.0, 0.0)], n=1000, obs=obs)
    acts = [r[2] for r in log]
    first = acts.index('last_resort')
    tail = acts[first:first + 40]
    check('최후 안전망: 움직여도 유지', all(a == 'last_resort' for a in tail),
          f'{sorted(set(tail))}')
    check('최후 안전망: 객체를 지나 통과', end > 8.0, f'{end:.1f}m 진행')
    check('최후 안전망: 통과 후 해제', 'last_resort_clear' in rec.kinds(),
          ','.join(sorted(set(rec.kinds()))))
    check('최후 안전망: 해제 후 원상복귀', acts[-1] == 'none', acts[-1])

    # 10. lookahead 보정 -- 해제는 "경로 복귀"다
    _p, pol, rec = _mk(planner=_StubPlanner())
    pol.planner._out.append(Plan(True, bundle={'path': _Straight()},
                                 rows=[], zones=[], csv_path='detour_1.csv',
                                 tight=True, first_lc_m=6.0, n_lc=1,
                                 start_lat=3.2, blend_m=8.0))
    pol._last_t = 0.0
    plan = pol.poll()
    check('촉박 교체: detour_tight 발화', 'detour_tight' in rec.kinds())
    check('촉박 교체: ld_boost 발화', 'detour_ld_boost' in rec.kinds())
    cmd = pol.update(0, 0.0, 0.0, 0.0, 5.0, [], 0.1)     # 경로에서 5m 벗어남
    check('촉박 교체: lookahead 상향', cmd.ld_min == DETOUR_LD_M,
          f'{cmd.ld_min}')
    for k in range(LD_RESTORE_N + 2):
        cmd = pol.update(0, 0.0, 0.0, 0.0, 5.0, [], 0.2 + k * 0.05)
    check('경로 밖: 상향 유지', cmd.ld_min == DETOUR_LD_M, f'{cmd.ld_min}')
    for k in range(LD_RESTORE_N + 2):
        cmd = pol.update(0, 0.0, 0.0, 0.0, 0.1, [], 1.0 + k * 0.05)
    check('경로 복귀: 원복', cmd.ld_min == 0.0, f'{cmd.ld_min}')
    check('경로 복귀: ld_restore 발화', 'detour_ld_restore' in rec.kinds())

    # 11. 재탐색 실패 후 재시도는 정지한 뒤에만 한다
    _p, pol, rec = _mk(planner=_StubPlanner(ok=False))
    _l, _s, t1 = _run(pol, lambda t, s: [_Fake(1, 40.0, 0.0)], n=200, v=0.0)
    n1 = len(pol.planner.jobs)
    # 주행 중에는 (RETRY_MIN_S가 지나도) 재시도하지 않는다
    _l, _s, t2 = _run(pol, lambda t, s: [_Fake(1, 40.0 + 5.0 * (t - t1), 0.0)],
                      n=200, v=5.0, t0=t1)
    check('실패 후 주행 중: 재시도 안 함', len(pol.planner.jobs) == n1,
          f'{len(pol.planner.jobs)} vs {n1}')
    _run(pol, lambda t, s: [_Fake(1, 40.0 + 5.0 * (t2 - t1), 0.0)],
         n=100, v=0.0, s0=5.0 * (t2 - t1), t0=t2)
    check('실패 후 정지 시: 재시도', len(pol.planner.jobs) == n1 + 1,
          f'{len(pol.planner.jobs)} vs {n1}')

    # 11b. 같은 객체는 MAX_TRIES_PER_OBJ회에서 멈춘다
    _p, pol, rec = _mk(planner=_StubPlanner(ok=False))
    car = lambda t, s: [_Fake(1, 40.0, 0.0)]
    _l, _s, t1 = _run(pol, car, n=1600, v=0.0)          # 80초: 재시도 여유 충분
    check(f'같은 객체: {MAX_TRIES_PER_OBJ}회에서 멈춘다',
          len(pol.planner.jobs) == MAX_TRIES_PER_OBJ,
          f'{len(pol.planner.jobs)}회 요청')

    # 11c. **경로를 갈아 끼워도 그 횟수는 남는다.** _B에만 두면 우회 성공마다
    #      0으로 돌아가 같은 객체를 무한히 다시 푼다.
    pol.rebind(pol.path, pol.rows, pol.zones, pol.bounds)
    _l, _s, t2 = _run(pol, car, n=800, v=0.0, t0=t1)
    check('경로 교체 후에도 유지', len(pol.planner.jobs) == MAX_TRIES_PER_OBJ,
          f'{len(pol.planner.jobs)}회 요청')

    # 11d. 리셋은 **"그 객체가 후보에서 사라졌을 때"**다 (정상 주행 복귀).
    _run(pol, lambda t, s: [], n=int(TRIES_RESET_S / 0.05) + 4, v=0.0, t0=t2)
    check('사라지면 기록을 버린다', not pol._hist and not pol._failed,
          f'hist={list(pol._hist)} failed={sorted(pol._failed)}')
    # 확인 1.5초 + 여유 1.5초. RETRY_MIN_S(5) 안이라 요청은 정확히 한 번이다.
    _run(pol, car, n=60, v=0.0, t0=t2 + TRIES_RESET_S + 1.0)
    check('되돌아오면 다시 시도', len(pol.planner.jobs) == MAX_TRIES_PER_OBJ + 1,
          f'{len(pol.planner.jobs)}회 요청')

    # 11e. 전체 상한 -- 3회로 묶었더니 네 번째 차량에서 영구 정지했다
    check(f'전체 상한 {MAX_DETOURS}회', MAX_DETOURS >= 20, f'{MAX_DETOURS}')
    _p, pol, _r = _mk(planner=_StubPlanner())
    pol.n_detours = 3               # 옛 상한
    _run(pol, car, n=400)
    check('우회 3회 뒤에도 발동', bool(pol.planner.jobs),
          f'{len(pol.planner.jobs)}회 요청')

    # 11f~11g. 탈출 블렌드 마커의 **부호**. 뒤집으면 실차에서 반대쪽이 켜진다.
    #          진실은 하나뿐이다 -- 결과 기하의 실제 횡이동 방향.
    import turnsignal as TS
    from pure_pursuit import Path

    def blend_case(lat_want):
        """ego가 새 경로에서 lat_want만큼 떨어진 상황을 합성한다."""
        import xodr_graph as G
        wps = [G.Waypoint('p1_to_p2', float(i), 0.0, 0.0, '1', -1, 0,
                          float(i), 3.2) for i in range(60)]
        # ego는 새 경로 첫 점 기준 lat_want만큼 왼쪽(양수)에 있다
        _blend_start(wps, 0.0, lat_want, 12.0)
        d = _mark_escape_blend(wps, lat_want, 12.0)
        moved = TS._lateral_ahead(Path([(w.x, w.y) for w in wps]), 0.0, 12.0)
        return d, moved

    for lat_want, want_dir in ((3.0, 'right'), (-3.0, 'left')):
        d, moved = blend_case(lat_want)
        ok_sign = (d == want_dir
                   and ((moved < -0.3) if want_dir == 'right' else (moved > 0.3)))
        check(f'탈출 블렌드 마커 부호 (lat {lat_want:+.1f}m -> {want_dir})',
              ok_sign, f'마커 {d or "없음"} / 실제 횡이동 {moved:+.2f}m')

    d, _m = blend_case(0.3)
    check('미세 보정에는 마커를 심지 않는다', d == '', f'마커 {d or "없음"}')

    # 13. 맥락 판단 -- 적색 정지선 앞 정지 차량은 대기열이라 동결한다
    _p, pol, rec = _mk(planner=_StubPlanner(), stoplines=[_SL(30.0)])
    _run(pol, lambda t, s: [_Fake(1, 20.0, 0.0)], n=400,
         sig_stopping=True, sig_state=1, sig_dist=30.0)
    check('적색 대기열: 20초 동결', not pol.planner.jobs,
          f'{len(pol.planner.jobs)}회 요청')
    check('적색 대기열: detour_context 대기열 판정',
          any(k == 'detour_context' and '적색 대기열' in d for k, d in rec.log),
          ','.join(rec.kinds()))

    # 13a'. 동결은 FREEZE_MAX_S에서 끝난다 -- 강제 적색 + 정지 차량 교착 방지
    _p, pol, rec = _mk(planner=_StubPlanner(), stoplines=[_SL(30.0)])
    o = lambda t, s: [_Fake(1, 20.0, 0.0)]
    kw = dict(sig_stopping=True, sig_state=1, sig_dist=30.0)
    _l, _s, t1 = _run(pol, o, n=int(FREEZE_MAX_S / 0.05) + 200, **kw)   # 70초
    check('동결 상한: 70초 미발동 (QUEUE 12초 진행 중)', not pol.planner.jobs
          and pol._blocks[1].tier == TIER_QUEUE,
          f'{len(pol.planner.jobs)}회, tier={pol._blocks[1].tier if 1 in pol._blocks else "?"}')
    _run(pol, o, n=100, t0=t1, **kw)                                       # 75초
    check('동결 상한: 60+12초 뒤 발동', len(pol.planner.jobs) == 1,
          f'{len(pol.planner.jobs)}회 요청')

    # 13b. 녹색인데 정지 -- 정지선이 코앞이어도 FAST(1.5s)
    _p, pol, rec = _mk(planner=_StubPlanner(), stoplines=[_SL(30.0)])
    _run(pol, lambda t, s: [_Fake(1, 20.0, 0.0)], n=40,
         sig_stopping=True, sig_green=True, sig_state=3, sig_dist=30.0)
    check('녹색인데 정지: 2초 안에 발동', len(pol.planner.jobs) == 1,
          f'{len(pol.planner.jobs)}회 요청')
    check('녹색인데 정지: 사유 기록',
          any(k == 'detour_context' and '녹색인데 정지' in d for k, d in rec.log))

    # 13c. **적색 대기 중 앞차가 사라지면** 정상 복귀한다 (옆으로 빠졌거나 지나감)
    _p, pol, rec = _mk(planner=_StubPlanner(), stoplines=[_SL(30.0)])
    _l, _s, t1 = _run(pol, lambda t, s: [_Fake(1, 20.0, 0.0)], n=100,
                      sig_stopping=True, sig_state=1, sig_dist=30.0)
    log, _s, t2 = _run(pol, lambda t, s: [], n=100, t0=t1,
                       sig_stopping=True, sig_state=1, sig_dist=30.0)
    check('앞차 소멸: 후보 비움', not pol._blocks and log[-1][2] == 'none',
          f'blocks={list(pol._blocks)} act={log[-1][2]}')
    check('앞차 소멸: 기록 정리', not pol._hist, f'{list(pol._hist)}')
    _run(pol, lambda t, s: [], n=60, t0=t2, sig_green=True, sig_state=3)
    check('앞차 소멸 후 녹색: 발동 없음', not pol.planner.jobs,
          f'{len(pol.planner.jobs)}회 요청')

    # 13d. 정지 차량 3대 연쇄 + 교차로 진입 20m 앞 -> QUEUE(12s)
    _p, pol, rec = _mk(planner=_StubPlanner(), junc_s=(60.0, 80.0))
    cars = [_Fake(1, 40.0, 0.0), _Fake(2, 50.0, 0.0), _Fake(3, 58.0, 0.0)]
    _run(pol, lambda t, s: cars, n=220)           # 11초
    check('교차로 연쇄: 11초 미발동', not pol.planner.jobs,
          f'{len(pol.planner.jobs)}회 요청 (tier={pol._blocks[1].tier if 1 in pol._blocks else "?"})')
    _run(pol, lambda t, s: cars, n=40)            # 13초
    # 스텁 플래너는 늘 실패하므로 나머지 두 대가 곧바로 재시도한다. 첫 요청이
    # 세 대를 한 그룹으로 묶었는지만 본다
    check('교차로 연쇄: 12초 뒤 발동 (3대 한 그룹)', pol.planner.jobs
          and len(pol.planner.jobs[0]['obj_ids']) == 3,
          f'{len(pol.planner.jobs)}회 요청 '
          f'{[j["obj_ids"] for j in pol.planner.jobs[:1]]}')
    check('교차로 연쇄: 대기열 경과 사유',
          any(k == 'detour_context' and '대기열이었으나' in d for k, d in rec.log),
          ','.join(rec.kinds()))

    # 13e. 신호 정지선 앞인데 신호가 NONE(미획득) -> QUEUE
    _p, pol, rec = _mk(planner=_StubPlanner(), stoplines=[_SL(60.0, True)])
    _run(pol, lambda t, s: [_Fake(1, 40.0, 0.0)], n=220, sig_state=0)
    check('신호 미획득: 11초 미발동', not pol.planner.jobs)
    _run(pol, lambda t, s: [_Fake(1, 40.0, 0.0)], n=40, sig_state=0)
    check('신호 미획득: 12초 뒤 발동', len(pol.planner.jobs) == 1)

    # 13f. 비신호 정지선 근처 단독 -> AMBIG(4s)
    _p, pol, rec = _mk(planner=_StubPlanner(), stoplines=[_SL(60.0, False)])
    _run(pol, lambda t, s: [_Fake(1, 40.0, 0.0)], n=60)   # 3초
    check('비신호 정지선 근처: 3초 미발동', not pol.planner.jobs)
    _run(pol, lambda t, s: [_Fake(1, 40.0, 0.0)], n=30)   # 4.5초
    check('비신호 정지선 근처: 4초 뒤 발동', len(pol.planner.jobs) == 1)

    # 13g. 60m 앞 정지 차량을 **달리는 중**에 건다 -- 블렌드 여유가 남는다
    _p, pol, rec = _mk(planner=_StubPlanner())
    log, s_end, _t = _run(pol, lambda t, s: [_Fake(1, 70.0, 0.0)], n=80, v=12.5)
    fired = next((r for r in log if r[2] == 'search'), None)
    check('주행 중 발동: 요청', fired is not None and len(pol.planner.jobs) == 1,
          f'{len(pol.planner.jobs)}회 요청')
    check('주행 중 발동: 객체 30m 이상 앞에서',
          fired is not None and 70.0 - fired[1] >= 30.0,
          f's={fired[1] if fired else None}')
    check('주행 중 발동: 속도가 워커에 전달', pol.planner.jobs
          and abs(pol.planner.jobs[0].get('ego_v', 0.0) - 12.5) < 1e-6)
    check('주행 중 사다리는 35m부터', BLEND_TRIES_MOVING[0] == 35.0
          and BLEND_TRIES_MOVING[-len(BLEND_TRIES):] == BLEND_TRIES)

    # 13h. 정지 보행자는 맥락과 무관하게 최소 AMBIG(4s)
    _p, pol, rec = _mk(planner=_StubPlanner())
    _run(pol, lambda t, s: [_Fake(1, 40.0, 0.0, dims=PED)], n=60)     # 3초
    check('정지 보행자: 3초 미발동', not pol.planner.jobs)
    _run(pol, lambda t, s: [_Fake(1, 40.0, 0.0, dims=PED)], n=30)     # 4.5초
    check('정지 보행자: 4초 뒤 발동', len(pol.planner.jobs) == 1)

    # 13i. 보행자가 3초에 한 프레임이라도 움직이면 확인 시간을 버린다
    _p, pol, rec = _mk(planner=_StubPlanner())
    def twitch(t, s):
        sp = 0.5 if 3.0 <= t < 3.05 else 0.0
        return [_Fake(1, 40.0, 0.0, speed=sp, dims=PED)]
    _run(pol, twitch, n=80)                        # 4.0초
    check('보행자 이동: 4초 미발동', not pol.planner.jobs,
          f'{len(pol.planner.jobs)}회 요청')
    check('보행자 이동: 리셋 이벤트',
          any(k == 'detour_context' and '이동 감지' in d for k, d in rec.log))
    _run(pol, twitch, n=70, t0=4.0)                # 7.5초 (3.05 + 4.0 = 7.05)
    check('보행자 이동: 다시 4초 뒤 발동', len(pol.planner.jobs) == 1,
          f'{len(pol.planner.jobs)}회 요청')

    # 13j. --detour-confirm 값을 주면 세 단계 전부 그 값 (옛 명령 호환)
    _p, pol, rec = _mk(planner=_StubPlanner(), confirm_s=30.0)
    _run(pol, lambda t, s: [_Fake(1, 40.0, 0.0)], n=400)
    check('균일 오버라이드: 20초 미발동', not pol.planner.jobs
          and pol._blocks[1].confirm == 30.0,
          f'{len(pol.planner.jobs)}회, confirm={pol._blocks[1].confirm if 1 in pol._blocks else "?"}')

    # 14. 옆 차로의 정지 객체 (9/11 실차 sc_jam -- 3차로 도로에서 2·3차로가 막혔는데
    #     3->2->(다시 3) 우회를 6번 실패하고 안전망이 두 대를 관통했다)
    # 14a. 옆 차로 객체가 job에 실린다 -- 회랑 그룹은 obj_ids/blocked, 옆 차로는 wide_obs
    _p, pol, rec = _mk(planner=_StubPlanner())
    _run(pol, lambda t, s: [_Fake(1, 40.0, 0.0), _Fake(2, 42.0, 3.3)], n=40)
    job = pol.planner.jobs[0] if pol.planner.jobs else {}
    w = [x for x in job.get('wide_obs', ()) if x['id'] == 2]
    check('옆 차로: 회랑 그룹만 obj_ids', job.get('obj_ids') == [1],
          f'{job.get("obj_ids")}')
    check('옆 차로: blocked는 노드당 개수', job.get('blocked') == {('1', 0, -1): 1},
          f'{job.get("blocked")}')
    check('옆 차로: wide_obs에 실림 (군집, need 2.3)',
          len(w) == 1 and w[0]['hard'] and abs(w[0]['need'] - 2.3) < 1e-9
          and w[0]['ref_node'] == ('1', 0, -1),
          f'{w}')
    check('옆 차로: 회랑 객체만 detour_context',
          all('id=1' in d for k, d in rec.log if k == 'detour_context'),
          f'{[d for k, d in rec.log if k == "detour_context"]}')

    # 14b. 옆 차로 객체만으로는 발동하지 않는다 -- 추적·acc 선적립만
    _p, pol, rec = _mk(planner=_StubPlanner())
    _run(pol, lambda t, s: [_Fake(2, 40.0, 3.3)], n=400)
    check('옆 차로 단독: 미발동', not pol.planner.jobs and 2 in pol._blocks
          and not pol._blocks[2].corridor,
          f'{len(pol.planner.jobs)}회, {2 in pol._blocks}')
    check('옆 차로 단독: acc 선적립', 2 in pol._blocks
          and pol._blocks[2].acc > DETOUR_CONFIRM_FAST_S,
          f'{pol._blocks[2].acc if 2 in pol._blocks else "?"}')
    check('옆 차로 단독: 이벤트 없음', 'detour_context' not in rec.kinds(),
          ','.join(rec.kinds()))

    # 14c. 군집 판정 -- 회랑 그룹에서 CHAIN_GAP_M 이하로 이어지면 hard, 아니면 페널티만
    _p, pol, rec = _mk(planner=_StubPlanner())
    _run(pol, lambda t, s: [_Fake(1, 20.0, 0.0), _Fake(2, 30.0, 3.3),
                            _Fake(3, 55.0, 3.3)], n=40)
    hard = {x['id']: x['hard'] for x in pol.planner.jobs[0]['wide_obs']}
    check('군집: 10m 앞 옆 차로는 hard, 35m 앞은 soft',
          hard == {2: True, 3: False}, f'{hard}')
    _p, pol, rec = _mk(planner=_StubPlanner())
    _run(pol, lambda t, s: [_Fake(1, 20.0, 0.0), _Fake(2, 30.0, 3.3),
                            _Fake(3, 42.0, 3.3), _Fake(4, 54.0, 6.6)], n=40)
    hard = {x['id']: x['hard'] for x in pol.planner.jobs[0]['wide_obs']}
    check('군집: 12m 간격으로 이어지면 전부 hard (반복 확장)',
          hard == {2: True, 3: True, 4: True}, f'{hard}')

    # 14d. 확인 시간이 rebind를 넘는다 -- 1.0초 보고 교체, 0.7초 뒤 발동 (합 1.7 >= 1.5)
    _p, pol, rec = _mk(planner=_StubPlanner())
    _run(pol, lambda t, s: [_Fake(1, 40.0, 0.0)], n=20)
    check('rebind 승계: 교체 전 미발동', not pol.planner.jobs)
    pol.rebind(pol.path, pol.rows, pol.zones, pol.bounds)
    check('rebind 승계: _blocks 유지', 1 in pol._blocks
          and abs(pol._blocks[1].acc - 0.95) < 1e-6,
          f'{pol._blocks[1].acc if 1 in pol._blocks else "없음"}')
    _run(pol, lambda t, s: [_Fake(1, 40.0, 0.0)], n=14, t0=1.0)
    check('rebind 승계: 교체 뒤 0.7초 만에 발동', bool(pol.planner.jobs),
          f'acc={pol._blocks[1].acc if 1 in pol._blocks else "?"}')

    # 14e. 회랑 승격 -- 옆 차로에서 3초 추적한 차가 새 경로(그 차로)에서는 회랑 안이다.
    #      교체 뒤 3프레임 안에 발동해야 한다 (목표: 감지 후 2초 이내)
    _p, pol, rec = _mk(planner=_StubPlanner())
    _run(pol, lambda t, s: [_Fake(1, 40.0, 3.3)], n=60)
    check('회랑 승격: 교체 전 알림 없음', 'detour_context' not in rec.kinds())
    p2 = _Straight(y0=3.3)
    rows2 = [Row(x, y, '1', 0, -2) for x, y in p2.pts]
    pol.rebind(p2, rows2, [], pol.bounds)
    _run(pol, lambda t, s: [_Fake(1, 40.0, 3.3)], n=3, t0=3.0)
    ctx = [d for k, d in rec.log if k == 'detour_context']
    check('회랑 승격: 3프레임 안에 발동', len(pol.planner.jobs) == 1,
          f'{len(pol.planner.jobs)}회')
    check('회랑 승격: detour_context 1건, 선적립 표기',
          len(ctx) == 1 and '선적립' in ctx[0], f'{ctx}')
    check('회랑 승격: 새 경로 노드로 페널티',
          pol.planner.jobs and pol.planner.jobs[0]['blocked'] == {('1', 0, -2): 1},
          f'{pol.planner.jobs[0]["blocked"] if pol.planner.jobs else "?"}')

    # 14f. 옆 차로 객체가 움직이면 조용히 버린다
    _p, pol, rec = _mk(planner=_StubPlanner())
    _run(pol, lambda t, s: [_Fake(2, 40.0, 3.3, speed=(1.0 if t > 1.0 else 0.0))],
         n=30)
    check('옆 차로 이동: 추적에서 제거', 2 not in pol._blocks)
    check('옆 차로 이동: 이벤트 없음', 'detour_context' not in rec.kinds(),
          ','.join(rec.kinds()))

    # 14g. _penalized -- 객체 1대당 가산, 차선변경 이탈은 무료
    import xodr_graph as G
    A, B, A2, B2 = ('1', 0, -1), ('1', 0, -2), ('1', 1, -1), ('1', 1, -2)
    g = G.Graph(pts={}, edges={A: [G.Edge(B, 25.0, 'lanechange'),
                                   G.Edge(A2, 10.0, 'section')],
                               B: [G.Edge(A, 25.0, 'lanechange'),
                                   G.Edge(B2, 10.0, 'section')]})
    g2 = DetourPlanner._penalized(g, {B: 2})
    cost = {(n, e.to): e.cost for n, es in g2.edges.items() for e in es}
    check('페널티: 막힌 노드 진입은 대수 x 5000',
          abs(cost[(A, B)] - (25.0 + 2 * BLOCK_PENALTY)) < 1e-9, f'{cost[(A, B)]}')
    check('페널티: 막힌 노드 직진 이탈도 가산',
          abs(cost[(B, B2)] - (10.0 + 2 * BLOCK_PENALTY)) < 1e-9, f'{cost[(B, B2)]}')
    check('페널티: 차선변경 이탈은 무료', abs(cost[(B, A)] - 25.0) < 1e-9)
    check('페널티: 무관한 엣지 그대로', abs(cost[(A, A2)] - 10.0) < 1e-9)

    # 14h. _lane_node -- 같은 도로·같은 부호 차로만, 반경 안만
    def _lp(x, y):
        return G.LanePt(x, x, y, 0.0, 3.3, 'both', '', 'both', '')
    pts = {('1', 0, -1): [_lp(i, 0.0) for i in range(21)],
           ('1', 0, -2): [_lp(i, -3.3) for i in range(21)],
           ('1', 0, 1): [_lp(i, 3.3) for i in range(21)],
           ('2', 0, -1): [_lp(i, -6.6) for i in range(21)]}
    li = G.LaneIndex(pts)
    check('차로 노드: 같은 도로 옆 차로',
          DetourPlanner._lane_node(li, 10.0, -3.4, ('1', 0, -1)) == ('1', 0, -2))
    check('차로 노드: 대향 차로는 None',
          DetourPlanner._lane_node(li, 10.0, 3.3, ('1', 0, -1)) is None)
    check('차로 노드: 다른 도로는 None',
          DetourPlanner._lane_node(li, 10.0, -6.6, ('1', 0, -1)) is None)
    check('차로 노드: 반경 밖은 None',
          DetourPlanner._lane_node(li, 10.0, -20.0, ('1', 0, -1)) is None)

    # 14i. 시도한 적 없는 옆 차로 객체는 최후 안전망을 못 건다 (게이트 확인)
    for oid, want in ((2, False), (1, True)):
        _p, pol, rec = _mk(planner=_StubPlanner(ok=False))
        obs2 = OB.ObstacleCmd(action='stop', obj_id=oid)
        _run(pol, lambda t, s: [_Fake(1, 8.0, 0.0), _Fake(2, 10.0, 3.3)],
             n=800, obs=obs2)
        check(f'안전망 게이트: obs가 id={oid}를 볼 때 {"발동" if want else "미발동"}',
              ('last_resort_pass' in rec.kinds()) == want,
              ','.join(sorted(set(rec.kinds()))))

    # 12. 실제 경로 -- 계획 단계 실측과 일치해야 한다
    data = _data_dir()
    wp = os.path.join(data, 'route_path_pt2.csv')
    if os.path.exists(wp) and os.path.exists(
            os.path.join(data, 'xodr_graph_edges.csv')):
        import pure_pursuit as PP
        import xodr_graph as G
        with open(wp, newline='', encoding='utf-8') as fh:
            segs = []
            for r in csv.DictReader(fh):
                if r['segment_id'] not in segs:
                    segs.append(r['segment_id'])
        path, _b = PP.load_path(wp, segs)
        rows = load_rows(wp, segs)
        check('실경로: rows가 path와 1:1', check_rows(path, rows) == '',
              check_rows(path, rows))
        pts, _d = G.load_lane_points(os.path.join(data, 'xodr_lanes.csv'))
        g = G.load_graph(os.path.join(data, 'xodr_graph_edges.csv'), pts)
        zs = scan_undetourable(path, rows, g)
        tot = sum(z.length for z in zs)
        check('실경로 pt2: 우회 불가 4구간 457.3m',
              len(zs) == 4 and abs(tot - 457.3) < 1.0,
              f'{len(zs)}구간 {tot:.1f}m')

    print()
    if fails:
        print(f'[det] 자기검사 실패 {len(fails)}건')
        for f in fails:
            print('  - ' + f)
        return 1
    print('[det] 자기검사 전부 통과')
    return 0


def main() -> None:
    import argparse
    data = _data_dir()
    ap = argparse.ArgumentParser(description='영구 정지 객체 우회 정책')
    ap.add_argument('--selftest', action='store_true', help='합성 시나리오 검사')
    ap.add_argument('--plan', action='store_true',
                    help='경로의 우회 가능성 진단 (대회 당일 4번 단계 전에 돌린다)')
    ap.add_argument('--path', help='waypoint CSV')
    ap.add_argument('--segments', help='쉼표 구분. 없으면 파일 순서 전부')
    ap.add_argument('--lanes', default=os.path.join(data, 'xodr_lanes.csv'))
    ap.add_argument('--graph-edges',
                    default=os.path.join(data, 'xodr_graph_edges.csv'))
    args = ap.parse_args()
    if args.selftest:
        raise SystemExit(_selftest())
    if args.plan:
        if not args.path:
            raise SystemExit('--plan 에는 --path 가 필요하다')
        raise SystemExit(_plan(args))
    ap.error('--selftest 또는 --plan --path 중 하나가 필요하다')


if __name__ == '__main__':
    main()
