#!/usr/bin/env python3
"""Pure Pursuit 경로 추종 제어기.

녹화해 둔 조밀한 waypoint(data/waypoints_v2.csv)를 추종한다.
9910으로 20Hz 상태를 받고 같은 주기로 조향·목표가속도를 돌려준다.

    python pure_pursuit.py --host 192.168.50.11 --segment p1_to_p2 --speed 10
    python pure_pursuit.py --segments p1_to_p2,p2_to_p3          # 연속 주행

동작 요약
    · 여러 구간을 주면 순서대로 이어붙여 하나의 경로로 다룬다. 누적거리와
      인덱스는 이어붙인 뒤 한 번에 계산하므로 구간 경계에서 끊기지 않는다
    · 시작 시 현재 위치에서 가장 가까운 경로 점을 전역 탐색해 거기서부터 추종
    · lookahead = 속도 비례, 4~10m로 제한 (교차로 회전반경 실측 14m보다 작게)
    · 목표속도는 --speed(km/h). 곡률이 큰 구간에서는 횡가속도 한계로 자동 감속
    · 경로 끝이 가까워지면 감속해 정지
    · --events로 정지선 목록을 주면 적색·황색 신호에서 정지선 앞에 선다
      (stoplines.py, 평가항목 7·9)
    · objects[]를 경로에 투영해 보행자·차량·장애물을 피한다
      (obstacles.py, 평가항목 10·11·14). --no-obstacle로 끈다
    · 차선변경·교차로 회전 3초 전에 방향지시등을 켠다
      (turnsignal.py, 평가항목 13). --no-turn-signal로 끈다
    · 횡단보도 위에 서게 될 것 같으면 그 앞에 서고, 이미 올라가 있으면
      안전할 때만 빠져나간다 (crosswalks.py, 평가항목 12). --no-crosswalk로 끈다
    · 리스폰(프레임 간 5m 이상 점프) 감지 시 경로 인덱스를 다시 전역 탐색
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import os
import tempfile
import time

from hlvtd_client import (
    Client, SpeedEstimator, detect_respawn,
    TURN_OFF, STEER_LIMIT, ACCEL_MAX, ACCEL_MIN,
)
from session_log import SessionLog, default_dir
import crosswalks
import detour
import obstacles
import stoplines
import turnsignal

WHEELBASE = 2.95            # m, 실차 검증에서 적절하다고 확인된 값
LD_MIN, LD_MAX = 4.0, 10.0  # m, lookahead 범위
LD_GAIN = 0.9               # lookahead = LD_GAIN * v[m/s]

LAT_ACCEL_MAX = 2.5         # m/s^2, 곡선 구간 감속 기준 횡가속도
SPEED_KP = 1.2              # 속도 오차 -> 목표가속도
DECEL_END = 1.5             # m/s^2, 경로 끝 감속률
STOP_SPEED = 0.3            # m/s, 이 아래면 정지로 본다

OFFSET_SLOW_M = 3.0         # m, 이만큼 경로에서 벗어나면 복귀 속도로 제한
RECOVER_SPEED = 3.0         # m/s (약 11km/h), 경로 복귀 중 상한

# 경로 끝 처리. 남은 거리가 lookahead보다 짧아지면 목표점이 코앞이라
# 목표 방위각이 발산하고 조향이 ±STEER_LIMIT로 포화된다.
END_HEADING_KP = 0.8        # 끝 접선과의 방위 오차 -> 조향
END_STEER_LIMIT = 0.12      # rad, 끝에서 허용할 조향 상한 (약 7°)

SEARCH_WINDOW = 60          # 최근접점 국소 탐색 범위(점 개수). 0.3m 간격 = 약 18m
JOIN_WARN_M = 3.0           # 구간 이음매가 이보다 벌어지면 경고


class Path:
    """waypoint 목록과 누적 거리. 인덱스 추적을 담당한다."""

    def __init__(self, points: list[tuple[float, float]]):
        if len(points) < 2:
            raise ValueError('경로 점이 2개 미만이다')
        self.pts = points
        self.s = [0.0] * len(points)          # 누적 거리
        for i in range(1, len(points)):
            dx = points[i][0] - points[i - 1][0]
            dy = points[i][1] - points[i - 1][1]
            self.s[i] = self.s[i - 1] + math.hypot(dx, dy)
        self.length = self.s[-1]

    def nearest(self, x: float, y: float, start: int | None = None) -> int:
        """최근접 점 인덱스. start를 주면 그 앞쪽 창 안에서만 찾는다.

        전역 탐색은 시작 시와 리스폰 후에만 쓴다. 경로가 스스로 근접하는
        구간에서 인덱스가 뒤로 튀는 것을 막기 위해 평소에는 전진 창만 본다.
        """
        if start is None:
            rng = range(len(self.pts))
        else:
            rng = range(start, min(start + SEARCH_WINDOW, len(self.pts)))
        return min(rng, key=lambda i: math.hypot(self.pts[i][0] - x,
                                                 self.pts[i][1] - y))

    def lookahead_point(self, i0: int, x: float, y: float,
                        ld: float) -> tuple[float, float, int, float]:
        """i0에서 앞으로 걸어가며 ego로부터 ld 이상 떨어진 첫 점을 찾는다.

        마지막 두 점 사이는 선형 보간한다. 끝까지 가면 마지막 점을 준다.
        실제 목표점까지의 거리도 함께 준다. 경로에서 크게 벗어났거나(리스폰)
        경로 끝에 닿으면 이 거리가 ld와 달라지므로 조향 계산에는 이 값을 쓴다.

        걷기는 i0가 아니라 경로를 따라 ld만큼 앞선 점에서 시작한다. 경로 위에
        있을 때는 결과가 같지만(호길이 < ld인 점은 직선거리도 항상 ld 미만),
        경로에서 크게 벗어났을 때 최근접점을 그대로 겨냥해 수직으로 파고드는
        것을 막아 접선 방향으로 복귀하게 한다.
        """
        start = self._advance(i0, ld)
        prev_d = math.hypot(self.pts[start][0] - x, self.pts[start][1] - y)
        for i in range(start + 1, len(self.pts)):
            px, py = self.pts[i]
            d = math.hypot(px - x, py - y)
            if d >= ld:
                qx, qy = self.pts[i - 1]
                t = (ld - prev_d) / (d - prev_d) if d > prev_d else 1.0
                t = max(0.0, min(1.0, t))
                tx, ty = qx + t * (px - qx), qy + t * (py - qy)
                return tx, ty, i, math.hypot(tx - x, ty - y)
            prev_d = d
        lx, ly = self.pts[-1]
        return lx, ly, len(self.pts) - 1, math.hypot(lx - x, ly - y)

    def curvature_at(self, i: int, span_m: float = 6.0) -> float:
        """i 앞쪽 구간의 곡률 [1/m]. 세 점을 지나는 원의 곡률로 근사한다."""
        a = self._advance(i, span_m)
        b = self._advance(a, span_m)
        if b >= len(self.pts) - 1:
            return 0.0
        (x1, y1), (x2, y2), (x3, y3) = self.pts[i], self.pts[a], self.pts[b]
        # 삼각형 넓이 x 2
        cross = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)
        d12 = math.hypot(x2 - x1, y2 - y1)
        d23 = math.hypot(x3 - x2, y3 - y2)
        d13 = math.hypot(x3 - x1, y3 - y1)
        denom = d12 * d23 * d13
        if denom < 1e-6:
            return 0.0
        return abs(2.0 * cross / denom)

    def _advance(self, i: int, dist_m: float) -> int:
        """i에서 경로를 따라 dist_m 앞선 인덱스."""
        target = self.s[i] + dist_m
        j = i
        while j < len(self.pts) - 1 and self.s[j] < target:
            j += 1
        return j

    def remaining(self, i: int) -> float:
        return self.length - self.s[i]

    def end_heading(self, back_m: float = 3.0) -> float:
        """마지막 구간의 접선 방위. 끝 두 점만 쓰면 녹화 노이즈에 흔들린다."""
        ex, ey = self.pts[-1]
        i = len(self.pts) - 1
        while i > 0 and self.length - self.s[i] < back_m:
            i -= 1
        return math.atan2(ey - self.pts[i][1], ex - self.pts[i][0])


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def load_path(csv_path: str, segments: list[str]) -> tuple[Path, list[tuple[str, int]]]:
    """구간들을 준 순서대로 이어붙여 하나의 경로로 만든다.

    누적거리도 인덱스도 이어붙인 다음 한 번에 계산하므로 구간 경계에서
    끊기지 않는다. 두 번째 반환값은 (구간 이름, 시작 인덱스) 목록.
    """
    by_seg: dict[str, list[tuple[float, float]]] = {}
    with open(csv_path, newline='', encoding='utf-8') as fh:
        for row in csv.DictReader(fh):
            by_seg.setdefault(row['segment_id'], []).append(
                (float(row['x']), float(row['y'])))

    missing = [n for n in segments if n not in by_seg]
    if missing:
        raise SystemExit(f'세그먼트 {", ".join(missing)} 없음. '
                         f'사용 가능: {", ".join(sorted(by_seg))}')

    pts: list[tuple[float, float]] = []
    bounds: list[tuple[str, int]] = []
    for name in segments:
        bounds.append((name, len(pts)))
        pts.extend(by_seg[name])
    return Path(pts), bounds


def _stop_target(s_ego: float, sig, obs) -> float:
    """sig/obs가 겨냥한 후륜축 정지 지점 중 가까운 쪽. 둘 다 없으면 nan.

    stop_avail은 "지금부터 몇 m 더 가서 설 작정인가"라서 s_ego를 더하면 경로
    누적거리가 된다. 횡단보도 정차 금지(항목 12)가 "그 자리가 횡단보도 위인가"를
    묻는 데만 쓴다.
    """
    best = float('nan')
    for avail in (sig.stop_avail, obs.stop_avail):
        if avail != avail:                      # nan = 정지 의사 없음
            continue
        s = s_ego + max(0.0, avail)
        if best != best or s < best:
            best = s
    return best


def steering_for(ego_x: float, ego_y: float, heading: float,
                 tx: float, ty: float, ld: float, wheelbase: float) -> tuple[float, float]:
    """Pure Pursuit 조향각과 목표점 방위각(alpha)을 계산한다.

    heading은 반시계 방향 양수. 차량 좌표계에서 y가 양수면 목표점이 왼쪽에
    있고, 프로토콜도 양수 조향이 좌회전이므로 부호 변환이 필요 없다.
    """
    dx, dy = tx - ego_x, ty - ego_y
    ch, sh = math.cos(heading), math.sin(heading)
    x_local = ch * dx + sh * dy
    y_local = -sh * dx + ch * dy
    alpha = math.atan2(y_local, x_local)
    delta = math.atan2(2.0 * wheelbase * math.sin(alpha), max(ld, 0.1))
    return max(-STEER_LIMIT, min(STEER_LIMIT, delta)), alpha


def run(args) -> None:
    target_speed = args.speed / 3.6                 # km/h -> m/s

    # 이벤트 CSV는 신호등(정지선)과 횡단보도가 **함께 쓴다.** 그래서 경로
    # 결정을 --no-signal 바깥에 둔다 — 안에 두면 --no-signal이 횡단보도까지
    # 조용히 꺼 버린다.
    ev_path = args.events or stoplines.default_events_path(args.waypoints)
    ev_ctx = {'t': 0.0, 'x': float('nan'), 'y': float('nan'), 'v': float('nan')}

    # 세션 로그를 먼저 연다. 원본 프레임을 파싱 전에 받아야 하므로
    # raw_sink를 Client에 물려서 연결한다.
    log = SessionLog(args.session, mode='pp', meta={
        'host': args.host, 'port': args.port,
        'waypoints': args.waypoints, 'segments': args.segments,
        'speed_kmh': args.speed, 'wheelbase': args.wheelbase,
        'route': args.route or '',
        'events': ev_path,
        'signal': (not args.no_signal) and bool(ev_path),
        'front_overhang': args.front_overhang,
        'tl_max_hold_s': args.tl_max_hold,
        'obstacles': not args.no_obstacle,
        'obs_lookahead_m': args.obs_lookahead,
        'obs_max_block_s': args.obs_max_block,
        'turn_signal': not args.no_turn_signal,
        'turn_junction': not args.no_junction_signal,
        'turn_lead_s': args.turn_lead,
        'crosswalk': (not args.no_crosswalk) and bool(ev_path),
        'cw_escape_after_s': args.cw_escape_after,
        'detour': not args.no_detour,
        'detour_confirm_s': args.detour_confirm,
        'last_resort_s': (0.0 if args.no_last_resort else args.last_resort_s),
    })
    log.copy_input(args.waypoints, 'waypoints.csv')
    log.copy_input(args.route, 'route.csv')   # 현장에서 받은 원본은 반드시 보존
    if ev_path:
        log.copy_input(ev_path, 'events_in.csv')

    # 정책이 내는 이벤트는 전부 같은 통로로 나간다. 콘솔에 찍고, 그때의
    # 위치·시각을 ev_ctx에서 꺼내 events.csv에 남긴다.
    def _policy_event(kind: str, detail: str) -> None:
        print()
        print(f'[pp] [{kind}] {detail}')
        log.event(kind, detail, t=ev_ctx['t'],
                  ego_x=ev_ctx['x'], ego_y=ev_ctx['y'], speed=ev_ctx['v'])

    def build_policies(wp_csv: str, ev_csv: str, segments: list,
                       verbose: bool = False) -> dict:
        """경로 하나에 딸린 정책 일습을 만든다.

        **시작 시와 우회 시가 같은 코드를 쓴다.** detour의 워커 스레드가 새
        경로를 만든 뒤 이것을 그대로 불러 정책까지 통째로 갈아 끼운다.
        stoplines/crosswalks/turnsignal이 전부 (x,y)로 스냅하므로 다른 경로에
        그대로 다시 붙는다 — 우회를 가능하게 하는 가장 큰 요인이다.

        CSV 읽기만 하고 소켓·로그는 건드리지 않아 워커 스레드에서 안전하다.
        """
        say = print if verbose else (lambda *a, **k: None)
        path, bounds = load_path(wp_csv, segments)
        eh = path.end_heading()
        say(f'[pp] 경로 {len(path.pts)}점, 길이 {path.length:.1f}m, '
            f'끝 접선 {math.degrees(eh):+.1f}°')
        for k, (name, start) in enumerate(bounds):
            end = bounds[k + 1][1] if k + 1 < len(bounds) else len(path.pts)
            say(f'[pp]   {k + 1}. {name}  {end - start}점  '
                f'{path.s[end - 1] - path.s[start]:.1f}m')
            if k:
                gap = path.s[start] - path.s[start - 1]     # 앞 구간 끝 -> 이 구간 시작
                warn = (f'  *** 경고: {JOIN_WARN_M:.0f}m 초과. 구간 순서가 맞는지, '
                        f'녹화가 끊기지 않았는지 확인할 것 ***'
                        if gap > JOIN_WARN_M else '')
                say(f'[pp]      이음매 {bounds[k - 1][0]} → {name}: {gap:.2f}m{warn}')

        out = {'path': path, 'bounds': bounds, 'end_heading': eh,
               'sig': None, 'obs': None, 'turn': None, 'xw': None,
               'n_tz': 0, 'n_xw': 0, 'n_sl': 0,
               'wp_csv': wp_csv, 'ev_csv': ev_csv, 'segments': list(segments)}

        # 신호등 정지 (평가항목 7·9). --events가 없고 waypoints 옆에 _events.csv도
        # 없으면 비활성이고, 그때는 예전과 완전히 같은 주행이 된다.
        sig_path = '' if args.no_signal else ev_csv
        if sig_path:
            sl_list, sl_warn = stoplines.load_for_path(path, sig_path)
            say(f'[pp] 정지선 {len(sl_list)}개 ({os.path.basename(sig_path)})')
            for line in stoplines.summary(sl_list):
                say('[pp]' + line)
            for w in sl_warn:
                say(f'[pp]   *** {w} ***')
            n_sig = sum(1 for x in sl_list if x.signalized)
            say(f'[pp] 그 중 신호 정지선 {n_sig}개. '
                f'범퍼 보정 {args.front_overhang:.3f}m')
            out['n_sl'] = len(sl_list)
            out['sig'] = stoplines.SignalPolicy(
                path, sl_list, front_overhang=args.front_overhang,
                max_hold=args.tl_max_hold, on_event=_policy_event)
        else:
            say('[pp] 신호등 정지 비활성 (--events 없음)')

        # 충돌 회피 (평가항목 10·11·14). --no-obstacle이면 예전과 같은 주행이 된다.
        if not args.no_obstacle:
            out['obs'] = obstacles.ObstaclePolicy(
                path, front_overhang=args.front_overhang,
                lookahead=args.obs_lookahead, max_block=args.obs_max_block,
                on_event=_policy_event)
            say('[pp] 충돌 회피 활성 (항목 10·11·14)')
            for line in obstacles.summary(out['obs']):
                say('[pp]' + line)
        else:
            say('[pp] 충돌 회피 비활성 (--no-obstacle)')

        # 방향지시등 (평가항목 13). 차선변경은 waypoint CSV의 lc_dir에서,
        # 교차로 회전은 경로 헤딩에서 나온다.
        if not args.no_turn_signal:
            tz_list, tz_warn = turnsignal.build_zones(
                path, wp_csv, junction=not args.no_junction_signal)
            n_lc = sum(1 for z in tz_list if z.kind == 'lc')
            say(f'[pp] 방향지시등 구간 {len(tz_list)}개 '
                f'(차선변경 {n_lc} / 회전 {len(tz_list) - n_lc}), '
                f'선행 {args.turn_lead:.1f}초')
            for line in turnsignal.summary(tz_list):
                say('[pp]' + line)
            for w in tz_warn:
                say(f'[pp]   *** {w} ***')
            out['n_tz'] = len(tz_list)
            out['turn'] = turnsignal.TurnSignalPolicy(
                path, tz_list, lead_s=args.turn_lead,
                v_ref=target_speed, on_event=_policy_event)
        else:
            say('[pp] 방향지시등 비활성 (--no-turn-signal)')

        # 횡단보도 정차 금지 (평가항목 12). 정지선과 같은 이벤트 CSV를 쓴다.
        if not args.no_crosswalk and ev_csv:
            xw_list, xw_warn = crosswalks.load_for_path(path, ev_csv)
            say(f'[pp] 횡단보도 {len(xw_list)}개')
            for line in crosswalks.summary(xw_list):
                say('[pp]' + line)
            for w in xw_warn:
                say(f'[pp]   *** {w} ***')
            out['n_xw'] = len(xw_list)
            out['xw'] = crosswalks.CrosswalkPolicy(
                path, xw_list, front_overhang=args.front_overhang,
                escape_after=args.cw_escape_after, on_event=_policy_event)
        elif args.no_crosswalk:
            say('[pp] 횡단보도 정차 금지 비활성 (--no-crosswalk)')
        else:
            say('[pp] 횡단보도 정차 금지 비활성 (이벤트 CSV 없음)')
        return out

    bundle = build_policies(args.waypoints, ev_path, args.segments, verbose=True)
    path, bounds = bundle['path'], bundle['bounds']
    end_heading = bundle['end_heading']
    policy, obs_policy = bundle['sig'], bundle['obs']
    turn_policy, xw_policy = bundle['turn'], bundle['xw']
    seg_starts = [start for _n, start in bounds]

    def seg_at(i: int) -> str:
        return bounds[bisect.bisect_right(seg_starts, i) - 1][0]

    log.note('path_points', len(path.pts))
    log.note('path_length_m', round(path.length, 2))
    log.note('stoplines', bundle['n_sl'])
    log.note('turn_zones', bundle['n_tz'])
    log.note('crosswalks', bundle['n_xw'])

    # 영구 정지 객체 우회 (완주 보장). 재탐색은 **워커 스레드**에서 돌아
    # 20Hz 루프를 막지 않는다. 그래프 로드가 실패해도 주행은 그대로 간다.
    det_policy = None
    detours_meta = []
    if args.no_detour:
        print('[pp] 우회 비활성 (--no-detour)')
    else:
        data_dir = os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data'))
        rows = detour.load_rows(args.waypoints, args.segments)
        bad = detour.check_rows(path, rows)
        if bad:
            print(f'[pp] 우회 비활성: {bad}')
            log.note('detour', False)
            log.note('detour_off_reason', bad)
        else:
            try:
                out_dir = log.dir or tempfile.mkdtemp(prefix='hlfma_detour_')
                planner = detour.DetourPlanner(
                    data_dir, out_dir, build_policies, on_event=_policy_event,
                    lanes_csv=args.lanes, edges_csv=args.graph_edges,
                    objects_csv=args.xodr_objects)
                zones = detour.scan_undetourable(
                    path, rows, planner.graph, planner.has_lc, planner.injunc)
                det_policy = detour.DetourPolicy(
                    path, rows, zones, bounds, planner=planner,
                    confirm_s=args.detour_confirm,
                    last_resort_s=args.last_resort_s,
                    use_last_resort=not args.no_last_resort,
                    on_event=_policy_event)
                print('[pp] 우회 활성 (완주 보장)')
                for line in detour.summary(det_policy):
                    print('[pp]' + line)
                und = sum(z.length for z in zones)
                log.note('undetourable_m', round(und, 1))
                log.note('undetourable_pct',
                         round(100.0 * und / max(path.length, 1e-6), 1))
                log.note('undetourable_zones', detour.zones_meta(zones))
            except Exception as exc:      # 우회가 안 되더라도 주행은 계속한다
                print(f'[pp] 우회 비활성: 그래프 준비 실패 {exc!r}')
                log.note('detour', False)
                log.note('detour_off_reason', repr(exc))
                det_policy = None

    client = Client(args.host, args.port, raw_sink=log.raw_sink)
    print(f'[pp] {args.host}:{args.port} 연결 시도...')
    client.connect()
    print('[pp] 연결됨. Ctrl+C로 종료\n')
    log.event('connect', f'{args.host}:{args.port}')

    speed_est = SpeedEstimator()
    prev_frame = None
    idx: int | None = None                          # None = 재탐색 필요
    last_cmd = (0.0, 0.0)                           # 예외 시 유지할 마지막 명령
    finished = False
    n = 0
    t0 = time.monotonic()

    try:
        while True:
            frame = client.recv_frame()
            if frame is None:
                print('\n[pp] 연결 끊김. 재연결...')
                log.event('disconnect', '수신 실패')
                client.connect()
                log.event('connect', '재연결')
                prev_frame, idx = None, None
                speed_est = SpeedEstimator()
                continue

            n += 1
            t = time.monotonic() - t0
            sig = stoplines.SignalCmd()        # 예외가 나도 로깅이 되게
            obs = obstacles.ObstacleCmd()
            xw = crosswalks.CrosswalkCmd()
            turn = turnsignal.TurnCmd()
            det = detour.DetourCmd()
            try:
                v = speed_est.update(frame.ego_x, frame.ego_y)
                ev_ctx.update(t=t, x=frame.ego_x, y=frame.ego_y, v=v)

                if detect_respawn(frame, prev_frame):
                    print('\n[pp] *** 리스폰 감지 — 경로 인덱스 재탐색 ***')
                    log.respawn(frame, prev_frame, v, t=t)
                    idx = None
                    finished = False
                    if policy is not None:
                        # 위치가 강제로 옮겨졌다. "지나갔다"는 판정을 되돌린다
                        policy.reset()
                    if obs_policy is not None:
                        obs_policy.reset()      # 추적도 전부 버린다
                    if turn_policy is not None:
                        turn_policy.reset()     # 점등 래치를 버린다
                    if xw_policy is not None:
                        xw_policy.reset()       # 정차·탈출 타이머를 버린다
                    if det_policy is not None:
                        det_policy.reset()      # 옛 위치 기준 관측을 버린다
                prev_frame = frame

                # 우회 재탐색 결과. **프레임 맨 위에서 갈아 끼운다** — 한
                # 프레임이 통째로 같은 경로 위에서 돌아야 한다. 교체는 참조
                # 대입뿐이고, 인덱스 재동기화는 리스폰 처리를 그대로 쓴다.
                if det_policy is not None:
                    plan = det_policy.poll()
                    if plan is not None:
                        nb = plan.bundle
                        path, bounds = nb['path'], nb['bounds']
                        end_heading = nb['end_heading']
                        seg_starts = [st for _nm, st in bounds]
                        policy, obs_policy = nb['sig'], nb['obs']
                        # 램프는 교체로 꺼지지 않는다. 방향이 같으면 점등
                        # 시각을 이어받아 로그에 없는 깜빡임을 만들지 않는다.
                        if nb['turn'] is not None:
                            nb['turn'].adopt(turn_policy, t)
                        turn_policy, xw_policy = nb['turn'], nb['xw']
                        det_policy.rebind(path, plan.rows, plan.zones, bounds)
                        idx = None          # 리스폰과 같은 전역 재탐색
                        # **meta의 waypoints/segments는 원본 그대로 둔다.**
                        # check_* 계수기가 그것으로 경로를 되살리는데, 우회
                        # 경로(수십 m)로 바꿔 놓으면 그 구간 밖의 신호·충돌을
                        # 아예 못 세게 된다. 우회로는 detours[]로만 남기고,
                        # 우회 구간의 투영 오차(차로폭 한 칸)는 감수한다.
                        detours_meta.append({
                            't': round(t, 2), 'csv': plan.csv_path,
                            'len_m': round(path.length, 1),
                            'n_lc': plan.n_lc, 'solid_lc': plan.solid_lc,
                            'first_lc_m': round(plan.first_lc_m, 1),
                            'tight': plan.tight, 'blocked': plan.blocked})
                        log.note('detours', detours_meta)
                        # 시작 시 값이 남아 있으면 "정책이 재구성되지 않았다"는
                        # 오해의 근거가 된다. 실제로 한 번 그랬다.
                        log.note('path_points', len(path.pts))
                        log.note('path_length_m', round(path.length, 2))
                        log.note('stoplines', nb['n_sl'])
                        log.note('turn_zones', nb['n_tz'])
                        log.note('crosswalks', nb['n_xw'])

                if idx is None:                     # 시작 / 리스폰 후 전역 탐색
                    idx = path.nearest(frame.ego_x, frame.ego_y)
                    off = math.hypot(path.pts[idx][0] - frame.ego_x,
                                     path.pts[idx][1] - frame.ego_y)
                    print(f'[pp] 경로 진입점 idx={idx} 이탈거리 {off:.2f}m'
                          + ('  ← 경로에서 멀다. 세그먼트를 확인할 것' if off > 10 else ''))
                else:
                    idx = path.nearest(frame.ego_x, frame.ego_y, start=idx)

                ld = max(LD_MIN, min(LD_MAX, LD_GAIN * v))
                # 촉박한 우회 교체 직후에는 lookahead를 올려 둔다. v=0에서
                # 4m면 3.2m 횡오프셋에 조향이 한계 0.35 rad로 포화된다.
                if det_policy is not None and det_policy.ld_floor > ld:
                    ld = min(LD_MAX, det_policy.ld_floor)
                tx, ty, _tgt_idx, ld_real = path.lookahead_point(
                    idx, frame.ego_x, frame.ego_y, ld)
                # 조향에는 실제 목표점 거리를 쓴다. 경로에서 멀리 떨어졌을 때
                # 명목 ld를 쓰면 조향이 락까지 포화되어 복귀가 거칠어진다.
                steer, _alpha = steering_for(
                    frame.ego_x, frame.ego_y, frame.ego_heading, tx, ty,
                    max(LD_MIN, ld_real), args.wheelbase)

                # 목표속도: 곡률 제한, 경로 이탈 시 복귀 속도, 경로 끝 감속 중 최소
                v_cmd = target_speed
                kappa = path.curvature_at(idx)
                if kappa > 1e-3:
                    v_cmd = min(v_cmd, math.sqrt(LAT_ACCEL_MAX / kappa))
                offset = math.hypot(path.pts[idx][0] - frame.ego_x,
                                    path.pts[idx][1] - frame.ego_y)
                if offset > OFFSET_SLOW_M:          # 리스폰 직후 등 — 천천히 복귀
                    v_cmd = min(v_cmd, RECOVER_SPEED)
                rem = path.remaining(idx)
                v_cmd = min(v_cmd, math.sqrt(2.0 * DECEL_END * max(0.0, rem - 1.0)))

                # 충돌 회피를 신호보다 **먼저** 돌린다. 항목 8 면책과
                # MAX_HOLD_S 동결에 obs.blocking이 필요하기 때문이다.
                # 의존은 이 한 방향뿐이다.
                if obs_policy is not None:
                    obs = obs_policy.update(idx, path.s[idx], v,
                                            frame.ego_x, frame.ego_y,
                                            frame.ego_heading, frame.objects, t)
                    v_cmd = min(v_cmd, obs.v_limit)

                # 신호등. 정지 프로파일이 경로 끝 감속과 같은 sqrt 형태라
                # min() 하나로 합쳐진다. 정지선은 경로 중간이므로 아래
                # 끝단 조향 인계(rem < ld)와는 무관하다.
                if policy is not None:
                    sig = policy.update(path.s[idx], v, frame.tl_id,
                                        frame.tl_state, t, blocked=obs.blocking)
                    v_cmd = min(v_cmd, sig.v_limit)

                # 횡단보도(항목 12)는 **신호·객체 뒤에** 돈다. 그 둘이 겨냥한
                # 정지 지점이 횡단보도 위인지를 봐야 하기 때문이다.
                if xw_policy is not None:
                    xw = xw_policy.update(
                        path.s[idx], v, t,
                        stop_s=_stop_target(path.s[idx], sig, obs),
                        tl_stop_ahead=(sig.stopping and sig.dist > 0.0),
                        front_clear=(None if obs_policy is None
                                     else obs.front_clear))
                    v_cmd = min(v_cmd, xw.v_limit)
                    # **min() 구조를 깨는 유일한 지점이다.** 항목 12(-3)를
                    # 피하려고 v_cmd를 끌어올린다. 항목 7(-6)·14(-6)이 걸린
                    # 상황에서는 crosswalks가 escape=False를 내므로 여기 오지
                    # 않는다. 아래 finished 처리가 이것보다 뒤라 완주가 이긴다.
                    if xw.escape:
                        v_cmd = max(v_cmd, xw.escape_v)

                # 우회(완주 보장)는 **신호·객체·횡단보도 뒤에** 돈다. 신호
                # 정지 중인지를 알아야 적색 대기열을 영구 정지로 오판하지 않고,
                # obstacles가 어느 객체를 보고 있는지가 최후 안전망의 게이트다.
                if det_policy is not None:
                    det = det_policy.update(
                        idx, path.s[idx], v, frame.ego_x, frame.ego_y,
                        frame.objects, t, sig_stopping=sig.stopping,
                        # 녹색(3/5)인데 못 가고 있으면 신호 대기가 아니라 객체
                        # 교착이다. 4(좌회전 전용)는 GO_STATES에 없고, 직진
                        # 차로에서의 4는 stoplines가 이미 _stop으로 보낸다.
                        sig_green=(sig.state in stoplines.GO_STATES),
                        obs=(None if obs_policy is None else obs),
                        finished=finished)
                    # 최후 안전망. xw.escape와 같은 성격이라 같은 방식으로
                    # 합류한다. 아래 finished 처리가 이것보다 뒤라 완주가 이긴다.
                    if det.v_floor > v_cmd:
                        v_cmd = det.v_floor

                # 경로 끝: pure pursuit 항을 접어두고 마지막 구간 접선을
                # 유지하는 조향으로 넘긴다. 끝점에 붙을수록 접선 항의 비중이
                # 커지고, 그 항은 END_STEER_LIMIT로 묶여 있어 포화되지 않는다.
                if rem < ld:
                    hold = END_HEADING_KP * wrap_pi(end_heading - frame.ego_heading)
                    hold = max(-END_STEER_LIMIT, min(END_STEER_LIMIT, hold))
                    w = max(0.0, min(1.0, rem / ld))
                    steer = w * steer + (1.0 - w) * hold

                if rem < 1.5 and v < STOP_SPEED:
                    finished = True

                if finished:
                    steer, v_cmd = 0.0, 0.0
                accel = SPEED_KP * (v_cmd - v)
                if v_cmd <= 0.0:                    # 정지 명령은 확실히 제동
                    accel = min(accel, -1.5)
                # 신호 정지는 속도 상한만으로는 프로파일을 못 따라간다.
                # P 제어의 정상상태 오차만큼 위로 달려 정지선을 넘는다.
                accel = min(accel, sig.accel_limit, xw.accel_limit)
                # obs.accel_limit은 막은 객체를 향한 감속 피드포워드라
                # 최후 안전망의 v_floor를 그대로 무력화한다. 그때만 뺀다.
                if not det.forcing:
                    accel = min(accel, obs.accel_limit)
                accel = max(ACCEL_MIN, min(ACCEL_MAX, accel))

                # 방향지시등(항목 13)은 속도에 영향이 없어 위 합류에 끼지
                # 않는다. CTRL의 3번째 바이트로만 나간다.
                if turn_policy is not None and not finished:
                    turn = turn_policy.update(path.s[idx], v, t)
                last_cmd = (steer, accel)
            except Exception as exc:                # 계산 실패해도 루프는 유지
                print(f'\n[pp] 계산 오류: {exc!r} — 마지막 명령 유지')
                log.event('error', repr(exc), t=t,
                          ego_x=frame.ego_x, ego_y=frame.ego_y)
                steer, accel = last_cmd
                tx = ty = float('nan')
                v, v_cmd, ld = speed_est.speed, 0.0, LD_MIN
                idx = idx or 0
                sig = stoplines.SignalCmd(action='error')
                obs = obstacles.ObstacleCmd(action='error')
                xw = crosswalks.CrosswalkCmd(action='error')
                turn = turnsignal.TurnCmd(action='error')
                det = detour.DetourCmd(action='error')

            if not client.send_control(steer, accel, turn.signal):
                print('\n[pp] 송신 실패. 재연결...')
                log.event('disconnect', '송신 실패', t=t,
                          ego_x=frame.ego_x, ego_y=frame.ego_y, speed=v)
                client.connect()
                log.event('connect', '재연결')
                prev_frame, idx = None, None
                continue

            seg = seg_at(idx)
            log.add_frame(frame, v, t=t, seg=seg, idx=idx,
                          tgt_x=tx, tgt_y=ty, ld=ld, steer=steer, accel=accel,
                          tl_act=sig.action, tl_dist=sig.dist,
                          ob_act=obs.action, ob_kind=obs.kind,
                          ob_id=obs.obj_id, ob_dist=obs.dist, ob_lat=obs.lat,
                          turn=turn.signal, turn_src=turn.src,
                          cw_act=xw.action, cw_dist=xw.dist,
                          dt_act=det.action, dt_dist=det.dist)

            print(f'[{n:5d}] ego({frame.ego_x:8.2f},{frame.ego_y:9.2f}) '
                  f'h={math.degrees(frame.ego_heading):+7.1f}° '
                  f'v={v * 3.6:5.1f}→{v_cmd * 3.6:5.1f}km/h | '
                  + (f'{seg} ' if len(bounds) > 1 else '')
                  + f'idx={idx:3d}/{len(path.pts) - 1} 남은{path.remaining(idx):6.1f}m '
                  f'tgt({tx:8.2f},{ty:9.2f}) Ld={ld:4.1f} | '
                  f'steer={steer:+.3f}rad({math.degrees(steer):+5.1f}°) '
                  f'a={accel:+5.2f}'
                  + ('' if sig.action in ('none', 'error')
                     else f' | TL:{sig.action:5s}{sig.dist:5.1f}m')
                  + ('' if obs.action in ('none', 'error')
                     else f' | OB:{obs.action:6s}{obs.kind[:3]}'
                          f'{obs.dist:5.1f}m{obs.lat:+5.1f}')
                  + ('' if xw.action in ('none', 'error')
                     else f' | XW:{xw.action:6s}{xw.dist:6.1f}m')
                  + ('' if det.action in ('none', 'error')
                     else f' | DT:{det.action:11s}{det.dist:6.1f}m')
                  + ('' if turn.signal == TURN_OFF
                     else f' | SIG:{turnsignal.SIGNAL_NAME[turn.signal]}')
                  + ('  [완료]' if finished else ''))

            if finished:
                print('\n[pp] 경로 끝 도달 — 정지 완료')
                log.event('finish', f'{path.length:.1f}m 주행 {n}프레임', t=t,
                          ego_x=frame.ego_x, ego_y=frame.ego_y, speed=v)
                break
    except KeyboardInterrupt:
        print('\n[pp] 사용자 종료')
        log.event('note', '사용자 종료(Ctrl+C)')
    finally:
        client.close()                              # close()가 감속 명령을 보낸다
        log.close()
        if args.session:
            print(f'[pp] 세션 로그: {args.session}')


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    default_csv = os.path.join(here, '..', 'data', 'waypoints_v2.csv')

    ap = argparse.ArgumentParser(description='Pure Pursuit 경로 추종')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=9910)
    # --segment 와 --segments 는 같은 인자. 기본값 없이 반드시 지정해야 한다.
    ap.add_argument('--segment', '--segments', dest='segments', required=True,
                    metavar='이름[,이름...]',
                    help='추종할 구간. 쉼표로 여러 개를 주면 순서대로 이어붙인다')
    ap.add_argument('--waypoints', default=os.path.normpath(default_csv))
    ap.add_argument('--speed', type=float, default=10.0, help='목표속도 [km/h]')
    ap.add_argument('--wheelbase', type=float, default=WHEELBASE, help='축거 [m]')
    ap.add_argument('--route', help='현장에서 받은 route CSV. 세션에 원본을 복사해 둔다')
    # --log 는 옛 이름. 이제 파일이 아니라 세션 디렉터리를 가리킨다.
    # --log '' 로 끄던 습관은 그대로 통한다.
    ap.add_argument('--events', default='',
                    help='정지선 이벤트 CSV (route_path_*_events.csv). '
                         '생략하면 --waypoints 옆의 _events.csv를 쓴다')
    ap.add_argument('--no-signal', action='store_true',
                    help='신호등 정지를 끈다 (회귀 비교용)')
    ap.add_argument('--front-overhang', type=float,
                    default=stoplines.FRONT_OVERHANG,
                    help='후륜축에서 앞범퍼까지 [m]. 9910 ego 좌표는 후륜축이다')
    ap.add_argument('--tl-max-hold', type=float,
                    default=stoplines.MAX_HOLD_S,
                    help='신호 대기 상한 [s]. 넘으면 교착으로 보고 출발한다')
    ap.add_argument('--no-obstacle', action='store_true',
                    help='충돌 회피를 끈다 (회귀 비교용)')
    ap.add_argument('--obs-lookahead', type=float,
                    default=obstacles.LOOKAHEAD_M,
                    help='충돌 회피가 볼 전방 거리 [m]')
    ap.add_argument('--obs-max-block', type=float,
                    default=obstacles.MAX_BLOCK_S,
                    help='보행자 대기 상한 [s]. 넘으면 교착으로 보고 기어간다')
    ap.add_argument('--no-turn-signal', action='store_true',
                    help='방향지시등을 끈다 (회귀 비교용)')
    ap.add_argument('--no-junction-signal', action='store_true',
                    help='교차로 회전 점등만 끈다. 차선변경 점등은 남는다')
    ap.add_argument('--turn-lead', type=float, default=turnsignal.LEAD_S,
                    help='방향지시등 선행 점등 시간 [s]. 요건은 3초')
    ap.add_argument('--no-crosswalk', action='store_true',
                    help='횡단보도 정차 금지를 끈다 (회귀 비교용)')
    ap.add_argument('--no-detour', action='store_true',
                    help='영구 정지 객체 우회와 최후 안전망을 끈다 (회귀 비교용)')
    ap.add_argument('--detour-confirm', type=float,
                    default=detour.BLOCK_CONFIRM_S,
                    help='막힘 확인 시간 [s]')
    ap.add_argument('--last-resort-s', type=float,
                    default=detour.LAST_RESORT_S,
                    help='최후 안전망 발동까지의 정지 지속 시간 [s]')
    ap.add_argument('--no-last-resort', action='store_true',
                    help='우회는 쓰되 최후 안전망만 끈다')
    ap.add_argument('--lanes', default='', help='xodr_lanes.csv (우회 재탐색용)')
    ap.add_argument('--graph-edges', default='',
                    help='xodr_graph_edges.csv (우회 재탐색용)')
    ap.add_argument('--xodr-objects', default='',
                    help='xodr_objects.csv (우회 경로의 이벤트 재생성용)')
    ap.add_argument('--cw-escape-after', type=float,
                    default=crosswalks.ESCAPE_AFTER_S,
                    help='횡단보도 위 정차 이 시간 뒤 탈출을 검토한다 [s]. '
                         '항목 12 한계는 3초')
    ap.add_argument('--session', '--log', dest='session',
                    default=default_dir('pp'),
                    help='세션 로그 디렉터리 (빈 문자열이면 기록 안 함)')

    args = ap.parse_args()
    args.segments = [s.strip() for s in args.segments.split(',') if s.strip()]
    if not args.segments:
        ap.error('--segment 값이 비어 있다')
    run(args)


if __name__ == '__main__':
    main()
