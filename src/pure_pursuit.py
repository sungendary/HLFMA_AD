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
    · 상대 운동으로 4초 안의 최근접점을 예측해 끼어들기·교차 진입에 미리
      감속한다 (ttc.py, 평가항목 14 강화). --no-ttc로 끈다
    · 차선변경·교차로 회전 3초 전에 방향지시등을 켠다
      (turnsignal.py, 평가항목 13). --no-turn-signal로 끈다
    · 횡단보도 위에 서게 될 것 같으면 그 앞에 서고, 이미 올라가 있으면
      안전할 때만 빠져나간다 (crosswalks.py, 평가항목 12). --no-crosswalk로 끈다
    · 전방 카메라(8554 RTSP)가 붉은 노면을 보면 25km/h로 제한한다
      (redzone.py, 평가항목 2). --no-redzone으로 끈다
    · 리스폰(프레임 간 5m 이상 점프) 감지 시 경로 인덱스를 다시 전역 탐색
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import os
import sys
import tempfile
import time

from hlvtd_client import (
    Client, SpeedEstimator, detect_respawn,
    TURN_OFF, STEER_LIMIT, ACCEL_MAX, ACCEL_MIN,
)
from session_log import SessionLog, default_dir
import crosswalks
import detour
import finish
import junction_yield
import obstacles
import redzone
import stoplines
import ttc
import turnsignal

# stdout이 파일·파이프로 리다이렉트되면 Windows에서는 cp949가 된다. cp949에 없는
# 문자가 이벤트 문자열에 섞이면 print가 UnicodeEncodeError를 내고, 그게 예외
# 핸들러 안의 print면 제어 루프가 죽는다. 로그가 깨지는 것보다 차가 서는 것이
# 훨씬 나쁘므로 못 적는 문자는 '?'로 바꿔 출력한다. (콘솔 직접 실행은 UTF-8이라
# 영향이 없다.)
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(errors='replace')
    except (AttributeError, ValueError):
        pass

WHEELBASE = 2.95            # m, 실차 검증에서 적절하다고 확인된 값
LD_MIN, LD_MAX = 4.0, 10.0  # m, lookahead 범위
LD_GAIN = 0.9               # lookahead = LD_GAIN * v[m/s]

LAT_ACCEL_MAX = 0.5         # m/s^2, 곡선 구간 감속 기준 횡가속도
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

    def __init__(self, points: list[tuple[float, float]],
                 n_real: int | None = None):
        if len(points) < 2:
            raise ValueError('경로 점이 2개 미만이다')
        self.pts = points
        self.s = [0.0] * len(points)          # 누적 거리
        for i in range(1, len(points)):
            dx = points[i][0] - points[i - 1][0]
            dy = points[i][1] - points[i - 1][1]
            self.s[i] = self.s[i - 1] + math.hypot(dx, dy)
        self.length = self.s[-1]
        # 끝의 가상 연장(finish.extend_points)을 뺀 실제 경로. 종료선은
        # finish_s에 있고, 그 뒤 length까지는 지나쳐 서기 위한 꼬리다.
        n = len(points) if n_real is None else max(2, min(n_real, len(points)))
        self.n_real = n
        self.finish_idx = n - 1
        self.finish_s = self.s[self.finish_idx]

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

    def replace_tail(self, tail: list[tuple[float, float]]) -> None:
        """실제 점(n_real) 뒤의 가상 꼬리를 제자리에서 갈아 끼운다.

        finish.py가 종료선(정지선·라바콘 쌍)에 맞춰 꼬리를 다시 낼 때 쓴다. 모든
        정책이 같은 Path 객체를 참조하므로 따로 알릴 필요가 없다. n_real /
        finish_idx / finish_s는 불변이고 앞쪽 s도 그대로다. 호출자는 꼬리를
        **짧게 만들지 않는다** (꼬리 위 인덱스를 들고 있는 정책이 있다).
        """
        base = self.pts[:self.n_real]
        self.pts = base + list(tail)
        s = self.s[:self.n_real]
        for i in range(self.n_real, len(self.pts)):
            dx = self.pts[i][0] - self.pts[i - 1][0]
            dy = self.pts[i][1] - self.pts[i - 1][1]
            s.append(s[-1] + math.hypot(dx, dy))
        self.s = s
        self.length = self.s[-1]

    def end_heading(self, back_m: float = 3.0) -> float:
        """마지막 구간의 접선 방위. 끝 두 점만 쓰면 녹화 노이즈에 흔들린다."""
        ex, ey = self.pts[-1]
        i = len(self.pts) - 1
        while i > 0 and self.length - self.s[i] < back_m:
            i -= 1
        return math.atan2(ey - self.pts[i][1], ex - self.pts[i][0])


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def load_path(csv_path: str, segments: list[str],
              extend_m: float | None = None) -> tuple[Path, list[tuple[str, int]]]:
    """구간들을 준 순서대로 이어붙여 하나의 경로로 만든다.

    누적거리도 인덱스도 이어붙인 다음 한 번에 계산하므로 구간 경계에서
    끊기지 않는다. 두 번째 반환값은 (구간 이름, 시작 인덱스) 목록.

    **끝을 접선 방향으로 extend_m 가상 연장한다** (기본 finish.FINISH_OVERRUN_M
    + 1m). 종료선(=마지막 정지선)이 경로 끝 그 자리라 그대로 서면 후륜축이
    못 넘긴다. 연장하면 기존 끝 감속이 저절로 종료선 너머에 세우고, 종료선이
    경로 내부 점이 되어 stoplines·obstacles·계수기가 전부 그대로 맞는다.
    sim 서버·check_*도 이 함수를 쓰므로 같은 기하를 본다. Path.finish_s가
    실제 끝이고 Path.length는 연장을 포함한다.
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
    n_real = len(pts)
    if extend_m is None:
        extend_m = finish.FINISH_OVERRUN_M + 1.0
    pts = pts + finish.extend_points(pts, extend_m)
    return Path(pts, n_real=n_real), bounds


def _stop_target(s_ego: float, sig, obs, tt, jy=None) -> float:
    """sig/obs/ttc가 겨냥한 후륜축 정지 지점 중 가까운 쪽. 아무도 없으면 nan.

    stop_avail은 "지금부터 몇 m 더 가서 설 작정인가"라서 s_ego를 더하면 경로
    누적거리가 된다. 횡단보도 정차 금지(항목 12)가 "그 자리가 횡단보도 위인가"를
    묻는 데만 쓴다. ttc를 넣은 것은 9/11 실차 -- 차로 내 라바콘 앞 TTC 정지가
    횡단보도 위였는데 crosswalks는 그 정지 의사를 몰랐다.
    """
    best = float('nan')
    for avail in (sig.stop_avail, obs.stop_avail, tt.stop_avail,
                  float('nan') if jy is None else jy.stop_avail):
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

    # 붉은 보호구역 소스. 카메라는 경로와 무관하므로 build_policies 밖에서
    # **한 번만** 만든다 -- 우회 교체마다 ffmpeg를 죽였다 살릴 이유가 없다.
    rz_on = not args.no_redzone
    rz_url = ''
    if rz_on:
        rz_url = ('stub:' + ';'.join(args.redzone_stub) if args.redzone_stub
                  else args.redzone_url
                  or redzone.rtsp_url(args.redzone_host or args.host))

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
        'ttc': not args.no_ttc,
        'ttc_gap_m': args.ttc_gap,
        'ttc_ctrv': not args.no_ttc_ctrv,
        'junction_yield': not args.no_junction_yield,
        'yield_max_s': args.yield_max_s,
        'turn_signal': not args.no_turn_signal,
        'turn_lead_s': args.turn_lead,
        'crosswalk': (not args.no_crosswalk) and bool(ev_path),
        'cw_escape_after_s': args.cw_escape_after,
        'detour': not args.no_detour,
        'detour_confirm_s': (args.detour_confirm if args.detour_confirm is not None
                             else f'{detour.DETOUR_CONFIRM_FAST_S}/'
                                  f'{detour.DETOUR_CONFIRM_S}/'
                                  f'{detour.DETOUR_CONFIRM_QUEUE_S}'),
        'last_resort_s': (0.0 if args.no_last_resort else args.last_resort_s),
        'redzone': rz_on,
        'redzone_url': rz_url,
        'zone_speed_kmh': args.zone_speed,
        'zone_hold_m': args.zone_hold,
        'redzone_crop': args.redzone_crop,
        'redzone_enter': args.redzone_enter,
        'redzone_exit': args.redzone_exit,
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

    # 교차로 진입 양보 (junction_yield). 교차로 소속·내부 차로 점열은 경로와
    # 무관하므로 한 번만 읽고 build_policies(우회 워커 스레드 포함)가 같이 쓴다.
    jy_data = {'jroads': {}, 'area': {}}
    if args.no_junction_yield:
        print('[pp] 교차로 진입 양보 비활성 (--no-junction-yield)')
    else:
        try:
            data_dir_jy = os.path.normpath(os.path.join(
                os.path.dirname(os.path.abspath(__file__)), '..', 'data'))
            jr = junction_yield.load_junction_roads(
                os.path.join(data_dir_jy, junction_yield.JROADS_CSV))
            if not jr:
                print(f'[pp]   *** {junction_yield.JROADS_CSV} 가 없다 ― 엣지 CSV 폴백. '
                      'python junction_yield.py --build 로 만들 것 ***')
                jr = junction_yield.fallback_junction_roads(
                    args.graph_edges
                    or os.path.join(data_dir_jy, 'xodr_graph_edges.csv'))
            jy_data['jroads'] = jr
            jy_data['area'] = junction_yield.load_area_points(
                args.lanes or os.path.join(data_dir_jy, 'xodr_lanes.csv'), jr)
        except Exception as exc:        # 못 읽어도 주행은 계속한다
            print(f'[pp] 교차로 진입 양보 비활성: {exc!r}')
            log.note('junction_yield', False)
            log.note('junction_yield_off_reason', repr(exc))
            jy_data = {'jroads': {}, 'area': {}}

    def build_policies(wp_csv: str, ev_csv: str, segments: list,
                       verbose: bool = False) -> dict:
        """경로 하나에 딸린 정책 일습을 만든다.

        **시작 시와 우회 시가 같은 코드를 쓴다.** detour의 워커 스레드가 새
        경로를 만든 뒤 이것을 그대로 불러 정책까지 통째로 갈아 끼운다.
        stoplines/crosswalks/turnsignal이 전부 (x,y)로 스냅하므로 다른 경로에
        그대로 다시 붙는다 ― 우회를 가능하게 하는 가장 큰 요인이다.

        CSV 읽기만 하고 소켓·로그는 건드리지 않아 워커 스레드에서 안전하다.
        """
        say = print if verbose else (lambda *a, **k: None)
        # 끝을 overrun+1m 연장한다. 프로파일이 rem=1에서 0이 되므로 +1이다
        path, bounds = load_path(wp_csv, segments,
                                 extend_m=args.finish_overrun + 1.0)
        # 종료선 (완주 판정)을 **먼저** 정한다. 종료선이 경로 끝 너머(끝 너머
        # 정지선)면 꼬리를 거기까지 다시 내고, 그 뒤에 정지선·횡단보도를 스냅해야
        # 끝 너머 정지선이 정상 정지선으로 붙는다. --no-signal과 무관하게 항상.
        fl, fl_tail, fl_warn = finish.load_for_path(
            path, ev_csv, zone_m=args.finish_zone, search_m=args.finish_search,
            overrun=args.finish_overrun,
            fallback_overrun=args.finish_fallback_overrun)
        path.replace_tail(fl_tail)
        fl.bind(path)
        eh = path.end_heading()
        say(f'[pp] 경로 {path.n_real}점, 길이 {path.finish_s:.1f}m '
            f'(+ 끝 꼬리 {path.length - path.finish_s:.1f}m), '
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
               'fin': None, 'finish': None, 'jy': None,
               'n_tz': 0, 'n_xw': 0, 'n_sl': 0,
               'wp_csv': wp_csv, 'ev_csv': ev_csv, 'segments': list(segments)}

        # 종료선 정책. 속도 상한은 내지 않고 정지 목표(stop_s)·통과·라바콘 쌍만 본다.
        out['finish'] = fl
        out['fin'] = finish.FinishPolicy(path, fl, overrun=args.finish_overrun,
                                         zone_m=args.finish_zone,
                                         on_event=_policy_event)
        for line in finish.summary(out['fin'], args.finish_overrun):
            say('[pp]' + line)
        for w in fl_warn:
            say(f'[pp]   *** {w} ***')

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
                finish_zone=args.finish_zone, on_event=_policy_event)
            say('[pp] 충돌 회피 활성 (항목 10·11·14)')
            for line in obstacles.summary(out['obs']):
                say('[pp]' + line)
        else:
            say('[pp] 충돌 회피 비활성 (--no-obstacle)')

        # 방향지시등 (평가항목 13). 차선변경(waypoint CSV의 lc_dir)에만 켠다.
        # 교차로 회전 점등은 채점 대상이 아니고 실차에서 차선변경 점등을
        # 방해해 9/11에 뺐다.
        if not args.no_turn_signal:
            tz_list, tz_warn = turnsignal.build_zones(path, wp_csv)
            say(f'[pp] 방향지시등 구간 {len(tz_list)}개 (차선변경만), '
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

        # 교차로 진입 양보 (평가항목 14). 비신호 교차로와 비보호 좌회전에서
        # 교차 차량이 있으면 입구 밖에 선다. 신호 유무는 정지선으로 가르므로
        # --no-signal과 무관하게 이벤트 CSV를 읽는다.
        if jy_data['jroads']:
            sls = (out['sig'].stoplines if out['sig'] is not None
                   else (stoplines.load_for_path(path, ev_csv)[0] if ev_csv else []))
            jz, jy_warn = junction_yield.load_for_path(
                path, wp_csv, segments, jy_data['jroads'], jy_data['area'], sls)
            for w in jy_warn:
                say(f'[pp]   *** {w} ***')
            out['jy'] = junction_yield.JunctionYieldPolicy(
                path, jz, front_overhang=args.front_overhang,
                max_hold=args.yield_max_s, on_event=_policy_event)
            say(f'[pp] 교차로 진입 양보 활성 ― 교차로 {len(jz)}곳')
            for line in junction_yield.summary(out['jy']):
                say('[pp]' + line)
        return out

    bundle = build_policies(args.waypoints, ev_path, args.segments, verbose=True)
    path, bounds = bundle['path'], bundle['bounds']
    end_heading = bundle['end_heading']
    policy, obs_policy = bundle['sig'], bundle['obs']
    turn_policy, xw_policy = bundle['turn'], bundle['xw']
    fin_policy = bundle['fin']
    jy_policy = bundle['jy']
    if jy_policy is not None:
        log.note('junction_zones', junction_yield.zones_meta(jy_policy))
    seg_starts = [start for _n, start in bounds]

    def seg_at(i: int) -> str:
        return bounds[bisect.bisect_right(seg_starts, i) - 1][0]

    def note_finish(p, fp) -> None:
        """종료선·꼬리 메타. 시작·우회 교체·라바콘 재조준 때마다 다시 쓴다.
        finish_tail은 check_* 계수기가 같은 기하로 투영하는 데 쓴다."""
        log.note('path_points', p.n_real)
        log.note('path_length_m', round(p.finish_s, 2))
        log.note('path_ext_m', round(p.length - p.finish_s, 2))
        log.note('finish_line_s', round(fp.line.s, 2))
        log.note('finish_line_src', fp.line.src)
        log.note('finish_line_ctrl', fp.line.ctrl_str())
        log.note('finish_line_xy', [round(fp.line.x, 3), round(fp.line.y, 3)])
        log.note('finish_stop_s', round(fp.stop_s, 2))
        log.note('finish_tail', [[round(x, 3), round(y, 3)]
                                 for x, y in p.pts[p.n_real:]])

    note_finish(path, fin_policy)
    log.note('finish_overrun_m', args.finish_overrun)
    log.note('finish_zone_m', args.finish_zone)
    log.note('finish_search_m', args.finish_search)
    log.note('finish_fallback_overrun_m', args.finish_fallback_overrun)
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
                    finish_zone=args.finish_zone,
                    on_event=_policy_event,
                    stoplines=(policy.stoplines if policy is not None else ()),
                    injunc=planner.injunc)
                print('[pp] 우회 활성 (완주 보장)')
                for line in detour.summary(det_policy):
                    print('[pp]' + line)
                # 보행자 횡단 완료의 "도로 밖" 판정은 planner의 차로 인덱스를
                # 쓴다. planner가 정책 뒤에 만들어져 여기서 물린다. 우회 교체
                # 시에는 obs.adopt()가 넘긴다
                if obs_policy is not None:
                    obs_policy.on_road = planner.on_road
                    print('[pp] 보행자 횡단 완료 판정에 도로 밖 조건 사용 '
                          f'(driving 차로 {obstacles.PED_OFFROAD_M:.0f}m 밖)')
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

    # TTC 충돌 예측 (평가항목 14 강화). 트랙·래치가 월드 좌표라 경로와 무관하다.
    # build_policies 밖에서 **한 번** 만들고 우회 교체 때는 rebind(path)만 한다.
    ttc_policy = None
    if args.no_ttc:
        print('[pp] TTC 충돌 예측 비활성 (--no-ttc)')
    else:
        if args.ttc_dcrit is not None:
            print('[pp]   *** --ttc-dcrit은 더 이상 쓰지 않는다 (중심 거리 -> 풋프린트 '
                  f'간격). --ttc-gap {args.ttc_gap:.2f}m를 쓴다 ***')
        ttc_policy = ttc.TtcPolicy(path, gap_crit=args.ttc_gap,
                                   use_ctrv=not args.no_ttc_ctrv,
                                   front_overhang=args.front_overhang,
                                   finish_zone=args.finish_zone,
                                   on_event=_policy_event)
        print('[pp] TTC 충돌 예측 활성 (항목 14 강화)')
        for line in ttc.summary(ttc_policy):
            print('[pp]' + line)

    # 붉은 보호구역 (평가항목 2). ffmpeg 파이프는 데몬 스레드가 읽고 루프는
    # 최신 판정값만 본다. 카메라가 안 붙어도 주행은 그대로 간다 (폴백).
    rz_policy = None
    if not rz_on:
        print('[pp] 붉은 보호구역 판정 비활성 (--no-redzone)')
    else:
        try:
            if args.redzone_stub:
                rz_src = redzone.StubCamera.from_path(
                    path, [redzone.parse_rect(r) for r in args.redzone_stub])
            else:
                rz_src = redzone.CameraReader(
                    rz_url, redzone.parse_crop(args.redzone_crop))
            rz_policy = redzone.RedzonePolicy(
                rz_src, zone_speed=args.zone_speed / 3.6,
                enter_ratio=args.redzone_enter, exit_ratio=args.redzone_exit,
                hold_m=args.zone_hold, on_event=_policy_event)
            print('[pp] 붉은 보호구역 판정 활성 (항목 2)')
            for line in redzone.summary(rz_policy):
                print('[pp]' + line)
        except Exception as exc:        # 카메라가 안 붙어도 주행은 계속한다
            print(f'[pp] 붉은 보호구역 판정 비활성: {exc!r}')
            log.note('redzone', False)
            log.note('redzone_off_reason', repr(exc))
            rz_policy = None

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
    sim_end = False                                 # ego(0,0) 종료 신호를 받았다
    origin_seen = False                             # 종료 아닌 ego(0,0) 경고 1회
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
            rz = redzone.RedzoneCmd()
            fin = finish.FinishCmd()
            tt = ttc.TtcCmd()
            jy = junction_yield.YieldCmd()

            # **ego(0,0)은 시뮬레이션 종료 신호다.** 완주 후 VTD가 ego(0,0),
            # heading 0, 객체 0을 보낸다. 리스폰 감지(5m 점프)로 흘리면 전역
            # 재탐색 뒤 accel=3.0을 낸다. 종료선을 지났으면(passed) 완주로 닫고,
            # 아니면 원인을 모르니 **정지 유지**만 한다 — near(30m 안)로 닫으면
            # 순간 오류 한 프레임에 종료선 앞에서 서 버릴 수 있다. prev_frame/idx를
            # 건드리지 않아 뒤에 실제 프레임이 오면 기존 리스폰 처리가 복구한다.
            # 실제 VTD에서만 온다 (sim은 --end-frames로 흉내낸다).
            if (prev_frame is not None and frame.ego_x == 0.0 and frame.ego_y == 0.0
                    and frame.ego_heading == 0.0 and not frame.objects):
                v = speed_est.speed
                at_end = finished or fin_policy.passed
                i0 = idx if idx is not None else 0
                if at_end and not sim_end:
                    sim_end = True
                    log.event('sim_end',
                              f'종료선 {fin_policy.max_lon:+.1f}m 지난 뒤 ego(0,0) 수신 '
                              f'― 시뮬레이션 종료로 본다', t=t,
                              ego_x=prev_frame.ego_x, ego_y=prev_frame.ego_y, speed=v)
                elif not at_end and not origin_seen:
                    origin_seen = True
                    log.event('ego_origin',
                              f'종료선을 지나지 않았는데 ego(0,0) 수신 '
                              f'(경로 s={path.s[i0]:.1f}) ― 정지 유지', t=t,
                              ego_x=prev_frame.ego_x, ego_y=prev_frame.ego_y, speed=v)
                steer, accel = 0.0, -1.5
                last_cmd = (steer, accel)
                if not client.send_control(steer, accel, TURN_OFF):
                    print('\n[pp] 송신 실패. 재연결...')
                    log.event('disconnect', '송신 실패', t=t)
                    client.connect()
                    log.event('connect', '재연결')
                    prev_frame, idx = None, None
                    continue
                log.add_frame(frame, v, t=t, seg=seg_at(i0), idx=i0,
                              steer=steer, accel=accel, fin_lon=fin_policy.max_lon)
                print(f'[{n:5d}] ego(0,0) 수신 ― '
                      + ('시뮬레이션 종료' if at_end else '종료선 미통과, 정지 유지'))
                if at_end:
                    print('\n[pp] 완주 ― VTD 종료 신호')
                    log.event('finish',
                              f'{path.finish_s:.1f}m 주행 {n}프레임, 종료선 '
                              f'{fin_policy.max_lon:+.1f}m 지나 VTD 종료', t=t,
                              ego_x=prev_frame.ego_x, ego_y=prev_frame.ego_y, speed=v)
                    break
                continue

            try:
                v = speed_est.update(frame.ego_x, frame.ego_y)
                ev_ctx.update(t=t, x=frame.ego_x, y=frame.ego_y, v=v)

                if detect_respawn(frame, prev_frame):
                    print('\n[pp] *** 리스폰 감지 ― 경로 인덱스 재탐색 ***')
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
                    if ttc_policy is not None:
                        ttc_policy.reset()      # 트랙·래치를 버린다
                    if rz_policy is not None:
                        rz_policy.reset()       # 히스테리시스 카운터를 버린다
                    fin_policy.reset()          # 종료선 통과 래치를 버린다
                    if jy_policy is not None:
                        jy_policy.reset()       # 양보 래치·트랙을 버린다
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
                        # 객체 트랙은 월드 좌표라 새 경로에서도 유효하다. 버리면
                        # trusted를 다시 채우는 동안 속도 예측이 꺼진다 (9/11)
                        if nb['obs'] is not None:
                            nb['obs'].adopt(obs_policy)
                        policy, obs_policy = nb['sig'], nb['obs']
                        # 램프는 교체로 꺼지지 않는다. 방향이 같으면 점등
                        # 시각을 이어받아 로그에 없는 깜빡임을 만들지 않는다.
                        if nb['turn'] is not None:
                            nb['turn'].adopt(turn_policy, t)
                        turn_policy, xw_policy = nb['turn'], nb['xw']
                        # 라바콘 종료선을 래치했으면 새 경로에도 이식한다 (우회는
                        # 끝 30m 앞까지 가능하고 라바콘 탐색은 130m 전부터 열린다)
                        nb['fin'].adopt(fin_policy)
                        fin_policy = nb['fin']
                        # 양보 중에 경로가 바뀌어도 양보는 이어진다
                        if nb['jy'] is not None:
                            nb['jy'].adopt(jy_policy)
                        jy_policy = nb['jy']
                        if jy_policy is not None:
                            log.note('junction_zones',
                                     junction_yield.zones_meta(jy_policy))
                        end_heading = path.end_heading()
                        det_policy.rebind(path, plan.rows, plan.zones, bounds,
                                          stoplines=(policy.stoplines
                                                     if policy is not None else ()))
                        if ttc_policy is not None:
                            ttc_policy.rebind(path)     # 트랙·래치는 월드 좌표라 남긴다
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
                        note_finish(path, fin_policy)
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
                # 종료선. 정지 목표 stop_s(종료선 + overrun + 1)까지 남은 거리가
                # rem이고, 아래 끝 감속·끝 조향 블렌드가 그것을 쓴다. 라바콘 쌍이
                # 보이면 여기서 꼬리를 갈아 끼우고 종료선을 옮긴다 (retargeted).
                # obstacles보다 앞이라 finish_zone_ignore와 무관하게 원본 objects를 본다.
                fin = fin_policy.update(idx, path.s[idx], frame.ego_x, frame.ego_y,
                                        frame.ego_heading, v, frame.objects, t)
                if fin.retargeted:
                    idx = min(idx, len(path.pts) - 1)
                    end_heading = path.end_heading()
                    note_finish(path, fin_policy)
                rem = fin.rem
                v_cmd = min(v_cmd, math.sqrt(2.0 * DECEL_END * max(0.0, rem - 1.0)))

                # 붉은 보호구역 (항목 2). 카메라 판정이라 다른 정책과 독립이고
                # 속도 상한 하나뿐이다. 뒤의 xw.escape_v(1.5m/s)·det.v_floor
                # (5km/h)는 25km/h보다 낮아 max() 합류와 충돌하지 않는다.
                if rz_policy is not None:
                    rz = rz_policy.update(v, t, frame.ego_x, frame.ego_y,
                                          frame.ego_heading)
                    v_cmd = min(v_cmd, rz.v_limit)

                # 충돌 회피를 신호보다 **먼저** 돌린다. 항목 8 면책과
                # MAX_HOLD_S 동결에 obs.blocking이 필요하기 때문이다.
                # 의존은 이 한 방향뿐이다.
                if obs_policy is not None:
                    obs = obs_policy.update(idx, path.s[idx], v,
                                            frame.ego_x, frame.ego_y,
                                            frame.ego_heading, frame.objects, t)
                    v_cmd = min(v_cmd, obs.v_limit)

                # 교차로 진입 양보. 입력이 objects와 받은 신호 상태뿐이라 stoplines에
                # 의존하지 않는다. yielding은 아래 ttc·stoplines·crosswalks·detour의
                # 교착 타이머를 얼린다 ― 교차 차량을 기다리는 것은 막힘이 아니다.
                if jy_policy is not None:
                    jy = jy_policy.update(idx, path.s[idx], v,
                                          frame.ego_x, frame.ego_y,
                                          frame.ego_heading, frame.objects,
                                          frame.tl_state, t)
                    v_cmd = min(v_cmd, jy.v_limit)

                # TTC 충돌 예측 (항목 14 강화). obstacles가 "지금 경로 위에 있는
                # 것"을 보는 데 반해 이것은 4초 안의 상대 운동을 본다. 같은 sqrt
                # 프로파일이라 min()으로 합쳐진다. 신호·우회는 이 값을 쓰지 않는다.
                # 우회 교체 프레임에는 위에서 rebind(path) + 전역 nearest를 거쳤으므로
                # idx/s_ego가 새 경로 기준이다.
                if ttc_policy is not None:
                    tt = ttc_policy.update(idx, path.s[idx], v,
                                           frame.ego_x, frame.ego_y,
                                           frame.ego_heading, frame.objects, t,
                                           yielding=jy.yielding)
                    v_cmd = min(v_cmd, tt.v_limit)

                # 신호등. 정지 프로파일이 경로 끝 감속과 같은 sqrt 형태라
                # min() 하나로 합쳐진다. 정지선은 경로 중간이므로 아래
                # 끝단 조향 인계(rem < ld)와는 무관하다.
                if policy is not None:
                    sig = policy.update(path.s[idx], v, frame.tl_id,
                                        frame.tl_state, t,
                                        blocked=(obs.blocking or jy.yielding),
                                        yielding=jy.yielding)
                    v_cmd = min(v_cmd, sig.v_limit)

                # 횡단보도(항목 12)는 **신호·객체 뒤에** 돈다. 그 둘이 겨냥한
                # 정지 지점이 횡단보도 위인지를 봐야 하기 때문이다.
                if xw_policy is not None:
                    xw = xw_policy.update(
                        path.s[idx], v, t,
                        stop_s=_stop_target(path.s[idx], sig, obs, tt, jy),
                        # 양보 정지는 신호 정지와 같은 취급 ― 탈출로 밀지 않는다
                        tl_stop_ahead=((sig.stopping and sig.dist > 0.0)
                                       or jy.yielding),
                        front_clear=(None if obs_policy is None
                                     else obs.front_clear),
                        # 보행자 때문에 서 있는 동안은 탈출하지 않는다 (항목 10 우선)
                        ped_hold=(obs.stopping
                                  and obs.kind == obstacles.PEDESTRIAN))
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
                        sig_state=sig.state, sig_dist=sig.dist,
                        obs=(None if obs_policy is None else obs),
                        finished=finished, yielding=jy.yielding)
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

                # 정지 목표에 닿았는데 종료선을 못 넘었으면 끝 접선으로 저속 직진한다
                # (creep). 최후 안전망과 같은 자리·같은 방식이고 완주가 우선이다.
                if fin.v_floor > v_cmd:
                    v_cmd = fin.v_floor
                # done은 종료선을 넘은 뒤에만 참이다 (creep 포기 시에만 예외)
                if fin.done:
                    finished = True

                if finished:
                    steer, v_cmd = 0.0, 0.0
                accel = SPEED_KP * (v_cmd - v)
                if v_cmd <= 0.0:                    # 정지 명령은 확실히 제동
                    accel = min(accel, -1.5)
                # 신호 정지는 속도 상한만으로는 프로파일을 못 따라간다.
                # P 제어의 정상상태 오차만큼 위로 달려 정지선을 넘는다.
                accel = min(accel, sig.accel_limit, xw.accel_limit,
                            jy.accel_limit)
                # obs.accel_limit은 막은 객체를 향한 감속 피드포워드라
                # 최후 안전망의 v_floor를 그대로 무력화한다. 그때만 뺀다.
                if not det.forcing:
                    accel = min(accel, obs.accel_limit, tt.accel_limit)
                accel = max(ACCEL_MIN, min(ACCEL_MAX, accel))

                # 방향지시등(항목 13)은 속도에 영향이 없어 위 합류에 끼지
                # 않는다. CTRL의 3번째 바이트로만 나간다.
                if turn_policy is not None and not finished:
                    turn = turn_policy.update(path.s[idx], v, t)
                last_cmd = (steer, accel)
            except Exception as exc:                # 계산 실패해도 루프는 유지
                print(f'\n[pp] 계산 오류: {exc!r} ― 마지막 명령 유지')
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
                rz = redzone.RedzoneCmd(action='error')
                fin = finish.FinishCmd(action='error')
                tt = ttc.TtcCmd(action='error')
                jy = junction_yield.YieldCmd(action='error')

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
                          dt_act=det.action, dt_dist=det.dist,
                          rz_ratio=rz.ratio, rz_far=rz.ratio_far,
                          rz_in=rz.in_code, fin_lon=fin.lon,
                          tt_act=tt.action, tt_tcpa=tt.t_cpa, tt_dcpa=tt.d_cpa,
                          jy_act=jy.action, jy_dist=jy.dist, jy_n=jy.n)

            print(f'[{n:5d}] ego({frame.ego_x:8.2f},{frame.ego_y:9.2f}) '
                  f'h={math.degrees(frame.ego_heading):+7.1f}° '
                  f'v={v * 3.6:5.1f}→{v_cmd * 3.6:5.1f}km/h | '
                  + (f'{seg} ' if len(bounds) > 1 else '')
                  + f'idx={idx:3d}/{path.finish_idx} 남은{path.finish_s - path.s[idx]:6.1f}m '
                  f'tgt({tx:8.2f},{ty:9.2f}) Ld={ld:4.1f} | '
                  f'steer={steer:+.3f}rad({math.degrees(steer):+5.1f}°) '
                  f'a={accel:+5.2f}'
                  + ('' if sig.action in ('none', 'error')
                     else f' | TL:{sig.action:5s}{sig.dist:5.1f}m')
                  + ('' if obs.action in ('none', 'error')
                     else f' | OB:{obs.action:6s}{obs.kind[:3]}'
                          f'{obs.dist:5.1f}m{obs.lat:+5.1f}')
                  + ('' if tt.action in ('none', 'error')
                     else f' | TT:{tt.action:5s}{tt.t_cpa:4.1f}s{tt.d_cpa:4.1f}m')
                  + ('' if jy.action in ('none', 'error')
                     else f' | JY:{jy.action:8s}{jy.dist:6.1f}m n={jy.n}')
                  + ('' if xw.action in ('none', 'error')
                     else f' | XW:{xw.action:6s}{xw.dist:6.1f}m')
                  + ('' if det.action in ('none', 'error')
                     else f' | DT:{det.action:11s}{det.dist:6.1f}m')
                  + ('' if rz.action in ('error', 'off')
                     else (' | RZ:lost' if rz.action == 'lost'
                           else f' | RZ:{rz.ratio_far:.2f}/{rz.ratio:.2f}'
                                + ('*' if rz.in_zone else '+' if rz.hold else ' ')))
                  + ('' if turn.signal == TURN_OFF
                     else f' | SIG:{turnsignal.SIGNAL_NAME[turn.signal]}')
                  + (f' | FIN:{fin.action[:5]}{fin.lon:+.1f}'
                     if fin.action in ('passed', 'done', 'creep') else '')
                  + ('  [완료]' if finished else ''))

            if finished:
                print(f'\n[pp] 경로 끝 도달 ― 정지 완료 (종료선 {fin.lon:+.1f}m '
                      + ('통과' if fin.passed else '*** 미통과 ***') + ')')
                log.event('finish',
                          f'{path.finish_s:.1f}m 주행 {n}프레임, '
                          f'종료선({fin_policy.line.label()}) {fin.lon:+.1f}m '
                          + ('지나 정지' if fin.passed else '― *** 미통과 정지 ***'), t=t,
                          ego_x=frame.ego_x, ego_y=frame.ego_y, speed=v)
                break
    except KeyboardInterrupt:
        print('\n[pp] 사용자 종료')
        log.event('note', '사용자 종료(Ctrl+C)')
    finally:
        client.close()                              # close()가 감속 명령을 보낸다
        if rz_policy is not None:
            log.note('redzone_stats', rz_policy.stats())
            rz_policy.close()                       # ffmpeg를 확실히 죽인다
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
    ap.add_argument('--no-ttc', action='store_true',
                    help='TTC 충돌 예측을 끈다 (회귀 비교용)')
    ap.add_argument('--ttc-gap', type=float, default=ttc.GAP_CRIT,
                    help='TTC 풋프린트 간격 임계 [m]. 예측 간격이 이 아래면 충돌 '
                         '경로로 본다 (교차·회전 상대는 예측 시각에 비례해 넓힌다)')
    ap.add_argument('--ttc-dcrit', type=float, default=None,
                    help='(폐기) 예전 중심 거리 임계. 주면 경고하고 무시한다')
    ap.add_argument('--no-ttc-ctrv', action='store_true',
                    help='TTC 등회전 예측을 끄고 직선 외삽만 쓴다 (회귀 비교용)')
    ap.add_argument('--no-junction-yield', action='store_true',
                    help='교차로 진입 양보를 끈다 (회귀 비교용)')
    ap.add_argument('--yield-max-s', type=float,
                    default=junction_yield.YIELD_MAX_S,
                    help='교차로 양보 상한 [s]. 넘으면 가장 긴 틈에서 출발한다')
    ap.add_argument('--no-turn-signal', action='store_true',
                    help='방향지시등을 끈다 (회귀 비교용)')
    ap.add_argument('--turn-lead', type=float, default=turnsignal.LEAD_S,
                    help='방향지시등 선행 점등 시간 [s]. 요건은 3초')
    ap.add_argument('--no-crosswalk', action='store_true',
                    help='횡단보도 정차 금지를 끈다 (회귀 비교용)')
    ap.add_argument('--no-detour', action='store_true',
                    help='영구 정지 객체 우회와 최후 안전망을 끈다 (회귀 비교용)')
    ap.add_argument('--detour-confirm', type=float, default=None,
                    help='막힘 확인 시간 [s]. 생략하면 맥락 3단계 '
                         f'({detour.DETOUR_CONFIRM_FAST_S}/{detour.DETOUR_CONFIRM_S}/'
                         f'{detour.DETOUR_CONFIRM_QUEUE_S}s). 주면 세 단계 전부 그 값')
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
    ap.add_argument('--no-redzone', action='store_true',
                    help='붉은 보호구역 카메라 판정을 끈다 (회귀 비교용)')
    ap.add_argument('--redzone-host', default='',
                    help='RTSP 호스트. 생략하면 --host')
    ap.add_argument('--redzone-url', default='',
                    help='RTSP URL 전체를 덮어쓴다 (기본 rtsp://<host>:'
                         f'{redzone.RTSP_PORT}/{redzone.RTSP_PATH}). '
                         'lavfi:<spec>도 된다')
    ap.add_argument('--zone-speed', type=float, default=redzone.ZONE_SPEED_KMH,
                    help='붉은 구역 지시속도 [km/h]. 평가 기준 30, 오버슈트 감안')
    ap.add_argument('--redzone-crop',
                    default=':'.join(str(v) for v in redzone.CROP),
                    help='카메라 crop W:H:X:Y (1920x1080). '
                         '위 절반이 진입, 아래 절반이 이탈 판정')
    ap.add_argument('--redzone-enter', type=float, default=redzone.ENTER_RATIO,
                    help='진입 임계 (max(far, near) 붉은 비율)')
    ap.add_argument('--redzone-exit', type=float, default=redzone.EXIT_RATIO,
                    help='이탈 임계 (near 띠 붉은 비율)')
    ap.add_argument('--zone-hold', type=float, default=redzone.ZONE_HOLD_M,
                    help='이탈 뒤 이 주행거리[m]만큼 구간 속도를 유지한다 '
                         '(붉은 노면이 끊기는 구간 대비). 0이면 끔')
    ap.add_argument('--redzone-stub', action='append', default=[],
                    metavar='x0,y0,x1,y1',
                    help='sim 전용. 카메라 대신 경로 위 두 끝점 사이를 붉은 구역으로 본다. '
                         '반복하면 회랑이 여러 개 (끊긴 노면 재현)')
    ap.add_argument('--finish-overrun', type=float,
                    default=finish.FINISH_OVERRUN_M,
                    help='종료선(경로 끝)을 이만큼 지나쳐 정지한다 [m]. 후륜축이 '
                         '두 라바콘 사이 종료선을 넘어야 완주다')
    ap.add_argument('--finish-zone', type=float, default=finish.FINISH_ZONE_M,
                    help='경로 끝 이 안은 종료 구간 [m] — 장애물(라바콘) 무시, '
                         '우회 금지. 정지선 종료선의 앞쪽 탐색 한계이기도 하다')
    ap.add_argument('--finish-search', type=float, default=finish.FINISH_SEARCH_M,
                    help='경로 끝 너머 이 안의 정지선(이벤트 CSV beyond_end)을 '
                         '종료선으로 본다 [m]')
    ap.add_argument('--finish-fallback-overrun', type=float,
                    default=finish.FINISH_FALLBACK_OVERRUN_M,
                    help='정지선도 라바콘도 없을 때 경로 끝을 이만큼 지나쳐 선다 [m]')
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
