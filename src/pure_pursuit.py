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
    · 리스폰(프레임 간 5m 이상 점프) 감지 시 경로 인덱스를 다시 전역 탐색
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import os
import time

from hlvtd_client import (
    Client, SpeedEstimator, detect_respawn,
    TURN_OFF, STEER_LIMIT, ACCEL_MAX, ACCEL_MIN,
)

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


class FrameLog:
    """수신 프레임과 송신 명령을 CSV로 남긴다 (사후 분석용)."""

    def __init__(self, path: str | None):
        self._fh = None
        if not path:
            return
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        self._fh = open(path, 'w', newline='', encoding='utf-8')
        self._w = csv.writer(self._fh)
        self._w.writerow(['t', 'ego_x', 'ego_y', 'heading', 'speed',
                          'seg', 'idx', 'tgt_x', 'tgt_y', 'ld', 'steer',
                          'accel', 'tl_id', 'tl_state'])
        self._n = 0

    def add(self, *row) -> None:
        if self._fh is None:
            return
        self._w.writerow([f'{v:.4f}' if isinstance(v, float) else v for v in row])
        self._n += 1
        if self._n % 20 == 0:
            self._fh.flush()

    def close(self) -> None:
        if self._fh:
            self._fh.close()
            self._fh = None


def run(args) -> None:
    path, bounds = load_path(args.waypoints, args.segments)
    end_heading = path.end_heading()
    seg_starts = [start for _n, start in bounds]

    def seg_at(i: int) -> str:
        return bounds[bisect.bisect_right(seg_starts, i) - 1][0]

    print(f'[pp] 경로 {len(path.pts)}점, 길이 {path.length:.1f}m, '
          f'끝 접선 {math.degrees(end_heading):+.1f}°')
    for k, (name, start) in enumerate(bounds):
        end = bounds[k + 1][1] if k + 1 < len(bounds) else len(path.pts)
        print(f'[pp]   {k + 1}. {name}  {end - start}점  '
              f'{path.s[end - 1] - path.s[start]:.1f}m')
        if k:
            gap = path.s[start] - path.s[start - 1]     # 앞 구간 끝 -> 이 구간 시작
            warn = (f'  *** 경고: {JOIN_WARN_M:.0f}m 초과. 구간 순서가 맞는지, '
                    f'녹화가 끊기지 않았는지 확인할 것 ***'
                    if gap > JOIN_WARN_M else '')
            print(f'[pp]      이음매 {bounds[k - 1][0]} → {name}: {gap:.2f}m{warn}')

    target_speed = args.speed / 3.6                 # km/h -> m/s
    client = Client(args.host, args.port)
    print(f'[pp] {args.host}:{args.port} 연결 시도...')
    client.connect()
    print('[pp] 연결됨. Ctrl+C로 종료\n')

    log = FrameLog(args.log)
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
                client.connect()
                prev_frame, idx = None, None
                speed_est = SpeedEstimator()
                continue

            n += 1
            try:
                v = speed_est.update(frame.ego_x, frame.ego_y)

                if detect_respawn(frame, prev_frame):
                    print('\n[pp] *** 리스폰 감지 — 경로 인덱스 재탐색 ***')
                    idx = None
                    finished = False
                prev_frame = frame

                if idx is None:                     # 시작 / 리스폰 후 전역 탐색
                    idx = path.nearest(frame.ego_x, frame.ego_y)
                    off = math.hypot(path.pts[idx][0] - frame.ego_x,
                                     path.pts[idx][1] - frame.ego_y)
                    print(f'[pp] 경로 진입점 idx={idx} 이탈거리 {off:.2f}m'
                          + ('  ← 경로에서 멀다. 세그먼트를 확인할 것' if off > 10 else ''))
                else:
                    idx = path.nearest(frame.ego_x, frame.ego_y, start=idx)

                ld = max(LD_MIN, min(LD_MAX, LD_GAIN * v))
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
                accel = max(ACCEL_MIN, min(ACCEL_MAX, accel))
                last_cmd = (steer, accel)
            except Exception as exc:                # 계산 실패해도 루프는 유지
                print(f'\n[pp] 계산 오류: {exc!r} — 마지막 명령 유지')
                steer, accel = last_cmd
                tx = ty = float('nan')
                v, v_cmd, ld = speed_est.speed, 0.0, LD_MIN
                idx = idx or 0

            if not client.send_control(steer, accel, TURN_OFF):
                print('\n[pp] 송신 실패. 재연결...')
                client.connect()
                prev_frame, idx = None, None
                continue

            seg = seg_at(idx)
            log.add(time.monotonic() - t0, frame.ego_x, frame.ego_y,
                    frame.ego_heading, v, seg, idx, tx, ty, ld, steer, accel,
                    frame.tl_id, frame.tl_state)

            print(f'[{n:5d}] ego({frame.ego_x:8.2f},{frame.ego_y:9.2f}) '
                  f'h={math.degrees(frame.ego_heading):+7.1f}° '
                  f'v={v * 3.6:5.1f}→{v_cmd * 3.6:5.1f}km/h | '
                  + (f'{seg} ' if len(bounds) > 1 else '')
                  + f'idx={idx:3d}/{len(path.pts) - 1} 남은{path.remaining(idx):6.1f}m '
                  f'tgt({tx:8.2f},{ty:9.2f}) Ld={ld:4.1f} | '
                  f'steer={steer:+.3f}rad({math.degrees(steer):+5.1f}°) '
                  f'a={accel:+5.2f}' + ('  [완료]' if finished else ''))

            if finished:
                print('\n[pp] 경로 끝 도달 — 정지 완료')
                break
    except KeyboardInterrupt:
        print('\n[pp] 사용자 종료')
    finally:
        client.close()                              # close()가 감속 명령을 보낸다
        log.close()
        if args.log:
            print(f'[pp] 로그 저장: {args.log}')


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    default_csv = os.path.join(here, '..', 'data', 'waypoints_v2.csv')
    default_log = os.path.join(here, '..', 'logs',
                               time.strftime('pp_%Y%m%d_%H%M%S.csv'))

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
    ap.add_argument('--log', default=os.path.normpath(default_log),
                    help='프레임 로그 CSV 경로 (빈 문자열이면 기록 안 함)')

    args = ap.parse_args()
    args.segments = [s.strip() for s in args.segments.split(',') if s.strip()]
    if not args.segments:
        ap.error('--segment 값이 비어 있다')
    run(args)


if __name__ == '__main__':
    main()
