#!/usr/bin/env python3
"""현장에서 받은 route CSV가 우리 지도로 커버되는지 즉시 확인한다.

    python route_check.py --route route.csv
    python route_check.py --route route.csv --waypoints ../data/xodr_reference_lines.csv

**경로를 만드는 도구가 아니다.** 15분 안에 "이 route를 지금 코드로 달릴 수 있나,
못 달린다면 얼마나 벗어나나"를 판정하는 진단 도구다. 미커버로 나와도 상관없다.
얼마나 벗어나는지 아는 것이 목적이다.

--waypoints 는 인자로 받으므로 xodr 파서가 나오면 그 출력 CSV(같은
segment_id,seq,x,y,heading 형식)로 그대로 바꿔 끼울 수 있다.

route CSV 형식 (CLAUDE.md)
    seq,x,y     VTD 월드 직교좌표. 첫 점=출발, 마지막 점=종료
    사이 지점은 두 개씩 짝: seq 2·3 = 교차로1 진입·진출, 4·5 = 교차로2 ...
    짝 사이 구간이 교차로 내부. **점만 직선으로 이으면 도로를 벗어난다.**
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import shutil
import subprocess
import sys
import time
from collections import defaultdict

COVERED_M = 3.0             # 이 안이면 녹화본이 그 점을 지난다고 본다
MARGINAL_M = 15.0           # 이 밖이면 사실상 미커버


def load_route(path: str) -> list[tuple[int, float, float]]:
    """seq,x,y. plot_waypoints_svg.load_route와 같은 스키마."""
    pts = []
    with open(path, encoding='utf-8-sig', newline='') as fh:
        for row in csv.DictReader(fh):
            pts.append((int(row['seq']), float(row['x']), float(row['y'])))
    if not pts:
        raise SystemExit(f'{path}: 점이 없다')
    return sorted(pts)


def load_segments(path: str) -> dict[str, list[tuple[float, float]]]:
    """segment_id,seq,x,y,heading ― pure_pursuit.load_path와 같은 그룹핑."""
    segs: dict[str, list[tuple[float, float]]] = defaultdict(list)
    with open(path, encoding='utf-8-sig', newline='') as fh:
        for row in csv.DictReader(fh):
            segs[row['segment_id']].append((float(row['x']), float(row['y'])))
    return dict(segs)


def point_to_segment(px, py, ax, ay, bx, by):
    """점-선분 수직거리와 선분 위 매개변수 t. 점-점 거리보다 정확하다.

    녹화본은 점 간격이 0.3m라 차이가 작지만, xodr 출력(1m 간격)으로
    갈아끼우면 의미가 생긴다.
    """
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    if den < 1e-12:
        return math.hypot(px - ax, py - ay), 0.0, ax, ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / den))
    qx, qy = ax + t * dx, ay + t * dy
    return math.hypot(px - qx, py - qy), t, qx, qy


def nearest_on_route(px, py, segs):
    """모든 세그먼트를 뒤져 가장 가까운 (세그먼트, 인덱스, 거리, 횡오프셋)."""
    best = None
    for name, pts in segs.items():
        for i in range(len(pts) - 1):
            (ax, ay), (bx, by) = pts[i], pts[i + 1]
            d, t, qx, qy = point_to_segment(px, py, ax, ay, bx, by)
            if best is None or d < best[2]:
                # 진행방향 왼쪽이 양수인 부호 있는 횡오프셋
                th = math.atan2(by - ay, bx - ax)
                lat = -math.sin(th) * (px - qx) + math.cos(th) * (py - qy)
                best = (name, i + (1 if t > 0.5 else 0), d, lat, len(pts))
    return best


def verdict(d: float) -> str:
    if d <= COVERED_M:
        return '커버'
    if d <= MARGINAL_M:
        return '경계'
    return '미커버'


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description='route CSV 커버리지 확인')
    ap.add_argument('--route', required=True, help='현장에서 받은 route CSV')
    ap.add_argument('--waypoints', default=os.path.normpath(
        os.path.join(here, '..', 'data', 'waypoints_v2.csv')),
        help='대조할 지도. xodr 출력 CSV로 바꿔 낄 수 있다')
    ap.add_argument('--svg', help='겹쳐 그린 SVG 출력 경로')
    ap.add_argument('--no-copy', action='store_true',
                    help='route CSV 원본 보존을 건너뛴다')
    args = ap.parse_args()

    route = load_route(args.route)
    segs = load_segments(args.waypoints)
    if not segs:
        sys.exit(f'{args.waypoints}: 세그먼트가 없다')

    # 현장에서 받은 route는 재현 불가능하다. 먼저 복사부터 한다.
    if not args.no_copy:
        dest = os.path.join(here, '..', 'logs',
                            time.strftime('route_%Y%m%d_%H%M%S.csv'))
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        try:
            shutil.copyfile(args.route, dest)
            print(f'[route] 원본 보존: {os.path.normpath(dest)}')
        except OSError as exc:
            print(f'[route] 복사 실패: {exc}')

    total_pts = sum(len(p) for p in segs.values())
    print(f'[route] {args.route}: {len(route)}점')
    print(f'[route] 지도 {os.path.basename(args.waypoints)}: '
          f'세그먼트 {len(segs)}개, 점 {total_pts}개\n')

    # ------------------------------------------------------------------
    print('=' * 74)
    print('1. route 각 점이 지도 위에 있나')
    print('=' * 74)
    print(f'{"seq":>4} {"x":>10} {"y":>10} {"최근접 세그먼트":>16} {"idx":>5} '
          f'{"거리":>8} {"횡":>7}  판정')

    hits = []
    for seq, x, y in route:
        name, idx, d, lat, n = nearest_on_route(x, y, segs)
        hits.append((seq, x, y, name, idx, d, lat, n))
        side = '좌' if lat > 0 else '우'
        print(f'{seq:>4} {x:10.2f} {y:10.2f} {name:>16} {idx:5d} '
              f'{d:7.2f}m {abs(lat):6.2f}{side}  {verdict(d)}')

    n_cov = sum(1 for h in hits if h[5] <= COVERED_M)
    n_mar = sum(1 for h in hits if COVERED_M < h[5] <= MARGINAL_M)
    print(f'\n커버 {n_cov} / 경계 {n_mar} / 미커버 {len(hits) - n_cov - n_mar}'
          f'  (총 {len(hits)}점)')

    # ------------------------------------------------------------------
    # CLAUDE.md의 짝 규칙: 1=출발, (2,3)=교차로1 진입·진출, (4,5)=교차로2 ...
    # 마지막=종료. 따라서 도로 구간은 1->2, 3->4, 5->6, ... 이고
    # 짝 사이(2->3, 4->5, ...)가 교차로 내부다.
    print('\n' + '=' * 74)
    print('2. 구간별 판정  (짝 규칙: 짝 사이가 교차로 내부)')
    print('=' * 74)

    for k in range(len(hits) - 1):
        a, b = hits[k], hits[k + 1]
        inside = (k % 2 == 1)          # 0-based: 1->2 이 교차로 내부
        kind = '교차로' if inside else '도로  '
        straight = math.hypot(b[1] - a[1], b[2] - a[2])
        same = a[3] == b[3]
        fwd = (b[4] - a[4]) if same else None
        note = ''
        if not same:
            note = f'세그먼트 바뀜 {a[3]}→{b[3]}'
        elif fwd is not None and fwd <= 0:
            note = f'*** 인덱스가 전진하지 않는다 ({a[4]}→{b[4]}) ― 순서 확인 ***'
        else:
            note = f'인덱스 {a[4]}→{b[4]} (+{fwd})'
        worst = max(a[5], b[5])
        print(f'  seq {a[0]}→{b[0]} {kind} 직선 {straight:7.2f}m  '
              f'최대이탈 {worst:6.2f}m  {verdict(worst)}  {note}')

    # ------------------------------------------------------------------
    print('\n' + '=' * 74)
    print('3. pure_pursuit 에 넣을 구간')
    print('=' * 74)

    order: list[str] = []
    for h in hits:
        if not order or order[-1] != h[3]:
            order.append(h[3])
    dupes = len(order) != len(set(order))

    if n_cov + n_mar == len(hits):
        print('제안:')
        print(f'  python pure_pursuit.py --host 192.168.50.11 --speed 20 \\')
        print(f'      --segments {",".join(order)} --route {args.route}')
        if dupes:
            print('\n  *** 같은 세그먼트가 여러 번 나온다. route가 같은 길을 '
                  '되돌아오거나 최근접 판정이 흔들린 것이다. SVG로 확인할 것 ***')
        if n_mar:
            print(f'\n  주의: 경계 판정 {n_mar}점. 그대로 달리면 경로 이탈 '
                  '리스폰(0.5초)이 날 수 있다')
    else:
        print('이 route는 현재 지도로 달릴 수 없다.')
        for seq, x, y, name, _i, d, lat, _n in hits:
            if d > MARGINAL_M:
                brg = math.degrees(math.atan2(y - 0, x - 0))
                print(f'  seq {seq} ({x:.1f},{y:.1f}) ― 가장 가까운 '
                      f'{name}에서 {d:.1f}m '
                      f'{"왼쪽" if lat > 0 else "오른쪽"}으로 벗어남')
        print('\n  → 이 구간을 수동 주행으로 녹화하거나(drive_and_record.py),')
        print('     xodr 도로망이 나온 뒤 --waypoints 를 바꿔 다시 확인할 것')

    # ------------------------------------------------------------------
    if args.svg:
        script = os.path.join(here, 'plot_waypoints_svg.py')
        cmd = [sys.executable, script, '--file', args.waypoints,
               '--route', args.route, '--out', args.svg, '--no-open']
        print(f'\n[route] SVG 생성: {args.svg}')
        try:
            subprocess.run(cmd, check=True)
        except (OSError, subprocess.CalledProcessError) as exc:
            print(f'[route] SVG 실패: {exc}')


if __name__ == '__main__':
    main()
