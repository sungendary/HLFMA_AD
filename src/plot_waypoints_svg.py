#!/usr/bin/env python3
"""waypoint를 SVG로 그린다. 표준 라이브러리만 사용한다.

matplotlib이 Windows 앱 제어 정책에 막히는 환경을 위한 대안.
결과 SVG는 브라우저로 열면 된다.

    python plot_waypoints_svg.py --file waypoints_split.csv
    python plot_waypoints_svg.py --file waypoints_split.csv --route route_example.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import webbrowser
from collections import defaultdict
from pathlib import Path

PALETTE = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4',
           '#008080', '#9a6324', '#800000', '#808000', '#000075',
           '#f032e6', '#469990', '#bfef45', '#fabed4', '#dcbeff']

CANVAS = 1400
MARGIN = 70


def load_waypoints(path: str) -> dict[str, list[tuple[float, float, float]]]:
    segs: dict[str, list[tuple[float, float, float]]] = defaultdict(list)
    with open(path, encoding='utf-8-sig') as f:
        for row in csv.DictReader(f):
            segs[row['segment_id']].append(
                (float(row['x']), float(row['y']), float(row.get('heading', 0) or 0)))
    return dict(segs)


def load_route(path: str) -> list[tuple[int, float, float]]:
    pts = []
    with open(path, encoding='utf-8-sig') as f:
        for row in csv.DictReader(f):
            pts.append((int(row['seq']), float(row['x']), float(row['y'])))
    return sorted(pts)


def seg_length(pts) -> float:
    return sum(math.dist(pts[i][:2], pts[i + 1][:2]) for i in range(len(pts) - 1))


def main() -> None:
    ap = argparse.ArgumentParser(description='waypoint SVG 뷰어')
    ap.add_argument('--file', default='waypoints.csv')
    ap.add_argument('--route', help='대회 경로 CSV (선택)')
    ap.add_argument('--out', default='waypoints.svg')
    ap.add_argument('--arrows', type=int, default=15,
                    help='진행 방향 화살표 간격 (점 개수)')
    ap.add_argument('--no-open', action='store_true')
    args = ap.parse_args()

    segments = load_waypoints(args.file)
    if not segments:
        print('waypoint가 없습니다.')
        return

    route = load_route(args.route) if args.route else []

    # 좌표 범위
    all_x = [p[0] for pts in segments.values() for p in pts] + [r[1] for r in route]
    all_y = [p[1] for pts in segments.values() for p in pts] + [r[2] for r in route]
    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)
    span = max(max_x - min_x, max_y - min_y, 1.0)
    scale = (CANVAS - 2 * MARGIN) / span
    cx, cy = (min_x + max_x) / 2, (min_y + max_y) / 2

    def to_px(x: float, y: float) -> tuple[float, float]:
        # VTD는 y가 위쪽이 +, SVG는 아래쪽이 + 이므로 y를 뒤집는다
        return (CANVAS / 2 + (x - cx) * scale,
                CANVAS / 2 - (y - cy) * scale)

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{CANVAS}" height="{CANVAS}" '
        f'viewBox="0 0 {CANVAS} {CANVAS}" style="background:#fafafa">',
        '<style>text{font-family:system-ui,sans-serif}</style>',
    ]

    # 100m 격자
    step = 100.0
    gx = math.floor(min_x / step) * step
    while gx <= max_x + step:
        px, _ = to_px(gx, 0)
        parts.append(f'<line x1="{px:.1f}" y1="0" x2="{px:.1f}" y2="{CANVAS}" '
                     f'stroke="#e0e0e0" stroke-width="1"/>')
        parts.append(f'<text x="{px + 3:.1f}" y="14" font-size="10" fill="#999">'
                     f'{gx:.0f}</text>')
        gx += step
    gy = math.floor(min_y / step) * step
    while gy <= max_y + step:
        _, py = to_px(0, gy)
        parts.append(f'<line x1="0" y1="{py:.1f}" x2="{CANVAS}" y2="{py:.1f}" '
                     f'stroke="#e0e0e0" stroke-width="1"/>')
        parts.append(f'<text x="3" y="{py - 3:.1f}" font-size="10" fill="#999">'
                     f'{gy:.0f}</text>')
        gy += step

    print(f'{"구간":<28} {"점":>6} {"길이(m)":>10}')
    print('-' * 48)
    total_len = 0.0

    for i, (name, pts) in enumerate(sorted(segments.items())):
        color = PALETTE[i % len(PALETTE)]
        length = seg_length(pts)
        total_len += length
        print(f'{name:<28} {len(pts):>6} {length:>10.1f}')

        coords = ' '.join(f'{px:.1f},{py:.1f}'
                          for px, py in (to_px(p[0], p[1]) for p in pts))
        parts.append(f'<polyline points="{coords}" fill="none" stroke="{color}" '
                     f'stroke-width="2.5" stroke-linejoin="round" opacity="0.85"/>')

        # 진행 방향 화살표
        if args.arrows > 0:
            for j in range(0, len(pts) - 1, args.arrows):
                x0, y0 = to_px(pts[j][0], pts[j][1])
                hd = pts[j][2]
                dx, dy = math.cos(hd) * 9, -math.sin(hd) * 9
                parts.append(
                    f'<line x1="{x0:.1f}" y1="{y0:.1f}" '
                    f'x2="{x0 + dx:.1f}" y2="{y0 + dy:.1f}" '
                    f'stroke="{color}" stroke-width="1.6"/>'
                    f'<circle cx="{x0 + dx:.1f}" cy="{y0 + dy:.1f}" r="2" '
                    f'fill="{color}"/>')

        sx, sy = to_px(pts[0][0], pts[0][1])
        ex, ey = to_px(pts[-1][0], pts[-1][1])
        parts.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="6" fill="{color}" '
                     f'stroke="#fff" stroke-width="2"/>')
        parts.append(f'<rect x="{ex - 5:.1f}" y="{ey - 5:.1f}" width="10" height="10" '
                     f'fill="{color}" stroke="#fff" stroke-width="2"/>')
        parts.append(f'<text x="{sx + 10:.1f}" y="{sy - 8:.1f}" font-size="13" '
                     f'font-weight="600" fill="{color}">{name}</text>')

    print('-' * 48)
    print(f'{"합계":<28} {sum(len(p) for p in segments.values()):>6} {total_len:>10.1f}\n')

    # 대회 경로 지점
    if route:
        pxy = [to_px(r[1], r[2]) for r in route]
        line = ' '.join(f'{p[0]:.1f},{p[1]:.1f}' for p in pxy)
        parts.append(f'<polyline points="{line}" fill="none" stroke="#d00" '
                     f'stroke-width="1.5" stroke-dasharray="6 4" opacity="0.6"/>')
        for (seq, rx, ry), (px, py) in zip(route, pxy):
            parts.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="8" fill="#d00" '
                         f'stroke="#fff" stroke-width="2"/>')
            parts.append(f'<text x="{px + 12:.1f}" y="{py + 5:.1f}" font-size="15" '
                         f'font-weight="700" fill="#d00">P{seq}</text>')

        print('경로 지점과 녹화 궤적의 거리:')
        for seq, rx, ry in route:
            best, bseg = min(
                ((math.hypot(p[0] - rx, p[1] - ry), n)
                 for n, pts in segments.items() for p in pts),
                default=(float('inf'), '-'))
            flag = '' if best < 5 else '   <- 미녹화'
            print(f'  P{seq}: {best:7.1f}m  ({bseg}){flag}')

    parts.append(f'<text x="{MARGIN}" y="{CANVAS - 20}" font-size="14" fill="#555">'
                 f'● 시작  ■ 끝  화살표=진행방향  격자 100m  '
                 f'총 {total_len:.0f}m / {len(segments)}구간</text>')
    parts.append('</svg>')

    out = Path(args.out).resolve()
    out.write_text('\n'.join(parts), encoding='utf-8')
    print(f'저장: {out}')
    if not args.no_open:
        webbrowser.open(out.as_uri())


if __name__ == '__main__':
    main()
