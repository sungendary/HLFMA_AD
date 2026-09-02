#!/usr/bin/env python3
"""세션 로그를 읽어 사후 분석 요약을 낸다.

    python analyze_session.py ../logs/pp_20260903_101500

무엇을 확인하려고 만들었나 — 미해결 목록에 직접 대응한다.

  · objects[].id 가 프레임 간 안정적인가 (미해결 #4)
    id별 등장 구간과 결손, 그리고 "같은 id인데 위치가 튀는" 경우를 찾는다.
    id가 재사용되면 추적 로직이 엉뚱한 객체를 따라가므로 치명적이다.
  · 신호등 state 전이가 언제 어디서 일어났나
  · 리스폰이 언제 몇 번, 어디서 어디로
  · 수신 주기가 실제로 20Hz인가 (로깅을 추가해 루프가 느려지지 않았는지)
  · 보행자 판별 임계값(length<1.2, height<2.2)이 실제 치수와 맞는가

표준 라이브러리만 쓴다.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict

FRAME_SIZE = 1109
FRAME_DT = 0.05
# 이보다 빠른 "이동"은 물리적으로 불가능하다 -> id 재사용을 의심한다
IMPLAUSIBLE_SPEED = 60.0        # m/s (216 km/h)


def read_csv(path: str) -> list[dict]:
    if not os.path.exists(path):
        return []
    with open(path, newline='', encoding='utf-8-sig') as fh:
        return list(csv.DictReader(fh))


def quantiles(vals: list[float]) -> dict:
    if not vals:
        return {}
    s = sorted(vals)
    def q(p): return s[min(len(s) - 1, int(p * len(s)))]
    return {'min': s[0], 'p50': q(0.5), 'p95': q(0.95), 'max': s[-1],
            'mean': sum(s) / len(s)}


def bar(n: int, total: int, width: int = 40) -> str:
    return '#' * max(1, round(width * n / total)) if n else ''


def section(title: str) -> None:
    print(f'\n{"=" * 72}\n{title}\n{"=" * 72}')


# ----------------------------------------------------------------------

def check_integrity(d: str, drive: list[dict], objs: list[dict]) -> None:
    section('1. 무결성')

    raw = os.path.join(d, 'frames.bin')
    if os.path.exists(raw):
        size = os.path.getsize(raw)
        n, rest = divmod(size, FRAME_SIZE)
        print(f'frames.bin   {size:,}B = {n}프레임' +
              (f'  *** 끝에 {rest}B 남음 (중간에 끊김) ***' if rest else '  (배수 일치)'))
        if n != len(drive):
            # raw_sink는 수신 즉시 호출되고 drive.csv는 제어까지 끝난 프레임만
            # 남는다. 재연결·예외가 있었다면 차이가 나는 것이 정상이다.
            print(f'             drive.csv {len(drive)}행과 {n - len(drive):+d} 차이 '
                  f'— 재연결/예외로 버려진 프레임 (events.csv 확인)')
    else:
        print('frames.bin   없음 — 원본이 안 남았다')

    print(f'drive.csv    {len(drive):,}행')
    print(f'objects.csv  {len(objs):,}행')

    per_frame = defaultdict(int)
    for r in objs:
        per_frame[r['n']] += 1
    bad = [r['n'] for r in drive if per_frame.get(r['n'], 0) != int(r['n_obj'])]
    if bad:
        print(f'*** 불일치: {len(bad)}개 프레임에서 n_obj와 objects.csv 행 수가 다르다 '
              f'(예: n={bad[:5]}) ***')
    else:
        print('             n_obj와 objects.csv 행 수 전부 일치')


def check_rate(drive: list[dict]) -> None:
    section('2. 수신 주기')
    ts = [float(r['t']) for r in drive]
    if len(ts) < 3:
        print('프레임이 너무 적다')
        return
    dts = [(b - a) * 1000.0 for a, b in zip(ts, ts[1:])]
    q = quantiles(dts)
    print(f'프레임 {len(ts)}개, {ts[-1] - ts[0]:.1f}초, '
          f'평균 {len(dts) / (ts[-1] - ts[0]):.2f} fps')
    print(f'간격 [ms]  평균 {q["mean"]:.1f}  중앙 {q["p50"]:.1f}  '
          f'p95 {q["p95"]:.1f}  최대 {q["max"]:.1f}   (공칭 50.0)')

    edges = [0, 40, 45, 50, 55, 60, 70, 100, 1e9]
    labels = ['<40', '40-45', '45-50', '50-55', '55-60', '60-70', '70-100', '>100']
    for lo, hi, lab in zip(edges, edges[1:], labels):
        c = sum(1 for v in dts if lo <= v < hi)
        if c:
            print(f'  {lab:>7}ms {c:5d} {bar(c, len(dts))}')
    slow = sum(1 for v in dts if v > 65.5)
    if slow:
        print(f'  실측 기준(최대 65.5ms)을 넘은 프레임 {slow}개 '
              f'({100 * slow / len(dts):.1f}%)')


def check_object_ids(objs: list[dict], respawns: set[int]) -> None:
    section('3. objects[].id 안정성  (미해결 #4)')
    if not objs:
        print('객체가 한 번도 잡히지 않았다 (빈 맵이거나 80m 이내에 아무것도 없음)')
        return
    if respawns:
        print(f'리스폰 {len(respawns)}회(n={sorted(respawns)}). ego가 순간이동하면'
              f' 주변 객체도 통째로 갈리므로 그 경계는 연속성 판정에서 뺀다.\n')

    tracks: dict[str, list[tuple[int, float, float, float, float]]] = defaultdict(list)
    for r in objs:
        tracks[r['id']].append((int(r['n']), float(r['t']), float(r['x']),
                                float(r['y']), float(r['speed'])))

    per_frame = defaultdict(int)
    for r in objs:
        per_frame[r['n']] += 1
    print(f'서로 다른 id {len(tracks)}개, 동시 최대 {max(per_frame.values())}개'
          + ('  *** 30슬롯 상한에 닿았다 — 가려진 객체가 있다 ***'
             if max(per_frame.values()) >= 30 else ''))

    print(f'\n{"id":>8} {"등장":>6} {"프레임구간":>13} {"결손":>5} '
          f'{"최대이동속도":>12}  판정')
    suspects = []
    for oid, pts in sorted(tracks.items(), key=lambda kv: -len(kv[1])):
        pts.sort()
        span = pts[-1][0] - pts[0][0] + 1
        gaps = span - len(pts)
        worst = 0.0
        for a, b in zip(pts, pts[1:]):
            dn = b[0] - a[0]
            # 리스폰을 사이에 낀 구간은 건너뛴다. 안 그러면 리스폰 한 번에
            # 모든 id가 "재사용 의심"으로 찍혀 판정 자체가 쓸모없어진다.
            # 경계를 한 프레임 넉넉히 잡는다. 이벤트의 n이 "직전까지 기록한
            # 프레임 수"라 리스폰 프레임 자신과 1 어긋난다. 한 쌍 더 버리는
            # 비용은 없고, 놓치면 모든 id가 오판된다.
            if any(a[0] <= r <= b[0] for r in respawns):
                continue
            dist = math.hypot(b[2] - a[2], b[3] - a[3])
            worst = max(worst, dist / max(dn * FRAME_DT, 1e-6))
        verdict = ''
        if worst > IMPLAUSIBLE_SPEED:
            verdict = '*** 위치 불연속 — id 재사용 의심'
            suspects.append(oid)
        elif gaps:
            verdict = '결손 있음 (80m 밖으로 나갔다 복귀?)'
        if len(tracks) <= 40 or len(pts) > 5 or verdict:
            print(f'{oid:>8} {len(pts):6d} {pts[0][0]:6d}-{pts[-1][0]:<6d} '
                  f'{gaps:5d} {worst:11.1f}m/s  {verdict}')

    print()
    if suspects:
        print(f'*** id {", ".join(suspects[:10])} 에서 위치가 튄다. '
              f'id를 그대로 믿고 추적하면 안 된다 ***')
    elif any(p[-1][0] - p[0][0] + 1 != len(p) for p in tracks.values()):
        print('결손은 있으나 위치는 전부 연속적이다 — id는 유지되는 것으로 보인다')
    else:
        print('모든 id가 끊김 없이 연속적이다 — id 추적 가능')


def check_pedestrian_threshold(objs: list[dict]) -> None:
    section('4. 보행자 판별 임계값  (length < 1.2 and height < 2.2)')
    if not objs:
        print('객체 없음')
        return
    dims: dict[str, tuple[float, float, float, int]] = {}
    for r in objs:
        dims[r['id']] = (float(r['length']), float(r['width']),
                         float(r['height']), int(r['is_ped']))
    print(f'{"id":>8} {"length":>8} {"width":>8} {"height":>8}  현재 판정')
    for oid, (ln, wd, ht, ped) in sorted(dims.items(),
                                         key=lambda kv: kv[1][0]):
        print(f'{oid:>8} {ln:8.2f} {wd:8.2f} {ht:8.2f}  '
              f'{"보행자" if ped else "차량"}')
    borderline = [o for o, (ln, _w, ht, _p) in dims.items()
                  if 0.9 < ln < 1.6 or 1.9 < ht < 2.5]
    if borderline:
        print(f'\n*** 임계값 근처 id: {", ".join(borderline)} — 기준을 다시 볼 것 ***')
    else:
        print('\n임계값에서 충분히 떨어져 있다')


def show_events(events: list[dict]) -> None:
    section('5. 이벤트')
    if not events:
        print('이벤트 없음')
        return
    by_kind = defaultdict(int)
    for e in events:
        by_kind[e['kind']] += 1
    print('  '.join(f'{k}={v}' for k, v in sorted(by_kind.items())) + '\n')
    for e in events:
        pos = ''
        if e['ego_x'] not in ('', 'nan'):
            pos = f'  ego({float(e["ego_x"]):8.2f},{float(e["ego_y"]):9.2f})'
            if e['speed'] not in ('', 'nan'):
                pos += f' v={float(e["speed"]) * 3.6:5.1f}km/h'
        print(f'  t={float(e["t"]):7.2f}s n={e["n"]:>6} {e["kind"]:<11} '
              f'{e["detail"]}{pos}')


def main() -> None:
    ap = argparse.ArgumentParser(description='세션 로그 분석')
    ap.add_argument('session', help='logs/<run_id> 디렉터리')
    args = ap.parse_args()

    d = args.session
    if not os.path.isdir(d):
        sys.exit(f'{d}: 디렉터리가 아니다')

    meta_path = os.path.join(d, 'meta.json')
    if os.path.exists(meta_path):
        with open(meta_path, encoding='utf-8') as fh:
            meta = json.load(fh)
        section(f'세션 {meta.get("run_id", os.path.basename(d))}')
        for k, v in meta.items():
            print(f'  {k:16} {v}')

    drive = read_csv(os.path.join(d, 'drive.csv'))
    objs = read_csv(os.path.join(d, 'objects.csv'))
    events = read_csv(os.path.join(d, 'events.csv'))
    if not drive:
        sys.exit('drive.csv가 비었다')

    respawns = {int(e['n']) for e in events if e['kind'] == 'respawn'}

    check_integrity(d, drive, objs)
    check_rate(drive)
    check_object_ids(objs, respawns)
    check_pedestrian_threshold(objs)
    show_events(events)
    print()


if __name__ == '__main__':
    main()
