#!/usr/bin/env python3
"""방향지시등·횡단보도 정차 계수기 (평가항목 13·12).

check_redlight.py / check_collision.py의 쌍둥이다. 세션 로그를 정책과 **같은
잣대로** 재고, 베이스라인(그 기능이 없던 로그)과 나란히 놓는다.

    python check_signals.py ../logs/sim_pt2_rule --baseline ../logs/sim_pt2_before

A 방향지시등 (항목 13, 경미 -3)
    차선변경 구간마다 **구간 시작 직전의 연속 점등 시간**을 잰다. 3초 이상이면
    합격이다. 교차로 회전 구간은 규칙이 강제하지 않으므로 참고로만 낸다.

    구간은 turnsignal.build_zones가 만든 것을 그대로 쓴다 — 제어기가 본 것과
    같은 구간이어야 판정이 의미가 있다.

B 횡단보도 정차 (항목 12, 경미 -3)
    후륜축이 횡단보도 구간 안에 있으면서 1km/h 이하인 **연속 구간의 길이**를
    잰다. 3초 이상이면 위반이다.

    **후륜축 기준이다.** 9910 ego 좌표가 곧 후륜축이므로 보정이 없다.

옛 로그에는 turn / cw_act 컬럼이 없다. 전부 get으로 읽어 그대로 0/빈칸으로
떨어지게 둔다 — 베이스라인이 "점등 0초"로 나오는 것이 정확한 기록이다.
"""

from __future__ import annotations

import argparse
import csv
import glob
import io
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import crosswalks
import turnsignal

LEAD_REQ_S = 3.0        # s, 항목 13 요건
DWELL_REQ_S = 3.0       # s, 항목 12 요건
DWELL_MS = 1.0 / 3.6    # m/s, 항목 12의 "1km/h 이하"
RESNAP_M = 6.0          # m, 경로에서 이보다 멀면 인덱스를 전역 재탐색


def repo_root() -> str:
    return os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))


# ----------------------------------------------------------------------
# 로드

def read_drive(session: str) -> list:
    """프레임 목록. 옛 로그에는 turn/cw_act가 없으므로 전부 get으로 읽는다."""
    out = []
    with io.open(os.path.join(session, 'drive.csv'), newline='',
                 encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            try:
                out.append({
                    'n': int(r['n']), 't': float(r['t']),
                    'x': float(r['ego_x']), 'y': float(r['ego_y']),
                    'v': float(r['speed']),
                    'turn': int(float(r.get('turn') or 0)),
                    'turn_src': (r.get('turn_src') or '').strip(),
                    'cw_act': (r.get('cw_act') or '').strip(),
                })
            except (ValueError, TypeError, KeyError):
                continue
    return out


def guess_inputs(session: str, waypoints: str, events: str,
                 segments: list) -> tuple:
    """meta.json에서 경로·구간·이벤트를 복원한다. (path, wp, ev)."""
    meta = {}
    try:
        with io.open(os.path.join(session, 'meta.json'), encoding='utf-8') as fh:
            meta = json.load(fh)
    except (OSError, ValueError):
        pass

    wp = waypoints or meta.get('waypoints') or ''
    if not os.path.exists(wp):
        wp = os.path.join(session, 'waypoints.csv')
    ev = events or meta.get('events') or ''
    if not os.path.exists(ev):
        cand = os.path.join(session, 'events_in.csv')
        ev = cand if os.path.exists(cand) else ''
    segs = segments or meta.get('segments') or []
    if not os.path.exists(wp) or not segs:
        return None, wp, ev
    from pure_pursuit import load_path
    path, _bounds = load_path(wp, list(segs))
    return path, wp, ev


def project(path, frames: list) -> None:
    """프레임마다 경로 진행거리 s를 붙인다. 제어기와 같은 전진 창 탐색."""
    idx = None
    for f in frames:
        if idx is None:
            idx = path.nearest(f['x'], f['y'])
        else:
            idx = path.nearest(f['x'], f['y'], start=idx)
            resid = math.hypot(path.pts[idx][0] - f['x'],
                               path.pts[idx][1] - f['y'])
            if resid > RESNAP_M:            # 리스폰 등 — 전역 재탐색
                idx = path.nearest(f['x'], f['y'])
        f['s'] = path.s[idx]


def expand(patterns: list) -> list:
    out = []
    for p in patterns:
        hits = sorted(glob.glob(p))
        out.extend(hits or [p])
    return out


# ----------------------------------------------------------------------
# A 방향지시등

def check_turn(frames: list, zones: list) -> list:
    """구간마다 (zone, 선행 점등 시간, 방향 일치 여부)."""
    out = []
    for z in zones:
        hit = next((i for i, f in enumerate(frames) if f['s'] >= z.s_start),
                   None)
        if hit is None:
            out.append((z, float('nan'), False))
            continue
        # 구간 시작 직전의 **연속** 점등만 센다. 중간에 꺼지면 거기서 끊긴다.
        j = hit
        while j > 0 and frames[j - 1]['turn'] == z.signal:
            j -= 1
        lead = frames[hit]['t'] - frames[j]['t'] if frames[hit]['turn'] == z.signal else 0.0
        out.append((z, lead, frames[hit]['turn'] == z.signal))
    return out


def stray_signal(frames: list, zones: list) -> float:
    """어느 구간에도 속하지 않는데 켜져 있던 시간 [s]."""
    total = 0.0
    for i, f in enumerate(frames):
        if f['turn'] == 0:
            continue
        inside = any(z.s_start - 200.0 <= f['s'] <= z.s_end + 5.0
                     and z.signal == f['turn'] for z in zones)
        if not inside:
            dt = f['t'] - frames[i - 1]['t'] if i else 0.0
            total += max(0.0, min(dt, 0.2))
    return total


# ----------------------------------------------------------------------
# B 횡단보도 정차

def check_crosswalk(frames: list, zones: list) -> list:
    """구간마다 (zone, 최장 정차 시간, 그때의 cw_act)."""
    out = []
    for z in zones:
        best, act = 0.0, ''
        start = None
        for f in frames:
            on = (z.s0 <= f['s'] <= z.s1) and f['v'] <= DWELL_MS
            if on:
                if start is None:
                    start = f['t']
                if f['t'] - start > best:
                    best, act = f['t'] - start, f['cw_act']
            else:
                start = None
        out.append((z, best, act))
    return out


# ----------------------------------------------------------------------

def report(session: str, waypoints: str, events: str, segments: list,
           junction: bool) -> dict:
    name = os.path.basename(os.path.normpath(session))
    frames = read_drive(session)
    if not frames:
        print(f'  {name}: drive.csv가 비어 있다')
        return {'lc_fail': 0, 'lc_total': 0, 'xw_viol': 0, 'xw_total': 0}

    path, wp, ev = guess_inputs(session, waypoints, events, segments)
    if path is None:
        print(f'  {name}: 경로를 복원할 수 없다 (waypoints/segments)')
        return {'lc_fail': 0, 'lc_total': 0, 'xw_viol': 0, 'xw_total': 0}
    project(path, frames)

    zones, _w = turnsignal.build_zones(path, wp, junction=junction)
    rows = check_turn(frames, zones)
    lc_fail = lc_total = 0

    print(f'  {name}  {len(frames)}프레임 {frames[-1]["t"]:.1f}초 '
          f'{frames[-1]["s"]:.1f}m')
    print('    A 방향지시등 (항목 13)')
    for z, lead, matched in rows:
        req = (z.kind == 'lc')
        lc_total += 1 if req else 0
        bad = req and not (matched and lead >= LEAD_REQ_S)
        lc_fail += 1 if bad else 0
        mark = ('FAIL' if bad else 'PASS') if req else '   .'
        print(f'      {mark}  s={z.s_start:7.1f}m  '
              f'{turnsignal.KIND_NAME[z.kind]:6s} {z.dir:5s}  '
              f'선행 {lead:5.2f}초'
              + ('' if matched else '  ← 미점등/방향 불일치'))
    stray = stray_signal(frames, zones)
    if stray > 0.5:
        print(f'      (구간 밖 점등 {stray:.1f}초 — 감점 항목은 아니다)')

    xw_zones, _w2 = ([], []) if not ev else crosswalks.load_for_path(path, ev)
    xw_rows = check_crosswalk(frames, xw_zones)
    xw_viol = sum(1 for _z, d, _a in xw_rows if d >= DWELL_REQ_S)
    print('    B 횡단보도 정차 (항목 12)')
    if not xw_zones:
        print('      횡단보도 없음 (이벤트 CSV 미지정)')
    for z, dwell, act in xw_rows:
        mark = 'VIOL' if dwell >= DWELL_REQ_S else 'ok  '
        print(f'      {mark}  s={z.s:7.1f}m  최장 정차 {dwell:5.2f}초'
              + (f'  cw_act={act}' if act else ''))

    return {'lc_fail': lc_fail, 'lc_total': lc_total,
            'xw_viol': xw_viol, 'xw_total': len(xw_zones)}


def main() -> None:
    ap = argparse.ArgumentParser(
        description='방향지시등·횡단보도 정차 계수기 (항목 13·12)')
    ap.add_argument('sessions', nargs='+', help='세션 디렉터리')
    ap.add_argument('--baseline', nargs='*', default=[],
                    help='비교할 베이스라인 세션')
    ap.add_argument('--path', default='',
                    help='waypoint CSV. 생략하면 meta.json에서 찾는다')
    ap.add_argument('--events', default='',
                    help='이벤트 CSV. 생략하면 meta.json / events_in.csv')
    ap.add_argument('--segments', default='',
                    help='쉼표로 구분한 구간 이름. 생략하면 meta.json')
    ap.add_argument('--no-junction-signal', action='store_true',
                    help='교차로 회전 구간을 빼고 본다')
    args = ap.parse_args()
    segs = [s.strip() for s in args.segments.split(',') if s.strip()]

    def run(label, sessions):
        tally = {'lc_fail': 0, 'lc_total': 0, 'xw_viol': 0, 'xw_total': 0}
        if not sessions:
            return tally
        print(f'\n== {label}')
        for s in expand(sessions):
            r = report(s, args.path, args.events, segs,
                       not args.no_junction_signal)
            for k in tally:
                tally[k] += r[k]
        return tally

    base = run('베이스라인', args.baseline)
    cur = run('대상', args.sessions)

    print('\n== 합계')
    print(f'{"":14s}{"차선변경 미점등":>16s}{"횡단보도 3초 정차":>18s}')
    if args.baseline:
        print(f'{"베이스라인":14s}{base["lc_fail"]:>10d}/{base["lc_total"]:<5d}'
              f'{base["xw_viol"]:>12d}/{base["xw_total"]:<5d}')
    print(f'{"대상":14s}{cur["lc_fail"]:>10d}/{cur["lc_total"]:<5d}'
          f'{cur["xw_viol"]:>12d}/{cur["xw_total"]:<5d}')
    raise SystemExit(1 if (cur['lc_fail'] or cur['xw_viol']) else 0)


if __name__ == '__main__':
    main()
