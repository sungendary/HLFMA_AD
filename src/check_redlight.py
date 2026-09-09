#!/usr/bin/env python3
"""적색 신호 통과 계수기 (평가항목 7·9).

세션 로그의 궤적을 정지선 좌표와 대조해 접근별로 판정한다. 베이스라인
(신호등 로직이 없던 로그)과 수정 후 로그를 **같은 잣대로** 잰다.

    python check_redlight.py ../logs/pp_20260904_111906
    python check_redlight.py ../logs/sim_pt2 --baseline ../logs/pp_20260904_1119*

--- 판정 기준을 정지선 5m 전 상태로 잡는 이유 ---

VTD는 정지선에 닿는 순간 신호를 NONE으로 내린다(사전테스트 6세션 실측:
이탈 지점이 후륜축 기준 -0.3 ~ +0.4m). 그래서 **교차로를 지나는 프레임의
tl_state는 이미 0이다.** 통과 시점 상태로 세면 위반이 0으로 나온다 ―
실제로 그렇게 세면 9회가 2회로 줄어 보인다. 접근 상태로 재야 한다.
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

from stoplines import (FRONT_OVERHANG, STATE_NAME, RED, YELLOW, NONE,
                       parse_ctrl)

LAT_MAX = 6.0           # m, 이보다 옆으로 벗어난 프레임은 그 정지선 접근이 아니다
RANGE_M = 200.0         # m, 접근으로 볼 최대 거리
BEHIND_M = 10.0         # m, 정지선을 지난 뒤 이만큼까지 본다
JUDGE_M = 5.0           # m, 판정 기준 거리 (위 docstring)
STOPPED_MS = 0.5        # m/s, 이 아래면 멈춘 것으로 본다
STOPPED_ZONE_M = 30.0   # m, 정지선 앞 이 안의 정차만 '정지선 정지'로 센다

STOP_ACTS = ('stop', 'hold', 'pause')


def repo_root() -> str:
    return os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))


# ----------------------------------------------------------------------

def load_stoplines(events_csv: str) -> list:
    out = []
    with io.open(events_csv, newline='', encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            if r.get('kind') != 'stopline':
                continue
            out.append({
                'x': float(r['x']), 'y': float(r['y']),
                's_along': float(r.get('s_along', 0) or 0),
                'ctrl': parse_ctrl(r.get('detail', '')),
            })
    out.sort(key=lambda d: d['s_along'])
    return out


def guess_events(session: str) -> str:
    """meta.json에서 어느 경로를 달렸는지 보고 이벤트 CSV를 고른다."""
    data = os.path.join(repo_root(), 'data')
    try:
        with io.open(os.path.join(session, 'meta.json'), encoding='utf-8') as fh:
            meta = json.load(fh)
    except OSError:
        return ''
    ev = meta.get('events') or ''
    if ev and os.path.exists(ev):
        return ev
    wp = os.path.basename(meta.get('waypoints', '')).lower()
    if 'pretest1' in wp or 'pt1' in wp:
        return os.path.join(data, 'route_path_pt1_events.csv')
    if 'pretest2' in wp or 'pt2' in wp:
        return os.path.join(data, 'route_path_pt2_events.csv')
    if 'v2' in wp or '_ex' in wp:
        return os.path.join(data, 'route_path_ex_events.csv')
    return ''


def read_drive(session: str) -> list:
    rows = []
    with io.open(os.path.join(session, 'drive.csv'), newline='',
                 encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append({
                    't': float(r['t']),
                    'x': float(r['ego_x']), 'y': float(r['ego_y']),
                    'h': float(r['heading']), 'v': float(r['speed']),
                    'tl_id': int(r['tl_id']), 'tl_state': int(r['tl_state']),
                    'act': (r.get('tl_act') or '').strip(),
                })
            except (ValueError, TypeError, KeyError):
                continue
    return rows


# ----------------------------------------------------------------------

def analyse(rows: list, sl: dict) -> dict | None:
    """정지선 하나에 대한 접근 요약. 접근한 적이 없으면 None."""
    sx, sy, ctrl = sl['x'], sl['y'], sl['ctrl']
    tr = []
    for r in rows:
        dx, dy = sx - r['x'], sy - r['y']
        ch, sh = math.cos(r['h']), math.sin(r['h'])
        lon = dx * ch + dy * sh
        lat = -dx * sh + dy * ch
        if abs(lat) <= LAT_MAX and -BEHIND_M <= lon <= RANGE_M:
            tr.append((lon, r))
    if not tr:
        return None

    ahead = [p for p in tr if p[0] > 0]
    if not ahead:
        return None

    res = {
        's_along': sl['s_along'], 'ctrl': ctrl,
        'acquire': None, 'judge': None, 'judge_v': None,
        'crossed': False, 'cross_state': None, 'bumper_crossed': False,
        'min_bumper_restrictive': None,
        'stopped_bumper': None, 'cmd_stop_from': None,
        'verdict': 'NO_APPROACH',
    }

    # 신호를 언제 처음 받았나 (해당 ctrl로 한정)
    for lon, r in tr:
        if r['tl_state'] != NONE and (not ctrl or r['tl_id'] in ctrl):
            if res['acquire'] is None or lon > res['acquire']:
                res['acquire'] = lon

    # 판정 시점: **범퍼가 정지선에 닿는 순간의 상태**.
    #
    # VTD는 후륜축이 정지선에 닿을 때 신호를 NONE으로 내린다. 범퍼는 그보다
    # FRONT_OVERHANG만큼 앞서 닿으므로 그 순간에는 신호가 아직 보인다.
    # 고정 거리(예: 5m 전)로 재면 안 된다 — 정지선 앞에 서서 적색을 기다리는
    # 동안의 프레임이 섞여 "정지하고 녹색에 출발한" 접근까지 위반으로 찍힌다.
    last_before = None
    for lon, r in tr:
        if lon >= FRONT_OVERHANG:
            last_before = r
        elif last_before is not None and not res['bumper_crossed']:
            res['bumper_crossed'] = True
            res['judge'] = last_before['tl_state']
            res['judge_v'] = last_before['v'] * 3.6
    if res['judge'] is None:            # 정지선까지 못 갔다
        lon_c, r_c = min(ahead, key=lambda p: p[0])
        res['judge'] = r_c['tl_state']
        res['judge_v'] = r_c['v'] * 3.6

    # 후륜축 통과 여부 (표시용)
    prev = None
    for lon, r in tr:
        if prev is not None and prev > 0 >= lon and not res['crossed']:
            res['crossed'] = True
            res['cross_state'] = r['tl_state']
        prev = lon

    # 제한 신호가 걸린 동안의 최소 범퍼 거리 (음수 = 정지선 침범)
    for lon, r in tr:
        if r['tl_state'] in (RED, YELLOW) and (not ctrl or r['tl_id'] in ctrl):
            b = lon - FRONT_OVERHANG
            if res['min_bumper_restrictive'] is None or b < res['min_bumper_restrictive']:
                res['min_bumper_restrictive'] = b

    # 정지선 앞에서 실제로 멈췄나. 출발 지점의 정지가 섞이지 않게
    # 정지선 STOPPED_ZONE_M 안에서만 센다.
    for lon, r in tr:
        if r['v'] < STOPPED_MS and 0 < lon <= STOPPED_ZONE_M:
            b = lon - FRONT_OVERHANG
            if res['stopped_bumper'] is None or b < res['stopped_bumper']:
                res['stopped_bumper'] = b

    # 제어기가 정지를 지령했나 (신규 tl_act 컬럼. 옛 로그에는 없다)
    for lon, r in tr:
        if r['act'] in STOP_ACTS:
            if res['cmd_stop_from'] is None or lon > res['cmd_stop_from']:
                res['cmd_stop_from'] = lon

    # 판정
    if res['bumper_crossed'] and res['judge'] == RED:
        res['verdict'] = 'RED_CROSS'
    elif res['bumper_crossed'] and res['judge'] == YELLOW:
        res['verdict'] = 'YELLOW_CLEAR'
    elif res['stopped_bumper'] is not None and res['stopped_bumper'] < 0:
        # 정지는 했는데 범퍼가 정지선을 넘은 채로 섰다 (항목 9)
        res['verdict'] = 'LINE_OVERRUN'
    elif res['stopped_bumper'] is not None:
        res['verdict'] = 'STOPPED'
    else:
        res['verdict'] = 'OK'
    return res


def run_session(session: str, events_csv: str) -> list:
    rows = read_drive(session)
    if not rows:
        return []
    out = []
    for sl in load_stoplines(events_csv):
        r = analyse(rows, sl)
        if r is not None:
            r['session'] = os.path.basename(os.path.normpath(session))
            out.append(r)
    return out


# ----------------------------------------------------------------------

def fmt(x, w=6, p=1, dash='-'):
    if x is None:
        return dash.rjust(w)
    return f'{x:{w}.{p}f}'


def print_table(results: list, title: str) -> dict:
    print()
    print('=' * 108)
    print(title)
    print('=' * 108)
    print(f'{"세션":<20} {"s_along":>8} {"ctrl":>9} {"획득":>7} '
          f'{"닿을때":>8} {"km/h":>6} {"통과":>5} {"최소범퍼":>9} '
          f'{"정지범퍼":>9} {"정지지령":>9}  판정')
    print('-' * 108)
    tally = {}
    for r in results:
        tally[r['verdict']] = tally.get(r['verdict'], 0) + 1
        ctrl = '|'.join(map(str, sorted(r['ctrl']))) if r['ctrl'] else '-'
        mark = ' <<<' if r['verdict'] in ('RED_CROSS', 'LINE_OVERRUN') else ''
        print(f'{r["session"]:<20} {r["s_along"]:8.1f} {ctrl:>9} '
              f'{fmt(r["acquire"], 7)} {STATE_NAME.get(r["judge"], "?"):>8} '
              f'{fmt(r["judge_v"], 6)} {"Y" if r["bumper_crossed"] else "N":>5} '
              f'{fmt(r["min_bumper_restrictive"], 9)} '
              f'{fmt(r["stopped_bumper"], 9)} {fmt(r["cmd_stop_from"], 9)}  '
              f'{r["verdict"]}{mark}')
    print('-' * 108)
    order = ['RED_CROSS', 'LINE_OVERRUN', 'YELLOW_CLEAR', 'STOPPED', 'OK']
    parts = [f'{k} {tally[k]}' for k in order if k in tally]
    parts += [f'{k} {v}' for k, v in tally.items() if k not in order]
    print(f'접근 {len(results)}건 ― ' + '  '.join(parts))
    return tally


def expand(patterns: list) -> list:
    out = []
    for p in patterns:
        hits = sorted(glob.glob(p))
        out.extend(h for h in (hits or [p]) if os.path.isdir(h))
    return out


def collect(sessions: list, events: str) -> list:
    res = []
    for s in sessions:
        ev = events or guess_events(s)
        if not ev or not os.path.exists(ev):
            print(f'[skip] {s}: 이벤트 CSV를 못 찾았다 (--events로 지정할 것)')
            continue
        if not os.path.exists(os.path.join(s, 'drive.csv')):
            print(f'[skip] {s}: drive.csv 없음')
            continue
        res.extend(run_session(s, ev))
    return res


def main() -> None:
    ap = argparse.ArgumentParser(description='적색 신호 통과 계수 (항목 7·9)')
    ap.add_argument('sessions', nargs='+', help='세션 디렉터리 (glob 가능)')
    ap.add_argument('--baseline', nargs='+', default=[],
                    help='비교 대상 베이스라인 세션')
    ap.add_argument('--events', default='',
                    help='정지선 이벤트 CSV. 생략하면 meta.json으로 추정')
    args = ap.parse_args()

    after = collect(expand(args.sessions), args.events)
    if not after:
        raise SystemExit('분석할 접근이 없다')

    if args.baseline:
        before = collect(expand(args.baseline), args.events)
        b = print_table(before, '베이스라인')
        a = print_table(after, '수정 후')
        print()
        print('=' * 108)
        print('비교')
        print('=' * 108)
        keys = ['RED_CROSS', 'LINE_OVERRUN', 'YELLOW_CLEAR', 'STOPPED', 'OK']
        for k in keys:
            bv, av = b.get(k, 0), a.get(k, 0)
            if bv or av:
                arrow = '->' if bv != av else '=='
                print(f'  {k:<14} {bv:3d} {arrow} {av:3d}')
        print(f'\n  적색 통과 {b.get("RED_CROSS", 0)}회 -> {a.get("RED_CROSS", 0)}회')
    else:
        print_table(after, '판정')


if __name__ == '__main__':
    main()
