#!/usr/bin/env python3
"""충돌·최소 접근거리 계수기 (평가항목 10·11·14).

세션 로그의 drive.csv와 objects.csv를 프레임 번호 n으로 조인해, 객체별로
"가장 가까이 붙었을 때 얼마나 남았나"를 잰다. 베이스라인(--no-obstacle로
받은 로그)과 수정 후를 **같은 잣대로** 잰다. check_redlight.py의 쌍둥이다.

    python check_collision.py ../logs/obs_pt1_t
    python check_collision.py ../logs/obs_pt2_t --baseline ../logs/obs_pt2_t_base

--- sim에는 충돌 모델이 없다 ---

sim_world.EgoSim은 순수 자전거 적분기라 객체를 들이받아도 아무 일이 없다.
그래서 충돌은 사후에 **풋프린트 간격**으로 잰다. 두 사각형을 ego 차체
좌표계의 두 축(종/횡)에 투영해 분리축으로 판정한다. 둘 다 겹쳐야 충돌이다.

--- ego 치수는 일부가 가정이다 ---

FRONT_OVERHANG 3.808m만 실측이고(후륜축 -> 앞범퍼), 뒤 오버행과 차폭은
승용차 4.8 x 1.9m 추정에서 나온 값이다. 절대값보다 **베이스라인과의 차이**를
보는 것이 이 리포트의 목적이다.
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

import obstacles
from stoplines import FRONT_OVERHANG

REAR_OVERHANG = 1.0         # m, **가정.** 전장 4.8m 추정에서 3.808을 뺀 값
EGO_HALF_W = 0.95           # m, **가정.** 차폭 1.9m 추정

COLLIDE_M = 0.0             # 이 아래면 겹쳤다
NEAR_M = 1.0                # 이 아래면 아슬아슬했다
RANGE_M = 80.0              # 9910 수신 반경
STOP_SPEED = 0.3            # m/s, 이 아래면 멈춘 것으로 본다
LATE_RESTART_S = 5.0        # s, 항목 8 감점 한계
STOPLINE_ZONE_M = 30.0      # m, 항목 8은 정지선 30m 이내에서만 걸린다

STOP_KINDS = ('ped_stop_actual', 'ped_stop_predicted', 'veh_stop')
GO_KINDS = ('ped_clear', 'ped_timeout', 'veh_timeout')


def repo_root() -> str:
    return os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))


# ----------------------------------------------------------------------
# 로드

def read_drive(session: str) -> dict:
    """n -> 프레임. 옛 로그에는 ob_* 컬럼이 없으므로 전부 get으로 읽는다."""
    out = {}
    with io.open(os.path.join(session, 'drive.csv'), newline='',
                 encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            try:
                out[int(r['n'])] = {
                    't': float(r['t']),
                    'x': float(r['ego_x']), 'y': float(r['ego_y']),
                    'h': float(r['heading']), 'v': float(r['speed']),
                    'tl_dist': float(r.get('tl_dist') or 'nan'),
                    'ob_act': (r.get('ob_act') or '').strip(),
                    'ob_id': int(float(r.get('ob_id') or -1)),
                }
            except (ValueError, TypeError, KeyError):
                continue
    return out


def read_events(session: str) -> list:
    out = []
    p = os.path.join(session, 'events.csv')
    if not os.path.exists(p):
        return out
    with io.open(p, newline='', encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            try:
                out.append({'t': float(r['t']), 'kind': r['kind'],
                            'detail': r.get('detail', '')})
            except (ValueError, TypeError, KeyError):
                continue
    return out


def guess_path(session: str):
    """meta.json의 waypoints/segments로 경로를 다시 만든다. 실패하면 None."""
    try:
        with io.open(os.path.join(session, 'meta.json'), encoding='utf-8') as fh:
            meta = json.load(fh)
    except OSError:
        return None
    wp = meta.get('waypoints') or ''
    if not os.path.exists(wp):
        wp = os.path.join(session, 'waypoints.csv')
    segs = meta.get('segments') or []
    if not os.path.exists(wp) or not segs:
        return None
    try:
        from pure_pursuit import load_path
        import finish
        path, _bounds = load_path(wp, list(segs))
        finish.apply_meta_tail(path, meta)      # 런타임 최종 꼬리 (라바콘·끝 너머 정지선)
        return path
    except Exception:
        return None


# ----------------------------------------------------------------------
# 기하

def gap_to(ego: dict, o: dict) -> tuple:
    """(풋프린트 간격, 종방향 lon, 횡방향 lat). 음수 간격이면 겹쳤다.

    객체 사각형을 ego 축에 투영한 반폭·반길이를 쓴다. 회전 사각형의
    분리축 판정이고, ego와 축이 나란할 때는 정확하다.
    """
    dx, dy = o['x'] - ego['x'], o['y'] - ego['y']
    ch, sh = math.cos(ego['h']), math.sin(ego['h'])
    lon = dx * ch + dy * sh
    lat = -dx * sh + dy * ch

    dh = o['heading'] - ego['h']
    c, s = abs(math.cos(dh)), abs(math.sin(dh))
    ext_lon = o['length'] * 0.5 * c + o['width'] * 0.5 * s
    ext_lat = o['length'] * 0.5 * s + o['width'] * 0.5 * c

    if lon >= 0.0:
        gap_lon = lon - ext_lon - FRONT_OVERHANG
    else:
        gap_lon = -lon - ext_lon - REAR_OVERHANG
    gap_lat = abs(lat) - ext_lat - EGO_HALF_W
    return max(gap_lon, gap_lat), lon, lat


# ----------------------------------------------------------------------

def analyse(session: str) -> list:
    """객체 id별 최근접 요약."""
    drive = read_drive(session)
    if not drive:
        return []
    path = guess_path(session)
    name = os.path.basename(os.path.normpath(session))

    per: dict = {}
    with io.open(os.path.join(session, 'objects.csv'), newline='',
                 encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            try:
                n = int(r['n'])
                ego = drive.get(n)
                if ego is None:
                    continue
                o = {'x': float(r['x']), 'y': float(r['y']),
                     'heading': float(r['heading']), 'speed': float(r['speed']),
                     'length': float(r['length']), 'width': float(r['width']),
                     'height': float(r['height'])}
                oid = int(float(r['id']))
            except (ValueError, TypeError, KeyError):
                continue

            gap, lon, lat = gap_to(ego, o)
            if math.hypot(o['x'] - ego['x'], o['y'] - ego['y']) > RANGE_M:
                continue

            e = per.get(oid)
            if e is None:
                e = per[oid] = {
                    'session': name, 'id': oid,
                    'cls': obstacles.classify(_Dim(o)),
                    'max_speed': 0.0, 'gap': float('inf'), 't': None,
                    'v': None, 'lon': None, 'lat': None,
                    'path_lat': None, 'acted': '', 'frames': 0,
                }
            e['frames'] += 1
            e['max_speed'] = max(e['max_speed'], o['speed'])
            if ego['ob_id'] == oid and ego['ob_act'] not in ('', 'none', 'error'):
                # 그 객체 때문에 실제로 지령을 낸 적이 있나
                if e['acted'] != 'creep':
                    e['acted'] = ego['ob_act']
            if gap < e['gap']:
                # 분류는 **최근접 프레임 기준**이다. id가 재사용되면 같은 id에
                # 다른 종류가 섞이는데, 판정에 쓰인 그 순간의 종류가 맞다
                e['cls'] = obstacles.classify(_Dim(o))
                e['gap'], e['t'] = gap, ego['t']
                e['v'], e['lon'], e['lat'] = ego['v'] * 3.6, lon, lat
                if path is not None:
                    i_lo, i_hi = 0, len(path.pts) - 1
                    _s, plat, _th = obstacles._project(path, i_lo, i_hi,
                                                      o['x'], o['y'])
                    e['path_lat'] = plat

    out = []
    for e in per.values():
        if e['gap'] <= COLLIDE_M:
            e['verdict'] = 'COLLISION'
        elif e['gap'] < NEAR_M:
            e['verdict'] = 'NEAR'
        elif e['acted']:
            e['verdict'] = 'REACTED'
        else:
            e['verdict'] = 'OK'
        out.append(e)
    out.sort(key=lambda d: d['gap'])
    return out


class _Dim:
    """obstacles.classify가 보는 속성만 갖는 어댑터."""
    __slots__ = ('height', 'width', 'length')

    def __init__(self, o):
        self.height, self.width = o['height'], o['width']
        self.length = o['length']


# ----------------------------------------------------------------------
# 재출발 지연 — 항목 8 위험 (녹색 정지선 30m 이내 5초 이상 정차)

def restarts(session: str) -> list:
    """정지 지령부터 실제로 다시 움직일 때까지의 초."""
    drive = read_drive(session)
    if not drive:
        return []
    ev = read_events(session)
    frames = sorted(drive.values(), key=lambda r: r['t'])
    out = []
    for e in ev:
        if e['kind'] not in STOP_KINDS:
            continue
        t0 = e['t']
        held = None        # 실제로 멈춘 시각
        for r in frames:
            if r['t'] < t0:
                continue
            if held is None:
                if r['v'] < STOP_SPEED:
                    held = r['t']
                elif r['t'] - t0 > 30.0:
                    break              # 아예 멈추지 않았다
                continue
            if r['v'] >= STOP_SPEED:
                d = r['tl_dist']
                out.append({
                    'session': os.path.basename(os.path.normpath(session)),
                    'kind': e['kind'], 't': held, 'held_s': r['t'] - held,
                    'tl_dist': d,
                    'risk': (r['t'] - held > LATE_RESTART_S
                             and d == d and 0.0 < d <= STOPLINE_ZONE_M),
                })
                break
        else:
            if held is not None:
                out.append({
                    'session': os.path.basename(os.path.normpath(session)),
                    'kind': e['kind'], 't': held,
                    'held_s': frames[-1]['t'] - held,
                    'tl_dist': frames[-1]['tl_dist'], 'risk': True,
                })
    return out


# ----------------------------------------------------------------------

def fmt(x, w=6, p=1, dash='-'):
    if x is None or (isinstance(x, float) and x != x):
        return dash.rjust(w)
    return f'{x:{w}.{p}f}'


def print_table(results: list, title: str) -> dict:
    print()
    print('=' * 104)
    print(title)
    print('=' * 104)
    print(f'{"세션":<16} {"id":>4} {"분류":<11} {"최대속도":>8} {"프레임":>6} '
          f'{"최소간격":>9} {"그때t":>7} {"km/h":>6} {"종":>7} {"횡":>7} '
          f'{"경로횡":>7} {"지령":>6}  판정')
    print('-' * 104)
    tally: dict = {}
    for r in results:
        tally[r['verdict']] = tally.get(r['verdict'], 0) + 1
        mark = ' <<<' if r['verdict'] in ('COLLISION', 'NEAR') else ''
        print(f'{r["session"]:<16} {r["id"]:>4} {r["cls"]:<11} '
              f'{r["max_speed"] * 3.6:8.1f} {r["frames"]:>6} '
              f'{fmt(r["gap"], 9, 2)} {fmt(r["t"], 7)} {fmt(r["v"], 6)} '
              f'{fmt(r["lon"], 7)} {fmt(r["lat"], 7)} {fmt(r["path_lat"], 7, 2)} '
              f'{r["acted"] or "-":>6}  {r["verdict"]}{mark}')
    print('-' * 104)
    order = ['COLLISION', 'NEAR', 'REACTED', 'OK']
    parts = [f'{k} {tally[k]}' for k in order if k in tally]
    print(f'객체 {len(results)}개 ― ' + '  '.join(parts))
    return tally


def print_restarts(rows: list) -> None:
    if not rows:
        return
    print()
    print('=' * 104)
    print('정지 후 재출발 지연 (항목 8: 녹색 정지선 30m 이내 5초 이상 정차는 감점)')
    print('=' * 104)
    print(f'{"세션":<16} {"계기":<20} {"정지 t":>8} {"정차":>7} '
          f'{"정지선까지":>10}  비고')
    print('-' * 104)
    for r in rows:
        note = '*** 항목 8 위험 ***' if r['risk'] else ''
        print(f'{r["session"]:<16} {r["kind"]:<20} {r["t"]:8.1f} '
              f'{r["held_s"]:7.1f} {fmt(r["tl_dist"], 10)}  {note}')
    worst = max(r['held_s'] for r in rows)
    n_risk = sum(1 for r in rows if r['risk'])
    print('-' * 104)
    print(f'정차 {len(rows)}회 ― 최장 {worst:.1f}초, 항목 8 위험 {n_risk}회')


def expand(patterns: list) -> list:
    out = []
    for p in patterns:
        hits = sorted(glob.glob(p))
        out.extend(h for h in (hits or [p]) if os.path.isdir(h))
    return out


def collect(sessions: list) -> tuple:
    res, rst = [], []
    for s in sessions:
        if not os.path.exists(os.path.join(s, 'objects.csv')):
            print(f'[skip] {s}: objects.csv 없음')
            continue
        res.extend(analyse(s))
        rst.extend(restarts(s))
    return res, rst


def main() -> None:
    ap = argparse.ArgumentParser(
        description='충돌·최소 접근거리 계수 (항목 10·11·14)')
    ap.add_argument('sessions', nargs='+', help='세션 디렉터리 (glob 가능)')
    ap.add_argument('--baseline', nargs='+', default=[],
                    help='비교 대상 베이스라인 세션 (--no-obstacle로 받은 것)')
    args = ap.parse_args()

    print(f'ego 풋프린트: 앞 {FRONT_OVERHANG:.3f}m (실측) / '
          f'뒤 {REAR_OVERHANG:.1f}m, 반폭 {EGO_HALF_W:.2f}m (둘 다 가정)')

    after, rst = collect(expand(args.sessions))
    if not after:
        raise SystemExit('분석할 객체가 없다')

    if args.baseline:
        before, brst = collect(expand(args.baseline))
        b = print_table(before, '베이스라인 (--no-obstacle)')
        a = print_table(after, '수정 후')
        print_restarts(rst)
        print()
        print('=' * 104)
        print('비교')
        print('=' * 104)
        for k in ['COLLISION', 'NEAR', 'REACTED', 'OK']:
            bv, av = b.get(k, 0), a.get(k, 0)
            if bv or av:
                print(f'  {k:<10} {bv:3d} {"->" if bv != av else "=="} {av:3d}')
        bg = min((r['gap'] for r in before), default=float('nan'))
        ag = min((r['gap'] for r in after), default=float('nan'))
        print(f'\n  충돌 {b.get("COLLISION", 0)}회 -> {a.get("COLLISION", 0)}회')
        print(f'  최소 접근거리 {bg:.2f}m -> {ag:.2f}m')
    else:
        print_table(after, '판정')
        print_restarts(rst)


if __name__ == '__main__':
    main()
