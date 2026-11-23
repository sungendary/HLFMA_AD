#!/usr/bin/env python3
"""TTC 충돌 예측·우회 지연 계수기 (평가항목 14 강화 / 완주 보장).

세션 로그의 drive.csv와 events.csv로 세 가지를 잰다. check_collision.py의 쌍둥이다.

  A  TTC 발화 ― ttc_warn / ttc_brake / ttc_stop / ttc_timeout 횟수, 제약 프레임 수,
     발화별 (t, id, 분류, t_cpa, d_cpa, 속도). **빈 도로에서는 brake/stop이 0이어야 한다**
  B  우회 지연 ― detour_start마다 그 객체를 처음 본 시각(detour_context)부터
     발동까지의 초, 발동 시 ego 속도(달리는 중 발동인가), 판정 티어
  C  신호 대기 중 우회 ― detour_start 프레임의 tl_act가 stop/hold이고 신호가
     RED/YELLOW면 대기열 오판이다. **0이어야 한다**
  D  교차로 진입 양보 ― junction_yield를 비신호/신호로 나눠 세고, 신호 건은 그때
     차량이 ego 통과 회랑과 겹쳤는지(항목 8 위험), junction_commit / junction_timeout
     횟수, 양보 대기 시간 합. **빈 도로에서는 yield·commit이 0이어야 한다**

    python check_ttc.py ../logs/tt_pt2_none --expect-quiet     # A brake/stop·C가 0이 아니면 exit 1
    python check_ttc.py ../logs/tt_pt2_many6 --baseline ../logs/det_many
"""

from __future__ import annotations

import argparse
import csv
import glob
import io
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

TTC_KINDS = ('ttc_warn', 'ttc_brake', 'ttc_stop', 'ttc_timeout', 'ttc_clear',
             'ttc_hold', 'ttc_hold_timeout', 'ttc_hold_clear')
HARD_KINDS = ('ttc_brake', 'ttc_stop', 'ttc_timeout', 'ttc_hold',
              'ttc_hold_timeout')
JY_KINDS = ('junction_yield', 'junction_go', 'junction_timeout',
            'junction_commit', 'junction_approach')
_WAIT = re.compile(r'양보 ([\d.]+)s 뒤 출발')
DET_KINDS = ('detour_start', 'detour_done', 'detour_failed', 'detour_impossible',
             'last_resort_pass')
RED, YELLOW = 1, 2
_ID = re.compile(r'id=(\d+)')
_V = re.compile(r'v=([\d.]+)km/h')
_TIER = re.compile(r'\((fast|ambig|queue|freeze):')


def read_drive(session: str) -> list:
    out = []
    with io.open(os.path.join(session, 'drive.csv'), newline='',
                 encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            try:
                out.append({
                    'n': int(r['n']), 't': float(r['t']),
                    'v': float(r['speed']),
                    'tl_act': (r.get('tl_act') or '').strip(),
                    'tl_state': int(float(r.get('tl_state') or 0)),
                    'tl_dist': float(r.get('tl_dist') or 'nan'),
                    'dt_act': (r.get('dt_act') or '').strip(),
                    'tt_act': (r.get('tt_act') or '').strip(),
                    'tt_tcpa': float(r.get('tt_tcpa') or 'nan'),
                    'tt_dcpa': float(r.get('tt_dcpa') or 'nan'),
                })
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
                out.append({'t': float(r['t']), 'n': int(float(r.get('n') or 0)),
                            'kind': r['kind'], 'detail': r.get('detail', ''),
                            'speed': float(r.get('speed') or 'nan')})
            except (ValueError, TypeError, KeyError):
                continue
    return out


# ----------------------------------------------------------------------

def analyse_ttc(session: str) -> dict:
    name = os.path.basename(os.path.normpath(session))
    drive = read_drive(session)
    ev = read_events(session)
    counts = {k: sum(1 for e in ev if e['kind'] == k) for k in TTC_KINDS}
    limited = [r for r in drive if r['tt_act'] not in ('', 'none', 'error')]
    hard = [r for r in limited
            if r['tt_act'] in ('brake', 'stop', 'timeout', 'hold')]
    min_tcpa = min((r['tt_tcpa'] for r in hard if r['tt_tcpa'] == r['tt_tcpa']),
                   default=float('nan'))
    fires = []
    for e in ev:
        if e['kind'] not in HARD_KINDS:
            continue
        m = _ID.search(e['detail'])
        fires.append({'t': e['t'], 'kind': e['kind'],
                      'id': int(m.group(1)) if m else -1,
                      'speed': e['speed'], 'detail': e['detail']})
    return {'session': name, 'frames': len(drive), 'counts': counts,
            'limited': len(limited), 'hard_frames': len(hard),
            'min_tcpa': min_tcpa, 'fires': fires,
            'has_cols': any(r['tt_act'] for r in drive)}


def analyse_junction(session: str) -> dict:
    """junction_* 이벤트 계수. 신호 교차로 양보는 회랑 겹침 여부까지."""
    name = os.path.basename(os.path.normpath(session))
    ev = read_events(session)
    out = {'session': name, 'yield_unsig': 0, 'yield_sig': 0, 'sig_no_overlap': 0,
           'commit': 0, 'timeout': 0, 'approach': 0, 'wait_s': 0.0, 'rows': []}
    for e in ev:
        k, d = e['kind'], e['detail']
        if k not in JY_KINDS:
            continue
        if k == 'junction_yield':
            sig = '비신호' not in d
            overlap = '회랑=Y' in d
            if sig:
                out['yield_sig'] += 1
                if not overlap:
                    out['sig_no_overlap'] += 1
            else:
                out['yield_unsig'] += 1
            out['rows'].append((e['t'], k, ('신호' if sig else '비신호')
                                + (' 회랑 겹침' if overlap else ' 회랑 밖'), d))
        elif k == 'junction_commit':
            out['commit'] += 1
            out['rows'].append((e['t'], k, '', d))
        elif k == 'junction_timeout':
            out['timeout'] += 1
            out['rows'].append((e['t'], k, '', d))
        elif k == 'junction_approach':
            out['approach'] += 1
        elif k == 'junction_go':
            m = _WAIT.search(d)
            if m:
                out['wait_s'] += float(m.group(1))
            out['rows'].append((e['t'], k, '', d))
    return out


def print_junction(rows: list, title: str) -> dict:
    print()
    print('=' * 104)
    print(title)
    print('=' * 104)
    print(f'{"세션":<18} {"비신호":>6} {"신호":>5} {"회랑밖":>6} {"commit":>6} '
          f'{"timeout":>7} {"접근":>5} {"대기s":>7}')
    print('-' * 104)
    tot = {'yield': 0, 'commit': 0, 'sig_no_overlap': 0, 'timeout': 0}
    for r in rows:
        print(f'{r["session"]:<18} {r["yield_unsig"]:>6} {r["yield_sig"]:>5} '
              f'{r["sig_no_overlap"]:>6} {r["commit"]:>6} {r["timeout"]:>7} '
              f'{r["approach"]:>5} {r["wait_s"]:>7.1f}')
        for t, k, tag, d in r['rows']:
            print(f'    {t:7.1f}s {k:<17} {tag:<14} {d[:64]}')
        tot['yield'] += r['yield_unsig'] + r['yield_sig']
        tot['commit'] += r['commit']
        tot['sig_no_overlap'] += r['sig_no_overlap']
        tot['timeout'] += r['timeout']
    print('-' * 104)
    print(f'교차로 양보 {tot["yield"]}회 (신호 교차로에서 회랑 밖 차량에 선 것 '
          f'{tot["sig_no_overlap"]}회), commit {tot["commit"]}회, '
          f'timeout {tot["timeout"]}회')
    return tot


def analyse_detour(session: str) -> list:
    """detour_start마다 지연·속도·티어·신호 대기 여부."""
    name = os.path.basename(os.path.normpath(session))
    drive = read_drive(session)
    by_n = {r['n']: r for r in drive}
    ev = read_events(session)
    first_seen: dict = {}       # id -> 처음 detour_context를 낸 시각
    out = []
    for e in ev:
        m = _ID.search(e['detail'])
        oid = int(m.group(1)) if m else -1
        if e['kind'] == 'detour_context' and oid >= 0 and oid not in first_seen:
            first_seen[oid] = e['t']
        if e['kind'] not in ('detour_start', 'detour_impossible'):
            continue
        fr = by_n.get(e['n'])
        # 프레임 번호가 없으면(옛 로그) 시각으로 가장 가까운 프레임
        if fr is None and drive:
            fr = min(drive, key=lambda r: abs(r['t'] - e['t']))
        mv = _V.search(e['detail'])
        v = float(mv.group(1)) if mv else (fr['v'] * 3.6 if fr else float('nan'))
        mt = _TIER.search(e['detail'])
        sig_wait = bool(fr and fr['tl_act'] in ('stop', 'hold')
                        and fr['tl_state'] in (RED, YELLOW))
        out.append({
            'session': name, 'kind': e['kind'], 't': e['t'], 'id': oid,
            'latency': (e['t'] - first_seen[oid]) if oid in first_seen else float('nan'),
            'v_kmh': v, 'tier': mt.group(1) if mt else '-',
            'sig_wait': sig_wait,
            'tl': f'{fr["tl_act"]}/{fr["tl_state"]}' if fr else '-',
        })
        first_seen.pop(oid, None)       # 다음 발동은 다시 잰다
    return out


# ----------------------------------------------------------------------

def fmt(x, w=6, p=1, dash='-'):
    if x is None or (isinstance(x, float) and x != x):
        return dash.rjust(w)
    return f'{x:{w}.{p}f}'


def print_ttc(rows: list, title: str) -> dict:
    print()
    print('=' * 104)
    print(title)
    print('=' * 104)
    print(f'{"세션":<18} {"프레임":>6} {"warn":>5} {"brake":>5} {"stop":>5} '
          f'{"tmout":>5} {"hold":>5} {"clear":>5} {"제약f":>6} {"강제f":>6} '
          f'{"min t_cpa":>9}  비고')
    print('-' * 104)
    tot = {k: 0 for k in TTC_KINDS}
    for r in rows:
        c = r['counts']
        for k in TTC_KINDS:
            tot[k] += c[k]
        note = '' if r['has_cols'] else '(tt_* 컬럼 없음 ― 옛 로그)'
        print(f'{r["session"]:<18} {r["frames"]:>6} {c["ttc_warn"]:>5} '
              f'{c["ttc_brake"]:>5} {c["ttc_stop"]:>5} {c["ttc_timeout"]:>5} '
              f'{c["ttc_hold"]:>5} '
              f'{c["ttc_clear"]:>5} {r["limited"]:>6} {r["hard_frames"]:>6} '
              f'{fmt(r["min_tcpa"], 9, 2)}  {note}')
        for f in r['fires']:
            print(f'    {f["t"]:7.1f}s {f["kind"]:<11} id={f["id"]:<4} '
                  f'v={fmt(f["speed"] * 3.6 if f["speed"] == f["speed"] else None, 5)}km/h  '
                  f'{f["detail"][:70]}')
    print('-' * 104)
    hard = (tot['ttc_brake'] + tot['ttc_stop'] + tot['ttc_timeout']
            + tot['ttc_hold'])
    print(f'TTC 발화 ― warn {tot["ttc_warn"]}  brake {tot["ttc_brake"]}  '
          f'stop {tot["ttc_stop"]}  timeout {tot["ttc_timeout"]}  '
          f'hold {tot["ttc_hold"]}  (강제 합계 {hard})')
    return tot


def print_detour(rows: list, title: str) -> dict:
    print()
    print('=' * 104)
    print(title)
    print('=' * 104)
    if not rows:
        print('  우회 발동 없음')
        return {'n': 0, 'sig_wait': 0, 'moving': 0, 'lat_max': float('nan')}
    print(f'{"세션":<18} {"종류":<18} {"t":>7} {"id":>4} {"지연s":>6} '
          f'{"속도":>6} {"티어":<7} {"신호":<10}  비고')
    print('-' * 104)
    n_wait = n_move = 0
    lats = []
    for r in rows:
        note = ''
        if r['sig_wait']:
            n_wait += 1
            note = '*** 신호 대기 중 우회 ***'
        if r['v_kmh'] == r['v_kmh'] and r['v_kmh'] > 5.0:
            n_move += 1
            note = (note + ' ' if note else '') + '달리는 중 발동'
        if r['latency'] == r['latency']:
            lats.append(r['latency'])
        print(f'{r["session"]:<18} {r["kind"]:<18} {r["t"]:7.1f} {r["id"]:>4} '
              f'{fmt(r["latency"], 6)} {fmt(r["v_kmh"], 6)} {r["tier"]:<7} '
              f'{r["tl"]:<10}  {note}')
    print('-' * 104)
    lat_max = max(lats) if lats else float('nan')
    lat_med = sorted(lats)[len(lats) // 2] if lats else float('nan')
    print(f'우회 발동 {len(rows)}회 ― 지연 중앙 {fmt(lat_med, 4)}s / 최대 {fmt(lat_max, 4)}s, '
          f'달리는 중 {n_move}회, 신호 대기 중 {n_wait}회')
    return {'n': len(rows), 'sig_wait': n_wait, 'moving': n_move, 'lat_max': lat_max}


def expand(patterns: list) -> list:
    out = []
    for p in patterns:
        hits = sorted(glob.glob(p))
        out.extend(h for h in (hits or [p]) if os.path.isdir(h))
    return out


def collect(sessions: list) -> tuple:
    a, b, c = [], [], []
    for s in sessions:
        if not os.path.exists(os.path.join(s, 'drive.csv')):
            print(f'[skip] {s}: drive.csv 없음')
            continue
        a.append(analyse_ttc(s))
        b.extend(analyse_detour(s))
        c.append(analyse_junction(s))
    return a, b, c


def main() -> None:
    ap = argparse.ArgumentParser(description='TTC 발화·우회 지연 계수 (항목 14 강화)')
    ap.add_argument('sessions', nargs='+', help='세션 디렉터리 (glob 가능)')
    ap.add_argument('--baseline', nargs='*', default=[],
                    help='비교 대상 베이스라인 세션 (--no-ttc / --detour-confirm 8)')
    ap.add_argument('--expect-quiet', action='store_true',
                    help='빈 도로 회귀: brake/stop/timeout 또는 신호 대기 중 우회가 '
                         '있으면 exit 1')
    args = ap.parse_args()

    after_t, after_d, after_j = collect(expand(args.sessions))
    if not after_t:
        raise SystemExit('분석할 세션이 없다')

    if args.baseline:
        bt, bd, bj = collect(expand(args.baseline))
        print_ttc(bt, '베이스라인 ― A. TTC 발화')
        print_detour(bd, '베이스라인 ― B/C. 우회 지연')
        print_junction(bj, '베이스라인 ― D. 교차로 진입 양보')
    tot = print_ttc(after_t, '수정 후 ― A. TTC 발화')
    det = print_detour(after_d, '수정 후 ― B/C. 우회 지연 · 신호 대기 중 우회')
    jun = print_junction(after_j, '수정 후 ― D. 교차로 진입 양보')

    hard = (tot['ttc_brake'] + tot['ttc_stop'] + tot['ttc_timeout']
            + tot['ttc_hold'])
    if args.expect_quiet:
        bad = []
        if hard:
            bad.append(f'TTC 강제 발화 {hard}회')
        if det['sig_wait']:
            bad.append(f'신호 대기 중 우회 {det["sig_wait"]}회')
        if jun['yield'] or jun['commit']:
            bad.append(f'교차로 양보 {jun["yield"]}회 / commit {jun["commit"]}회 '
                       '(교차 차량 없는 도로)')
        print()
        print('[quiet] ' + ('통과' if not bad else 'FAIL: ' + ', '.join(bad)))
        raise SystemExit(1 if bad else 0)


if __name__ == '__main__':
    main()
