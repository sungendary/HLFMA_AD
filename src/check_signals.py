#!/usr/bin/env python3
"""방향지시등·횡단보도 정차 계수기 (평가항목 13·12).

check_redlight.py / check_collision.py의 쌍둥이다. 세션 로그를 정책과 **같은
잣대로** 재고, 베이스라인(그 기능이 없던 로그)과 나란히 놓는다.

    python check_signals.py ../logs/sim_pt2_rule --baseline ../logs/sim_pt2_before

A 방향지시등 (항목 13, 경미 -3)
    차선변경 구간마다 **구간 시작 직전의 연속 점등 시간**을 잰다. 3초 이상이면
    합격이다. 교차로 회전 구간은 규칙이 강제하지 않으므로 참고로만 낸다.

    구간은 turnsignal.build_zones가 만든 것을 그대로 쓴다 ― 제어기가 본 것과
    같은 구간이어야 판정이 의미가 있다.

B 횡단보도 정차 (항목 12, 경미 -3)
    후륜축이 횡단보도 구간 안에 있으면서 1km/h 이하인 **연속 구간의 길이**를
    잰다. 3초 이상이면 위반이다.

    **후륜축 기준이다.** 9910 ego 좌표가 곧 후륜축이므로 보정이 없다.

옛 로그에는 turn / cw_act 컬럼이 없다. 전부 get으로 읽어 그대로 0/빈칸으로
떨어지게 둔다 ― 베이스라인이 "점등 0초"로 나오는 것이 정확한 기록이다.

--- 우회 주행 (leg 분할) ---

detour.py가 경로를 갈아 끼우면 **그 뒤로는 원본 경로를 달리지 않는다.** 그런데
meta.json의 waypoints는 일부러 원본으로 남긴다(우회로로 바꾸면 그 밖의 신호·충돌을
아예 못 센다). 그래서 원본 경로로만 채점하면 **달린 적도 없는 차선변경이 미점등으로
찍힌다** ― logs/pp_20260909_152624가 실제로 2/2 FAIL로 나왔고, 제어기는 그때
우회로의 차선변경에 정상 점등하고 있었다.

meta.json의 detours[]가 교체 시각과 CSV를 기록하므로, 주행을 그 시각으로 **leg**로
잘라 각 leg를 **그때 실제로 달린 경로**로 채점한다. 우회가 없으면 leg가 하나뿐이라
예전과 글자 그대로 같은 출력이 나온다.

--path / --events / --segments를 명시하면 leg 분할을 끄고 그 경로 하나로만 잰다.
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


def guess_inputs(session: str, meta: dict, waypoints: str, events: str,
                 segments: list) -> tuple:
    """meta.json에서 첫 leg의 경로·이벤트·구간을 복원한다. (wp, ev, segs)."""
    wp = waypoints or meta.get('waypoints') or ''
    if not os.path.exists(wp):
        wp = os.path.join(session, 'waypoints.csv')
    ev = events or meta.get('events') or ''
    if not os.path.exists(ev):
        cand = os.path.join(session, 'events_in.csv')
        ev = cand if os.path.exists(cand) else ''
    return wp, ev, list(segments or meta.get('segments') or [])


def read_meta(session: str) -> dict:
    try:
        with io.open(os.path.join(session, 'meta.json'), encoding='utf-8') as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def legs_of(session: str, meta: dict, wp: str, ev: str, segs: list) -> list:
    """주행 구간(leg) 목록. [{label, wp, ev, segs, t0, t1}, ...].

    meta.detours[]가 없으면 leg 하나 = 예전 동작이다.

    **CSV는 basename으로 세션 디렉터리 안에서 찾는다** ― meta에 박힌 절대경로는
    로그를 옮기면 깨진다. 이벤트 CSV는 detour_N_events.csv 규약을 그대로 쓴다
    (detour.DetourPlanner._solve).
    """
    legs = [{'label': os.path.basename(wp) or 'waypoints.csv',
             'wp': wp, 'ev': ev, 'segs': list(segs), 'detour': False,
             't0': float('-inf'), 't1': float('inf')}]
    det = meta.get('detours') or []
    for k, d in enumerate(det):
        try:
            t0 = float(d['t'])
        except (KeyError, TypeError, ValueError):
            continue
        csv_path = os.path.join(session, os.path.basename(d.get('csv') or ''))
        if not os.path.exists(csv_path):
            print(f'      *** detours[{k}]의 {os.path.basename(csv_path)}가 '
                  f'없다 ― 그 leg는 앞 경로로 잰다 ***')
            continue
        ev_path = os.path.splitext(csv_path)[0] + '_events.csv'
        legs[-1]['t1'] = t0
        legs.append({'label': os.path.basename(csv_path), 'wp': csv_path,
                     'ev': ev_path if os.path.exists(ev_path) else '',
                     'segs': [], 'detour': True,
                     't0': t0, 't1': float('inf')})
    return legs


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

def check_turn(frames: list, zones: list, partial: bool = False) -> list:
    """구간마다 (zone, 선행 점등 시간, 방향 일치, 측정 가능, 앞선 소등).

    partial=True면 이 프레임 묶음이 주행의 일부(leg)라는 뜻이다. **점등 방향이
    맞는데 leg가 구간 시작 3초 앞을 못 덮으면 측정불가다** ― 선행 점등이 앞
    leg에 걸쳐 있는 것이지 위반이 아니다. 방향이 틀린 것은 그 프레임만 보면
    되므로 leg가 짧아도 판정할 수 있다.

    다섯째 값은 **연속 점등 구간 바로 앞의 소등 길이와 그 앞의 같은 방향 점등
    시간**이다 (없으면 None). 판정에는 쓰지 않는다 ― turnsignal의 정차 소등
    (turn_pause)이 선행 3초를 갉아먹었는지 사후에 보려고 낸다.
    """
    out = []
    for z in zones:
        hit = next((i for i, f in enumerate(frames) if f['s'] >= z.s_start),
                   None)
        if hit is None:
            out.append((z, float('nan'), False, True, None))
            continue
        # 구간 시작 직전의 **연속** 점등만 센다. 중간에 꺼지면 거기서 끊긴다.
        j = hit
        while j > 0 and frames[j - 1]['turn'] == z.signal:
            j -= 1
        matched = frames[hit]['turn'] == z.signal
        lead = frames[hit]['t'] - frames[j]['t'] if matched else 0.0
        known = not (partial and matched and lead < LEAD_REQ_S
                     and frames[hit]['t'] - frames[0]['t'] < LEAD_REQ_S)
        out.append((z, lead, matched, known, _prior_gap(frames, j, z)))
    return out


def _prior_gap(frames: list, j: int, z) -> tuple | None:
    """연속 점등이 시작된 j 바로 앞의 소등 구간.

    (소등 시간, 그 전의 같은 방향 점등 시간, 소등 시작 t, 소등 끝 t).
    소등이 없거나 그 앞이 같은 방향이 아니면 None. 앞뒤가 같은 방향이라는
    조건이 "끊긴 하나의 점등"을 뜻한다 -- 정차 소등(turn_pause)이 선행 3초를
    갉아먹었는지가 이 값으로 드러난다.
    """
    k = j
    while k > 0 and frames[k - 1]['turn'] == 0:
        k -= 1
    if k == j or k == 0 or frames[k - 1]['turn'] != z.signal:
        return None
    m = k
    while m > 0 and frames[m - 1]['turn'] == z.signal:
        m -= 1
    return (frames[j]['t'] - frames[k]['t'], frames[k]['t'] - frames[m]['t'],
            frames[k]['t'], frames[j]['t'])


def read_pauses(session: str) -> list:
    """events.csv의 turn_pause 시각. 없으면 빈 목록.

    소등 구간이 **정차 소등인지** 그냥 두 구간 사이의 정상 소등인지는 이걸로만
    가른다. 시간 길이로 어림하면 45초짜리 신호 대기 소등과 구분이 안 된다.
    """
    out = []
    p = os.path.join(session, 'events.csv')
    if not os.path.exists(p):
        return out
    try:
        with io.open(p, newline='', encoding='utf-8') as fh:
            for r in csv.DictReader(fh):
                if (r.get('kind') or '') == 'turn_pause':
                    out.append(float(r['t']))
    except (OSError, ValueError, KeyError):
        return []
    return out


def lit_before_moving(frames: list, z, move_ms: float = DWELL_MS) -> float:
    """차가 처음 움직이기 시작한 프레임까지의 **연속 점등 시간** [s].

    우회 탈출 블렌드는 구간 시작이 경로 원점(s=0)이라 check_turn의 선행이 늘
    0초로 나온다 -- 교체 프레임이 곧 판정 프레임이기 때문이다. 그 숫자는
    램프가 아니라 측정의 성질이다. 정지 상태에서 켜 두고 출발했다면 실제로는
    그만큼 앞서 알린 것이므로 따로 낸다.
    """
    mv = next((i for i, f in enumerate(frames) if f['v'] > move_ms), None)
    if mv is None:
        return 0.0
    j = mv
    while j > 0 and frames[j - 1]['turn'] == z.signal:
        j -= 1
    if frames[mv]['turn'] != z.signal:
        return 0.0
    return frames[mv]['t'] - frames[j]['t']


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

def report(session: str, waypoints: str, events: str, segments: list) -> dict:
    name = os.path.basename(os.path.normpath(session))
    frames = read_drive(session)
    tally = {'lc_fail': 0, 'lc_total': 0, 'lc_unknown': 0,
             'xw_viol': 0, 'xw_total': 0}
    if not frames:
        print(f'  {name}: drive.csv가 비어 있다')
        return tally

    meta = read_meta(session)
    wp, ev, segs = guess_inputs(session, meta, waypoints, events, segments)
    # 경로를 손으로 지정했으면 leg 분할을 끄고 그 경로 하나로만 잰다.
    manual = bool(waypoints or events or segments)
    legs = ([{'label': os.path.basename(wp), 'wp': wp, 'ev': ev,
              'segs': list(segs), 't0': float('-inf'), 't1': float('inf')}]
            if manual else legs_of(session, meta, wp, ev, segs))

    print(f'  {name}  {len(frames)}프레임 {frames[-1]["t"]:.1f}초'
          + (f'  (leg {len(legs)}개 ― 우회 {len(legs) - 1}회)'
             if len(legs) > 1 else ''))

    from pure_pursuit import load_path
    stray = 0.0
    lc_rows, xw_rows_all = [], []
    leg_frames = {}
    pauses = read_pauses(session)
    for k, leg in enumerate(legs):
        fr = [f for f in frames if leg['t0'] <= f['t'] < leg['t1']]
        leg_frames[k] = fr
        lsegs = leg['segs'] or (turnsignal._segments_of(leg['wp'])
                                if os.path.exists(leg['wp']) else [])
        if not fr or not os.path.exists(leg['wp']) or not lsegs:
            print(f'    leg {k + 1} {leg["label"]}: 경로를 복원할 수 없다')
            continue
        path, _bounds = load_path(leg['wp'], list(lsegs))
        if k == len(legs) - 1:
            # 런타임 최종 꼬리(라바콘·끝 너머 정지선)로 끝 프레임을 같은 기하로 투영
            import finish
            finish.apply_meta_tail(path, meta)
        project(path, fr)
        s_lo = min(f['s'] for f in fr)
        s_hi = max(f['s'] for f in fr)
        if len(legs) > 1:
            print(f'    leg {k + 1}  {leg["label"]}  '
                  f's {s_lo:.1f}~{s_hi:.1f}m  '
                  f't {max(0.0, fr[0]["t"]):.1f}~{fr[-1]["t"]:.1f}초')

        # 차선변경 구간만 나온다. 옛 CSV의 뒤집힌 양수 차로 마커는 build_zones가
        # lane_id로 정정하므로 **옛 로그를 다시 재면 그때의 (틀린) 점등이 방향
        # 불일치로 찍힌다** — 계수기 결함이 아니라 그때 실제로 틀렸던 것이다
        all_zones, _w = turnsignal.build_zones(path, leg['wp'])
        # **그 leg에서 실제로 지난 구간만** 채점한다. 우회로 교체 뒤의 옛 경로
        # 구간을 세면 달린 적 없는 차선변경이 미점등으로 찍힌다.
        zones = [z for z in all_zones if s_lo <= z.s_start <= s_hi]
        lc_rows.extend((k, r) for r in
                       check_turn(fr, zones, partial=len(legs) > 1))
        # 구간 밖 점등은 **거르지 않은** 구간 목록으로 잰다. leg가 끝나기 전에
        # 켜 둔 선행 점등(그 구간은 다음 leg에서 사라진다)까지 세면 안 된다.
        stray += stray_signal(fr, all_zones)

        xw_zones = ([] if not leg['ev']
                    else crosswalks.load_for_path(path, leg['ev'])[0])
        # **중심이 아니라 겹침으로 고른다.** det_many8에서 차가 s=129.5에 섰는데
        # 횡단보도 중심이 s=131.5라, 중심으로 걸렀더니 22.55초짜리 실제 위반이
        # 통째로 빠졌다. 후륜축이 구간 안에 있으면 위반이다.
        xw_zones = [z for z in xw_zones if z.s0 <= s_hi and z.s1 >= s_lo]
        xw_rows_all.extend((k, r) for r in check_crosswalk(fr, xw_zones))

    print('    A 방향지시등 (항목 13)')
    for k, (z, lead, matched, known, gap) in lc_rows:
        req = (z.kind == 'lc')
        # 우회 탈출 블렌드 -- 교체와 동시에 결정되는 기동이라 선행 3초가
        # 원리적으로 불가능하다. 마커가 blend_m/2에 있고 폭이 ±10m라 s_start는
        # 늘 0으로 잘린다. 합계에서는 측정불가로 빠지지만 **실제로 몇 초
        # 켰는지는 위에 그대로 찍힌다.**
        escape = req and legs[k].get('detour') and z.s_start <= 0.01
        # 소등 구간이 **정차 소등이었는지**는 이벤트로만 가른다. 길이로 어림하면
        # 두 구간 사이의 정상 소등(위 49.6초 같은 것)까지 섞인다.
        paused = bool(gap and any(gap[2] - 0.1 <= tp <= gap[3] + 0.1
                                  for tp in pauses))
        stopped_lit = (lit_before_moving(leg_frames.get(k) or [], z)
                       if escape else 0.0)
        if req and known:
            tally['lc_total'] += 1
        bad = req and known and not (matched and lead >= LEAD_REQ_S)
        tally['lc_fail'] += 1 if bad else 0
        tally['lc_unknown'] += 1 if (req and not known) else 0
        mark = ('FAIL' if bad else 'PASS') if req else '   .'
        if req and not known:
            mark = '  ??'
        print(f'      {mark}  s={z.s_start:7.1f}m  '
              + (f'[leg {k + 1}] ' if len(legs) > 1 else '')
              + f'{turnsignal.KIND_NAME[z.kind]:6s} {z.dir:5s}  '
                f'선행 {lead:5.2f}초'
              + ('' if matched else '  ← 미점등/방향 불일치')
              + (f'  (앞서 정차 소등 {gap[0]:.1f}초 / 그 전 {gap[1]:.1f}초 점등)'
                 if paused else '')
              + (f'  ← 우회 탈출 블렌드 (선행 3초 불가, '
                 f'출발 직전까지 {stopped_lit:.2f}초 점등)' if escape else '')
              + ('  ← leg가 3초 앞을 못 덮는다 (측정불가)'
                 if req and not known and not escape else ''))
    if not lc_rows:
        print('      점등 구간 없음')
    if stray > 0.5:
        print(f'      (구간 밖 점등 {stray:.1f}초 ― 감점 항목은 아니다)')

    print('    B 횡단보도 정차 (항목 12)')
    if not xw_rows_all:
        print('      횡단보도 없음 (이벤트 CSV 미지정)')
    for k, (z, dwell, act) in xw_rows_all:
        tally['xw_total'] += 1
        if dwell >= DWELL_REQ_S:
            tally['xw_viol'] += 1
        mark = 'VIOL' if dwell >= DWELL_REQ_S else 'ok  '
        print(f'      {mark}  s={z.s:7.1f}m  '
              + (f'[leg {k + 1}] ' if len(legs) > 1 else '')
              + f'최장 정차 {dwell:5.2f}초'
              + (f'  cw_act={act}' if act else ''))

    return tally


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
    args = ap.parse_args()
    segs = [s.strip() for s in args.segments.split(',') if s.strip()]

    def run(label, sessions):
        tally = {'lc_fail': 0, 'lc_total': 0, 'lc_unknown': 0,
                 'xw_viol': 0, 'xw_total': 0}
        if not sessions:
            return tally
        print(f'\n== {label}')
        for s in expand(sessions):
            r = report(s, args.path, args.events, segs)
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
    unknown = base['lc_unknown'] + cur['lc_unknown']
    if unknown:
        print(f'  차선변경 {unknown}건은 leg 경계에 걸려 측정불가라 뺐다 '
              f'(우회 교체가 구간 시작 3초 안에 일어났다)')
    raise SystemExit(1 if (cur['lc_fail'] or cur['xw_viol']) else 0)


if __name__ == '__main__':
    main()
