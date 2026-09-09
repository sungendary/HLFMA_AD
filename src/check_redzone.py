#!/usr/bin/env python3
"""붉은 보호구역 계수기 (평가항목 2).

check_redlight / check_collision / check_signals의 쌍둥이다. 세션 로그를 실측
붉은 구간(기본: 경로 2의 (359,-71)~(515,-137))과 대조해 네 가지를 잰다.

    python check_redzone.py ../logs/rz_stub --baseline ../logs/rz_off
    python check_redzone.py ../logs/pp_2026091* --zone 359,-71,515,-137

A 분리도      구간 안/밖의 rz_far / rz_ratio 분포(p10/p50/p90). 임계값 튜닝 근거.
B 진입·이탈   redzone_enter/exit 이벤트의 경로 진행거리 s를 구간 s와 비교해
              "N m 일찍/늦게". **진입이 늦거나 이탈이 이르면 FAIL** ― 그 반대는
              저속 주행이라 무감점이다. 홀드(9/10)가 있는 로그는 속도 제한이 실제로
              풀린 redzone_hold_end를 이탈로 본다. 구간 안 카메라 이탈·재진입은
              참고로만 찍고, 재진입이 3회 이상이면 경고한다(끊김이 홀드보다 길다).
C 구간 내 속도 후륜축이 구간 s 안일 때 speed의 p50/p95/최대. 최대 > --limit면 FAIL.
              **추정치다** ― 평가는 VTD 내부 속도라 여유를 둬야 한다.
D 카메라      camera_lost / back / flapping 횟수, 프레임 주기 p95와 65.5ms 초과율
              (카메라 스레드가 20Hz를 갉아먹었는지).

구간 좌표는 사각형이 아니라 **경로 위 s 범위**로 바꿔 쓴다. 두 끝점을
path.nearest로 스냅한다 ― 도로가 대각선이라 사각형은 모서리가 도로 밖으로 나간다.
우회 세션은 원본 경로 기준이라 우회 구간의 투영 오차를 감수한다(다른 계수기와 같다).

옛 로그(rz_* 컬럼 없음)는 A·B를 측정불가로 내고 C·D만 잰다 ― 베이스라인이
"구간 내 45km/h"로 나오는 것이 정확한 기록이다.
"""

from __future__ import annotations

import argparse
import csv
import io
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from check_signals import expand, guess_inputs, project, read_meta
import redzone

DEFAULT_ZONE = (359.0, -71.0, 515.0, -137.0)   # 경로 2 붉은 구간 (육안 확인)
LIMIT_KMH = 30.0            # 항목 2 제한속도
PERIOD_WARN_MS = 65.5       # 프레임 주기 기준 (사전테스트 실측 잣대)
SNAP_WARN_M = 8.0           # 구간 끝점이 경로에서 이보다 멀면 경고


def read_drive(session: str) -> list:
    out = []
    with io.open(os.path.join(session, 'drive.csv'), newline='',
                 encoding='utf-8') as fh:
        rdr = csv.DictReader(fh)
        has_rz = 'rz_in' in (rdr.fieldnames or [])
        for r in rdr:
            try:
                f = {'n': int(r['n']), 't': float(r['t']),
                     'x': float(r['ego_x']), 'y': float(r['ego_y']),
                     'v': float(r['speed'])}
            except (ValueError, TypeError, KeyError):
                continue
            if has_rz:
                f['far'] = _f(r.get('rz_far'))
                f['near'] = _f(r.get('rz_ratio'))
                f['in'] = int(float(r.get('rz_in') or 0))
            out.append(f)
    return out, has_rz


def _f(s) -> float:
    try:
        return float(s)
    except (TypeError, ValueError):
        return float('nan')


def read_events(session: str) -> list:
    out = []
    p = os.path.join(session, 'events.csv')
    if not os.path.exists(p):
        return out
    with io.open(p, newline='', encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            k = (r.get('kind') or '')
            if not k.startswith('redzone_'):
                continue
            out.append({'t': _f(r.get('t')), 'kind': k,
                        'detail': r.get('detail') or '',
                        'x': _f(r.get('ego_x')), 'y': _f(r.get('ego_y')),
                        'v': _f(r.get('speed'))})
    return out


def pct(vals: list, p: float) -> float:
    if not vals:
        return float('nan')
    s = sorted(vals)
    k = max(0, min(len(s) - 1, int(round(p / 100.0 * (len(s) - 1)))))
    return s[k]


def zone_range(path, zone: tuple) -> tuple:
    """(s0, s1, resid0, resid1). 두 끝점을 경로에 스냅해 s 범위로."""
    x0, y0, x1, y1 = zone
    i0, i1 = path.nearest(x0, y0), path.nearest(x1, y1)
    r0 = math.hypot(path.pts[i0][0] - x0, path.pts[i0][1] - y0)
    r1 = math.hypot(path.pts[i1][0] - x1, path.pts[i1][1] - y1)
    s0, s1 = sorted((path.s[i0], path.s[i1]))
    return s0, s1, r0, r1


def report(session: str, waypoints: str, segments: list, zone: tuple,
           limit_kmh: float) -> dict:
    name = os.path.basename(os.path.normpath(session))
    tally = {'zones': 0, 'enter_fail': 0, 'exit_fail': 0, 'speed_fail': 0,
             'lost': 0, 'flap': 0, 'unknown': 0, 'reenter': 0}
    frames, has_rz = read_drive(session)
    if not frames:
        print(f'  {name}: drive.csv가 비어 있다')
        return tally

    meta = read_meta(session)
    wp, _ev, segs = guess_inputs(session, meta, waypoints, '', segments)
    if not os.path.exists(wp) or not segs:
        print(f'  {name}: 경로를 복원할 수 없다 ({wp}, {segs})')
        return tally
    from pure_pursuit import load_path
    path, _bounds = load_path(wp, list(segs))
    project(path, frames)
    s0, s1, r0, r1 = zone_range(path, zone)
    print(f'  {name}  {len(frames)}프레임 {frames[-1]["t"]:.1f}초  '
          f'redzone={"on" if meta.get("redzone") else "off/없음"}'
          + (f'  ({meta.get("redzone_url")})' if meta.get('redzone_url') else ''))
    print(f'    붉은 구간 s {s0:.1f}~{s1:.1f}m ({s1 - s0:.1f}m)  '
          f'끝점 스냅 잔차 {r0:.1f} / {r1:.1f}m'
          + ('  *** 경로에서 멀다 ― 이 경로에 붉은 구간이 없거나 --zone이 틀렸다 ***'
             if max(r0, r1) > SNAP_WARN_M else ''))
    zone_ok = max(r0, r1) <= SNAP_WARN_M
    if zone_ok:
        tally['zones'] = 1
    inside = [f for f in frames if s0 <= f['s'] <= s1]
    outside = [f for f in frames if not (s0 - 30.0 <= f['s'] <= s1 + 30.0)]

    # A 분리도
    print('    A 분리도 (안 p10/p50/p90 | 밖 p10/p50/p90)')
    if not has_rz:
        print('      측정불가 ― rz_* 컬럼이 없는 로그')
        tally['unknown'] += 1
    else:
        for key, label in (('far', 'far  (진입)'), ('near', 'near (이탈)')):
            vi = [f[key] for f in inside if f[key] == f[key]]
            vo = [f[key] for f in outside if f[key] == f[key]]
            print(f'      {label}  안 {pct(vi, 10):.2f}/{pct(vi, 50):.2f}/{pct(vi, 90):.2f}'
                  f'  밖 {pct(vo, 10):.2f}/{pct(vo, 50):.2f}/{pct(vo, 90):.2f}'
                  + (f'  ← 안 p10 {pct(vi, 10):.2f} < 진입 임계 {redzone.ENTER_RATIO}'
                     if key == 'far' and vi and pct(vi, 10) < redzone.ENTER_RATIO else '')
                  + (f'  ← 밖 p90 {pct(vo, 90):.2f} >= 이탈 임계 {redzone.EXIT_RATIO}'
                     if key == 'near' and vo and pct(vo, 90) >= redzone.EXIT_RATIO else ''))
        n_z = sum(1 for f in inside if f['in'] == 1)
        n_h = sum(1 for f in inside if f['in'] == 2)
        print(f'      rz_in: 구간 안 zone {n_z} / hold {n_h} / 전체 {len(inside)}프레임'
              f'  구간 밖(±30m 제외) zone {sum(1 for f in outside if f["in"] == 1)}'
              f' / hold {sum(1 for f in outside if f["in"] == 2)} / {len(outside)}')

    # B 진입·이탈 시점
    evs = read_events(session)
    print('    B 진입·이탈 (경로 s 기준, 음수=일찍)')
    enters = [e for e in evs if e['kind'] == 'redzone_enter']
    exits = [e for e in evs if e['kind'] == 'redzone_exit']
    hold_ends = [e for e in evs if e['kind'] == 'redzone_hold_end']
    for e in evs:
        if e['x'] == e['x']:
            e['s'] = path.s[path.nearest(e['x'], e['y'])]
        else:
            e['s'] = float('nan')
    if not has_rz:
        print('      측정불가 ― rz_* 컬럼이 없는 로그')
    elif not zone_ok:
        print('      구간이 이 경로에 없다 ― 진입/이탈 이벤트 '
              f'{len(enters)}/{len(exits)}건은 참고만')
        for e in enters + exits:
            print(f'        {e["kind"]:13s} t={e["t"]:7.2f} s={e["s"]:7.1f}  {e["detail"]}')
    else:
        near_enter = [e for e in enters if s0 - 60.0 <= e['s'] <= s1]
        # 홀드가 있는 로그는 속도 제한이 실제로 풀린 hold_end가 이탈이다. 카메라
        # 이탈(redzone_exit)은 구간 안에서도 나고(끊긴 노면) 홀드가 그것을 잇는다.
        release, rel_label = (hold_ends, '해제') if hold_ends else (exits, '이탈')
        near_exit = [e for e in release if s0 <= e['s'] <= s1 + 150.0]
        if not near_enter:
            tally['enter_fail'] += 1
            print(f'      FAIL  진입 없음 (구간 s {s0:.1f}에서 redzone_enter가 없다)')
        for e in near_enter[:1]:
            d = e['s'] - s0
            bad = d > 0.0
            tally['enter_fail'] += 1 if bad else 0
            print(f'      {"FAIL" if bad else "PASS"}  진입 s={e["s"]:7.1f}  '
                  f'{d:+6.1f}m ({"늦게" if bad else "일찍"})  '
                  f'v={e["v"] * 3.6:4.1f}km/h  t={e["t"]:.2f}  {e["detail"]}')
        for e in near_enter[1:]:
            print(f'      ....  재진입 s={e["s"]:7.1f}  v={e["v"] * 3.6:4.1f}km/h  '
                  f't={e["t"]:.2f}  {e["detail"]}')
        tally['reenter'] += max(0, len(near_enter) - 1)
        if len(near_enter) >= 3:
            print(f'      WARN  구간 안 들락날락 {len(near_enter)}회 ― 저속이라 감점은 '
                  f'아니지만 끊김이 홀드보다 길다는 뜻이다 (--zone-hold 확인)')
        for e in exits:
            if s0 <= e['s'] < s1 and hold_ends:
                print(f'      ....  카메라 이탈 s={e["s"]:7.1f}  v={e["v"] * 3.6:4.1f}km/h  '
                      f't={e["t"]:.2f}  (구간 안 ― 홀드가 잇는다)')
        if not near_exit:
            print(f'      ....  {rel_label} 없음 (구간 끝 s {s1:.1f} 뒤 이벤트가 없다 ― '
                  f'경로 끝이면 정상)')
        for e in near_exit:
            d = e['s'] - s1
            bad = d < 0.0
            tally['exit_fail'] += 1 if bad else 0
            print(f'      {"FAIL" if bad else "PASS"}  {rel_label} s={e["s"]:7.1f}  '
                  f'{d:+6.1f}m ({"일찍" if bad else "늦게"})  '
                  f'v={e["v"] * 3.6:4.1f}km/h  t={e["t"]:.2f}  {e["detail"]}')
        stray = [e for e in enters if e not in near_enter]
        if stray:
            print(f'      (구간 밖 진입 {len(stray)}건 ― 오탐. 저속 주행이라 감점은 아니다: '
                  + ', '.join(f's={e["s"]:.0f}' for e in stray) + ')')

    # C 구간 내 속도
    print(f'    C 구간 내 속도 (추정치, 제한 {limit_kmh:.0f}km/h)')
    if not zone_ok:
        print('      구간이 이 경로에 없다')
    elif not inside:
        print('      구간을 지나지 않았다 (미완주?)')
    else:
        vs = [f['v'] * 3.6 for f in inside]
        vmax = max(vs)
        # 진입 직후 감속 중인 프레임은 어디까지 넘겼는지 따로 본다
        over = [f for f in inside if f['v'] * 3.6 > limit_kmh]
        over_m = (max(f['s'] for f in over) - s0) if over else 0.0
        bad = vmax > limit_kmh
        tally['speed_fail'] += 1 if bad else 0
        print(f'      {"FAIL" if bad else "PASS"}  p50 {pct(vs, 50):.1f}  p95 {pct(vs, 95):.1f}  '
              f'최대 {vmax:.1f}km/h  ({len(inside)}프레임, {inside[-1]["t"] - inside[0]["t"]:.1f}초)'
              + (f'  ← 초과 {len(over)}프레임, 구간 시작부터 {over_m:.1f}m까지' if over else ''))

    # D 카메라·주기
    lost = [e for e in evs if e['kind'] == 'redzone_camera_lost']
    back = [e for e in evs if e['kind'] == 'redzone_camera_back']
    flap = [e for e in evs if e['kind'] == 'redzone_flapping']
    tally['lost'] += len(lost)
    tally['flap'] += len(flap)
    dts = [(frames[i]['t'] - frames[i - 1]['t']) * 1000.0
           for i in range(1, len(frames))]
    over_n = sum(1 for d in dts if d > PERIOD_WARN_MS)
    print('    D 카메라 / 주기')
    print(f'      camera_lost {len(lost)}  back {len(back)}  flapping {len(flap)}'
          + (f'  (첫 lost t={lost[0]["t"]:.1f}s: {lost[0]["detail"]})' if lost else ''))
    st = meta.get('redzone_stats') or {}
    if st:
        print(f'      카메라 판정 {st.get("frames", 0)}프레임'
              + (f' / 수신 {st.get("decoded")} (입력 {st.get("fps_in")}fps, '
                 f'판정 {st.get("fps_eval")}fps)' if 'decoded' in st else '')
              + f'  ffmpeg 재기동 {st.get("restarts", 0)}'
              f'  진입 {st.get("enter", 0)}  이탈 {st.get("exit", 0)}'
              + (f'  홀드 {st.get("hold")} / 만료 {st.get("hold_end")}' if 'hold' in st else '')
              + (f'  last_err={st.get("last_err")}' if st.get('last_err') else ''))
    if dts:
        print(f'      프레임 주기 p50 {pct(dts, 50):.1f}  p95 {pct(dts, 95):.1f}  '
              f'최대 {max(dts):.1f}ms  {PERIOD_WARN_MS}ms 초과 '
              f'{100.0 * over_n / len(dts):.2f}%')
    return tally


def main() -> None:
    ap = argparse.ArgumentParser(description='붉은 보호구역 계수기 (항목 2)')
    ap.add_argument('sessions', nargs='+', help='세션 디렉터리')
    ap.add_argument('--baseline', nargs='*', default=[],
                    help='비교할 베이스라인 세션 (--no-redzone 등)')
    ap.add_argument('--path', default='',
                    help='waypoint CSV. 생략하면 meta.json에서 찾는다')
    ap.add_argument('--segments', default='',
                    help='쉼표로 구분한 구간 이름. 생략하면 meta.json')
    ap.add_argument('--zone', default=','.join(str(v) for v in DEFAULT_ZONE),
                    help='붉은 구간 양 끝 x0,y0,x1,y1 (기본: 경로 2 실측)')
    ap.add_argument('--limit', type=float, default=LIMIT_KMH,
                    help='구간 제한속도 [km/h]')
    args = ap.parse_args()
    segs = [s.strip() for s in args.segments.split(',') if s.strip()]
    zone = redzone.parse_rect(args.zone)

    def run(label, sessions):
        tally = {'zones': 0, 'enter_fail': 0, 'exit_fail': 0, 'speed_fail': 0,
                 'lost': 0, 'flap': 0, 'unknown': 0, 'reenter': 0}
        if not sessions:
            return tally
        print(f'\n== {label}')
        for s in expand(sessions):
            if not os.path.exists(os.path.join(s, 'drive.csv')):
                print(f'  [skip] {s}: drive.csv 없음')
                continue
            r = report(s, args.path, segs, zone, args.limit)
            for k in tally:
                tally[k] += r[k]
        return tally

    base = run('베이스라인', args.baseline)
    cur = run('대상', args.sessions)

    print('\n== 합계 (구간 수 기준)')
    print(f'{"":14s}{"늦은 진입":>10s}{"이른 이탈":>10s}{"속도 초과":>10s}'
          f'{"camera_lost":>13s}{"flapping":>10s}')
    if args.baseline:
        print(f'{"베이스라인":14s}{base["enter_fail"]:>6d}/{base["zones"]:<3d}'
              f'{base["exit_fail"]:>6d}/{base["zones"]:<3d}'
              f'{base["speed_fail"]:>6d}/{base["zones"]:<3d}'
              f'{base["lost"]:>13d}{base["flap"]:>10d}')
    print(f'{"대상":14s}{cur["enter_fail"]:>6d}/{cur["zones"]:<3d}'
          f'{cur["exit_fail"]:>6d}/{cur["zones"]:<3d}'
          f'{cur["speed_fail"]:>6d}/{cur["zones"]:<3d}'
          f'{cur["lost"]:>13d}{cur["flap"]:>10d}')
    if cur['reenter']:
        print(f'  구간 안 재진입 {cur["reenter"]}회 (끊긴 노면을 홀드가 이었는지 B 절 참고)')
    if cur['unknown']:
        print(f'  {cur["unknown"]}세션은 rz_* 컬럼이 없어 A·B를 재지 못했다')
    raise SystemExit(1 if (cur['enter_fail'] or cur['exit_fail']
                           or cur['speed_fail']) else 0)


if __name__ == '__main__':
    main()
