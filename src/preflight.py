#!/usr/bin/env python3
"""현장 셋업 점검. 한 번 실행해서 전부 확인하고 다음에 칠 명령을 출력한다.

    python preflight.py --host 192.168.50.11
    python preflight.py --host 192.168.50.11 --lidar

현장 소요시간이 15분이다. 무엇이 안 되는지 찾느라 시간을 쓰지 않는 것이
이 스크립트의 유일한 목적이다. 확인 순서는 실패했을 때 원인이 좁혀지는 순서다.

**9910은 연결을 하나만 허용한다.** 그래서 마지막에 소켓을 확실히 닫는다.
이게 열려 있으면 pure_pursuit이 붙지 못한다.
"""

from __future__ import annotations

import argparse
import importlib
import math
import os
import socket
import statistics
import struct
import sys
import time

from hlvtd_client import (
    Client, FRAME_SIZE, SpeedEstimator, TL_STATE, pack_control,
)

OK, WARN, BAD = '  [OK]', '  [주의]', '  [실패]'


class Report:
    def __init__(self):
        self.fail = 0
        self.warn = 0

    def ok(self, msg): print(f'{OK} {msg}')

    def warn_(self, msg):
        self.warn += 1
        print(f'{WARN} {msg}')

    def bad(self, msg):
        self.fail += 1
        print(f'{BAD} {msg}')


def step(title: str) -> None:
    print(f'\n--- {title} ' + '-' * max(0, 60 - len(title)))


def check_env(rep: Report, here: str) -> None:
    step('1. 실행 환경')
    v = sys.version_info
    if (v.major, v.minor) >= (3, 10):
        rep.ok(f'Python {v.major}.{v.minor}.{v.micro}')
    else:
        rep.bad(f'Python {v.major}.{v.minor} — 3.10 이상이 필요하다 '
                f'(union 타입 문법). conda activate hlfma 를 했나?')
    # numpy는 확인하지 않는다. 이 PC에서는 앱 제어 정책에 차단돼 있고
    # (import는 통과해도 numpy.linalg에서 DLL 로드 실패), 우리 코드는
    # 표준 라이브러리만 쓴다. 대신 우리 모듈이 실제로 import되는지를 본다.
    for mod in ('hlvtd_client', 'session_log', 'pure_pursuit'):
        try:
            importlib.import_module(mod)
            rep.ok(f'{mod}.py import')
        except Exception as exc:
            rep.bad(f'{mod}.py import 실패: {exc!r}')

    root = os.path.normpath(os.path.join(here, '..'))
    for name in ('data/waypoints_v2.csv',):
        p = os.path.join(root, name)
        if os.path.exists(p):
            rep.ok(f'{name} ({os.path.getsize(p):,}B)')
        else:
            rep.bad(f'{name} 없음')

    logs = os.path.join(root, 'logs')
    try:
        os.makedirs(logs, exist_ok=True)
        probe = os.path.join(logs, '.preflight')
        with open(probe, 'w') as fh:
            fh.write('ok')
        os.remove(probe)
        rep.ok(f'logs/ 쓰기 가능')
    except OSError as exc:
        rep.bad(f'logs/ 쓰기 불가: {exc} — 세션 로그가 안 남는다')


def check_reachable(rep: Report, host: str, port: int) -> bool:
    step(f'2. {host}:{port} 도달')
    t0 = time.monotonic()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(3.0)
    try:
        s.connect((host, port))
        rep.ok(f'TCP 연결 성공 ({1000 * (time.monotonic() - t0):.0f}ms)')
        return True
    except socket.timeout:
        rep.bad('타임아웃 — 랜선, IP 설정(nmcli con up hlfma), '
                '방화벽 순으로 확인할 것')
    except OSError as exc:
        rep.bad(f'{exc}\n         VTD가 안 떴거나 Setup이 00_HL_VTD가 아니다.\n'
                f'         mainRS / mainRS_Broadcast / mainRS_LiDAR 창 3개가 '
                f'떠 있는지 확인할 것')
    finally:
        s.close()
    return False


def check_stream(rep: Report, host: str, port: int, n_frames: int) -> None:
    step(f'3. 9910 수신 {n_frames}프레임')
    client = Client(host, port)
    try:
        client.connect(retry_forever=False)
    except OSError as exc:
        rep.bad(f'연결 실패: {exc}')
        return

    stamps, frames = [], []
    try:
        for _ in range(n_frames):
            f = client.recv_frame()
            if f is None:
                rep.bad(f'{len(frames)}프레임 만에 끊겼다')
                break
            stamps.append(time.monotonic())
            frames.append(f)
            client.send_control(0.0, 0.0)       # 정지 명령만 — 안전하다
    except (OSError, ValueError, struct.error) as exc:
        rep.bad(f'수신 오류: {exc!r}')
    finally:
        client.close()                          # 반드시 닫는다. 연결은 하나뿐이다

    if len(frames) < 3:
        return

    rep.ok(f'프레임 {FRAME_SIZE}B 파싱 {len(frames)}개 성공')

    # 첫 간격은 연결 직후라 핸드셰이크가 섞인다. 주기 판정에서 뺀다.
    dts = [1000 * (b - a) for a, b in zip(stamps, stamps[1:])][1:]
    mean, mx = statistics.mean(dts), max(dts)
    line = f'수신 주기 평균 {mean:.1f}ms  최대 {mx:.1f}ms  ({1000 / mean:.2f} fps)'
    if 45 <= mean <= 56 and mx < 90:
        rep.ok(line + '  (공칭 50.0ms)')
    else:
        rep.warn_(line + ' — 20Hz에서 벗어난다. 속도 추정이 흔들릴 수 있다')

    f0, fl = frames[0], frames[-1]
    moved = math.hypot(fl.ego_x - f0.ego_x, fl.ego_y - f0.ego_y)
    est = SpeedEstimator()
    for f, t in zip(frames, stamps):
        est.update(f.ego_x, f.ego_y, t)
    print(f'       ego ({fl.ego_x:.2f}, {fl.ego_y:.2f})  '
          f'heading {math.degrees(fl.ego_heading):+.1f}°  '
          f'속도 {est.kmh:.1f}km/h  (관측 중 이동 {moved:.2f}m)')

    n_obj = [len(f.objects) for f in frames]
    ids = {o.id for f in frames for o in f.objects}
    peds = {o.id for f in frames for o in f.objects if o.is_pedestrian}
    print(f'       객체 {min(n_obj)}~{max(n_obj)}개, 서로 다른 id {len(ids)}개'
          + (f' (보행자 판정 {len(peds)}개)' if peds else ''))
    if max(n_obj) >= 30:
        rep.warn_('30슬롯 상한에 닿았다 — 80m 안에 객체가 가득하다')

    states = {f.tl_state for f in frames}
    print(f'       신호등 id={fl.tl_id} '
          f'state={"/".join(sorted(TL_STATE.get(s, str(s)) for s in states))}')
    if states == {0}:
        rep.warn_('신호등이 계속 미할당(0)이다 — 교차로에서 떨어져 있으면 정상')


def check_lidar(rep: Report, port: int, seconds: float) -> None:
    step(f'4. LiDAR UDP {port} ({seconds:.0f}초)')
    HDR = struct.Struct('<IHIQdHHIIHBB')
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 22)
    s.settimeout(1.0)
    try:
        s.bind(('0.0.0.0', port))
    except OSError as exc:
        rep.bad(f'바인드 실패: {exc}')
        return

    pkts, byts, frames_seen, modes = 0, 0, set(), set()
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            try:
                data, _ = s.recvfrom(65535)
            except socket.timeout:
                continue
            pkts += 1
            byts += len(data)
            if len(data) >= HDR.size:
                h = HDR.unpack_from(data, 0)
                frames_seen.add(h[2])           # frame_id
                modes.add(h[10])                # coordinate_mode
    finally:
        s.close()

    if not pkts:
        rep.warn_('패킷 없음 — 실행 순서(랜선 → nmcli → lmgrd → vtdStart.sh)를 '
                  '지켰나. 순서가 틀리면 LiDAR 전송기가 소켓 생성에 실패한다')
        return
    dur = time.monotonic() - t0
    rep.ok(f'패킷 {pkts}개, 프레임 {len(frames_seen)}개 '
           f'({len(frames_seen) / dur:.1f} fps), {8 * byts / dur / 1e6:.1f} Mbps')
    if modes and modes != {1}:
        rep.warn_(f'coordinate_mode={modes} — 1(월드 좌표)이 아니면 변환이 필요하다')


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description='현장 셋업 점검')
    ap.add_argument('--host', default='192.168.50.11', help='VTD PC IP')
    ap.add_argument('--port', type=int, default=9910)
    ap.add_argument('--frames', type=int, default=40, help='수신 확인 프레임 수')
    ap.add_argument('--lidar', action='store_true', help='UDP 9912도 확인')
    ap.add_argument('--lidar-port', type=int, default=9912)
    ap.add_argument('--lidar-seconds', type=float, default=3.0)
    args = ap.parse_args()

    print('=' * 66)
    print(f'  HLFMA preflight   {time.strftime("%Y-%m-%d %H:%M:%S")}   '
          f'대상 {args.host}:{args.port}')
    print('=' * 66)

    rep = Report()
    check_env(rep, here)
    if check_reachable(rep, args.host, args.port):
        check_stream(rep, args.host, args.port, args.frames)
    else:
        print('       (연결이 안 되므로 수신 확인은 건너뛴다)')
    if args.lidar:
        check_lidar(rep, args.lidar_port, args.lidar_seconds)

    print('\n' + '=' * 66)
    if rep.fail:
        print(f'  실패 {rep.fail}개, 주의 {rep.warn}개 — 위 항목부터 해결할 것')
        print('=' * 66)
        sys.exit(1)

    print(f'  이상 없음 (주의 {rep.warn}개). 9910 소켓은 닫았다.')
    print('=' * 66)
    print(f"""
  다음 순서:

  1) route CSV를 받으면 먼저 커버리지 확인
     python route_check.py --route <받은파일> --svg ../logs/coverage.svg

  2) 제안된 구간으로 주행 (--route 를 주면 원본이 세션에 복사된다)
     python pure_pursuit.py --host {args.host} --speed 20 \\
         --segments <제안된목록> --route <받은파일>

  3) 수동 주행·녹화가 필요하면
     python drive_and_record.py --host {args.host} --segment <이름> \\
         --out ../data/waypoints_pretest.csv

  주행이 끝나면 logs/ 아래 세션 디렉터리를 통째로 챙겨올 것.
  frames.bin 이 있으면 집에서 replay_server.py 로 그대로 재현된다.
""")


if __name__ == '__main__':
    main()
