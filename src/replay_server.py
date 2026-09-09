#!/usr/bin/env python3
"""녹화한 frames.bin을 9910에서 다시 흘려보내는 재생 서버.

VTD 없이 수신·기록·이벤트 검출 경로를 통째로 시험하기 위한 것이다.
사전테스트에서 녹화한 frames.bin이 그대로 집에서의 테스트 벤치가 된다 ―
실제 코스의 신호등 전이와 실제 NPC가 담긴 데이터로 로직을 개발할 수 있다.

    python replay_server.py --frames ../archive/frames.bin --loop
    python replay_server.py --frames ../logs/pp_20260903_101500 --rate 2

    (다른 창에서)
    python pure_pursuit.py --segment p1_to_p2 --host 127.0.0.1

기본 모드는 **폐루프가 아니다.** 받은 제어는 통계만 내고 버린다. 보낸 조향이
차량 거동에 반영되지 않으므로 제어 로직 자체의 품질은 이걸로 판단할 수 없다.
확인할 수 있는 것은 기록·파싱·이벤트 검출 경로뿐이다.

--sim을 주면 **폐루프**가 된다. 받은 CTRL로 자전거 모델을 적분해 ego를 직접
만들고, 신호는 로그에서 뽑은 ctrl별 위상표로 재생하며, objects[]는 녹화
프레임 블록을 그대로 흘린다 (sim_world.py). 정지->유지->녹색출발이 실제로
일어나므로 적색 통과 횟수를 진짜로 셀 수 있다.

    python replay_server.py --sim --path ../data/route_path_pt2.csv
                            --tl-from ../logs/pp_20260904_111906 --once
"""

from __future__ import annotations

import argparse
import csv
import io
import math
import os
import socket
import struct
import threading
import time

from hlvtd_client import CTRL, FRAME_SIZE, TL_STATE, parse_frame
import sim_world
import stoplines as SL

FRAME_DT = 0.05             # VTD는 20.0Hz 고정 스텝


def load_frames(path: str) -> list[bytes]:
    """frames.bin 또는 그것을 담은 세션 디렉터리에서 프레임을 읽는다."""
    if os.path.isdir(path):
        path = os.path.join(path, 'frames.bin')
    with open(path, 'rb') as fh:
        blob = fh.read()

    n, rest = divmod(len(blob), FRAME_SIZE)
    if n == 0:
        raise SystemExit(f'{path}: 온전한 프레임이 없다 ({len(blob)}B)')
    if rest:
        print(f'[replay] 경고: 끝에 {rest}B가 남는다 (중간에 끊긴 녹화). 버린다')
    print(f'[replay] {path}: {n}프레임 ({n * FRAME_DT:.1f}초 분량)')
    return [blob[i * FRAME_SIZE:(i + 1) * FRAME_SIZE] for i in range(n)]


class ControlStats:
    """수신한 9바이트 제어 명령.

    기본 재생에서는 통계만 낸다. --sim에서는 last가 그대로 차량 입력이 된다.
    """

    def __init__(self):
        self.count = 0
        self.last = (0.0, 0.0, 0)
        self.bad = 0


def read_control(conn: socket.socket, stats: ControlStats,
                 alive: threading.Event) -> None:
    buf = bytearray()
    while alive.is_set():
        try:
            chunk = conn.recv(4096)
        except OSError:
            break
        if not chunk:
            break
        buf.extend(chunk)
        while len(buf) >= CTRL.size:
            try:
                stats.last = CTRL.unpack_from(buf, 0)
                stats.count += 1
            except struct.error:
                stats.bad += 1
            del buf[:CTRL.size]
    alive.clear()


# ----------------------------------------------------------------------
# 개루프 — 녹화 프레임 그대로

def serve_one(conn: socket.socket, frames: list[bytes], args) -> None:
    stats = ControlStats()
    alive = threading.Event()
    alive.set()
    threading.Thread(target=read_control, args=(conn, stats, alive),
                     daemon=True).start()

    period = FRAME_DT / max(0.01, args.rate)
    i = min(args.start, len(frames) - 1)
    loops = 0
    sent = 0
    t0 = time.monotonic()
    next_t = t0
    last_print = 0.0

    while alive.is_set():
        if i >= len(frames):
            loops += 1
            if not args.loop:
                print(f'[replay] 끝까지 재생 완료 ({sent}프레임). 연결을 닫는다')
                break
            i = 0
        try:
            conn.sendall(frames[i])
        except OSError:
            print('[replay] 클라이언트가 끊었다')
            break
        i += 1
        sent += 1

        now = time.monotonic()
        if now - last_print >= 1.0:
            last_print = now
            f = parse_frame(frames[i - 1])
            steer, accel, sig = stats.last
            print(f'[replay] {i:5d}/{len(frames)} (loop {loops}) '
                  f'ego({f.ego_x:8.2f},{f.ego_y:9.2f}) obj={len(f.objects):2d} '
                  f'신호={TL_STATE.get(f.tl_state, f.tl_state)} | '
                  f'제어 수신 {stats.count:5d}개 '
                  f'steer={steer:+.3f} accel={accel:+.2f} sig={sig}')

        # 절대 시각으로 스케줄해 드리프트를 막는다
        next_t += period
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.monotonic()

    alive.clear()
    dur = time.monotonic() - t0
    print(f'[replay] 세션 종료: {sent}프레임 {dur:.1f}초 '
          f'({sent / dur if dur else 0:.1f} fps 송신), '
          f'제어 {stats.count}개 수신 (불량 {stats.bad})')
    try:
        conn.close()
    except OSError:
        pass


# ----------------------------------------------------------------------
# 폐루프 — 받은 CTRL로 ego를 적분

def build_sim(args):
    """--sim에 필요한 것을 한 번만 만든다. (path, stoplines, SignalSource, track)"""
    from pure_pursuit import load_path

    segs = args.segments
    if not segs:
        segs = []
        with io.open(args.path, newline='', encoding='utf-8') as fh:
            for row in csv.DictReader(fh):
                if row['segment_id'] not in segs:
                    segs.append(row['segment_id'])
    path, _bounds = load_path(args.path, segs)

    ev = args.events or SL.default_events_path(args.path)
    if not ev:
        raise SystemExit('--sim에는 정지선 이벤트 CSV가 필요하다 (--events)')
    sl_list, warn = SL.load_for_path(path, ev)
    for w in warn:
        print(f'[sim] *** {w} ***')

    ctrls = sorted({min(x.ctrl) for x in sl_list if x.signalized})
    sched = sim_world.TLSchedule.from_session(args.tl_from, ctrls)
    sched.force = args.tl_force
    if args.tl_seed is not None:
        sched.randomize(args.tl_seed)
    src = sim_world.SignalSource(sl_list, sched, acquire_m=args.tl_acquire)

    track = sim_world.ObjectTrack.from_session(
        args.objects_from or args.tl_from, path, args.objects)

    # 합성 정지 객체. 녹화 로그에는 경로를 막는 정지 차량이 하나도 없어
    # 우회를 검증할 방법이 이것뿐이다.
    injected = [sim_world.Injected.parse(spec, path,
                                         sim_world.INJECT_ID_BASE + k)
                for k, spec in enumerate(args.inject_object or ())]

    print(f'[sim] 경로 {path.length:.1f}m ({len(segs)}구간), '
          f'정지선 {len(sl_list)}개 중 신호 {len(src.sl)}개')
    print('[sim] 신호 위상표'
          + (f' (강제 {args.tl_force})' if args.tl_force else ''))
    for line in sched.describe():
        print('[sim]' + line)
    print(f'[sim] 객체 재생 모드={track.mode} 프레임={len(track.blocks)}')
    for o in injected:
        i = path.nearest(o.x, o.y)
        print(f'[sim] 주입 id={o.id} s={path.s[i]:.1f}m '
              f'({o.x:.1f},{o.y:.1f}) '
              f'{o.length:.2f}x{o.width:.2f}x{o.height:.2f} v={o.speed:.1f}')
    return path, sl_list, src, track, injected


def serve_sim(conn: socket.socket, ctx, args) -> None:
    path, _sl_list, src, track, injected = ctx
    stats = ControlStats()
    alive = threading.Event()
    alive.set()
    threading.Thread(target=read_control, args=(conn, stats, alive),
                     daemon=True).start()

    hx, hy = path.pts[0]
    nx, ny = path.pts[min(5, len(path.pts) - 1)]
    ego = sim_world.EgoSim(hx, hy, math.atan2(ny - hy, nx - hx), args.wheelbase)

    period = FRAME_DT / max(0.01, args.rate)
    idx = 0
    n = 0
    next_t = time.monotonic()
    last_print = 0.0
    sim_t = 0.0
    done_since = None

    while alive.is_set():
        idx = path.nearest(ego.x, ego.y, start=idx)
        s_ego = path.s[idx]
        tl_id, tl_state = src.sample(s_ego, sim_t)
        blob = sim_world.overlay(track.at(s_ego, sim_t), injected, ego)
        try:
            conn.sendall(sim_world.pack_frame(ego, blob, tl_id, tl_state))
        except OSError:
            print('[sim] 클라이언트가 끊었다')
            break
        n += 1

        # 다음 스텝. 제어 스레드가 갱신한 최신 명령을 쓴다
        steer, accel, _sig = stats.last
        ego.step(steer, accel, FRAME_DT)
        sim_t += FRAME_DT

        if path.remaining(idx) < 1.5 and ego.v < 0.3:
            if done_since is None:
                done_since = sim_t
                print(f'[sim] 경로 끝 도달 ― sim {sim_t:.1f}초, {s_ego:.1f}m')
            elif sim_t - done_since > 2.0:
                print('[sim] 완주. 연결을 닫는다')
                break
        if sim_t > args.sim_timeout:
            print(f'[sim] 제한시간 {args.sim_timeout:.0f}초 초과 ― 중단')
            break

        now = time.monotonic()
        if now - last_print >= 1.0:
            last_print = now
            print(f'[sim] t={sim_t:6.1f}s s={s_ego:7.1f}/{path.length:.1f}m '
                  f'v={ego.v * 3.6:5.1f}km/h '
                  f'신호 id={tl_id:4d} {TL_STATE.get(tl_state, tl_state):10s} | '
                  f'제어 {stats.count:5d}개 steer={steer:+.3f} accel={accel:+.2f}')

        next_t += period
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.monotonic()

    alive.clear()
    print(f'[sim] 세션 종료: {n}프레임 sim {sim_t:.1f}초, '
          f'주행 {path.s[idx]:.1f}m, 제어 {stats.count}개 수신 (불량 {stats.bad})')
    try:
        conn.close()
    except OSError:
        pass


# ----------------------------------------------------------------------

def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description='frames.bin 재생 서버 (가짜 VTD)')
    ap.add_argument('--frames', default=os.path.normpath(
        os.path.join(here, '..', 'archive', 'frames.bin')),
        help='frames.bin 경로 또는 세션 디렉터리')
    ap.add_argument('--bind', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=9910)
    ap.add_argument('--rate', type=float, default=1.0, help='재생 배속')
    ap.add_argument('--start', type=int, default=0, help='시작 프레임 번호')
    ap.add_argument('--loop', action='store_true', help='끝나면 처음부터 반복')
    ap.add_argument('--once', action='store_true',
                    help='클라이언트 하나만 받고 종료')

    sim = ap.add_argument_group('--sim (폐루프)')
    sim.add_argument('--sim', action='store_true',
                     help='받은 CTRL로 ego를 적분하는 폐루프 모드')
    sim.add_argument('--path', default='',
                     help='주행할 waypoint CSV (route_path_*.csv)')
    sim.add_argument('--segment', '--segments', dest='segments', default='',
                     help='구간. 생략하면 파일에 나온 순서 전부')
    sim.add_argument('--events', default='',
                     help='정지선 이벤트 CSV. 생략하면 --path 옆의 _events.csv')
    sim.add_argument('--tl-from', default='',
                     help='신호 위상표를 뽑을 세션 디렉터리')
    sim.add_argument('--tl-seed', type=int, default=None,
                     help='ctrl별 위상 오프셋을 이 시드로 흔든다')
    sim.add_argument('--tl-force', choices=('red', 'green'), default='',
                     help='모든 신호를 고정한다 (최악 시험)')
    sim.add_argument('--tl-acquire', type=float,
                     default=sim_world.DEFAULT_ACQUIRE_M,
                     help='신호를 보고하기 시작하는 거리 [m]')
    sim.add_argument('--inject-object', action='append', metavar='SPEC',
                     help='경로 위에 합성 정지 객체를 놓는다. 반복 지정 가능. '
                          's=250,lat=0,l=4.39,w=1.81,h=1.35,v=0[,id=7] '
                          '(s=경로 호길이, lat=부호 횡거리 좌가 양수)')
    sim.add_argument('--objects', choices=('s', 't', 'none'), default='s',
                     help='objects[] 재생 색인: s=진행거리 t=시간 none=없음')
    sim.add_argument('--objects-from', default='',
                     help='객체를 뽑을 세션. 생략하면 --tl-from과 같다')
    sim.add_argument('--wheelbase', type=float, default=sim_world.WHEELBASE)
    sim.add_argument('--sim-timeout', type=float, default=400.0,
                     help='sim 시간 상한 [s]')

    args = ap.parse_args()
    args.segments = [x.strip() for x in args.segments.split(',') if x.strip()]

    if args.sim:
        if not args.path:
            raise SystemExit('--sim에는 --path가 필요하다')
        ctx = build_sim(args)
        frames = None
    else:
        ctx = None
        frames = load_frames(args.frames)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.bind, args.port))
    srv.listen(1)
    tag = 'sim' if args.sim else 'replay'
    print(f'[{tag}] {args.bind}:{args.port} 대기 중  '
          f'({args.rate}배속{", 반복" if args.loop else ""})  Ctrl+C로 종료')

    try:
        while True:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f'[{tag}] 접속: {addr[0]}:{addr[1]}')
            if args.sim:
                serve_sim(conn, ctx, args)
            else:
                serve_one(conn, frames, args)
            if args.once:
                break
    except KeyboardInterrupt:
        print('\n[replay] 종료')
    finally:
        srv.close()


if __name__ == '__main__':
    main()
