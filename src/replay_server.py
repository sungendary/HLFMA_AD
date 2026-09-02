#!/usr/bin/env python3
"""녹화한 frames.bin을 9910에서 다시 흘려보내는 재생 서버.

VTD 없이 수신·기록·이벤트 검출 경로를 통째로 시험하기 위한 것이다.
사전테스트에서 녹화한 frames.bin이 그대로 집에서의 테스트 벤치가 된다 —
실제 코스의 신호등 전이와 실제 NPC가 담긴 데이터로 로직을 개발할 수 있다.

    python replay_server.py --frames ../archive/frames.bin --loop
    python replay_server.py --frames ../logs/pp_20260903_101500 --rate 2

    (다른 창에서)
    python pure_pursuit.py --segment p1_to_p2 --host 127.0.0.1

**폐루프가 아니다.** 받은 제어는 통계만 내고 버린다. 보낸 조향이 차량 거동에
반영되지 않으므로 제어 로직 자체의 품질은 이걸로 판단할 수 없다.
확인할 수 있는 것은 기록·파싱·이벤트 검출 경로뿐이다.
"""

from __future__ import annotations

import argparse
import os
import socket
import struct
import threading
import time

from hlvtd_client import CTRL, FRAME_SIZE, TL_STATE, parse_frame

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
    """수신한 9바이트 제어 명령. 폐루프가 아니므로 통계만 낸다."""

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
    args = ap.parse_args()

    frames = load_frames(args.frames)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.bind, args.port))
    srv.listen(1)
    print(f'[replay] {args.bind}:{args.port} 대기 중  '
          f'({args.rate}배속{", 반복" if args.loop else ""})  Ctrl+C로 종료')

    try:
        while True:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f'[replay] 접속: {addr[0]}:{addr[1]}')
            serve_one(conn, frames, args)
            if args.once:
                break
    except KeyboardInterrupt:
        print('\n[replay] 종료')
    finally:
        srv.close()


if __name__ == '__main__':
    main()
