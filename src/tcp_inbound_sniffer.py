#!/usr/bin/env python3
"""HLVTD 9910 수신 데이터 형식 확인용 스니퍼.

VTD가 참가자에게 보내는 데이터가 JSON 텍스트인지 바이너리 구조체인지,
한 프레임이 몇 바이트인지, 몇 Hz로 오는지를 눈으로 확인한다.

제어 명령은 정지(0,0,OFF)만 20Hz로 계속 보낸다. 차는 움직이지 않는다.

실행:
    python3 tcp_inbound_sniffer.py                    # 로컬 VTD
    python3 tcp_inbound_sniffer.py --host 192.168.50.11
    python3 tcp_inbound_sniffer.py --dump frames.bin  # 원본 바이트 저장
"""

import argparse
import socket
import struct
import sys
import threading
import time

CTRL = struct.Struct('<ffB')          # steering, targetAccel, turnSignal
STOP = CTRL.pack(0.0, 0.0, 0)


def hexdump(data: bytes, limit: int = 256) -> None:
    """16바이트씩 hex + ASCII 출력."""
    for off in range(0, min(len(data), limit), 16):
        row = data[off:off + 16]
        hexs = ' '.join(f'{b:02x}' for b in row).ljust(47)
        text = ''.join(chr(b) if 32 <= b < 127 else '.' for b in row)
        print(f'  {off:04x}  {hexs}  |{text}|')
    if len(data) > limit:
        print(f'  ... (총 {len(data)}B 중 {limit}B만 표시)')


def looks_like_text(data: bytes) -> bool:
    if not data:
        return False
    printable = sum(1 for b in data[:512] if 32 <= b < 127 or b in (9, 10, 13))
    return printable / min(len(data), 512) > 0.85


def sender(sock: socket.socket, stop_flag: threading.Event) -> None:
    """제어 채널을 살려두기 위해 정지 명령을 20Hz로 계속 전송."""
    while not stop_flag.is_set():
        try:
            sock.sendall(STOP)
        except OSError:
            return
        time.sleep(0.05)


def main() -> int:
    ap = argparse.ArgumentParser(description='HLVTD 9910 inbound sniffer')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=9910)
    ap.add_argument('--seconds', type=float, default=10.0, help='관찰 시간')
    ap.add_argument('--dump', help='원본 바이트를 저장할 파일')
    args = ap.parse_args()

    print(f'[sniffer] {args.host}:{args.port} 연결 시도...')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    try:
        sock.connect((args.host, args.port))
    except OSError as exc:
        print(f'[sniffer] 연결 실패: {exc}', file=sys.stderr)
        print('  VTD가 실행 중이고 시나리오가 재생 상태인지 확인하세요.', file=sys.stderr)
        return 1
    print('[sniffer] 연결됨. 정지 명령을 20Hz로 송신하며 수신 데이터를 관찰합니다.\n')

    stop_flag = threading.Event()
    threading.Thread(target=sender, args=(sock, stop_flag), daemon=True).start()

    dump = open(args.dump, 'wb') if args.dump else None
    sock.settimeout(1.0)

    chunks = 0
    total = 0
    arrivals = []
    t_end = time.monotonic() + args.seconds

    try:
        while time.monotonic() < t_end:
            try:
                data = sock.recv(65536)
            except socket.timeout:
                continue
            if not data:
                print('[sniffer] 서버가 연결을 닫았습니다.')
                break

            chunks += 1
            total += len(data)
            arrivals.append(time.monotonic())
            if dump:
                dump.write(data)

            if chunks <= 3:
                kind = 'TEXT(JSON?)' if looks_like_text(data) else 'BINARY'
                print(f'--- chunk #{chunks}  {len(data)}B  판정={kind} ---')
                hexdump(data)
                if looks_like_text(data):
                    preview = data[:400].decode('utf-8', 'replace')
                    print(f'  텍스트 미리보기:\n  {preview}\n')
                else:
                    print()
    finally:
        stop_flag.set()
        sock.close()
        if dump:
            dump.close()

    print('=' * 60)
    if chunks == 0:
        print('수신 데이터 없음 — VTD가 참가자에게 아무것도 보내지 않거나,')
        print('시나리오가 재생 중이 아닐 수 있습니다.')
        return 0

    sizes = {}
    print(f'총 {chunks}개 청크 / {total}B 수신')
    if len(arrivals) > 1:
        span = arrivals[-1] - arrivals[0]
        print(f'평균 수신율: {(len(arrivals) - 1) / span:.1f} Hz  (20Hz 예상)')
    print(f'평균 청크 크기: {total / chunks:.1f}B')
    if args.dump:
        print(f'원본 저장 완료: {args.dump}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
