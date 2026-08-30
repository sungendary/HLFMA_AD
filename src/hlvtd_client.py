#!/usr/bin/env python3
"""HLVTD 9910 프로토콜 — 파서와 클라이언트.

수신 프레임 1109바이트 고정, 20Hz, 헤더/구분자 없음.
    Ego            24B  = 6 x float32
    Object x 30  1080B  = 30 x (uint32 + 8 x float32)
    TrafficLight    5B  = int32 + uint8
송신 제어 9바이트.
    steering float32 [rad], targetAccel float32 [m/s^2], turnSignal uint8

단독 실행하면 실시간 모니터로 동작한다.
    python hlvtd_client.py --host 192.168.50.11
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import time
from dataclasses import dataclass, field

EGO = struct.Struct('<6f')          # 24 B
OBJ = struct.Struct('<I8f')         # 36 B
TL = struct.Struct('<iB')           # 5 B
CTRL = struct.Struct('<ffB')        # 9 B

OBJECT_SLOTS = 30
FRAME_SIZE = EGO.size + OBJ.size * OBJECT_SLOTS + TL.size    # 1109

TURN_OFF, TURN_LEFT, TURN_RIGHT = 0, 1, 2

TL_STATE = {
    0: 'NONE', 1: 'RED', 2: 'YELLOW', 3: 'GREEN',
    5: 'GREEN+LEFT', 4: 'LEFT', 6: 'FLASHING',
}

# 제어 명령 실용 범위 (주최측 예제 기준)
STEER_LIMIT = 0.35          # rad, 양수 = 좌회전
ACCEL_MAX = 3.0             # m/s^2
ACCEL_MIN = -6.0

# 리스폰 감지 임계값. 20Hz에서 한 프레임에 이만큼 움직이는 것은 불가능하다.
RESPAWN_JUMP_M = 5.0


@dataclass
class Obj:
    id: int
    x: float
    y: float
    z: float
    heading: float
    speed: float
    length: float
    width: float
    height: float

    @property
    def is_pedestrian(self) -> bool:
        """치수로 보행자를 판별한다. objects[]에 종류 필드가 없어서 필요하다.

        보행자 약 0.5 x 0.5 x 1.7m, 승용차 약 4.8 x 1.9 x 1.5m.
        실측 로그로 임계값을 조정할 것.
        """
        return self.length < 1.2 and self.height < 2.2

    def distance_from(self, x: float, y: float) -> float:
        return math.hypot(self.x - x, self.y - y)


@dataclass
class Frame:
    ego_x: float
    ego_y: float
    ego_z: float
    ego_heading: float
    ego_pitch: float
    ego_roll: float
    objects: list = field(default_factory=list)      # id != 0 인 유효 슬롯만
    tl_id: int = 0
    tl_state: int = 0

    @property
    def tl_name(self) -> str:
        return TL_STATE.get(self.tl_state, f'UNKNOWN({self.tl_state})')


def parse_frame(buf: bytes) -> Frame:
    if len(buf) != FRAME_SIZE:
        raise ValueError(f'프레임 크기가 {len(buf)}B (기대 {FRAME_SIZE}B)')

    x, y, z, h, p, r = EGO.unpack_from(buf, 0)
    frame = Frame(x, y, z, h, p, r)

    off = EGO.size
    for _ in range(OBJECT_SLOTS):
        oid, ox, oy, oz, oh, spd, ln, wd, ht = OBJ.unpack_from(buf, off)
        off += OBJ.size
        if oid != 0:                      # id == 0 인 슬롯은 무효
            frame.objects.append(Obj(oid, ox, oy, oz, oh, spd, ln, wd, ht))

    frame.tl_id, frame.tl_state = TL.unpack_from(buf, off)
    return frame


def pack_control(steering: float, accel: float, turn_signal: int = TURN_OFF) -> bytes:
    steering = max(-STEER_LIMIT, min(STEER_LIMIT, steering))
    accel = max(ACCEL_MIN, min(ACCEL_MAX, accel))
    return CTRL.pack(steering, accel, turn_signal)


class Client:
    """9910 연결. 프레임 단위 수신과 제어 송신을 담당한다."""

    def __init__(self, host: str = '127.0.0.1', port: int = 9910):
        self.host = host
        self.port = port
        self.sock: socket.socket | None = None
        self._buf = bytearray()

    def connect(self, retry_forever: bool = True) -> None:
        while True:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.connect((self.host, self.port))
                self.sock = s
                self._buf.clear()
                return
            except OSError:
                if not retry_forever:
                    raise
                print(f'[client] {self.host}:{self.port} 연결 대기 중...')
                time.sleep(1.0)

    def recv_frame(self) -> Frame | None:
        """한 프레임을 받는다. 연결이 끊기면 None."""
        while len(self._buf) < FRAME_SIZE:
            try:
                chunk = self.sock.recv(65536)
            except OSError:
                return None
            if not chunk:
                return None
            self._buf.extend(chunk)

        raw = bytes(self._buf[:FRAME_SIZE])
        del self._buf[:FRAME_SIZE]
        return parse_frame(raw)

    def send_control(self, steering: float, accel: float,
                     turn_signal: int = TURN_OFF) -> bool:
        try:
            self.sock.sendall(pack_control(steering, accel, turn_signal))
            return True
        except OSError:
            return False

    def close(self) -> None:
        if self.sock:
            try:
                self.sock.sendall(pack_control(0.0, ACCEL_MIN, TURN_OFF))
            except OSError:
                pass
            self.sock.close()
            self.sock = None


class SpeedEstimator:
    """egoSpeed가 전송되지 않으므로 위치 차분으로 추정한다.

    20Hz에서 30km/h면 프레임당 약 0.42m 이동. 노이즈가 있어 저역통과를 건다.
    리스폰으로 위치가 순간이동하면 속도 추정을 초기화한다.
    """

    def __init__(self, alpha: float = 0.3):
        self.alpha = alpha
        self.speed = 0.0
        self._prev = None
        self._prev_t = None

    def update(self, x: float, y: float, now: float | None = None) -> float:
        now = now if now is not None else time.monotonic()
        if self._prev is None:
            self._prev, self._prev_t = (x, y), now
            return 0.0

        dt = now - self._prev_t
        if dt <= 1e-4:
            return self.speed

        d = math.hypot(x - self._prev[0], y - self._prev[1])
        self._prev, self._prev_t = (x, y), now

        if d > RESPAWN_JUMP_M:            # 리스폰 — 추정 초기화
            self.speed = 0.0
            return 0.0

        raw = d / dt
        self.speed += self.alpha * (raw - self.speed)
        return self.speed

    @property
    def kmh(self) -> float:
        return self.speed * 3.6


def detect_respawn(frame: Frame, prev: Frame | None) -> bool:
    if prev is None:
        return False
    return math.hypot(frame.ego_x - prev.ego_x, frame.ego_y - prev.ego_y) > RESPAWN_JUMP_M


def _monitor(host: str, port: int) -> None:
    """실시간 모니터. 정지 명령만 보내며 수신 내용을 표시한다."""
    client = Client(host, port)
    client.connect()
    print(f'[monitor] 연결됨. frame={FRAME_SIZE}B  Ctrl+C로 종료\n')

    speed = SpeedEstimator()
    prev = None
    last_print = 0.0

    try:
        while True:
            frame = client.recv_frame()
            if frame is None:
                print('\n[monitor] 연결 끊김. 재연결...')
                client.connect()
                prev = None
                continue

            client.send_control(0.0, 0.0)
            v = speed.update(frame.ego_x, frame.ego_y)

            if detect_respawn(frame, prev):
                print('\n[monitor] *** 리스폰 감지 ***')
            prev = frame

            now = time.monotonic()
            if now - last_print >= 0.5:
                last_print = now
                nearest = min(frame.objects,
                              key=lambda o: o.distance_from(frame.ego_x, frame.ego_y),
                              default=None)
                near = '없음'
                if nearest:
                    kind = '보행자' if nearest.is_pedestrian else '차량'
                    d = nearest.distance_from(frame.ego_x, frame.ego_y)
                    near = f'{kind} id={nearest.id} {d:5.1f}m {nearest.speed * 3.6:5.1f}km/h'
                print(f'ego ({frame.ego_x:9.2f}, {frame.ego_y:9.2f}) '
                      f'h={math.degrees(frame.ego_heading):+7.1f}° '
                      f'v={speed.kmh:5.1f}km/h | '
                      f'obj={len(frame.objects):2d} 최근접 {near} | '
                      f'신호 id={frame.tl_id} {frame.tl_name}')
    except KeyboardInterrupt:
        print('\n[monitor] 종료')
    finally:
        client.close()


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='HLVTD 실시간 모니터')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=9910)
    _monitor(ap.parse_args().host, ap.parse_args().port)
