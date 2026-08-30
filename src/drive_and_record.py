#!/usr/bin/env python3
"""WASD 수동 주행 + waypoint 녹화.

9910 연결은 하나만 열 수 있으므로 조작과 녹화를 한 프로세스에서 처리한다.
주행하는 동안 ego 위치를 20Hz로 CSV에 기록해 도로망 지도를 만든다.

    python drive_and_record.py --host 192.168.50.11 --segment path01

조작
    W / S   가속 / 감속        A / D   좌 / 우 조향
    Q / E   좌 / 우 지시등      Space   비상감속
    R       녹화 시작 / 일시정지
    Esc     종료

출력: waypoints.csv  (segment_id, seq, x, y, heading)
같은 파일에 계속 이어 붙으므로 구간마다 --segment 이름만 바꿔가며 실행하면 된다.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import threading
import time
import tkinter as tk

from hlvtd_client import (
    Client, SpeedEstimator, detect_respawn,
    TURN_OFF, TURN_LEFT, TURN_RIGHT, STEER_LIMIT, ACCEL_MAX, ACCEL_MIN,
)

STEER_RATE = 0.45           # rad/s, 키를 누르고 있을 때 조향 증가율
STEER_RETURN = 0.8          # rad/s, 키를 뗐을 때 복원율
MIN_POINT_GAP_M = 0.3       # 이 거리 이상 움직였을 때만 점을 기록

SIGNAL_NAME = {TURN_OFF: 'OFF', TURN_LEFT: 'LEFT', TURN_RIGHT: 'RIGHT'}
KEYS = {'w', 'a', 's', 'd', 'q', 'e', 'r', 'space', 'escape'}


class Recorder:
    """waypoint를 CSV에 이어 붙인다."""

    def __init__(self, path: str, segment: str):
        self.path = path
        self.segment = segment
        self.count = 0
        self.enabled = False
        self._last = None
        new_file = not os.path.exists(path)
        self._fh = open(path, 'a', newline='', encoding='utf-8')
        self._writer = csv.writer(self._fh)
        if new_file:
            self._writer.writerow(['segment_id', 'seq', 'x', 'y', 'heading'])

    def add(self, x, y, heading):
        if not self.enabled:
            return
        if self._last is not None:
            gap = math.hypot(x - self._last[0], y - self._last[1])
            if gap < MIN_POINT_GAP_M:
                return
            if gap > 5.0:
                self.enabled = False          # 리스폰 — 녹화 자동 중단
                print(f'\n*** 리스폰 감지 ({gap:.1f}m) — 녹화 중단. '
                      f'이 구간은 버리고 다시 하세요 ***\n')
                return
        self._writer.writerow([self.segment, self.count, f'{x:.3f}', f'{y:.3f}',
                               f'{heading:.5f}'])
        self._last = (x, y)
        self.count += 1
        if self.count % 50 == 0:
            self._fh.flush()


def main() -> None:
    ap = argparse.ArgumentParser(description='수동 주행 + waypoint 녹화')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=9910)
    ap.add_argument('--segment', default='seg01', help='이번에 녹화할 구간 이름')
    ap.add_argument('--out', default='waypoints.csv')
    ap.add_argument('--rate', type=float, default=20.0)
    args = ap.parse_args()

    client = Client(args.host, args.port)
    print(f'[drive] {args.host}:{args.port} 연결 시도...')
    client.connect()
    print('[drive] 연결됨')

    recorder = Recorder(args.out, args.segment)
    speed = SpeedEstimator()

    state = {'frame': None, 'respawn': 0, 'alive': True}
    lock = threading.Lock()

    def receiver() -> None:
        prev = None
        while state['alive']:
            frame = client.recv_frame()
            if frame is None:
                if state['alive']:
                    print('[drive] 연결 끊김. 재연결...')
                    try:
                        client.connect()
                    except OSError:
                        break
                    prev = None
                continue
            speed.update(frame.ego_x, frame.ego_y)
            with lock:
                if detect_respawn(frame, prev):
                    state['respawn'] += 1
                state['frame'] = frame
            prev = frame

    threading.Thread(target=receiver, daemon=True).start()

    pressed: set[str] = set()
    ctrl = {'steer': 0.0, 'signal': TURN_OFF, 'last_tick': time.monotonic()}
    interval_ms = max(16, int(1000.0 / args.rate))

    root = tk.Tk()
    root.title('HLVTD Drive & Record')
    root.geometry('560x230')

    lbl_ego = tk.Label(root, text='', font=('Consolas', 11), justify='left')
    lbl_ego.pack(pady=(12, 4))
    lbl_ctrl = tk.Label(root, text='', font=('Consolas', 10), justify='left')
    lbl_ctrl.pack()
    lbl_rec = tk.Label(root, text='', font=('Consolas', 12, 'bold'))
    lbl_rec.pack(pady=6)
    tk.Label(root, text='W/S 가속·감속   A/D 조향   Q/E 지시등   Space 제동\n'
                        'R 녹화 토글   Esc 종료',
             font=('Consolas', 9), fg='gray').pack(pady=8)

    def close() -> None:
        state['alive'] = False
        client.close()
        recorder.close()
        print(f'[drive] 저장 완료: {args.out}  '
              f'구간={args.segment}  점={recorder.count}개')
        root.destroy()

    def on_press(event) -> None:
        key = event.keysym.lower()
        if key not in KEYS:
            return
        if key == 'escape':
            close()
            return
        if key == 'r' and 'r' not in pressed:
            recorder.enabled = not recorder.enabled
        elif key == 'q' and 'q' not in pressed:
            ctrl['signal'] = TURN_OFF if ctrl['signal'] == TURN_LEFT else TURN_LEFT
        elif key == 'e' and 'e' not in pressed:
            ctrl['signal'] = TURN_OFF if ctrl['signal'] == TURN_RIGHT else TURN_RIGHT
        pressed.add(key)

    def on_release(event) -> None:
        pressed.discard(event.keysym.lower())

    root.bind_all('<KeyPress>', on_press)
    root.bind_all('<KeyRelease>', on_release)
    root.bind('<FocusOut>', lambda _e: pressed.clear())
    root.focus_force()

    def tick() -> None:
        now = time.monotonic()
        dt = max(0.001, min(now - ctrl['last_tick'], 0.5))
        ctrl['last_tick'] = now

        accel = 0.0
        if 'space' in pressed or 's' in pressed:
            accel = ACCEL_MIN
        elif 'w' in pressed:
            accel = ACCEL_MAX

        # 화면 기준 A=좌, D=우. 프로토콜은 양수가 좌회전이다.
        steer = ctrl['steer']
        if 'a' in pressed and 'd' not in pressed:
            steer += STEER_RATE * dt
        elif 'd' in pressed and 'a' not in pressed:
            steer -= STEER_RATE * dt
        elif steer > 0:
            steer = max(0.0, steer - STEER_RETURN * dt)
        elif steer < 0:
            steer = min(0.0, steer + STEER_RETURN * dt)
        ctrl['steer'] = max(-STEER_LIMIT, min(STEER_LIMIT, steer))

        if not client.send_control(ctrl['steer'], accel, ctrl['signal']):
            print('[drive] 송신 실패')
            close()
            return

        with lock:
            frame = state['frame']
            respawns = state['respawn']

        if frame is not None:
            recorder.add(frame.ego_x, frame.ego_y, frame.ego_heading)
            lbl_ego.config(
                text=f'x={frame.ego_x:9.2f}  y={frame.ego_y:9.2f}  '
                     f'heading={math.degrees(frame.ego_heading):+7.1f}°\n'
                     f'속도={speed.kmh:5.1f}km/h   객체={len(frame.objects):2d}개   '
                     f'신호={frame.tl_name}({frame.tl_id})   리스폰={respawns}')
        else:
            lbl_ego.config(text='수신 대기 중...')

        lbl_ctrl.config(text=f'steering={ctrl["steer"]:+.3f}rad  '
                             f'accel={accel:+.1f}m/s²  '
                             f'signal={SIGNAL_NAME[ctrl["signal"]]}')
        if recorder.enabled:
            lbl_rec.config(text=f'● 녹화 중  [{args.segment}]  {recorder.count}점',
                           fg='red')
        else:
            lbl_rec.config(text=f'○ 정지  [{args.segment}]  {recorder.count}점  '
                                f'— R키로 시작', fg='gray')
        root.after(interval_ms, tick)

    root.protocol('WM_DELETE_WINDOW', close)
    root.after(interval_ms, tick)
    root.mainloop()


if __name__ == '__main__':
    main()
