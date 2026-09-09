#!/usr/bin/env python3
"""폐루프 시뮬레이션 세계 ― replay_server --sim이 쓴다.

개루프 재생(frames.bin 그대로 흘리기)으로는 "적색 통과가 몇 번으로 줄었나"를
잴 수 없다. 내가 제동을 걸어도 녹화된 ego는 그대로 정지선을 지나가기 때문이다.
여기서는 **받은 CTRL로 ego를 직접 적분한다.** 그래서 정지->유지->녹색출발이
실제로 일어나고 위반 횟수가 진짜 숫자가 된다.

세 부분으로 되어 있다.

  EgoSim        후륜축 자전거 모델. 9910 ego 좌표계와 같은 기준점이다
  TLSchedule    로그에서 뽑은 ctrl별 위상표. 어느 ctrl을 보고할지도 실측 거동을
                흉내낸다 (획득거리 안이면 그 ctrl, 정지선을 넘으면 NONE)
  ObjectTrack   녹화 프레임의 objects[] 블록을 그대로 재생한다. 지금은 신호만
                쓰지만 9/9 충돌회피·방향지시등이 바로 필요로 한다

신호 위상을 로그 시간축 그대로 재생하지 않는 이유: 우리가 정지하면 시간이
흐르는 동안 녹화 쪽 ego는 다음 교차로로 가 버려 대응이 깨진다. 그래서 ctrl별
주기표로 바꿔 재생한다. 관측되지 않은 위상은 기본값으로 채운다.
"""

from __future__ import annotations

import csv
import io
import math
import os
import random
import struct

from hlvtd_client import EGO, OBJ, TL, FRAME_SIZE, OBJECT_SLOTS, TL_STATE

NONE, RED, YELLOW, GREEN, LEFT, GREEN_LEFT, FLASHING = 0, 1, 2, 3, 4, 5, 6

FRAME_DT = 0.05
WHEELBASE = 2.95

# 관측이 잘려 주기를 못 구한 위상의 기본값 [s]
DEFAULT_GREEN_S = 25.0
DEFAULT_YELLOW_S = 3.0          # 실측 확정: YELLOW -> RED는 정확히 3초
DEFAULT_RED_S = 25.0

DEFAULT_ACQUIRE_M = 120.0       # 실측 획득거리 65~200m의 중간값
OBJ_BLOCK = OBJ.size * OBJECT_SLOTS
EMPTY_OBJECTS = b'\x00' * OBJ_BLOCK


# ----------------------------------------------------------------------
# ego

class EgoSim:
    """후륜축 자전거 모델. 20Hz로 적분한다."""

    def __init__(self, x: float, y: float, heading: float,
                 wheelbase: float = WHEELBASE):
        self.x, self.y, self.h = x, y, heading
        self.v = 0.0
        self.L = wheelbase

    def step(self, steer: float, accel: float, dt: float = FRAME_DT) -> None:
        # 위치를 먼저 적분한다 (전진 오일러). 20Hz면 오차가 무시할 수준이다
        self.x += self.v * math.cos(self.h) * dt
        self.y += self.v * math.sin(self.h) * dt
        self.h = (self.h + self.v / self.L * math.tan(steer) * dt)
        self.h = (self.h + math.pi) % (2.0 * math.pi) - math.pi
        self.v = max(0.0, self.v + accel * dt)      # 후진은 없다


# ----------------------------------------------------------------------
# 신호

def _runs_from_drive(session: str):
    """drive.csv에서 (ctrl_id, state, 지속시간, 잘렸는지) 구간을 뽑는다."""
    path = os.path.join(session, 'drive.csv')
    rows = []
    with io.open(path, newline='', encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append((float(r['t']), int(r['tl_id']), int(r['tl_state'])))
            except (ValueError, TypeError, KeyError):
                continue
    runs = []
    cur = None
    for t, tid, st in rows:
        key = (tid, st)
        if cur is None or cur[0] != key:
            if cur is not None:
                runs.append((cur[0][0], cur[0][1], t - cur[1]))
            cur = (key, t)
    if cur is not None and rows:
        runs.append((cur[0][0], cur[0][1], rows[-1][0] - cur[1]))
    return runs


class TLSchedule:
    """ctrl별 주기표. state_at(ctrl, t)로 상태를 낸다."""

    def __init__(self, cycles: dict, offsets: dict | None = None,
                 force: str = ''):
        self.cycles = cycles                 # ctrl -> [(state, dur), ...]
        self.offsets = offsets or {}
        self.force = force

    # -- 생성 -----------------------------------------------------------
    @classmethod
    def from_session(cls, session: str, ctrls) -> 'TLSchedule':
        """로그에서 ctrl별 주기를 뽑는다. 못 뽑은 ctrl은 기본 주기를 쓴다.

        구간의 첫/마지막은 우리가 그 신호를 보기 시작한/그만 본 시점이라
        잘려 있다. 그래서 **중간 구간만** 지속시간의 근거로 삼는다.
        """
        dur = {}        # (ctrl, state) -> [지속시간]
        green_of = {}   # ctrl -> 관측된 녹색 계열 state
        runs = _runs_from_drive(session) if session else []
        # 같은 ctrl의 연속 구간만 모아 중간 여부를 판정한다
        by_ctrl = {}
        for tid, st, d in runs:
            if tid == 0 or st == NONE:
                continue
            by_ctrl.setdefault(tid, []).append((st, d))
        for tid, seq in by_ctrl.items():
            for k, (st, d) in enumerate(seq):
                if st in (GREEN, GREEN_LEFT):
                    green_of[tid] = st
                if 0 < k < len(seq) - 1:        # 중간 구간만 온전하다
                    dur.setdefault((tid, st), []).append(d)

        def pick(tid, st, default):
            vals = sorted(dur.get((tid, st), []))
            if not vals:
                return default
            return vals[len(vals) // 2]

        cycles = {}
        for c in ctrls:
            g = green_of.get(c, GREEN_LEFT)
            cycles[c] = [
                (g, pick(c, g, DEFAULT_GREEN_S)),
                (YELLOW, pick(c, YELLOW, DEFAULT_YELLOW_S)),
                (RED, pick(c, RED, DEFAULT_RED_S)),
            ]
        return cls(cycles)

    def randomize(self, seed: int) -> None:
        """ctrl마다 위상 오프셋을 흔든다. 같은 코스를 다른 신호로 시험한다."""
        rng = random.Random(seed)
        for c, cyc in self.cycles.items():
            total = sum(d for _s, d in cyc)
            self.offsets[c] = rng.uniform(0.0, total)

    # -- 조회 -----------------------------------------------------------
    def state_at(self, ctrl: int, t: float) -> int:
        if self.force == 'red':
            return RED
        if self.force == 'green':
            return GREEN_LEFT
        cyc = self.cycles.get(ctrl)
        if not cyc:
            return GREEN_LEFT
        total = sum(d for _s, d in cyc)
        if total <= 0:
            return GREEN_LEFT
        u = (t + self.offsets.get(ctrl, 0.0)) % total
        for st, d in cyc:
            if u < d:
                return st
            u -= d
        return cyc[-1][0]

    def describe(self) -> list:
        out = []
        for c in sorted(self.cycles):
            parts = ' '.join(f'{TL_STATE.get(s, s)}:{d:.0f}s'
                             for s, d in self.cycles[c])
            off = self.offsets.get(c, 0.0)
            out.append(f'  ctrl {c:4d}  {parts}   오프셋 {off:.1f}s')
        return out


class SignalSource:
    """어느 ctrl을 어떤 상태로 보고할지. 실측 거동을 흉내낸다.

    · 다음 신호 정지선까지 거리가 획득거리 이내면 그 ctrl을 보고한다
      (실측 65~200m)
    · **후륜축이 정지선을 넘으면 NONE.** 실측 이탈 지점이 -0.3~+0.4m다
    """

    def __init__(self, stoplines, schedule: TLSchedule,
                 acquire_m: float = DEFAULT_ACQUIRE_M):
        self.sl = [s for s in stoplines if s.signalized]
        self.schedule = schedule
        self.acquire = acquire_m

    def sample(self, s_ego: float, t: float) -> tuple[int, int]:
        for sl in self.sl:
            d = sl.s - s_ego
            if 0.0 <= d <= self.acquire:
                ctrl = min(sl.ctrl)     # 9910은 하나만 준다. 실측과 같은 쪽을 고른다
                return ctrl, self.schedule.state_at(ctrl, t)
        return 0, NONE


# ----------------------------------------------------------------------
# 객체

class ObjectTrack:
    """녹화 프레임의 objects[] 1080B 블록을 그대로 재생한다.

    좌표는 월드계라 변환이 필요 없다. 색인 방식이 둘이다.

      's' ― 녹화 ego의 경로 진행거리로 색인한다. 공간적으로 충실하다.
            우리가 멈추면 객체도 멈추므로 보수적이다 (기본값)
      't' ― 경과 시간으로 색인한다. NPC 움직임은 사실적이지만 우리가 멈추면
            위치 대응이 깨진다
    """

    def __init__(self, blocks: list, keys: list, mode: str = 's'):
        self.blocks = blocks        # list[bytes] 각 1080B
        self.keys = keys            # 오름차순 색인값 (진행거리 또는 시간)
        self.mode = mode

    @classmethod
    def from_session(cls, session: str, path, mode: str = 's') -> 'ObjectTrack':
        if mode == 'none' or not session:
            return cls([], [], 'none')
        bin_path = os.path.join(session, 'frames.bin')
        with open(bin_path, 'rb') as fh:
            blob = fh.read()
        n = len(blob) // FRAME_SIZE
        blocks, keys = [], []
        idx = None
        for i in range(n):
            off = i * FRAME_SIZE
            x, y = EGO.unpack_from(blob, off)[:2]
            blocks.append(blob[off + EGO.size:off + EGO.size + OBJ_BLOCK])
            if mode == 't':
                keys.append(i * FRAME_DT)
            else:
                idx = path.nearest(x, y) if idx is None else path.nearest(x, y, start=idx)
                keys.append(path.s[idx])
        if mode == 's':
            # 진행거리는 단조여야 이분 탐색이 성립한다
            for i in range(1, len(keys)):
                if keys[i] < keys[i - 1]:
                    keys[i] = keys[i - 1]
        return cls(blocks, keys, mode)

    def at(self, s_ego: float, t: float) -> bytes:
        if not self.blocks:
            return EMPTY_OBJECTS
        key = t if self.mode == 't' else s_ego
        lo, hi = 0, len(self.keys) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self.keys[mid] < key:
                lo = mid + 1
            else:
                hi = mid
        return self.blocks[lo]


# ----------------------------------------------------------------------
# 합성 객체 주입

OBJ_RANGE_M = 80.0          # 9910 명세의 수신 반경. 이 밖은 넣지 않는다
INJECT_ID_BASE = 900        # 녹화 id와 겹치지 않게


class Injected:
    """경로 위에 놓는 합성 정지 객체.

    녹화 로그에는 **경로를 막는 정지 차량이 하나도 없다** (사전테스트 6세션
    전부). 우회를 검증하려면 만들어 넣는 수밖에 없다. --objects 재생 블록의
    빈 슬롯에 끼워 넣으므로 재생 자체는 그대로 간다.
    """

    __slots__ = ('id', 'x', 'y', 'heading', 'speed', 'length', 'width',
                 'height')

    def __init__(self, oid, x, y, heading, speed, length, width, height):
        self.id = oid
        self.x, self.y, self.heading = x, y, heading
        self.speed = speed
        self.length, self.width, self.height = length, width, height

    @classmethod
    def parse(cls, spec: str, path, oid: int) -> 'Injected':
        """s=250,lat=0,l=4.39,w=1.81,h=1.35,v=0[,id=7] -> 월드 좌표로 고정.

        s(경로 호길이)와 lat(부호 횡거리, 좌가 양수)을 시작 시 한 번만
        (x,y,heading)으로 바꾼다. 이후 움직이지 않는다.
        """
        kv = {}
        for part in spec.split(','):
            part = part.strip()
            if not part:
                continue
            if '=' not in part:
                raise SystemExit(f'--inject-object 형식 오류: {part!r} '
                                 '(s=250,lat=0,l=4.39,w=1.81,h=1.35,v=0)')
            k, val = part.split('=', 1)
            kv[k.strip()] = float(val)
        if 's' not in kv:
            raise SystemExit(f'--inject-object 에는 s= 가 필요하다: {spec!r}')

        s = kv['s']
        lat = kv.get('lat', 0.0)
        i = 0
        while i < len(path.pts) - 2 and path.s[i + 1] < s:
            i += 1
        ax, ay = path.pts[i]
        bx, by = path.pts[i + 1]
        seg = max(1e-6, path.s[i + 1] - path.s[i])
        u = min(1.0, max(0.0, (s - path.s[i]) / seg))
        hx, hy = ax + u * (bx - ax), ay + u * (by - ay)
        th = math.atan2(by - ay, bx - ax)
        # 부호 규약은 obstacles._project 와 같다 (진행방향 좌측이 양수)
        x = hx - math.sin(th) * lat
        y = hy + math.cos(th) * lat
        return cls(int(kv.get('id', oid)), x, y, th, kv.get('v', 0.0),
                   kv.get('l', 4.39), kv.get('w', 1.81), kv.get('h', 1.35))

    def __repr__(self) -> str:
        return (f'<Injected {self.id} ({self.x:.1f},{self.y:.1f}) '
                f'{self.length:.2f}x{self.width:.2f}x{self.height:.2f} '
                f'v={self.speed:.1f}>')


def overlay(block: bytes, objs: list, ego) -> bytes:
    """1080B 블록의 빈 슬롯(id==0)에 합성 객체를 끼워 넣는다.

    빈 슬롯이 없으면 가장 먼 녹화 객체를 밀어낸다 -- 실제 VTD도 가까운 순
    30개만 주므로 같은 규칙이다.
    """
    if not objs:
        return block
    near = [o for o in objs
            if math.hypot(o.x - ego.x, o.y - ego.y) <= OBJ_RANGE_M]
    if not near:
        return block
    buf = bytearray(block)
    free, far = [], []
    for k in range(OBJECT_SLOTS):
        off = k * OBJ.size
        oid, ox, oy = OBJ.unpack_from(buf, off)[:3]
        if oid == 0:
            free.append(k)
        else:
            far.append((math.hypot(ox - ego.x, oy - ego.y), k))
    far.sort(reverse=True)
    for o in near:
        if free:
            k = free.pop(0)
        elif far:
            k = far.pop(0)[1]
        else:
            break
        OBJ.pack_into(buf, k * OBJ.size, o.id, o.x, o.y, 0.0, o.heading,
                      o.speed, o.length, o.width, o.height)
    return bytes(buf)


# ----------------------------------------------------------------------

def pack_frame(ego: EgoSim, objects: bytes, tl_id: int, tl_state: int) -> bytes:
    """9910 수신 형식 그대로 1109B를 만든다."""
    buf = bytearray(FRAME_SIZE)
    EGO.pack_into(buf, 0, ego.x, ego.y, 0.0, ego.h, 0.0, 0.0)
    buf[EGO.size:EGO.size + OBJ_BLOCK] = objects
    TL.pack_into(buf, EGO.size + OBJ_BLOCK, tl_id, tl_state)
    return bytes(buf)
