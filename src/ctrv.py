#!/usr/bin/env python3
"""등속·등회전율(CTRV) 예측 ― ttc.py와 junction_yield.py가 같이 쓴다.

9/12 본대회(pp_20260912_154040)에서 교차로를 좌회전해 들어오는 NPC 행렬을
직선으로 외삽해 놓쳤다. 3초간 헤딩이 -1.22 -> +0.11(약 0.4rad/s)로 돌았는데
직선 외삽은 그 차가 우리 앞을 지나쳐 간다고 계산했다.

9910에는 요레이트가 없다. 트랙의 최근 헤딩 몇 개로 추정한다.

  · dt는 측정값이 아니라 **공칭 0.05s**다. 프레임 도착 간격은 40/40/40/80ms로
    흔들리지만 VTD는 20Hz 고정 스텝이라 프레임 사이 헤딩 변화는 0.05s 분이다
    (hlvtd_client.SpeedEstimator와 같은 이유)
  · 프레임 간 헤딩이 YAW_JUMP 넘게 튀면 이력을 버린다 (id 재사용, 리스폰)
  · |요레이트| < YAW_MIN이면 직선이다. 도로 곡률을 따라가는 완만한 회전까지
    등회전으로 외삽하면 4초 뒤 위치가 도로 밖으로 나간다
  · **누적 회전은 TURN_SAT(90도)에서 멈춘다.** 0.4rad/s를 8초 돌리면 3.2rad이라
    제자리에서 맴도는 궤적이 된다. 교차로 회전은 90도 안팎에서 끝난다
"""

from __future__ import annotations

import math

FRAME_DT = 0.05             # s, VTD 고정 스텝
HIST_N = 5                  # 프레임, 요레이트 추정에 쓰는 헤딩 개수
YAW_MIN = 0.05              # rad/s, 이 아래는 직선
YAW_MAX = 0.8               # rad/s, 상한 (30km/h에서 회전반경 10m)
YAW_JUMP = 0.3              # rad, 프레임 간 이만큼 튀면 다른 객체다 (6rad/s)
YAW_MIN_SPEED = 1.0         # m/s, 이보다 느리면 헤딩이 의미 없다
TURN_SAT = math.pi / 2.0    # rad, 누적 회전 포화


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class HeadingHist:
    """최근 HIST_N개의 헤딩(unwrap). 트랙마다 하나씩 든다."""

    __slots__ = ('h',)

    def __init__(self):
        self.h = []

    def clear(self) -> None:
        self.h = []

    def push(self, heading: float) -> None:
        if self.h:
            d = wrap(heading - self.h[-1])
            if abs(d) > YAW_JUMP:
                self.h = [heading]      # 튐. 이력을 버리고 새로 시작한다
                return
            heading = self.h[-1] + d
        self.h.append(heading)
        if len(self.h) > HIST_N:
            del self.h[0]

    def rate(self, speed: float) -> float:
        """요레이트 [rad/s]. 직선으로 볼 때는 0.0."""
        n = len(self.h)
        if n < 3 or abs(speed) < YAW_MIN_SPEED:
            return 0.0
        w = (self.h[-1] - self.h[0]) / ((n - 1) * FRAME_DT)
        if abs(w) < YAW_MIN:
            return 0.0
        return max(-YAW_MAX, min(YAW_MAX, w))


def point(x: float, y: float, h: float, v: float, w: float, tau: float) -> tuple:
    """τ초 뒤의 (x, y, heading). w == 0이면 직선이다."""
    if w == 0.0:
        return x + v * tau * math.cos(h), y + v * tau * math.sin(h), h
    t_sat = TURN_SAT / abs(w)
    tt = tau if tau < t_sat else t_sat
    a = w * tt
    r = v / w
    px = x + r * (math.sin(h + a) - math.sin(h))
    py = y - r * (math.cos(h + a) - math.cos(h))
    hh = h + a
    if tau > t_sat:
        rest = tau - t_sat
        px += v * rest * math.cos(hh)
        py += v * rest * math.sin(hh)
    return px, py, hh


def points(x: float, y: float, h: float, v: float, w: float,
           n: int, step: float) -> list:
    """τ = 0, step, ..., n*step의 (x, y, heading) 목록."""
    return [point(x, y, h, v, w, k * step) for k in range(n + 1)]


# ----------------------------------------------------------------------
# ego 통과 시간과 판정 시간창 (junction_yield의 양보, ttc의 정지 중 hold가 같이 쓴다)

T_HORIZON_MIN = 4.0         # s
T_HORIZON_MAX = 8.0         # s
T_MARGIN = 1.5              # s, 통과 시간에 얹는 여유
DEPART_ACCEL = 1.5          # m/s^2, 출발 가속 가정


def t_clear(v: float, dist: float, v_pass: float,
            accel: float = DEPART_ACCEL) -> float:
    """속도 v에서 accel로 v_pass까지 가속하며 dist를 지나는 시간."""
    v = max(0.0, v)
    v_pass = max(v_pass, v, 1.0)
    d_acc = (v_pass * v_pass - v * v) / (2.0 * accel)
    if d_acc >= dist:
        return (-v + math.sqrt(v * v + 2.0 * accel * max(0.0, dist))) / accel
    return (v_pass - v) / accel + (dist - d_acc) / v_pass


def horizon(tc: float) -> float:
    """판정 시간창 = max(4, 통과 시간 + 1.5), 상한 8s.

    4s 고정이면 5초 간격 행렬의 틈으로 출발한다 ― 정지 출발로 교차로를 지나는 데
    7초가 넘는다.
    """
    return max(T_HORIZON_MIN, min(T_HORIZON_MAX, tc + T_MARGIN))
