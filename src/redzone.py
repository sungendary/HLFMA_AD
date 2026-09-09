#!/usr/bin/env python3
"""붉은 보호구역 카메라 판정 (평가항목 2).

xodr에는 붉은 노면 정보가 없다(9/6 전수 확인 ― roadMark color는 standard/yellow
둘뿐, material 0건, <surface> 651개 전부 빈 태그, <speed> 0건). 주최측은 "붉은색
노면만 제한속도 구간으로 평가한다. 바닥에 30이라 적혀 있어도 붉지 않으면 50km/h로
주행해도 무방하다"고 했으므로 **8554 RTSP 전방 카메라가 유일한 근거다.**

    rtsp://<VTD_PC_IP>:8554/front     (주최측 실습 매뉴얼·mediamtx.yml paths: front)
    1920x1080 h264 20fps. broadcast는 평가 측정용이라 쓰지 않는다.

numpy / OpenCV / PyAV는 이 PC에서 앱 제어 정책에 막혀 있다. ffmpeg 서브프로세스가
RTSP를 디코드해 **crop + scale한 rawvideo(rgb24)를 stdout 파이프로** 주고, 순수
파이썬이 bytes를 3바이트씩 돌며 붉은 픽셀 비율을 센다. 64x64면 4,096px라
5fps에서 부담이 없다.

--- 구조 (stoplines / obstacles와 같다) ---

- CameraReader   데몬 스레드. ffmpeg를 띄우고 프레임마다 (far, near, seq)를 갱신한다.
                 죽으면 RESTART_BACKOFF_S 뒤 다시 띄운다. **20Hz 루프를 절대 막지
                 않는다** ― 루프는 튜플 하나를 읽을 뿐이다
- RedzonePolicy  매 프레임 update()로 히스테리시스를 돌려 v_limit 하나를 낸다.
                 pure_pursuit이 min()으로 섞는다. --no-redzone으로 끈다
- StubCamera     sim 전용. 경로 위 회랑으로 카메라를 흉내낸다. replay_server에는
                 8554가 없어 구간 내 감속을 볼 방법이 이것뿐이다

--- crop을 두 띠로 나눈 이유 ---

45km/h(12.5m/s)에서 25km/h까지 2.5m/s²로 21.6m가 든다. 차 바로 앞 노면만 보면
붉은 구역에 **들어간 뒤에야** 감속을 시작한다. 그래서 crop(기본 rows 640~1040)을
세로로 반 갈라 **위 절반(far, 멀리)은 진입**, **아래 절반(near, 코앞)은 이탈**에
쓴다. ffmpeg 하나로 "일찍 진입하고 늦게 이탈"이 기하학적으로 보장된다.
평가는 후륜축이 붉은 구역 위일 때 속도로 하고 제한속도 이하 저속은 무감점이므로
이 치우침은 손해가 아니다. 반대(늦게 진입·일찍 이탈)만 감점이다.

--- 9/10 실주행이 고친 것 ---

진입은 **max(far, near) >= ENTER_RATIO 1프레임**이다. 실주행에서 진입이 10.7m 늦어
구간 초반 98.8m를 44km/h로 달렸다. 분리도가 완벽해서(구간 안 far p50 0.35, 밖 0.00)
2프레임 확인이 필요 없고, 한쪽 띠만 붉게 잡히는 프레임이 실제로 있었다.

붉은 노면이 **중간에 끊긴다** (경로 2에서 27~28m, 그리고 정지선-교차로 26m).
그래서 이탈 뒤에도 ZONE_HOLD_M 동안 25km/h를 **유지(홀드)**하고, 홀드 중 다시
붉으면 거리를 리셋한다. 기준이 시간이 아니라 **주행거리**인 이유는 정지선 적색
대기(18초)가 시간 홀드를 그냥 만료시키기 때문이다. 저속은 무감점이라 공짜다.

프레임률: 요청 5fps인데 실주행은 2.2fps였다. ffmpeg 9.0.1에서 `-fflags nobuffer`가
디코드 프레임의 40%를 버리고(로컬 실측 120/200) 그 위에 `fps=` 필터가 타임스탬프
기준으로 한 번 더 솎았다. 둘 다 뺐다 ― **모든 프레임을 읽고 리더 스레드가 벽시계로
FPS 간격마다만 판정한다.** decoded / fps_in 통계로 현장에서 확인한다.

--- 폴백 ---

카메라가 아예 안 되면(ffmpeg 없음, URL 오류, LOST_S 동안 프레임 없음)
redzone_camera_lost를 내고 v_limit=inf로 간다 ― 붉은 구간에서 항목 2 감점 1회를
받되 완주는 한다. 프레임이 다시 오면 redzone_camera_back으로 다시 켠다.
다만 FLAP_WINDOW_S 안에 복구가 FLAP_MAX회면 네트워크가 불안정한 것이라
그 세션은 아예 끄고(redzone_flapping) 재접속도 멈춘다.

    python redzone.py --selftest
    python redzone.py --tune --host 192.168.50.11          # 실시간 ratio (임계값 조정)
    python redzone.py --snapshot crop.png --host 192.168.50.11   # crop 위치 눈으로 확인
"""

from __future__ import annotations

import collections
import math
import os
import shutil
import subprocess
import threading
import time

RTSP_PORT = 8554
RTSP_PATH = 'front'

CROP = (600, 400, 660, 640)     # W, H, X, Y. 1920x1080 기준 하단 중앙 rows 640~1040
SCALE = (64, 64)                # 축소 후 W, H. 위 절반이 far, 아래 절반이 near
FPS = 5                         # 판정 주기. 45km/h에서 2.5m마다 한 번. 파이프는 전 프레임을 읽는다

RED_RATIO = 1.4                 # r > g*RED_RATIO and r > b*RED_RATIO
RED_MIN = 60                    # 그리고 r > RED_MIN (어두운 픽셀 배제)

ENTER_RATIO = 0.30              # max(far, near)가 이 이상이면 진입 카운트
EXIT_RATIO = 0.15               # near 띠의 붉은 비율이 이 미만이면 이탈 카운트
ENTER_FRAMES = 1                # 분리도가 완벽해 확인 프레임이 필요 없다 (9/10 실주행)
EXIT_FRAMES = 10                # 5fps 기준 2.0초 = 25km/h에서 14m. 짧은 끊김은 이탈이 안 난다

ZONE_SPEED_KMH = 25.0           # 주최측 기준 30. 지시 30에서 최대 39.5 실측 오버슈트 감안
ZONE_HOLD_M = 50.0              # 이탈 뒤 이만큼 더 25km/h 유지. 실측 끊김 최대 28m + 여유
HOLD_JUMP_M = 10.0              # 프레임 간 이동이 이보다 크면 리스폰 ― 홀드 거리에 더하지 않는다
RATE_WINDOW_S = 5.0             # fps 실측 창

LOST_S = 5.0                    # 이만큼 새 프레임이 없으면 camera_lost
RESTART_BACKOFF_S = 2.0         # ffmpeg 재기동 간격
RTSP_TIMEOUT_US = 3_000_000     # ffmpeg -timeout. 연결·수신이 이만큼 막히면 ffmpeg가 끝난다
FLAP_WINDOW_S = 10.0            # 이 안에
FLAP_MAX = 3                    # 복구가 이만큼이면 세션 동안 끈다

STUB_LOOKAHEAD_M = 10.0         # StubCamera의 far 띠가 보는 거리
STUB_NEAR_M = 3.0               # StubCamera의 near 띠가 보는 거리
STUB_HALF_W = 4.0               # StubCamera 회랑 반폭 (차로폭 3.2보다 넉넉하게)

INF = float('inf')
NAN = float('nan')


def rtsp_url(host: str, port: int = RTSP_PORT, path: str = RTSP_PATH) -> str:
    return f'rtsp://{host}:{port}/{path}'


def parse_crop(text: str) -> tuple:
    """'W:H:X:Y' -> (W, H, X, Y). ffmpeg crop 필터와 같은 순서."""
    parts = [int(p) for p in text.replace('x', ':').replace(',', ':').split(':')]
    if len(parts) != 4 or min(parts) < 0 or parts[0] <= 0 or parts[1] <= 0:
        raise ValueError(f'crop은 W:H:X:Y 형식이어야 한다: {text!r}')
    return tuple(parts)


def parse_scale(text: str) -> tuple:
    parts = [int(p) for p in text.replace('x', ':').replace(',', ':').split(':')]
    if len(parts) != 2 or min(parts) < 1:
        raise ValueError(f'scale은 WxH 형식이어야 한다: {text!r}')
    return tuple(parts)


def parse_rect(text: str) -> tuple:
    """'x0,y0,x1,y1' -> (x0, y0, x1, y1)."""
    parts = [float(p) for p in text.split(',')]
    if len(parts) != 4:
        raise ValueError(f'사각형은 x0,y0,x1,y1 형식이어야 한다: {text!r}')
    return tuple(parts)


# ----------------------------------------------------------------------
# 붉은 픽셀 비율

def red_ratio(buf: bytes, w: int, h: int, ratio: float = RED_RATIO,
              rmin: int = RED_MIN) -> tuple:
    """rgb24 프레임의 (far, near) 붉은 픽셀 비율.

    위 h//2 행이 far(멀리), 나머지가 near(코앞). 아스팔트는 R≈G≈B 무채색이고
    붉은 노면은 R이 확연히 높다. 흰 차선(R≈G≈B 밝음)과 노란 차선(R≈G≫B)은
    r > g*ratio에서 걸러진다.
    """
    n_px = len(buf) // 3
    rows_far = h // 2
    split = rows_far * w * 3
    if split > len(buf):
        split = len(buf)

    def count(lo: int, hi: int) -> int:
        n = 0
        for i in range(lo, hi - 2, 3):
            r = buf[i]
            if r > rmin and r > buf[i + 1] * ratio and r > buf[i + 2] * ratio:
                n += 1
        return n

    px_far = split // 3
    px_near = n_px - px_far
    far = count(0, split) / px_far if px_far else 0.0
    near = count(split, len(buf)) / px_near if px_near else 0.0
    return far, near


# ----------------------------------------------------------------------
# ffmpeg

def ffmpeg_args(url: str, crop: tuple = CROP, scale: tuple = SCALE,
                fps: int = FPS, ffmpeg: str = 'ffmpeg') -> list:
    """RTSP(또는 lavfi:) -> crop -> scale -> rgb24 rawvideo를 stdout으로."""
    w, h, x, y = crop
    if url.startswith('lavfi:'):
        # -re: 합성 소스를 실시간 속도로 읽는다. 없으면 ffmpeg가 최대 속도로 뿜어
        # (실측 548fps) 리더 스레드가 CPU를 먹고 20Hz 루프가 흔들린다.
        src = ['-re', '-f', 'lavfi', '-i', url[len('lavfi:'):]]
    else:
        # low_delay: 디코더 출력 지연을 줄인다. **-fflags nobuffer는 쓰지 않는다** ―
        # ffmpeg 9.0.1에서 디코드된 프레임의 40%를 버린다 (1080p h264 20fps 실시간
        # 파이프 실측 120/200, low_delay 유무·B-frame 유무 무관). 실주행 2.2fps의 원인.
        # -timeout(µs): 소켓 I/O 한계. 없으면 연결 거부·스트림 정지에서 ffmpeg가
        # 10초 넘게 매달려 재기동이 안 된다 (실측). 3초면 끊고 다시 붙는다.
        src = ['-rtsp_transport', 'tcp', '-timeout', str(RTSP_TIMEOUT_US),
               '-flags', 'low_delay', '-i', url]
    # fps= 필터를 쓰지 않는다. 타임스탬프 기준으로 솎는데 RTSP에서 그 기준이 흔들려
    # 5fps 요청이 2.2fps로 나왔다. 전 프레임을 내보내고 리더가 벽시계로 솎는다 (fps 인자는
    # 호출부 호환용으로만 남긴다).
    del fps
    return [ffmpeg, '-nostdin', '-hide_banner', '-loglevel', 'error', *src,
            '-an', '-sn',
            '-vf', f'crop={w}:{h}:{x}:{y},scale={scale[0]}:{scale[1]}',
            '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-']


def _popen_kwargs() -> dict:
    kw = {}
    if os.name == 'nt':
        kw['creationflags'] = getattr(subprocess, 'CREATE_NO_WINDOW', 0)
    return kw


def find_ffmpeg(name: str = 'ffmpeg') -> str:
    return shutil.which(name) or ''


def grab_ratio(url: str, crop: tuple = CROP, scale: tuple = SCALE,
               timeout: float = 8.0, ffmpeg: str = 'ffmpeg',
               ratio: float = RED_RATIO, rmin: int = RED_MIN):
    """프레임 한 장을 받아 (far, near)를 낸다. 실패하면 None. preflight용."""
    exe = find_ffmpeg(ffmpeg)
    if not exe:
        return None
    args = ffmpeg_args(url, crop, scale, fps=FPS, ffmpeg=exe)
    args.insert(-1, '-frames:v')
    args.insert(-1, '1')
    try:
        r = subprocess.run(args, stdin=subprocess.DEVNULL, capture_output=True,
                           timeout=timeout, **_popen_kwargs())
    except (OSError, subprocess.TimeoutExpired):
        return None
    need = scale[0] * scale[1] * 3
    if len(r.stdout) < need:
        return None
    return red_ratio(r.stdout[:need], scale[0], scale[1], ratio, rmin)


class CameraReader:
    """ffmpeg 파이프를 읽는 데몬 스레드. latest()는 (far, near, seq)."""

    def __init__(self, url: str, crop: tuple = CROP, scale: tuple = SCALE,
                 fps: int = FPS, ffmpeg: str = 'ffmpeg',
                 red_ratio_: float = RED_RATIO, red_min: int = RED_MIN,
                 backoff_s: float = RESTART_BACKOFF_S):
        self.url = url
        self.crop, self.scale, self.fps = tuple(crop), tuple(scale), fps
        self.ffmpeg = ffmpeg
        self.red_ratio, self.red_min = red_ratio_, red_min
        self.backoff_s = backoff_s
        self.frame_bytes = scale[0] * scale[1] * 3
        self._latest = (0.0, 0.0, 0)
        self._stop = threading.Event()
        self._proc = None
        self._th = None
        self.exe = ''
        self.last_err = ''
        self.frames = 0            # 판정한 프레임 (FPS 간격)
        self.decoded = 0           # 파이프에서 읽은 전체 프레임
        self.dropped = 0           # 읽었지만 판정하지 않은 프레임 (= decoded - frames)
        self.restarts = 0
        # fps 실측. 리더 스레드만 append/popleft 하고 rates()는 len과 양끝만 읽는다.
        self._win_dec = collections.deque()
        self._win_eval = collections.deque()

    # -- 외부 인터페이스 (StubCamera와 같다) --------------------------------
    def start(self) -> bool:
        self.exe = find_ffmpeg(self.ffmpeg)
        if not self.exe:
            self.last_err = f'{self.ffmpeg} 없음 (PATH)'
            return False
        self._th = threading.Thread(target=self._loop, daemon=True,
                                    name='redzone-cam')
        self._th.start()
        return True

    def latest(self) -> tuple:
        return self._latest          # 튜플 대입은 원자적이라 락이 필요 없다

    def observe(self, x: float, y: float, heading: float, t: float) -> None:
        pass                         # 실제 카메라는 위치를 모른다

    def rates(self) -> tuple:
        """최근 RATE_WINDOW_S의 (입력 fps, 판정 fps). 프레임이 끊겼으면 0."""
        now = time.monotonic()
        out = []
        for dq in (self._win_dec, self._win_eval):
            try:
                n = len(dq)
                if n < 2 or now - dq[-1] > RATE_WINDOW_S:
                    out.append(0.0)
                    continue
                span = max(now - dq[0], 1e-3)
                out.append(n / span)
            except IndexError:       # 리더가 비우는 순간
                out.append(0.0)
        return tuple(out)

    @property
    def alive(self) -> bool:
        return (self._th is not None and self._th.is_alive()
                and not self._stop.is_set())

    def stop(self) -> None:
        self._stop.set()
        self._kill()
        if self._th is not None:
            self._th.join(1.0)

    # -- 내부 ------------------------------------------------------------
    def _kill(self) -> None:
        p = self._proc
        if p is not None and p.poll() is None:
            try:
                p.kill()
            except OSError:
                pass

    def _spawn(self):
        return subprocess.Popen(
            ffmpeg_args(self.url, self.crop, self.scale, self.fps, self.exe),
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            # stderr를 비우지 않으면 파이프가 차서 ffmpeg가 멈춘다
            stderr=subprocess.DEVNULL, bufsize=0, **_popen_kwargs())

    def _loop(self) -> None:
        first = True
        while not self._stop.is_set():
            if not first:
                self.restarts += 1
                if self._stop.wait(self.backoff_s):
                    break
            first = False
            try:
                self._proc = self._spawn()
            except OSError as exc:
                self.last_err = f'ffmpeg 실행 실패 {exc!r}'
                continue
            try:
                self._read(self._proc.stdout)
            except Exception as exc:            # 스레드가 죽으면 안 된다
                self.last_err = repr(exc)
            finally:
                self._kill()
                try:
                    self._proc.wait(1.0)
                except Exception:
                    pass

    def _read(self, fh) -> None:
        need = self.frame_bytes
        # 전 프레임(20fps x 12,288B = 245KB/s)을 읽어 파이프를 비우고 판정은 벽시계로
        # 1/fps 간격마다만 한다. 타임스탬프에 기대지 않으므로 소스가 어떻게 찍든
        # 판정률이 fps다. 폭주 소스(비실시간 lavfi)도 같은 상한에 걸린다.
        min_gap = 1.0 / max(self.fps, 1)
        last = 0.0
        while not self._stop.is_set():
            buf = bytearray()
            while len(buf) < need:
                chunk = fh.read(need - len(buf))
                if not chunk:
                    self.last_err = 'ffmpeg 종료 (EOF)'
                    return
                buf += chunk
            now = time.monotonic()
            self.decoded += 1
            self._push(self._win_dec, now)
            if now - last < min_gap:
                self.dropped += 1
                continue
            last = now
            far, near = red_ratio(bytes(buf), self.scale[0], self.scale[1],
                                  self.red_ratio, self.red_min)
            self.frames += 1
            self._push(self._win_eval, now)
            self._latest = (far, near, self.frames)

    @staticmethod
    def _push(dq, now: float) -> None:
        dq.append(now)
        while dq and now - dq[0] > RATE_WINDOW_S:
            dq.popleft()


def _dist_to_polyline(x: float, y: float, poly: list) -> float:
    """점에서 폴리라인(선분 열)까지의 최단 거리."""
    best = INF
    if len(poly) == 1:
        return math.hypot(poly[0][0] - x, poly[0][1] - y)
    for k in range(len(poly) - 1):
        ax, ay = poly[k]
        bx, by = poly[k + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        d = math.hypot(ax + u * dx - x, ay + u * dy - y)
        if d < best:
            best = d
    return best


class StubCamera:
    """sim 전용 합성 카메라. 후륜축 앞 lookahead 지점이 **경로 회랑** 안이면 붉다.

    붉은 구간은 사각형이 아니라 **경로 위 두 끝점 사이의 폴리라인 ± 반폭**이다.
    사각형으로 두면 도로가 휘어 상자 밖으로 나갈 때마다 껐다 켰다 한다 ― 실제로
    경로 2의 (359,-71)~(515,-137)에서 세 번 토글했다. from_path()로 만든다.

    far 띠는 STUB_LOOKAHEAD_M 앞, near 띠는 STUB_NEAR_M 앞을 본다. 실제 crop이
    보는 거리와 같다는 보장은 없다 ― 현장 crop 검증은 --snapshot이 맡는다.

    회랑은 여러 개일 수 있다 ― **끊긴 붉은 노면**(9/10 실주행)을 sim에서 만드는
    유일한 수단이다. 하나라도 안이면 붉다.
    """

    def __init__(self, polys: list, half_width: float = STUB_HALF_W,
                 lookahead_m: float = STUB_LOOKAHEAD_M,
                 near_m: float = STUB_NEAR_M, fps: int = FPS, label: str = ''):
        if not polys:
            raise ValueError('StubCamera: 폴리라인이 비었다')
        if isinstance(polys[0][0], (int, float)):      # 폴리라인 하나를 그대로 준 경우
            polys = [polys]
        self.polys = [[(float(px), float(py)) for px, py in poly] for poly in polys]
        if any(not p for p in self.polys):
            raise ValueError('StubCamera: 빈 폴리라인이 있다')
        self.half_width = half_width
        self.lookahead_m, self.near_m = lookahead_m, near_m
        self.period = 1.0 / fps
        self._latest = (0.0, 0.0, 0)
        self._next_t = None
        self.frames = 0
        self.restarts = 0
        self.stopped = False
        self.last_err = ''
        self.url = label or f'stub:{len(self.polys)}회랑'

    @property
    def poly(self) -> list:
        return self.polys[0]

    @classmethod
    def from_path(cls, path, rects, **kw) -> 'StubCamera':
        """(x0,y0,x1,y1)[들]을 경로에 스냅해 그 사이 경로점을 회랑으로 쓴다."""
        if isinstance(rects[0], (int, float)):
            rects = [rects]
        polys, labels = [], []
        for x0, y0, x1, y1 in rects:
            i0, i1 = sorted((path.nearest(x0, y0), path.nearest(x1, y1)))
            polys.append(path.pts[i0:i1 + 1])
            labels.append(f'{x0:.0f},{y0:.0f}~{x1:.0f},{y1:.0f}'
                          f'(s {path.s[i0]:.1f}~{path.s[i1]:.1f})')
        return cls(polys, label='stub:' + ' + '.join(labels), **kw)

    def start(self) -> bool:
        return True

    def stop(self) -> None:
        self.stopped = True

    def latest(self) -> tuple:
        return self._latest

    @property
    def alive(self) -> bool:
        return not self.stopped

    def inside(self, x: float, y: float) -> bool:
        return any(_dist_to_polyline(x, y, p) <= self.half_width for p in self.polys)

    def observe(self, x: float, y: float, heading: float, t: float) -> None:
        if self.stopped or x != x:
            return
        if self._next_t is None:
            self._next_t = t
        if t < self._next_t:
            return
        while self._next_t <= t:
            self._next_t += self.period
        c, s = math.cos(heading), math.sin(heading)
        far = 1.0 if self.inside(x + self.lookahead_m * c,
                                 y + self.lookahead_m * s) else 0.0
        near = 1.0 if self.inside(x + self.near_m * c,
                                  y + self.near_m * s) else 0.0
        self.frames += 1
        self._latest = (far, near, self.frames)


# ----------------------------------------------------------------------
# 정책

class RedzoneCmd:
    """한 프레임의 판정 결과. stoplines.SignalCmd와 같은 모양이다."""

    __slots__ = ('v_limit', 'in_zone', 'hold', 'ratio', 'ratio_far', 'action', 'reason')

    def __init__(self, v_limit=INF, in_zone=False, hold=False, ratio=NAN,
                 ratio_far=NAN, action='none', reason=''):
        self.v_limit = v_limit          # m/s. inf면 제약 없음
        self.in_zone = in_zone
        self.hold = hold                # 이탈 뒤 ZONE_HOLD_M 유지 중
        self.ratio = ratio              # near 띠 붉은 비율 (이탈 판정)
        self.ratio_far = ratio_far      # far 띠 붉은 비율
        self.action = action            # none / zone / hold / lost / off / error
        self.reason = reason

    @property
    def in_code(self) -> int:
        """drive.csv rz_in: 0 밖 / 1 구간 / 2 홀드."""
        return 1 if self.in_zone else (2 if self.hold else 0)

    def __repr__(self) -> str:
        return (f'<RedzoneCmd {self.action} v<={self.v_limit:.1f} '
                f'far={self.ratio_far:.2f} near={self.ratio:.2f}>')


class RedzonePolicy:
    """붉은 노면 위에서 ZONE_SPEED로 제한한다. 순수 로직 ― 소스만 갈아 끼우면 된다.

    상태 기계
        밖 --max(far,near)>=ENTER x ENTER_FRAMES--> 구간 --near<EXIT x EXIT_FRAMES--> 홀드
        홀드 --주행거리 hold_m--> 밖,   홀드 --붉은 프레임--> 구간 (거리 리셋)
    홀드 중에도 v_limit은 구간과 같다. 카메라를 잃어도 홀드는 거리로 끝난다.
    """

    def __init__(self, source, zone_speed: float = ZONE_SPEED_KMH / 3.6,
                 enter_ratio: float = ENTER_RATIO, exit_ratio: float = EXIT_RATIO,
                 enter_frames: int = ENTER_FRAMES, exit_frames: int = EXIT_FRAMES,
                 hold_m: float = ZONE_HOLD_M,
                 lost_s: float = LOST_S, flap_window: float = FLAP_WINDOW_S,
                 flap_max: int = FLAP_MAX, on_event=None):
        self.src = source
        self.zone_speed = zone_speed
        self.enter_ratio, self.exit_ratio = enter_ratio, exit_ratio
        self.enter_frames, self.exit_frames = max(1, enter_frames), max(1, exit_frames)
        self.hold_m = max(0.0, hold_m)
        self.lost_s = lost_s
        self.flap_window, self.flap_max = flap_window, flap_max
        self._ev = on_event or (lambda kind, detail: None)

        self.in_zone = False
        self.hold = False
        self.lost = False
        self.disabled = False
        self.off_reason = ''
        self._n_enter = self._n_exit = 0
        self._seq = 0
        self._t_frame = None            # 마지막 새 프레임의 (정책) 시각
        self._hold_dist = 0.0           # 홀드 중 누적 주행거리
        self._last_xy = None
        self._last_t = None
        self._backs = collections.deque()
        self.n_lost = self.n_back = self.n_enter = self.n_exit = 0
        self.n_hold = self.n_hold_end = 0
        self._pending_lost = ''
        if not self.src.start():
            self._pending_lost = getattr(self.src, 'last_err', '') or '소스 시작 실패'

    # -- 외부 인터페이스 --------------------------------------------------
    def update(self, v: float, t: float, x: float = NAN, y: float = NAN,
               heading: float = 0.0) -> RedzoneCmd:
        if self.disabled:
            return RedzoneCmd(action='off', reason=self.off_reason)
        self.src.observe(x, y, heading, t)
        far, near, seq = self.src.latest()
        if self._t_frame is None:
            self._t_frame = t
        if self._pending_lost:
            reason, self._pending_lost = self._pending_lost, ''
            self._go_lost(reason)

        # 홀드 거리는 카메라와 무관하게 매 프레임 쌓는다 (lost 중에도)
        self._advance(v, t, x, y)

        if seq != self._seq:
            self._seq = seq
            self._t_frame = t
            if self.lost:
                self._back(t)
            if not self.disabled:
                self._step(far, near, x, y)
        elif not self.lost and t - self._t_frame > self.lost_s:
            err = getattr(self.src, 'last_err', '')
            self._go_lost(f'{self.lost_s:.0f}초 프레임 없음'
                          + (f' ({err})' if err else ''))

        # 재진입이 같은 프레임에서 이겼으면 hold가 이미 꺼져 있다
        if self.hold and self._hold_dist >= self.hold_m:
            self.hold = False
            self.n_hold_end += 1
            self._ev('redzone_hold_end',
                     f'누적 {self._hold_dist:.1f}m ({x:.1f},{y:.1f}) -> 지시속도')

        if self.disabled:
            return RedzoneCmd(action='off', reason=self.off_reason)
        if self.lost:
            return RedzoneCmd(v_limit=self.zone_speed if self.hold else INF,
                              hold=self.hold, action='lost',
                              reason='카메라 없음 ― ' + ('홀드 유지' if self.hold
                                                      else '지시속도 유지'))
        if self.in_zone:
            return RedzoneCmd(v_limit=self.zone_speed, in_zone=True,
                              ratio=near, ratio_far=far, action='zone')
        if self.hold:
            return RedzoneCmd(v_limit=self.zone_speed, hold=True,
                              ratio=near, ratio_far=far, action='hold',
                              reason=f'{self._hold_dist:.0f}/{self.hold_m:.0f}m')
        return RedzoneCmd(ratio=near, ratio_far=far)

    def reset(self) -> None:
        """리스폰. 카운터만 버린다. in_zone·홀드는 그대로 ― 카메라가 다시 정하고
        점프는 _advance가 거리에 더하지 않는다."""
        self._n_enter = self._n_exit = 0
        self._last_xy = None

    def close(self) -> None:
        self.src.stop()

    def stats(self) -> dict:
        out = {'frames': getattr(self.src, 'frames', 0),
               'restarts': getattr(self.src, 'restarts', 0),
               'dropped': getattr(self.src, 'dropped', 0)}
        rates = getattr(self.src, 'rates', None)
        if callable(rates):                 # 실제 카메라만. 스텁은 수신 개념이 없다
            fps_in, fps_eval = rates()
            out.update(decoded=getattr(self.src, 'decoded', 0),
                       fps_in=round(fps_in, 2), fps_eval=round(fps_eval, 2))
        out.update(
                lost=self.n_lost, back=self.n_back,
                enter=self.n_enter, exit=self.n_exit,
                hold=self.n_hold, hold_end=self.n_hold_end,
                disabled=self.disabled, off_reason=self.off_reason,
                last_err=getattr(self.src, 'last_err', ''))
        return out

    # -- 내부 ------------------------------------------------------------
    def _advance(self, v: float, t: float, x: float, y: float) -> None:
        """홀드 거리 누적. (x,y)가 있으면 프레임 간 이동, 없으면(--tune) v*dt."""
        d = 0.0
        if x == x and y == y:
            if self._last_xy is not None:
                d = math.hypot(x - self._last_xy[0], y - self._last_xy[1])
                if d > HOLD_JUMP_M:         # 리스폰 ― 이동이 아니다
                    d = 0.0
            self._last_xy = (x, y)
        elif self._last_t is not None and v == v:
            d = max(0.0, v) * max(0.0, t - self._last_t)
        self._last_t = t
        if self.hold:
            self._hold_dist += d

    def _start_hold(self, why: str) -> None:
        if self.hold_m <= 0.0:
            return
        self.hold = True
        self._hold_dist = 0.0
        self.n_hold += 1
        self._ev('redzone_hold',
                 f'{why} ― {self.hold_m:.0f}m 동안 {self.zone_speed * 3.6:.0f}km/h 유지')

    def _step(self, far: float, near: float, x: float, y: float) -> None:
        red = max(far, near) >= self.enter_ratio
        if not self.in_zone:
            self._n_enter = self._n_enter + 1 if red else 0
            if self._n_enter >= self.enter_frames:
                was_hold, dist = self.hold, self._hold_dist
                self.in_zone, self.hold = True, False
                self._n_enter = self._n_exit = 0
                self.n_enter += 1
                self._ev('redzone_enter',
                         f'far={far:.2f} near={near:.2f} ({x:.1f},{y:.1f}) '
                         f'-> {self.zone_speed * 3.6:.0f}km/h'
                         + (f' (홀드 중 재진입, {dist:.1f}m에서)' if was_hold else ''))
        else:
            self._n_exit = self._n_exit + 1 if near < self.exit_ratio else 0
            if self._n_exit >= self.exit_frames:
                self.in_zone = False
                self._n_enter = self._n_exit = 0
                self.n_exit += 1
                self._ev('redzone_exit',
                         f'far={far:.2f} near={near:.2f} ({x:.1f},{y:.1f})')
                self._start_hold('카메라 이탈')

    def _go_lost(self, reason: str) -> None:
        self.lost = True
        self.n_lost += 1
        was = self.in_zone
        self.in_zone = False
        self._n_enter = self._n_exit = 0
        self._ev('redzone_camera_lost',
                 reason + ' ― 전 구간 지시속도로 간다'
                 + (' (구간 중 해제)' if was else ''))
        if was:
            self._start_hold('카메라 소실')      # 구간 안에서 잃었으면 홀드로 넘긴다

    def _back(self, t: float) -> None:
        self.lost = False
        self.n_back += 1
        self._n_enter = self._n_exit = 0
        self._backs.append(t)
        while self._backs and t - self._backs[0] > self.flap_window:
            self._backs.popleft()
        self._ev('redzone_camera_back',
                 f'프레임 재개 (복구 {len(self._backs)}회/{self.flap_window:.0f}s)')
        if len(self._backs) >= self.flap_max:
            self.disabled = True
            self.off_reason = 'flapping'
            self.src.stop()
            self._ev('redzone_flapping',
                     f'{self.flap_window:.0f}초 안에 복구 {len(self._backs)}회 ― '
                     f'이 세션은 카메라 판정을 끈다 (재접속 중단)')


def summary(policy: RedzonePolicy) -> list:
    """시작 시 콘솔에 뿌릴 요약 줄."""
    src = policy.src
    lines = [f'  소스 {getattr(src, "url", "?")}']
    if isinstance(src, CameraReader):
        w, h, x, y = src.crop
        lines.append(f'  ffmpeg {src.exe or src.ffmpeg}  crop {w}x{h}+{x}+{y} '
                     f'-> {src.scale[0]}x{src.scale[1]}  판정 {src.fps}fps (전 프레임 수신)  '
                     f'붉음: r>{src.red_min} & r>{src.red_ratio:.1f}·g,b')
    lines.append(f'  진입 max(far,near)>={policy.enter_ratio:.2f} x{policy.enter_frames}  '
                 f'이탈 near<{policy.exit_ratio:.2f} x{policy.exit_frames}  '
                 f'구간 {policy.zone_speed * 3.6:.0f}km/h  이탈 뒤 홀드 {policy.hold_m:.0f}m')
    lines.append(f'  폴백: {policy.lost_s:.0f}초 프레임 없으면 해제, '
                 f'{policy.flap_window:.0f}초 안 복구 {policy.flap_max}회면 세션 동안 끔')
    if policy._pending_lost:
        lines.append(f'  *** 카메라 시작 실패: {policy._pending_lost} ― '
                     f'붉은 구역 판정 없이 간다 ***')
    return lines


# ----------------------------------------------------------------------
# CLI — 현장 도구

def _bar(r: float, width: int = 20) -> str:
    n = int(round(max(0.0, min(1.0, r)) * width))
    return '#' * n + '.' * (width - n)


def _histogram(vals: list, step: float = 0.05) -> list:
    if not vals:
        return ['  (없음)']
    bins = collections.Counter(min(int(v / step), int(1.0 / step)) for v in vals)
    total = len(vals)
    out = []
    for k in range(int(1.0 / step) + 1):
        c = bins.get(k, 0)
        if c:
            out.append(f'  {k * step:4.2f}~{(k + 1) * step:4.2f}  {c:5d}  '
                       f'{"*" * int(40 * c / total)}')
    return out


def tune(url: str, crop: tuple, scale: tuple, fps: int, enter: float,
         exit_: float, red_ratio_: float, red_min: int, ffmpeg: str) -> int:
    """실시간 ratio. 9910은 열지 않는다 ― 연결 하나만 받으므로 수동 주행과 겹친다."""
    cam = CameraReader(url, crop, scale, fps, ffmpeg, red_ratio_, red_min)

    def ev(kind, detail):
        print(f'\n  [{kind}] {detail}')

    pol = RedzonePolicy(cam, enter_ratio=enter, exit_ratio=exit_, on_event=ev)
    for line in summary(pol):
        print(line)
    print('  Ctrl+C로 끝내면 히스토그램을 낸다\n')
    print('  홀드는 주행거리 기준이라 여기(위치 없음)서는 재진입 전까지 유지된다\n')
    fars, nears = [], []
    t0 = time.monotonic()
    seq = 0
    last_msg = last_rate = t0
    try:
        while True:
            t = time.monotonic() - t0
            cmd = pol.update(0.0, t)
            far, near, s = cam.latest()
            if s != seq:
                seq = s
                fars.append(far)
                nears.append(near)
                state = 'IN ZONE' if pol.in_zone else ('HOLD' if pol.hold else '-')
                print(f'  {t:7.2f}s  far {far:.2f} [{_bar(far)}]  '
                      f'near {near:.2f} [{_bar(near)}]  '
                      f'E{pol._n_enter}/{pol.enter_frames} '
                      f'X{pol._n_exit}/{pol.exit_frames}  '
                      f'{state:8s} {cmd.action}')
            elif time.monotonic() - last_msg > 2.0:
                last_msg = time.monotonic()
                print(f'  {t:7.2f}s  프레임 대기 중... '
                      f'({cam.last_err or "아직 없음"}, 재기동 {cam.restarts})')
            if time.monotonic() - last_rate > 2.0:
                last_rate = time.monotonic()
                fi, fe = cam.rates()
                print(f'  {t:7.2f}s  ** 입력 {fi:.1f} fps / 판정 {fe:.1f} fps '
                      f'(수신 {cam.decoded}, 판정 {cam.frames}) ― 입력이 20 근처여야 한다')
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        pol.close()
    fi, fe = cam.rates()
    print(f'\n프레임 수신 {cam.decoded} / 판정 {len(fars)}개, 재기동 {cam.restarts}회, '
          f'마지막 창 입력 {fi:.1f} / 판정 {fe:.1f} fps')
    print('far 히스토그램 (진입 판정)')
    print('\n'.join(_histogram(fars)))
    print('near 히스토그램 (이탈 판정)')
    print('\n'.join(_histogram(nears)))
    return 0


def snapshot(url: str, out: str, crop: tuple, scale: tuple,
             ffmpeg: str, timeout: float = 15.0) -> int:
    """전체 프레임에 crop 사각형과 far/near 경계를 그린 PNG + crop 원본 PNG.

    표준 라이브러리로 PNG를 쓰는 게 아니라 ffmpeg가 직접 쓴다.
    """
    exe = find_ffmpeg(ffmpeg)
    if not exe:
        print(f'{ffmpeg} 없음')
        return 1
    w, h, x, y = crop
    base, ext = os.path.splitext(out)
    ext = ext or '.png'
    full, cropped = base + ext, base + '_crop' + ext
    src = (['-f', 'lavfi', '-i', url[6:]] if url.startswith('lavfi:')
           else ['-rtsp_transport', 'tcp', '-i', url])
    box = (f'drawbox=x={x}:y={y}:w={w}:h={h}:color=red@0.9:t=4,'
           f'drawbox=x={x}:y={y + h // 2}:w={w}:h=1:color=yellow@0.9:t=2')
    jobs = [(full, box), (cropped, f'crop={w}:{h}:{x}:{y}')]
    for path, vf in jobs:
        args = [exe, '-y', '-nostdin', '-hide_banner', '-loglevel', 'error',
                *src, '-frames:v', '1', '-vf', vf, path]
        try:
            r = subprocess.run(args, stdin=subprocess.DEVNULL,
                               capture_output=True, timeout=timeout,
                               **_popen_kwargs())
        except subprocess.TimeoutExpired:
            print(f'{path}: {timeout:.0f}초 타임아웃 ― 스트림이 안 온다')
            return 1
        if r.returncode != 0 or not os.path.exists(path):
            print(f'{path}: ffmpeg 실패\n{r.stderr.decode("utf-8", "ignore")}')
            return 1
        print(f'저장: {path}')
    rat = grab_ratio(url, crop, scale, timeout=timeout, ffmpeg=exe)
    if rat is not None:
        print(f'지금 프레임 far={rat[0]:.2f} near={rat[1]:.2f}')
    print('빨간 사각형이 노면만 덮는지, 노란 선 위(far)가 더 먼 노면인지 확인할 것')
    return 0


# ----------------------------------------------------------------------
# 자기검사

class _Rec:
    def __init__(self):
        self.ev = []

    def __call__(self, kind, detail):
        self.ev.append((kind, detail))

    def kinds(self):
        return [k for k, _d in self.ev]


class _FakeSource:
    """스크립트된 (far, near) 프레임을 주는 소스."""

    def __init__(self, ok: bool = True):
        self.ok = ok
        self._latest = (0.0, 0.0, 0)
        self.frames = 0
        self.stopped = 0
        self.last_err = '' if ok else 'fake: 시작 실패'
        self.url = 'fake:'

    def start(self) -> bool:
        return self.ok

    def stop(self) -> None:
        self.stopped += 1

    def latest(self) -> tuple:
        return self._latest

    def observe(self, x, y, heading, t) -> None:
        pass

    def feed(self, far: float, near: float) -> None:
        self.frames += 1
        self._latest = (far, near, self.frames)


def _rgb(w: int, h: int, top, bottom=None) -> bytes:
    """위 절반 top색, 아래 절반 bottom색인 rgb24 프레임."""
    bottom = bottom or top
    rows = []
    for r in range(h):
        rows.append(bytes(top if r < h // 2 else bottom) * w)
    return b''.join(rows)


def _selftest() -> int:
    ok = True

    def check(name, cond, detail=''):
        nonlocal ok
        ok = ok and bool(cond)
        print(f'  {"PASS" if cond else "FAIL"}  {name}'
              + (f'  - {detail}' if detail else ''))

    print('붉은 보호구역 카메라 판정 자기검사 (항목 2)')
    w, h = 8, 4

    # 1. red_ratio
    far, near = red_ratio(_rgb(w, h, (160, 64, 64)), w, h)
    check('붉은 노면: (1.0, 1.0)', far == 1.0 and near == 1.0, f'{far},{near}')
    far, near = red_ratio(_rgb(w, h, (90, 90, 90)), w, h)
    check('회색 아스팔트: (0.0, 0.0)', far == 0.0 and near == 0.0, f'{far},{near}')
    far, near = red_ratio(_rgb(w, h, (160, 64, 64), (90, 90, 90)), w, h)
    check('위만 붉음: (1.0, 0.0)', far == 1.0 and near == 0.0, f'{far},{near}')
    far, near = red_ratio(_rgb(w, h, (240, 240, 240)), w, h)
    check('흰 차선은 붉지 않다', far == 0.0, f'{far}')
    far, near = red_ratio(_rgb(w, h, (230, 200, 40)), w, h)
    check('노란 차선은 붉지 않다', far == 0.0, f'{far}')
    far, near = red_ratio(_rgb(w, h, (50, 10, 10)), w, h)
    check('어두운 픽셀은 RED_MIN에서 걸러진다', far == 0.0, f'{far}')
    row_red = bytes((160, 64, 64)) * w
    row_gray = bytes((90, 90, 90)) * w
    far, near = red_ratio(row_red + row_gray + row_red + row_gray, w, 4)
    check('절반 붉음: 0.5', abs(far - 0.5) < 1e-9 and abs(near - 0.5) < 1e-9,
          f'{far},{near}')

    # 2. 히스테리시스 + 홀드
    rec = _Rec()
    src = _FakeSource()
    pol = RedzonePolicy(src, on_event=rec)
    dt = 1.0 / FPS
    t = 0.0
    pos = [0.0]                     # x 좌표. 홀드 거리는 여기서 잰다

    def frame(far, near, move=2.0):
        nonlocal t
        src.feed(far, near)
        t += dt
        pos[0] += move
        return pol.update(12.5, t, pos[0], 0.0)

    def coast(m, far=0.0, near=0.0, step=2.0):
        c = None
        for _ in range(int(round(m / step))):
            c = frame(far, near, step)
        return c

    for _ in range(3):
        c = frame(0.0, 0.0)
    check('회색에서는 무제약', c.v_limit == INF and c.action == 'none', repr(c))
    c = frame(0.6, 0.0)
    check(f'붉은 {ENTER_FRAMES}프레임에 진입', c.in_zone and c.action == 'zone', repr(c))
    check('구간 v_limit = 25km/h',
          abs(c.v_limit - ZONE_SPEED_KMH / 3.6) < 1e-9, f'{c.v_limit * 3.6:.1f}')
    check('redzone_enter 발화', rec.kinds().count('redzone_enter') == 1,
          ','.join(rec.kinds()))
    check('rz_in 코드: 구간 1', c.in_code == 1, f'{c.in_code}')
    for _ in range(EXIT_FRAMES + 2):
        c = frame(0.0, 0.5)
    check('far 소실·near 붉음이면 유지 (늦게 이탈)', c.in_zone, repr(c))
    for _ in range(EXIT_FRAMES - 1):
        c = frame(0.0, 0.05)
    check(f'near {EXIT_FRAMES - 1}프레임으로는 이탈하지 않는다', c.in_zone, repr(c))
    c = frame(0.0, 0.5)
    c = frame(0.0, 0.05)
    check('이탈 카운트도 리셋된다', c.in_zone, repr(c))
    for _ in range(EXIT_FRAMES - 1):
        c = frame(0.0, 0.05)
    check(f'near 연속 {EXIT_FRAMES}프레임에 이탈 -> 홀드 (v_limit 그대로)',
          not c.in_zone and c.hold and c.action == 'hold'
          and abs(c.v_limit - ZONE_SPEED_KMH / 3.6) < 1e-9, repr(c))
    check('redzone_exit 직후 redzone_hold 발화',
          rec.kinds()[-2:] == ['redzone_exit', 'redzone_hold'], ','.join(rec.kinds()))
    check('rz_in 코드: 홀드 2', c.in_code == 2, f'{c.in_code}')
    c = coast(30.0)
    check('홀드 30m: 아직 25km/h', c.hold and c.v_limit < INF, repr(c))
    c = frame(0.0, 0.6)
    check('near만 붉어도 재진입 (홀드 거리 리셋)',
          c.in_zone and not c.hold and rec.kinds()[-1] == 'redzone_enter'
          and '재진입' in rec.ev[-1][1], rec.ev[-1][1])
    for _ in range(EXIT_FRAMES):
        c = frame(0.0, 0.0)
    check('다시 이탈 -> 홀드', c.hold and not c.in_zone, repr(c))
    c = coast(ZONE_HOLD_M - 2.0)
    check(f'홀드 {ZONE_HOLD_M - 2:.0f}m: 유지', c.hold, repr(c))
    c = coast(4.0)
    check(f'홀드 {ZONE_HOLD_M:.0f}m 뒤 해제: v_limit=inf, redzone_hold_end',
          not c.hold and c.v_limit == INF and c.action == 'none'
          and rec.kinds()[-1] == 'redzone_hold_end', repr(c) + ' ' + rec.ev[-1][1])
    c = frame(0.6, 0.6)
    for _ in range(EXIT_FRAMES):
        c = frame(0.0, 0.0)
    c = frame(0.0, 0.0, 200.0)
    check('홀드 중 리스폰 점프(200m)는 거리에 안 들어간다',
          c.hold and pol._hold_dist < 1.0, f'{pol._hold_dist:.1f}m')
    src0 = _FakeSource()
    pol0 = RedzonePolicy(src0, hold_m=0.0)
    src0.feed(0.9, 0.9)
    pol0.update(12.5, 0.2, 0.0, 0.0)
    for k in range(EXIT_FRAMES):
        src0.feed(0.0, 0.0)
        c = pol0.update(12.5, 0.4 + 0.2 * k, 2.0 * k, 0.0)
    check('hold_m=0이면 홀드 없이 바로 해제', not c.hold and c.v_limit == INF, repr(c))

    # 3. lost — 프레임이 멈추면 LOST_S 뒤 해제. 구간 중이었으면 홀드로 넘긴다
    rec = _Rec()
    src = _FakeSource()
    pol = RedzonePolicy(src, on_event=rec)
    t = 0.0
    for _ in range(ENTER_FRAMES):
        src.feed(0.8, 0.8)
        t += dt
        c = pol.update(12.5, t)               # 위치 없이 -> 거리는 v*dt
    check('lost 전: 구간 안', c.in_zone)
    t_stop = t
    while t - t_stop < LOST_S - 0.05:
        t += 0.05
        c = pol.update(12.5, t)
    check(f'{LOST_S:.0f}초 전까지는 마지막 값 유지', c.in_zone and c.action == 'zone',
          repr(c))
    t += 0.1
    c = pol.update(12.5, t)
    check(f'{LOST_S:.0f}초 뒤 lost: in_zone=False, 구간 중이었으니 홀드',
          c.action == 'lost' and not c.in_zone and c.hold and c.v_limit < INF, repr(c))
    check('redzone_camera_lost 발화', 'redzone_camera_lost' in rec.kinds(),
          ','.join(rec.kinds()))
    t_lost = t
    while t - t_lost < ZONE_HOLD_M / 12.5 + 0.3:
        t += 0.05
        c = pol.update(12.5, t)
    check('lost 중에도 홀드는 v*dt 거리로 끝난다 -> v_limit=inf',
          c.action == 'lost' and not c.hold and c.v_limit == INF
          and 'redzone_hold_end' in rec.kinds(), repr(c))
    # 4. back — 다시 오면 켜지고 붉으면 바로 진입
    src.feed(0.8, 0.8)
    t += 0.05
    c = pol.update(12.5, t)
    check('redzone_camera_back 발화', 'redzone_camera_back' in rec.kinds(),
          ','.join(rec.kinds()))
    check('복구 직후 붉은 프레임이면 바로 진입', c.in_zone and c.action == 'zone',
          repr(c))

    # 5. flapping — 창 안에 FLAP_MAX회 복구면 세션 동안 끔.
    # 기본 LOST_S(5초)로는 (5+0.05)*3이 10초 창을 넘으므로 lost_s를 줄여 만든다.
    rec = _Rec()
    src = _FakeSource()
    pol = RedzonePolicy(src, lost_s=1.0, on_event=rec)
    t = 0.0
    src.feed(0.0, 0.0)
    pol.update(0.0, t)
    for _k in range(FLAP_MAX):
        t += 1.2
        pol.update(0.0, t)
        src.feed(0.9, 0.9)
        t += 0.05
        c = pol.update(0.0, t)
    check('redzone_flapping 발화', 'redzone_flapping' in rec.kinds(),
          ','.join(rec.kinds()))
    check('flapping 뒤 action=off', c.action == 'off' and c.v_limit == INF, repr(c))
    check('소스 stop() 호출됨 (재접속 중단)', src.stopped >= 1, f'{src.stopped}')
    for _ in range(ENTER_FRAMES + 2):
        src.feed(0.9, 0.9)
        t += dt
        c = pol.update(12.5, t)
    check('꺼진 뒤에는 붉어도 무제약', c.action == 'off' and c.v_limit == INF, repr(c))
    rec = _Rec()
    src = _FakeSource()
    pol = RedzonePolicy(src, lost_s=1.0, flap_window=3.0, on_event=rec)
    t = 0.0
    src.feed(0.0, 0.0)
    pol.update(0.0, t)
    for _k in range(FLAP_MAX):
        t += 2.0
        pol.update(0.0, t)
        src.feed(0.0, 0.0)
        t += 0.05
        c = pol.update(0.0, t)
    check('창 밖 복구는 flapping이 아니다',
          'redzone_flapping' not in rec.kinds() and c.action == 'none',
          ','.join(rec.kinds()))

    # 6. 시작 실패 — 즉시 lost
    rec = _Rec()
    pol = RedzonePolicy(_FakeSource(ok=False), on_event=rec)
    c = pol.update(0.0, 0.0)
    check('소스 시작 실패면 첫 프레임에 lost',
          c.action == 'lost' and 'redzone_camera_lost' in rec.kinds(), repr(c))

    # 7. reset — 이탈 카운터 초기화 (진입은 1프레임이라 리셋할 것이 없다)
    src = _FakeSource()
    pol = RedzonePolicy(src)
    src.feed(0.9, 0.9)
    pol.update(0.0, 0.2, 0.0, 0.0)
    for k in range(EXIT_FRAMES - 1):
        src.feed(0.0, 0.0)
        pol.update(0.0, 0.4 + 0.2 * k, 0.0, 0.0)
    pol.reset()
    src.feed(0.0, 0.0)
    c = pol.update(0.0, 5.0, 0.0, 0.0)
    check('reset 뒤 이탈 카운터 0 (구간 유지)', c.in_zone, repr(c))

    # 8. StubCamera — 회랑 2개 (100,10)~(150,10), (180,10)~(250,10), 반폭 4m.
    # 사이 30m가 끊긴 붉은 노면이다 ― 홀드가 이어야 한다.
    stub = StubCamera([[(100.0, 10.0), (150.0, 10.0)],
                       [(180.0, 10.0), (250.0, 10.0)]])
    rec = _Rec()
    pol = RedzonePolicy(stub, on_event=rec)
    t, x = 0.0, 50.0
    seen = []
    while x < 400.0:
        t += 0.05
        x += 12.5 * 0.05
        c = pol.update(12.5, t, x, 10.0, 0.0)
        seen.append((x, c.in_zone, c.v_limit < INF))
    first_in = next((sx for sx, i, _l in seen if i), None)
    check(f'Stub: 회랑 앞에서 진입 (far {STUB_LOOKAHEAD_M:.0f}m + {ENTER_FRAMES}프레임)',
          first_in is not None
          and 100.0 - STUB_LOOKAHEAD_M - STUB_HALF_W <= first_in <= 100.0,
          f'진입 x={first_in}')
    check('Stub: 끊긴 30m를 홀드가 잇는다 (x 100~250 내내 25km/h)',
          all(lim for sx, _i, lim in seen if 100.0 <= sx <= 250.0),
          ','.join(rec.kinds()))
    last_lim = max(sx for sx, _i, lim in seen if lim)
    check(f'Stub: 마지막 회랑 끝 + 홀드 {ZONE_HOLD_M:.0f}m 뒤에 풀린다',
          250.0 + ZONE_HOLD_M <= last_lim <= 250.0 + ZONE_HOLD_M + 40.0
          and not seen[-1][2] and rec.kinds().count('redzone_hold_end') == 1,
          f'해제 x={last_lim:.1f}')
    check('Stub: 회랑 옆 5m는 밖, 사이 30m도 밖',
          not stub.inside(125.0, 15.0) and stub.inside(125.0, 13.5)
          and stub.inside(200.0, 10.0) and not stub.inside(165.0, 10.0),
          f'{_dist_to_polyline(125.0, 15.0, stub.poly):.1f}m')

    # 9. ffmpeg 파이프 (있을 때만)
    exe = find_ffmpeg()
    if not exe:
        print('  SKIP  ffmpeg 파이프 ― ffmpeg가 PATH에 없다')
    else:
        for color, name, lo, hi in (('0xA04040', '붉은 lavfi', 0.9, 1.01),
                                    ('0x606060', '회색 lavfi', -0.01, 0.05)):
            cam = CameraReader(f'lavfi:color=c={color}:s=1920x1080:r=20')
            cam.start()
            t0 = time.monotonic()
            got = None
            t_first = None
            hold_s = 3.0 if lo > 0.5 else 0.0     # 붉은 쪽에서만 프레임률을 잰다
            while time.monotonic() - t0 < 6.0 + hold_s:
                far, near, seq = cam.latest()
                if seq >= 1 and t_first is None:
                    t_first = time.monotonic()
                if t_first is not None and time.monotonic() - t_first >= hold_s:
                    got = (far, near)
                    break
                time.sleep(0.05)
            dec, fr = cam.decoded, cam.frames
            fi, fe = cam.rates()
            cam.stop()
            check(f'ffmpeg {name}: 프레임 수신', got is not None,
                  f'{time.monotonic() - t0:.1f}s {cam.last_err}')
            if got:
                check(f'ffmpeg {name}: far/near 범위', lo <= got[0] <= hi
                      and lo <= got[1] <= hi, f'{got[0]:.2f},{got[1]:.2f}')
            if hold_s:
                check(f'ffmpeg {name}: 3초에 수신 >= 40, 판정 8~25 '
                      f'(전 프레임 읽고 {FPS}fps로 솎는다)',
                      dec >= 40 and 8 <= fr <= 25,
                      f'수신 {dec} 판정 {fr}  입력 {fi:.1f}fps / 판정 {fe:.1f}fps')
            check(f'ffmpeg {name}: 프로세스 종료',
                  cam._proc is None or cam._proc.poll() is not None)
        rat = grab_ratio('lavfi:color=c=0xA04040:s=1920x1080:r=20')
        check('grab_ratio 원샷', rat is not None and rat[0] > 0.9,
              f'{rat}')
        cam = CameraReader('lavfi:nosuchsource=1')
        cam.start()
        time.sleep(1.5)
        cam.stop()
        check('잘못된 소스: 프레임 0, 스레드 정리', cam.frames == 0 and not cam.alive,
              f'{cam.last_err} 재기동 {cam.restarts}')

    print('전부 통과' if ok else '실패 있음')
    return 0 if ok else 1


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description='붉은 보호구역 카메라 판정 (항목 2)')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--tune', action='store_true',
                    help='실시간 far/near ratio를 찍는다. Ctrl+C면 히스토그램')
    ap.add_argument('--snapshot', metavar='PNG',
                    help='전체 프레임에 crop 사각형을 그린 PNG를 저장한다')
    ap.add_argument('--host', default='192.168.50.11')
    ap.add_argument('--url', default='',
                    help=f'전체 URL 덮어쓰기 (기본 rtsp://<host>:{RTSP_PORT}/{RTSP_PATH}). '
                         f'lavfi:<spec>도 된다')
    ap.add_argument('--crop', default=':'.join(str(v) for v in CROP),
                    help='W:H:X:Y (1920x1080 기준)')
    ap.add_argument('--scale', default=f'{SCALE[0]}x{SCALE[1]}')
    ap.add_argument('--fps', type=int, default=FPS)
    ap.add_argument('--enter', type=float, default=ENTER_RATIO)
    ap.add_argument('--exit', type=float, default=EXIT_RATIO)
    ap.add_argument('--red-ratio', type=float, default=RED_RATIO)
    ap.add_argument('--red-min', type=int, default=RED_MIN)
    ap.add_argument('--ffmpeg', default='ffmpeg')
    args = ap.parse_args()

    if args.selftest:
        raise SystemExit(_selftest())
    url = args.url or rtsp_url(args.host)
    crop, scale = parse_crop(args.crop), parse_scale(args.scale)
    if args.snapshot:
        raise SystemExit(snapshot(url, args.snapshot, crop, scale, args.ffmpeg))
    if args.tune:
        raise SystemExit(tune(url, crop, scale, args.fps, args.enter, args.exit,
                              args.red_ratio, args.red_min, args.ffmpeg))
    ap.error('--selftest / --tune / --snapshot 중 하나를 줄 것')


if __name__ == '__main__':
    main()
