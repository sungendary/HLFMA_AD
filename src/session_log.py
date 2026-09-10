#!/usr/bin/env python3
"""세션 로그 ― 한 번의 주행에서 나온 모든 것을 디렉터리 하나에 남긴다.

파싱해서 고른 열만 남기면 "그때 이것도 기록할 걸"이 반드시 나온다. 그래서
**수신 원본 바이트를 먼저 통째로 남긴다.** 1109B x 20Hz x 120초 = 2.7MB뿐이라
비용이 없고, 나중에 무엇이든 복원된다. 사전테스트 녹화본이 그대로 집에서의
재생 테스트 벤치가 되기도 한다 (replay_server.py).

    logs/<run_id>/
        frames.bin     수신 원본 1109B 프레임. 손실 없음
        drive.csv      프레임당 제어 상태 한 줄
        objects.csv    프레임당 객체 수만큼 ― objects[].id 안정성 판정의 근거
        events.csv     이산 사건만 (리스폰, 신호 전이, 연결 끊김 ...)
        meta.json      실행 인자, 시작 시각, 커밋, 프레임 수
        route.csv      현장에서 받은 route CSV 원본 복사
        waypoints.csv  사용한 waypoint 파일 원본 복사

20Hz 루프 안에서 쓰이므로 블로킹하지 않는다. 전부 버퍼링하고 FLUSH_EVERY
프레임마다 한 번만 flush한다. drive_and_record는 수신 스레드에서 쓰므로
모든 쓰기를 락으로 감싼다.
"""

from __future__ import annotations

import csv
import json
import math
import os
import shutil
import threading
import time

from hlvtd_client import FRAME_SIZE, TL_STATE

FLUSH_EVERY = 20            # 프레임. 기존 FrameLog와 같은 주기
NAN = float('nan')

DRIVE_COLS = ['n', 't', 'ego_x', 'ego_y', 'heading', 'speed', 'seg', 'idx',
              'tgt_x', 'tgt_y', 'ld', 'steer', 'accel', 'tl_id', 'tl_state',
              'n_obj', 'tl_act', 'tl_dist',
              'ob_act', 'ob_kind', 'ob_id', 'ob_dist', 'ob_lat',
              'turn', 'turn_src', 'cw_act', 'cw_dist',
              'dt_act', 'dt_dist',
              'rz_ratio', 'rz_far', 'rz_in', 'fin_lon']
OBJECT_COLS = ['n', 't', 'id', 'x', 'y', 'z', 'heading', 'speed',
               'length', 'width', 'height', 'dist', 'is_ped']
EVENT_COLS = ['t', 'n', 'kind', 'detail', 'ego_x', 'ego_y', 'speed']


def repo_root() -> str:
    return os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))


def default_dir(mode: str) -> str:
    """logs/<mode>_YYYYmmdd_HHMMSS"""
    return os.path.join(repo_root(), 'logs', time.strftime(f'{mode}_%Y%m%d_%H%M%S'))


def git_commit() -> str:
    """서브프로세스 없이 .git에서 현재 커밋을 읽는다. 실패하면 빈 문자열.

    로그에 코드 버전을 박아두면 나중에 "이 로그는 어느 코드에서 나왔나"를
    되짚을 수 있다. 셋업 시간을 늘리지 않으려고 subprocess는 쓰지 않는다.
    """
    try:
        git = os.path.join(repo_root(), '.git')
        with open(os.path.join(git, 'HEAD'), encoding='utf-8') as fh:
            head = fh.read().strip()
        if not head.startswith('ref: '):
            return head
        with open(os.path.join(git, head[5:]), encoding='utf-8') as fh:
            return fh.read().strip()
    except OSError:
        return ''


def _fmt(v) -> str:
    if isinstance(v, float):
        return f'{v:.4f}'
    return v


class SessionLog:
    """한 세션의 기록. path가 비면 모든 메서드가 no-op이 된다."""

    def __init__(self, path: str | None, mode: str = 'run',
                 meta: dict | None = None):
        self.dir = path or ''
        self.enabled = bool(path)
        self.t0 = time.monotonic()
        self.n_frames = 0
        self._lock = threading.Lock()
        self._since_flush = 0
        self._prev_tl: tuple[int, int] | None = None
        self._prev_seg: str | None = None
        self._raw = self._drive_fh = self._obj_fh = self._ev_fh = None

        if not self.enabled:
            return

        os.makedirs(self.dir, exist_ok=True)
        self._raw = open(os.path.join(self.dir, 'frames.bin'), 'wb')
        self._drive_fh, self._drive = self._open_csv('drive.csv', DRIVE_COLS)
        self._obj_fh, self._obj = self._open_csv('objects.csv', OBJECT_COLS)
        self._ev_fh, self._ev = self._open_csv('events.csv', EVENT_COLS)

        self._meta = {
            'run_id': os.path.basename(os.path.normpath(self.dir)),
            'mode': mode,
            'started': time.strftime('%Y-%m-%d %H:%M:%S'),
            'started_epoch': time.time(),
            'git_commit': git_commit(),
            'frame_size': FRAME_SIZE,
            'n_frames': 0,
            'duration_s': 0.0,
        }
        self._meta.update(meta or {})
        self._write_meta()

    def _open_csv(self, name: str, cols: list[str]):
        fh = open(os.path.join(self.dir, name), 'w', newline='', encoding='utf-8')
        w = csv.writer(fh)
        w.writerow(cols)
        return fh, w

    def _write_meta(self) -> None:
        with open(os.path.join(self.dir, 'meta.json'), 'w', encoding='utf-8') as fh:
            json.dump(self._meta, fh, ensure_ascii=False, indent=2)

    # ------------------------------------------------------------------
    # 수신 원본

    def raw_sink(self, raw: bytes) -> None:
        """Client(raw_sink=...)에 그대로 넘긴다. 파싱 전에 호출되므로
        파싱이 실패하는 프레임도 남는다."""
        if self._raw is None:
            return
        with self._lock:
            self._raw.write(raw)

    # ------------------------------------------------------------------
    # 프레임

    def add_frame(self, frame, speed: float, *, t: float | None = None,
                  seg: str = '', idx: int = -1,
                  tgt_x: float = NAN, tgt_y: float = NAN, ld: float = NAN,
                  steer: float = 0.0, accel: float = 0.0,
                  tl_act: str = '', tl_dist: float = NAN,
                  ob_act: str = '', ob_kind: str = '', ob_id: int = -1,
                  ob_dist: float = NAN, ob_lat: float = NAN,
                  turn: int = 0, turn_src: str = '',
                  cw_act: str = '', cw_dist: float = NAN,
                  dt_act: str = '', dt_dist: float = NAN,
                  rz_ratio: float = NAN, rz_far: float = NAN,
                  rz_in: int = 0, fin_lon: float = NAN) -> None:
        """한 프레임의 상태와 그때 보낸 명령을 기록한다.

        신호등 전이와 구간 전환은 여기서 자동으로 events.csv에도 남긴다.
        호출자가 잊어버릴 수 있는 것을 데이터가 있는 쪽에서 처리한다.
        """
        if not self.enabled:
            return
        t = self.now() if t is None else t

        with self._lock:
            self.n_frames += 1
            n = self.n_frames

            self._drive.writerow([_fmt(v) for v in (
                n, t, frame.ego_x, frame.ego_y, frame.ego_heading, speed,
                seg, idx, tgt_x, tgt_y, ld, steer, accel,
                frame.tl_id, frame.tl_state, len(frame.objects),
                tl_act, tl_dist,
                ob_act, ob_kind, ob_id, ob_dist, ob_lat,
                turn, turn_src, cw_act, cw_dist,
                dt_act, dt_dist, rz_ratio, rz_far, rz_in, fin_lon)])

            for o in frame.objects:
                self._obj.writerow([_fmt(v) for v in (
                    n, t, o.id, o.x, o.y, o.z, o.heading, o.speed,
                    o.length, o.width, o.height,
                    o.distance_from(frame.ego_x, frame.ego_y),
                    1 if o.is_pedestrian else 0)])

            tl = (frame.tl_id, frame.tl_state)
            if self._prev_tl is not None and tl != self._prev_tl:
                pid, pstate = self._prev_tl
                detail = (f'{TL_STATE.get(pstate, pstate)}->'
                          f'{TL_STATE.get(tl[1], tl[1])}')
                if pid != tl[0]:
                    detail += f' (id {pid}->{tl[0]})'
                self._event_locked(t, 'tl_change', detail,
                                   frame.ego_x, frame.ego_y, speed)
            self._prev_tl = tl

            if seg and self._prev_seg is not None and seg != self._prev_seg:
                self._event_locked(t, 'seg_change', f'{self._prev_seg}->{seg}',
                                   frame.ego_x, frame.ego_y, speed)
            if seg:
                self._prev_seg = seg

            self._since_flush += 1
            if self._since_flush >= FLUSH_EVERY:
                self._since_flush = 0
                self._flush_locked()

    # ------------------------------------------------------------------
    # 이벤트

    def event(self, kind: str, detail: str = '', *, t: float | None = None,
              ego_x: float = NAN, ego_y: float = NAN,
              speed: float = NAN) -> None:
        """이산 사건.

        kind는 connect / disconnect / respawn / tl_change / seg_change /
        finish / error / note,

        신호등 정지(stoplines.py)가 내는
        tl_stop / tl_go / tl_pause / tl_late / tl_yellow_go / tl_fallback /
        tl_timeout / tl_green_stuck / tl_unknown_state,

        충돌 회피(obstacles.py)가 내는
        ped_stop_actual / ped_stop_predicted / ped_clear / ped_timeout /
        veh_follow / veh_stop / veh_timeout / obstacle_slow /
        obstacle_clear / obj_id_reuse,

        방향지시등(turnsignal.py)이 내는
        turn_on / turn_off / turn_conflict,

        횡단보도 정차 금지(crosswalks.py)가 내는
        xw_guard / xw_guard_infeasible / xw_dwell / crosswalk_escape /
        xw_escape_blocked / xw_clear,

        영구 정지 객체 우회(detour.py)가 내는
        detour_start / detour_done / detour_failed / detour_impossible /
        detour_tight / detour_solid_lc / detour_ld_boost / detour_ld_restore /
        last_resort_pass / last_resort_clear,

        붉은 보호구역(redzone.py)이 내는
        redzone_enter / redzone_exit / redzone_hold / redzone_hold_end /
        redzone_camera_lost / redzone_camera_back / redzone_flapping,

        종료선(finish.py)이 내는 finish_line_passed / finish_short,
        pure_pursuit이 ego(0,0)에 내는 sim_end / ego_origin,
        obstacles가 종료 구간 라바콘에 내는 finish_zone_ignore 중 하나.
        """
        if not self.enabled:
            return
        t = self.now() if t is None else t
        with self._lock:
            self._event_locked(t, kind, detail, ego_x, ego_y, speed)
            self._ev_fh.flush()         # 이벤트는 드무니 바로 내린다

    def _event_locked(self, t, kind, detail, ego_x, ego_y, speed) -> None:
        self._ev.writerow([_fmt(v) for v in
                           (t, self.n_frames, kind, detail, ego_x, ego_y, speed)])

    def respawn(self, frame, prev, speed: float = NAN,
                t: float | None = None) -> None:
        """리스폰. 이동 전후 위치를 둘 다 남긴다 ― 지금은 콘솔에만 찍히고 사라진다."""
        jump = math.hypot(frame.ego_x - prev.ego_x, frame.ego_y - prev.ego_y)
        self.event('respawn',
                   f'{jump:.1f}m ({prev.ego_x:.2f},{prev.ego_y:.2f})->'
                   f'({frame.ego_x:.2f},{frame.ego_y:.2f})',
                   t=t, ego_x=frame.ego_x, ego_y=frame.ego_y, speed=speed)

    # ------------------------------------------------------------------

    def now(self) -> float:
        return time.monotonic() - self.t0

    def copy_input(self, src: str | None, name: str) -> None:
        """현장에서 받은 route CSV처럼 재현 불가능한 입력을 세션 안에 복사한다."""
        if not self.enabled or not src or not os.path.exists(src):
            return
        try:
            shutil.copyfile(src, os.path.join(self.dir, name))
        except OSError as exc:
            print(f'[log] {name} 복사 실패: {exc}')

    def note(self, key: str, value) -> None:
        if self.enabled:
            self._meta[key] = value

    def _flush_locked(self) -> None:
        for fh in (self._raw, self._drive_fh, self._obj_fh, self._ev_fh):
            if fh:
                fh.flush()
        # meta도 같이 갱신한다. 현장에서 프로세스를 강제 종료하면 close()가
        # 돌지 않아 n_frames가 0으로 남는다 — 실제로 그렇게 됐다.
        self._meta['n_frames'] = self.n_frames
        self._meta['duration_s'] = round(self.now(), 3)
        self._write_meta()

    def close(self) -> None:
        if not self.enabled:
            return
        with self._lock:
            self._meta['ended'] = time.strftime('%Y-%m-%d %H:%M:%S')
            self._flush_locked()            # n_frames / duration도 여기서 갱신
            for fh in (self._raw, self._drive_fh, self._obj_fh, self._ev_fh):
                if fh:
                    fh.close()
            self._raw = self._drive_fh = self._obj_fh = self._ev_fh = None
            self.enabled = False
