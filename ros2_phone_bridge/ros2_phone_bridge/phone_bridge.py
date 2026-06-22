"""phone_bridge — 안드로이드 폰(촬영 카메라) ↔ Jetson USB 브리지.

데이터 흐름:
  [폰 후면 카메라] --scrcpy(USB)--> /dev/videoN(v4l2loopback) --cv2--> /phone/image
  [폰 배터리] --adb dumpsys battery--> /phone/battery (BatteryState)
  /phone_cmd(String) 수신:
    record_toggle : Jetson에서 ffmpeg로 v4l2 장치를 mp4 녹화 시작/종료.
                    종료 시 adb push 로 폰 저장소(DCIM)에 파일 전송 → /phone/recording
    zoom_in/zoom_out/focus : 자리만(로그). scrcpy 카메라 런타임 줌 제어가
                    까다로워 추후 구현(앱/Camera2 경로 확정 후).

설계 메모:
  - 영상 캡처와 녹화를 분리: 항상 v4l2 장치를 읽어 미리보기(/phone/image)를
    내보내고, 녹화는 같은 장치를 ffmpeg가 추가로 읽어 파일로 저장한다.
    (단일 카메라 파이프라인 → 안정적. 폰 카메라 앱과 충돌 없음.)
  - scrcpy 실행은 launch/스크립트가 담당(권장). manage_scrcpy:=true 면
    이 노드가 직접 scrcpy 서브프로세스를 띄운다(개발 편의).
  - mock:=true 면 폰/adb/scrcpy/cv2 없이 합성 영상+가짜 배터리로 동작
    (UI/배선 점검용).

토픽:
  발행: /phone/image (sensor_msgs/Image, bgr8)
        /phone/battery (sensor_msgs/BatteryState)
        /phone/recording (std_msgs/Bool)
  구독: /phone_cmd (std_msgs/String)
"""
import os
import re
import shlex
import signal
import subprocess
import threading
import time
from datetime import datetime
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32
from sensor_msgs.msg import Image, BatteryState

from .util import parse_battery_level


class PhoneBridge(Node):
    def __init__(self):
        super().__init__("phone_bridge")
        # ---- 파라미터 ----
        self.declare_parameter("mock", False)
        self.declare_parameter("video_device", "/dev/video2")  # scrcpy v4l2 sink
        self.declare_parameter("publish_rate", 20.0)           # /phone/image Hz
        self.declare_parameter("battery_period", 15.0)         # 배터리 폴링 s
        self.declare_parameter("adb_serial", "")               # 다중 기기 시 지정
        self.declare_parameter("record_dir", "/tmp/solcam_rec")
        self.declare_parameter("phone_push_dir", "/sdcard/DCIM/solcam")
        self.declare_parameter("manage_scrcpy", False)
        self.declare_parameter("scrcpy_extra", "")             # 추가 인자
        self.declare_parameter("scrcpy_bin", "scrcpy")         # scrcpy 실행파일 경로
        self.declare_parameter("adb_bin", "adb")               # adb 실행파일 경로
        self.declare_parameter("camera_size", "1280x720")      # scrcpy 카메라 해상도
        self.declare_parameter("camera_facing", "back")        # front/back
        self.declare_parameter("scrcpy_watchdog", True)        # 죽으면 자동 재기동
        self.declare_parameter("v4l2_reset_cmd", "")           # 꼬임 자동복구 명령(예: sudo .../reset_v4l2.sh 2)
        self.declare_parameter("wedge_timeout", 10.0)          # s, scrcpy 생존+프레임없음 N초 → 장치 꼬임 판정
        self.declare_parameter("camera_zoom", 1.0)             # 시작 줌 배율(scrcpy --camera-zoom)
        self.declare_parameter("zoom_step", 0.25)              # 줌 1스텝 폭
        self.declare_parameter("zoom_min", 1.0)
        self.declare_parameter("zoom_max", 8.0)                # scrcpy 가 카메라 실제 범위로 클램프
        self.declare_parameter("zoom_apply_delay", 0.4)        # s, 줌 입력 멈춘 뒤 scrcpy 재기동까지(디바운스)

        gp = self.get_parameter
        self.mock = bool(gp("mock").value)
        self.video_device = str(gp("video_device").value)
        self.publish_rate = float(gp("publish_rate").value)
        self.battery_period = float(gp("battery_period").value)
        self.adb_serial = str(gp("adb_serial").value)
        self.record_dir = str(gp("record_dir").value)
        self.phone_push_dir = str(gp("phone_push_dir").value)
        self.manage_scrcpy = bool(gp("manage_scrcpy").value)
        self.scrcpy_extra = str(gp("scrcpy_extra").value)
        self.scrcpy_bin = str(gp("scrcpy_bin").value)
        self.adb_bin = str(gp("adb_bin").value)
        self.camera_size = str(gp("camera_size").value)
        self.camera_facing = str(gp("camera_facing").value)
        self.scrcpy_watchdog = bool(gp("scrcpy_watchdog").value)
        self.v4l2_reset_cmd = str(gp("v4l2_reset_cmd").value)
        self.wedge_timeout = float(gp("wedge_timeout").value)
        self.zoom = float(gp("camera_zoom").value)            # 현재 적용된 줌
        self.zoom_target = self.zoom                          # 디바운스 목표 줌
        self.zoom_step = float(gp("zoom_step").value)
        self.zoom_min = float(gp("zoom_min").value)
        self.zoom_max = float(gp("zoom_max").value)
        self.zoom_apply_delay = float(gp("zoom_apply_delay").value)
        self._last_zoom_cmd = 0.0

        # ---- 상태 ----
        self.cap = None                 # cv2.VideoCapture
        self.scrcpy_proc = None         # scrcpy subprocess
        self.rec_proc = None            # ffmpeg subprocess(녹화, stdin 파이프)
        self.rec_path = None
        self.recording = False
        self._rec_lock = threading.Lock()   # 녹화 start/stop ↔ 캡처스레드 write 직렬화
        self._mock_phase = 0
        self._battery = 100
        # 영상 재연결(케이블 흔들림/scrcpy 재시작 대비)
        self._read_fail = 0
        self._reopen_after = 5        # 연속 read 실패 N회 후 재연결
        self._last_reopen = 0.0
        self._reopen_interval = 1.0   # 재연결 시도 최소 간격 s
        self._last_scrcpy_restart = 0.0
        self._scrcpy_restart_interval = 3.0  # scrcpy 재기동 최소 간격 s
        self._last_good_frame = time.time()  # 마지막 정상 프레임 시각(꼬임 감지)
        self._last_v4l2_reset = 0.0
        self._v4l2_reset_interval = 15.0     # v4l2 리로드 최소 간격 s
        self._wedge_warned = False
        self._zoom_warned = False
        self._cap_lock = threading.RLock()   # cv2.VideoCapture 는 thread-unsafe →
        # 캡처 스레드의 read/open 과 워치독/health 스레드의 release 를 직렬화(케이블 플랩 시 세그폴트 방지)

        # ---- pub/sub ----
        self.pub_img = self.create_publisher(Image, "/phone/image", 1)
        self.pub_batt = self.create_publisher(BatteryState, "/phone/battery", 10)
        self.pub_rec = self.create_publisher(Bool, "/phone/recording", 10)
        self.pub_zoom = self.create_publisher(Float32, "/phone/zoom", 10)   # 현재/목표 줌 배율(UI 표시용)
        self.create_subscription(String, "/phone_cmd", self._cmd_cb, 10)

        # ---- 영상 소스 준비 ----
        self._cap_stop = False
        self._cap_thread = None
        if not self.mock:
            if self.manage_scrcpy:
                self._start_scrcpy()
            self._open_capture()
            # 캡처는 별도 스레드에서 — cap.read() 블로킹이 ROS executor(워치독/배터리)를
            # 멈추지 않게 한다(영상 끊김 시 굳는 현상 방지).
            self._cap_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._cap_thread.start()

        # ---- 타이머 ----
        if self.mock:
            self.create_timer(1.0 / max(1.0, self.publish_rate), self._tick_image_mock)
        self.create_timer(self.battery_period, self._tick_battery)
        if not self.mock and self.manage_scrcpy and self.scrcpy_watchdog:
            self.create_timer(2.0, self._tick_scrcpy_watchdog)  # scrcpy 생존 감시
        if not self.mock and self.manage_scrcpy:
            self.create_timer(3.0, self._tick_health)  # 장치 꼬임 감지→자동 리로드
            self.create_timer(0.2, self._tick_zoom)    # 줌 디바운스 적용(입력 멈추면 scrcpy 1회 재기동)
        self._tick_battery()  # 시작 즉시 1회
        self._publish_recording()  # 초기 false 알림
        self.pub_zoom.publish(Float32(data=float(self.zoom)))  # 초기 줌 알림

        mode = "MOCK" if self.mock else f"v4l2={self.video_device}"
        self.get_logger().info(f"phone_bridge 시작 ({mode})")

    # ================= 영상 =================
    def _start_scrcpy(self):
        """폰 후면 카메라를 v4l2 sink 로 스트림하는 scrcpy 자식 프로세스.

        - 띄우기 전 잔여 scrcpy/폰서버를 먼저 정리(고아·장치 꼬임 방지).
        - start_new_session=True 로 별도 프로세스 그룹 → 종료 시 그룹째 정리.
        - 고정 sleep 없이, 캡처는 _open_capture 의 포맷 준비 게이트가 처리.
        """
        self._kill_scrcpy()  # 잔여 정리(고아 scrcpy/폰서버)
        cmd = [self.scrcpy_bin, "--video-source=camera",
               f"--camera-facing={self.camera_facing}",
               f"--camera-size={self.camera_size}",
               f"--v4l2-sink={self.video_device}",
               "--no-audio", "--no-window", "--no-playback"]
        # --camera-zoom 은 scrcpy 4.0+ 전용. 3.x 에선 이 플래그가 있으면 즉시 종료되므로
        #  줌이 1.0 이 아닐 때(실제 줌 사용 시)만 붙인다. (잿슨 scrcpy=3.3.4)
        if abs(self.zoom - 1.0) > 1e-3:
            cmd.insert(4, f"--camera-zoom={self.zoom:.2f}")
        if self.adb_serial:
            cmd += ["-s", self.adb_serial]
        if self.scrcpy_extra:
            cmd += shlex.split(self.scrcpy_extra)
        env = dict(os.environ)
        env["ADB"] = self.adb_bin
        sc_dir = os.path.dirname(self.scrcpy_bin)
        if sc_dir:
            env["PATH"] = sc_dir + os.pathsep + env.get("PATH", "")
        env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
        try:
            self.scrcpy_proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True, env=env)
            self._last_scrcpy_restart = time.time()
            self._last_good_frame = time.time()   # 재기동 직후 grace
            self.get_logger().info(
                f"scrcpy 시작(pid={self.scrcpy_proc.pid}, {self.camera_size})")
        except FileNotFoundError:
            self.get_logger().error(
                f"scrcpy 실행 실패 — scrcpy_bin='{self.scrcpy_bin}' 확인")
            self.scrcpy_proc = None

    def _kill_scrcpy(self):
        """관리 중인 scrcpy를 그룹 단위로 graceful 종료하고, 잔여 scrcpy/폰서버 정리.
        강제 kill 로 인한 고아 프로세스·v4l2 장치 꼬임을 막는다."""
        proc = self.scrcpy_proc
        if proc is not None and proc.poll() is None:
            try:
                pg = os.getpgid(proc.pid)
                os.killpg(pg, signal.SIGTERM)      # graceful(scrcpy가 폰서버 정리)
                try:
                    proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    os.killpg(pg, signal.SIGKILL)  # 폴백
            except (ProcessLookupError, PermissionError):
                pass
        self.scrcpy_proc = None
        # 우리가 소유하지 않은 잔여 로컬 scrcpy(같은 sink) 정리
        try:
            subprocess.run(["pkill", "-f", f"v4l2-sink={self.video_device}"],
                           capture_output=True, timeout=3.0)
        except Exception:
            pass
        # 폰쪽 scrcpy 서버 잔여 정리(HAL 스턱 방지)
        try:
            subprocess.run(self._adb("shell", "pkill", "-f",
                                     "com.genymobile.scrcpy"),
                           capture_output=True, timeout=4.0)
        except Exception:
            pass

    def _device_format_ready(self):
        """v4l2 장치에 유효 포맷이 잡혔는지(writer가 헤더를 썼는지) 확인.
        exclusive_caps=1 에선 reader가 writer보다 먼저 열면 포맷이 잠기므로,
        준비된 뒤에만 캡처를 연다(장치 꼬임 방지)."""
        try:
            r = subprocess.run(["v4l2-ctl", "-d", self.video_device,
                                "--get-fmt-video"],
                               capture_output=True, text=True, timeout=3.0)
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return True  # v4l2-ctl 없으면 게이트 생략(기존 동작 유지)
        if r.returncode != 0:
            return False
        m = re.search(r"Width/Height\s*:\s*(\d+)/(\d+)", r.stdout)
        return bool(m and int(m.group(1)) > 0 and int(m.group(2)) > 0)

    def _tick_scrcpy_watchdog(self):
        """scrcpy가 죽었으면(케이블 빠짐 등) 자동 재기동."""
        if self.scrcpy_proc is None or self.scrcpy_proc.poll() is None:
            return  # 미관리이거나 살아있음
        now = time.time()
        if now - self._last_scrcpy_restart < self._scrcpy_restart_interval:
            return
        self.get_logger().warn("scrcpy 종료 감지 → 자동 재기동")
        self._release_cap()   # 블로킹된 read 깨우고, 포맷 재준비 후 캡처 스레드가 재오픈
        self._start_scrcpy()

    def _tick_health(self):
        """scrcpy는 살아있는데 N초 넘게 프레임이 없으면 v4l2 장치 꼬임으로 보고,
        v4l2_reset_cmd 가 설정돼 있으면 v4l2loopback 을 리로드해 자동 복구한다.
        (scrcpy 급단절/케이블 뽑힘 후 /dev/videoN VIDIOC_G_FMT 잠김 대응.)"""
        if self.recording:
            return  # 녹화 중엔 ffmpeg도 장치를 잡고 있어 판단 보류
        if time.time() - self._last_good_frame < self.wedge_timeout:
            return  # 프레임 정상
        # scrcpy 가 죽은 거면 scrcpy 워치독이 먼저 살린다
        if self.scrcpy_proc is None or self.scrcpy_proc.poll() is not None:
            return
        if self._device_format_ready():
            return  # 포맷은 정상 — 프레임만 잠깐 비는 것, 더 기다림
        # 여기 도달 = scrcpy 살아있는데 포맷 못 잡음 = 장치 꼬임
        now = time.time()
        if now - self._last_v4l2_reset < self._v4l2_reset_interval:
            return
        if not self.v4l2_reset_cmd:
            if not self._wedge_warned:
                self.get_logger().error(
                    "v4l2 장치 꼬임 감지 — 수동 리로드 필요: "
                    "sudo modprobe -r v4l2loopback && sudo modprobe v4l2loopback "
                    "video_nr=N card_label=solcam_phone exclusive_caps=1. "
                    "v4l2_reset_cmd 파라미터를 주면 자동 복구함.")
                self._wedge_warned = True
            return
        self._last_v4l2_reset = now
        self.get_logger().warn("v4l2 장치 꼬임 → 자동 리로드 시도")
        self._release_cap()
        self._kill_scrcpy()       # 모듈이 사용 중이면 리로드 안 되므로 먼저 정리
        try:
            subprocess.run(shlex.split(self.v4l2_reset_cmd), timeout=15.0)
        except Exception as e:
            self.get_logger().error(f"v4l2 리로드 실패: {e}")
            return
        self._last_good_frame = time.time()   # 리로드 후 grace
        self._start_scrcpy()
        self.get_logger().info("v4l2 리로드 + scrcpy 재기동 완료")

    def _open_capture(self):
        try:
            import cv2
            self._cv2 = cv2
        except ImportError:
            self.get_logger().error("python3-opencv(cv2) 미설치 — 영상 발행 불가")
            self.cap = None
            return
        # 기존 캡처가 있으면 먼저 해제(재연결 시 핸들 누수 방지)
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
        # writer(scrcpy) 포맷 준비 전에는 열지 않음 — exclusive_caps 장치 꼬임 방지
        if not self.mock and not self._device_format_ready():
            self.cap = None
            return
        # ★백엔드를 V4L2 로 못박는다. 미지정 시 잿슨 OpenCV 가 GStreamer 백엔드로
        #  열어 v4l2loopback(YU12) 협상이 프레임당 ~0.45s 걸려 ~2Hz 로 추락한다.
        #  CAP_V4L2 로 열면 장치 fps(30) 그대로 읽힌다(BUFFERSIZE 설정도 V4L2 경로에서 먹음).
        cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().warn(
                f"{self.video_device} 열기 실패 — scrcpy/v4l2loopback 확인")
            try:
                cap.release()
            except Exception:
                pass
            self.cap = None
            return
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 지연/스테일 프레임 최소화
        except Exception:
            pass
        self.cap = cap

    def _tick_image_mock(self):
        self.pub_img.publish(self._to_image_msg(self._mock_frame()))

    def _capture_loop(self):
        """별도 스레드: v4l2 장치에서 프레임을 읽어 /phone/image 로 발행.
        read() 가 블로킹돼도 ROS executor(워치독/배터리)는 영향을 받지 않는다."""
        pub_period = 1.0 / max(1.0, self.publish_rate)
        last_pub = 0.0
        while rclpy.ok() and not self._cap_stop:
            with self._cap_lock:            # read/open/release 직렬화(thread-unsafe 세그폴트 방지)
                if self.cap is None:
                    self._try_reopen()      # 포맷 준비됐을 때만 열림(게이트)
                opened = self.cap is not None
                if opened:
                    try:
                        ok, frame = self.cap.read()
                    except Exception:
                        ok, frame = False, None
                else:
                    ok, frame = False, None
            if not opened:
                time.sleep(self._reopen_interval)
                continue
            if not ok or frame is None:
                self._read_fail += 1
                if self._read_fail >= self._reopen_after:
                    self._release_cap()     # 닫고 다음 루프에서 재오픈
                    self._read_fail = 0
                time.sleep(0.05)
                continue
            self._read_fail = 0
            self._last_good_frame = time.time()
            self._wedge_warned = False
            # cv2.read 가 장치 fps 로 페이싱 → 발행만 publish_rate 로 throttle.
            now = time.time()
            if now - last_pub >= pub_period:
                self.pub_img.publish(self._to_image_msg(frame))
                if self.recording:          # 녹화도 같은 캐던스 → 영상 속도 정확
                    self._write_rec_frame(frame)
                last_pub = now

    def _release_cap(self):
        with self._cap_lock:
            if self.cap is not None:
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None

    def _try_reopen(self):
        """끊긴 v4l2 장치를 다시 연다(포맷 준비 게이트는 _open_capture 안에서)."""
        now = time.time()
        if now - self._last_reopen < self._reopen_interval:
            return
        self._last_reopen = now
        self._open_capture()
        if self.cap is not None:
            self._read_fail = 0
            self.get_logger().info(f"{self.video_device} 재연결 성공")

    def _mock_frame(self, w=640, h=360):
        """움직이는 그라데이션 합성 프레임(BGR)."""
        self._mock_phase = (self._mock_phase + 4) % 256
        x = np.linspace(0, 255, w).astype(np.int32)
        row = ((x + self._mock_phase) % 256).astype(np.uint8)   # int32→uint8 (오버플로 방지)
        g = np.tile(row, (h, 1)).astype(np.int32)
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        frame[:, :, 0] = g.astype(np.uint8)              # B
        frame[:, :, 1] = (g // 2 + 64).astype(np.uint8)  # G
        frame[:, :, 2] = (255 - g).astype(np.uint8)      # R
        return frame

    def _to_image_msg(self, frame):
        h, w = frame.shape[:2]
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "phone_camera"
        msg.height = h
        msg.width = w
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = np.ascontiguousarray(frame, dtype=np.uint8).tobytes()
        return msg

    # ================= 배터리 =================
    def _adb(self, *args):
        cmd = [self.adb_bin]
        if self.adb_serial:
            cmd += ["-s", self.adb_serial]
        return cmd + list(args)

    def _tick_battery(self):
        if self.mock:
            self._battery = max(0, self._battery - 1)
            self._publish_battery(self._battery)
            return
        try:
            out = subprocess.run(self._adb("shell", "dumpsys", "battery"),
                                 capture_output=True, text=True, timeout=4.0)
            lvl = parse_battery_level(out.stdout)
            if lvl is not None:
                self._publish_battery(lvl)
        except (FileNotFoundError, subprocess.TimeoutExpired) as e:
            self.get_logger().warn(f"배터리 조회 실패: {e}")

    def _publish_battery(self, level):
        m = BatteryState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.percentage = float(level) / 100.0
        m.present = True
        self.pub_batt.publish(m)

    # ================= 명령 =================
    def _cmd_cb(self, msg):
        cmd = msg.data.strip()
        if cmd == "record_toggle":
            self._toggle_record()
        elif cmd == "zoom_in":
            self._bump_zoom(+self.zoom_step)
        elif cmd == "zoom_out":
            self._bump_zoom(-self.zoom_step)
        else:
            self.get_logger().warn(f"알 수 없는 phone_cmd: {cmd}")

    # ================= 줌 (scrcpy --camera-zoom 재기동, 디바운스) =================
    def _bump_zoom(self, delta):
        """줌 목표만 갱신(클램프). 실제 적용(scrcpy 재기동)은 _tick_zoom 이
        입력이 멈춘 뒤 한 번만 → 꾹 누르는 동안 깜빡임 없이 램프."""
        if not self.manage_scrcpy:   # _tick_zoom 이 managed 일 때만 돌아 적용됨
            if not self._zoom_warned:
                self.get_logger().warn(
                    "줌은 manage_scrcpy:=true 에서만 적용됨(scrcpy 재기동 방식). "
                    "solcam.sh run 으로 띄우세요.")
                self._zoom_warned = True
            return
        tgt = max(self.zoom_min, min(self.zoom_max, self.zoom_target + delta))
        self.zoom_target = round(tgt, 2)
        self._last_zoom_cmd = time.time()
        self.pub_zoom.publish(Float32(data=float(self.zoom_target)))   # UI에 목표 배율 실시간 표시

    def _tick_zoom(self):
        if self.recording:
            return   # 녹화 중엔 scrcpy 재기동(줌 적용) 미룸 → 영상 끊김 방지
        if abs(self.zoom_target - self.zoom) < 1e-3:
            return
        if time.time() - self._last_zoom_cmd < self.zoom_apply_delay:
            return  # 아직 입력 중 — 멈출 때까지 대기(디바운스)
        self.zoom = self.zoom_target
        self.get_logger().info(f"줌 {self.zoom:.2f}x 적용 → scrcpy 재기동")
        self._release_cap()
        self._start_scrcpy()   # 새 --camera-zoom 으로 재기동(_kill_scrcpy 내부 호출)

    def _publish_recording(self):
        self.pub_rec.publish(Bool(data=self.recording))

    def _toggle_record(self):
        if self.recording:
            self._stop_record()
        else:
            self._start_record()

    def _start_record(self):
        if self.mock:
            self.recording = True
            self._publish_recording()
            self.get_logger().info("[MOCK] 녹화 시작")
            return
        with self._rec_lock:
            if self.recording:
                return
            self.recording = True
            self.rec_proc = None     # 첫 프레임에서 lazy open(해상도 확정 후)
        self._publish_recording()
        self.get_logger().info("녹화 시작 (첫 프레임부터 기록)")

    def _rec_open(self, w, h):
        """캡처 프레임을 mp4 로 인코딩하는 ffmpeg(stdin 파이프) 기동.
        장치를 다시 열지 않고 phone_bridge 가 이미 읽는 프레임을 그대로 받는다."""
        os.makedirs(self.record_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.rec_path = os.path.join(self.record_dir, f"solcam_{ts}.mp4")
        fps = max(1, int(round(self.publish_rate)))   # 발행 캐던스와 동일 → 속도 정확
        cmd = ["ffmpeg", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24",
               "-s", f"{w}x{h}", "-r", str(fps), "-i", "-",
               "-c:v", "libx264", "-preset", "ultrafast", "-pix_fmt", "yuv420p",
               self.rec_path]
        try:
            self.rec_proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                             stdout=subprocess.DEVNULL,
                                             stderr=subprocess.DEVNULL)
            self.get_logger().info(f"녹화 인코더 시작 → {self.rec_path} ({w}x{h}@{fps})")
        except FileNotFoundError:
            self.get_logger().error("ffmpeg 미설치 — 녹화 불가")
            self.rec_proc = None

    def _write_rec_frame(self, frame):
        with self._rec_lock:
            if not self.recording:
                return
            if self.rec_proc is None:
                h, w = frame.shape[:2]
                self._rec_open(w, h)
                if self.rec_proc is None:
                    return
            try:
                self.rec_proc.stdin.write(
                    np.ascontiguousarray(frame, dtype=np.uint8).tobytes())
            except (BrokenPipeError, ValueError, OSError):
                self.get_logger().warn("녹화 인코더 파이프 끊김 → 녹화 중단")
                self._finalize_locked(push=False)

    def _stop_record(self):
        if self.mock:
            self.recording = False
            self._publish_recording()
            self.get_logger().info("[MOCK] 녹화 종료")
            return
        with self._rec_lock:
            self.recording = False
            self._finalize_locked(push=True)   # stdin close → mp4 마무리 → 폰 전송
        self._publish_recording()

    def _finalize_locked(self, push):
        """녹화 ffmpeg 를 정상 종료(stdin close → 파일 마무리)하고 옵션으로 폰 전송.
        ★_rec_lock 을 잡은 채 호출할 것."""
        proc = self.rec_proc
        path = self.rec_path
        self.rec_proc = None
        if proc is None:
            return   # 프레임이 한 장도 안 들어와 인코더 미기동
        try:
            proc.stdin.close()       # 입력 종료 → ffmpeg 가 mp4 헤더 마무리(손상 없음)
            proc.wait(timeout=5.0)
        except Exception:
            try:
                proc.terminate()
            except Exception:
                pass
        self.get_logger().info(f"녹화 종료 → {path}")
        if push:
            threading.Thread(target=self._push_to_phone, args=(path,),
                             daemon=True).start()

    def _push_to_phone(self, path):
        if not path or not os.path.exists(path):
            return
        try:
            subprocess.run(self._adb("shell", "mkdir", "-p", self.phone_push_dir),
                           capture_output=True, timeout=10.0)
            dst = self.phone_push_dir.rstrip("/") + "/" + os.path.basename(path)
            r = subprocess.run(self._adb("push", path, dst),
                               capture_output=True, text=True, timeout=120.0)
            if r.returncode == 0:
                self.get_logger().info(f"폰 전송 완료: {dst}")
            else:
                self.get_logger().warn(f"폰 전송 실패: {r.stderr.strip()}")
        except (FileNotFoundError, subprocess.TimeoutExpired) as e:
            self.get_logger().warn(f"폰 전송 오류: {e}")

    # ================= 종료 =================
    def destroy_node(self):
        self._cap_stop = True
        if self._cap_thread is not None:
            self._cap_thread.join(timeout=2.0)
        with self._rec_lock:          # 진행 중 녹화 마무리(종료 시 전송은 생략)
            self.recording = False
            self._finalize_locked(push=False)
        self._release_cap()
        if self.manage_scrcpy:
            self._kill_scrcpy()   # graceful 그룹 종료 + 폰서버 정리(고아 방지)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PhoneBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
