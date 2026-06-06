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
import subprocess
import threading
import time
from datetime import datetime
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
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

        # ---- 상태 ----
        self.cap = None                 # cv2.VideoCapture
        self.scrcpy_proc = None         # scrcpy subprocess
        self.rec_proc = None            # ffmpeg subprocess
        self.rec_path = None
        self.recording = False
        self._mock_phase = 0
        self._battery = 100

        # ---- pub/sub ----
        self.pub_img = self.create_publisher(Image, "/phone/image", 1)
        self.pub_batt = self.create_publisher(BatteryState, "/phone/battery", 10)
        self.pub_rec = self.create_publisher(Bool, "/phone/recording", 10)
        self.create_subscription(String, "/phone_cmd", self._cmd_cb, 10)

        # ---- 영상 소스 준비 ----
        if not self.mock:
            if self.manage_scrcpy:
                self._start_scrcpy()
            self._open_capture()

        # ---- 타이머 ----
        self.create_timer(1.0 / max(1.0, self.publish_rate), self._tick_image)
        self.create_timer(self.battery_period, self._tick_battery)
        self._tick_battery()  # 시작 즉시 1회
        self._publish_recording()  # 초기 false 알림

        mode = "MOCK" if self.mock else f"v4l2={self.video_device}"
        self.get_logger().info(f"phone_bridge 시작 ({mode})")

    # ================= 영상 =================
    def _start_scrcpy(self):
        """폰 후면 카메라를 v4l2 sink 로 스트림하는 scrcpy 서브프로세스."""
        cmd = ["scrcpy", "--video-source=camera", "--camera-facing=back",
               f"--v4l2-sink={self.video_device}", "--no-audio",
               "--no-window", "--no-playback"]
        if self.adb_serial:
            cmd += ["-s", self.adb_serial]
        if self.scrcpy_extra:
            cmd += shlex.split(self.scrcpy_extra)
        try:
            self.scrcpy_proc = subprocess.Popen(cmd)
            time.sleep(2.0)  # 장치 생성 대기
            self.get_logger().info("scrcpy 카메라 스트림 시작")
        except FileNotFoundError:
            self.get_logger().error("scrcpy 미설치 — 스크립트로 직접 실행하거나 설치 필요")

    def _open_capture(self):
        try:
            import cv2
            self._cv2 = cv2
            self.cap = cv2.VideoCapture(self.video_device)
            if not self.cap.isOpened():
                self.get_logger().warn(
                    f"{self.video_device} 열기 실패 — scrcpy/v4l2loopback 확인")
                self.cap = None
        except ImportError:
            self.get_logger().error("python3-opencv(cv2) 미설치 — 영상 발행 불가")
            self.cap = None

    def _tick_image(self):
        if self.mock:
            frame = self._mock_frame()
        elif self.cap is not None:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                return
        else:
            return
        self.pub_img.publish(self._to_image_msg(frame))

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
        cmd = ["adb"]
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
        elif cmd in ("zoom_in", "zoom_out", "focus"):
            self.get_logger().info(f"[TODO] 폰 카메라 {cmd} — 추후 구현(자리만)")
        else:
            self.get_logger().warn(f"알 수 없는 phone_cmd: {cmd}")

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
        os.makedirs(self.record_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.rec_path = os.path.join(self.record_dir, f"solcam_{ts}.mp4")
        cmd = ["ffmpeg", "-y", "-f", "v4l2", "-i", self.video_device,
               "-c:v", "libx264", "-preset", "ultrafast", "-pix_fmt", "yuv420p",
               self.rec_path]
        try:
            # stdin=PIPE: 'q'로 정상 종료(파일 손상 방지)
            self.rec_proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                             stdout=subprocess.DEVNULL,
                                             stderr=subprocess.DEVNULL)
            self.recording = True
            self._publish_recording()
            self.get_logger().info(f"녹화 시작 → {self.rec_path}")
        except FileNotFoundError:
            self.get_logger().error("ffmpeg 미설치 — 녹화 불가")
            self.rec_proc = None

    def _stop_record(self):
        self.recording = False
        self._publish_recording()
        if self.mock:
            self.get_logger().info("[MOCK] 녹화 종료")
            return
        if self.rec_proc is not None:
            try:
                self.rec_proc.communicate(input=b"q", timeout=5.0)
            except Exception:
                self.rec_proc.terminate()
            self.rec_proc = None
            self.get_logger().info(f"녹화 종료 → {self.rec_path}")
            # 폰으로 전송은 백그라운드(블로킹 방지)
            path = self.rec_path
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
        if self.rec_proc is not None:
            try:
                self.rec_proc.communicate(input=b"q", timeout=3.0)
            except Exception:
                self.rec_proc.terminate()
        if self.cap is not None:
            self.cap.release()
        if self.scrcpy_proc is not None:
            self.scrcpy_proc.terminate()
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
