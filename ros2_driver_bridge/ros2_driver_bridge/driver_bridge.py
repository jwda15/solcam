#!/usr/bin/env python3
# ============================================================================
#  driver_bridge.py — control_node(ROS2) ↔ STM32F407 드라이버 보드 UART 브리지
#
#  팀원 펌웨어(D:/capstone/yaw, STM32F407)는 ROS를 모르고 USART1 바이너리
#  프레임만 안다. 이 노드가 Jetson에서 그 사이를 잇는다.
#
#  구독: /control_cmd  (ros2_control_node/ControlCmd)  6자유도 명령
#  발행: /odom         (nav_msgs/Odometry)             휠 엔코더 정기구학+적분
#        /top_yaw_state(std_msgs/Float32)              상단 yaw 현재각[rad]
#
#  ── UART 프로토콜 (펌웨어 main.c 기준) ─────────────────────────────────────
#  Jetson → STM32  (28바이트):
#    [0]      0xAA 헤더
#    [1..4]   vx           float32 LE  (m/s,  +전방)
#    [5..8]   vy           float32 LE  (m/s,  +좌측)
#    [9..12]  wz           float32 LE  (rad/s,+CCW)
#    [13..16] lift_target  float32 LE  (m, 절대 목표 높이)
#    [17]     lift_active  uint8       (0=현위치 유지)
#    [18..21] yaw_target   float32 LE  (rad, 상단yaw 목표각)
#    [22]     yaw_active   uint8       (0=현위치 유지)
#    [23..26] reserved     (0, 펌웨어가 무시)
#    [27]     checksum     uint8       = sum(bytes[0..26]) & 0xFF
#
#  STM32 → Jetson (19바이트):
#    [0]      0xBB 헤더
#    [1..2]   enc1  int16 LE   (지난 주기 카운트 델타)
#    [3..4]   enc2  int16 LE
#    [5..6]   enc3  int16 LE
#    [7..8]   enc4  int16 LE
#    [9..12]  lift_height float32 LE (m)   ※현재 미사용(참고/디버그)
#    [13..16] yaw_angle   float32 LE (rad) → /top_yaw_state
#    [17]     reserved (0)
#    [18]     checksum uint8   = sum(bytes[0..17]) & 0xFF
#
#  ※ 펌웨어 원본 UART_SendStatus는 체크섬 위치/크기 버그가 있었다(아래 README
#    참고). 본 브리지는 "안전수정 패치된 펌웨어"의 위 레이아웃을 기준으로 한다.
#    펌웨어를 재플래시할 때 watchdog 패치와 함께 적용해야 RX 체크섬이 맞다.
# ============================================================================

import math
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from ros2_control_node.msg import ControlCmd

try:
    import serial  # pyserial
except ImportError:  # mock 모드에선 serial 없이도 import 통과
    serial = None

# ---- 프레임 상수 (펌웨어 #define 과 일치) ----
HEADER_TX = 0xAA          # Jetson → STM32
HEADER_RX = 0xBB          # STM32 → Jetson
TX_SIZE = 28
RX_SIZE = 19


def _checksum(data: bytes) -> int:
    """펌웨어 calc_checksum 과 동일: 바이트 단순합의 하위 8비트."""
    return sum(data) & 0xFF


def _clip(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


class DriverBridge(Node):
    def __init__(self):
        super().__init__('driver_bridge')

        # ---------------- 파라미터 ----------------
        # 시리얼
        self.declare_parameter('port', '/dev/ttyUSB0')   # Jetson 핀 UART면 /dev/ttyTHS1
        self.declare_parameter('baud', 115200)           # 펌웨어 USART1=115200
        self.declare_parameter('mock', False)            # true면 시리얼 없이 로직만(테스트)

        # 차체 기구 상수 (펌웨어와 동일하게 맞출 것)
        self.declare_parameter('wheel_radius', 0.05)     # m  (WHEEL_R)
        self.declare_parameter('wheel_lx', 0.36)         # m  (WHEEL_LX, 좌우 절반)
        self.declare_parameter('wheel_ly', 0.26)         # m  (WHEEL_LY, 전후 절반)

        # ★엔코더 분해능 — 반드시 실측 캘리브. 휠 1회전당 카운트(쿼드 x4 + 기어 포함).
        #   JGB37-520. 펌웨어는 raw 카운트만 보내므로 여기서 환산한다.
        self.declare_parameter('encoder_cpr', 1320.0)    # PLACEHOLDER. 실측 후 교체!

        # enc1..4(수신순서) → 물리 휠 [FL, FR, RL, RR] 매핑과 부호.
        #   기본: enc1=FL, enc2=FR, enc3=RL, enc4=RR. 실기에서 한 바퀴씩 돌려보고 확정.
        self.declare_parameter('encoder_order', [0, 1, 2, 3])  # tx index 순서
        self.declare_parameter('encoder_signs', [1, 1, 1, 1])  # +1 / -1

        # 발행/통신 주기
        self.declare_parameter('tx_rate', 50.0)          # Hz, STM32로 명령 송신(하트비트)
        self.declare_parameter('cmd_timeout', 0.3)       # s, /control_cmd 끊기면 0속도 송신
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('verify_rx_checksum', True)  # 깨진 프레임 버림
        # STM 상태(엔코더) 포맷:
        #   'ascii'  — 현 보드 펌웨어: "e1,e2,e3,e4\n" 텍스트 (yaw각 미전송)
        #   'binary' — repo driver_firmware: 0xBB 19바이트 (yaw각 포함)
        #   보드에 구워진 펌웨어에 맞출 것. 둘 다 엔코더로 /odom 을 계산해 발행한다.
        self.declare_parameter('status_format', 'ascii')

        # ---- 몸체속도 폐루프 보정 (엔코더 피드백) ----
        #  펌웨어 휠 제어가 개루프 PWM이라, 측면/자전/대각선에서 바퀴 부하 비대칭으로
        #  실제 속도가 지령과 어긋난다(언더드라이브/드리프트). firmware UART는 per-wheel을
        #  못 받으므로, 엔코더 정기구학으로 얻은 "실측 몸체속도"를 지령과 비교해 우리가
        #  보낼 (vx,vy,wz)를 PI(+피드포워드)로 보정한다. 3DOF(몸체) 레벨 폐루프.
        #  ★전제: encoder_cpr/encoder_signs 가 맞아야 함(아니면 오히려 악화) → 기본 OFF.
        #    /odom twist 가 실제와 맞는지 확인 후 wheel_fb_enable:=true 로 켤 것.
        self.declare_parameter('wheel_fb_enable', False)   # ★캘리브 후 true
        self.declare_parameter('wheel_fb_kp', 0.4)         # 비례(즉각 보정, 작게)
        self.declare_parameter('wheel_fb_ki', 1.2)         # 적분[1/s](정상상태 스케일오차 제거 주역)
        self.declare_parameter('wheel_fb_corr_v', 0.15)    # m/s, 선속도 축당 최대 보정량
        self.declare_parameter('wheel_fb_corr_w', 0.6)     # rad/s, 각속도 최대 보정량
        self.declare_parameter('wheel_fb_out_v', 0.40)     # m/s, 보정 후 선속도 벡터크기 상한(=펌웨어 V_MAX)
        self.declare_parameter('wheel_fb_out_w', 2.0)      # rad/s, 보정 후 각속도 상한
        self.declare_parameter('wheel_fb_min_cmd', 0.02)   # 이하 지령은 0취급(적분 리셋, 보정 안 함)
        self.declare_parameter('wheel_fb_meas_timeout', 0.2)  # s, 실측 끊기면 보정 중단

        gp = self.get_parameter
        self.port = gp('port').value
        self.baud = int(gp('baud').value)
        self.mock = bool(gp('mock').value)
        self.r = float(gp('wheel_radius').value)
        self.lx = float(gp('wheel_lx').value)
        self.ly = float(gp('wheel_ly').value)
        self.cpr = float(gp('encoder_cpr').value)
        self.enc_order = list(gp('encoder_order').value)
        self.enc_signs = list(gp('encoder_signs').value)
        self.tx_rate = float(gp('tx_rate').value)
        self.cmd_timeout = float(gp('cmd_timeout').value)
        self.publish_odom = bool(gp('publish_odom').value)
        self.odom_frame = gp('odom_frame').value
        self.base_frame = gp('base_frame').value
        self.verify_rx_checksum = bool(gp('verify_rx_checksum').value)
        self.status_format = str(gp('status_format').value).lower()

        self.fb_enable    = bool(gp('wheel_fb_enable').value)
        self.fb_kp        = float(gp('wheel_fb_kp').value)
        self.fb_ki        = float(gp('wheel_fb_ki').value)
        self.fb_corr_v    = float(gp('wheel_fb_corr_v').value)
        self.fb_corr_w    = float(gp('wheel_fb_corr_w').value)
        self.fb_out_v     = float(gp('wheel_fb_out_v').value)
        self.fb_out_w     = float(gp('wheel_fb_out_w').value)
        self.fb_min_cmd   = float(gp('wheel_fb_min_cmd').value)
        self.fb_meas_tmo  = float(gp('wheel_fb_meas_timeout').value)

        # ---------------- 상태 ----------------
        self._cmd_lock = threading.Lock()
        self._latest_cmd = None          # 최근 ControlCmd (없으면 None)
        self._last_cmd_time = self.get_clock().now()
        self._warned_timeout = False

        # odom 적분 상태
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self._last_rx_time = None        # 직전 상태프레임 수신 시각(monotonic)

        # 몸체속도 폐루프 상태 (RX스레드가 측정 기록 → TX타이머가 읽어 보정)
        self._meas_lock = threading.Lock()
        self._meas = None                # 최근 실측 (vx,vy,wz)
        self._meas_time = self.get_clock().now()
        self._fb_ix = self._fb_iy = self._fb_iw = 0.0   # 축별 적분기
        self._last_tx_time = self.get_clock().now()

        # ---------------- 시리얼 ----------------
        self.ser = None
        if not self.mock:
            if serial is None:
                self.get_logger().error(
                    "pyserial 미설치. 'pip install pyserial' 또는 mock:=true")
                raise RuntimeError("pyserial required")
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                self.get_logger().info(f"시리얼 열림: {self.port} @ {self.baud}")
            except Exception as e:
                self.get_logger().error(f"시리얼 열기 실패 ({self.port}): {e}")
                raise
        else:
            self.get_logger().warning("MOCK 모드: 시리얼 없이 동작 (TX/RX 미수행)")

        # ---------------- ROS 입출력 ----------------
        self.cmd_sub = self.create_subscription(
            ControlCmd, '/control_cmd', self._cmd_cb, 10)
        self.yaw_pub = self.create_publisher(
            Float32, '/top_yaw_state', qos_profile_sensor_data)
        if self.publish_odom:
            self.odom_pub = self.create_publisher(
                Odometry, '/odom', qos_profile_sensor_data)

        # TX 하트비트 타이머 (control_node 50Hz와 무관하게 일정하게 송신)
        self.tx_timer = self.create_timer(1.0 / self.tx_rate, self._tx_step)

        # RX 수신 스레드
        self._rx_running = True
        if not self.mock:
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()

        self.get_logger().info(
            f"driver_bridge 시작. tx={self.tx_rate:.0f}Hz cpr={self.cpr:.0f} "
            f"r={self.r} L={self.lx + self.ly:.2f} mock={self.mock}")

    # ====================================================================
    #  /control_cmd 콜백 — 최신 명령만 캐시
    # ====================================================================
    def _cmd_cb(self, msg: ControlCmd):
        with self._cmd_lock:
            self._latest_cmd = msg
            self._last_cmd_time = self.get_clock().now()

    # ====================================================================
    #  TX: 최신 명령(또는 타임아웃 시 0속도)을 28바이트로 패킹해 송신
    # ====================================================================
    def _tx_step(self):
        with self._cmd_lock:
            cmd = self._latest_cmd
            last = self._last_cmd_time

        timed_out = (self.get_clock().now() - last).nanoseconds * 1e-9 > self.cmd_timeout

        if cmd is None or timed_out:
            # control_node가 끊기면 몸체 정지(리프트/yaw는 현위치 유지).
            vx = vy = wz = 0.0
            lift_target, lift_active = 0.0, 0
            yaw_target, yaw_active = 0.0, 0
            self._fb_reset()   # 정지 → 적분기 리셋(재개 시 튐/크리프 방지)
            if cmd is not None and timed_out and not self._warned_timeout:
                self.get_logger().warning("/control_cmd 타임아웃 → 0속도 송신")
                self._warned_timeout = True
        else:
            self._warned_timeout = False
            vx, vy, wz = cmd.body_vx, cmd.body_vy, cmd.body_yaw_rate
            lift_target = cmd.lift_height_target
            lift_active = 1 if cmd.lift_active else 0
            yaw_target = cmd.top_yaw_target
            yaw_active = 1 if cmd.top_yaw_active else 0
            # ★몸체속도 폐루프 보정 (enable 시): 실측과 지령 오차를 PI(+FF)로 메움.
            vx, vy, wz = self._apply_wheel_fb(vx, vy, wz)

        frame = self._pack_cmd(vx, vy, wz, lift_target, lift_active,
                               yaw_target, yaw_active)
        if self.ser is not None:
            try:
                self.ser.write(frame)
            except Exception as e:
                self.get_logger().error(f"시리얼 write 실패: {e}")

    @staticmethod
    def _pack_cmd(vx, vy, wz, lift_target, lift_active,
                  yaw_target, yaw_active) -> bytes:
        buf = bytearray(TX_SIZE)             # 전부 0 (reserved 23..26 = 0)
        buf[0] = HEADER_TX
        struct.pack_into('<f', buf, 1, float(vx))
        struct.pack_into('<f', buf, 5, float(vy))
        struct.pack_into('<f', buf, 9, float(wz))
        struct.pack_into('<f', buf, 13, float(lift_target))
        buf[17] = lift_active & 0xFF
        struct.pack_into('<f', buf, 18, float(yaw_target))
        buf[22] = yaw_active & 0xFF
        buf[27] = _checksum(buf[0:27])       # sum(bytes[0..26])
        return bytes(buf)

    # ====================================================================
    #  몸체속도 폐루프 보정 — 실측(엔코더 정기구학) vs 지령 오차를 PI(+FF)로 보정.
    #   firmware 가 per-wheel 명령을 못 받으므로 보낼 (vx,vy,wz) 자체를 조정한다.
    #   ★전제: encoder_cpr/encoder_signs 캘리브로 /odom twist 가 실제와 맞아야 함.
    #   루프 지연(UART+20ms 상태+측정노이즈)이 있어 게인은 보수적(주로 적분).
    # ====================================================================
    def _fb_reset(self):
        self._fb_ix = self._fb_iy = self._fb_iw = 0.0

    def _apply_wheel_fb(self, vx, vy, wz):
        if not self.fb_enable:
            return vx, vy, wz

        now = self.get_clock().now()
        dt = (now - self._last_tx_time).nanoseconds * 1e-9
        self._last_tx_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.tx_rate          # 비정상 dt 보호

        # 지령이 사실상 0 → 적분 리셋 + 보정 안 함 (정지 중 크리프/와인드업 방지)
        if (abs(vx) < self.fb_min_cmd and abs(vy) < self.fb_min_cmd and
                abs(wz) < self.fb_min_cmd):
            self._fb_reset()
            return vx, vy, wz

        # 최신 실측 속도 (신선도 확인 — 끊기면 보정 보류, 피드포워드만)
        with self._meas_lock:
            meas = self._meas
            mt = self._meas_time
        if meas is None or (now - mt).nanoseconds * 1e-9 > self.fb_meas_tmo:
            return vx, vy, wz
        mvx, mvy, mwz = meas

        ex, ey, ew = vx - mvx, vy - mvy, wz - mwz
        # anti-windup: ki·적분 이 보정상한을 넘지 않게 적분기를 클램프
        iv_lim = self.fb_corr_v / max(self.fb_ki, 1e-6)
        iw_lim = self.fb_corr_w / max(self.fb_ki, 1e-6)
        self._fb_ix = _clip(self._fb_ix + ex * dt, -iv_lim, iv_lim)
        self._fb_iy = _clip(self._fb_iy + ey * dt, -iv_lim, iv_lim)
        self._fb_iw = _clip(self._fb_iw + ew * dt, -iw_lim, iw_lim)
        cx = _clip(self.fb_kp * ex + self.fb_ki * self._fb_ix, -self.fb_corr_v, self.fb_corr_v)
        cy = _clip(self.fb_kp * ey + self.fb_ki * self._fb_iy, -self.fb_corr_v, self.fb_corr_v)
        cw = _clip(self.fb_kp * ew + self.fb_ki * self._fb_iw, -self.fb_corr_w, self.fb_corr_w)

        ox, oy, ow = vx + cx, vy + cy, wz + cw
        # 출력 클램프: 선속도 벡터크기 ≤ out_v, |각속도| ≤ out_w (firmware 도 재클램프함)
        mag = math.hypot(ox, oy)
        if mag > self.fb_out_v and mag > 1e-9:
            ox *= self.fb_out_v / mag
            oy *= self.fb_out_v / mag
        ow = _clip(ow, -self.fb_out_w, self.fb_out_w)
        return ox, oy, ow

    # ====================================================================
    #  RX: 0xBB 헤더 동기화 → 19바이트 파싱 → odom / top_yaw_state 발행
    # ====================================================================
    def _rx_loop(self):
        # 보드 상태 포맷에 따라 분기 (둘 다 엔코더 → /odom 공통 계산).
        if self.status_format == 'ascii':
            self._rx_loop_ascii()
        else:
            self._rx_loop_binary()

    # ---- 바이너리(0xBB 19B) 수신 — repo driver_firmware ----
    def _rx_loop_binary(self):
        buf = bytearray()
        while self._rx_running and rclpy.ok():
            try:
                chunk = self.ser.read(64)
            except Exception as e:
                self.get_logger().error(f"시리얼 read 실패: {e}")
                continue
            if not chunk:
                continue
            buf.extend(chunk)
            # 헤더 기준으로 프레임 잘라내기 (1바이트 단위 재동기화)
            while len(buf) >= RX_SIZE:
                if buf[0] != HEADER_RX:
                    del buf[0]               # 헤더 찾을 때까지 한 바이트씩 버림
                    continue
                frame = bytes(buf[0:RX_SIZE])
                if self.verify_rx_checksum and _checksum(frame[0:RX_SIZE - 1]) != frame[RX_SIZE - 1]:
                    del buf[0]               # 체크섬 불일치 → 한 칸 밀어 재동기화
                    continue
                del buf[0:RX_SIZE]
                self._handle_status_binary(frame)

    # ---- ASCII("e1,e2,e3,e4\n") 수신 — 현 보드 펌웨어 ----
    def _rx_loop_ascii(self):
        buf = bytearray()
        while self._rx_running and rclpy.ok():
            try:
                chunk = self.ser.read(64)
            except Exception as e:
                self.get_logger().error(f"시리얼 read 실패: {e}")
                continue
            if not chunk:
                continue
            buf.extend(chunk)
            while b'\n' in buf:
                line, _, rest = buf.partition(b'\n')
                buf = bytearray(rest)
                self._handle_status_ascii(line)

    def _handle_status_binary(self, frame: bytes):
        # '<B hhhh ff B B' = 1+8+4+4+1+1 = 19
        (_hdr, e1, e2, e3, e4, lift_h, yaw_a, _rsv, _chk) = \
            struct.unpack('<Bhhhhff BB', frame)
        # 상단 yaw 현재각 발행 (펌웨어가 직접 rad로 줌)
        self.yaw_pub.publish(Float32(data=float(yaw_a)))
        if self.publish_odom:
            self._odom_from_encoders(e1, e2, e3, e4)

    def _handle_status_ascii(self, line: bytes):
        # "e1,e2,e3,e4" — 앞 4개 정수만 사용(뒤 잉여 필드/잡음 무시).
        #  ※ ASCII 펌웨어는 상단yaw각을 안 보내므로 /top_yaw_state 는 미발행.
        #    (OAK가 yaw 스테이지에 없을 땐 theta_head=0 이 오히려 맞음)
        try:
            parts = line.decode('ascii', errors='ignore').strip().split(',')
            if len(parts) < 4:
                return
            e1, e2, e3, e4 = (int(parts[i]) for i in range(4))
        except (ValueError, IndexError):
            return
        if self.publish_odom:
            self._odom_from_encoders(e1, e2, e3, e4)

    # ---- 엔코더 델타 4개 → 메카넘 정기구학 → /odom (두 포맷 공용) ----
    #  ※ 엔코더는 "지난 주기 카운트 델타"로 가정(펌웨어가 매 루프 카운터 리셋).
    #    값이 누적이면 odom이 폭주하니, 그 경우 펌웨어/파서를 델타로 맞출 것.
    def _odom_from_encoders(self, e1, e2, e3, e4):
        now = self.get_clock().now()
        if self._last_rx_time is None:
            self._last_rx_time = now
            return
        dt = (now - self._last_rx_time).nanoseconds * 1e-9
        self._last_rx_time = now
        if dt <= 0.0 or dt > 0.5:            # 비정상 dt 보호
            return

        # 수신순서 enc[0..3] → 물리 휠 매핑/부호 적용
        raw = [e1, e2, e3, e4]
        w = [0.0, 0.0, 0.0, 0.0]             # 휠 각속도 [FL, FR, RL, RR] (rad/s)
        for wheel_idx in range(4):
            src = self.enc_order[wheel_idx]
            counts = self.enc_signs[wheel_idx] * raw[src]
            dtheta = 2.0 * math.pi * counts / self.cpr   # 휠 회전각[rad]
            w[wheel_idx] = dtheta / dt
        w_fl, w_fr, w_rl, w_rr = w

        # 메카넘 정기구학 (펌웨어 역기구학의 역; L = lx + ly)
        L = self.lx + self.ly
        vx = self.r / 4.0 * (w_fl + w_fr + w_rl + w_rr)
        vy = self.r / 4.0 * (-w_fl + w_fr + w_rl - w_rr)
        wz = self.r / (4.0 * L) * (-w_fl + w_fr - w_rl + w_rr)

        # 폐루프 보정용 최신 실측 속도 공유 (TX 타이머가 읽음)
        with self._meas_lock:
            self._meas = (vx, vy, wz)
            self._meas_time = now

        # 오도메트리 적분 (odom 프레임)
        c, s = math.cos(self.odom_yaw), math.sin(self.odom_yaw)
        self.odom_x += (vx * c - vy * s) * dt
        self.odom_y += (vx * s + vy * c) * dt
        self.odom_yaw = self._wrap(self.odom_yaw + wz * dt)

        self._publish_odom(now, vx, vy, wz)

    def _publish_odom(self, stamp, vx, vy, wz):
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.odom_x
        msg.pose.pose.position.y = self.odom_y
        msg.pose.pose.orientation = self._yaw_to_quat(self.odom_yaw)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        # 휠 데드레커닝이라 yaw/위치 공분산은 보수적으로 큼(참고용)
        msg.pose.covariance[0] = 0.05    # x
        msg.pose.covariance[7] = 0.05    # y
        msg.pose.covariance[35] = 0.1    # yaw
        self.odom_pub.publish(msg)

    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    @staticmethod
    def _wrap(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def destroy_node(self):
        self._rx_running = False
        # 종료 시 안전 정지 1회 송신
        if self.ser is not None:
            try:
                self.ser.write(self._pack_cmd(0, 0, 0, 0, 0, 0, 0))
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriverBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
