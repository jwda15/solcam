#!/usr/bin/env python3
"""키보드 teleop (모드0 IDLE 수동주행 + 모드 전환).

pygame 창에 포커스를 두고 조작한다. pygame.key.get_pressed()로 매 프레임
키 "전체 상태"를 폴링하므로, 방향키 동시 입력(대각선)과 a/d 회전이 자연스럽게
합쳐진다 (termios 방식은 동시키가 안 됨).

조작:
  ↑/↓        전진/후진 (body vx)
  ←/→        좌/우 평행이동 (body vy, 메카넘)
  a / d      좌회전 / 우회전 (body yaw)
  space      즉시 정지
  m          모드 선택 오버레이 열기 → 숫자키(0~5)로 모드 변경
  esc        종료

발행:
  /teleop_cmd  (geometry_msgs/Twist)  몸체 목표속도 (control_node 모드0이 사용)
  /control_mode(std_msgs/Int32)       m + 숫자키로 모드 변경

※ teleop은 모드0에서만 실제 주행에 반영된다(다른 모드는 자율 제어).
  모드 변경은 어느 모드에서나 먹는다.
"""
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

MODE_NAMES = {0: "IDLE(수동)", 1: "FOLLOW", 2: "ROTATE", 3: "(미구현)",
              4: "(미구현)", 5: "(미구현)"}


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.declare_parameter("speed", 0.3)       # m/s, 평면 목표속도
        self.declare_parameter("yaw_rate", 0.8)    # rad/s, 회전 목표속도
        self.declare_parameter("rate", 20.0)       # Hz, 발행 주기
        self.speed = float(self.get_parameter("speed").value)
        self.yaw_rate = float(self.get_parameter("yaw_rate").value)
        period = 1.0 / float(self.get_parameter("rate").value)

        self.pub_cmd = self.create_publisher(Twist, "/teleop_cmd", 10)
        self.pub_mode = self.create_publisher(Int32, "/control_mode", 10)

        import pygame
        self.pg = pygame
        pygame.init()
        self.screen = pygame.display.set_mode((480, 320))
        pygame.display.set_caption("solcam teleop — 클릭 후 조작")
        self.font = pygame.font.Font(None, 30)
        self.font_s = pygame.font.Font(None, 22)
        self.mode_select = False     # m 오버레이 상태
        self.last_mode = None
        self.create_timer(period, self._tick)

    def _tick(self):
        pg = self.pg
        for e in pg.event.get():
            if e.type == pg.QUIT:
                rclpy.shutdown(); return
            if e.type == pg.KEYDOWN:
                if e.key == pg.K_ESCAPE:
                    rclpy.shutdown(); return
                if self.mode_select:
                    self._handle_mode_key(e.key)
                elif e.key == pg.K_m:
                    self.mode_select = True

        if not self.mode_select:
            self._publish_velocity()
        self._render()

    def _publish_velocity(self):
        pg = self.pg
        keys = pg.key.get_pressed()
        vx = (keys[pg.K_UP] - keys[pg.K_DOWN]) * self.speed       # +전방
        vy = (keys[pg.K_LEFT] - keys[pg.K_RIGHT]) * self.speed    # +좌측
        wz = (keys[pg.K_a] - keys[pg.K_d]) * self.yaw_rate        # +CCW(좌회전)
        if keys[pg.K_SPACE]:
            vx = vy = wz = 0.0
        # 대각선은 크기가 √2배가 되므로 평면속도를 speed로 정규화
        import math
        mag = math.hypot(vx, vy)
        if mag > self.speed:
            vx *= self.speed / mag
            vy *= self.speed / mag
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.pub_cmd.publish(msg)

    def _handle_mode_key(self, key):
        pg = self.pg
        num = {pg.K_0: 0, pg.K_1: 1, pg.K_2: 2, pg.K_3: 3, pg.K_4: 4, pg.K_5: 5}
        if key in num:
            self.pub_mode.publish(Int32(data=num[key]))
            self.last_mode = num[key]
            self.get_logger().info(f"모드 변경 → {num[key]} {MODE_NAMES.get(num[key], '')}")
            self.mode_select = False
        elif key in (pg.K_m, pg.K_ESCAPE):
            self.mode_select = False   # 취소

    def _render(self):
        scr = self.screen
        scr.fill((18, 18, 22))
        if self.mode_select:
            self._line(scr, "모드 선택 — 숫자키", self.font, 20, (90, 200, 140))
            y = 70
            for k, name in MODE_NAMES.items():
                self._line(scr, f"  {k}.  {name}", self.font_s, y, (210, 210, 210)); y += 30
            self._line(scr, "m/esc = 취소", self.font_s, y + 6, (130, 130, 130))
        else:
            self._line(scr, "수동 주행 (모드0)", self.font, 16, (210, 210, 210))
            self._line(scr, "↑↓ 전후 / ←→ 좌우 / a,d 회전", self.font_s, 56, (170, 170, 170))
            self._line(scr, "space 정지 / m 모드선택 / esc 종료", self.font_s, 84, (170, 170, 170))
            if self.last_mode is not None:
                self._line(scr, f"최근 모드: {self.last_mode} {MODE_NAMES.get(self.last_mode,'')}",
                           self.font_s, 130, (120, 120, 120))
        self.pg.display.flip()

    def _line(self, scr, txt, font, y, color):
        scr.blit(font.render(txt, True, color), (24, y))


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
