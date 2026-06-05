"""LCD UI 노드 (최소 구현 — 디자인 다듬기는 추후).

7인치 HDMI LCD(1024x600, Jetson 연결)에 /gesture_ui(JSON)를 렌더링한다.
  IDLE: 상태 한 줄 (추후: 촬영 카메라 미러링 자리)
  MENU: 현재 경로 + 항목 목록 + 유지(hold) 진행바 + 마지막 실행 결과

pygame이 없으면 콘솔 출력으로 폴백(개발 PC에서 토픽 확인용).
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

GESTURE_KO = {"one": "1", "two": "2", "three": "3", "four": "4"}


class UiNode(Node):
    def __init__(self):
        super().__init__("gesture_ui_node")
        self.declare_parameter("fullscreen", True)
        self.declare_parameter("width", 1024)
        self.declare_parameter("height", 600)
        self.snap = {"state": "IDLE", "path": [], "items": [],
                     "hold_gesture": "", "hold_progress": 0.0,
                     "last_action": "", "ui_flags": {}}
        self.create_subscription(String, "/gesture_ui", self._cb, 10)

        try:
            import pygame
            self.pygame = pygame
            pygame.init()
            size = (int(self.get_parameter("width").value),
                    int(self.get_parameter("height").value))
            flags = pygame.FULLSCREEN if self.get_parameter("fullscreen").value else 0
            self.screen = pygame.display.set_mode(size, flags)
            # 한글 폰트: 시스템에 있는 것 탐색 (없으면 기본)
            name = pygame.font.match_font("nanumgothic,notosanscjk,malgungothic") or None
            self.font_l = pygame.font.Font(name, 64)
            self.font_m = pygame.font.Font(name, 44)
            self.font_s = pygame.font.Font(name, 30)
            self.create_timer(1.0 / 20.0, self._render)
            self.get_logger().info("pygame UI 시작")
        except Exception as e:  # pygame 미설치/디스플레이 없음 → 콘솔 폴백
            self.pygame = None
            self.create_timer(0.5, self._render_console)
            self.get_logger().warn(f"pygame 불가({e}) → 콘솔 폴백")

    def _cb(self, msg: String):
        try:
            self.snap = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    # ----- 렌더링 ---------------------------------------------------------
    def _render(self):
        pg, s = self.pygame, self.snap
        for e in pg.event.get():
            if e.type == pg.QUIT:
                rclpy.shutdown()
        scr = self.screen
        scr.fill((12, 12, 16))
        w, h = scr.get_size()

        if s["state"] == "IDLE":
            self._text(scr, "solcam", self.font_l, (w // 2, h // 2 - 40), (90, 200, 140))
            self._text(scr, "따봉을 1.5초 들면 메뉴", self.font_s, (w // 2, h // 2 + 40), (150, 150, 150))
            if s.get("last_action"):
                self._text(scr, "최근: " + s["last_action"], self.font_s, (w // 2, h - 40), (120, 120, 120))
        else:
            path = " > ".join(s.get("path", []))
            self._text(scr, path, self.font_m, (w // 2, 60), (240, 240, 240))
            items = s.get("items", [])
            y = 150
            for it in items:
                num = GESTURE_KO.get(it["gesture"], it["gesture"])
                hl = (it["gesture"] == s.get("hold_gesture"))
                color = (90, 200, 140) if hl else (200, 200, 200)
                self._text(scr, f"{num}.  {it['label']}", self.font_m, (w // 2, y), color)
                y += 70
            self._text(scr, "손바닥 = 뒤로 / 닫기", self.font_s, (w // 2, h - 90), (130, 130, 130))
            # 유지(hold) 진행바
            prog = float(s.get("hold_progress", 0.0))
            if prog > 0:
                bw = int((w - 200) * prog)
                pg.draw.rect(scr, (60, 60, 70), (100, h - 50, w - 200, 18), border_radius=9)
                pg.draw.rect(scr, (90, 200, 140), (100, h - 50, bw, 18), border_radius=9)
        pg.display.flip()

    def _text(self, scr, txt, font, center, color):
        surf = font.render(txt, True, color)
        scr.blit(surf, surf.get_rect(center=center))

    def _render_console(self):
        s = self.snap
        if s["state"] == "MENU":
            items = " | ".join(f"{i['gesture']}:{i['label']}" for i in s.get("items", []))
            print(f"[UI] {' > '.join(s['path'])}  {items}  "
                  f"hold={s['hold_gesture']}({s['hold_progress']:.0%})")


def main(args=None):
    rclpy.init(args=args)
    node = UiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
