#!/usr/bin/env python3
"""LCD UI 프리뷰 — ROS 없이 Windows/PC에서 디자인 확인 (키보드로 제스처 흉내).

준비:  pip install pygame numpy
실행:  cd ros2_gesture_node
       python tools/ui_preview.py

조작 (실제 손동작 대신 키보드):
  L      = 따봉(like)  → 메뉴 열기
  1~4    = one~four    → 카테고리/항목 선택 (꾹 누르면 차오름)
  P      = palm        → 뒤로 / 닫기
  (키를 떼면 '제스처 없음'. 실제 hold/끊김/확정/플래시 동작 그대로 재현)
  R      = 녹화 토글 (REC 표시 확인),  B = 배터리 감소(데모)
  ESC    = 종료

실제 손동작 노드(gesture_node)의 상태기계(menu.py)와 렌더(hud.py)를 그대로
쓰므로, 여기서 보이는 게 LCD에 뜨는 화면과 동일하다. (배경 영상은 없음 → CAMERA)
"""
import os
import sys
import time

# ros2_gesture_node 패키지 임포트 경로 추가 (tools/ 의 부모)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pygame
from ros2_gesture_node.menu import MenuStateMachine, build_menu
from ros2_gesture_node.hud import Hud

STEPS = {"dist_step": 0.3, "heading_step_deg": 15.0, "lift_step": 0.1}
KEY_GESTURE = {pygame.K_l: "like", pygame.K_1: "one", pygame.K_2: "two",
               pygame.K_3: "three", pygame.K_4: "four", pygame.K_p: "palm"}


def current_gesture():
    keys = pygame.key.get_pressed()
    for k, g in KEY_GESTURE.items():
        if keys[k]:
            return g
    return None


def main():
    pygame.init()
    screen = pygame.display.set_mode((1024, 600))
    pygame.display.set_caption("solcam LCD preview — L=따봉 1~4=선택 P=손바닥 R=REC B=batt ESC=종료")
    sm = MenuStateMachine(build_menu(STEPS))
    hud = Hud(pygame)
    clock = pygame.time.Clock()
    mode, battery, recording, rec_start = 1, 87, False, 0.0
    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    running = False
                elif e.key == pygame.K_r:
                    recording = not recording
                    rec_start = time.time()
                elif e.key == pygame.K_b:
                    battery = max(0, battery - 5)
        # 상태기계 갱신 + 발행 사건에서 모드 반영(데모)
        for ev in sm.update(current_gesture(), time.time()):
            if ev.kind == "action" and ev.action.kind == "mode":
                mode = ev.action.payload["mode"]
        hud.draw(screen, sm.snapshot(), mode=mode, battery=battery,
                 recording=recording, rec_start=rec_start, frame=None)
        pygame.display.flip()
        clock.tick(30)
    pygame.quit()


if __name__ == "__main__":
    main()
