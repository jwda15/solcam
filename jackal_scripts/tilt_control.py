#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
tilt_control.py  [DEPRECATED — 내일 작업에선 사용 안 함]
===============
※ 이 스크립트는 Dynamixel SDK 직접 + USB 시리얼 직결 방식.
※ 실제 자칼은 dynamixel_workbench_controllers ROS 노드가 띄워져 있음.
   같은 USB 포트를 두 곳에서 잡을 수 없으므로 이 스크립트는 충돌 위험.
※ 내일은 아래 명령으로 틸트 (검증된 방식):
     roslaunch dynamixel_workbench_controllers dynamixel_controllers.launch
     rosservice call /dynamixel_workbench/dynamixel_command \\
       "command: '' id: 1 addr_name: 'Goal_Position' value: 1536"   # 45도
※ 본 스크립트는 나중에 워크벤치 노드 없는 환경에서 단독 사용할 때를 위해
   보관만 함. 부호 정의가 워크벤치 기준과 반대일 수 있으니 사용 전 검증 필요.

D435i 카메라 틸트 (Dynamixel 1축) 제어 스크립트.

용도:
  - 데이터 수집 시작 전, 카메라를 원하는 각도로 기울여놓기
  - 저텍스처 시퀀스: 바닥을 비스듬히 보도록 아래로 틸트 (예: -25°)
  - 캡스톤 팔로우: 정면 또는 약간 위 (예: 0~+5°)

각도 정의:
  0   = 정면(수평)
  음수 = 아래로 틸트 (바닥 보기)
  양수 = 위로 틸트  (천장 보기)

사용:
  python tilt_control.py --angle -25     # -25도로 즉시 이동
  python tilt_control.py --angle 0       # 정면
  python tilt_control.py --interactive   # 키보드로 ±5도씩 조절
  python tilt_control.py --home          # 0도로
"""

import argparse
import sys
import time

# Dynamixel SDK 임포트
try:
    from dynamixel_sdk import (PortHandler, PacketHandler,
                                COMM_SUCCESS)
except ImportError:
    print('dynamixel_sdk 없음. 설치:')
    print('  pip install dynamixel-sdk')
    sys.exit(1)

# ============== 수정 포인트 (이전 설정과 동일) ==============
DEVICE_NAME      = '/dev/ttyUSB0'   # U2D2 위치. ls /dev/ttyUSB* 로 확인
BAUDRATE         = 1000000          # XL430 기본 1Mbps
PROTOCOL_VERSION = 2.0
DXL_ID           = 1                # 모터 ID

# XL430 / XM430 control table
ADDR_TORQUE_ENABLE  = 64
ADDR_GOAL_POSITION  = 116
ADDR_PRESENT_POS    = 132
ADDR_OPERATING_MODE = 11
ADDR_PROFILE_VEL    = 112
TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0
POSITION_CONTROL_MODE = 3

# 0~4095 = 0~360°
TICKS_PER_REV  = 4096
CENTER_TICK    = 2048      # 모터의 0도 = 2048 (조립 기준)
PROFILE_VEL    = 50        # 부드러운 이동
# 안전 한계
MIN_DEG = -45.0
MAX_DEG = +45.0
# =============================================================


def deg_to_tick(deg):
    deg = max(MIN_DEG, min(MAX_DEG, deg))
    return CENTER_TICK + int(deg / 360.0 * TICKS_PER_REV)


def tick_to_deg(tick):
    return (tick - CENTER_TICK) * 360.0 / TICKS_PER_REV


class Tilt:
    def __init__(self, port=DEVICE_NAME, baud=BAUDRATE, dxl_id=DXL_ID):
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.id = dxl_id

        if not self.port_handler.openPort():
            raise RuntimeError(f'포트 열기 실패: {port}')
        if not self.port_handler.setBaudRate(baud):
            raise RuntimeError(f'baudrate 설정 실패: {baud}')
        print(f'포트 연결: {port} @ {baud}')

        self._write1(ADDR_TORQUE_ENABLE,  TORQUE_DISABLE)
        self._write1(ADDR_OPERATING_MODE, POSITION_CONTROL_MODE)
        self._write4(ADDR_PROFILE_VEL,    PROFILE_VEL)
        self._write1(ADDR_TORQUE_ENABLE,  TORQUE_ENABLE)
        print('초기화 완료. 토크 ON.')

    def _check(self, comm, err, label):
        if comm != COMM_SUCCESS:
            print(f'[{label}] COMM_ERR: {self.packet_handler.getTxRxResult(comm)}')
        elif err != 0:
            print(f'[{label}] DXL_ERR: {self.packet_handler.getRxPacketError(err)}')

    def _write1(self, addr, val):
        c, e = self.packet_handler.write1ByteTxRx(self.port_handler, self.id, addr, val)
        self._check(c, e, f'write1 addr={addr}')

    def _write4(self, addr, val):
        c, e = self.packet_handler.write4ByteTxRx(self.port_handler, self.id, addr, val)
        self._check(c, e, f'write4 addr={addr}')

    def _read4(self, addr):
        v, c, e = self.packet_handler.read4ByteTxRx(self.port_handler, self.id, addr)
        self._check(c, e, f'read4 addr={addr}')
        return v

    def set_angle(self, deg, wait=True):
        tick = deg_to_tick(deg)
        print(f'  → goal: {deg:+.1f}° (tick {tick})')
        self._write4(ADDR_GOAL_POSITION, tick)
        if wait:
            for _ in range(100):
                cur = self._read4(ADDR_PRESENT_POS)
                if abs(cur - tick) < 10:
                    break
                time.sleep(0.05)
            print(f'  ← 현재: {tick_to_deg(self._read4(ADDR_PRESENT_POS)):+.1f}°')

    def get_angle(self):
        return tick_to_deg(self._read4(ADDR_PRESENT_POS))

    def shutdown(self, disable_torque=False):
        if disable_torque:
            self._write1(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            print('토크 OFF')
        self.port_handler.closePort()
        print('포트 닫음')


def interactive(t: Tilt):
    print()
    print('  키 입력 (Enter):')
    print('   w/s : +5° / -5°')
    print('   a/d : +1° / -1°')
    print('   0   : 0°(정면)')
    print('   숫자값 : 그 각도로 (예: -25)')
    print('   q   : 종료')
    print()
    while True:
        try:
            cur = t.get_angle()
            cmd = input(f'[현재 {cur:+.1f}°] > ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if cmd == 'q': break
        elif cmd == 'w': t.set_angle(cur + 5.0)
        elif cmd == 's': t.set_angle(cur - 5.0)
        elif cmd == 'a': t.set_angle(cur + 1.0)
        elif cmd == 'd': t.set_angle(cur - 1.0)
        elif cmd == '0': t.set_angle(0.0)
        else:
            try:
                t.set_angle(float(cmd))
            except ValueError:
                print('  ?')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--angle', type=float, help='이동할 각도 (deg)')
    p.add_argument('--home', action='store_true', help='0도로')
    p.add_argument('--interactive', action='store_true')
    p.add_argument('--port', default=DEVICE_NAME)
    p.add_argument('--id', type=int, default=DXL_ID)
    p.add_argument('--release', action='store_true',
                   help='종료 시 토크 OFF (수동 회전 가능)')
    args = p.parse_args()

    t = Tilt(port=args.port, dxl_id=args.id)
    try:
        if args.home:
            t.set_angle(0.0)
        elif args.angle is not None:
            t.set_angle(args.angle)
        elif args.interactive:
            interactive(t)
        else:
            print('--angle / --home / --interactive 중 하나')
    finally:
        t.shutdown(disable_torque=args.release)


if __name__ == '__main__':
    main()
