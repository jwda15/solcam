#!/usr/bin/env python3
# ============================================================================
#  test_protocol.py — UART 프로토콜·정기구학 단위검증 (ROS/시리얼 불필요)
#
#  실행:  python3 test/test_protocol.py
#  목적:  브리지의 패킹/언패킹/체크섬이 STM32 펌웨어의 바이트 처리와
#         정확히 일치하는지, 메카넘 정/역기구학이 왕복 일치하는지 확인.
# ============================================================================
import math
import struct
import sys

HEADER_TX = 0xAA
HEADER_RX = 0xBB
TX_SIZE = 28
RX_SIZE = 19


def checksum(data):
    return sum(data) & 0xFF


# ---- 브리지 TX 패킹 (driver_bridge._pack_cmd 와 동일 로직) ----
def bridge_pack_cmd(vx, vy, wz, lift_t, lift_a, yaw_t, yaw_a):
    buf = bytearray(TX_SIZE)
    buf[0] = HEADER_TX
    struct.pack_into('<f', buf, 1, vx)
    struct.pack_into('<f', buf, 5, vy)
    struct.pack_into('<f', buf, 9, wz)
    struct.pack_into('<f', buf, 13, lift_t)
    buf[17] = lift_a & 0xFF
    struct.pack_into('<f', buf, 18, yaw_t)
    buf[22] = yaw_a & 0xFF
    buf[27] = checksum(buf[0:27])
    return bytes(buf)


# ---- 펌웨어 RxCpltCallback 의 파싱을 그대로 모사 ----
def firmware_parse_cmd(buf):
    assert len(buf) == TX_SIZE
    assert buf[0] == HEADER_TX, "헤더 불일치"
    # 펌웨어: calc_checksum(buf, UART_RX_SIZE-1=27) == buf[27]
    assert checksum(buf[0:27]) == buf[27], "체크섬 불일치"
    vx = struct.unpack_from('<f', buf, 1)[0]
    vy = struct.unpack_from('<f', buf, 5)[0]
    wz = struct.unpack_from('<f', buf, 9)[0]
    lift_t = struct.unpack_from('<f', buf, 13)[0]
    lift_a = buf[17]
    yaw_t = struct.unpack_from('<f', buf, 18)[0]
    yaw_a = buf[22]
    return (vx, vy, wz, lift_t, lift_a, yaw_t, yaw_a)


# ---- 펌웨어 UART_SendStatus(패치본) 의 바이트 구성을 모사 ----
def firmware_pack_status(e1, e2, e3, e4, lift_h, yaw_a):
    buf = bytearray(RX_SIZE)
    buf[0] = HEADER_RX
    struct.pack_into('<h', buf, 1, e1)
    struct.pack_into('<h', buf, 3, e2)
    struct.pack_into('<h', buf, 5, e3)
    struct.pack_into('<h', buf, 7, e4)
    struct.pack_into('<f', buf, 9, lift_h)
    struct.pack_into('<f', buf, 13, yaw_a)
    buf[17] = 0                       # reserved
    buf[18] = checksum(buf[0:18])     # sum(bytes[0..17])
    return bytes(buf)


# ---- 브리지 RX 언패킹 (driver_bridge._handle_status 와 동일) ----
def bridge_parse_status(frame):
    assert len(frame) == RX_SIZE
    assert frame[0] == HEADER_RX
    assert checksum(frame[0:RX_SIZE - 1]) == frame[RX_SIZE - 1], "RX 체크섬 불일치"
    (_h, e1, e2, e3, e4, lift_h, yaw_a, _r, _c) = struct.unpack('<Bhhhhff BB', frame)
    return (e1, e2, e3, e4, lift_h, yaw_a)


# ---- 메카넘 기구학 ----
R, LX, LY = 0.05, 0.36, 0.26
L = LX + LY


def inverse(vx, vy, wz):
    """펌웨어 Mecanum_SetVelocity 의 역기구학."""
    return (
        (vx - vy - L * wz) / R,   # FL
        (vx + vy + L * wz) / R,   # FR
        (vx + vy - L * wz) / R,   # RL
        (vx - vy + L * wz) / R,   # RR
    )


def forward(w_fl, w_fr, w_rl, w_rr):
    """브리지의 정기구학."""
    vx = R / 4.0 * (w_fl + w_fr + w_rl + w_rr)
    vy = R / 4.0 * (-w_fl + w_fr + w_rl - w_rr)
    wz = R / (4.0 * L) * (-w_fl + w_fr - w_rl + w_rr)
    return vx, vy, wz


def approx(a, b, tol=1e-4):
    return abs(a - b) <= tol


def main():
    n_pass = n_fail = 0

    def check(name, cond):
        nonlocal n_pass, n_fail
        if cond:
            n_pass += 1
            print(f"  PASS  {name}")
        else:
            n_fail += 1
            print(f"  FAIL  {name}")

    print("[1] 명령 프레임 왕복 (브리지 pack → 펌웨어 parse)")
    cases = [
        (0.0, 0.0, 0.0, 0.0, 0, 0.0, 0),
        (0.35, -0.1, 0.3, 0.25, 1, 1.57, 1),
        (-0.4, 0.4, -0.7, 0.5, 1, -3.14, 0),
    ]
    for c in cases:
        frame = bridge_pack_cmd(*c)
        check(f"len==28 ({c[0]})", len(frame) == TX_SIZE)
        vx, vy, wz, lt, la, yt, ya = firmware_parse_cmd(frame)
        check("vx", approx(vx, c[0]))
        check("vy", approx(vy, c[1]))
        check("wz", approx(wz, c[2]))
        check("lift_target", approx(lt, c[3]))
        check("lift_active", la == c[4])
        check("yaw_target", approx(yt, c[5]))
        check("yaw_active", ya == c[6])

    print("[2] 상태 프레임 왕복 (펌웨어 pack → 브리지 parse)")
    s_cases = [
        (0, 0, 0, 0, 0.0, 0.0),
        (12, -5, 30, -30, 0.32, 1.05),
        (-300, 300, 100, -100, 0.5, -2.0),
    ]
    for s in s_cases:
        frame = firmware_pack_status(*s)
        check(f"len==19 ({s[0]})", len(frame) == RX_SIZE)
        e1, e2, e3, e4, lh, ya = bridge_parse_status(frame)
        check("enc", (e1, e2, e3, e4) == s[0:4])
        check("lift_h", approx(lh, s[4]))
        check("yaw_a", approx(ya, s[5]))

    print("[3] 메카넘 정/역기구학 왕복 (vx,vy,wz → 휠 → vx,vy,wz)")
    for (vx, vy, wz) in [(0.3, 0, 0), (0, 0.2, 0), (0, 0, 0.5),
                         (0.25, -0.15, 0.3), (-0.2, 0.1, -0.4)]:
        wheels = inverse(vx, vy, wz)
        rx, ry, rwz = forward(*wheels)
        check(f"vx ({vx},{vy},{wz})", approx(rx, vx))
        check("vy", approx(ry, vy))
        check("wz", approx(rwz, wz))

    print("[4] 체크섬 손상 검출")
    f = bytearray(bridge_pack_cmd(0.1, 0.2, 0.3, 0.1, 1, 0.5, 1))
    f[5] ^= 0xFF                       # vy 바이트 1개 손상
    bad = checksum(f[0:27]) != f[27]
    check("손상 프레임은 체크섬 불일치", bad)

    print(f"\n결과: {n_pass} PASS / {n_fail} FAIL")
    return 0 if n_fail == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
