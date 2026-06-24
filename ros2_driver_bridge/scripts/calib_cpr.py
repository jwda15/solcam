#!/usr/bin/env python3
"""calib_cpr.py — 휠 엔코더 CPR(1회전당 카운트) 실측 도우미. ROS 불필요.

펌웨어가 매 주기 보내는 엔코더 "델타"(e1,e2,e3,e4)를 누적해서, 바퀴를 손으로
정확히 N바퀴 돌렸을 때의 누적 카운트로 CPR 을 구한다.
  CPR = |누적카운트| / N   (기어비·쿼드라처 체배 다 포함된 실측값)

★전제: driver_bridge 가 시리얼 포트를 점유하면 안 됨 → 먼저 정지!
    (solcam.sh stop  또는  pkill -f driver_bridge)

사용:
    python3 calib_cpr.py [port] [baud]
    예) python3 calib_cpr.py /dev/ttyUSB0 115200

절차:
    1) 로봇을 들어 바퀴가 공중에 뜨게(또는 한 바퀴씩 자유회전 가능하게).
    2) 스크립트 실행 → 'z'+Enter 로 0으로 리셋.
    3) 한 바퀴(또는 N바퀴)를 천천히 정확히 돌린다.
    4) Enter 만 누르면 현재 누적 카운트(e1..e4) 출력 → 돌린 바퀴 값이 그 바퀴 CPR.
       (4바퀴 다 한 바퀴씩 돌리고 한 번에 봐도 됨. 부호가 −면 그 바퀴 enc_signs 반전 필요.)
    5) 4바퀴 평균을 driver_params.yaml 의 encoder_cpr 에 넣기.

종료: Ctrl+C
"""
import sys
import threading

try:
    import serial
except ImportError:
    sys.exit("pyserial 필요: pip install pyserial")

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

acc = [0, 0, 0, 0]
lock = threading.Lock()
running = True


def reader(ser):
    buf = bytearray()
    while running:
        try:
            chunk = ser.read(64)
        except Exception:
            continue
        if not chunk:
            continue
        buf.extend(chunk)
        while b"\n" in buf:
            line, _, rest = buf.partition(b"\n")
            buf = bytearray(rest)
            try:
                parts = line.decode("ascii", "ignore").strip().split(",")
                d = [int(parts[i]) for i in range(4)]   # 앞 4개 = 엔코더 델타
            except (ValueError, IndexError):
                continue
            with lock:
                for i in range(4):
                    acc[i] += d[i]


def main():
    global running
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        sys.exit(f"시리얼 열기 실패 ({PORT}): {e}\n  driver_bridge 가 떠 있으면 먼저 정지하세요.")
    print(f"열림: {PORT} @ {BAUD}")
    print("바퀴 돌리고 Enter=누적출력 / 'z'+Enter=리셋 / Ctrl+C=종료")
    t = threading.Thread(target=reader, args=(ser,), daemon=True)
    t.start()
    try:
        while True:
            cmd = input().strip().lower()
            with lock:
                if cmd == "z":
                    for i in range(4):
                        acc[i] = 0
                    print("  → 0으로 리셋")
                else:
                    print(f"  누적 카운트: e1={acc[0]:+d}  e2={acc[1]:+d}  "
                          f"e3={acc[2]:+d}  e4={acc[3]:+d}")
                    print("    (한 바퀴 돌렸으면 이 값이 그 바퀴 CPR. 부호−면 enc_signs 반전)")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        running = False
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
