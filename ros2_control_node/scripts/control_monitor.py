#!/usr/bin/env python3
"""control_node 출력 한 줄 모니터.

/control_cmd (ControlCmd)  : 몸체/리프트/상단yaw 명령 (50Hz)
/control_debug(ControlDebug): mode/engaged/owner_global (맥락)

50Hz 그대로는 너무 빠르니 기본 5Hz로 한 줄씩 찍는다.
값이 바뀔 때는 무조건 찍어서(변화 감지) 키 누름/움직임에 바로 반응 보이게.
"""
import math
import rclpy
from rclpy.node import Node
from ros2_control_node.msg import ControlCmd, ControlDebug

MODE = {0: "IDLE", 1: "FOLLOW", 2: "ROTATE", 3: "?3", 4: "?4", 5: "?5"}


class Mon(Node):
    def __init__(self):
        super().__init__("control_monitor")
        self.dbg = None
        self.n = 0
        self.last = None
        self.create_subscription(ControlCmd, "/control_cmd", self.on_cmd, 10)
        self.create_subscription(ControlDebug, "/control_debug", self.on_dbg, 10)
        print("# mode/eng/ownerOK | BODY vx vy wz | LIFT h act | TOPYAW tgt act", flush=True)

    def on_dbg(self, m):
        self.dbg = m

    def on_cmd(self, m):
        self.n += 1
        # 변화 키: 반올림해서 비교 (떨림 무시)
        key = (round(m.body_vx, 3), round(m.body_vy, 3), round(m.body_yaw_rate, 3),
               round(m.lift_height_target, 4), m.lift_active,
               round(m.top_yaw_target, 4), m.top_yaw_active)
        changed = (key != self.last)
        # 5Hz 주기 출력 또는 변화 시 즉시 출력
        if not changed and (self.n % 10 != 0):
            return
        self.last = key
        d = self.dbg
        if d is not None:
            ctx = f"m{d.mode}:{MODE.get(d.mode,'?'):6s} eng={int(d.engaged)} oOK={int(d.owner_global_valid)}"
        else:
            ctx = "m?:------ eng=? oOK=?"
        mark = "*" if changed else " "
        print(f"{mark}{ctx} | BODY {m.body_vx:+.3f} {m.body_vy:+.3f} {m.body_yaw_rate:+.3f}"
              f" | LIFT {m.lift_height_target:+.3f} {int(m.lift_active)}"
              f" | TOPYAW {m.top_yaw_target:+.4f} {int(m.top_yaw_active)}", flush=True)


def main():
    rclpy.init()
    try:
        rclpy.spin(Mon())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
