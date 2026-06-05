# ControlCmd.msg
# 제어 노드 → 드라이버 노드(팀원)로 보내는 6자유도 명령.
# 제어 노드는 "로봇이 어떻게 움직일지"만 내고, 모터 추력/스텝 매핑은
# 드라이버 노드의 책임이다. (제어 노드는 모터 개수를 모른다.)
#
# ★모터 종류에 맞춘 명령 의미:
#   - 메카넘 휠(4, DC+엔코더): 속도 명령
#   - 리프트(1, NEMA23 스텝):  위치 명령(목표 높이)
#   - 상단 yaw(1, NEMA17 스텝): 위치 명령(목표 각)
#
# 좌표계: 몸체 x 전방(+), y 좌측(+), yaw CCW(+).
std_msgs/Header header

# --- 메카넘 (속도) ---
float32 body_vx          # m/s,  몸체 전후 (+전방)
float32 body_vy          # m/s,  몸체 좌우 (+좌측)
float32 body_yaw_rate    # rad/s, 몸체 회전 (+CCW=좌회전)

# --- 리프트 (스텝, 위치) ---
float32 lift_height_target  # m, 목표 높이 (절대)
bool    lift_active         # false면 드라이버가 현 위치 유지(명령 무시)

# --- 상단 yaw (스텝, 위치) ---
float32 top_yaw_target      # rad, 상단 yaw 스테이지 목표 각
bool    top_yaw_active      # false면 현 위치 유지
