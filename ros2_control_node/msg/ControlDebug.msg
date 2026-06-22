# ControlDebug.msg
# 제어 노드 내부 상태 모니터링용 (실주행 튜닝: rqt_plot / ros2 topic echo)
# 활성 모드에서 매 제어 틱(50Hz) 발행. 드라이버는 이 토픽을 쓰지 않는다.
std_msgs/Header header

int32 mode                 # 현재 모드 (0=IDLE 1=FOLLOW 2=ROTATE ...)
bool  engaged              # 기준(선분 등) 캡처 완료 여부
bool  gesture_hold         # 손동작 세션 중(몸체 일시정지)
bool  owner_global_valid   # 주인 글로벌 추정 유효 여부

float32 owner_gx           # m, 주인 글로벌 위치 추정 x
float32 owner_gy           # m, 주인 글로벌 위치 추정 y
float32 seg_distance       # m, 유지 중인 선분 거리 D (모드1)
float32 seg_angle          # rad, 유지 중인 선분 글로벌각 φ (모드1)
float32 heading_offset     # rad, 손동작 헤딩 오프셋
float32 top_yaw_target     # rad, 상단 yaw 목표각 (발행값과 동일)

float32 cmd_vx             # m/s, 실제 발행된 몸체 속도 (클램프 후)
float32 cmd_vy             # m/s
float32 cmd_wz             # rad/s

# UI 튜닝 표시용: 현재(측정) vs 타겟(유지목표) 주인 거리/방위각
float32 owner_distance     # m, 현재 주인 수평거리 (카메라 측정)
float32 owner_azimuth      # rad, 현재 주인 방위각 (+우측)
float32 target_distance    # m, 유지 목표 거리 (모드별)
float32 target_azimuth     # rad, 유지 목표 방위각 (락온=0)
