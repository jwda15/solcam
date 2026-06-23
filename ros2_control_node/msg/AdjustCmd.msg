# AdjustCmd.msg
# 손동작 supervisor(팀원) → 제어 노드 "조정 명령".
# 따봉 트리거 후의 손동작 하나가 이 메시지 하나로 변환된다는 그림.
#
# 사용법: param 에 무엇을 조정할지(아래 상수), value 에 값,
#         delta=true 면 증분(+=value), false 면 절대값(=value).
#
# 새 손동작 기능이 생기면 여기에 상수만 추가하면 된다 (확장 지점).
#   예) 모드3 pan 속도, 모드4 구도 프리셋 번호 등.
#
# 적용 범위:
#   SEG_*          : 모드1(FOLLOW) 전용 — 유지할 선분 조정
#   HEADING_OFFSET : 모드 공통 — 몸체(=촬영 카메라) 헤딩을 주인 방향에서
#                    offset 만큼 비튼다. OAK-D(상단 yaw)는 계속 주인 락온
#                    → 촬영 카메라가 주인이 아닌 방향을 보게 할 수 있다.
#   LIFT_HEIGHT    : 모드 공통 — 리프트 목표 높이 (z_min~z_max 클램프)
std_msgs/Header header

int32   param    # 조정 대상 (아래 상수)
float32 value    # 값
bool    delta    # true=증분, false=절대값

# --- param 상수 ---
int32 PARAM_SEG_DISTANCE=0     # 모드1 선분 거리 D [m]      (절대/증분)
int32 PARAM_SEG_ANGLE=1        # 모드1 선분 글로벌각 φ [rad] (절대/증분)
int32 PARAM_HEADING_OFFSET=2   # 몸체 헤딩 오프셋 [rad]      (절대/증분)
int32 PARAM_LIFT_HEIGHT=3      # 리프트 목표 높이 [m]        (절대/증분)
int32 PARAM_BODY_VX=4          # 로봇기준 전후 속도 [m/s]   (휠 jog, odom 무관·모든 모드)
int32 PARAM_BODY_VY=5          # 로봇기준 좌우 속도 [m/s]   (+ = 좌)
int32 PARAM_BODY_WZ=6          # 로봇기준 자전 각속도 [rad/s] (+ = 반시계)
