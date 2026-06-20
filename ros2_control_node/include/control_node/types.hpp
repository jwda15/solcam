// ============================================================================
//  types.hpp  —  제어 노드 공통 타입 / 좌표계 헬퍼 (선언; 정의는 types.cpp)
//
//  좌표계 약속 (전체 패키지 공통):
//   - 글로벌(odom) 평면: x_g, y_g [m], yaw_g [rad, CCW+]. 휠 오도메트리가 추정.
//     원점/방향은 노드 시작(또는 모드 진입) 시점 기준 상대 프레임.
//   - 몸체(base): x 전방(+), y 좌측(+), yaw CCW(+).  (ROS REP-103)
//   - 카메라(OAK): spatial_x 우측(+), spatial_y 아래(+), spatial_z 전방(+).
//     azimuth = atan2(spatial_x, spatial_z) → 주인이 우측이면 +.
//   - 상단 yaw 스테이지: theta_head [rad, CCW+], 몸체 전방 기준. 스텝모터라 정확.
//
//  "선분 유지"(모드1): 주인-로봇을 잇는 선분의 길이(D)와 글로벌 방향을
//   처음 세팅값으로 유지. 주인 글로벌 위치는 (로봇 글로벌 포즈 + 상단yaw각 +
//   카메라 azimuth/distance)로 합성한다. 주인의 yaw(방향)는 쓰지 않음(점으로만).
// ============================================================================
#ifndef CONTROL_NODE__TYPES_HPP_
#define CONTROL_NODE__TYPES_HPP_

namespace control_node
{

// ----------------------------------------------------------------------------
//  Vec2 / Pose2D : 2D 평면 기하 (글로벌 위치·포즈 계산용)
// ----------------------------------------------------------------------------
struct Vec2
{
  double x = 0.0;
  double y = 0.0;
};

struct Pose2D
{
  double x   = 0.0;   // m
  double y   = 0.0;   // m
  double yaw = 0.0;   // rad, CCW+
};

// 각도를 [-pi, pi] 로 정규화
double wrapAngle(double a);

// dt 보호: 0 이하·비정상적으로 큰 값이면 기본값으로 대체
double guardDt(double dt, double fallback = 0.02);

struct ControlCommand;   // 전방 선언 (아래 정의)

// ★발행 직전 최종 안전 클램프 — 평면속도 벡터 크기 ≤ v_max, |yaw rate| ≤ w_max.
//  제어기 내부에도 한계가 있지만, 모드/제어기 버그·슬루 과도 구간까지 막는
//  "마지막 관문". ControlNode::publish() 가 호출. (한계값: params.hpp 맨 위)
void applySafetyLimits(ControlCommand & cmd, double v_max, double w_max);

// ----------------------------------------------------------------------------
//  RobotOdom : 휠 오도메트리가 준 로봇 글로벌 포즈 스냅샷.
//  IMU 없음 → yaw 는 휠 적분값(장기 드리프트 있음). 제어 입력으로 사용.
// ----------------------------------------------------------------------------
struct RobotOdom
{
  bool   valid = false;
  Pose2D pose;            // 글로벌 x,y,yaw
};

// ----------------------------------------------------------------------------
//  OwnerState : 트래킹 노드(/owner_pose)가 준 주인 상태의 스냅샷.
//  제어기는 ROS 메시지가 아니라 이 구조체를 본다(테스트 용이성).
// ----------------------------------------------------------------------------
struct OwnerState
{
  bool   is_detected = false;
  double spatial_x   = 0.0;
  double spatial_y   = 0.0;   // +아래
  double spatial_z   = 0.0;   // +전방
  double azimuth     = 0.0;   // rad, +우측
  double distance    = 0.0;   // m (수평거리)
  double confidence  = 0.0;
  int    track_id    = -1;
};

// ----------------------------------------------------------------------------
//  ControlCommand : 제어기가 산출하는 6자유도 명령 (내부 표현).
//
//  ★모터 종류에 맞춘 명령 의미:
//    - 메카넘(4): 속도 명령  → body_vx, body_vy, body_yaw_rate
//    - 리프트(스텝): 위치 명령 → lift_height_target (목표 높이[m])
//    - 상단 yaw(스텝): 위치 명령 → top_yaw_target (목표 각[rad])
//  스텝모터는 "초당 몇" 보다 "어디로"가 자연스러우므로 위치로 보낸다.
//  드라이버 노드(팀원)가 위치 명령을 스텝 펄스로 변환한다.
// ----------------------------------------------------------------------------
struct ControlCommand
{
  // 메카넘 (속도)
  double body_vx       = 0.0;   // m/s,  +전방
  double body_vy       = 0.0;   // m/s,  +좌측
  double body_yaw_rate = 0.0;   // rad/s, +CCW

  // 리프트 (스텝, 위치)
  double lift_height_target = 0.0;   // m, 목표 높이 (절대)
  bool   lift_active        = false; // false면 드라이버가 현 위치 유지

  // 상단 yaw (스텝, 위치)
  double top_yaw_target = 0.0;   // rad, 목표 스테이지 각
  bool   top_yaw_active = false; // false면 현 위치 유지

  // 메카넘 속도만 0 (정지). 스텝 목표는 호출부에서 별도 관리.
  void stopBody();

  // 전체 안전정지: 몸체 정지 + 스텝 비활성(현 위치 유지)
  void zero();
};

// ----------------------------------------------------------------------------
//  UserAdjust : 손동작 supervisor가 /adjust_cmd 로 조정하는 "사용자 설정".
//  모드와 무관하게 ControlNode 가 들고 있다가 매 스텝 제어기에 넘긴다
//  → 모드를 바꿔도 오프셋/리프트 설정이 유지된다.
//
//  heading_offset: 몸체(=촬영 카메라) 헤딩을 주인 방향에서 비트는 각.
//    0 이면 촬영 카메라도 주인을 정면으로. ±면 OAK-D(상단 yaw, 주인 락온
//    유지)와 다른 방향을 촬영 → "주인 외 풍경 촬영" 같은 연출.
// ----------------------------------------------------------------------------
struct UserAdjust
{
  double heading_offset = 0.0;   // rad, 몸체 헤딩 오프셋 (CCW+)

  // ----- 리프트: 시간(꾹 누름) 기반 제어 -----
  //  스텝모터 위치피드백이 없어 "절대 목표"가 누적·드리프트로 안 멈추는 문제 →
  //  손동작 명령이 들어오는 "동안만" 방향에 맞게 행정 끝점으로 보내 이동시키고,
  //  명령이 끊기면(손 뗌) 정지. control_node 가 lift_active_now 를 채운다.
  int  lift_dir        = 0;      // +1=올림 / -1=내림 / 0=명령 없음
  bool lift_active_now = false;  // 손동작 리프트 명령이 최근(타임아웃 내) 들어왔는가
};

// ----------------------------------------------------------------------------
//  주행 모드. FOLLOW(모드1)·ROTATE(모드2) 구현. 나머지는 자리만.
// ----------------------------------------------------------------------------
enum class Mode : int
{
  IDLE       = 0,   // 대기: 모든 출력 0
  FOLLOW     = 1,   // 선분(거리+글로벌각) 유지 (구현됨)
  ROTATE     = 2,   // 제자리 회전 추적: 위치 고정, 몸체 yaw만 주인 추종 (구현됨)
  FOLLOW2    = 3,   // leash(줄): 거리만 유지, 구도/odom 불필요 (구현됨)
  ORBIT      = 4,   // 공전: 주인 중심 반지름 유지하며 천천히 돌기 (구현됨)
};

}  // namespace control_node

#endif  // CONTROL_NODE__TYPES_HPP_
