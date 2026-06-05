// ============================================================================
//  types.cpp  —  공통 타입/헬퍼 정의 (선언: include/control_node/types.hpp)
// ============================================================================
#include "control_node/types.hpp"

#include <algorithm>
#include <cmath>

namespace control_node
{

// 각도를 [-pi, pi] 로 정규화
double wrapAngle(double a)
{
  while (a >  M_PI) { a -= 2.0 * M_PI; }
  while (a < -M_PI) { a += 2.0 * M_PI; }
  return a;
}

// dt 보호: 타이머 지연/시계 점프로 dt가 0 이하·1초 초과면 기본값 사용
double guardDt(double dt, double fallback)
{
  if (dt <= 0.0 || dt > 1.0) { return fallback; }
  return dt;
}

// 메카넘 속도만 0 (정지). 스텝 목표는 호출부에서 별도 관리.
void ControlCommand::stopBody()
{
  body_vx = body_vy = body_yaw_rate = 0.0;
}

// 전체 안전정지: 몸체 정지 + 스텝 비활성(드라이버가 현 위치 유지)
void ControlCommand::zero()
{
  stopBody();
  lift_active = false;
  top_yaw_active = false;
}

// ★발행 직전 최종 안전 클램프 (선언부 설명 참조)
//  - 평면속도: 벡터 "크기"를 줄임 (방향 보존 — 축별 클램프는 방향이 틀어짐)
//  - yaw rate: 절대값 클램프
void applySafetyLimits(ControlCommand & cmd, double v_max, double w_max)
{
  double v = std::hypot(cmd.body_vx, cmd.body_vy);
  if (v > v_max && v > 1e-9) {
    double scale = v_max / v;
    cmd.body_vx *= scale;
    cmd.body_vy *= scale;
  }
  cmd.body_yaw_rate = std::clamp(cmd.body_yaw_rate, -w_max, w_max);
}

}  // namespace control_node
