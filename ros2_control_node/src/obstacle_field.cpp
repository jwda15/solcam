// ============================================================================
//  obstacle_field.cpp  —  ObstacleField 정의
//  (선언/설계 설명: include/control_node/obstacle_field.hpp)
// ============================================================================
#include "control_node/obstacle_field.hpp"

#include <algorithm>
#include <cmath>

namespace control_node
{

void ObstacleField::setThresholds(double stop_dist, double slow_dist)
{
  stop_dist_ = stop_dist;
  slow_dist_ = std::max(slow_dist, stop_dist + 1e-3);   // slow > stop 보장
}

void ObstacleField::update(const std::vector<double> & distances,
                           const std::vector<double> & directions)
{
  distances_ = distances;
  directions_ = directions;
  have_data_ = !distances_.empty() &&
               distances_.size() == directions_.size();
}

void ObstacleField::clear()
{
  have_data_ = false;
}

bool ObstacleField::hasData() const
{
  return have_data_;
}

// ----------------------------------------------------------------------------
//  속도벡터 보정: 이동방향과 같은 쪽을 보는 센서 중 가장 가까운 장애물
//  기준으로 속도를 스케일다운. (원리는 헤더 상단 주석 참조)
// ----------------------------------------------------------------------------
void ObstacleField::apply(double & vx, double & vy) const
{
  if (!have_data_) { return; }

  double speed = std::hypot(vx, vy);
  if (speed < 1e-6) { return; }   // 정지 중엔 보정 불필요

  // 이동 방향(몸체 기준 방위). atan2(vy, vx): +x전방, +y좌측 기준.
  double move_dir = std::atan2(vy, vx);

  // 모든 센서를 훑어, 이동방향과 "같은 쪽"을 보는 센서 중
  // 가장 강한 감속(가장 가까운 장애물)을 채택한다.
  double min_scale = 1.0;

  for (size_t i = 0; i < distances_.size(); ++i) {
    double r = distances_[i];
    if (!std::isfinite(r) || r >= slow_dist_) { continue; }  // 멀면 무시

    // 이동방향과 센서방향의 각도차. cos>0 이면 그 방향으로 가는 중.
    double dtheta = angleDiff(move_dir, directions_[i]);
    double align = std::cos(dtheta);
    if (align <= 0.0) { continue; }   // 센서 반대쪽으로 가면 무관

    // 거리 기반 감속 비율 (stop=0 … slow=1)
    double dist_scale;
    if (r <= stop_dist_) {
      dist_scale = 0.0;
    } else {
      dist_scale = (r - stop_dist_) / (slow_dist_ - stop_dist_);
    }

    // 정렬도(align)만큼 가중: 정면으로 향할수록 더 강하게 적용
    double scale = 1.0 - align * (1.0 - dist_scale);
    min_scale = std::min(min_scale, scale);
  }

  min_scale = std::clamp(min_scale, 0.0, 1.0);
  vx *= min_scale;
  vy *= min_scale;
}

double ObstacleField::angleDiff(double a, double b)
{
  double d = a - b;
  while (d >  M_PI) { d -= 2.0 * M_PI; }
  while (d < -M_PI) { d += 2.0 * M_PI; }
  return d;
}

}  // namespace control_node
