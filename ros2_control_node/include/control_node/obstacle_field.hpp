// ============================================================================
//  obstacle_field.hpp  —  측면 근접센서 기반 간단 장애물 회피 (선언)
//
//  몸체 측면에 단 초음파 3 + ToF 3 센서의 거리값을 받아,
//  "그 방향으로 가려는 속도"를 안전하게 깎는다(velocity scaling 방식).
//
//  설계 의도:
//   - 경로계획(SLAM/코스트맵) 같은 무거운 회피가 아니라, 캡스톤 시연용
//     "간단 회피". 막힌 방향 성분만 부드럽게 줄인다.
//   - 제어기(FollowController)와 분리. 제어기는 "가고 싶은 속도"를 만들고,
//     ObstacleField 가 마지막에 "갈 수 있는 속도"로 보정한다.
//   - 따라서 어떤 모드(1,2,…)든 공통으로 끼울 수 있다.
//
//  동작 원리 (정렬도 가중 감속):
//   각 센서는 방향 d_i 와 거리 r_i 를 준다. 이동 속도벡터 v=(vx,vy) 가 센서
//   방향과 같은 쪽을 향할수록(정렬도 align=cos(Δθ)>0) 더 강하게 감속한다.
//   거리 기반 비율(stop=0…slow=1)을 정렬도로 가중해 속도"벡터 크기"를 줄인다.
//     - r >= slow_dist : 영향 없음(scale=1)
//     - stop_dist < r < slow_dist : 선형 감속
//     - r <= stop_dist 이고 정면(align=1) : 완전 정지(scale=0)
//   ※현재는 벡터 전체를 스케일한다(정면 장애물엔 감속/정지). 측면으로
//    "흘려보내는" 성분분해식 회피는 6센서 실장 후 과제로 둔다(헤더 TODO).
//
//  (정의: src/obstacle_field.cpp / 임계값: params.hpp ObstacleParams)
// ============================================================================
#ifndef CONTROL_NODE__OBSTACLE_FIELD_HPP_
#define CONTROL_NODE__OBSTACLE_FIELD_HPP_

#include <vector>

namespace control_node
{

class ObstacleField
{
public:
  ObstacleField() = default;

  // stop_dist : 이 거리 이하이면 해당 방향 이동 완전 차단 [m]
  // slow_dist : 이 거리부터 감속 시작 [m] (slow_dist > stop_dist 보장)
  void setThresholds(double stop_dist, double slow_dist);

  // 센서 측정값 갱신. distances[i] 와 directions[i] 가 같은 센서.
  //   distances  : 각 방향 장애물 거리 [m] (inf/큰값 = 비어있음)
  //   directions : 각 센서 방향 (몸체 기준 방위, rad, CCW+)
  void update(const std::vector<double> & distances,
              const std::vector<double> & directions);

  // 센서 데이터가 오래되어 신뢰 못할 때 비활성화
  void clear();

  // 유효한 센서 데이터를 들고 있는가
  bool hasData() const;

  // 속도벡터 보정.
  //   입력  (vx,vy) : 제어기가 만든 "가고 싶은" 몸체속도
  //   출력  (vx,vy) : 막힌 방향을 깎은 "갈 수 있는" 속도
  //  회전(yaw)은 건드리지 않는다(제자리 회전은 측면충돌 위험이 낮고,
  //  단순 회피 범위를 넘어서므로). 필요시 후속 확장.
  void apply(double & vx, double & vy) const;

private:
  // 두 각도의 차이를 [-pi, pi] 로 정규화
  static double angleDiff(double a, double b);

  double stop_dist_ = 0.20;   // m
  double slow_dist_ = 0.60;   // m
  std::vector<double> distances_;
  std::vector<double> directions_;
  bool have_data_ = false;
};

}  // namespace control_node

#endif  // CONTROL_NODE__OBSTACLE_FIELD_HPP_
