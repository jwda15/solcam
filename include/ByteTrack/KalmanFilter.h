#pragma once

#include "Eigen/Dense"
#include "ByteTrack/Rect.h"

namespace byte_track
{
// ============================================================
// [3D 확장] 5차원 관측 벡터 타입: (x, y, z, a, h)
//   x, y : bbox 중심 픽셀 좌표
//   z    : OAK-D가 리턴한 depth (mm)
//   a    : aspect ratio = width / height
//   h    : bbox 높이 (픽셌)
// ============================================================
template<typename T>
using Xyzah = Eigen::Matrix<T, 1, 5, Eigen::RowMajor>;

class KalmanFilter
{
public:
    // --------------------------------------------------------
    // 타입 별칭 정의
    // --------------------------------------------------------

    // 관측값 (detection 스트럭): [x, y, z, a, h] 5차원
    using DetectBox = Xyzah<float>;

    // Kalman 상태 벡터: [x, y, z, a, h, vx, vy, vz, va, vh] 10차원
    // 앞 5개는 위치, 뒤 5개는 각 위치의 속도 (1프레임/tick)
    using StateMean = Eigen::Matrix<float, 1, 10, Eigen::RowMajor>;

    // 10x10 공분산 행렬: 각 상태 변수의 불확실성을 표현
    using StateCov  = Eigen::Matrix<float, 10, 10, Eigen::RowMajor>;

    // 관측 공간에서의 예측 평균 (H*x): 5차원
    using StateHMean = Eigen::Matrix<float, 1, 5, Eigen::RowMajor>;

    // 관측 공간에서의 예측 공분산 (S = H*P*H^T + R): 5x5
    using StateHCov  = Eigen::Matrix<float, 5, 5, Eigen::RowMajor>;

    // --------------------------------------------------------
    // 생성자 파라미터
    //   std_weight_position : x, y, h 노이즈 가중치 (1/20 = 기본값)
    //   std_weight_velocity : vx, vy, vh 노이즈 가중치 (1/160)
    //   std_weight_depth    : z, vz 노이즈 가중치 (1/20 = 기본은 xy와 동일)
    //                         OAK-D depth 노이즈가 크면 이 값을 크게
    // --------------------------------------------------------
    KalmanFilter(const float& std_weight_position = 1. / 20,
                 const float& std_weight_velocity = 1. / 160,
                 const float& std_weight_depth    = 1. / 20);

    // 새 트랙 초기화: detection 하나로 mean, covariance 설정
    // 속도 초기값은 0, 불확실성은 위치 크기에 비례해서 크게 설정
    void initiate(StateMean& mean, StateCov& covariance, const DetectBox& measurement);

    // 예측 단계: 등속도 모델로 x_new = x + v*dt 적용
    // detection 없이도 매 프레임 호출해서 위치를 시간 엄데이트
    void predict(StateMean& mean, StateCov& covariance);

    // 보정 단계: 새 detection으로 Kalman 상태 업데이트
    // 예측값과 관측값을 가중 합산해서 최적해 추정
    void update(StateMean& mean, StateCov& covariance, const DetectBox& measurement);

    // --------------------------------------------------------
    // [Mahalanobis 거리] 예측 상태 vs 주어진 detection 간 거리 계산
    //
    // 공식: d = (z - H*x),  dist = d * S^-1 * d^T
    //   z   : 실제 detection [x,y,z,a,h]
    //   H*x : Kalman이 예측한 관측값
    //   S   : 관측 공분산 (H*P*H^T + R)
    //
    // 반환값: 카이제곱 거리 (5-DOF 기준 95% 임계: 11.070)
    //   거리 < 11.070 → 신뢰할 수 있는 매칭
    //   거리 > 11.070 → 다른 사람일 가능성 높음
    // --------------------------------------------------------
    float gatingDistance(const StateMean& mean, const StateCov& covariance,
                         const DetectBox& measurement) const;

private:
    float std_weight_position_;  // xy 위치 노이즈 가중치
    float std_weight_velocity_;  // xy 속도 노이즈 가중치
    float std_weight_depth_;     // z (depth) 노이즈 가중치

    // F 행렬 (10x10): 상태 전이 행렬
    // x_new = F * x  (등속도 모델)
    Eigen::Matrix<float, 10, 10, Eigen::RowMajor> motion_mat_;

    // H 행렬 (5x10): 상태 오 관측 변환 행렬
    // 상태 [x,y,z,a,h, vx,vy,vz,va,vh] → 관측 [x,y,z,a,h]
    Eigen::Matrix<float, 5, 10, Eigen::RowMajor>  update_mat_;

    // 관측 공간 투영: mean, covariance → 관측 공간으로 변환 + 노이즈 추가
    void project(StateHMean& projected_mean, StateHCov& projected_covariance,
                 const StateMean& mean, const StateCov& covariance);
};
}