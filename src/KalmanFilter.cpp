#include "ByteTrack/KalmanFilter.h"

#include <cstddef>
#include <stdexcept>
#include <algorithm>

namespace {
// [0524 z단위 수정] depth(z)의 노이즈 스케일 하한 (mm).
// 기존엔 z 노이즈를 픽셀 높이 h에 비례시켰으나(차원 불일치),
// stereo depth 오차는 거리에 비례해 커지므로 z 자체에 비례시킨다.
// z가 0에 가까울 때 노이즈가 0으로 죄어드는 걸 막는 하한.
// (OAK 근거리 한계 ~200mm 고려, 0.5m 기준 노이즈는 보장)
constexpr float kDepthScaleFloorMm = 500.0f;

// z 노이즈 스케일 계산: max(z, floor)에 비례.
inline float depthNoiseScale(float z)
{
    return std::max(z, kDepthScaleFloorMm);
}
} // namespace

byte_track::KalmanFilter::KalmanFilter(const float& std_weight_position,
                                       const float& std_weight_velocity,
                                       const float& std_weight_depth) :
    std_weight_position_(std_weight_position),
    std_weight_velocity_(std_weight_velocity),
    std_weight_depth_(std_weight_depth)
{
    // --------------------------------------------------------
    // 상태 인덱스: [x, y, z, a, h, vx, vy, vz, va, vh]
    //                     0  1  2  3  4   5   6   7   8   9
    // ndim=5: 위치 변수 개수 (대응하는 속도 5개도 자동으로 성립)
    // --------------------------------------------------------
    constexpr size_t ndim = 5;
    constexpr float dt = 1.0f;  // 1 프레임 = 1 tick

    // F 행렬 초기화: 10x10 단위행렬
    motion_mat_ = Eigen::Matrix<float, 10, 10, Eigen::RowMajor>::Identity();

    // H 행렬 초기화: 5x10 영행렬
    update_mat_ = Eigen::Matrix<float, 5, 10, Eigen::RowMajor>::Zero();

    // F 행렬 설정: 위치에 속도를 더함
    // motion_mat_[i][i+5] = dt → x += vx, y += vy, z += vz, ...
    for (size_t i = 0; i < ndim; i++)
    {
        motion_mat_(i, ndim + i) = dt;
    }

    // H 행렬 설정: 상태 [0~4]를 간단히 관측 (속도는 관측 안 함)
    // update_mat_[i][i] = 1 → [x,y,z,a,h] 직접 관측
    for (size_t i = 0; i < ndim; i++)
    {
        update_mat_(i, i) = 1.0f;
    }
}

void byte_track::KalmanFilter::initiate(StateMean &mean, StateCov &covariance, const DetectBox &measurement)
{
    // 상태 초기화: 위치는 detection으로, 속도는 0으로 설정
    // mean = [x, y, z, a, h, 0, 0, 0, 0, 0]
    mean.block<1, 5>(0, 0) = measurement.block<1, 5>(0, 0);  // 위치 복사
    mean.block<1, 5>(0, 5) = Eigen::Matrix<float, 1, 5>::Zero();  // 속도 = 0

    // 노이즈 초기값은 bbox 높이(h)에 비례해서 설정
    // 비슷한 크기의 객체라면 비슷한 노이즈 스케일을 나타냄
    const float h = measurement[4];  // bbox 높이
    const float zc = depthNoiseScale(measurement[2]);  // [0524] z 노이즈 스케일 (depth에 비례)

    StateMean std_vec;
    // --- 위치 원소 노이즈 (2배 가중치: 초기 불확실성 크게) ---
    std_vec(0) = 2 * std_weight_position_ * h;  // x 노이즈
    std_vec(1) = 2 * std_weight_position_ * h;  // y 노이즈
    std_vec(2) = 2 * std_weight_depth_    * zc; // z 노이즈 (depth, mm) [0524: h→zc]
    std_vec(3) = 1e-2f;                          // a 노이즈 (aspect ratio는 안정적)
    std_vec(4) = 2 * std_weight_position_ * h;  // h 노이즈
    // --- 속도 원소 노이즈 (10배: 속도 초기값이 0이라 어디든 간 불확실) ---
    std_vec(5) = 10 * std_weight_velocity_ * h; // vx 노이즈
    std_vec(6) = 10 * std_weight_velocity_ * h; // vy 노이즈
    std_vec(7) = 10 * std_weight_depth_    * zc;// vz 노이즈 [0524: h→zc]
    std_vec(8) = 1e-5f;                          // va 노이즈 (aspect ratio 변화 거의 없음)
    std_vec(9) = 10 * std_weight_velocity_ * h; // vh 노이즈

    // 공분산 = std^2 대각 행렬
    StateMean tmp = std_vec.array().square();
    covariance = tmp.asDiagonal();
}

void byte_track::KalmanFilter::predict(StateMean &mean, StateCov &covariance)
{
    // 현재 상태의 h를 기준으로 프로세스 노이즈 코배리언스 Q 계산
    const float h = mean(4);  // bbox 높이
    const float zc = depthNoiseScale(mean(2));  // [0524] z 노이즈 스케일 (depth에 비례)

    StateMean std_vec;
    // 예측 단계의 노이즈는 initiate보다 작은 가중치 사용
    std_vec(0) = std_weight_position_ * h;   // x
    std_vec(1) = std_weight_position_ * h;   // y
    std_vec(2) = std_weight_depth_    * zc;  // z [0524: h→zc]
    std_vec(3) = 1e-2f;                       // a
    std_vec(4) = std_weight_position_ * h;   // h
    std_vec(5) = std_weight_velocity_ * h;   // vx
    std_vec(6) = std_weight_velocity_ * h;   // vy
    std_vec(7) = std_weight_depth_    * zc;  // vz [0524: h→zc]
    std_vec(8) = 1e-5f;                       // va
    std_vec(9) = std_weight_velocity_ * h;   // vh

    // Q 행렬: 프로세스 노이즈 공분산
    StateMean tmp = std_vec.array().square();
    StateCov motion_cov = tmp.asDiagonal();

    // 예측: mean = mean * F^T,  P = F * P * F^T + Q
    mean = mean * motion_mat_.transpose();
    covariance = motion_mat_ * covariance * motion_mat_.transpose() + motion_cov;
}

void byte_track::KalmanFilter::update(StateMean &mean, StateCov &covariance, const DetectBox &measurement)
{
    // 1. 현재 상태를 관측 공간으로 투영 (H*x, S)
    StateHMean projected_mean;
    StateHCov  projected_cov;
    project(projected_mean, projected_cov, mean, covariance);

    // 2. Kalman gain K = P*H^T * S^-1
    //    Cholesky 분해로 S^-1 연산 안정적으로 해결
    Eigen::Matrix<float, 5, 10> B = (covariance * update_mat_.transpose()).transpose();
    Eigen::Matrix<float, 10, 5> kalman_gain = (projected_cov.llt().solve(B)).transpose();

    // 3. innovation = 실제 관측값 - 예측 관측값
    Eigen::Matrix<float, 1, 5> innovation = measurement - projected_mean;

    // 4. 상태 보정: mean += K * innovation
    const auto correction = innovation * kalman_gain.transpose();
    mean = (mean.array() + correction.array()).matrix();

    // 5. 공분산 보정: P -= K * S * K^T
    covariance = covariance - kalman_gain * projected_cov * kalman_gain.transpose();
}

float byte_track::KalmanFilter::gatingDistance(const StateMean& mean, const StateCov& covariance,
                                                const DetectBox& measurement) const
{
    // 현재 상태를 관측 공간으로 투영
    StateHMean projected_mean;
    StateHCov  projected_cov;
    // project()가 private이지만 로직적으로는 const 함수이므로 const_cast 사용
    const_cast<KalmanFilter*>(this)->project(
        projected_mean, projected_cov, mean, covariance);

    // d = z - H*x (관측값 - 예측값)
    Eigen::Matrix<float, 1, 5> d = measurement - projected_mean;

    // Mahalanobis 거리 = d * S^-1 * d^T
    // Cholesky 분해로 S^-1 연산 (수치 안정적)
    Eigen::LLT<Eigen::Matrix<float, 5, 5>> llt(projected_cov);
    if (llt.info() != Eigen::Success)
    {
        // Cholesky 실패 (S가 양정치 행렬이 아닐 때) → 유클리드 거리로 fallback
        return d.squaredNorm();
    }
    // L^-1 * d^T 를 계산해서 제곱합 = d * S^-1 * d^T
    Eigen::Matrix<float, 5, 1> z = llt.matrixL().solve(d.transpose());
    return z.squaredNorm();  // = 카이제곱 통계량 (5-DOF)
}

void byte_track::KalmanFilter::project(StateHMean &projected_mean, StateHCov &projected_covariance,
                                       const StateMean& mean, const StateCov& covariance)
{
    const float h = mean(4);  // bbox 높이 기준으로 노이즈 스케일
    const float zc = depthNoiseScale(mean(2));  // [0524] z 관측 노이즈 스케일 (depth에 비례)

    // 관측 노이즈 R: 각 관측 변수의 측정 노이즈
    DetectBox std_vec;
    std_vec << std_weight_position_ * h,   // x 노이즈
               std_weight_position_ * h,   // y 노이즈
               std_weight_depth_    * zc,  // z 노이즈 (depth는 별도 가중치) [0524: h→zc]
               1e-1f,                       // a 노이즈 (이 값이 크면 aspect ratio를 많이 신뢰)
               std_weight_position_ * h;   // h 노이즈

    // 예측 수단: H*x,  H*P*H^T
    projected_mean = mean * update_mat_.transpose();
    projected_covariance = update_mat_ * covariance * update_mat_.transpose();

    // R 더하기: 투영된 공분산에 관측 노이즈 삽입
    Eigen::Matrix<float, 5, 5> diag = std_vec.asDiagonal();
    projected_covariance += diag.array().square().matrix();
}
