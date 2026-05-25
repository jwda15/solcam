#pragma once

#include "ByteTrack/Rect.h"
#include "ByteTrack/KalmanFilter.h"

#include <cstddef>

namespace byte_track
{
// STrack 상태머신: 트랙이 현재 어떤 단계에 있는지
enum class STrackState {
    New     = 0,  // 신규 탐지, 아직 확정 전
    Tracked = 1,  // 정상 추적 중
    Lost    = 2,  // 일시적으로 detection 안 됨 (복구 대기 중)
    Removed = 3,  // max_time_lost 초과 또는 제거
};

// STrack: 한 명의 사람(track_id)를 나타내는 클래스
// 내부에 KalmanFilter를 갖고 있어서 detection에 보정하거나 독립적으로 예측
// BYTETracker가 shared_ptr<STrack>으로 관리
class STrack
{
public:
    // 생성자: detection 하나로 STrack 초기화
    // depth는 OAK-D spatialCoords.z (mm), 아직 track_id 미부여
    STrack(const Rect<float>& rect, const float& depth, const float& score,
           const float& std_weight_position = 1.f/20,
           const float& std_weight_velocity = 1.f/160,
           const float& std_weight_depth    = 1.f/20);
    ~STrack();

    // --------------------------------------------------------
    // Getter: 트랙 상태 조회
    // --------------------------------------------------------
    const Rect<float>& getRect() const;            // 2D bbox
    const STrackState& getSTrackState() const;     // Tracked/Lost 등
    const bool& isActivated() const;              // activate() 호출 여부
    const float& getScore() const;                // detection confidence
    const size_t& getTrackId() const;             // 고유 ID (1부터 증가)
    const size_t& getFrameId() const;             // 마지막으로 업데이트된 프레임 번호
    const size_t& getStartFrameId() const;        // 시작 프레임 번호
    const size_t& getTrackletLength() const;      // 업데이트된 회수

    // [3D 확장] 현재 depth 값 (Kalman z 축 기준)
    const float& getDepth() const;

    // --------------------------------------------------------
    // [3D 확장] BYTETracker의 Mahalanobis 코스트 계산에서
    // Kalman 내부 상태를 읽어야 하므로 getter 노출
    // --------------------------------------------------------
    const KalmanFilter& getKalmanFilter() const { return kalman_filter_; }
    const KalmanFilter::StateMean& getMean() const { return mean_; }
    const KalmanFilter::StateCov& getCovariance() const { return covariance_; }

    // --------------------------------------------------------
    // 트랙 상태 전환
    // --------------------------------------------------------

    // 신규 트랙 확정 등록: Kalman initiate() 호출 + track_id 부여
    void activate(const size_t& frame_id, const size_t& track_id);

    // lost 실단 detection이 다시 나타났을 때 재활성화
    void reActivate(const STrack &new_track, const size_t &frame_id, const int &new_track_id = -1);

    // detection 없이 Kalman만으로 위치 예측 (Lost 상태일 때 vh=0)
    void predict();

    // 매칭된 detection으로 Kalman 업데이트 + rect_, depth_ 갱신
    void update(const STrack &new_track, const size_t &frame_id);

    // 상태 표시만 바꿀, 트랙 데이터는 유지
    void markAsLost();
    void markAsRemoved();

private:
    KalmanFilter kalman_filter_;       // 이 트랙 전용 Kalman 필터 인스턴스
    KalmanFilter::StateMean mean_;     // 10D 상태 벡터 [x,y,z,a,h,vx,vy,vz,va,vh]
    KalmanFilter::StateCov covariance_; // 10x10 공분산 행렬

    Rect<float> rect_;   // 현재 2D bbox (관측 업데이트 or Kalman 복원)
    float depth_;        // 현재 depth 추정값 (mm), OAK-D에서 수신
    STrackState state_;  // 현재 트랙 상태

    bool is_activated_;        // activate() 호출 여부 (false면 아직 미확정)
    float score_;              // 가장 최근 detection score
    size_t track_id_;          // 고유 추적 ID
    size_t frame_id_;          // 마지막 업데이트 프레임
    size_t start_frame_id_;    // 이 트랙이 시작된 프레임
    size_t tracklet_len_;      // update() 호출 횟수 (track 지속 길이)

    // Kalman mean에서 rect_을 백프로 복원
    // mean_[0~3] = (cx, cy, a, h) → rect_ (x, y, w, h) 로 변환
    void updateRect();
};
}