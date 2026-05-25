#include "ByteTrack/STrack.h"

#include <cstddef>

// --------------------------------------------------------
// STrack 생성자
// detection 하나로 STrack 객체 설정. 이 시점에는 track_id가 없음
// activate() 호출 시 track_id 부여
// --------------------------------------------------------
byte_track::STrack::STrack(const Rect<float>& rect, const float& depth, const float& score,
                           const float& std_weight_position,
                           const float& std_weight_velocity,
                           const float& std_weight_depth) :
    kalman_filter_(std_weight_position, std_weight_velocity, std_weight_depth),
    mean_(),                // 10D 상태 벡터 (activate 전까지는 미사용)
    covariance_(),          // 10x10 공분산 행렬
    rect_(rect),            // 2D bbox
    depth_(depth),          // OAK-D에서 받은 depth (mm)
    state_(STrackState::New), // 추적 시작은 New 상태
    is_activated_(false),   // activate() 호출 전엔 false
    score_(score),          // detection confidence
    track_id_(0),           // 아직 ID 미부여
    frame_id_(0),
    start_frame_id_(0),
    tracklet_len_(0)
{
}

byte_track::STrack::~STrack()
{
}

// --- Getter 구현체 ---

const byte_track::Rect<float>& byte_track::STrack::getRect() const
{
    return rect_;  // Kalman으로 복원되거나 detection으로 업데이트된 2D bbox
}

const byte_track::STrackState& byte_track::STrack::getSTrackState() const
{
    return state_;
}

const bool& byte_track::STrack::isActivated() const
{
    return is_activated_;  // false면 비활성 트랙, 매칭은 되지만 출력 안 됨
}

const float& byte_track::STrack::getScore() const
{
    return score_;
}

const size_t& byte_track::STrack::getTrackId() const
{
    return track_id_;  // 1부터 시작, activate() 호출 시 track_id_count_++로 부여
}

const size_t& byte_track::STrack::getFrameId() const
{
    return frame_id_;  // 마지막으로 update()/activate() 된 프레임 번호
}

const size_t& byte_track::STrack::getStartFrameId() const
{
    return start_frame_id_;  // removeDuplicateStracks에서 얼마나 오래된 트랙인지 판단에 사용
}

const size_t& byte_track::STrack::getTrackletLength() const
{
    return tracklet_len_;  // update() 호출 횟수, 직선 추적 지속 시간
}

// [3D 확장] 현재 depth 값 getter
const float& byte_track::STrack::getDepth() const
{
    return depth_;
}

void byte_track::STrack::activate(const size_t& frame_id, const size_t& track_id)
{
    // 5D measurement 조립: [x, y, z, a, h]
    // Rect.getXyah() = (cx, cy, a, h)에 depth를 끄워 넣음
    KalmanFilter::DetectBox meas;
    const auto xyah = rect_.getXyah();
    meas << xyah[0], xyah[1], depth_, xyah[2], xyah[3];  // x, y, z, a, h
    kalman_filter_.initiate(mean_, covariance_, meas);    // Kalman 상태 시작

    updateRect();  // mean_에서 rect_ 업데이트

    state_ = STrackState::Tracked;
    // 모든 트랙은 2연속 매칭 후 확정 (non_active → update()에서 is_activated_=true)
    // 1프레임 즉시 활성화 특례 제거: 초반 깜빡임 원인이었음
    track_id_ = track_id;        // BYTETracker가 부여한 ID
    frame_id_ = frame_id;
    start_frame_id_ = frame_id;
    tracklet_len_ = 0;
}

void byte_track::STrack::reActivate(const STrack &new_track, const size_t &frame_id, const int &new_track_id)
{
    // lost 실단 트랙이 detection으로 다시 매칭되었을 때
    // Kalman update()로 상태 보정
    KalmanFilter::DetectBox meas;
    const auto xyah = new_track.getRect().getXyah();
    meas << xyah[0], xyah[1], new_track.getDepth(), xyah[2], xyah[3];
    kalman_filter_.update(mean_, covariance_, meas);

    updateRect();  // Kalman 보정 후 rect_ 복원

    state_ = STrackState::Tracked;
    is_activated_ = true;
    score_ = new_track.getScore();
    depth_ = new_track.getDepth();  // depth도 갱신
    if (0 <= new_track_id)
    {
        track_id_ = new_track_id;  // 선택적 ID 교체 (일반적으로 -1 넘겨 기존 ID 유지)
    }
    frame_id_ = frame_id;
    tracklet_len_ = 0;  // 재활성 후는 길이 리셋
}

void byte_track::STrack::predict()
{
    if (state_ != STrackState::Tracked)
    {
        // Lost 상태에서는 높이 변화 속도(vh)를 0으로 고정
        // 높이가 나타날 때는 동일한 크기라고 가정
        // [3D 확장] 10D state에서 vh는 인덱스 9번
        mean_[9] = 0;
    }
    kalman_filter_.predict(mean_, covariance_);  // F*mean, F*P*F^T + Q
}

void byte_track::STrack::update(const STrack &new_track, const size_t &frame_id)
{
    // 정상 매칭된 detection으로 Kalman 보정
    KalmanFilter::DetectBox meas;
    const auto xyah = new_track.getRect().getXyah();
    meas << xyah[0], xyah[1], new_track.getDepth(), xyah[2], xyah[3];
    kalman_filter_.update(mean_, covariance_, meas);

    updateRect();  // Kalman 보정 후 2D bbox 갱신

    state_ = STrackState::Tracked;
    is_activated_ = true;
    score_ = new_track.getScore();
    depth_ = new_track.getDepth();  // [3D 확장] depth도 갱신
    frame_id_ = frame_id;
    tracklet_len_++;  // 지속 시간 증가
}

void byte_track::STrack::markAsLost()
{
    state_ = STrackState::Lost;  // detection 누락, Kalman만으로 위치 유지
}

void byte_track::STrack::markAsRemoved()
{
    state_ = STrackState::Removed;  // max_time_lost 초과, 완전 제거 대상
}

void byte_track::STrack::updateRect()
{
    // Kalman mean_에서 2D bbox를 복원
    // mean_ 형식: [x, y, z, a, h, ...] 여기서 x=cx, y=cy
    // a = width/height, h = height
    rect_.width()  = mean_[3] * mean_[4];  // width = a * h
    rect_.height() = mean_[4];             // height = h
    rect_.x()      = mean_[0] - rect_.width()  / 2;  // tl_x = cx - w/2
    rect_.y()      = mean_[1] - rect_.height() / 2;  // tl_y = cy - h/2
}
