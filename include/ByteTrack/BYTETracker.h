#pragma once

#include "ByteTrack/STrack.h"
#include "ByteTrack/lapjv.h"
#include "ByteTrack/Object.h"

#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <vector>

namespace byte_track
{
class BYTETracker
{
public:
    using STrackPtr = std::shared_ptr<STrack>;

    BYTETracker(const int& frame_rate = 30,
                const int& track_buffer = 30,
                const float& track_thresh = 0.5,
                const float& high_thresh = 0.6,
                const float& match_thresh = 0.8,
                const float& std_weight_position = 1.f/20,
                const float& std_weight_velocity = 1.f/160,
                const float& std_weight_depth    = 1.f/20);
    ~BYTETracker();

    std::vector<STrackPtr> update(const std::vector<Object>& objects);

    // Lost 트랙 풀 조회 (외부 노출).
    // tracking_node가 owner_id 매칭 실패 시 KF 외삽값 메우는 용도.
    // 참조 반환이라 BYTETracker 수명 안에서만 유효.
    const std::vector<STrackPtr>& getLostStracks() const { return lost_stracks_; }

private:
    std::vector<STrackPtr> jointStracks(const std::vector<STrackPtr> &a_tlist,
                                        const std::vector<STrackPtr> &b_tlist) const;

    std::vector<STrackPtr> subStracks(const std::vector<STrackPtr> &a_tlist,
                                      const std::vector<STrackPtr> &b_tlist) const;

    void removeDuplicateStracks(const std::vector<STrackPtr> &a_stracks,
                                const std::vector<STrackPtr> &b_stracks,
                                std::vector<STrackPtr> &a_res,
                                std::vector<STrackPtr> &b_res) const;

    void linearAssignment(const std::vector<std::vector<float>> &cost_matrix,
                          const int &cost_matrix_size,
                          const int &cost_matrix_size_size,
                          const float &thresh,
                          std::vector<std::vector<int>> &matches,
                          std::vector<int> &a_unmatched,
                          std::vector<int> &b_unmatched) const;

    // 1차 매칭 전용: Kalman 예측 상태와 detection 간 Mahalanobis cost.
    // (depth가 상태에 포함된 5D 관측이라 IoU가 아닌 통계적 거리를 쓴다.)
    std::vector<std::vector<float>> calcMahalanobisCost(const std::vector<STrackPtr> &a_tracks,
                                                        const std::vector<STrackPtr> &b_tracks) const;

    // 2차/미확정 매칭 전용: 순수 IoU 기반 cost (저신뢰도 detection에 사용)
    std::vector<std::vector<float>> calcIouCost(const std::vector<STrackPtr> &a_tracks,
                                                const std::vector<STrackPtr> &b_tracks) const;

    // IoU 행렬 계산 (calcIouCost / removeDuplicateStracks의 하위 헬퍼)
    std::vector<std::vector<float>> calcIouMatrix(const std::vector<Rect<float>> &a_rect,
                                                  const std::vector<Rect<float>> &b_rect) const;

    double execLapjv(const std::vector<std::vector<float> > &cost,
                     std::vector<int> &rowsol,
                     std::vector<int> &colsol,
                     bool extend_cost = false,
                     float cost_limit = std::numeric_limits<float>::max(),
                     bool return_cost = true) const;

private:
    const float track_thresh_;
    const float high_thresh_;
    const float match_thresh_;
    const size_t max_time_lost_;

    // Kalman 노이즈 파라미터 (저장해서 STrack 생성 시 전달)
    const float std_weight_position_;
    const float std_weight_velocity_;
    const float std_weight_depth_;

    size_t frame_id_;
    size_t track_id_count_;

    std::vector<STrackPtr> tracked_stracks_;
    std::vector<STrackPtr> lost_stracks_;
    std::vector<STrackPtr> removed_stracks_;
};
}