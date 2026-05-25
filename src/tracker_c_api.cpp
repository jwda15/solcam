#include "ByteTrack/tracker_c_api.h"
#include "ByteTrack/BYTETracker.h"
#include "ByteTrack/Object.h"
#include "ByteTrack/Rect.h"
#include <vector>
#include <algorithm>

extern "C" {

void* create_tracker(int frame_rate, int track_buffer,
                     float track_thresh, float high_thresh, float match_thresh,
                     float std_weight_position, float std_weight_velocity, float std_weight_depth)
{
    // BYTETracker 생성 (Kalman 노이즈 파라미터 전달)
    return new byte_track::BYTETracker(frame_rate, track_buffer,
                                       track_thresh, high_thresh, match_thresh,
                                       std_weight_position, std_weight_velocity, std_weight_depth);
}

int update_tracker(void* tracker_ptr,
                   CObject* dets, int n_dets,
                   CTrackResult* results, int max_results)
{
    auto* tracker = reinterpret_cast<byte_track::BYTETracker*>(tracker_ptr);

    // CObject → byte_track::Object 변환
    std::vector<byte_track::Object> objects;
    objects.reserve(n_dets);
    for (int i = 0; i < n_dets; i++) {
        byte_track::Rect<float> rect(dets[i].x, dets[i].y, dets[i].w, dets[i].h);
        objects.emplace_back(rect, dets[i].depth, dets[i].label, dets[i].prob);
    }

    // 트래커 업데이트
    auto tracks = tracker->update(objects);

    // STrack → CTrackResult 변환
    int n = std::min((int)tracks.size(), max_results);
    for (int i = 0; i < n; i++) {
        auto tlbr = tracks[i]->getRect().getTlbr();
        results[i].x1       = tlbr[0];
        results[i].y1       = tlbr[1];
        results[i].x2       = tlbr[2];
        results[i].y2       = tlbr[3];
        results[i].track_id = (int)tracks[i]->getTrackId();
        results[i].label    = 0;
        results[i].score    = tracks[i]->getScore();
        results[i].depth    = tracks[i]->getKalmanDepth();  // [0525] raw 대신 Kalman 평활 z (위치와 동일 소스)
    }
    return n;
}

int get_lost_tracks(void* tracker_ptr,
                    CTrackResult* results, int max_results)
{
    auto* tracker = reinterpret_cast<byte_track::BYTETracker*>(tracker_ptr);
    const auto& lost = tracker->getLostStracks();

    int n = std::min((int)lost.size(), max_results);
    for (int i = 0; i < n; i++) {
        // lost STrack의 rect_은 markAsLost 시점의 마지막 알려진 bbox.
        // BYTETracker::update() 안에서 매 프레임 lost들도 STrack::predict() 거쳐서
        // mean_은 외삽됐지만 rect_은 갱신 안 됨 (updateRect 호출 안 함).
        // 그래서 mean_[0..4] = (cx, cy, z, a, h)로부터 직접 bbox 재구성.
        const auto& mean = lost[i]->getMean();
        float cx = mean[0];
        float cy = mean[1];
        float z  = mean[2];   // depth (mm)
        float a  = mean[3];   // aspect ratio = w/h
        float h  = mean[4];   // height
        float w  = a * h;

        results[i].x1       = cx - w * 0.5f;
        results[i].y1       = cy - h * 0.5f;
        results[i].x2       = cx + w * 0.5f;
        results[i].y2       = cy + h * 0.5f;
        results[i].track_id = (int)lost[i]->getTrackId();
        results[i].label    = 0;
        results[i].score    = lost[i]->getScore();
        results[i].depth    = z;
    }
    return n;
}

void destroy_tracker(void* tracker_ptr)
{
    delete reinterpret_cast<byte_track::BYTETracker*>(tracker_ptr);
}

} // extern "C"
