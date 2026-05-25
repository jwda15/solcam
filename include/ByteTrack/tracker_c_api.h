#pragma once

// Python ctypes에서 호출할 C 인터페이스
// C++ 클래스를 extern "C" 함수로 래핑

struct CObject {
    float x, y, w, h;  // bbox (top-left x, y, width, height) 픽셀
    float depth;        // OAK-D depth (mm), 없으면 0
    int   label;        // 클래스 ID (person=0)
    float prob;         // confidence
};

struct CTrackResult {
    float x1, y1, x2, y2;  // bbox (top-left, bottom-right)
    int   track_id;          // 고유 트랙 ID
    int   label;             // 클래스 ID
    float score;             // confidence
    float depth;             // KF z 추정 (mm). lost 트랙도 KF predict 결과가 들어감
};

extern "C" {
    // 트래커 생성: BYTETracker 인스턴스를 힙에 할당하고 포인터 반환
    void* create_tracker(int frame_rate, int track_buffer,
                         float track_thresh, float high_thresh, float match_thresh,
                         float std_weight_position, float std_weight_velocity, float std_weight_depth);

    // 트래커 업데이트: detection 배열을 받아 트랙 결과 반환
    // 반환값: 활성 트랙 수
    int update_tracker(void* tracker,
                       CObject* dets, int n_dets,
                       CTrackResult* results, int max_results);

    // Lost 트랙 조회 (KF로 외삽된 bbox/depth 포함).
    // 주인이 매칭 실패해서 update_tracker 결과에 안 잡혔을 때
    // tracking_node가 KF 예측값으로 메우는 용도.
    // 반환값: 채워진 lost 트랙 수
    int get_lost_tracks(void* tracker,
                       CTrackResult* results, int max_results);

    // 트래커 소멸: 힙 메모리 해제
    void  destroy_tracker(void* tracker);
}
