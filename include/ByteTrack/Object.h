#pragma once

#include "ByteTrack/Rect.h"

namespace byte_track
{
// Object: YOLO 등 detector가 한 프레임에서 검출한 하나의 객체
// BYTETracker::update()에 vector<Object>로 넘겨주면 트래킹 시작
struct Object
{
    Rect<float> rect;   // 2D bounding box (x, y, width, height) - 픽셀 단위

    // [3D 확장] OAK-D SpatialDetectionNetwork가 계산한 depth (mm 단위)
    // bbox 중심 주변 ROI의 depth 평균값으로, Kalman z 축에 직접 들어감
    float depth;

    int label;          // 클래스 ID (person = 0)
    float prob;         // detection confidence score (0.0 ~ 1.0)

    // 생성자: OAK-D에서 받은 값을 그대로 넣어주면 됨
    // 예: Object(rect, spatialCoords.z, 0, confidence)
    Object(const Rect<float> &_rect,
           const float &_depth,
           const int &_label,
           const float &_prob);
};
}