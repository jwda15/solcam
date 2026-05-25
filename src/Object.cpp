#include "ByteTrack/Object.h"

// OAK-D에서 받은 detection 결과를 Object 구조체로 생성
// 사용 예: Object(rect, spatialCoords.z, 0, confidence)
byte_track::Object::Object(const Rect<float> &_rect,
                           const float &_depth,  // OAK-D spatial z값 (mm)
                           const int &_label,     // 클래스 ID (person=0)
                           const float &_prob)    // detection confidence
    : rect(_rect), depth(_depth), label(_label), prob(_prob)
{
}