#include "ByteTrack/Rect.h"

#include <algorithm>

// Rect: 2D 바운딩 박스를 나타내는 템플릿 클래스
// 내부 데이터는 tlwh (top-left x, top-left y, width, height) 형식으로 저장
// KalmanFilter와 BYTETracker가 bbox 좌표 변환에 사용

// 생성자: top-left 좌표(x, y)와 크기(width, height)로 초기화
template <typename T>
byte_track::Rect<T>::Rect(const T &x, const T &y, const T &width, const T &height) :
    tlwh({x, y, width, height})
{
}

template <typename T>
byte_track::Rect<T>::~Rect()
{
}

// --- 좌표 접근자 (읽기 전용) ---
// tlwh[0]=x, tlwh[1]=y, tlwh[2]=width, tlwh[3]=height

template <typename T>
const T& byte_track::Rect<T>::x() const
{
    return tlwh[0];  // top-left x
}

template <typename T>
const T& byte_track::Rect<T>::y() const
{
    return tlwh[1];  // top-left y
}

template <typename T>
const T& byte_track::Rect<T>::width() const
{
    return tlwh[2];
}

template <typename T>
const T& byte_track::Rect<T>::height() const
{
    return tlwh[3];
}

// --- 좌표 접근자 (쓰기 가능) ---
// STrack::updateRect()에서 Kalman mean으로 bbox를 갱신할 때 사용

template <typename T>
T& byte_track::Rect<T>::x()
{
    return tlwh[0];
}

template <typename T>
T& byte_track::Rect<T>::y()
{
    return tlwh[1];
}

template <typename T>
T& byte_track::Rect<T>::width()
{
    return tlwh[2];
}

template <typename T>
T& byte_track::Rect<T>::height()
{
    return tlwh[3];
}

// --- 코너 좌표 ---

template <typename T>
const T& byte_track::Rect<T>::tl_x() const
{
    return tlwh[0];  // top-left x (x()와 동일)
}

template <typename T>
const T& byte_track::Rect<T>::tl_y() const
{
    return tlwh[1];  // top-left y (y()와 동일)
}

template <typename T>
T byte_track::Rect<T>::br_x() const
{
    return tlwh[0] + tlwh[2];  // bottom-right x = x + width
}

template <typename T>
T byte_track::Rect<T>::br_y() const
{
    return tlwh[1] + tlwh[3];  // bottom-right y = y + height
}

// tlwh → tlbr 변환: (x, y, w, h) → (x1, y1, x2, y2)
// IoU 계산 등에서 사용
template <typename T>
byte_track::Tlbr<T> byte_track::Rect<T>::getTlbr() const
{
    return {
        tlwh[0],              // x1 = top-left x
        tlwh[1],              // y1 = top-left y
        tlwh[0] + tlwh[2],   // x2 = x + width
        tlwh[1] + tlwh[3],   // y2 = y + height
    };
}

// tlwh → xyah 변환: (x, y, w, h) → (cx, cy, a, h)
// Kalman filter measurement 조립 시 사용
// cx = 중심 x,  cy = 중심 y,  a = 가로세로비(w/h),  h = 높이
template <typename T>
byte_track::Xyah<T> byte_track::Rect<T>::getXyah() const
{
    return {
        tlwh[0] + tlwh[2] / 2,  // cx = x + w/2
        tlwh[1] + tlwh[3] / 2,  // cy = y + h/2
        tlwh[2] / tlwh[3],      // a  = width / height
        tlwh[3],                 // h  = height
    };
}

// IoU (Intersection over Union) 계산
// removeDuplicateStracks()에서 중복 트랙 제거에 사용
// 반환값: 0.0 (겹침 없음) ~ 1.0 (완전히 겹침)
template<typename T>
float byte_track::Rect<T>::calcIoU(const Rect<T>& other) const
{
    const float box_area = (other.tlwh[2] + 1) * (other.tlwh[3] + 1);  // other의 bbox 면적

    // 교집합 너비: 두 박스의 x축 겹치는 구간
    const float iw = std::min(tlwh[0] + tlwh[2], other.tlwh[0] + other.tlwh[2])
                   - std::max(tlwh[0], other.tlwh[0]) + 1;
    float iou = 0;
    if (iw > 0)
    {
        // 교집합 높이: 두 박스의 y축 겹치는 구간
        const float ih = std::min(tlwh[1] + tlwh[3], other.tlwh[1] + other.tlwh[3])
                       - std::max(tlwh[1], other.tlwh[1]) + 1;
        if (ih > 0)
        {
            // ua = 합집합 면적 = 면적A + 면적B - 교집합
            const float ua = (tlwh[0] + tlwh[2] - tlwh[0] + 1) * (tlwh[1] + tlwh[3] - tlwh[1] + 1)
                           + box_area - iw * ih;
            iou = iw * ih / ua;  // IoU = 교집합 / 합집합
        }
    }
    return iou;
}

// tlbr → Rect 변환: (x1, y1, x2, y2) → Rect(x, y, w, h)
template<typename T>
byte_track::Rect<T> byte_track::generate_rect_by_tlbr(const byte_track::Tlbr<T>& tlbr)
{
    // width = x2 - x1,  height = y2 - y1
    return byte_track::Rect<T>(tlbr[0], tlbr[1], tlbr[2] - tlbr[0], tlbr[3] - tlbr[1]);
}

// xyah → Rect 변환: (cx, cy, a, h) → Rect(x, y, w, h)
template<typename T>
byte_track::Rect<T> byte_track::generate_rect_by_xyah(const byte_track::Xyah<T>& xyah)
{
    const auto width = xyah[2] * xyah[3];  // width = a * h
    // x = cx - w/2,  y = cy - h/2
    return byte_track::Rect<T>(xyah[0] - width / 2, xyah[1] - xyah[3] / 2, width, xyah[3]);
}

// 명시적 인스턴스화: 템플릿 클래스는 헤더만으로 링크가 안 되므로
// 사용할 타입(int, float)을 여기서 명시적으로 컴파일
template class byte_track::Rect<int>;
template class byte_track::Rect<float>;

template byte_track::Rect<int> byte_track::generate_rect_by_tlbr<int>(const byte_track::Tlbr<int>&);
template byte_track::Rect<float> byte_track::generate_rect_by_tlbr<float>(const byte_track::Tlbr<float>&);

template byte_track::Rect<int> byte_track::generate_rect_by_xyah<int>(const byte_track::Xyah<int>&);
template byte_track::Rect<float> byte_track::generate_rect_by_xyah<float>(const byte_track::Xyah<float>&);
