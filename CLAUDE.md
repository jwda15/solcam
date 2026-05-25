# ByteTrack 3D 프로젝트 컨텍스트

## 프로젝트 목적
팔로잉 로봇용 주인 추적 시스템.
OAK-D S2 카메라로 사람을 탐지하고 ByteTrack으로 주인을 지속 추적.
Jetson Orin Nano Super + Ubuntu 22.04 + ROS2 Humble 배포 예정.

## 하드웨어
- 카메라: OAK-D S2 (RVC2, MyriadX VPU)
  - YOLOv8n을 .blob 변환해서 온보드 추론
  - SpatialDetectionNetwork → bbox + XYZ (mm) 출력
- 추론: Jetson Orin Nano Super
  - ByteTrack C++ 노드 실행
- 개발환경: Windows WSL2 Ubuntu 24.04 (빌드/테스트용)

## 프로젝트 구조
```
ByteTrack-cpp/
├── include/ByteTrack/
│   ├── KalmanFilter.h      ← 10D 상태로 확장됨 (3D Kalman)
│   ├── BYTETracker.h       ← calcIouDistanceOnly() 추가됨
│   ├── STrack.h            ← depth_ 필드, Kalman getter 추가
│   ├── Object.h            ← depth 필드 추가
│   ├── Rect.h
│   └── tracker_c_api.h     ← Python/ROS2 연동용 C 인터페이스
├── src/
│   ├── KalmanFilter.cpp    ← 10D 확장 + Mahalanobis gatingDistance
│   ├── BYTETracker.cpp     ← 1차:Mahalanobis, 2차/4차:IoU
│   ├── STrack.cpp
│   ├── Object.cpp
│   ├── Rect.cpp
│   └── tracker_c_api.cpp   ← create/update/destroy_tracker C API
├── build/
│   └── libbytetrack.so     ← 빌드 결과물
├── ros2_tracking_node/     ← ROS2 패키지
│   ├── msg/OwnerPose.msg
│   ├── src/tracking_node.cpp
│   ├── CMakeLists.txt
│   └── package.xml
└── test_tracking.py        ← TUM fr3/walking_xyz 테스트 스크립트
```

## 핵심 수정 내용 (원본 대비)

### 1. Kalman Filter 3D 확장
- **기존**: 8D state (x, y, a, h, vx, vy, va, vh)
- **변경**: 10D state (x, y, z, a, h, vx, vy, vz, va, vh)
- z = OAK-D depth (mm)
- DetectBox: 4D → 5D (Xyazh)

### 2. 매칭 방식 변경
- **1차 매칭**: IoU → Mahalanobis distance (3D)
  - chi2 임계값 11.070 (5-DOF, 95%)
  - 함수명 calcIouDistance() 유지 (내부 로직만 교체)
- **2차/4차 매칭**: Mahalanobis → IoU (calcIouDistanceOnly)
  - 저신뢰도 detection은 depth 부정확 → IoU가 더 안정적
- **중복 제거**: calcIouDistanceOnly() 사용

### 3. Object 구조체
```cpp
struct Object {
    Rect<float> rect;
    float depth;   // OAK-D spatialCoords.z (mm)
    int label;
    float prob;
};
```

### 4. C API (tracker_c_api.h/cpp)
Python ctypes 및 ROS2에서 호출용:
```cpp
void* create_tracker(int frame_rate, int track_buffer,
                     float track_thresh, float high_thresh, float match_thresh);
int   update_tracker(void* tracker, CObject* dets, int n_dets,
                     CTrackResult* results, int max_results);
void  destroy_tracker(void* tracker);
```

## ROS2 노드 현황

### tracking_node.cpp
- 주인 초기화: 화면 중앙에 가장 가까운 트랙을 target_id로 고정
- Lost 처리: max_lost_frames(기본 30) 초과 시 재초기화
- publish: /owner_pose (OwnerPose.msg)
- **현재 상태**: 더미 타이머로 동작 (OAK-D 미연결)
- **TODO**: depthai_ros_msgs/SpatialDetectionArray subscriber로 교체

### OwnerPose.msg
```
std_msgs/Header header
bool is_detected
float32 x          # 픽셀 좌표
float32 y
float32 z          # depth (m)
float32 confidence
int32 track_id
```

## 빌드 방법

### ByteTrack 라이브러리
```bash
cd ByteTrack-cpp/build
cmake ..
make -j$(nproc)
# 결과: build/libbytetrack.so
```

### ROS2 패키지 (Jazzy 기준, Ubuntu 24.04)
```bash
# conda 비활성화 필수!
conda deactivate

source /opt/ros/jazzy/setup.bash
cd ~/tracking_ws
ln -s /mnt/d/ByteTrack-cpp/ros2_tracking_node src/ros2_tracking_node  # 최초 1회
colcon build --packages-select ros2_tracking_node
```

### 빌드 트러블슈팅
- `ModuleNotFoundError: No module named 'em'` → conda 환경 비활성화 후 재빌드
- conda PATH가 섞이면: `export PATH=/usr/bin:/usr/local/bin:$PATH`

## 테스트
TUM fr3/walking_xyz 데이터셋으로 테스트 완료:
- 데이터셋: /mnt/d/mot_test/rgbd_dataset_freiburg3_walking_xyz
- 결과 영상: /mnt/d/mot_test/output_tracking.avi
- 859프레임, ID 부여 및 Lost/Reactivate 동작 확인

```bash
conda activate bytetrack
cd /mnt/d/ByteTrack-cpp
python test_tracking.py
```

## TODO (우선순위 순)
1. ROS2 빌드 em 모듈 문제 해결 (conda PATH 충돌)
2. depthai_ros_msgs 연동 (OAK-D 도착 후)
3. YOLOv8n → .blob 변환 (OAK-D 도착 후)
4. 팀원 얼굴인식 모듈과 주인 초기화 연동 (선택)
5. Jetson 배포 테스트
