# 주인 위치 메시지
# tracking_node가 매 프레임 publish
std_msgs/Header header

bool is_detected        # 주인 탐지 여부 (false = Lost 상태)

# 카메라 좌표계 기준 3D 위치 (단위: m)
# OAK-D SpatialDetectionNetwork 출력값 기준
float32 spatial_x       # 좌우 (+ = 오른쪽)
float32 spatial_y       # 상하 (+ = 아래)
float32 spatial_z       # 전후 depth (+ = 앞)

# 극좌표 (로봇 제어에 직접 사용)
float32 azimuth         # 수평 방위각 (rad), atan2(x, z)  0=정면, +=오른쪽
float32 distance        # 수평 거리 (m), sqrt(x^2 + z^2)

float32 confidence      # detection score (0.0 ~ 1.0)
int32 track_id          # ByteTrack이 부여한 트랙 ID
