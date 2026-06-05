# ProximityArray.msg
# 몸체 측면 근접센서(초음파 3 + ToF 3 = 총 6개)의 거리 측정값 묶음.
# 센서 노드(팀원)가 발행하고, 제어 노드의 ObstacleField가 구독한다.
#
# distances 배열의 각 원소는 해당 방향으로 측정된 장애물까지의 거리(m).
# 측정 불가/범위초과는 +inf 또는 max_range 로 채운다(센서 노드 약속).
#
# directions 배열은 각 센서가 바라보는 방향(몸체 기준 방위각, rad, CCW+).
#   예) 왼쪽=+pi/2, 오른쪽=-pi/2, 후방=pi 등.
#   distances[i] 와 directions[i] 가 짝을 이룬다(같은 인덱스 = 같은 센서).
#
# 초음파/ToF를 굳이 구분하지 않는 이유:
#   제어(회피) 입장에서 필요한 건 "그 방향이 얼마나 가까운가"뿐이다.
#   센서 종류별 신뢰도 차이는 센서 노드가 흡수해서 거리값으로만 넘긴다.
std_msgs/Header header

float32[] distances     # m, 각 센서의 측정 거리 (범위초과는 max_range/inf)
float32[] directions    # rad, 각 센서의 방향 (몸체 기준, CCW+). distances와 같은 길이
