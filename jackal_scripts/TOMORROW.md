# 내일 자칼 데이터 수집 — 명령어 모음

자칼 환경 (이전과 동일):
- ssh jackal@192.168.0.30 (비번 자칼 그대로)
- Ubuntu 18.04 + ROS Melodic
- catkin_ws 위치: `~/catkin_ws`
- 매 셸 첫 두 줄:
  ```bash
  source /opt/ros/melodic/setup.bash
  source ~/catkin_ws/devel/setup.bash
  ```

목표:
- (A) 저텍스처 시퀀스 — 잔디깎이/사각형 자율주행, 바닥 비스듬히
- 캡스톤(B)는 야외에선 안 함. 연구실 와서 따로.

---

## 0. 사전 체크

### 0-1. SSH 붙기
```bash
ssh jackal@192.168.0.30
# 비번 입력
# 둘 이상 창 필요. tmux 쓰자:
tmux new -s jackal
# 분할: Ctrl+b "  (가로) / Ctrl+b %  (세로)
# 이동: Ctrl+b 화살표
# detach: Ctrl+b d
# 재접속: tmux attach -t jackal
```

### 0-2. ROS 환경 한 번 점검
```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
echo $ROS_MASTER_URI         # http://localhost:11311 또는 자칼 IP
roscore &                    # 보통 자동 떠있음. ps로 확인
ps -ef | grep rosmaster
```

### 0-3. D435i 인식
```bash
lsusb | grep -i intel
ls /dev/video*
```

### 0-4. Dynamixel USB 인식
```bash
ls /dev/ttyUSB*    # workbench가 잡을 포트
# 권한 부족 시:
sudo chmod 666 /dev/ttyUSB0
```

### 0-5. 자칼 SSD 용량
```bash
df -h ~        # bag 한 개에 GB 단위. 5~10GB 여유 확인.
```

---

## 1. 카메라 + Dynamixel 띄우기

### T1) D435i RealSense
```bash
source /opt/ros/melodic/setup.bash
roslaunch realsense2_camera rs_camera.launch \
  align_depth:=true \
  enable_color:=true \
  enable_depth:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=linear_interpolation \
  color_width:=640 color_height:=480 color_fps:=30 \
  depth_width:=640 depth_height:=480 depth_fps:=30
```
확인 (다른 창):
```bash
source /opt/ros/melodic/setup.bash
rostopic hz /camera/color/image_raw                   # ~30Hz
rostopic hz /camera/aligned_depth_to_color/image_raw  # ~30Hz
rostopic hz /imu/data                                 # 자칼 본체 IMU
```

### T2) Dynamixel workbench (틸트용)
```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch dynamixel_workbench_controllers dynamixel_controllers.launch
```

### T3) 틸트 각도 세팅 (검증된 tick 값)
다른 창에서:
```bash
source /opt/ros/melodic/setup.bash

# 0도 (정면)
rosservice call /dynamixel_workbench/dynamixel_command \
  "command: '' id: 1 addr_name: 'Goal_Position' value: 2048"

# 45도 (저텍스처 메인)
rosservice call /dynamixel_workbench/dynamixel_command \
  "command: '' id: 1 addr_name: 'Goal_Position' value: 1536"

# 90도 (더 비스듬, 거의 수직 아래)
rosservice call /dynamixel_workbench/dynamixel_command \
  "command: '' id: 1 addr_name: 'Goal_Position' value: 1024"

# 다른 검증된 각도 (폰 메모 기준):
#   30도 → 1741
#   50도 → 1587
#   60도 → 1331
```

`rqt`로 GUI 통해서도 가능:
```bash
rqt
# Plugins → Services → Service Caller
# /dynamixel_workbench/dynamixel_command 선택
# Expression: 'Goal_Position', value 입력 → Call
```

---

## 2. 저텍스처 시퀀스 수집

### 2-1. 자율주행 스크립트 자칼로 옮기기 (처음 한 번)
노트북 WSL에서:
```bash
scp /mnt/d/solcam/jackal_scripts/lawnmower.py \
    /mnt/d/solcam/jackal_scripts/square.py \
    jackal@192.168.0.30:~/
```

### 2-2. 틸트 시작 각도 (예: 45도)
```bash
rosservice call /dynamixel_workbench/dynamixel_command \
  "command: '' id: 1 addr_name: 'Goal_Position' value: 1536"
```

### 2-3. rosbag 녹화 시작
새 tmux 창:
```bash
source /opt/ros/melodic/setup.bash
mkdir -p ~/data/$(date +%Y%m%d) && cd ~/data/$(date +%Y%m%d)

# 검증된 토픽 셋 (이전 메모 그대로)
rosbag record -O lawnmower45_$(date +%H%M%S).bag \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /camera/aligned_depth_to_color/image_raw \
  /camera/aligned_depth_to_color/camera_info \
  /imu/data \
  /odometry/filtered \
  /tf /tf_static
```
파일명에 틸트 각도 넣어두면 나중에 정리 편함 (`lawnmower45_`, `square90_` 등).

### 2-4. 자율주행 실행
새 tmux 창:
```bash
source /opt/ros/melodic/setup.bash

# 잔디깎이 (파라미터는 lawnmower.py 맨 위 5~7줄만 수정)
python ~/lawnmower.py

# 또는 사각형
python ~/square.py
```

비상 정지:
```bash
# 어느 창에서든:
rostopic pub -1 /jackaljandi/stop std_msgs/Bool "data: true"
# 또는 스크립트 Ctrl+C
# 또는 자칼 본체 E-stop
```

### 2-5. 종료
1. 자율주행 스크립트 자연 종료 또는 Ctrl+C
2. rosbag Ctrl+C
3. bag 검증:
```bash
rosbag info ~/data/$(date +%Y%m%d)/lawnmower45_*.bag
# duration, topics, msg 수 확인
```

### 2-6. 다른 시나리오로 반복
```bash
# 틸트 90도로 바꿈
rosservice call /dynamixel_workbench/dynamixel_command \
  "command: '' id: 1 addr_name: 'Goal_Position' value: 1024"

# 새 rosbag 시작 (파일명에 90 표기)
rosbag record -O lawnmower90_$(date +%H%M%S).bag \
  /camera/color/image_raw /camera/color/camera_info \
  /camera/aligned_depth_to_color/image_raw \
  /camera/aligned_depth_to_color/camera_info \
  /imu/data /odometry/filtered /tf /tf_static

# 다른 창에서 자율주행 다시
python ~/square.py
```

---

## 3. bag 회수

### 3-1. 종료 시 틸트 0도 복귀 (안전)
```bash
rosservice call /dynamixel_workbench/dynamixel_command \
  "command: '' id: 1 addr_name: 'Goal_Position' value: 2048"
```

### 3-2. 노트북으로 가져오기
노트북 WSL에서:
```bash
mkdir -p /mnt/d/jackal_data
rsync -avh --progress \
    jackal@192.168.0.30:~/data/ \
    /mnt/d/jackal_data/
```
D: 드라이브에서 바로 보임.

### 3-3. 노트북에서 bag 검증 (선택)
ROS 노트북에 안 깔았으면 자칼에서 검증해도 OK. 노트북에 깔았으면:
```bash
rosbag info /mnt/d/jackal_data/<날짜>/lawnmower45_*.bag
```

---

## 4. 트러블슈팅

### 카메라 hz 안 나옴
- USB3 포트 맞는지 (USB2면 fps 떨어짐)
- `align_depth:=true` 빠진 거 아닌지
- realsense 드라이버 버전 (자칼에 이미 깔려있는 거 그대로 쓰는 게 안전)

### Dynamixel 응답 없음
- workbench launch 안 떠있음 → T2 다시
- USB 권한 → `sudo chmod 666 /dev/ttyUSB0`
- 같은 포트 다른 노드 점유 → `lsof /dev/ttyUSB0`

### 자칼 안 움직임
- E-stop 풀렸는지 (빨간 버튼)
- deadman 누르고 있어야 하는 모드인지
- `/cmd_vel` echo로 메시지 가는지: `rostopic echo /cmd_vel`
- 다른 노드가 cmd_vel 가로채는지: `rostopic info /cmd_vel`

### rosbag 토픽 못 찾음
- `rostopic list`로 정확한 이름 확인
- realsense launch에 namespace가 붙는 경우가 있음

---

## 5. 캡스톤 팔로우 (B) — 연구실 들어와서

야외에선 안 함. 연구실 와이파이/이더넷에서 따로 진행.
LAPTOP_SETUP.txt Part 3 참조.

핵심:
1. 노트북 WSL2 mirrored mode 활성화
2. ros1_bridge 띄움 (Melodic ↔ Humble)
3. ros2_follow_node + tracking_node + yolo_detector 실행
4. follow_mode 0 → 2 → 1 순서로 안전 검증
