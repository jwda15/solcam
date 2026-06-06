# ros2_gesture_node

손동작 supervisor. OAK RGB 영상에서 HaGRID 제스처를 인식해 계층 메뉴를
운영하고, 결과를 제어 노드 토픽으로 발행한다. LCD(7인치 HDMI) UI 포함.

## 메뉴 구조

따봉(like) 1.5초 유지 → 메뉴 열림(몸체 감속 정지) → 손가락 개수로 선택.
palm은 모드번호 five와 헷갈려 메뉴 번호는 one~four만 쓴다.

```
like 1.5s ─▶ 메인 메뉴
  one    주행 모드   1 팔로우 · 2 회전 · 3 정지
  two    거리·구도   1 멀리 · 2 가까이 · 3 헤딩좌 · 4 헤딩우
  three  리프트      1 올리기 · 2 내리기
  four   촬영·시스템 1 폰 카메라(줌/포커스, 자리만) · 2 OAK 화면 · 3 전원(자리만)
  dislike(거꾸로 따봉)  한 단계 뒤로 (메인에서는 닫기) / 10초 무입력 = 자동 취소
```

선택 = 제스처 1.5초 유지. 조절 항목(거리·헤딩·리프트·줌)은 실행 후 메뉴에
머물러 손을 유지하면 1.5초마다 반복 실행된다. 카테고리 진입 직후에는
손을 뗄 때까지 같은 제스처를 무시해 "들고 있다가 두 단계 연속 선택"을 막는다.

## 토픽

- 구독: `/oak/rgb/image_raw` (oak_detector `publish_rgb:=true`),
  `/gesture_mock` (mock 모드 주입용)
- 발행: `/gesture_active`(Bool), `/control_mode`(Int32),
  `/adjust_cmd`(AdjustCmd), `/phone_cmd`·`/system_cmd`(String, 자리만),
  `/gesture_ui`(String JSON — LCD UI/디버그)

## 파일

| 파일 | 역할 |
|------|------|
| `menu.py` | 메뉴 트리 정의(`build_menu`) + 상태기계. ROS 무관 — 항목 추가는 여기만 |
| `recognizer.py` | HaGRID YOLO 인식기(ultralytics) + Mock. HaGRID 클래스 별칭 통합 |
| `gesture_node.py` | ROS 입출력. 인식 → 상태기계 → 사건을 토픽으로 번역 |
| `hud.py` | UI 렌더(ROS 무관) — ui_node와 프리뷰 공용 |
| `ui_node.py` | LCD UI(pygame). 영상 배경 + 하단 메뉴 독(보라 fill + 흰 플래시), 모드·REC·배터리 |
| `config/gesture_params.yaml` | 유지시간·조절폭 등 파라미터 전부 |
| `test/test_menu.py` | 상태기계 단위테스트 10건 (ROS 불필요) |

## 준비물

```bash
pip install "ultralytics>=8.2" pygame   # mock 모드는 둘 다 불필요
./models/download.sh                     # YOLOv10n_gestures.pt (22MB)
# 모델 라이선스: CC BY-SA 4.0 변형 (hagrid 저장소 license/ 참조) → 깃에 커밋 금지
```

## UI 미리보기 (Windows/PC, ROS 불필요)

LCD 화면을 ROS 없이 키보드로 확인:

```bash
pip install pygame numpy
cd ros2_gesture_node
python tools/ui_preview.py
```
L=따봉(메뉴 열기) · 1~4=선택(꾹) · K=거꾸로 따봉(뒤로/닫기) · R=REC · B=배터리 · ESC.

pygame 설치가 안 되면(예: Python 3.14처럼 최신이라 pygame wheel이 아직 없으면)
설치가 전혀 필요 없는 tkinter 버전을 쓴다 — 키/동작 동일:

```bash
python tools/ui_preview_tk.py
```
실제 상태기계(menu.py)를 그대로 써서 동작·타이밍이 LCD와 동일하다
(배경 영상만 없으면 CAMERA 표시).

### 폰 카메라를 배경에 깔기 (Windows)

폰을 윈도우에서 웹캠으로 잡으면(안드 14+ USB 웹캠 모드, 또는 DroidCam/Iriun)
프리뷰 배경에 실제 폰 영상이 깔린다. cv2/pygame 불필요 — ffmpeg만 쓴다:

```bash
pip install imageio-ffmpeg                       # ffmpeg 바이너리 자동(또는 시스템 ffmpeg)
python tools/ui_preview_tk.py --list-cameras     # 장치 이름 확인
python tools/ui_preview_tk.py --camera "장치이름"  # 그 카메라를 배경으로
```

## 실행

```bash
# 모델·카메라 없이 전체 체인 테스트 (mock)
ros2 launch ros2_gesture_node gesture.launch.py recognizer:=mock ui:=false
ros2 topic pub /gesture_mock std_msgs/String "data: like" -r 5   # 따봉 주입
ros2 topic echo /gesture_ui                                       # 상태 확인

# 실기
ros2 launch ros2_gesture_node gesture.launch.py

# 상태기계 단위테스트
python3 -m pytest test/test_menu.py -v
```

## TODO

- 주인 bbox ROI 크롭 (행인 손 무시) — owner bbox 토픽 합의 후
- 2~3m 거리 실측 테스트 (512×384 프리뷰로 부족하면 OAK 고해상 스트림 추가)
- LCD UI 디자인 다듬기 + 촬영 카메라(폰) 미러링
- /phone_cmd, /system_cmd 수신측 구현
