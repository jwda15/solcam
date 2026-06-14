# ros2_gesture_node

손동작 supervisor. OAK RGB 영상에서 **MediaPipe Hands(손 21관절)** 로 제스처를
인식해 계층 메뉴를 운영하고, 결과를 제어 노드 토픽으로 발행한다. LCD UI 포함.

## 인식 (0615 개편: 방향 손동작)

손 관절로 포즈+방향을 읽어 **양손 인식, 손바닥/손등 무관**(손가락 방향만 본다).

- 따봉(like) 1.5초 → 메뉴 열림. 거꾸로 따봉(dislike) = 뒤로/닫기. 10초 무입력 = 닫기.
- **권총(검지)** 의 가리키는 방향(↑↓←→) 으로 Wheel/Lift 를 직관적으로 조작.
- **쓰리건(엄지+검지+중지)** 좌우로 자전.
- 손가락 개수(1~4)는 목록 선택(Mode/Other/메인)에만 쓴다.
  ※ 단일 검지는 인식기에선 "방향(point_*)"으로만 나오고, 손가락-개수 맥락에선
    노드가 'one'(=검지 1개)으로 문맥 변환한다.

## 메뉴 구조

```
따봉 1.5s ─▶ 메인 (몸체 감속 정지)
  ① Mode    1 Idle · 2 Follow · 3 Rotate · 4 More(1 Follow2 · 2 Orbit)   ← 손가락 개수
  ② Wheel   검지 ←/→ 공전(좌=CCW·우=CW)   검지 ↑/↓ 거리(상=멀리·하=가까이)
            쓰리건 ←/→ 자전(좌=CW·우=CCW)  V(두 손가락) = 촬영방향 주인 리셋
  ③ Lift    검지 ↑/↓ = 올림/내림
  ④ Other   1 Phone(Zoom±) · 2 OAK view · 3 Rec · 4 More(1 Help · 2 Power OFF · 3 SolCam Quit)
  거꾸로 따봉 = 한 단계 뒤로(메인에서는 닫기)
```

공전(Wheel ←/→)은 `SEG_ANGLE`(φ), 자전(쓰리건)은 `HEADING_OFFSET`, 거리는
`SEG_DISTANCE`, 리셋은 `HEADING_OFFSET` 절대 0. 방향 부호는 "주인 시점에서
자연스러움" 기준(좌 공전=CCW, 좌 자전=CW)이며, 실차에서 반대면
`gesture_params.yaml` 의 `orbit_sign`/`spin_sign` 만 −1.0 으로 뒤집으면 된다.

선택 = 제스처 1.5초 유지. 방향/조절 항목(공전·자전·거리·리프트·줌)은 발동 후
손을 유지하면 짧은 주기로 연속(jog) 실행된다. 카테고리 진입 직후에는 손을 뗄
때까지 같은 제스처를 무시해 "들고 있다가 두 단계 연속 선택"을 막는다.

LCD는 작은 화면이라 상단바(모드/배터리/REC)는 숨김. Power OFF=로봇 전원,
SolCam Quit=`solcam stop`, Help=도움말(역따봉으로 닫기)은 Other>More 안에 있다.

## 토픽

- 구독: `/oak/rgb/image_raw` (oak_detector `publish_rgb:=true`),
  `/gesture_mock` (mock 모드 주입용)
- 발행: `/gesture_active`(Bool), `/control_mode`(Int32),
  `/adjust_cmd`(AdjustCmd), `/phone_cmd`·`/system_cmd`(String),
  `/gesture_ui`(String JSON — LCD UI/디버그)

## 파일

| 파일 | 역할 |
|------|------|
| `menu.py` | 메뉴 트리(`build_menu`)+상태기계. 방향 키(p_up/…, gun_*, two)로 라우팅. ROS 무관 |
| `recognizer.py` | **MediaPipeHandsRecognizer**(포즈+방향) / Mock / (구)HaGRID |
| `gesture_node.py` | ROS 입출력 + 문맥변환(point_* → p_* 또는 one) |
| `hud.py` | UI 렌더(ROS 무관) — 방향 메뉴는 글리프 카드, 개수 메뉴는 숫자 카드 |
| `ui_node.py` | LCD UI(pygame) |
| `config/gesture_params.yaml` | 유지시간·조절폭·방향부호 등 |
| `test/test_menu.py` | 상태기계 단위테스트 18건 (ROS 불필요) |

## 준비물

```bash
pip install mediapipe        # 손 21관절. ★Python 3.8~3.12 (3.14 휠 없음). 잿슨 OK.
# mock/미리보기는 mediapipe 불필요.
```

## UI 미리보기 (PC, ROS·MediaPipe 불필요)

MediaPipe 가 안 깔리는 환경(예: Python 3.14)에서도 키보드로 동작 확인:

```bash
python tools/ui_preview_tk.py        # 설치 전혀 불필요(tkinter 기본 포함)
```

조작(키보드로 손동작 흉내):
```
L = 따봉(메뉴 열기)      K = 거꾸로 따봉(뒤로/닫기)
1~4 = 손가락 개수(Mode/Other 선택)
↑↓←→ = 권총 방향(Wheel/Lift)   Z/X = 쓰리건 자전 좌/우   2 = V(휠 리셋)
R = 녹화 토글   B = 배터리--   ESC = 종료
```

실제 상태기계(menu.py)를 그대로 써서 동작·타이밍이 LCD와 동일하다.
폰 카메라를 배경에 까는 방법은 `--camera` 옵션(아래) 참고.

```bash
python tools/ui_preview_tk.py --list-cameras       # 장치 이름
python tools/ui_preview_tk.py --camera "장치이름"   # 배경에 폰 영상
```

## 실행

```bash
# 모델·카메라 없이 전체 체인 테스트 (mock — canonical 라벨 주입)
ros2 launch ros2_gesture_node gesture.launch.py recognizer:=mock ui:=false
ros2 topic pub /gesture_mock std_msgs/String "data: like" -r 5      # 따봉
ros2 topic pub /gesture_mock std_msgs/String "data: point_left" -r 5 # 방향
ros2 topic echo /gesture_ui

# 실기 (MediaPipe)
ros2 launch ros2_gesture_node gesture.launch.py

# 상태기계 단위테스트
python3 -m pytest test/test_menu.py -v
```

## TODO

- 실기 카메라에서 포즈 임계값(특히 쓰리건·엄지 폄) 튜닝. 불안정하면 자전을
  Wheel 안 숫자 서브카테고리로 빼는 폴백.
- 주인 bbox ROI 크롭(행인 손 무시) — owner bbox 토픽 합의 후.
- FOLLOW2/ORBIT 에서 "촬영방향 리셋"(현재 FOLLOW용 HEADING_OFFSET=0)을
  rel0 재캡처(=주인 정면)로 확장하려면 제어 노드 지원 추가.
- /phone_cmd·/system_cmd 수신측 구현.
