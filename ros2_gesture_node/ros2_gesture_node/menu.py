"""메뉴 상태기계 — ROS 무관 순수 파이썬. (단위테스트: test/test_menu.py)

동작 규약 (0615 개편: 방향 손동작):
  IDLE: like(따봉)만 감시. trigger_hold(1.5s) 유지 → 메뉴 열림.
  MENU: 현재 노드의 자식 키로 들어온 제스처만 유효(select_hold 유지 = 확정),
        dislike(거꾸로 따봉) = 한 단계 뒤로 (루트에서는 메뉴 닫기),
        menu_timeout(10s) 무입력 = 자동 취소.

제스처 어휘 (recognizer 가 우리 어휘로 정규화해 넘김):
  like / dislike                         : 메뉴 열기 / 뒤로
  one~four                               : 손가락 개수 — 리스트 선택(Mode/Other, 메인)
  p_up / p_down / p_left / p_right        : 권총(검지) 방향 — Lift/Wheel 방향 조작
  gun_left / gun_right                    : 쓰리건(엄지+검지+중지) 좌우 — 자전
  ("two" = V 두 손가락은 Wheel 안에서 '촬영방향 리셋'으로도 쓰임)

방향 매핑(주인 시점 자연스러움 기준; 부호는 gesture_node 에서 yaml 로 뒤집기 가능):
  Wheel  ←/→(검지) = 공전: 좌=CCW(+φ) 우=CW(−φ)        [SEG_ANGLE]
         ↑/↓(검지) = 거리: 상=멀리(+) 하=가까이(−)      [SEG_DISTANCE]
         ←/→(쓰리건)= 자전: 좌=CW(−off) 우=CCW(+off)     [HEADING_OFFSET]
         V(two)    = 촬영방향 주인쪽 리셋(HEADING_OFFSET 절대 0)
  Lift   ↑/↓(검지) = 올림/내림                            [LIFT_HEIGHT]

확정/오인식 방지:
  - 같은 제스처가 hold 시간 유지돼야 발동. 짧은 끊김은 dropout_tol(0.4s)까지 허용.
  - stay=True 항목(연속 조작: 공전/자전/거리/리프트/줌)은 최초 발동 뒤 손 뗄
    때까지 repeat_interval 주기로 같은 명령을 "연속(jog)"으로 낸다.
  - 그 외(카테고리 진입/모드 변경/리셋)는 손 뗄 때까지 같은 제스처 무시.

메뉴 트리는 build_menu()의 데이터가 전부 — 항목 추가는 여기만 고치면 된다.
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional

# 제스처 어휘
TRIGGER = "like"
BACK = "dislike"                       # 거꾸로 따봉 = 뒤로/닫기
COUNT_KEYS = ("one", "two", "three", "four")   # 손가락 개수(리스트 선택)
POINT_KEYS = ("p_up", "p_down", "p_left", "p_right")   # 권총(검지) 방향
GUN_KEYS = ("gun_left", "gun_right")   # 쓰리건 좌우(자전)
SELECT_KEYS = COUNT_KEYS               # (구코드/테스트 호환 별칭)


@dataclass
class Action:
    """메뉴 리프가 실행하는 동작. kind별 payload를 노드가 토픽으로 변환한다.
    kind: 'mode' | 'adjust'(AdjustCmd) | 'phone' | 'system' | 'ui'
    stay: True면 실행 후 같은 메뉴에 머무르며 손을 뗄 때까지 연속 발동."""
    kind: str
    name: str
    payload: dict = field(default_factory=dict)
    stay: bool = False


@dataclass
class MenuNode:
    label: str
    children: Optional[Dict[str, "MenuNode"]] = None   # 제스처 → 자식
    action: Optional[Action] = None                    # 리프면 동작
    dialog: Optional[str] = None                        # 설정 시 중앙 확인 다이얼로그(프롬프트)

    @property
    def is_leaf(self) -> bool:
        return self.action is not None


@dataclass
class Event:
    """update()가 돌려주는 사건. 노드가 이걸 보고 토픽을 발행한다."""
    kind: str                      # 'open' | 'close' | 'navigate' | 'action'
    reason: str = ""               # close: 'done' | 'back' | 'timeout'
    action: Optional[Action] = None


def build_menu(p: dict) -> MenuNode:
    """메뉴 트리 정의. p = 부호까지 반영된 조절 스텝(gesture_node 에서 옴):
        dist_step(m,+멀리) · seg_angle_step(rad,+CCW) ·
        heading_step(rad,+CCW=좌) · lift_step(m,+올림)
    ★새 항목/카테고리 추가는 여기에 한 줄 — 코드 다른 곳 수정 불필요.

    1단(손가락 개수): ① Mode  ② Wheel  ③ Lift  ④ Other.
    Mode/Other 는 목록 선택이라 손가락 개수, Wheel/Lift 는 방향(권총/쓰리건)."""
    da = p["seg_angle_step"]      # 공전 1스텝 (rad, +CCW)
    dh = p["heading_step"]        # 자전/헤딩 1스텝 (rad, +CCW=좌)
    dd = p["dist_step"]           # 거리 1스텝 (m, +멀리)
    dl = p["lift_step"]           # 리프트 1스텝 (m, +올림)
    jl = p["jog_lin"]            # 휠 jog 전후/좌우 속도 (m/s, 로봇기준)
    ja = p["jog_ang"]            # 휠 jog 자전 각속도 (rad/s, 로봇기준)
    jr = p["radial_jog"]         # 주인기준 거리 jog (m/s, +=접근)
    jo = p["orbit_jog"]          # 주인기준 공전 jog (m/s, +=CCW)

    def adj(name, param, value, delta=True, stay=True):
        return MenuNode(name, action=Action(
            "adjust", name, {"param": param, "value": value, "delta": delta}, stay=stay))

    root = MenuNode("Main", children={
        # ── ① 주행 모드 (손가락 개수) ──────────────────────────────
        "one": MenuNode("Mode", children={
            # ★Follow2 를 앞으로(자주 씀, odom 불필요). 순서: Idle/Follow2/Follow1/More
            "one":   MenuNode("Idle",    action=Action("mode", "Idle",    {"mode": 0})),
            "two":   MenuNode("Follow2", action=Action("mode", "Follow2", {"mode": 3})),
            "three": MenuNode("Follow1", action=Action("mode", "Follow1", {"mode": 1})),
            "four":  MenuNode("More", children={      # 4칸 한계 → 추가 모드 묶음
                "one": MenuNode("Rotate", action=Action("mode", "Rotate", {"mode": 2})),
                "two": MenuNode("Orbit",  action=Action("mode", "Orbit",  {"mode": 4})),
            }),
        }),
        # ── ② 휠(차체 이동) — 로봇기준 순수 jog (odom 無, 모든 모드 공통) ─────
        #   검지 ↑/↓ 전/후(BODY_VX) · 검지 ←/→ 좌/우 측면(BODY_VY)
        #   쓰리건 ←/→ 자전 시계/반시계(BODY_WZ). delta=False(절대 속도, 손 떼면 정지).
        #   메뉴를 나가면 모드가 '바뀐 주인 거리/방향'으로 재시작(control_node 재engage).
        "two": MenuNode("Wheel", children={
            "p_up":    adj("Forward",  "BODY_VX", +jl, delta=False),  # 상 = 전진
            "p_down":  adj("Backward", "BODY_VX", -jl, delta=False),  # 하 = 후진
            "p_left":  adj("Left",     "BODY_VY", +jl, delta=False),  # 좌 = 좌측면(+vy)
            "p_right": adj("Right",    "BODY_VY", -jl, delta=False),  # 우 = 우측면
            "gun_left":  adj("Spin CW",  "BODY_WZ", -ja, delta=False),  # 좌 = 시계 자전
            "gun_right": adj("Spin CCW", "BODY_WZ", +ja, delta=False),  # 우 = 반시계 자전
            # V(two) → More: ★주인기준 모션 jog(공전/거리) — 모든 모드에서 동작.
            #   목표값 조정이 아니라 "도는/다가가는 운동" 자체. 손 떼면 정지, 메뉴 나가면 재engage.
            # ── More: 촬영 구도 프리셋(손가락 개수) + 공전 jog(검지) ──
            #   ★Front/Right/Back/Left = 촬영카메라(몸체)가 OAK(주인) 기준 볼 방향.
            #     고르면 몸체가 그만큼 자전하고, OAK는 반대로 돌아 주인을 다시 화면중앙에
            #     맞춘다(개루프 킥 → OAK 영상으로 자가 보정). 정확한 몸체-OAK 각이 확정됨.
            #   ★Pan(쓰리건)·Farther/Closer(주인기준 전후진) 제거 — Front=정면구도로 대체.
            "two": MenuNode("More", children={
                "one":   MenuNode("Front", action=Action("yaw", "Front", {"deg": 0})),
                "two":   MenuNode("Right", action=Action("yaw", "Right", {"deg": 90})),
                "three": MenuNode("Back",  action=Action("yaw", "Back",  {"deg": 180})),
                "four":  MenuNode("Left",  action=Action("yaw", "Left",  {"deg": 270})),
                "p_left":  adj("Orbit CCW", "ORBIT_JOG", +jo, delta=False),  # 주인 둘레 반시계
                "p_right": adj("Orbit CW",  "ORBIT_JOG", -jo, delta=False),  # 시계
            }),
        }),
        # ── ③ 리프트 — 검지 상/하 ────────────────────────────────
        "three": MenuNode("Lift", children={
            "p_up":   adj("Lift Up",   "LIFT_HEIGHT", +dl),
            "p_down": adj("Lift Down", "LIFT_HEIGHT", -dl),
        }),
        # ── ④ 아더(촬영·시스템, 손가락 개수) ──────────────────────
        "four": MenuNode("Other", children={
            "one": MenuNode("Phone", children={
                "one": MenuNode("Zoom +", action=Action("phone", "Zoom +", {"cmd": "zoom_in"},  stay=True)),
                "two": MenuNode("Zoom -", action=Action("phone", "Zoom -", {"cmd": "zoom_out"}, stay=True)),
            }),
            "two":   MenuNode("Oak", action=Action("ui", "Oak", {"press": "oak_cycle"})),
            "three": MenuNode("Rec", action=Action("phone", "Rec", {"cmd": "record_toggle"})),
            "four":  MenuNode("More", children={      # 시스템: 도움말/전원/종료
                "one":   MenuNode("Help", action=Action("ui", "Help", {"toggle": "help"})),
                # Power OFF: 중앙 다이얼로그 2단계. 선택지 위치 교차(1 No/2 Yes → 1 Yes/2 No)
                "two":   MenuNode("Power OFF",
                                  dialog="Power off the robot?", children={
                    "one": MenuNode("No",  action=Action("cancel", "No", {})),
                    "two": MenuNode("Yes",
                                    dialog="SolCam will shut down. Proceed?", children={
                        "one": MenuNode("Yes", action=Action("system", "Power OFF", {"cmd": "shutdown"})),
                        "two": MenuNode("No",  action=Action("cancel", "No", {})),
                    }),
                }),
                # SolCam Quit: 중앙 다이얼로그 1단계
                "three": MenuNode("SolCam Quit",
                                  dialog="Quit SolCam?", children={
                    "one": MenuNode("No",  action=Action("cancel", "No", {})),
                    "two": MenuNode("Yes", action=Action("system", "SolCam Quit", {"cmd": "quit"})),
                }),
            }),
        }),
    })
    return root


class MenuStateMachine:
    """제스처 라벨 스트림 → 메뉴 사건. 시간은 호출자가 주입(테스트 용이)."""

    def __init__(self, root: MenuNode, trigger_hold=1.5, select_hold=1.5,
                 menu_timeout=10.0, dropout_tol=0.4, repeat_interval=0.12):
        self.root = root
        self.trigger_hold = trigger_hold
        self.select_hold = select_hold
        self.menu_timeout = menu_timeout
        self.dropout_tol = dropout_tol
        self.repeat_interval = repeat_interval
        self.state = "IDLE"
        self.path: List[MenuNode] = []
        self.last_action_name = ""
        self._cand: Optional[str] = None
        self._hold_start = 0.0
        self._last_seen = 0.0
        self._await_release: Optional[str] = None
        self._last_activity = 0.0
        self._repeating = False

    # ------------------------------------------------------------------
    def update(self, gesture: Optional[str], t: float) -> List[Event]:
        """매 인식 프레임 호출. gesture=None은 '아무것도 인식 안 됨'."""
        ev: List[Event] = []

        if self._await_release is not None:
            if gesture == self._await_release:
                self._touch_activity(t)
                gesture = None
            else:
                self._await_release = None

        if self.state == "IDLE":
            self._update_hold(gesture if gesture == TRIGGER else None, t,
                              self.trigger_hold, lambda: self._open(t, ev))
        else:  # MENU
            cur = self.path[-1]
            # 유효 = 뒤로(dislike) 또는 현재 노드의 자식 키로 들어온 제스처.
            valid = gesture if (gesture == BACK or
                                (cur.children and gesture in cur.children)) else None
            if valid is not None:
                self._touch_activity(t)
            if t - self._last_activity > self.menu_timeout:
                self._close("timeout", ev)
                return ev
            need = (self.repeat_interval
                    if (self._repeating and valid is not None and valid == self._cand)
                    else self.select_hold)
            self._update_hold(valid, t, need,
                              lambda: self._fire(valid, t, ev))
        return ev

    # ------------------------------------------------------------------
    def snapshot(self) -> dict:
        """LCD(UI)·디버그용 현재 상태."""
        cur = self.path[-1] if self.path else None
        items = []
        if cur and cur.children:
            for g, node in cur.children.items():
                items.append({"gesture": g, "label": node.label})
        progress = 0.0
        need = self.trigger_hold if self.state == "IDLE" else self.select_hold
        if self._repeating:
            progress = 1.0
        elif self._cand is not None and need > 0:
            progress = min(1.0, (self._last_seen - self._hold_start) / need)
        return {
            "state": self.state,
            "dialog": (cur.dialog if cur else None),
            "path": [n.label for n in self.path],
            "items": items,
            "hold_gesture": self._cand or "",
            "hold_progress": round(progress, 3),
            "repeating": self._repeating,
            "last_action": self.last_action_name,
        }

    # ----- 내부 ---------------------------------------------------------
    def _update_hold(self, gesture, t, need, on_fire):
        if gesture is None:
            if self._cand is not None and t - self._last_seen > self.dropout_tol:
                self._cand = None
                self._repeating = False
            return
        if gesture != self._cand:
            self._cand = gesture
            self._hold_start = t
            self._repeating = False
        self._last_seen = t
        if t - self._hold_start >= need:
            on_fire()

    def _touch_activity(self, t):
        self._last_activity = t

    def _open(self, t, ev):
        self.state = "MENU"
        self.path = [self.root]
        self._cand = None
        self._repeating = False
        self._await_release = TRIGGER
        self._last_activity = t
        ev.append(Event("open"))

    def _close(self, reason, ev):
        self.state = "IDLE"
        self.path = []
        self._cand = None
        self._repeating = False
        ev.append(Event("close", reason=reason))

    def _fire(self, gesture, t, ev):
        cur = self.path[-1]
        self._repeating = False
        if gesture == BACK:
            if len(self.path) > 1:
                self.path.pop()
                ev.append(Event("navigate"))
                self._await_release = BACK
                self._cand = None
            else:
                self._close("back", ev)
                self._await_release = BACK
            return
        child = cur.children[gesture]
        if child.is_leaf:
            self.last_action_name = child.action.name
            ev.append(Event("action", action=child.action))
            if child.action.stay:
                self._hold_start = t
                self._repeating = True
            else:
                self._close("done", ev)
                self._await_release = gesture
        else:
            self.path.append(child)
            ev.append(Event("navigate"))
            self._await_release = gesture
            self._cand = None
