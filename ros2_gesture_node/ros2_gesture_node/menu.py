"""메뉴 상태기계 — ROS 무관 순수 파이썬. (단위테스트: test/test_menu.py)

동작 규약 (설계 합의 0605):
  IDLE: like(따봉)만 감시. trigger_hold(1.5s) 유지 → 메뉴 열림.
  MENU: one~four = 항목 선택(select_hold 유지 = 확정),
        dislike(거꾸로 따봉) = 한 단계 뒤로 (루트에서는 메뉴 닫기),
        menu_timeout(10s) 무입력 = 자동 취소.
  ※ palm(보자기)은 모드 번호 five와 헷갈려 메뉴 번호로 쓰지 않는다 (one~four만).

확정/오인식 방지:
  - 같은 제스처가 hold 시간 동안 유지되어야 발동. 짧은 인식 끊김은
    dropout_tol(0.4s)까지 허용 (프레임 드랍/깜빡임 흡수).
  - 발동 후:
      stay=True 항목(모터 연속 조작: 거리/리프트/줌 등)은 최초 select_hold
        발동 뒤 손을 뗄 때까지 repeat_interval(0.12s) 주기로 같은 명령을
        "연속(jog)"으로 계속 낸다 → 모터가 점진적으로 계속 움직인다.
      그 외(카테고리 진입, 모드 변경 등)는 손을 뗄 때까지 같은
        제스처를 무시 → "길게 들고 있다가 두 단계 연속 선택" 사고 방지.

메뉴 트리는 build_menu()의 데이터가 전부 — 항목 추가는 여기만 고치면 된다.
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional

# 사용하는 제스처 어휘 (HaGRID 클래스의 부분집합; 별칭 통합은 recognizer 쪽)
TRIGGER = "like"
BACK = "dislike"   # 거꾸로 따봉(reverse thumbs-up) = 뒤로/닫기
SELECT_KEYS = ("one", "two", "three", "four")


@dataclass
class Action:
    """메뉴 리프가 실행하는 동작. kind별 payload를 노드가 토픽으로 변환한다.
    kind: 'mode'(주행모드) | 'adjust'(AdjustCmd) | 'phone'(폰 카메라, 자리)
          | 'system'(전원 등, 자리) | 'ui'(LCD 표시 토글)
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
    """메뉴 트리 정의. p = 조절 폭 파라미터 (yaml에서 옴).
    ★새 항목/카테고리 추가는 여기에 한 줄 — 코드 다른 곳 수정 불필요.

    1단: Mode(주행모드) / Wheel(차체 이동) / Lift(리프트) / Other(카메라·시스템)."""
    deg = 3.141592653589793 / 180.0
    return MenuNode("Main", children={
        "one": MenuNode("Mode", children={
            "one":   MenuNode("Idle",   action=Action("mode", "Idle",   {"mode": 0})),
            "two":   MenuNode("Follow", action=Action("mode", "Follow", {"mode": 1})),
            "three": MenuNode("Rotate", action=Action("mode", "Rotate", {"mode": 2})),
            "four":  MenuNode("More", children={      # 추가 팔로우 스타일(4칸 한계로 묶음)
                "one": MenuNode("Follow2", action=Action("mode", "Follow2", {"mode": 3})),
                "two": MenuNode("Orbit",   action=Action("mode", "Orbit",   {"mode": 4})),
            }),
        }),
        # 차체(휠) 이동 — 구도 3종을 카테고리로 분리(각자 방향 2개).
        #  Distance: 주인과의 거리 D            (SEG_DISTANCE)
        #  Bearing : 주인 주위 공전(원운동)     (SEG_ANGLE) ← 거리 유지하며 방위각만 변경.
        #            모드1(FOLLOW)에서 목표점 = 주인 − D·(cosφ,sinφ) 라 φ를 바꾸면
        #            주인 주위를 돈다. 몸체는 계속 주인을 향하고(ownerBearing+offset),
        #            상단 yaw(OAK)는 주인 락온 유지 → 공전 중에도 카메라가 주인을 본다.
        #            Pan 오프셋이 걸려 있으면 그 상대 구도도 유지된 채 공전.
        #  Pan     : 촬영 카메라 헤딩 오프셋     (HEADING_OFFSET) ← 주인 아닌 방향 보기.
        "two": MenuNode("Wheel", children={
            "one": MenuNode("Distance", children={
                "one": MenuNode("Farther", action=Action("adjust", "Dist +%.1fm" % p["dist_step"],
                                                         {"param": "SEG_DISTANCE", "value": +p["dist_step"], "delta": True}, stay=True)),
                "two": MenuNode("Closer",  action=Action("adjust", "Dist -%.1fm" % p["dist_step"],
                                                         {"param": "SEG_DISTANCE", "value": -p["dist_step"], "delta": True}, stay=True)),
            }),
            # 공전: SEG_ANGLE +φ = CCW(반시계). (★실차에서 회전 방향 부호 확인할 것)
            "two": MenuNode("Bearing", children={
                "one": MenuNode("CCW", action=Action("adjust", "Arc CCW %.0f" % p["bearing_step_deg"],
                                                     {"param": "SEG_ANGLE", "value": +p["bearing_step_deg"] * deg, "delta": True}, stay=True)),
                "two": MenuNode("CW",  action=Action("adjust", "Arc CW %.0f" % p["bearing_step_deg"],
                                                     {"param": "SEG_ANGLE", "value": -p["bearing_step_deg"] * deg, "delta": True}, stay=True)),
            }),
            # 헤딩 오프셋: CCW+ = 좌. (★실차에서 좌우 부호 확인할 것)
            "three": MenuNode("Pan", children={
                "one": MenuNode("Pan L", action=Action("adjust", "Pan L %.0f" % p["heading_step_deg"],
                                                       {"param": "HEADING_OFFSET", "value": +p["heading_step_deg"] * deg, "delta": True}, stay=True)),
                "two": MenuNode("Pan R", action=Action("adjust", "Pan R %.0f" % p["heading_step_deg"],
                                                       {"param": "HEADING_OFFSET", "value": -p["heading_step_deg"] * deg, "delta": True}, stay=True)),
            }),
        }),
        "three": MenuNode("Lift", children={
            "one": MenuNode("Up", action=Action("adjust", "Lift +%.2fm" % p["lift_step"],
                                                    {"param": "LIFT_HEIGHT", "value": +p["lift_step"], "delta": True}, stay=True)),
            "two": MenuNode("Down", action=Action("adjust", "Lift -%.2fm" % p["lift_step"],
                                                    {"param": "LIFT_HEIGHT", "value": -p["lift_step"], "delta": True}, stay=True)),
        }),
        "four": MenuNode("Other", children={
            "one": MenuNode("Phone", children={      # 폰 카메라: 줌(scrcpy --camera-zoom)
                "one":   MenuNode("Zoom +", action=Action("phone", "Zoom +", {"cmd": "zoom_in"},  stay=True)),
                "two":   MenuNode("Zoom -", action=Action("phone", "Zoom -", {"cmd": "zoom_out"}, stay=True)),
            }),
            "two":   MenuNode("OAK view",  action=Action("ui", "OAK view", {"toggle": "oak_view"})),  # 단발 ON/OFF 토글(REC식)
            "three": MenuNode("Power off", action=Action("system", "Power off", {"cmd": "shutdown"})),  # 자리만 (/system_cmd)
            "four":  MenuNode("Rec", action=Action("phone", "Rec", {"cmd": "record_toggle"})),  # 폰 촬영 시작/종료 토글 (/phone_cmd)
        }),
    })


class MenuStateMachine:
    """제스처 라벨 스트림 → 메뉴 사건. 시간은 호출자가 주입(테스트 용이)."""

    def __init__(self, root: MenuNode, trigger_hold=1.5, select_hold=1.5,
                 menu_timeout=10.0, dropout_tol=0.4, repeat_interval=0.12):
        self.root = root
        self.trigger_hold = trigger_hold
        self.select_hold = select_hold
        self.menu_timeout = menu_timeout
        self.dropout_tol = dropout_tol
        # stay 항목(모터 조작) 연속 모드: 최초 select_hold 발동 후, 손을 뗄
        #  때까지 이 짧은 주기로 같은 명령을 계속 낸다(거리/리프트 점진 이동).
        self.repeat_interval = repeat_interval
        self.state = "IDLE"
        self.path: List[MenuNode] = []        # MENU일 때 [root, ...]
        self.last_action_name = ""
        self._cand: Optional[str] = None      # 유지 판정 중인 제스처
        self._hold_start = 0.0
        self._last_seen = 0.0
        self._await_release: Optional[str] = None  # 발동 후 손 뗄 때까지 무시
        self._last_activity = 0.0
        self._repeating = False               # stay 항목 연속 발동 중?

    # ------------------------------------------------------------------
    def update(self, gesture: Optional[str], t: float) -> List[Event]:
        """매 인식 프레임 호출. gesture=None은 '아무것도 인식 안 됨'."""
        ev: List[Event] = []

        # 발동 직후 릴리즈 대기: 같은 제스처가 계속 보이면 무시
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
            valid = gesture if (gesture == BACK or
                                (gesture in SELECT_KEYS and cur.children and gesture in cur.children)) else None
            if valid is not None:           # 메뉴와 무관한 손동작은 타임아웃 리셋 안 함
                self._touch_activity(t)
            if t - self._last_activity > self.menu_timeout:
                self._close("timeout", ev)
                return ev
            # 연속(jog) 중이면 짧은 주기로, 아니면 select_hold(1.5s)로 발동 판정
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
            for g in SELECT_KEYS:
                if g in cur.children:
                    items.append({"gesture": g, "label": cur.children[g].label})
        progress = 0.0
        need = self.trigger_hold if self.state == "IDLE" else self.select_hold
        if self._repeating:
            progress = 1.0   # 연속(jog) 중엔 게이지를 꽉 채워 활성 표시
        elif self._cand is not None and need > 0:
            progress = min(1.0, (self._last_seen - self._hold_start) / need)
        return {
            "state": self.state,
            "path": [n.label for n in self.path],
            "items": items,
            "hold_gesture": self._cand or "",
            "hold_progress": round(progress, 3),
            "repeating": self._repeating,   # 연속(jog) 적용 중?
            "last_action": self.last_action_name,
        }

    # ----- 내부 ---------------------------------------------------------
    def _update_hold(self, gesture, t, need, on_fire):
        """유지 판정: 같은 제스처 need초 유지(끊김 dropout_tol 허용) → on_fire."""
        if gesture is None:
            if self._cand is not None and t - self._last_seen > self.dropout_tol:
                self._cand = None
                self._repeating = False   # 손을 떼면 연속 모드 종료
            return
        if gesture != self._cand:
            self._cand = gesture
            self._hold_start = t
            self._repeating = False       # 다른 제스처면 연속 모드 초기화
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
        self._repeating = False           # 기본은 단발. stay 리프만 아래서 켠다
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
                self._hold_start = t          # 다음 발동 기준점 갱신
                self._repeating = True        # 이후 repeat_interval 주기로 연속
            else:
                self._close("done", ev)
                self._await_release = gesture
        else:
            self.path.append(child)
            ev.append(Event("navigate"))
            self._await_release = gesture     # 떼기 전 연속 선택 방지
            self._cand = None


