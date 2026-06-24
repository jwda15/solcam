"""메뉴 상태기계 단위테스트 (방향 손동작 개편판) — ROS 없이:
    cd ros2_gesture_node && python3 -m pytest test/test_menu.py -v
"""
import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from ros2_gesture_node.menu import MenuStateMachine, build_menu

STEPS = {"dist_step": 0.3, "seg_angle_step": math.radians(8.0),
         "heading_step": math.radians(15.0), "lift_step": 0.1,
         "jog_lin": 0.12, "jog_ang": 0.6}


def make_sm():
    return MenuStateMachine(build_menu(STEPS), trigger_hold=1.5,
                            select_hold=1.5, menu_timeout=10.0, dropout_tol=0.4)


def feed(sm, gesture, t0, dur, dt=0.1):
    evs, t = [], t0
    while t <= t0 + dur + 1e-9:
        evs += sm.update(gesture, t)
        t += dt
    return evs, t


def kinds(evs):
    return [e.kind for e in evs]


def open_menu(sm):
    feed(sm, "like", 0.0, 1.7)
    assert sm.state == "MENU"
    feed(sm, None, 1.8, 0.1)
    return 2.0


def nav(sm, key, t, dur=1.6):
    """카테고리 진입(또는 항목 선택). 진입 후 손 떼기까지 처리하고 다음 t 반환."""
    feed(sm, key, t, dur)
    t += dur + 0.1
    feed(sm, None, t, 0.1); t += 0.2
    return t


# ---------- 메뉴 열기/유지 ----------
def test_trigger_opens_menu():
    sm = make_sm()
    evs, _ = feed(sm, "like", 0.0, 1.4)
    assert sm.state == "IDLE" and not evs
    evs, _ = feed(sm, "like", 1.5, 0.2)
    assert "open" in kinds(evs) and sm.state == "MENU"


def test_short_like_ignored():
    sm = make_sm()
    feed(sm, "like", 0.0, 1.0)
    feed(sm, None, 1.1, 1.0)
    evs, _ = feed(sm, "like", 3.0, 1.0)
    assert sm.state == "IDLE" and not evs


def test_dropout_tolerated():
    sm = make_sm()
    feed(sm, "like", 0.0, 0.8)
    feed(sm, None, 0.9, 0.2)
    evs, _ = feed(sm, "like", 1.2, 0.8)
    assert sm.state == "MENU"


# ---------- 모드(손가락 개수) ----------
def test_navigate_and_mode_action():
    sm = make_sm()
    t = open_menu(sm)
    t = nav(sm, "one", t)                         # Mode 카테고리
    assert sm.path[-1].label == "Mode"
    evs, t = feed(sm, "three", t, 1.6)            # Follow1(모드1) — 순서변경
    acts = [e for e in evs if e.kind == "action"]
    assert acts and acts[0].action.payload == {"mode": 1}
    assert "close" in kinds(evs) and sm.state == "IDLE"


def test_more_submodes():
    sm = make_sm()
    t = open_menu(sm)
    t = nav(sm, "one", t)                         # Mode
    t = nav(sm, "four", t)                        # More
    evs, t = feed(sm, "one", t, 1.6)              # Rotate(모드2) — More 안 1번
    acts = [e for e in evs if e.kind == "action"]
    assert acts and acts[0].action.payload == {"mode": 2}
    # Orbit(모드4)은 More 안 2번
    sm2 = make_sm(); t2 = open_menu(sm2)
    t2 = nav(sm2, "one", t2); t2 = nav(sm2, "four", t2)
    evs2, _ = feed(sm2, "two", t2, 1.6)
    a2 = [e for e in evs2 if e.kind == "action"]
    assert a2 and a2[0].action.payload == {"mode": 4}


# ---------- 휠: 로봇기준 jog (BODY_VX/VY/WZ, odom 무관) ----------
def test_wheel_jog_translate():
    sm = make_sm(); t = open_menu(sm); t = nav(sm, "two", t)   # Wheel
    a = [e for e in feed(sm, "p_up", t, 1.6)[0] if e.kind == "action"][0].action
    assert a.payload["param"] == "BODY_VX" and a.payload["value"] > 0
    assert a.payload["delta"] is False        # 절대 속도

    sm = make_sm(); t = open_menu(sm); t = nav(sm, "two", t)
    a = [e for e in feed(sm, "p_down", t, 1.6)[0] if e.kind == "action"][0].action
    assert a.payload["param"] == "BODY_VX" and a.payload["value"] < 0

    sm = make_sm(); t = open_menu(sm); t = nav(sm, "two", t)
    a = [e for e in feed(sm, "p_left", t, 1.6)[0] if e.kind == "action"][0].action
    assert a.payload["param"] == "BODY_VY" and a.payload["value"] > 0   # 좌 = +vy


def test_wheel_jog_spin():
    sm = make_sm(); t = open_menu(sm); t = nav(sm, "two", t)
    a = [e for e in feed(sm, "gun_left", t, 1.6)[0] if e.kind == "action"][0].action
    assert a.payload["param"] == "BODY_WZ" and a.payload["value"] < 0   # 좌 = 시계(CW,-)
    sm = make_sm(); t = open_menu(sm); t = nav(sm, "two", t)
    a = [e for e in feed(sm, "gun_right", t, 1.6)[0] if e.kind == "action"][0].action
    assert a.payload["param"] == "BODY_WZ" and a.payload["value"] > 0   # 우 = 반시계(+)


def test_wheel_jog_repeats_and_stops():
    sm = make_sm(); t = open_menu(sm); t = nav(sm, "two", t)
    evs, t = feed(sm, "p_up", t, 3.2)             # 전진 3.2초 유지 → 연속 jog
    acts = [e for e in evs if e.kind == "action"]
    assert len(acts) >= 5 and all(a.action.payload["param"] == "BODY_VX" for a in acts)
    assert sm.snapshot()["hold_progress"] == 1.0
    evs, t = feed(sm, None, t, 0.6)               # 손 뗌 → 정지
    assert sm.snapshot()["hold_progress"] == 0.0


# ---------- 휠 > More: 이전 odom 기반 명령 (V 자리) ----------
def test_wheel_more_old_commands():
    sm = make_sm(); t = open_menu(sm); t = nav(sm, "two", t)   # Wheel
    t = nav(sm, "two", t)                                      # More (V 자리)
    assert sm.path[-1].label == "More"
    a = [e for e in feed(sm, "p_left", t, 1.6)[0] if e.kind == "action"][0].action
    assert a.payload["param"] == "SEG_ANGLE" and a.payload["value"] > 0   # 공전
    sm = make_sm(); t = open_menu(sm); t = nav(sm, "two", t); t = nav(sm, "two", t)
    a = [e for e in feed(sm, "p_up", t, 1.6)[0] if e.kind == "action"][0].action
    assert a.payload["param"] == "SEG_DISTANCE" and a.payload["value"] > 0  # 거리


def test_wheel_more_face_owner_reset():
    sm = make_sm(); t = open_menu(sm); t = nav(sm, "two", t); t = nav(sm, "two", t)
    a = [e for e in feed(sm, "two", t, 1.6)[0] if e.kind == "action"][0].action
    assert a.payload == {"param": "HEADING_OFFSET", "value": 0.0, "delta": False}


# ---------- 리프트: 방향(권총) ----------
def test_lift_directions():
    sm = make_sm()
    t = open_menu(sm)
    t = nav(sm, "three", t)                       # Lift
    assert sm.path[-1].label == "Lift"
    evs, t = feed(sm, "p_up", t, 1.6)
    a = [e for e in evs if e.kind == "action"][0].action
    assert a.payload["param"] == "LIFT_HEIGHT" and a.payload["value"] > 0
    feed(sm, None, t, 0.5); t += 0.6
    evs, t = feed(sm, "p_down", t, 1.6)
    a = [e for e in evs if e.kind == "action"][0].action
    assert a.payload["param"] == "LIFT_HEIGHT" and a.payload["value"] < 0


# ---------- 뒤로/타임아웃/오인식 ----------
def test_dislike_back_then_close():
    sm = make_sm()
    t = open_menu(sm)
    t = nav(sm, "three", t)                       # Lift 진입
    evs, t = feed(sm, "dislike", t, 1.6)          # 뒤로 → 메인
    assert "navigate" in kinds(evs) and sm.state == "MENU"
    feed(sm, None, t, 0.1); t += 0.2
    evs, t = feed(sm, "dislike", t, 1.6)          # 메인에서 → 닫기
    close = [e for e in evs if e.kind == "close"]
    assert close and close[0].reason == "back" and sm.state == "IDLE"


def test_timeout_closes():
    sm = make_sm()
    t = open_menu(sm)
    evs, _ = feed(sm, None, t, 10.5)
    close = [e for e in evs if e.kind == "close"]
    assert close and close[0].reason == "timeout" and sm.state == "IDLE"


def test_no_double_select_without_release():
    sm = make_sm()
    t = open_menu(sm)
    evs, t = feed(sm, "one", t, 4.0)              # Mode 진입 후 계속 유지
    acts = [e for e in evs if e.kind == "action"]
    assert not acts                               # 하위 자동선택 금지
    assert sm.path[-1].label == "Mode"


def test_invalid_gesture_in_menu_ignored():
    sm = make_sm()
    t = open_menu(sm)
    t = nav(sm, "three", t)                       # Lift (p_up/p_down만)
    evs, _ = feed(sm, "four", t, 2.0)             # 없는 키 → 무시
    assert not [e for e in evs if e.kind == "action"]
    assert sm.state == "MENU"


def test_point_ignored_at_main():
    """메인 메뉴에서는 방향 권총이 자식 키가 아니라 무시된다."""
    sm = make_sm()
    t = open_menu(sm)
    evs, _ = feed(sm, "p_up", t, 2.0)
    assert not [e for e in evs if e.kind == "action"]
    assert sm.path[-1].label == "Main"


# ---------- 아더(손가락 개수) ----------
def test_phone_zoom_leaf():
    sm = make_sm()
    t = open_menu(sm)
    t = nav(sm, "four", t)                        # Other
    t = nav(sm, "one", t)                         # Phone
    evs, _ = feed(sm, "one", t, 1.6)             # Zoom +
    a = [e for e in evs if e.kind == "action"][0].action
    assert a.kind == "phone" and a.payload["cmd"] == "zoom_in"


def test_record_toggle_leaf():
    sm = make_sm()
    t = open_menu(sm)
    t = nav(sm, "four", t)                        # Other
    evs, _ = feed(sm, "three", t, 1.6)           # Rec (이제 3번)
    a = [e for e in evs if e.kind == "action"][0].action
    assert a.kind == "phone" and a.payload["cmd"] == "record_toggle"


def _goto_more(sm):
    t = open_menu(sm)
    t = nav(sm, "four", t)                        # Other
    t = nav(sm, "four", t)                        # More
    return t


def test_more_help_leaf():
    sm = make_sm(); t = _goto_more(sm)
    a = [e for e in feed(sm, "one", t, 1.6)[0] if e.kind == "action"][0].action
    assert a.kind == "ui" and a.payload == {"toggle": "help"}


def test_quit_dialog_yes():
    sm = make_sm(); t = _goto_more(sm)
    t = nav(sm, "three", t)                    # SolCam Quit → 다이얼로그
    assert sm.snapshot()["dialog"] == "Quit SolCam?"
    a = [e for e in feed(sm, "two", t, 1.6)[0] if e.kind == "action"][0].action  # 2 Yes
    assert a.kind == "system" and a.payload == {"cmd": "quit"}


def test_quit_dialog_no_cancels():
    sm = make_sm(); t = _goto_more(sm)
    t = nav(sm, "three", t)
    evs, _ = feed(sm, "one", t, 1.6)           # 1 No
    acts = [e for e in evs if e.kind == "action"]
    assert acts and acts[0].action.kind == "cancel"
    assert sm.state == "IDLE"                   # No=단발 → 메뉴 닫힘


def test_poweroff_dialog_two_steps():
    sm = make_sm(); t = _goto_more(sm)
    t = nav(sm, "two", t)                       # Power OFF → 다이얼로그1
    assert sm.snapshot()["dialog"] == "Power off the robot?"
    t = nav(sm, "two", t)                       # 2 Yes → 다이얼로그2
    assert sm.snapshot()["dialog"] == "SolCam will shut down. Proceed?"
    a = [e for e in feed(sm, "one", t, 1.6)[0] if e.kind == "action"][0].action  # 1 Yes
    assert a.kind == "system" and a.payload == {"cmd": "shutdown"}


def test_poweroff_dialog1_no_cancels():
    sm = make_sm(); t = _goto_more(sm)
    t = nav(sm, "two", t)                       # Power OFF 다이얼로그1
    evs, _ = feed(sm, "one", t, 1.6)           # 1 No
    assert [e for e in evs if e.kind == "action"][0].action.kind == "cancel"
