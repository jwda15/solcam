"""메뉴 상태기계 단위테스트 — ROS 없이 실행:
    cd ros2_gesture_node && python3 -m pytest test/test_menu.py -v
시간을 직접 주입해 유지/끊김/타임아웃 시나리오를 결정적으로 검증한다.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from ros2_gesture_node.menu import MenuStateMachine, build_menu

STEPS = {"dist_step": 0.3, "heading_step_deg": 15.0, "lift_step": 0.1}


def make_sm():
    return MenuStateMachine(build_menu(STEPS), trigger_hold=1.5,
                            select_hold=1.5, menu_timeout=10.0, dropout_tol=0.4)


def feed(sm, gesture, t0, dur, dt=0.1):
    """gesture를 t0부터 dur초 동안 dt 간격으로 공급, 모든 이벤트 수집."""
    evs, t = [], t0
    while t <= t0 + dur + 1e-9:
        evs += sm.update(gesture, t)
        t += dt
    return evs, t


def kinds(evs):
    return [e.kind for e in evs]


def test_trigger_opens_menu():
    sm = make_sm()
    evs, _ = feed(sm, "like", 0.0, 1.4)          # 1.4초: 아직
    assert sm.state == "IDLE" and not evs
    evs, _ = feed(sm, "like", 1.5, 0.2)          # 누적 1.6초 → 열림
    assert "open" in kinds(evs) and sm.state == "MENU"


def test_short_like_ignored():
    sm = make_sm()
    feed(sm, "like", 0.0, 1.0)                   # 1.0초만 유지
    feed(sm, None, 1.1, 1.0)                     # 끊김 (dropout 초과 → 리셋)
    evs, _ = feed(sm, "like", 3.0, 1.0)          # 다시 1.0초 — 합산되면 안 됨
    assert sm.state == "IDLE" and not evs


def test_dropout_tolerated():
    sm = make_sm()
    feed(sm, "like", 0.0, 0.8)
    feed(sm, None, 0.9, 0.2)                     # 0.3초 끊김 (< 0.4 허용)
    evs, _ = feed(sm, "like", 1.2, 0.8)          # 이어서 유지 → 총 2.0초
    assert sm.state == "MENU"


def open_menu(sm):
    feed(sm, "like", 0.0, 1.7)
    assert sm.state == "MENU"
    feed(sm, None, 1.8, 0.1)                     # 따봉 릴리즈
    return 2.0


def test_navigate_and_mode_action():
    sm = make_sm()
    t = open_menu(sm)
    evs, t = feed(sm, "one", t, 1.6)             # Mode 카테고리
    assert "navigate" in kinds(evs)
    feed(sm, None, t, 0.1); t += 0.2             # 릴리즈
    evs, t = feed(sm, "two", t, 1.6)             # 회전 모드 선택
    acts = [e for e in evs if e.kind == "action"]
    assert acts and acts[0].action.payload == {"mode": 2}
    assert "close" in kinds(evs) and sm.state == "IDLE"   # stay=False → 닫힘


def test_stay_item_repeats_while_held():
    """모터 조작(stay): 최초 1.5초 발동 후 손을 뗄 때까지 연속(jog)으로
    같은 명령을 계속 낸다."""
    sm = make_sm()
    t = open_menu(sm)
    _, t = feed(sm, "two", t, 1.6)               # Wheel(차체 이동) 진입
    feed(sm, None, t, 0.1); t += 0.2
    evs, t = feed(sm, "one", t, 3.2)             # 'Farther' 3.2초 유지
    acts = [e for e in evs if e.kind == "action"]
    assert len(acts) >= 5                        # 1.5초 후 연속 반복 → 다수
    assert all(a.action.payload["param"] == "SEG_DISTANCE" for a in acts)
    assert sm.snapshot()["hold_progress"] == 1.0  # 연속 중 게이지 꽉 참
    assert sm.state == "MENU"                    # stay=True → 메뉴 유지


def test_stay_repeat_stops_on_release():
    """손을 떼면 연속이 멈추고, 다시 들면 1.5초 재무장이 필요하다."""
    sm = make_sm()
    t = open_menu(sm)
    _, t = feed(sm, "two", t, 1.6)               # Wheel
    feed(sm, None, t, 0.1); t += 0.2
    _, t = feed(sm, "one", t, 1.8)               # 무장 + 연속 시작
    evs, t = feed(sm, None, t, 0.6)              # 손 뗌 → 연속 정지
    assert sm.snapshot()["hold_progress"] == 0.0
    evs, t = feed(sm, "one", t, 0.5)             # 0.5초만 → 재무장 안 됨
    assert not [e for e in evs if e.kind == "action"]


def test_dislike_back_then_close():
    sm = make_sm()
    t = open_menu(sm)
    _, t = feed(sm, "three", t, 1.6)             # 리프트 진입
    feed(sm, None, t, 0.1); t += 0.2
    evs, t = feed(sm, "dislike", t, 1.6)            # 거꾸로따봉 뒤로 → 메인
    assert "navigate" in kinds(evs) and sm.state == "MENU"
    feed(sm, None, t, 0.1); t += 0.2
    evs, t = feed(sm, "dislike", t, 1.6)            # 메인에서 거꾸로따봉 → 닫기
    close = [e for e in evs if e.kind == "close"]
    assert close and close[0].reason == "back" and sm.state == "IDLE"


def test_timeout_closes():
    sm = make_sm()
    t = open_menu(sm)
    evs, _ = feed(sm, None, t, 10.5)             # 10초 넘게 무입력
    close = [e for e in evs if e.kind == "close"]
    assert close and close[0].reason == "timeout" and sm.state == "IDLE"


def test_no_double_select_without_release():
    """카테고리 진입 후 같은 제스처를 계속 들고 있어도 하위 항목이
    자동 선택되면 안 된다 (릴리즈 요구)."""
    sm = make_sm()
    t = open_menu(sm)
    evs, t = feed(sm, "one", t, 4.0)             # 4초 내내 'one' 유지
    acts = [e for e in evs if e.kind == "action"]
    assert not acts                              # 팔로우(하위 1번) 자동선택 금지
    assert sm.path[-1].label == "Mode"


def test_invalid_gesture_in_menu_ignored():
    sm = make_sm()
    t = open_menu(sm)
    _, t = feed(sm, "three", t, 1.6)             # 리프트 (항목 1,2뿐)
    feed(sm, None, t, 0.1); t += 0.2
    evs, _ = feed(sm, "four", t, 2.0)            # 없는 항목 → 무시
    assert not [e for e in evs if e.kind == "action"]
    assert sm.state == "MENU"


def test_phone_and_system_leaves():
    sm = make_sm()
    t = open_menu(sm)
    _, t = feed(sm, "four", t, 1.6)              # Other (촬영·시스템)
    feed(sm, None, t, 0.1); t += 0.2
    _, t = feed(sm, "one", t, 1.6)               # 폰 카메라
    feed(sm, None, t, 0.1); t += 0.2
    evs, _ = feed(sm, "one", t, 1.6)             # 줌 +
    acts = [e for e in evs if e.kind == "action"]
    assert acts and acts[0].action.kind == "phone"
    assert acts[0].action.payload["cmd"] == "zoom_in"


def test_record_toggle_leaf():
    """메인 4번(Other)의 Rec = 폰 촬영 시작/종료 토글."""
    sm = make_sm()
    t = open_menu(sm)
    _, t = feed(sm, "four", t, 1.6)              # Other 진입
    feed(sm, None, t, 0.1); t += 0.2
    evs, _ = feed(sm, "four", t, 1.6)            # Rec
    acts = [e for e in evs if e.kind == "action"]
    assert acts and acts[0].action.kind == "phone"
    assert acts[0].action.payload["cmd"] == "record_toggle"
