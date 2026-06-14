"""제스처 인식기 — 프레임(BGR ndarray) → (라벨, 신뢰도).

0615 개편: 방향 손동작을 위해 손 21랜드마크(MediaPipe Hands) 기반으로 전환.
랜드마크가 있으면 손가락 방향(상/하/좌/우)·양손·손바닥/손등 무관 인식이 된다.

구현체:
  MediaPipeHandsRecognizer : 손 관절 → 포즈+방향 (실기 기본)
  MockRecognizer           : 외부 주입 라벨 그대로 반환 (카메라 없이 개발용)
  HagridYoloRecognizer     : (구) HaGRID YOLO — 방향 불가, 호환용으로만 잔존

★출력 어휘(canonical) — gesture_node 가 메뉴 맥락에 맞게 변환한다:
  like / dislike                         : 따봉 / 거꾸로 따봉
  point_up / point_down / point_left / point_right : 권총(검지) 방향
        (손가락-개수 맥락에선 노드가 'one'(=검지 1개)으로 변환)
  gun_left / gun_right                   : 쓰리건(엄지+검지+중지) 좌우 = 자전
  two / three / four                     : 손가락 개수 (two=V는 휠에서 리셋)

★MediaPipe 는 파이썬 3.14 휠이 없을 수 있음(3.8~3.12 권장). 잿슨(JetPack 3.10) OK.
  PC 미리보기는 Mock/키보드로(아래 ui_preview_tk) 동작 확인.
"""
import math
from typing import Optional, Tuple


class Recognizer:
    def infer(self, frame_bgr) -> Tuple[Optional[str], float]:
        raise NotImplementedError


# ───────────────────────── MediaPipe Hands ─────────────────────────
class MediaPipeHandsRecognizer(Recognizer):
    """손 21랜드마크로 포즈+방향 판정. 양손 중 가장 신뢰도 높은 손 1개 채택.
    손바닥/손등 무관 — 관절의 상대 위치(폄/굽힘)와 검지 방향벡터만 본다.
    """

    # landmark index
    WRIST = 0
    TIP = {"thumb": 4, "index": 8, "middle": 12, "ring": 16, "pinky": 20}
    PIP = {"thumb": 3, "index": 6, "middle": 10, "ring": 14, "pinky": 18}
    MCP = {"thumb": 2, "index": 5, "middle": 9, "ring": 13, "pinky": 17}

    def __init__(self, max_hands: int = 2, det_conf: float = 0.6,
                 track_conf: float = 0.5, dir_dead_deg: float = 25.0):
        import mediapipe as mp   # 지연 임포트 (mock 모드는 불필요)
        self._hands = mp.solutions.hands.Hands(
            static_image_mode=False, max_num_hands=max_hands,
            min_detection_confidence=det_conf, min_tracking_confidence=track_conf)
        # 좌우상하 경계 부근(±dir_dead_deg)에서 떨림 방지용 마진은 호출부에 위임.
        self.dir_dead = dir_dead_deg

    def infer(self, frame_bgr):
        import numpy as np  # noqa
        # MediaPipe 는 RGB 입력
        rgb = frame_bgr[:, :, ::-1]
        res = self._hands.process(rgb)
        if not res.multi_hand_landmarks:
            return None, 0.0
        # 신뢰도 높은 손 선택
        best_i, best_score = 0, -1.0
        if res.multi_handedness:
            for i, h in enumerate(res.multi_handedness):
                s = h.classification[0].score
                if s > best_score:
                    best_i, best_score = i, s
        lm = res.multi_hand_landmarks[best_i].landmark
        pts = [(p.x, p.y) for p in lm]   # 정규화 [0,1], y 아래로 증가
        label = self._classify(pts)
        return label, (best_score if best_score > 0 else 1.0)

    # ----- 포즈 분류 -----
    def _d(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _ext(self, pts, finger) -> bool:
        """손가락 폄 여부 — 끝이 PIP보다 손목에서 더 멀면 폄(방향 무관)."""
        w = pts[self.WRIST]
        return self._d(pts[self.TIP[finger]], w) > self._d(pts[self.PIP[finger]], w) * 1.05

    def _thumb_ext(self, pts) -> bool:
        """엄지 폄 — 엄지끝이 검지MCP에서 충분히 떨어졌고, IP보다 멀면 폄."""
        ref = pts[self.MCP["index"]]
        far = self._d(pts[self.TIP["thumb"]], ref) > self._d(pts[self.PIP["thumb"]], ref) * 1.1
        return far

    def _dir4(self, vx, vy):
        """벡터(이미지좌표, y아래+) → 'up/down/left/right'. y 뒤집어 화면방향."""
        ang = math.degrees(math.atan2(-vy, vx))   # +x=0°, +up=90°
        if -45 <= ang <= 45:
            return "right"
        if 45 < ang < 135:
            return "up"
        if ang >= 135 or ang <= -135:
            return "left"
        return "down"

    def _classify(self, pts) -> Optional[str]:
        idx = self._ext(pts, "index")
        mid = self._ext(pts, "middle")
        rng = self._ext(pts, "ring")
        pky = self._ext(pts, "pinky")
        thb = self._thumb_ext(pts)

        n_long = sum((idx, mid, rng, pky))   # 엄지 제외 폄 개수

        # 엄지만 → 따봉/거꾸로 따봉 (엄지 방향)
        if thb and n_long == 0:
            tx = pts[self.TIP["thumb"]][0] - pts[self.WRIST][0]
            ty = pts[self.TIP["thumb"]][1] - pts[self.WRIST][1]
            d = self._dir4(tx, ty)
            if d == "up":
                return "like"
            if d == "down":
                return "dislike"
            return None

        # 검지 방향벡터(MCP→TIP)
        ix = pts[self.TIP["index"]][0] - pts[self.MCP["index"]][0]
        iy = pts[self.TIP["index"]][1] - pts[self.MCP["index"]][1]
        idir = self._dir4(ix, iy)

        # 쓰리건: 엄지+검지+중지 폄, 약지·새끼 굽힘 → 자전(좌/우)
        if thb and idx and mid and not rng and not pky:
            if idir in ("left", "right"):
                return "gun_" + idir
            return None   # 세로 쓰리건은 미사용

        # 권총(검지만) → 방향
        if idx and not mid and not rng and not pky:
            return "point_" + idir

        # 손가락 개수 (엄지 굽힘 기준): 2/3/4
        if not thb:
            if idx and mid and not rng and not pky:
                return "two"      # V
            if idx and mid and rng and not pky:
                return "three"
            if idx and mid and rng and pky:
                return "four"
        return None


# ───────────────────────── Mock (개발용) ─────────────────────────
class MockRecognizer(Recognizer):
    """set_gesture()로 주입한 canonical 라벨을 반환(만료시간 지나면 None).
    /gesture_mock(String)로 주입:
      ros2 topic pub /gesture_mock std_msgs/String "data: like" -r 5
      ros2 topic pub /gesture_mock std_msgs/String "data: point_up" -r 5
    """

    def __init__(self, expire_s: float = 0.5):
        self.expire_s = expire_s
        self._label: Optional[str] = None
        self._stamp = 0.0

    def set_gesture(self, label: str, t: float):
        self._label = label if label else None
        self._stamp = t

    def infer_at(self, t: float):
        if self._label is not None and t - self._stamp <= self.expire_s:
            return self._label, 1.0
        return None, 0.0

    def infer(self, frame_bgr):
        return self._label, 1.0 if self._label else 0.0


# ───────────────────── (구) HaGRID YOLO — 호환용 ─────────────────────
HAGRID_ALIASES = {
    "like": "like", "dislike": "dislike", "dislike_inverted": "dislike",
    "two_up": "two", "peace": "two", "peace_inverted": "two",
    "three": "three", "three2": "three", "four": "four",
}


class HagridYoloRecognizer(Recognizer):
    """(구) HaGRID 검출 YOLO. 방향(상하좌우)을 못 줘서 새 방향 체계엔 부적합 —
    호환/폴백용으로만 남김. 기본 인식기는 MediaPipeHandsRecognizer."""

    def __init__(self, model_path: str, conf_threshold: float = 0.5, imgsz: int = 640):
        import os
        from ultralytics import YOLO
        model_path = os.path.expanduser(model_path)
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"제스처 모델이 없음: {model_path}")
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.imgsz = imgsz

    def infer(self, frame_bgr):
        results = self.model.predict(frame_bgr, imgsz=self.imgsz,
                                     conf=self.conf_threshold, verbose=False)
        best_label, best_conf = None, 0.0
        for r in results:
            if r.boxes is None:
                continue
            for b in r.boxes:
                conf = float(b.conf[0])
                raw = r.names[int(b.cls[0])]
                label = HAGRID_ALIASES.get(raw)
                if label is not None and conf > best_conf:
                    best_label, best_conf = label, conf
        return best_label, best_conf
