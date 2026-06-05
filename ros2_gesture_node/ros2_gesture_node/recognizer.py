"""제스처 인식기 — 프레임(BGR ndarray) → (라벨, 신뢰도).

구현체:
  HagridYoloRecognizer : HaGRID 사전학습 YOLO (ultralytics 로드; .pt/.onnx/.engine)
  MockRecognizer       : 외부에서 주입한 라벨을 그대로 반환 (카메라 없이 개발용)

인식기를 갈아끼워도(예: MediaPipe 폴백) 노드는 이 인터페이스만 본다.

HaGRID 클래스 → 우리 어휘 매핑:
  HaGRID에는 'two'라는 클래스가 없고 two_up/peace 가 두 손가락이다.
  three는 three/three2 두 변형이 있다. 모두 ALIASES로 통합한다.
"""
from typing import Optional, Tuple

# HaGRID 클래스명 → 메뉴 어휘 (menu.py 의 TRIGGER/BACK/SELECT_KEYS)
ALIASES = {
    "like": "like",
    "one": "one",
    "two_up": "two", "two_up_inverted": "two", "peace": "two", "peace_inverted": "two",
    "three": "three", "three2": "three",
    "four": "four",
    "palm": "palm", "stop": "palm", "stop_inverted": "palm",
}


class Recognizer:
    """인터페이스. infer는 우리 어휘로 정규화된 라벨을 반환한다."""

    def infer(self, frame_bgr) -> Tuple[Optional[str], float]:
        raise NotImplementedError


class HagridYoloRecognizer(Recognizer):
    """HaGRID 사전학습 YOLO 검출기.
    모델: hukenovs/hagrid 의 YOLO 체크포인트(.pt) 또는 ONNX/TensorRT 변환본.
    ultralytics 가 전/후처리(리사이즈·NMS)를 처리하므로 형식 걱정 없음.
    프레임 전체에서 손 제스처를 검출하고 최고 신뢰도 1개를 채택한다.
    TODO: 주인 bbox ROI 크롭(행인 손 무시) — owner bbox 토픽 합의 후.
    """

    def __init__(self, model_path: str, conf_threshold: float = 0.5, imgsz: int = 640):
        from ultralytics import YOLO   # 지연 임포트 (mock 모드는 불필요)
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
                label = ALIASES.get(raw)
                if label is not None and conf > best_conf:
                    best_label, best_conf = label, conf
        return best_label, best_conf


class MockRecognizer(Recognizer):
    """개발용: set_gesture()로 주입된 라벨을 반환 (만료시간 지나면 None).
    노드가 /gesture_mock(String) 토픽을 받아 주입한다 →
      ros2 topic pub /gesture_mock std_msgs/String "data: like" -r 5
    로 카메라·모델 없이 전체 체인(메뉴→adjust_cmd 발행)을 테스트할 수 있다."""

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

    def infer(self, frame_bgr):   # 이미지 기반 경로와의 호환용
        return self._label, 1.0 if self._label else 0.0
