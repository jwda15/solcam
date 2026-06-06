"""ROS 무관 헬퍼 (단위테스트 대상)."""
import re
from typing import Optional


def parse_battery_level(dumpsys_text: str) -> Optional[int]:
    """`adb shell dumpsys battery` 출력에서 level(0~100)을 추출."""
    m = re.search(r"^\s*level:\s*(\d+)", dumpsys_text, re.MULTILINE)
    if not m:
        return None
    return max(0, min(100, int(m.group(1))))
