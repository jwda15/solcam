"""parse_battery_level 단위테스트 — ROS 없이:
    cd ros2_phone_bridge && python3 -m pytest test/test_battery_parse.py -v
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from ros2_phone_bridge.util import parse_battery_level

DUMP = """Current Battery Service state:
  AC powered: false
  USB powered: true
  status: 2
  health: 2
  present: true
  level: 87
  scale: 100
  temperature: 305
"""


def test_parse_normal():
    assert parse_battery_level(DUMP) == 87


def test_parse_missing():
    assert parse_battery_level("no battery info here") is None


def test_parse_clamp():
    assert parse_battery_level("  level: 250") == 100
    assert parse_battery_level("  level: 0") == 0


def test_parse_first_level_only():
    txt = "  level: 42\n  some_level: 9"
    assert parse_battery_level(txt) == 42
