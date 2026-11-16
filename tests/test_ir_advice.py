import threading
import unittest
from unittest.mock import Mock, patch

from navigation import Config, Navigator, choose_turn
from robot_navigation import Hardware


class AdviceTests(unittest.TestCase):
    def test_left_calibration_applies_to_default_and_turn(self):
        self.assertEqual(Config().turn_left_deg_per_sec, 63)
        h = Hardware(Config(), Mock(), threading.Event())
        h.pulse = Mock()
        h.turn(-45)
        self.assertAlmostEqual(h.pulse.call_args.args[1], 45 / 63)

    def test_advice_prefers_clear_side_without_overriding_ultrasound(self):
        c = Config(sensor_forward_offset_cm=11)
        scan = {45: 100, 68: 100, 90: 14, 112: 50, 135: 50}
        angle, _ = choose_turn(scan, c, preferred_sides={"left"})
        self.assertEqual(angle, -45)
        scan[135] = 12
        angle, _ = choose_turn(scan, c, preferred_sides={"left"})
        self.assertEqual(angle, 45)
        scan[90] = 4
        self.assertIsNone(choose_turn(scan, c, preferred_sides={"left"})[0])

    def test_both_ir_active_still_allow_ultrasound_choice(self):
        h = Mock()
        h.forward_runtime_seconds.return_value = 0
        h.available_turn_sides.return_value = set()
        n = Navigator(Config(sensor_forward_offset_cm=11), h, Mock(), ir_assist=True)
        n.plan_route({45: 100, 68: 100, 90: 14, 112: 100, 135: 100})
        h.turn.assert_called_once_with(45)

    def test_favor_left_when_right_ir_active(self):
        h = Mock()
        h.forward_runtime_seconds.return_value = 0
        h.available_turn_sides.return_value = {"left"}
        n = Navigator(Config(sensor_forward_offset_cm=11), h, Mock(), ir_assist=True)
        n.plan_route({45: 100, 68: 100, 90: 14, 112: 100, 135: 100})
        h.turn.assert_called_once_with(-45)

    def test_distant_ir_does_not_interrupt_advisory_turn(self):
        h = Hardware(Config(), Mock(), threading.Event(), ir_assist=True)
        h.position = 90
        h.gpio = Mock()
        h.gpio.LOW = 0
        h.left = Mock()
        h.right = Mock()
        h.ir_blocked = Mock(return_value=False)
        states = iter([(False, True)] * 10 + [(True, True)] * 50)
        h.turn_ir_state = Mock(side_effect=lambda: next(states, (True, True)))
        clock = [0.0]
        h.wait = lambda seconds: clock.__setitem__(0, clock[0] + seconds)
        with patch("robot_navigation.time.monotonic", side_effect=lambda: clock[0]):
            h.turn(-45)
        self.assertGreaterEqual(h.turn_ir_state.call_count, 2)
        self.assertFalse(h.moving)
        h.left.ChangeDutyCycle.assert_called_with(0)

    def test_advisory_and_hard_guard_flags_cannot_be_combined(self):
        import subprocess
        import sys
        from pathlib import Path
        script = Path(__file__).resolve().parents[1] / "robot_navigation.py"
        for extra in ("--ir-turn-guard", "--use-ir"):
            result = subprocess.run([sys.executable, str(script), "--mode", "auto", "--ir-assist", extra], capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn("cannot be combined", result.stderr)
