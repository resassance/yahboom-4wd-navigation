import threading
import unittest
from unittest.mock import Mock, patch

from navigation import Config, Navigator, ObstacleDetected
from robot_navigation import Hardware
from test_infrared import calibrated


class MotionTests(unittest.TestCase):
    def hardware(self, config=None):
        events = []
        h = Hardware(config or calibrated(), lambda event, **data: events.append((event, data)), threading.Event())
        h.position = 90
        h.gpio = Mock()
        h.gpio.LOW = 0
        h.left = Mock()
        h.right = Mock()
        return h, events

    def test_ir_brackets_front_measurement_without_stopping(self):
        h, _ = self.hardware()
        h.moving = True
        h.cruising_since = 0
        calls = []
        h.read_classification_ir = lambda: calls.append("ir")
        h.raw_distance = lambda *args, **kw: calls.append("ultrasound") or 40
        h.front_distance()
        self.assertEqual(calls, ["ir", "ultrasound", "ir"])
        self.assertTrue(h.moving)

    def test_classification_requires_both_reads_consistent(self):
        h, _ = self.hardware()
        p = h.config.ir_obstacle_patterns["permitted"]
        h.raw_distance = Mock(return_value=10)
        h.read_classification_ir = Mock(side_effect=[p, p, p, p, p, [1, 0, 0, 1]])
        self.assertEqual(h.classify_obstacle()["label"], "unknown")
        self.assertFalse(h.moving)

    def test_push_invalid_ultrasound_brakes(self):
        h, events = self.hardware()
        h.ir_blocked = Mock(return_value=False)
        h.raw_distance = Mock(side_effect=[10, None])
        h.wait = Mock()
        result = h.push_obstacle("permitted")
        self.assertEqual(result, "invalid_distance")
        self.assertFalse(h.moving)
        self.assertEqual(h.left.ChangeDutyCycle.call_args.args, (0,))
        self.assertTrue(any(event == "push_end" for event, _ in events))

    def test_push_refuses_unconfirmed_obstacle_or_legacy_ir(self):
        h, _ = self.hardware()
        with self.assertRaises(ValueError):
            h.push_obstacle("unknown")
        h.ir_blocked = Mock(return_value=True)
        with self.assertRaises(ObstacleDetected):
            h.push_obstacle("permitted")
        h.left.ChangeDutyCycle.assert_not_called()

    def test_push_deadline_and_low_pwm(self):
        h, events = self.hardware()
        h.ir_blocked = Mock(return_value=False)
        clock = [0.0]
        h.raw_distance = Mock(return_value=9)
        def wait(seconds):
            clock[0] += seconds
        h.wait = wait
        with patch("robot_navigation.time.monotonic", side_effect=lambda: clock[0]):
            self.assertEqual(h.push_obstacle("permitted"), "time_limit")
        self.assertLessEqual(clock[0], 0.3)
        self.assertIn(unittest.mock.call(10), h.left.ChangeDutyCycle.call_args_list)
        self.assertFalse(h.moving)

    def test_stop_request_during_push_brakes(self):
        h, _ = self.hardware()
        h.ir_blocked = Mock(return_value=False)
        h.raw_distance = Mock(return_value=10)
        h.wait = Mock(side_effect=InterruptedError("Stop requested"))
        with self.assertRaises(InterruptedError):
            h.push_obstacle("permitted")
        self.assertFalse(h.moving)
        self.assertEqual(h.left.ChangeDutyCycle.call_args.args, (0,))

    def test_push_only_once_until_bypass_completed(self):
        h = Mock()
        h.front_distance.return_value = 10
        h.ir_blocked.return_value = False
        h.forward_runtime_seconds.return_value = 0
        h.classify_obstacle.return_value = {"label": "permitted", "reason": "confirmed", "stable_samples": 3}
        nav = Navigator(calibrated(), h, Mock())
        nav.scan = Mock(return_value={45: 5, 68: 5, 90: 10, 112: 5, 135: 5})
        nav.plan_route = Mock(return_value=True)
        for _ in range(2):
            nav.phase = "cruise"
            nav.tick()
        self.assertEqual(h.push_obstacle.call_count, 1)
        nav.bypass_drive_seconds = nav.config.min_return_progress_seconds
        nav.path.complete()
        nav.resume_course()
        nav.tick()
        self.assertEqual(h.push_obstacle.call_count, 2)

    def test_unknown_and_dangerous_take_existing_bypass(self):
        for label in ("unknown", "dangerous"):
            h = Mock()
            h.front_distance.return_value = 10
            h.ir_blocked.return_value = False
            h.forward_runtime_seconds.return_value = 0
            h.classify_obstacle.return_value = {"label": label, "reason": "test", "stable_samples": 0}
            nav = Navigator(Config(), h, Mock())
            nav.scan = Mock(return_value={45: 5, 68: 5, 90: 10, 112: 5, 135: 5})
            nav.plan_route = Mock(return_value=True)
            nav.tick()
            h.push_obstacle.assert_not_called()
            nav.plan_route.assert_called_once()

    def test_push_stops_even_if_logging_blocks(self):
        import time
        h, _ = self.hardware(calibrated(push_seconds=0.03))
        h.ir_blocked = Mock(return_value=False)
        motor_off = threading.Event()
        def pwm(value):
            if value == 0:
                motor_off.set()
        h.left.ChangeDutyCycle.side_effect = pwm
        def raw(sample, **kwargs):
            if kwargs:
                self.assertTrue(motor_off.wait(0.15))
            return 10
        h.raw_distance = raw
        h.push_obstacle("permitted")
        self.assertTrue(motor_off.is_set())

    def test_push_rechecks_working_distance_before_enabling_motors(self):
        h, _ = self.hardware()
        h.ir_blocked = Mock(return_value=False)
        h.raw_distance = Mock(return_value=30)
        self.assertEqual(h.push_obstacle("permitted"), "outside_working_range")
        h.left.ChangeDutyCycle.assert_not_called()

    def test_false_clear_after_push_does_not_rearm_push(self):
        h = Mock()
        h.front_distance.return_value = 10
        h.ir_blocked.return_value = False
        h.forward_runtime_seconds.return_value = 0
        h.classify_obstacle.return_value = {"label": "permitted", "reason": "confirmed", "stable_samples": 3}
        nav = Navigator(calibrated(), h, Mock())
        nav.scan = Mock(return_value={45: 100, 68: 100, 90: 100, 112: 100, 135: 100})
        nav.tick()
        nav.resume_course()
        nav.tick()
        self.assertEqual(h.push_obstacle.call_count, 1)

    def test_normal_front_guard_keeps_stopping_at_threshold(self):
        for distance in (None, 10, 9):
            h, _ = self.hardware()
            h.moving = True
            h.cruising_since = 0
            h.read_classification_ir = Mock()
            h.raw_distance = Mock(return_value=distance)
            self.assertEqual(h.front_distance(), distance)
            self.assertFalse(h.moving)

    def test_push_read_deadline_does_not_trigger_ultrasound(self):
        h, events = self.hardware()
        h.wait = Mock()
        with patch("robot_navigation.time.monotonic", return_value=1):
            self.assertIsNone(h.raw_distance(1, deadline=1))
        h.gpio.output.assert_not_called()
        self.assertEqual(events[-1][1]["reason"], "push_deadline")

    def test_custom_inputs_initialized_and_cleaned(self):
        import sys
        h, _ = self.hardware(calibrated(ir_classification_pins=[5, 4], ir_obstacle_patterns={"permitted": [1, 0]}, ir_background_pattern=[0, 0]))
        gpio = h.gpio
        gpio.BCM = 11
        gpio.IN = 1
        gpio.OUT = 0
        gpio.input.return_value = 0
        h.point = Mock()
        with patch.dict(sys.modules, {"RPi": Mock(GPIO=gpio), "RPi.GPIO": gpio}):
            h.initialize()
        self.assertIn(unittest.mock.call(5, gpio.IN), gpio.setup.call_args_list)
        self.assertIn(unittest.mock.call(4, gpio.IN), gpio.setup.call_args_list)
        h.close()
        self.assertTrue({4, 5}.issubset(gpio.cleanup.call_args.args[0]))

    def test_clear_corridor_after_push_resumes_without_second_push(self):
        h = Mock()
        h.front_distance.return_value = 10
        h.ir_blocked.return_value = False
        h.forward_runtime_seconds.return_value = 0
        h.classify_obstacle.return_value = {"label": "permitted", "reason": "confirmed", "stable_samples": 3}
        h.push_obstacle.return_value = "time_limit"
        nav = Navigator(calibrated(), h, Mock())
        nav.scan = Mock(return_value={45: 100, 68: 100, 90: 100, 112: 100, 135: 100})
        nav.bypass_drive_seconds = 0.5
        nav.tick()
        self.assertEqual(nav.bypass_drive_seconds, 0)
        self.assertEqual(nav.phase, "cruise")
        self.assertTrue(nav.push_attempted)
        h.turn.assert_not_called()

    def test_invalid_push_measurement_waits_without_turning(self):
        h = Mock()
        h.front_distance.return_value = 10
        h.ir_blocked.return_value = False
        h.forward_runtime_seconds.return_value = 0
        h.classify_obstacle.return_value = {"label": "permitted", "reason": "confirmed", "stable_samples": 3}
        h.push_obstacle.return_value = "invalid_distance"
        nav = Navigator(calibrated(), h, Mock())
        nav.scan = Mock(return_value={45: 100, 68: 100, 90: 100, 112: 100, 135: 100})
        nav.tick()
        self.assertEqual(nav.phase, "waiting")
        h.turn.assert_not_called()
        h.advance.assert_not_called()
