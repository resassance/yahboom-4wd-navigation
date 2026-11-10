import threading
import unittest
from unittest.mock import Mock, patch

from navigation import Config, Navigator, ObstacleDetected
from robot_navigation import Hardware


class ApproachTests(unittest.TestCase):
    def hardware(self, **values):
        h = Hardware(Config(sensor_forward_offset_cm=11, **values), Mock(), threading.Event())
        h.gpio = Mock()
        h.gpio.LOW = 0
        h.gpio.input.return_value = 1
        h.left = Mock()
        h.right = Mock()
        h.position = 90
        return h

    def test_defaults_and_short_approach_interval(self):
        c = Config()
        self.assertEqual(c.safe_distance_cm, 14)
        self.assertLess(c.forward_interval(20), c.forward_interval(100))
        self.assertEqual(c.forward_interval(20), 0.02)

    def test_slowdown_changes_pwm_without_braking(self):
        h = self.hardware()
        h.moving = True
        h.cruising_since = 1
        h.read_classification_ir = Mock()
        h.raw_distance = Mock(return_value=30)
        h.front_distance()
        self.assertTrue(h.moving)
        h.left.ChangeDutyCycle.assert_called_with(h.config.slow_speed)
        self.assertNotIn(unittest.mock.call(0), h.left.ChangeDutyCycle.call_args_list)

    def test_threshold_stops_and_invalid_reading_stops(self):
        for distance in (14, 13, None):
            h = self.hardware()
            h.moving = True
            h.cruising_since = 1
            h.read_classification_ir = Mock()
            h.raw_distance = Mock(return_value=distance)
            h.front_distance()
            self.assertFalse(h.moving)
            h.left.ChangeDutyCycle.assert_called_with(0)

    def test_far_clear_path_uses_normal_speed(self):
        h = self.hardware()
        h.moving = True
        h.cruising_since = 1
        h.read_classification_ir = Mock()
        h.raw_distance = Mock(return_value=100)
        h.front_distance()
        h.left.ChangeDutyCycle.assert_called_with(h.config.move_speed)

    def test_maneuver_speed_stays_constant_for_path_memory(self):
        h = self.hardware()
        h.set_maneuver_mode(True)
        h.moving = True
        h.cruising_since = 1
        h.read_classification_ir = Mock()
        h.raw_distance = Mock(return_value=100)
        h.front_distance()
        h.left.ChangeDutyCycle.assert_called_with(h.config.slow_speed)

    def test_new_settings_validation(self):
        for values in (dict(slow_speed=0), dict(slow_speed=26), dict(slow_distance_cm=10), dict(approach_step_seconds=0), dict(approach_step_seconds=0.2), dict(left_drive_trim_pwm=-16)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                Config(**values).validate()

    def test_approach_model_reduces_overshoot_at_given_plant_speeds(self):
        h = self.hardware()
        h.read_classification_ir = Mock()
        h.measure = Mock(return_value=45)
        clock = [0.0]
        distance = [45.0]
        pwm = [0]
        seen = []
        def apply(value):
            pwm[0] = value
            seen.append(value)
        def wait(seconds):
            seconds = max(0, seconds)
            speed = {25: 35, 15: 18, 0: 0}[pwm[0]]
            distance[0] -= speed * seconds
            clock[0] += seconds
        def raw(sample):
            wait(max(0, h.config.sample_interval_seconds - (clock[0] - h.last_trigger)))
            h.last_trigger = clock[0]
            return distance[0]
        h.left.ChangeDutyCycle.side_effect = apply
        h.wait = wait
        h.raw_distance = raw
        with patch("robot_navigation.time.monotonic", side_effect=lambda: clock[0]):
            for _ in range(200):
                front = h.front_distance()
                if front <= h.config.forward_threshold():
                    break
                h.advance(h.config.forward_interval(front))
        self.assertFalse(h.moving)
        self.assertGreaterEqual(distance[0], 12)
        self.assertLessEqual(distance[0], 14)
        self.assertIn(25, seen)
        self.assertIn(15, seen)


class TurnGuardTests(unittest.TestCase):
    def hardware(self):
        h = Hardware(Config(sensor_forward_offset_cm=11), Mock(), threading.Event(), ir_turn_guard=True)
        h.gpio = Mock()
        h.gpio.LOW = 0
        h.left = Mock()
        h.right = Mock()
        h.position = 90
        h.ir_blocked = Mock(return_value=False)
        return h

    def test_active_side_excluded_from_choice(self):
        h = self.hardware()
        h.gpio.input.side_effect = lambda pin: {12: 1, 17: 0}[pin]
        self.assertEqual(h.available_turn_sides(), {"left"})

    def test_target_side_blocks_before_turn_without_losing_heading(self):
        h = self.hardware()
        h.gpio.input.side_effect = lambda pin: {12: 1, 17: 0}[pin]
        with self.assertRaises(ObstacleDetected) as cm:
            h.turn(45)
        self.assertFalse(cm.exception.heading_uncertain)
        h.left.ChangeDutyCycle.assert_not_called()

    def test_new_detection_during_turn_brakes_and_invalidates_heading(self):
        h = self.hardware()
        h.turn_ir_state = Mock(side_effect=[(False, False), (False, True)])
        clock = [0.0]
        h.wait = lambda seconds: clock.__setitem__(0, clock[0] + seconds)
        with patch("robot_navigation.time.monotonic", side_effect=lambda: clock[0]):
            with self.assertRaises(ObstacleDetected) as cm:
                h.turn(45)
        self.assertTrue(cm.exception.heading_uncertain)
        self.assertFalse(h.moving)
        h.left.ChangeDutyCycle.assert_called_with(0)

    def test_turn_away_from_existing_detection_allowed_if_ultrasound_clear(self):
        h = self.hardware()
        h.turn_ir_state = Mock(return_value=(True, False))
        clock = [0.0]
        h.wait = lambda seconds: clock.__setitem__(0, clock[0] + seconds)
        with patch("robot_navigation.time.monotonic", side_effect=lambda: clock[0]):
            h.turn(45)
        self.assertFalse(h.moving)

    def test_both_sides_block_both_turns(self):
        h = self.hardware()
        h.gpio.input.return_value = 0
        self.assertEqual(h.available_turn_sides(), set())

    def test_guard_optional(self):
        h = Hardware(Config(), Mock(), threading.Event())
        self.assertEqual(h.available_turn_sides(), {"left", "right"})

    def test_navigation_chooses_other_side_but_keeps_geometry_check(self):
        h = Mock()
        h.forward_runtime_seconds.return_value = 0
        h.available_turn_sides.return_value = {"left"}
        n = Navigator(Config(sensor_forward_offset_cm=11), h, Mock(), ir_turn_guard=True)
        n.plan_route({45: 100, 68: 100, 90: 14, 112: 100, 135: 100})
        self.assertEqual(n.escape_sign, -1)
        h.turn.assert_called_once_with(-45)
        h.turn.reset_mock()
        n.plan_route({45: 100, 68: 100, 90: 4, 112: 100, 135: 100})
        h.turn.assert_not_called()

    def test_opposite_side_reappearing_during_turn_is_new_hazard(self):
        h = self.hardware()
        states = iter([(True, False), (False, False), (True, False)])
        h.turn_ir_state = Mock(side_effect=lambda: next(states, (True, False)))
        clock = [0.0]
        h.wait = lambda seconds: clock.__setitem__(0, clock[0] + seconds)
        with patch("robot_navigation.time.monotonic", side_effect=lambda: clock[0]):
            with self.assertRaises(ObstacleDetected) as cm:
                h.turn(45)
        self.assertTrue(cm.exception.heading_uncertain)
        self.assertFalse(h.moving)

    def test_recorded_second_turn_obstacle_still_blocks_rotation(self):
        h = Mock()
        h.forward_runtime_seconds.return_value = 0
        n = Navigator(Config(sensor_forward_offset_cm=11), h, Mock())
        n.heading = 45
        n.guarded_turn(45, {45: 52.085, 68: 52.359, 90: 54.689, 112: 126.609, 135: 6.226}, "turning_out")
        h.turn.assert_not_called()
        self.assertEqual(n.phase, "waiting")
