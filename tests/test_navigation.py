import contextlib
import io
import json
import tempfile
import threading
import unittest
from collections import deque
from pathlib import Path
from unittest.mock import Mock, patch

from navigation import Config, Navigator, choose_turn, filtered_distance, return_angles, rotation_clear
from robot_navigation import Hardware, PINS, SessionLog, load_config, main


def config():
    return Config(robot_width_cm=18, robot_length_cm=24, sensor_forward_offset_cm=10)


def scan(left=80, right=60, front=18):
    return {0: right, 45: right, 90: front, 135: left, 180: left}


class FakeHardware:
    def __init__(self, distances, scans=(), ir=False):
        self.distances = deque(distances)
        self.scans = deque(scans)
        self.commands = []
        self.ir = ir

    def ir_blocked(self):
        return self.ir

    def measure(self, angle):
        self.commands.append(("measure", angle))
        return self.distances.popleft()

    def scan(self, angles):
        self.commands.append(("scan", tuple(angles)))
        return self.scans.popleft()

    def brake(self, reason):
        self.commands.append(("brake", reason))

    def forward(self, seconds):
        self.commands.append(("forward", seconds))

    def turn(self, angle):
        self.commands.append(("turn", angle))


class PolicyTests(unittest.TestCase):
    def test_filter_requires_three_valid_values_and_rejects_invalids(self):
        c = config()
        for values in ([], [None] * 5, [20], [20, 21, None, -1, float("nan")]):
            self.assertIsNone(filtered_distance(values, c))
        self.assertEqual(filtered_distance([20, 21, 22, None, 1000], c), 21)
        self.assertEqual(filtered_distance([20, 20, 21, 22, 300], c), 21)

    def test_more_space_on_right_overrides_left_preference(self):
        angle, details = choose_turn(scan(left=35, right=100), config())
        self.assertEqual(angle, 45)
        self.assertEqual(details["side"], "right")

    def test_compare_minimum_sector_clearance(self):
        values = {0: 100, 45: 25, 90: 18, 135: 60, 180: 60}
        self.assertEqual(choose_turn(values, config())[0], -45)

    def test_blocked_and_unknown_sides_do_not_choose_a_turn(self):
        c = config()
        self.assertIsNone(choose_turn(scan(left=5, right=5), c)[0])
        for angle in (0, 45, 90, 135, 180):
            values = scan()
            values[angle] = None
            self.assertIsNone(choose_turn(values, c)[0])

    def test_width_blocks_rotation_even_if_front_is_clear(self):
        c = Config(robot_width_cm=40, robot_length_cm=30, sensor_forward_offset_cm=10)
        self.assertFalse(rotation_clear(scan(left=25, right=25, front=30), c))

    def test_auto_requires_actual_dimensions(self):
        with self.assertRaises(ValueError):
            Config().validate(require_geometry=True)
        config().validate(require_geometry=True)

    def test_invalid_config_is_rejected(self):
        for changes in ({"samples": 2}, {"step_seconds": 2}, {"move_speed": 101}, {"servo_step_deg": 0}, {"min_valid_samples": 3.2}, {"turn_right_deg_per_sec": float("nan")}):
            with self.assertRaises(ValueError):
                Config(**changes).validate()

    def test_positive_right_heading_looks_left_to_original_course(self):
        angles = return_angles(45, config())
        self.assertEqual(angles[1], 135)
        self.assertEqual(return_angles(-45, config())[1], 45)

    def test_invalid_front_never_moves(self):
        hw = FakeHardware([None])
        nav = Navigator(config(), hw, Mock())
        self.assertFalse(nav.tick())
        self.assertFalse(any(kind in ("forward", "turn") for kind, _ in hw.commands))

    def test_ir_obstacle_never_moves(self):
        hw = FakeHardware([], ir=True)
        self.assertFalse(Navigator(config(), hw, Mock()).tick())
        self.assertEqual(hw.commands, [("brake", "ir_obstacle")])

    def test_forward_leg_is_measured_again_after_turn(self):
        hw = FakeHardware([18, 10], [scan(left=35, right=80)])
        nav = Navigator(config(), hw, Mock())
        self.assertTrue(nav.tick())
        self.assertFalse(nav.tick())
        self.assertEqual(nav.heading, 45)
        self.assertFalse(any(kind == "forward" for kind, _ in hw.commands))

    def test_complete_bypass_waits_for_minimum_steps_and_two_clear_scans(self):
        c = config()
        corridor = dict.fromkeys(return_angles(45, c), 80)
        hw = FakeHardware([18] + [80] * 20, [scan(left=35, right=80), corridor, corridor, scan(front=80)])
        nav = Navigator(c, hw, Mock())
        self.assertTrue(nav.tick())
        for _ in range(c.min_bypass_steps):
            self.assertTrue(nav.tick())
        self.assertEqual(sum(kind == "scan" for kind, _ in hw.commands), 1)
        self.assertTrue(nav.tick())
        self.assertEqual(nav.heading, 45)
        self.assertTrue(nav.tick())
        self.assertEqual(nav.heading, 0)
        self.assertEqual([value for kind, value in hw.commands if kind == "turn"], [45, -45])
        self.assertEqual(hw.commands[-1], ("turn", -45))

    def test_unknown_return_ray_never_counts_as_clear(self):
        c = config()
        corridor = dict.fromkeys(return_angles(45, c), 80)
        corridor[return_angles(45, c)[0]] = None
        hw = FakeHardware([80, 80], [corridor])
        nav = Navigator(c, hw, Mock())
        nav.heading = 45
        nav.bypass_steps = c.min_bypass_steps
        self.assertTrue(nav.tick())
        self.assertEqual(nav.clear_checks, 0)
        self.assertFalse(any(kind == "turn" for kind, _ in hw.commands))

    def test_new_front_obstacle_after_return_scan_stops(self):
        c = config()
        corridor = dict.fromkeys(return_angles(45, c), 80)
        hw = FakeHardware([80, 5], [corridor])
        nav = Navigator(c, hw, Mock())
        nav.heading = 45
        nav.bypass_steps = c.min_bypass_steps
        self.assertFalse(nav.tick())
        self.assertEqual(hw.commands[-1], ("brake", "front_changed_after_scan"))

    def test_return_rotation_is_guarded(self):
        c = config()
        corridor = dict.fromkeys(return_angles(45, c), 80)
        hw = FakeHardware([80], [corridor, scan(left=4, front=80)])
        nav = Navigator(c, hw, Mock())
        nav.heading = 45
        nav.bypass_steps = c.min_bypass_steps
        nav.clear_checks = 1
        self.assertFalse(nav.tick())
        self.assertEqual(hw.commands[-1], ("brake", "return_rotation_blocked"))

    def test_bypass_has_a_finite_limit(self):
        c = config()
        hw = FakeHardware([80], [dict.fromkeys(return_angles(45, c), 10)])
        nav = Navigator(c, hw, Mock())
        nav.heading = 45
        nav.bypass_steps = c.max_bypass_steps
        self.assertFalse(nav.tick())
        self.assertEqual(hw.commands[-1], ("brake", "bypass_step_limit"))

    def test_clear_front_on_rescan_does_not_turn(self):
        hw = FakeHardware([18], [scan(front=80)])
        nav = Navigator(config(), hw, Mock())
        self.assertTrue(nav.tick())
        self.assertEqual(nav.heading, 0)
        self.assertFalse(any(kind in ("turn", "forward") for kind, _ in hw.commands))

    def test_blocked_bypass_cannot_reuse_previous_turn(self):
        hw = FakeHardware([18, 5], [scan(left=35, right=80)])
        nav = Navigator(config(), hw, Mock())
        self.assertTrue(nav.tick())
        self.assertFalse(nav.tick())
        self.assertEqual([value for kind, value in hw.commands if kind == "turn"], [45])


class HardwareTests(unittest.TestCase):
    def hardware(self):
        hw = Hardware(config(), Mock(), threading.Event())
        hw.gpio = Mock(LOW=0)
        hw.gpio.input.return_value = 1
        hw.left = Mock()
        hw.right = Mock()
        hw.servo = Mock()
        hw.position = 90
        return hw

    def test_filter_makes_exactly_five_sensor_calls(self):
        hw = self.hardware()
        hw.raw_distance = Mock(side_effect=[20, 21, 22, None, 400])
        self.assertEqual(hw.measure(90), 21.5)
        self.assertEqual(hw.raw_distance.call_count, 5)

    def test_invalid_measurement_does_not_divide_by_zero(self):
        hw = self.hardware()
        hw.raw_distance = Mock(side_effect=[20, 21, None, None, None])
        self.assertIsNone(hw.measure(90))

    def test_interruption_brakes_both_motors(self):
        hw = self.hardware()
        hw.ir_blocked = Mock(return_value=False)
        hw.wait = Mock(side_effect=InterruptedError("cancel"))
        with self.assertRaises(InterruptedError):
            hw.forward(0.1)
        self.assertEqual(hw.left.ChangeDutyCycle.call_args.args, (0,))
        self.assertEqual(hw.right.ChangeDutyCycle.call_args.args, (0,))
        self.assertFalse(hw.moving)

    def test_gpio_exception_brakes_both_motors(self):
        hw = self.hardware()
        hw.ir_blocked = Mock(return_value=False)
        hw.gpio.output.side_effect = [RuntimeError("gpio"), None, None, None, None]
        with self.assertRaises(RuntimeError):
            hw.forward(0.1)
        self.assertEqual(hw.left.ChangeDutyCycle.call_args.args, (0,))
        self.assertEqual(hw.right.ChangeDutyCycle.call_args.args, (0,))

    def test_ir_change_during_pulse_brakes_both_motors(self):
        hw = self.hardware()
        hw.use_ir = True
        hw.ir_blocked = Mock(side_effect=[False, True])
        hw.wait = Mock()
        with self.assertRaises(InterruptedError):
            hw.forward(0.1)
        self.assertEqual(hw.left.ChangeDutyCycle.call_args.args, (0,))
        self.assertEqual(hw.right.ChangeDutyCycle.call_args.args, (0,))

    def test_cleanup_releases_gpio_even_if_brake_fails(self):
        hw = self.hardware()
        hw.left.ChangeDutyCycle.side_effect = RuntimeError("pwm")
        with self.assertRaises(RuntimeError):
            hw.close()
        hw.left.stop.assert_called_once()
        hw.right.stop.assert_called_once()
        hw.servo.stop.assert_called_once()
        hw.gpio.cleanup.assert_called_once()

    def test_scan_returns_sensor_to_centre_after_measurement_failure(self):
        hw = self.hardware()
        hw.point = Mock()
        hw.measure = Mock(side_effect=RuntimeError("measurement"))
        with self.assertRaises(RuntimeError):
            hw.scan((0, 45, 90, 135, 180))
        hw.point.assert_called_once_with(90)

    def test_servo_off_centre_blocks_motion(self):
        hw = self.hardware()
        hw.position = 45
        with self.assertRaises(RuntimeError):
            hw.forward(0.1)
        hw.left.ChangeDutyCycle.assert_not_called()

    def test_echo_stuck_high_is_invalid(self):
        hw = self.hardware()
        hw.wait = Mock()
        hw.gpio.input.return_value = 1
        self.assertIsNone(hw.raw_distance(1))
        self.assertEqual(hw.emit.call_args.kwargs["reason"], "echo_high_before_trigger")

    def test_no_echo_times_out(self):
        hw = self.hardware()
        hw.config.echo_timeout_seconds = 0.001
        hw.gpio.input.return_value = 0
        self.assertIsNone(hw.raw_distance(1))
        self.assertEqual(hw.emit.call_args.kwargs["reason"], "echo_rise_timeout")

    def test_long_echo_times_out(self):
        hw = self.hardware()
        hw.config.echo_timeout_seconds = 0.001
        hw.gpio.input.side_effect = lambda pin: next(levels, 1)
        levels = iter((0, 1))
        self.assertIsNone(hw.raw_distance(1))
        self.assertEqual(hw.emit.call_args.kwargs["reason"], "echo_fall_timeout")

    def test_ir_polarity_and_opt_in(self):
        hw = self.hardware()
        hw.gpio.input.return_value = 0
        self.assertFalse(hw.ir_blocked())
        hw.use_ir = True
        self.assertTrue(hw.ir_blocked())
        hw.config.ir_active_level = 1
        self.assertFalse(hw.ir_blocked())

    def test_existing_pin_mapping_is_preserved(self):
        self.assertEqual([PINS[name] for name in ("IN1", "IN2", "IN3", "IN4", "ENA", "ENB", "echo", "trigger", "servo", "ir_left", "ir_right")], [20, 21, 19, 26, 16, 13, 0, 1, 23, 12, 17])

    def test_main_default_scan_is_stationary_and_always_closes(self):
        with tempfile.TemporaryDirectory() as folder, patch("robot_navigation.Hardware") as factory, contextlib.redirect_stdout(io.StringIO()):
            hw = factory.return_value
            self.assertEqual(main(["--log-dir", folder]), 0)
            hw.scan.assert_called_once_with((0, 45, 90, 135, 180))
            hw.forward.assert_not_called()
            hw.turn.assert_not_called()
            hw.close.assert_called_once()
            records = [json.loads(line) for line in next(Path(folder).glob("*/events.jsonl")).read_text().splitlines()]
            self.assertEqual(records[-1]["event"], "session_end")
            self.assertIn("source_sha256", records[0])

    def test_initialization_failure_still_closes_and_logs_traceback(self):
        with tempfile.TemporaryDirectory() as folder, patch("robot_navigation.Hardware") as factory, contextlib.redirect_stdout(io.StringIO()):
            factory.return_value.initialize.side_effect = RuntimeError("init failure")
            self.assertEqual(main(["--log-dir", folder]), 1)
            factory.return_value.close.assert_called_once()
            records = [json.loads(line) for line in next(Path(folder).glob("*/events.jsonl")).read_text().splitlines()]
            self.assertIn("init failure", next(item["traceback"] for item in records if item["event"] == "exception"))

    def test_invalid_auto_config_does_not_initialize_gpio(self):
        with patch("robot_navigation.Hardware") as factory, contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as raised:
                main(["--mode", "auto"])
            self.assertEqual(raised.exception.code, 2)
            factory.assert_not_called()


if __name__ == "__main__":
    unittest.main()
