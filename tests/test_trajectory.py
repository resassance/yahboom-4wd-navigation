import math
import unittest
from unittest.mock import Mock

from navigation import Config, Navigator
from trajectory import PathMemory


class MemoryTests(unittest.TestCase):
    def memory(self, **values):
        events = []
        config = Config(sensor_forward_offset_cm=11, **values)
        memory = PathMemory(config, lambda event, **kw: events.append((event, kw)))
        memory.begin(0)
        return memory, events

    def test_accumulates_actual_runtime_and_signed_displacement(self):
        m, _ = self.memory()
        m.update(1.5, 90)
        m.update(1.5, 0)
        m.update(3.5, 0)
        self.assertAlmostEqual(m.lateral_seconds, 1.5)
        self.assertAlmostEqual(m.forward_seconds, 2)
        m.update(5, -90)
        self.assertAlmostEqual(m.lateral_seconds, 0)

    def test_diagonal_and_left_paths(self):
        m, _ = self.memory()
        m.update(1, -45)
        self.assertAlmostEqual(m.lateral_seconds, -2 ** -0.5)
        self.assertAlmostEqual(m.forward_seconds, 2 ** -0.5)

    def test_begin_does_not_erase_pending_return(self):
        m, _ = self.memory()
        m.update(1, 90)
        m.begin(1)
        self.assertAlmostEqual(m.lateral_seconds, 1)

    def test_time_only_without_calibrated_speed(self):
        m, _ = self.memory()
        m.update(1, 90)
        self.assertIsNone(m.details()["lateral_estimate_cm"])

    def test_optional_wall_reference_needs_planar_consistent_scans(self):
        m, events = self.memory(drive_speed_cm_s=10, range_reference_heading_deg=90)
        scan = lambda front: {angle: front / math.cos(math.radians(90-angle)) for angle in (45, 68, 90, 112, 135)}
        m.observe_scan(scan(100), 0, 90)
        m.update(1, 90)
        m.observe_scan(scan(88), 1, 90)
        self.assertAlmostEqual(m.lateral_seconds, 1.2)
        self.assertTrue(any(e == "path_range_correction" for e, _ in events))

    def test_object_switch_or_turn_does_not_count_as_motion(self):
        for front in (None, 20, 110):
            m, _ = self.memory(drive_speed_cm_s=10, range_reference_heading_deg=90)
            m.observe_scan({45: 140, 68: 108, 90: 100, 112: 108, 135: 140}, 0, 90)
            m.update(1, 90)
            m.observe_scan({45: 140, 68: 108, 90: front, 112: 108, 135: 140}, 1, 90)
            self.assertAlmostEqual(m.lateral_seconds, 1)
        m, _ = self.memory(drive_speed_cm_s=10, range_reference_heading_deg=90)
        m.observe_scan({45: 140, 68: 108, 90: 100, 112: 108, 135: 140}, 0, 90)
        m.record_turn(90, 0)
        m.update(1, 0)
        self.assertAlmostEqual(m.lateral_seconds, 0)

    def test_invalid_calibration_settings(self):
        for settings in (dict(drive_speed_cm_s=0), dict(drive_speed_cm_s=True), dict(drive_speed_cm_s=float("nan")), dict(range_reference_heading_deg=90), dict(drive_speed_cm_s=10, range_reference_heading_deg=180), dict(return_probe_seconds=0), dict(return_tolerance_seconds=1)):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                Config(**settings).validate()


class ReturnTests(unittest.TestCase):
    def navigator(self, side=90):
        hardware = Mock()
        hardware.runtime = 0.0
        hardware.forward_runtime_seconds.side_effect = lambda: hardware.runtime
        hardware.ir_blocked.return_value = False
        hardware.front_distance.return_value = 100
        hardware.scan.return_value = {a: 100 for a in (45, 68, 90, 112, 135)}
        hardware.advance.side_effect = lambda seconds: setattr(hardware, "runtime", hardware.runtime + seconds)
        n = Navigator(Config(sensor_forward_offset_cm=11), hardware, Mock())
        n.path.begin(0)
        hardware.runtime = 1.0
        n.heading = side
        n.update_progress()
        n.heading = 0
        n.trajectory.start()
        return n, hardware

    def test_returns_to_line_before_reporting_bypass_complete(self):
        for side in (-90, 90):
            n, h = self.navigator(side)
            phases = []
            for _ in range(200):
                phases.append(n.phase)
                n.tick()
                if not n.path.active:
                    break
            self.assertFalse(n.path.active)
            self.assertEqual(n.heading, 0)
            self.assertLessEqual(abs(n.path.lateral_seconds), n.config.return_tolerance_seconds)
            self.assertIn("trajectory_cross", phases)
            self.assertGreater(h.runtime, 1)

    def test_obstacle_during_return_keeps_target_and_brakes(self):
        n, h = self.navigator()
        n.phase = "trajectory_cross"
        n.heading = -90
        h.front_distance.return_value = 8
        n.tick()
        self.assertTrue(n.path.active)
        self.assertAlmostEqual(n.path.lateral_seconds, 1)
        h.advance.assert_not_called()
        h.brake.assert_called()

    def test_invalid_scan_never_authorizes_turn(self):
        n, h = self.navigator()
        h.scan.return_value = {90: None}
        n.phase = "trajectory_align"
        n.tick()
        h.turn.assert_not_called()
        h.advance.assert_not_called()
        self.assertTrue(n.path.active)

    def test_unknown_heading_prevents_return(self):
        n, h = self.navigator()
        n.heading_known = False
        n.tick()
        h.turn.assert_not_called()
        h.advance.assert_not_called()

    def test_resume_course_starts_return_instead_of_resetting_memory(self):
        n, h = self.navigator()
        n.phase = "probe_return"
        n.resume_course()
        self.assertTrue(n.path.active)
        self.assertNotEqual(n.phase, "cruise")

    def test_unconfigured_ir_does_not_change_ultrasonic_guard(self):
        n, h = self.navigator()
        n.phase = "trajectory_follow"
        h.front_distance.return_value = None
        n.tick()
        h.advance.assert_not_called()
        self.assertTrue(n.path.active)

    def test_repeated_ir_block_keeps_pending_phase(self):
        n, h = self.navigator()
        h.ir_blocked.return_value = True
        n.tick()
        n.tick()
        self.assertEqual(n.trajectory.pending_phase, "trajectory_follow")
        h.ir_blocked.return_value = False
        n.tick()
        self.assertEqual(n.phase, "trajectory_follow")

    def test_bypass_mode_does_not_classify_or_push(self):
        n, h = self.navigator()
        n.path.active = False
        n.phase = "cruise"
        n.obstacle_mode = "bypass"
        h.front_distance.return_value = 10
        n.tick()
        h.classify_obstacle.assert_not_called()
        h.push_obstacle.assert_not_called()
        self.assertTrue(n.path.active)

    def test_resume_after_wait_rechecks_wide_corridor(self):
        n, h = self.navigator()
        n.heading = -90
        n.trajectory.observed_runtime = h.runtime
        n.trajectory.wait("trajectory_cross", "ir_obstacle")
        n.tick()
        h.scan.return_value = {a: 8 for a in (45, 68, 90, 112, 135)}
        n.tick()
        h.advance.assert_not_called()
        self.assertEqual(n.phase, "trajectory_wait")

    def test_stopped_return_measurements_keep_existing_motor_pulse_length(self):
        n, h = self.navigator()
        n.heading = -90
        n.phase = "trajectory_cross"
        n.trajectory.observed_runtime = h.runtime
        h.front_distance.return_value = 20
        n.tick()
        h.advance.assert_called_once_with(n.config.step_seconds)
        h.brake.assert_called_with("trajectory_cross_step")
