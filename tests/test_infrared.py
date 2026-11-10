import unittest
from dataclasses import replace

from navigation import Config, classify_infrared


def calibrated(**changes):
    values = dict(sensor_forward_offset_cm=11, ir_classification_pins=[3, 5, 4, 18], ir_obstacle_patterns={"dangerous": [1, 0, 0, 1], "permitted": [1, 1, 0, 1]}, ir_background_pattern=[1, 1, 1, 1], ir_min_distance_cm=8, ir_max_distance_cm=12)
    values.update(changes)
    return Config(**values)


class InfraredTests(unittest.TestCase):
    def test_default_remains_unknown(self):
        self.assertEqual(classify_infrared(Config(), [[0, 0, 0, 0]] * 3, [10] * 3)["label"], "unknown")

    def test_confirmed_materials(self):
        for label, pattern in calibrated().ir_obstacle_patterns.items():
            result = classify_infrared(calibrated(), [pattern] * 3, [10] * 3)
            self.assertEqual(result["label"], label)
            self.assertEqual(result["stable_samples"], 3)

    def test_unknown_on_noise_missing_data_distance_or_background(self):
        c = calibrated()
        p = c.ir_obstacle_patterns["permitted"]
        for readings, distances in [([p] * 2, [10] * 2), ([p, [1, 0, 0, 1], p], [10] * 3), ([p] * 3, [10, None, 10]), ([p] * 3, [10, 13, 10]), ([c.ir_background_pattern] * 3, [10] * 3), ([[1, 1, 1, 0]] * 3, [10] * 3)]:
            with self.subTest(readings=readings, distances=distances):
                self.assertEqual(classify_infrared(c, readings, distances)["label"], "unknown")

    def test_pin_three_does_not_decide_type(self):
        c = calibrated()
        p = c.ir_obstacle_patterns["permitted"]
        self.assertEqual(classify_infrared(c, [[0] + p[1:]] * 3, [10] * 3)["label"], "permitted")

    def test_reject_invalid_calibration(self):
        cases = [dict(ir_classification_pins=[3]), dict(ir_classification_pins=[5, 5]), dict(ir_classification_pins=[20]), dict(ir_confirm_samples=1), dict(push_seconds=0.31), dict(push_speed=26), dict(ir_obstacle_patterns={"permitted": [1, 1, 1, 1]}), dict(ir_obstacle_patterns={"permitted": [1, 1, 0, 1], "dangerous": [0, 1, 0, 1]}), dict(ir_background_pattern=None), dict(ir_min_distance_cm=None)]
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                calibrated(**changes).validate()

    def test_new_stopping_threshold_preserves_geometry(self):
        self.assertEqual(Config().safe_distance_cm, 10)
        self.assertEqual(replace(Config(), sensor_forward_offset_cm=0).forward_threshold(), 17)

    def test_malformed_numeric_parameters_rejected_cleanly(self):
        for changes in (dict(push_seconds="fast"), dict(push_speed=None), dict(move_speed="fast"), dict(max_distance_cm="far")):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                calibrated(**changes).validate()

    def test_default_group_is_upper_obstacle_inputs(self):
        self.assertEqual(Config().ir_classification_pins, [12, 17])

    def test_upper_inputs_can_classify_after_calibration(self):
        c = Config(ir_classification_pins=[12, 17], ir_obstacle_patterns={"dangerous": [0, 1], "permitted": [1, 0]}, ir_background_pattern=[1, 1], ir_min_distance_cm=8, ir_max_distance_cm=12)
        c.validate()
        self.assertEqual(classify_infrared(c, [[1, 0]] * 3, [10] * 3)["label"], "permitted")
        self.assertEqual(classify_infrared(c, [[0, 1]] * 3, [10] * 3)["label"], "dangerous")
