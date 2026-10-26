import json
import math
from dataclasses import dataclass
from numbers import Real


def _number(value, name, minimum, maximum, integer=False):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    if integer and not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")


def _hue_range(value, name):
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError(f"{name} must contain two hue values")
    for hue in value:
        _number(hue, name, 0, 179, integer=True)
    if value[0] > value[1]:
        raise ValueError(f"{name} must be ordered")


@dataclass(frozen=True)
class ColorConfig:
    fps: float = 5
    width: int = 320
    roi: tuple = (0.15, 0.15, 0.85, 0.75)
    min_area_ratio: float = 0.04
    min_saturation: int = 100
    min_value: int = 60
    confirm_frames: int = 3
    red_hue_ranges: tuple = ((0, 10), (170, 179))
    blue_hue_range: tuple = (95, 130)

    def validate(self):
        _number(self.fps, "fps", 1, 10)
        _number(self.width, "width", 160, 640, integer=True)
        _number(self.min_area_ratio, "min_area_ratio", 0, 1)
        if self.min_area_ratio == 0:
            raise ValueError("min_area_ratio must be positive")
        _number(self.min_saturation, "min_saturation", 0, 255, integer=True)
        _number(self.min_value, "min_value", 0, 255, integer=True)
        _number(self.confirm_frames, "confirm_frames", 1, 1000, integer=True)
        if not isinstance(self.roi, (tuple, list)) or len(self.roi) != 4:
            raise ValueError("roi must contain x0, y0, x1, y1")
        for coordinate in self.roi:
            _number(coordinate, "roi", 0, 1)
        if self.roi[0] >= self.roi[2] or self.roi[1] >= self.roi[3]:
            raise ValueError("roi must have positive width and height")
        if not isinstance(self.red_hue_ranges, (tuple, list)) or len(self.red_hue_ranges) != 2:
            raise ValueError("red_hue_ranges must contain two hue ranges")
        for value in self.red_hue_ranges:
            _hue_range(value, "red_hue_ranges")
        _hue_range(self.blue_hue_range, "blue_hue_range")


def load_color_config(path):
    with open(path, encoding="utf-8") as source:
        values = json.load(source)
    if not isinstance(values, dict):
        raise ValueError("color configuration must be an object")
    try:
        config = ColorConfig(**values)
    except TypeError as error:
        raise ValueError(f"invalid color configuration: {error}") from error
    config.validate()
    return config


class ColorDetector:
    def __init__(self, config):
        if not isinstance(config, ColorConfig):
            raise ValueError("config must be a ColorConfig")
        config.validate()
        self.config = config
        self._candidate = "unknown"
        self._consecutive = 0

    def process(self, frame):
        import cv2
        import numpy as np

        if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3 or min(frame.shape[:2]) == 0:
            self._candidate = "unknown"
            self._consecutive = 0
            raise ValueError("frame must be a non-empty BGR uint8 image")
        original_height, original_width = frame.shape[:2]
        if original_width > self.config.width:
            height = max(1, round(original_height * self.config.width / original_width))
            frame = cv2.resize(frame, (self.config.width, height), interpolation=cv2.INTER_AREA)
        height, width = frame.shape[:2]
        x0 = min(width - 1, math.floor(self.config.roi[0] * width))
        y0 = min(height - 1, math.floor(self.config.roi[1] * height))
        x1 = max(x0 + 1, math.ceil(self.config.roi[2] * width))
        y1 = max(y0 + 1, math.ceil(self.config.roi[3] * height))
        cropped = frame[y0:y1, x0:x1]
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        kernel = np.ones((3, 3), dtype=np.uint8)

        def largest_component(hue_ranges):
            mask = np.zeros(cropped.shape[:2], dtype=np.uint8)
            for low, high in hue_ranges:
                color_mask = cv2.inRange(hsv, (low, self.config.min_saturation, self.config.min_value), (high, 255, 255))
                mask = cv2.bitwise_or(mask, color_mask)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
            if count <= 1:
                return 0.0, None
            component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
            area = int(stats[component, cv2.CC_STAT_AREA])
            left = int(stats[component, cv2.CC_STAT_LEFT]) + x0
            top = int(stats[component, cv2.CC_STAT_TOP]) + y0
            right = left + int(stats[component, cv2.CC_STAT_WIDTH])
            bottom = top + int(stats[component, cv2.CC_STAT_HEIGHT])
            return area / cropped.shape[0] / cropped.shape[1], original_box(left, top, right, bottom)

        def original_box(left, top, right, bottom):
            left = max(0, min(original_width - 1, math.floor(left * original_width / width)))
            top = max(0, min(original_height - 1, math.floor(top * original_height / height)))
            right = min(original_width, math.ceil(right * original_width / width))
            bottom = min(original_height, math.ceil(bottom * original_height / height))
            return [left, top, right - left, bottom - top]

        red_ratio, red_box = largest_component(self.config.red_hue_ranges)
        blue_ratio, blue_box = largest_component((self.config.blue_hue_range,))
        red = red_ratio >= self.config.min_area_ratio
        blue = blue_ratio >= self.config.min_area_ratio
        candidate = "unknown"
        reason = "ambiguous" if red and blue else "no_color"
        box = None
        if red != blue:
            candidate = "red" if red else "blue"
            box = red_box if red else blue_box
        if candidate == "unknown":
            self._candidate = "unknown"
            self._consecutive = 0
        elif candidate == self._candidate:
            self._consecutive = min(self._consecutive + 1, self.config.confirm_frames)
        else:
            self._candidate = candidate
            self._consecutive = 1
        label = "unknown"
        if candidate != "unknown":
            if self._consecutive >= self.config.confirm_frames:
                label = candidate
                reason = "confirmed"
            else:
                reason = "confirming"
        return {
            "label": label,
            "candidate": candidate,
            "reason": reason,
            "red_ratio": red_ratio,
            "blue_ratio": blue_ratio,
            "box": box,
            "roi_box": original_box(x0, y0, x1, y1),
        }
