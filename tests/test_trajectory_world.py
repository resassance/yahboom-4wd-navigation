import math
import unittest

from navigation import Config, Navigator


class World:
    def __init__(self, obstacles):
        self.obstacles = obstacles
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.runtime = 0.0
        self.turns = []
        self.brakes = []

    def forward_runtime_seconds(self):
        return self.runtime

    def ir_blocked(self):
        return False

    def brake(self, reason):
        self.brakes.append(reason)

    def wait_for_retry(self, seconds):
        pass

    def classify_obstacle(self):
        return {"label": "unknown", "reason": "not_calibrated", "stable_samples": 0}

    def distance(self, angle):
        h = math.radians(self.heading)
        origin = (self.x + 11 * math.cos(h), self.y + 11 * math.sin(h))
        bearing = h + math.radians(90 - angle)
        direction = (math.cos(bearing), math.sin(bearing))
        distances = [350.0]
        for xmin, xmax, ymin, ymax in self.obstacles:
            low, high = -float("inf"), float("inf")
            for value, velocity, lower, upper in zip(origin, direction, (xmin, ymin), (xmax, ymax)):
                if abs(velocity) < 1e-9:
                    if not lower <= value <= upper:
                        high = -float("inf")
                        break
                else:
                    ends = sorted(((lower-value)/velocity, (upper-value)/velocity))
                    low, high = max(low, ends[0]), min(high, ends[1])
            if high >= max(0, low) and low > 0:
                distances.append(low)
        return round(min(distances), 3)

    def front_distance(self):
        return self.distance(90)

    def scan(self, angles):
        return {angle: self.distance(angle) for angle in angles}

    def assert_clear(self):
        angle = math.radians(self.heading)
        forward = (math.cos(angle), math.sin(angle))
        side = (-math.sin(angle), math.cos(angle))
        chassis = [(self.x + a * 12 * forward[0] + b * 9.25 * side[0], self.y + a * 12 * forward[1] + b * 9.25 * side[1]) for a in (-1, 1) for b in (-1, 1)]
        for xmin, xmax, ymin, ymax in self.obstacles:
            box = [(x, y) for x in (xmin, xmax) for y in (ymin, ymax)]
            separated = False
            for dx, dy in ((1, 0), (0, 1), forward, side):
                first = [x * dx + y * dy for x, y in chassis]
                second = [x * dx + y * dy for x, y in box]
                if max(first) < min(second) or max(second) < min(first):
                    separated = True
                    break
            if not separated:
                raise AssertionError(("chassis_collision", self.x, self.y, self.heading))

    def advance(self, seconds):
        steps = max(1, math.ceil(seconds * 10 / 0.2))
        for _ in range(steps):
            self.runtime += seconds / steps
            self.x += 10 * seconds / steps * math.cos(math.radians(self.heading))
            self.y += 10 * seconds / steps * math.sin(math.radians(self.heading))
            self.assert_clear()

    def turn(self, degrees):
        steps = max(1, math.ceil(abs(degrees)))
        for _ in range(steps):
            self.heading += degrees / steps
            self.assert_clear()
        self.turns.append(degrees)


class WorldTests(unittest.TestCase):
    def test_obstacle_bypassed_and_original_line_recovered(self):
        cases = [([(36, 46, -7, 7)], 10), ([(36, 46, -7, 7)], None), ([(36, 60, -7, 7), (-100, 200, 55, 65)], 10)]
        for obstacles, speed in cases:
            with self.subTest(obstacles=obstacles, speed=speed):
                events = []
                h = World(obstacles)
                n = Navigator(Config(sensor_forward_offset_cm=11, drive_speed_cm_s=speed), h, lambda event, **kw: events.append((event, kw)), obstacle_mode="bypass")
                for step in range(1200):
                    n.tick()
                    if any(event == "path_return_complete" for event, _ in events):
                        break
                self.assertTrue(any(event == "path_return_complete" for event, _ in events), (n.phase, n.heading, h.x, h.y))
                self.assertAlmostEqual(h.heading, 0)
                self.assertLessEqual(abs(h.y), n.config.return_tolerance_seconds * 10 + 1e-6)
                self.assertGreater(h.x, obstacles[0][1])
                self.assertTrue(any(event == "trajectory_cross_start" for event, _ in events))
