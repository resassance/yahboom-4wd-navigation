import math
import statistics
from dataclasses import asdict, dataclass


@dataclass
class Config:
    safe_distance_cm: float = 20.0
    resume_distance_cm: float = 30.0
    clearance_margin_cm: float = 5.0
    robot_width_cm: float = None
    robot_length_cm: float = None
    sensor_forward_offset_cm: float = None
    move_speed: float = 25.0
    turn_speed: float = 10.0
    step_seconds: float = 0.1
    turn_angle_deg: float = 45.0
    turn_right_deg_per_sec: float = 60.0
    turn_left_deg_per_sec: float = 60.0
    min_bypass_steps: int = 4
    clear_checks_to_resume: int = 2
    max_bypass_steps: int = 30
    samples: int = 5
    min_valid_samples: int = 3
    sample_interval_seconds: float = 0.06
    echo_timeout_seconds: float = 0.03
    max_distance_cm: float = 400.0
    servo_step_deg: int = 3
    servo_step_seconds: float = 0.02
    servo_settle_seconds: float = 0.3
    ir_active_level: int = 0

    def validate(self, require_geometry=False):
        values = asdict(self)
        dimensions = {"robot_width_cm", "robot_length_cm", "sensor_forward_offset_cm"}
        integers = {"samples", "min_valid_samples", "min_bypass_steps", "max_bypass_steps", "clear_checks_to_resume", "servo_step_deg", "ir_active_level"}
        for name, value in values.items():
            if name in dimensions and value is None:
                continue
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f"Invalid numeric configuration: {name}")
            if name in integers and not isinstance(value, int):
                raise ValueError(f"Configuration must be an integer: {name}")
            if name not in {"sensor_forward_offset_cm", "ir_active_level"} and value <= 0:
                raise ValueError(f"Configuration must be positive: {name}")
        if not 0 < self.move_speed <= 100 or not 0 < self.turn_speed <= 100:
            raise ValueError("PWM speed must be between 0 and 100")
        if not 10 <= self.turn_angle_deg <= 60:
            raise ValueError("Turn angle must be between 10 and 60 degrees")
        if not 3 <= self.min_valid_samples <= self.samples <= 15:
            raise ValueError("Require at least 3 valid samples, at most 15 samples")
        if self.ir_active_level not in (0, 1):
            raise ValueError("IR active level must be 0 or 1")
        if self.min_bypass_steps >= self.max_bypass_steps:
            raise ValueError("Bypass minimum must be less than bypass limit")
        if self.step_seconds > 0.5 or self.servo_step_deg > 15:
            raise ValueError("Motion step or servo step is too large for this test version")
        if self.resume_distance_cm < self.safe_distance_cm:
            raise ValueError("Resume distance must be at least the stopping threshold")
        if require_geometry and any(values[name] is None for name in dimensions):
            raise ValueError("Measure and configure robot_width_cm, robot_length_cm, sensor_forward_offset_cm before auto mode")
        if all(values[name] is not None for name in dimensions):
            if abs(self.sensor_forward_offset_cm) > self.robot_length_cm:
                raise ValueError("Check sensor offset: use signed distance from chassis centre")
            if self.forward_threshold() >= self.max_distance_cm:
                raise ValueError("Robot dimensions exceed sensor range")

    def forward_threshold(self):
        nose = 0.0 if self.robot_length_cm is None else self.robot_length_cm / 2 - self.sensor_forward_offset_cm
        return max(self.safe_distance_cm, nose + self.clearance_margin_cm)

    def rotation_threshold(self, angle):
        radius = max(math.hypot(self.robot_width_cm, self.robot_length_cm) / 2, abs(self.sensor_forward_offset_cm)) + self.clearance_margin_cm
        theta = math.radians(90 - angle)
        offset = self.sensor_forward_offset_cm
        return -offset * math.cos(theta) + math.sqrt(radius * radius - (offset * math.sin(theta)) ** 2)


def filtered_distance(samples, config):
    valid = sorted(x for x in samples if isinstance(x, (int, float)) and math.isfinite(x) and 0 < x <= config.max_distance_cm)
    if len(valid) < config.min_valid_samples:
        return None
    return statistics.median(valid)


def rotation_clear(scan, config):
    return all(scan.get(angle) is not None and scan[angle] > config.rotation_threshold(angle) for angle in (0, 45, 90, 135, 180))


def choose_turn(scan, config):
    if not rotation_clear(scan, config):
        return None, {"reason": "rotation_clearance_or_measurement"}
    threshold = max(config.forward_threshold(), config.robot_width_cm / 2 + config.clearance_margin_cm)
    scores = {"left": min(scan[135], scan[180]), "right": min(scan[0], scan[45])}
    allowed = {side: score for side, score in scores.items() if score >= threshold}
    if not allowed:
        return None, {"reason": "both_sides_blocked", "scores_cm": scores, "threshold_cm": threshold}
    side = max(allowed, key=allowed.get)
    angle = -config.turn_angle_deg if side == "left" else config.turn_angle_deg
    return angle, {"reason": "largest_minimum_clearance", "side": side, "scores_cm": scores, "threshold_cm": threshold}


def return_angles(heading, config):
    spread = math.degrees(math.atan2(config.robot_width_cm / 2 + config.clearance_margin_cm, config.resume_distance_cm))
    angles = tuple(round(90 + heading + delta) for delta in (-spread, 0, spread))
    return angles if all(0 <= angle <= 180 for angle in angles) else None


class Navigator:
    def __init__(self, config, hardware, emit):
        self.config = config
        self.hardware = hardware
        self.emit = emit
        self.heading = 0.0
        self.bypass_steps = 0
        self.clear_checks = 0
        self.state = None

    def transition(self, state, **details):
        if state != self.state:
            self.emit("state", previous=self.state, state=state, heading_estimate_deg=self.heading, **details)
            self.state = state

    def halt(self, reason):
        self.hardware.brake(reason)
        self.transition("stopped", reason=reason)
        return False

    def tick(self):
        if self.hardware.ir_blocked():
            return self.halt("ir_obstacle")
        front = self.hardware.measure(90)
        self.emit("front_check", distance_cm=front, threshold_cm=self.config.forward_threshold())
        if front is None:
            return self.halt("invalid_front_measurement")
        if self.heading == 0:
            if front >= self.config.forward_threshold():
                self.transition("forward")
                self.hardware.forward(self.config.step_seconds)
                return True
            self.transition("scanning")
            scan = self.hardware.scan((0, 45, 90, 135, 180))
            if scan.get(90) is None:
                return self.halt("invalid_scan_front")
            if scan[90] >= self.config.forward_threshold():
                self.emit("turn_decision", angle_deg=None, reason="front_clear_on_rescan", scan_cm=scan)
                return True
            angle, details = choose_turn(scan, self.config)
            self.emit("turn_decision", angle_deg=angle, scan_cm=scan, **details)
            if angle is None:
                return self.halt(details["reason"])
            self.transition("turning_out", angle_deg=angle)
            self.hardware.turn(angle)
            self.heading = angle
            self.bypass_steps = 0
            self.clear_checks = 0
            return True
        self.transition("bypassing")
        if front < self.config.forward_threshold():
            return self.halt("bypass_front_blocked")
        if self.bypass_steps >= self.config.min_bypass_steps:
            angles = return_angles(self.heading, self.config)
            if angles is None:
                return self.halt("return_direction_outside_servo_range")
            corridor = self.hardware.scan(angles)
            clear = all(value is not None and value >= self.config.resume_distance_cm for value in corridor.values())
            self.clear_checks = self.clear_checks + 1 if clear else 0
            self.emit("return_corridor", scan_cm=corridor, clear=clear, consecutive_clear=self.clear_checks, bypass_steps=self.bypass_steps)
            if self.clear_checks >= self.config.clear_checks_to_resume:
                scan = self.hardware.scan((0, 45, 90, 135, 180))
                if not rotation_clear(scan, self.config):
                    return self.halt("return_rotation_blocked")
                self.transition("returning", angle_deg=-self.heading)
                self.hardware.turn(-self.heading)
                self.heading = 0.0
                self.bypass_steps = 0
                self.clear_checks = 0
                self.transition("forward")
                return True
        if self.bypass_steps >= self.config.max_bypass_steps:
            return self.halt("bypass_step_limit")
        front = self.hardware.measure(90)
        if front is None or front < self.config.forward_threshold():
            return self.halt("front_changed_after_scan")
        self.hardware.forward(self.config.step_seconds)
        self.bypass_steps += 1
        self.emit("bypass_progress", steps=self.bypass_steps, heading_estimate_deg=self.heading)
        return True
