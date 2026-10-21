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
    left_drive_trim_pwm: float = 0.0
    right_drive_trim_pwm: float = 0.0
    turn_speed: float = 10.0
    step_seconds: float = 0.1
    turn_angle_deg: float = 45.0
    max_escape_angle_deg: float = 90.0
    turn_right_deg_per_sec: float = 60.0
    turn_left_deg_per_sec: float = 60.0
    min_bypass_steps: int = 4
    return_probe_steps: int = 8
    clear_checks_to_resume: int = 2
    max_bypass_steps: int = 30
    samples: int = 5
    min_valid_samples: int = 3
    measurement_retries: int = 1
    sample_interval_seconds: float = 0.06
    echo_timeout_seconds: float = 0.03
    max_distance_cm: float = 400.0
    servo_min_deg: int = 45
    servo_max_deg: int = 135
    servo_step_deg: int = 3
    servo_step_seconds: float = 0.02
    servo_settle_seconds: float = 0.3
    ir_active_level: int = 0

    def validate(self, require_geometry=False):
        values = asdict(self)
        dimensions = {"robot_width_cm", "robot_length_cm", "sensor_forward_offset_cm"}
        integers = {"samples", "min_valid_samples", "measurement_retries", "min_bypass_steps", "return_probe_steps", "max_bypass_steps", "clear_checks_to_resume", "servo_step_deg", "servo_min_deg", "servo_max_deg", "ir_active_level"}
        for name, value in values.items():
            if name in dimensions and value is None:
                continue
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f"Invalid numeric configuration: {name}")
            if name in integers and not isinstance(value, int):
                raise ValueError(f"Configuration must be an integer: {name}")
            if name not in {"sensor_forward_offset_cm", "ir_active_level", "measurement_retries", "left_drive_trim_pwm", "right_drive_trim_pwm", "servo_min_deg"} and value <= 0:
                raise ValueError(f"Configuration must be positive: {name}")
        if not 0 < self.move_speed <= 100 or not 0 < self.turn_speed <= 100:
            raise ValueError("PWM speed must be between 0 and 100")
        for trim in (self.left_drive_trim_pwm, self.right_drive_trim_pwm):
            if abs(trim) > 10 or not 0 < self.move_speed + trim <= 100:
                raise ValueError("Drive trim must be within -10..10 and leave PWM within 0..100")
        if not 0 <= self.measurement_retries <= 2:
            raise ValueError("Measurement retries must be between 0 and 2")
        if not 0 <= self.servo_min_deg < 90 < self.servo_max_deg <= 180:
            raise ValueError("Servo limits must satisfy 0 <= min < 90 < max <= 180")
        if not 45 <= self.max_escape_angle_deg <= 90:
            raise ValueError("Maximum escape heading must be between 45 and 90 degrees")
        if not 10 <= self.turn_angle_deg <= 60:
            raise ValueError("Turn angle must be between 10 and 60 degrees")
        if not 3 <= self.min_valid_samples <= self.samples <= 15:
            raise ValueError("Require at least 3 valid samples, at most 15 samples")
        if self.ir_active_level not in (0, 1):
            raise ValueError("IR active level must be 0 or 1")
        if self.min_bypass_steps >= self.max_bypass_steps:
            raise ValueError("Bypass minimum must be less than bypass limit")
        if self.return_probe_steps < self.min_bypass_steps:
            raise ValueError("Return probe interval must be at least the bypass observation interval")
        if self.step_seconds > 0.5 or self.servo_step_deg > 15:
            raise ValueError("Motion step or servo step is too large for this test version")
        if self.resume_distance_cm < self.safe_distance_cm:
            raise ValueError("Resume distance must be at least the stopping threshold")
        if require_geometry and any(values[name] is None for name in dimensions):
            raise ValueError("Measure and configure robot_width_cm, robot_length_cm, sensor_forward_offset_cm before auto mode")
        if all(values[name] is not None for name in dimensions):
            if abs(self.sensor_forward_offset_cm) > self.robot_length_cm / 2:
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


def scan_valid(scan, config):
    required = scan_angles(config)
    return all(angle in scan for angle in required) and all(isinstance(angle, (int, float)) and math.isfinite(angle) and config.servo_min_deg <= angle <= config.servo_max_deg and not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value) and 0 < value <= config.max_distance_cm for angle, value in scan.items())


def rotation_clearance(scan, config):
    if not scan_valid(scan, config):
        return False, {"reason": "unknown_rotation_measurement"}
    blocked = [{"angle_deg": angle, "distance_cm": value, "threshold_cm": round(config.rotation_threshold(angle), 3)} for angle, value in scan.items() if value <= config.rotation_threshold(angle)]
    return not blocked, {"reason": "rotation_clear" if not blocked else "rotation_space_blocked", "blocking_rays": blocked}


def rotation_clear(scan, config):
    return rotation_clearance(scan, config)[0]


def maneuver_angle(config):
    return math.floor(min(config.turn_angle_deg, 90 - config.servo_min_deg, config.servo_max_deg - 90))


def scan_angles(config):
    values = [config.servo_min_deg + (90 - config.servo_min_deg) * fraction for fraction in (0, 0.3, 0.6, 0.8, 1)]
    values += [90 + (config.servo_max_deg - 90) * fraction for fraction in (0.2, 0.4, 0.7, 1)]
    return tuple(sorted(set(round(value) for value in values)))


def route_clearance(scan, config, distance=None):
    distance = config.forward_threshold() if distance is None else distance
    half_width = config.robot_width_cm / 2 + config.clearance_margin_cm
    limit = config.sensor_forward_offset_cm + distance
    if not scan_valid(scan, config):
        return False, {"reason": "unknown_corridor_measurement", "distance_required_cm": distance}
    blocked = []
    for angle, value in scan.items():
        theta = math.radians(90 - angle)
        x = config.sensor_forward_offset_cm + value * math.cos(theta)
        y = value * math.sin(theta)
        if -config.robot_length_cm / 2 - config.clearance_margin_cm <= x <= limit and abs(y) <= half_width:
            blocked.append({"angle_deg": angle, "distance_cm": value, "forward_from_centre_cm": round(x, 3), "side_from_centre_cm": round(y, 3)})
    clear = scan[90] > distance and not blocked
    return clear, {"reason": "corridor_clear" if clear else "robot_width_corridor_blocked", "distance_required_cm": distance, "half_width_with_margin_cm": half_width, "blocking_rays": blocked}


def choose_turn(scan, config):
    clear, rotation = rotation_clearance(scan, config)
    if not clear:
        return None, {"reason": "rotation_clearance_or_measurement", "rotation": rotation}
    angle = maneuver_angle(config)
    if angle < 10:
        return None, {"reason": "servo_view_too_narrow_for_probe"}
    scores = {"right": min(value for bearing, value in scan.items() if bearing <= 90 - angle), "left": min(value for bearing, value in scan.items() if bearing >= 90 + angle)}
    threshold = config.forward_threshold()
    allowed = {side: score for side, score in scores.items() if score > threshold}
    if not allowed:
        return None, {"reason": "both_escape_directions_blocked", "scores_cm": scores, "threshold_cm": threshold}
    best = max(allowed.values())
    side = "right" if "right" in allowed and allowed["right"] >= best - config.clearance_margin_cm else max(allowed, key=allowed.get)
    turn = angle if side == "right" else -angle
    return turn, {"reason": "escape_probe_then_rescan", "side": side, "scores_cm": scores, "threshold_cm": threshold, "actual_turn_deg": angle, "translation_authorized": False}


def navigation_plan(scan, config):
    front = scan.get(90)
    if isinstance(front, bool) or not isinstance(front, (int, float)) or not math.isfinite(front) or not 0 < front <= config.max_distance_cm:
        return {"action": "stop", "reason": "invalid_front_measurement", "angle_deg": None}
    if front > config.forward_threshold():
        return {"action": "forward", "reason": "front_clear", "angle_deg": None}
    angle, details = choose_turn(scan, config)
    return {"action": "turn" if angle is not None else "stop", "angle_deg": angle, **details}


class Navigator:
    def __init__(self, config, hardware, emit):
        self.config = config
        self.hardware = hardware
        self.emit = emit
        self.heading = 0.0
        self.bypass_steps = 0
        self.steps_since_scan = 0
        self.steps_since_probe = 0
        self.probe_interval = config.return_probe_steps
        self.clear_checks = 0
        self.state = None
        self.phase = "cruise"
        self.escape_sign = 0
        self.bypass_heading = 0.0

    def transition(self, state, **details):
        if state != self.state:
            self.emit("state", previous=self.state, state=state, heading_estimate_deg=self.heading, phase=self.phase, **details)
            self.state = state

    def halt(self, reason):
        self.hardware.brake(reason)
        self.phase = "stopped"
        self.transition("stopped", reason=reason)
        return False

    def scan(self):
        self.hardware.brake("bypass_observation")
        return self.hardware.scan(scan_angles(self.config))

    def guarded_turn(self, angle, scan, state):
        clear, details = rotation_clearance(scan, self.config)
        self.emit("rotation_check", clear=clear, angle_command_deg=angle, heading_estimate_deg=self.heading, phase=self.phase, **details)
        if not clear:
            return self.halt("rotation_blocked_during_" + self.phase)
        self.hardware.brake("before_turn")
        self.transition(state, angle_deg=angle)
        self.hardware.turn(angle)
        self.heading = round(self.heading + angle, 6)
        self.emit("heading_updated", heading_estimate_deg=self.heading, angle_command_deg=angle, phase=self.phase)
        return True

    def advance_bypass(self):
        if self.bypass_steps >= self.config.max_bypass_steps:
            return self.halt("bypass_step_limit")
        front = self.hardware.front_distance()
        self.emit("front_check", distance_cm=front, threshold_cm=self.config.forward_threshold(), phase=self.phase)
        if front is None:
            return self.halt("invalid_front_measurement")
        if front <= self.config.forward_threshold():
            self.hardware.brake("bypass_front_obstacle")
            self.phase = "probe_return"
            self.bypass_heading = self.heading
            self.clear_checks = 0
            self.steps_since_scan = 0
            self.emit("return_requested", reason="bypass_front_obstacle", heading_estimate_deg=self.heading)
            return True
        self.transition("bypassing")
        self.hardware.advance(self.config.step_seconds)
        self.bypass_steps += 1
        self.steps_since_scan += 1
        self.steps_since_probe += 1
        self.emit("bypass_progress", steps=self.bypass_steps, steps_since_scan=self.steps_since_scan, heading_estimate_deg=self.heading)
        return True

    def tick(self):
        if self.hardware.ir_blocked():
            return self.halt("ir_obstacle")
        if self.phase == "stopped":
            return False
        if self.phase == "cruise":
            front = self.hardware.front_distance()
            self.emit("front_check", distance_cm=front, threshold_cm=self.config.forward_threshold(), phase=self.phase)
            if front is None:
                return self.halt("invalid_front_measurement")
            if front > self.config.forward_threshold():
                self.transition("forward")
                self.hardware.advance(self.config.step_seconds)
                return True
            self.hardware.brake("front_obstacle")
            self.transition("scanning")
            scan = self.scan()
            if not scan_valid(scan, self.config):
                return self.halt("invalid_escape_scan")
            if scan[90] > self.config.forward_threshold():
                self.emit("turn_decision", angle_deg=None, reason="front_clear_on_rescan", scan_cm=scan)
                return True
            angle, details = choose_turn(scan, self.config)
            self.emit("turn_decision", angle_deg=angle, scan_cm=scan, **details)
            if angle is None:
                return self.halt(details["reason"])
            self.escape_sign = 1 if angle > 0 else -1
            self.phase = "escape"
            self.bypass_steps = 0
            self.steps_since_scan = 0
            self.steps_since_probe = 0
            self.probe_interval = self.config.return_probe_steps
            self.clear_checks = 0
            return self.guarded_turn(angle, scan, "turning_out")
        if self.phase == "bypass" and self.steps_since_scan < self.config.min_bypass_steps:
            return self.advance_bypass()
        scan = self.scan()
        if not scan_valid(scan, self.config):
            return self.halt("invalid_bypass_scan")
        if self.phase == "escape":
            clear, details = route_clearance(scan, self.config)
            self.emit("escape_corridor", heading_estimate_deg=self.heading, scan_cm=scan, clear=clear, **details)
            if abs(self.heading) < 90:
                if abs(self.heading) >= self.config.max_escape_angle_deg:
                    return self.halt("escape_heading_limit_does_not_allow_lateral_bypass")
                angle = self.escape_sign * min(maneuver_angle(self.config), self.config.max_escape_angle_deg - abs(self.heading))
                if abs(angle) < 1:
                    return self.halt("escape_turn_limit")
                return self.guarded_turn(angle, scan, "turning_further_out")
            if not clear:
                return self.halt("escape_route_blocked")
            self.phase = "bypass"
            self.bypass_heading = self.heading
            self.steps_since_scan = 0
            return self.advance_bypass()
        if self.phase == "bypass":
            if self.bypass_steps >= self.config.max_bypass_steps:
                return self.halt("bypass_step_limit")
            if self.steps_since_probe < self.probe_interval:
                clear, details = route_clearance(scan, self.config)
                self.emit("bypass_corridor", scan_cm=scan, clear=clear, steps_since_probe=self.steps_since_probe, **details)
                if clear:
                    self.steps_since_scan = 0
                    return self.advance_bypass()
                self.emit("return_requested", reason="bypass_corridor_blocked", heading_estimate_deg=self.heading)
            self.phase = "probe_return"
            self.bypass_heading = self.heading
            self.clear_checks = 0
            angle = -math.copysign(min(maneuver_angle(self.config), abs(self.heading)), self.heading)
            return self.guarded_turn(angle, scan, "probing_original_course")
        if self.phase == "probe_return":
            if abs(self.heading) > 0.01:
                angle = -math.copysign(min(maneuver_angle(self.config), abs(self.heading)), self.heading)
                return self.guarded_turn(angle, scan, "probing_original_course")
            clear, details = route_clearance(scan, self.config, self.config.resume_distance_cm)
            self.clear_checks = self.clear_checks + 1 if clear else 0
            self.emit("return_corridor", scan_cm=scan, clear=clear, consecutive_clear=self.clear_checks, bypass_steps=self.bypass_steps, **details)
            if self.clear_checks >= self.config.clear_checks_to_resume:
                self.phase = "cruise"
                self.heading = 0.0
                self.bypass_steps = 0
                self.steps_since_scan = 0
                self.steps_since_probe = 0
                self.transition("forward", reason="original_course_corridor_confirmed")
                self.emit("bypass_complete", heading_estimate_deg=0.0)
                return True
            if clear:
                return True
            self.phase = "restore_bypass"
            self.probe_interval = self.config.min_bypass_steps
        if self.phase == "restore_bypass":
            delta = self.bypass_heading - self.heading
            if abs(delta) > 0.01:
                angle = math.copysign(min(maneuver_angle(self.config), abs(delta)), delta)
                return self.guarded_turn(angle, scan, "restoring_bypass_course")
            clear, details = route_clearance(scan, self.config)
            self.emit("escape_corridor", heading_estimate_deg=self.heading, scan_cm=scan, clear=clear, **details)
            if not clear:
                self.phase = "escape"
                return True
            self.phase = "bypass"
            self.steps_since_scan = 0
            self.steps_since_probe = 0
            return self.advance_bypass()
        return self.halt("unknown_navigation_phase")
