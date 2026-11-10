import math
import statistics
from dataclasses import asdict, dataclass, field

from trajectory import PathMemory, TrajectoryReturn


class ObstacleDetected(InterruptedError):
    def __init__(self, reason, heading_uncertain=False):
        super().__init__(reason)
        self.heading_uncertain = heading_uncertain


@dataclass
class Config:
    safe_distance_cm: float = 10.0
    resume_distance_cm: float = 30.0
    clearance_margin_cm: float = 5.0
    robot_width_cm: float = 18.5
    robot_length_cm: float = 24.0
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
    bypass_segment_seconds: float = 1.5
    max_return_attempts: int = 3
    retry_pause_seconds: float = 1.0
    min_return_progress_seconds: float = 0.3
    bypass_observation_seconds: float = 0.4
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

    ir_classification_pins: list = field(default_factory=lambda: [12, 17])
    ir_obstacle_patterns: dict = field(default_factory=dict)
    ir_background_pattern: list = None
    ir_min_distance_cm: float = None
    ir_max_distance_cm: float = None
    ir_confirm_samples: int = 3
    push_speed: float = 10.0
    push_seconds: float = 0.3

    return_probe_seconds: float = 1.5
    return_tolerance_seconds: float = 0.03
    drive_speed_cm_s: float = None
    range_reference_heading_deg: float = None

    def validate(self, require_geometry=False):
        values = asdict(self)
        ir_fields = {"ir_classification_pins", "ir_obstacle_patterns", "ir_background_pattern", "ir_min_distance_cm", "ir_max_distance_cm", "drive_speed_cm_s", "range_reference_heading_deg"}
        pins = self.ir_classification_pins
        available = {2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 14, 15, 17, 18, 25}
        if not isinstance(pins, (list, tuple)) or not 1 <= len(pins) <= 8 or any(type(pin) is not int or pin not in available for pin in pins) or len(set(pins)) != len(pins) or not set(pins) - {3}:
            raise ValueError("Choose distinct unreserved IR inputs including an input other than BCM 3")
        if type(self.ir_confirm_samples) is not int or not 3 <= self.ir_confirm_samples <= 20:
            raise ValueError("IR confirmation requires 3..20 samples")
        if not isinstance(self.ir_obstacle_patterns, dict) or set(self.ir_obstacle_patterns) - {"dangerous", "permitted"}:
            raise ValueError("IR patterns must map dangerous/permitted to input levels")
        patterns = list(self.ir_obstacle_patterns.values())
        if patterns:
            patterns.append(self.ir_background_pattern)
        for pattern in patterns:
            if not isinstance(pattern, (list, tuple)) or len(pattern) != len(pins) or any(type(level) is not int or level not in (0, 1) for level in pattern):
                raise ValueError("Each IR pattern and background must specify 0/1 for every configured pin")
        signatures = [tuple(level for pin, level in zip(pins, pattern) if pin != 3) for pattern in patterns]
        if len(set(signatures)) != len(signatures):
            raise ValueError("IR materials and background must differ on inputs other than BCM 3")
        dimensions = {"robot_width_cm", "robot_length_cm", "sensor_forward_offset_cm"}
        integers = {"samples", "min_valid_samples", "measurement_retries", "min_bypass_steps", "return_probe_steps", "max_bypass_steps", "clear_checks_to_resume", "max_return_attempts", "servo_step_deg", "servo_min_deg", "servo_max_deg", "ir_active_level"}
        for name, value in values.items():
            if name in ir_fields:
                continue
            if name in dimensions and value is None:
                continue
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f"Invalid numeric configuration: {name}")
            if name in integers and not isinstance(value, int):
                raise ValueError(f"Configuration must be an integer: {name}")
            if name not in {"sensor_forward_offset_cm", "ir_active_level", "measurement_retries", "left_drive_trim_pwm", "right_drive_trim_pwm", "servo_min_deg"} and value <= 0:
                raise ValueError(f"Configuration must be positive: {name}")
        if not 0.1 <= self.return_probe_seconds <= 10 or not 0 < self.return_tolerance_seconds <= 0.1:
            raise ValueError("Return probe must be 0.1..10 seconds; tolerance must be 0..0.1 seconds")
        if self.drive_speed_cm_s is not None and (isinstance(self.drive_speed_cm_s, bool) or not isinstance(self.drive_speed_cm_s, (int, float)) or not math.isfinite(self.drive_speed_cm_s) or not 0 < self.drive_speed_cm_s <= 100):
            raise ValueError("Measured drive speed must be 0..100 cm/s")
        reference = self.range_reference_heading_deg
        if reference is not None and (isinstance(reference, bool) or not isinstance(reference, (int, float)) or not math.isfinite(reference) or not -90 <= reference <= 90 or self.drive_speed_cm_s is None):
            raise ValueError("A fixed wall reference requires a measured drive speed and heading within -90..90")
        bounds = (self.ir_min_distance_cm, self.ir_max_distance_cm)
        if patterns or any(value is not None for value in bounds):
            if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) for value in bounds) or not 0 < bounds[0] <= bounds[1] <= self.max_distance_cm:
                raise ValueError("Set a measured IR working distance range")
        if not 0 < self.push_seconds <= 0.3 or not 0 < self.push_speed <= min(10, self.move_speed):
            raise ValueError("Push must last at most 0.3 seconds at PWM no higher than 10 or move_speed")
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
        if not 0.1 <= self.bypass_segment_seconds <= 10:
            raise ValueError("Bypass segment must be between 0.1 and 10 seconds")
        if not 1 <= self.max_return_attempts <= 10:
            raise ValueError("Return attempts must be between 1 and 10")
        if not 0.1 <= self.retry_pause_seconds <= 10:
            raise ValueError("Retry pause must be between 0.1 and 10 seconds")
        if not 0.1 <= self.min_return_progress_seconds <= 10:
            raise ValueError("Return progress interval must be between 0.1 and 10 seconds")
        if not 0.1 <= self.bypass_observation_seconds <= 1:
            raise ValueError("Bypass observation interval must be between 0.1 and 1 second")
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
        nose = 0.0 if self.robot_length_cm is None or self.sensor_forward_offset_cm is None else self.robot_length_cm / 2 - self.sensor_forward_offset_cm
        return max(self.safe_distance_cm, nose + self.clearance_margin_cm)

    def rotation_threshold(self, angle):
        radius = max(math.hypot(self.robot_width_cm, self.robot_length_cm) / 2, abs(self.sensor_forward_offset_cm)) + self.clearance_margin_cm
        theta = math.radians(90 - angle)
        offset = self.sensor_forward_offset_cm
        return -offset * math.cos(theta) + math.sqrt(radius * radius - (offset * math.sin(theta)) ** 2)


def classify_infrared(config, readings, distances):
    result = {"label": "unknown", "stable_samples": 0, "reason": "not_calibrated"}
    if not config.ir_obstacle_patterns:
        return result
    if len(readings) < config.ir_confirm_samples or len(readings) != len(distances):
        return {**result, "reason": "insufficient_samples"}
    labels = []
    for levels, distance in zip(readings, distances):
        if isinstance(distance, bool) or not isinstance(distance, (int, float)) or not math.isfinite(distance) or not config.ir_min_distance_cm <= distance <= config.ir_max_distance_cm:
            return {**result, "reason": "outside_working_range_or_invalid_distance"}
        if len(levels) != len(config.ir_classification_pins) or any(type(value) is not int or value not in (0, 1) for value in levels):
            return {**result, "reason": "invalid_levels"}
        matches = [label for label, pattern in config.ir_obstacle_patterns.items() if all(value == expected for pin, value, expected in zip(config.ir_classification_pins, levels, pattern) if pin != 3)]
        labels.append(matches[0] if len(matches) == 1 else "unknown")
    if "unknown" in labels or len(set(labels)) != 1:
        return {**result, "reason": "unknown_or_conflicting_signals"}
    return {"label": labels[0], "stable_samples": len(labels), "reason": "confirmed"}


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
    values = [config.servo_min_deg + (90 - config.servo_min_deg) * fraction for fraction in (0, 0.5, 1)]
    values += [90 + (config.servo_max_deg - 90) * fraction for fraction in (0.5, 1)]
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


def choose_turn(scan, config, excluded_sides=()):
    clear, rotation = rotation_clearance(scan, config)
    if not clear:
        return None, {"reason": "rotation_clearance_or_measurement", "rotation": rotation}
    angle = maneuver_angle(config)
    if angle < 10:
        return None, {"reason": "servo_view_too_narrow_for_probe"}
    scores = {"right": min(value for bearing, value in scan.items() if bearing <= 90 - angle), "left": min(value for bearing, value in scan.items() if bearing >= 90 + angle)}
    threshold = config.forward_threshold()
    allowed = {side: score for side, score in scores.items() if score > threshold and side not in excluded_sides}
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
    def __init__(self, config, hardware, emit, obstacle_mode="ir"):
        self.config = config
        self.hardware = hardware
        self.emit = emit
        self.heading = 0.0
        self.heading_known = True
        self.bypass_steps = 0
        self.return_attempts = 0
        self.segment_number = 0
        self.segment_started_runtime = None
        self.corridor_observed_runtime = None
        self.segment_drive_seconds = 0.0
        self.bypass_drive_seconds = 0.0
        self.state = None
        self.phase = "cruise"
        self.escape_sign = 0
        self.bypass_heading = 0.0
        self.last_front_distance = None
        self.blocked_routes = {}
        self.clear_counts = {}
        self.waiting_reason = None
        self.waiting_reference = None
        self.waiting_candidate = None
        self.waiting_heading = None
        self.waiting_runtime = None
        self.push_attempted = False
        self.obstacle_mode = obstacle_mode
        self.path = PathMemory(config, emit)
        self.trajectory = TrajectoryReturn(self)

    def motion_details(self):
        return {
            "phase": self.phase,
            "side": "right" if self.escape_sign > 0 else "left" if self.escape_sign < 0 else None,
            "heading_estimate_deg": self.heading,
            "heading_known": self.heading_known,
            "segment_number": self.segment_number,
            "segment_drive_seconds": round(self.segment_drive_seconds, 6),
            "return_attempt": self.return_attempts,
            "front_distance_cm": self.last_front_distance,
        }

    def transition(self, state, **details):
        if state != self.state:
            self.emit("state", previous=self.state, state=state, **self.motion_details(), **details)
            self.state = state

    def update_progress(self):
        self.path.update(self.hardware.forward_runtime_seconds(), self.heading)
        if self.segment_started_runtime is not None:
            elapsed = max(0.0, self.hardware.forward_runtime_seconds() - self.segment_started_runtime)
            self.bypass_drive_seconds += max(0.0, elapsed - self.segment_drive_seconds)
            self.segment_drive_seconds = elapsed

    def refresh_routes(self):
        runtime = self.hardware.forward_runtime_seconds()
        reopened = [heading for heading, started in self.blocked_routes.items() if runtime - started >= self.config.min_return_progress_seconds - 1e-6]
        for heading in reopened:
            del self.blocked_routes[heading]
            self.clear_counts.pop(heading, None)
        if reopened:
            self.emit("route_memory_updated", reason="new_forward_progress", reopened_headings_deg=reopened, **self.motion_details())

    def block_route(self, heading, reason):
        self.refresh_routes()
        self.blocked_routes[heading] = self.hardware.forward_runtime_seconds()
        self.clear_counts[heading] = 0
        self.emit("route_blocked", route_heading_deg=heading, reason=reason, **self.motion_details())

    def observe_route(self, scan):
        if self.heading not in self.blocked_routes:
            return
        distance = self.config.resume_distance_cm if abs(self.heading) < 0.01 else self.config.forward_threshold()
        clear, _ = route_clearance(scan, self.config, distance + self.config.clearance_margin_cm)
        self.clear_counts[self.heading] = self.clear_counts.get(self.heading, 0) + 1 if clear else 0
        if self.clear_counts[self.heading] >= 2:
            del self.blocked_routes[self.heading]
            self.clear_counts.pop(self.heading, None)
            self.emit("route_memory_updated", reason="stable_clear_corridor", reopened_headings_deg=[self.heading], **self.motion_details())

    def halt(self, reason):
        self.hardware.brake(reason)
        self.update_progress()
        self.phase = "stopped"
        self.transition("stopped", reason=reason)
        return False

    def wait_for_path(self, reason, scan=None):
        self.hardware.brake(reason)
        self.update_progress()
        runtime = self.hardware.forward_runtime_seconds()
        if self.waiting_heading != self.heading or self.waiting_runtime is None or runtime - self.waiting_runtime >= self.config.min_return_progress_seconds:
            self.waiting_reference = dict(scan) if scan is not None and scan_valid(scan, self.config) else None
            self.waiting_candidate = None
            self.waiting_heading = self.heading
            self.waiting_runtime = runtime
        self.waiting_reason = reason
        self.phase = "waiting"
        self.transition("waiting", reason=reason)
        self.emit("navigation_wait", reason=reason, blocked_headings_deg=sorted(self.blocked_routes), **self.motion_details())
        return True

    def recover_obstacle(self, reason, heading_uncertain=False):
        previous_phase = self.phase
        self.heading_known = self.heading_known and not heading_uncertain
        self.hardware.brake(reason)
        self.update_progress()
        if previous_phase.startswith("trajectory_"):
            return self.trajectory.wait(previous_phase, "heading_unknown" if not self.heading_known else reason)
        if self.heading_known and self.phase == "bypass":
            self.block_route(self.heading, reason)
        return self.wait_for_path("heading_unknown" if not self.heading_known else reason)

    def scan(self):
        self.hardware.brake("bypass_observation")
        self.update_progress()
        result = self.hardware.scan(scan_angles(self.config))
        self.last_front_distance = result.get(90)
        self.refresh_routes()
        if scan_valid(result, self.config):
            self.path.observe_scan(result, self.hardware.forward_runtime_seconds(), self.heading)
            self.observe_route(result)
        else:
            self.path.range_anchor = None
        return result

    def guarded_turn(self, angle, scan, state):
        clear, details = rotation_clearance(scan, self.config)
        self.emit("rotation_check", clear=clear, angle_command_deg=angle, **self.motion_details(), **details)
        if not clear:
            return self.wait_for_path("rotation_space_blocked", scan)
        self.hardware.brake("before_turn")
        self.transition(state, angle_deg=angle)
        self.update_progress()
        previous_heading = self.heading
        self.hardware.turn(angle)
        self.heading = round(self.heading + angle, 6)
        self.path.record_turn(previous_heading, self.heading)
        self.emit("heading_updated", angle_command_deg=angle, **self.motion_details())
        return True

    def turn_towards(self, heading, scan, state):
        delta = heading - self.heading
        angle = math.copysign(min(maneuver_angle(self.config), abs(delta)), delta)
        if abs(angle) < 1:
            return self.wait_for_path("servo_view_too_narrow_for_probe", scan)
        return self.guarded_turn(angle, scan, state)

    def resume_course(self):
        self.update_progress()
        if self.path.active:
            return self.trajectory.start()
        if self.bypass_drive_seconds >= self.config.min_return_progress_seconds:
            self.push_attempted = False
        self.phase = "cruise"
        self.segment_started_runtime = None
        self.transition("forward", reason="original_course_corridor_confirmed")
        self.emit("bypass_complete", bypass_drive_seconds=round(self.bypass_drive_seconds, 6), **self.motion_details())
        return True

    def select_side(self, heading, scan):
        if self.config.max_escape_angle_deg < 90:
            return self.wait_for_path("escape_heading_limit_does_not_allow_lateral_bypass", scan)
        self.bypass_heading = heading
        self.escape_sign = 1 if heading > 0 else -1
        self.phase = "escape"
        self.emit("route_selected", target_heading_deg=heading, **self.motion_details())
        if abs(heading - self.heading) > 0.01:
            return self.turn_towards(heading, scan, "turning_out")
        return self.align_side(scan)

    def plan_route(self, scan):
        self.refresh_routes()
        if not scan_valid(scan, self.config):
            return self.wait_for_path("invalid_navigation_scan")
        if abs(self.heading) < 0.01:
            clear, details = route_clearance(scan, self.config, self.config.resume_distance_cm)
            if clear and 0 not in self.blocked_routes:
                return self.resume_course()
            if not clear:
                self.block_route(0, details["reason"])
            if self.bypass_heading in (-90, 90) and self.bypass_heading not in self.blocked_routes:
                return self.select_side(self.bypass_heading, scan)
            excluded = ["right" if heading > 0 else "left" for heading in self.blocked_routes if heading != 0]
            angle, details = choose_turn(scan, self.config, excluded)
            self.emit("turn_decision", angle_deg=angle, scan_cm=scan, **details)
            if angle is None:
                for side, score in details.get("scores_cm", {}).items():
                    if score <= self.config.forward_threshold():
                        self.block_route(90 if side == "right" else -90, "escape_direction_unavailable")
                return self.wait_for_path(details["reason"], scan)
            return self.select_side(90 if angle > 0 else -90, scan)
        if 0 not in self.blocked_routes:
            self.phase = "probe_return"
            return self.turn_towards(0, scan, "probing_original_course")
        candidates = [self.bypass_heading, 90, -90]
        for heading in dict.fromkeys(candidates):
            if heading not in (-90, 90) or heading in self.blocked_routes:
                continue
            if abs(heading - self.heading) < 0.01:
                clear, details = route_clearance(scan, self.config)
                if not clear:
                    self.block_route(heading, details["reason"])
                    continue
            return self.select_side(heading, scan)
        self.phase = "recenter"
        return self.turn_towards(0, scan, "returning_to_observation")

    def align_side(self, scan):
        target = self.bypass_heading or self.escape_sign * 90
        if target not in (-90, 90):
            return self.halt("invalid_bypass_heading")
        if abs(target - self.heading) > 0.01:
            return self.turn_towards(target, scan, "turning_further_out")
        clear, details = route_clearance(scan, self.config)
        self.emit("escape_corridor", scan_cm=scan, clear=clear, **self.motion_details(), **details)
        if not clear:
            self.block_route(self.heading, details["reason"])
            self.phase = "replan"
            return self.plan_route(scan)
        if self.heading in self.blocked_routes:
            self.phase = "replan"
            return self.plan_route(scan)
        return self.begin_bypass()

    def begin_bypass(self):
        self.phase = "bypass"
        self.bypass_heading = self.heading
        self.segment_number += 1
        self.segment_started_runtime = self.hardware.forward_runtime_seconds()
        self.corridor_observed_runtime = self.segment_started_runtime
        self.segment_drive_seconds = 0.0
        self.emit("bypass_segment_start", target_seconds=self.config.bypass_segment_seconds, **self.motion_details())
        return self.advance_bypass()

    def request_return(self, reason):
        self.hardware.brake(reason)
        self.update_progress()
        self.refresh_routes()
        if 0 in self.blocked_routes:
            self.phase = "replan"
            self.emit("return_deferred", reason="original_course_already_blocked_at_this_position", **self.motion_details())
            return True
        self.phase = "probe_return"
        self.return_attempts += 1
        self.emit("return_requested", reason=reason, **self.motion_details())
        return True

    def finish_bypass_segment(self):
        return self.request_return("bypass_segment_complete")

    def corridor_observation_due(self):
        baseline = self.corridor_observed_runtime if self.corridor_observed_runtime is not None else self.segment_started_runtime
        return baseline is not None and self.hardware.forward_runtime_seconds() - baseline >= self.config.bypass_observation_seconds - 1e-6

    def observe_bypass_corridor(self):
        scan = self.scan()
        self.corridor_observed_runtime = self.hardware.forward_runtime_seconds()
        if not scan_valid(scan, self.config):
            return self.wait_for_path("invalid_navigation_scan")
        clear, details = route_clearance(scan, self.config)
        self.emit("bypass_corridor", scan_cm=scan, clear=clear, **self.motion_details(), **details)
        if not clear:
            self.block_route(self.heading, details["reason"])
            return self.request_return("bypass_corridor_blocked")
        return True

    def advance_bypass(self):
        self.update_progress()
        if self.segment_drive_seconds >= self.config.bypass_segment_seconds - 1e-6:
            return self.finish_bypass_segment()
        if self.corridor_observation_due():
            self.observe_bypass_corridor()
            if self.phase != "bypass":
                return True
        front = self.hardware.front_distance()
        self.last_front_distance = front
        self.update_progress()
        self.emit("front_check", distance_cm=front, threshold_cm=self.config.forward_threshold(), **self.motion_details())
        if front is None:
            return self.wait_for_path("invalid_front_measurement")
        if front <= self.config.forward_threshold():
            self.hardware.brake("bypass_front_obstacle")
            self.update_progress()
            self.block_route(self.heading, "bypass_front_obstacle")
            return self.request_return("bypass_front_obstacle")
        remaining = self.config.bypass_segment_seconds - self.segment_drive_seconds
        if remaining <= 1e-6:
            return self.finish_bypass_segment()
        if self.corridor_observation_due():
            self.observe_bypass_corridor()
            if self.phase != "bypass":
                return True
        observed = self.corridor_observed_runtime if self.corridor_observed_runtime is not None else self.hardware.forward_runtime_seconds()
        until_observation = self.config.bypass_observation_seconds - (self.hardware.forward_runtime_seconds() - observed)
        self.transition("bypassing")
        self.hardware.advance(min(self.config.step_seconds, remaining, max(1e-6, until_observation)))
        self.bypass_steps += 1
        self.update_progress()
        self.emit("bypass_progress", steps=self.bypass_steps, **self.motion_details())
        if self.segment_drive_seconds >= self.config.bypass_segment_seconds - 1e-6:
            return self.finish_bypass_segment()
        return True

    def wait_tick(self):
        self.hardware.wait_for_retry(self.config.retry_pause_seconds)
        if not self.heading_known:
            self.emit("navigation_wait", reason="heading_unknown", **self.motion_details())
            return True
        scan = self.scan()
        if not scan_valid(scan, self.config):
            return self.wait_for_path("invalid_navigation_scan")
        margin = max(1.0, self.config.clearance_margin_cm)
        if self.waiting_reference is None or self.waiting_heading != self.heading:
            self.waiting_reference = dict(scan)
            self.waiting_heading = self.heading
        elif any(abs(scan[angle] - self.waiting_reference[angle]) >= margin for angle in scan):
            if self.waiting_candidate is not None and all(abs(scan[angle] - self.waiting_candidate[angle]) <= margin for angle in scan):
                self.blocked_routes.clear()
                self.clear_counts.clear()
                self.waiting_reference = dict(scan)
                self.waiting_candidate = None
                self.emit("route_memory_updated", reason="stable_scene_change", **self.motion_details())
            else:
                self.waiting_candidate = dict(scan)
        else:
            self.waiting_candidate = None
        return self.plan_route(scan)

    def tick(self):
        self.update_progress()
        if self.phase == "stopped":
            return False
        if self.hardware.ir_blocked():
            if self.phase.startswith("trajectory_"):
                self.hardware.brake("ir_obstacle")
                self.hardware.wait_for_retry(self.config.retry_pause_seconds)
                return self.trajectory.wait(self.phase, "ir_obstacle")
            self.recover_obstacle("ir_obstacle")
            self.hardware.wait_for_retry(self.config.retry_pause_seconds)
            return True
        self.refresh_routes()
        if self.phase.startswith("trajectory_"):
            return self.trajectory.tick()
        if self.phase == "waiting":
            return self.wait_tick()
        if self.phase == "cruise":
            if abs(self.heading) > 0.01:
                self.phase = "replan"
                return True
            front = self.hardware.front_distance()
            self.update_progress()
            self.last_front_distance = front
            self.emit("front_check", distance_cm=front, threshold_cm=self.config.forward_threshold(), **self.motion_details())
            if front is None:
                return self.wait_for_path("invalid_front_measurement")
            if front > self.config.forward_threshold():
                self.transition("forward")
                self.hardware.advance(self.config.step_seconds)
                return True
            self.hardware.brake("front_obstacle")
            self.update_progress()
            self.bypass_steps = 0
            self.return_attempts = 0
            self.segment_number = 0
            self.segment_started_runtime = None
            self.segment_drive_seconds = 0.0
            self.bypass_drive_seconds = 0.0
            self.bypass_heading = 0.0
            self.escape_sign = 0
            classification = self.hardware.classify_obstacle() if self.obstacle_mode == "ir" else {"label": "unknown", "stable_samples": 0, "reason": "bypass_mode"}
            action = "push" if classification["label"] == "permitted" and not self.push_attempted else "bypass"
            self.emit("obstacle_action", action=action, push_attempted=self.push_attempted, **classification)
            if action == "push":
                self.push_attempted = True
                reason = self.hardware.push_obstacle(classification["label"])
                scan = self.scan()
                clear, details = route_clearance(scan, self.config, self.config.resume_distance_cm)
                self.emit("push_assessment", clear=clear, push_stop_reason=reason, scan_cm=scan, **details)
                if reason == "invalid_distance":
                    return self.wait_for_path("invalid_push_measurement", scan)
                if clear:
                    self.phase = "cruise"
                    self.transition("forward", reason="corridor_clear_after_push")
                    return True
            self.path.begin(self.hardware.forward_runtime_seconds())
            self.block_route(0, "front_obstacle")
            self.phase = "replan"
            self.transition("scanning")
            return self.plan_route(self.scan())
        if self.phase == "bypass":
            return self.advance_bypass()
        if self.phase not in ("escape", "restore_bypass", "probe_return", "replan", "recenter"):
            return self.halt("unknown_navigation_phase")
        scan = self.scan()
        if not scan_valid(scan, self.config):
            return self.wait_for_path("invalid_navigation_scan")
        if self.phase in ("escape", "restore_bypass"):
            return self.align_side(scan)
        if self.phase == "probe_return":
            if abs(self.heading) > 0.01:
                return self.turn_towards(0, scan, "probing_original_course")
            clear, details = route_clearance(scan, self.config, self.config.resume_distance_cm)
            self.emit("return_corridor", scan_cm=scan, clear=clear, **self.motion_details(), **details)
            if clear and 0 not in self.blocked_routes:
                return self.resume_course()
            if not clear:
                self.block_route(0, details["reason"])
            return self.plan_route(scan)
        if self.phase == "recenter" and abs(self.heading) > 0.01:
            return self.turn_towards(0, scan, "returning_to_observation")
        return self.plan_route(scan)
