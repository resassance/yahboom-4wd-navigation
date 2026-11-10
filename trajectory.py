import math
from dataclasses import replace


class PathMemory:
    def __init__(self, config, emit):
        self.config = config
        self.emit = emit
        self.active = False
        self.lateral_seconds = 0.0
        self.forward_seconds = 0.0
        self.runtime = 0.0
        self.segments = []
        self.range_anchor = None

    def begin(self, runtime):
        if self.active:
            return
        self.active = True
        self.runtime = runtime
        self.lateral_seconds = 0.0
        self.forward_seconds = 0.0
        self.segments = []
        self.range_anchor = None
        self.emit("path_memory_start", **self.details())

    def details(self):
        speed = self.config.drive_speed_cm_s
        return {
            "path_active": self.active,
            "lateral_drive_seconds": round(self.lateral_seconds, 6),
            "forward_drive_seconds": round(self.forward_seconds, 6),
            "lateral_estimate_cm": None if speed is None else round(self.lateral_seconds * speed, 3),
            "forward_estimate_cm": None if speed is None else round(self.forward_seconds * speed, 3),
            "position_source": "motor_time_with_range_corrections" if any(segment["kind"] == "range_correction" for segment in self.segments) else "motor_time_estimate",
        }

    def update(self, runtime, heading):
        delta = max(0.0, runtime - self.runtime)
        self.runtime = runtime
        if not self.active or delta <= 0:
            return
        self.lateral_seconds += delta * math.sin(math.radians(heading))
        self.forward_seconds += delta * math.cos(math.radians(heading))
        if self.segments and self.segments[-1]["kind"] == "drive" and self.segments[-1]["heading_deg"] == heading:
            self.segments[-1]["motor_seconds"] += delta
        else:
            self.segments.append({"kind": "drive", "heading_deg": heading, "motor_seconds": delta})
        self.emit("path_progress", heading_deg=heading, motor_seconds=round(delta, 6), **self.details())

    def record_turn(self, previous, heading):
        self.range_anchor = None
        if self.active:
            self.segments.append({"kind": "turn", "from_deg": previous, "to_deg": heading})
            self.emit("path_turn", from_deg=previous, to_deg=heading, **self.details())

    def observe_scan(self, scan, runtime, heading):
        reference = self.config.range_reference_heading_deg
        if not self.active or reference is None or abs(heading - reference) > 0.01:
            self.range_anchor = None
            return
        rays = [(angle, scan.get(angle)) for angle in (68, 90, 112)]
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= self.config.max_distance_cm for _, value in rays):
            self.range_anchor = None
            return
        front = scan[90]
        if any(abs(value * math.cos(math.radians(90 - angle)) - front) > 2 for angle, value in rays):
            self.range_anchor = None
            self.emit("path_range_rejected", reason="reference_not_planar", heading_deg=heading)
            return
        anchor = self.range_anchor
        self.range_anchor = (front, runtime, heading)
        if anchor is None or anchor[2] != heading:
            return
        elapsed = runtime - anchor[1]
        if elapsed <= 0:
            return
        measured = anchor[0] - front
        expected = elapsed * self.config.drive_speed_cm_s
        if not 0 <= measured <= expected * 1.5 + 1 or abs(measured - expected) > max(2, expected * 0.5):
            self.emit("path_range_rejected", reason="inconsistent_reference_motion", measured_cm=measured, expected_cm=expected)
            return
        correction = measured / self.config.drive_speed_cm_s - elapsed
        self.lateral_seconds += correction * math.sin(math.radians(heading))
        self.forward_seconds += correction * math.cos(math.radians(heading))
        self.segments.append({"kind": "range_correction", "heading_deg": heading, "correction_seconds": correction})
        self.emit("path_range_correction", measured_cm=measured, expected_cm=expected, reference_heading_deg=heading, **self.details())

    def complete(self):
        self.active = False
        self.range_anchor = None
        self.emit("path_return_complete", segments=self.segments, **self.details())


class TrajectoryReturn:
    def __init__(self, navigator):
        self.nav = navigator
        self.follow_started = 0.0
        self.observed_runtime = 0.0
        self.pending_phase = None

    def start(self):
        n = self.nav
        n.segment_started_runtime = None
        n.hardware.brake("trajectory_return_start")
        n.update_progress()
        self.follow_started = n.hardware.forward_runtime_seconds()
        self.observed_runtime = self.follow_started
        n.phase = "trajectory_follow" if abs(n.path.lateral_seconds) > n.config.return_tolerance_seconds else "trajectory_restore"
        n.emit("trajectory_return_start", **n.path.details())
        return True

    def wait(self, phase, reason):
        n = self.nav
        n.hardware.brake(reason)
        n.update_progress()
        if phase != "trajectory_wait":
            self.pending_phase = phase
        self.observed_runtime = -n.config.bypass_observation_seconds
        n.phase = "trajectory_wait"
        n.transition("waiting", reason=reason)
        n.emit("trajectory_return_wait", resume_phase=phase, reason=reason, **n.path.details())
        return True

    def turn(self, target, scan, phase):
        n = self.nav
        if abs(n.heading - target) < 0.01:
            return True
        n.turn_towards(target, scan, "returning_to_trajectory")
        if n.phase == "waiting":
            resume = "trajectory_back_out" if phase == "trajectory_align" else phase
            return self.wait(resume, n.waiting_reason)
        return True

    def cross_clearance(self, scan):
        from navigation import route_clearance

        n = self.nav
        radius = math.hypot(n.config.robot_width_cm, n.config.robot_length_cm) / 2
        depth = n.config.resume_distance_cm
        if n.config.drive_speed_cm_s is not None:
            depth = max(depth, abs(n.path.lateral_seconds) * n.config.drive_speed_cm_s + radius - n.config.sensor_forward_offset_cm + n.config.clearance_margin_cm)
        corridor_config = replace(n.config, robot_width_cm=2 * radius)
        clear, details = route_clearance(scan, corridor_config, depth)
        n.emit("trajectory_cross_corridor", clear=clear, **details)
        return clear, details

    def tick(self):
        from navigation import route_clearance, scan_valid

        n = self.nav
        if n.phase == "trajectory_wait":
            n.hardware.wait_for_retry(n.config.retry_pause_seconds)
            n.phase = self.pending_phase
            return True
        if not n.heading_known:
            return self.wait(n.phase, "heading_unknown")
        if n.phase == "trajectory_follow":
            if abs(n.heading) > 0.01:
                return self.wait("trajectory_back_out", "unexpected_follow_heading")
            runtime = n.hardware.forward_runtime_seconds()
            if runtime - self.follow_started >= n.config.return_probe_seconds:
                n.hardware.brake("trajectory_inward_probe")
                n.update_progress()
                n.phase = "trajectory_align"
                return True
            if runtime - self.observed_runtime >= n.config.bypass_observation_seconds:
                scan = n.scan()
                self.observed_runtime = n.hardware.forward_runtime_seconds()
                clear, details = route_clearance(scan, n.config)
                if not clear:
                    return self.wait("trajectory_follow", details["reason"])
            front = n.hardware.front_distance()
            n.update_progress()
            if front is None or front <= n.config.forward_threshold():
                return self.wait("trajectory_align", "invalid_front_measurement" if front is None else "trajectory_front_obstacle")
            n.hardware.advance(min(n.config.forward_interval(front), max(0.001, n.config.return_probe_seconds - (n.hardware.forward_runtime_seconds() - self.follow_started))))
            return True
        if n.phase == "trajectory_cross":
            remaining = abs(n.path.lateral_seconds)
            target = -90 if n.path.lateral_seconds > 0 else 90
            if remaining <= n.config.return_tolerance_seconds:
                n.hardware.brake("trajectory_lateral_target_reached")
                n.update_progress()
                n.phase = "trajectory_restore"
                return True
            if abs(n.heading - target) > 0.01:
                return self.wait("trajectory_restore", "return_direction_mismatch")
            if n.hardware.forward_runtime_seconds() - self.observed_runtime >= n.config.bypass_observation_seconds:
                scan = n.scan()
                self.observed_runtime = n.hardware.forward_runtime_seconds()
                clear, details = self.cross_clearance(scan)
                if not clear:
                    return self.wait("trajectory_back_out", details["reason"])
            remaining = abs(n.path.lateral_seconds)
            new_target = -90 if n.path.lateral_seconds > 0 else 90
            if remaining <= n.config.return_tolerance_seconds or new_target != target:
                n.phase = "trajectory_restore"
                return True
            front = n.hardware.front_distance()
            n.update_progress()
            if front is None or front <= n.config.forward_threshold():
                return self.wait("trajectory_cross" if front is None else "trajectory_back_out", "invalid_front_measurement" if front is None else "trajectory_cross_obstacle")
            try:
                n.hardware.advance(min(n.config.step_seconds, remaining))
            finally:
                n.hardware.brake("trajectory_cross_step")
                n.update_progress()
            return True
        phase = n.phase
        scan = n.scan()
        if not scan_valid(scan, n.config):
            return self.wait(phase, "invalid_trajectory_scan")
        if phase == "trajectory_align":
            target = -90 if n.path.lateral_seconds > 0 else 90
            if abs(n.heading - target) > 0.01:
                return self.turn(target, scan, phase)
            clear, details = self.cross_clearance(scan)
            if not clear:
                n.phase = "trajectory_back_out"
                return True
            n.phase = "trajectory_cross"
            self.observed_runtime = n.hardware.forward_runtime_seconds()
            n.emit("trajectory_cross_start", **n.path.details())
            return True
        if phase in ("trajectory_back_out", "trajectory_restore"):
            if abs(n.heading) > 0.01:
                return self.turn(0, scan, phase)
            if phase == "trajectory_back_out":
                return self.start()
            clear, details = route_clearance(scan, n.config, n.config.resume_distance_cm)
            if not clear:
                return self.wait(phase, details["reason"])
            if abs(n.path.lateral_seconds) > n.config.return_tolerance_seconds:
                return self.start()
            n.path.complete()
            return n.resume_course()
        return n.halt("unknown_trajectory_phase")
