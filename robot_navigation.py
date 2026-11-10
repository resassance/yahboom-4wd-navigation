import argparse
import hashlib
import json
import signal
import sys
import threading
import time
import traceback
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

from navigation import Config, Navigator, ObstacleDetected, classify_infrared, filtered_distance, navigation_plan, scan_angles
from color_detection import ColorDetector, load_color_config


PINS = {
    "IN1": 20, "IN2": 21, "IN3": 19, "IN4": 26,
    "ENA": 16, "ENB": 13, "key": 8, "echo": 0, "trigger": 1,
    "LED_R": 22, "LED_G": 27, "LED_B": 24, "servo": 23,
    "ir_left": 12, "ir_right": 17,
}


class SessionLog:
    def __init__(self, root):
        name = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        self.directory = Path(root).resolve() / name
        self.directory.mkdir(parents=True)
        self.events = (self.directory / "events.jsonl").open("w", encoding="utf-8", buffering=1)
        self.readable = (self.directory / "session.log").open("w", encoding="utf-8", buffering=1)
        self.started = time.monotonic()
        self.lock = threading.Lock()
        self.latest = "initializing"
        self.sequence = 0

    def emit(self, event, **details):
        with self.lock:
            self.sequence += 1
            record = {
                "sequence": self.sequence,
                "time_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                "elapsed_seconds": round(time.monotonic() - self.started, 6),
                "event": event,
                **details,
            }
            self.events.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
            line = f"[{record['elapsed_seconds']:8.3f}] {event}: {json.dumps(details, ensure_ascii=False)}"
            self.readable.write(line + "\n")
            if event != "distance_raw":
                print(line, flush=True)
            if event in {"state", "motor_start", "motor_stop", "distance_filtered", "turn_decision", "exception"}:
                self.latest = event + ": " + json.dumps(details, ensure_ascii=True)

    def close(self):
        self.events.close()
        self.readable.close()


class Hardware:
    def __init__(self, config, emit, stop, use_ir=False):
        self.config = config
        self.emit = emit
        self.stop = stop
        self.use_ir = use_ir
        self.gpio = None
        self.left = None
        self.right = None
        self.servo = None
        self.position = None
        self.last_trigger = 0.0
        self.moving = False
        self.cruising_since = None
        self.forward_completed_seconds = 0.0
        self.measurement_attempt = 1
        self.input_pins = set()
        self.motor_lock = threading.RLock()

    def initialize(self):
        import RPi.GPIO as GPIO
        self.gpio = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for name in ("ENA", "ENB", "IN1", "IN2", "IN3", "IN4", "LED_R", "LED_G", "LED_B", "servo", "trigger"):
            GPIO.setup(PINS[name], GPIO.OUT, initial=GPIO.LOW)
        for name in ("key", "echo", "ir_left", "ir_right"):
            GPIO.setup(PINS[name], GPIO.IN)
        self.input_pins = set(self.config.ir_classification_pins)
        for pin in self.input_pins:
            GPIO.setup(pin, GPIO.IN)
        self.left = GPIO.PWM(PINS["ENA"], 2000)
        self.left.start(0)
        self.right = GPIO.PWM(PINS["ENB"], 2000)
        self.right.start(0)
        self.servo = GPIO.PWM(PINS["servo"], 50)
        self.servo.start(0)
        self.emit("gpio_initialized", numbering="BCM", pins=PINS, ir_protection_enabled=self.use_ir)
        self.point(90)
        self.ir_blocked()

    def wait(self, seconds):
        if self.stop.wait(max(0.0, seconds)):
            raise InterruptedError("Stop requested")

    def wait_for_retry(self, seconds):
        self.wait(seconds)

    def ir_blocked(self):
        left = int(self.gpio.input(PINS["ir_left"]))
        right = int(self.gpio.input(PINS["ir_right"]))
        blocked = self.use_ir and self.config.ir_active_level in (left, right)
        self.emit("ir_reading", left_raw=left, right_raw=right, active_level=self.config.ir_active_level, protection_enabled=self.use_ir, blocked=blocked)
        return blocked

    def read_classification_ir(self):
        levels = [int(self.gpio.input(pin)) for pin in self.config.ir_classification_pins]
        self.emit("ir_obstacle_reading", pins_bcm=self.config.ir_classification_pins, levels=levels)
        return levels

    def classify_obstacle(self):
        self.brake("obstacle_classification")
        if not self.config.ir_obstacle_patterns:
            result = classify_infrared(self.config, [], [])
        else:
            readings, distances = [], []
            for sample in range(self.config.ir_confirm_samples):
                before = self.read_classification_ir()
                self.measurement_attempt = 1
                distance = self.raw_distance(sample + 1)
                after = self.read_classification_ir()
                readings.extend((before, after))
                distances.extend((distance, distance))
                self.emit("ir_obstacle_sample", pins_bcm=self.config.ir_classification_pins, before=before, after=after, distance_cm=distance, valid=distance is not None)
            result = classify_infrared(self.config, readings, distances)
        self.emit("obstacle_classification", pins_bcm=self.config.ir_classification_pins, **result)
        return result

    def push_obstacle(self, label):
        if label != "permitted":
            raise ValueError("Push requires a confirmed permitted obstacle")
        self.config.validate()
        if self.stop.is_set():
            raise InterruptedError("Stop requested")
        if self.moving or self.position != 90:
            raise RuntimeError("Push requires stopped motors and centred sensor")
        if self.ir_blocked():
            raise ObstacleDetected("IR obstacle before push")
        initial = self.raw_distance(1)
        if initial is None:
            self.emit("push_end", reason="invalid_distance", duration_actual_seconds=0.0)
            return "invalid_distance"
        if self.config.ir_min_distance_cm is None or not self.config.ir_min_distance_cm <= initial <= self.config.ir_max_distance_cm:
            self.emit("push_end", reason="outside_working_range", duration_actual_seconds=0.0)
            return "outside_working_range"
        started = time.monotonic()
        deadline = started + self.config.push_seconds
        reason = "interrupted"
        self.emit("push_start", duration_limit_seconds=self.config.push_seconds, pwm=self.config.push_speed, distance_cm=initial)
        watchdog_errors = []

        def stop_push():
            try:
                self.brake("push_time_limit")
            except Exception as exc:
                watchdog_errors.append(exc)

        watchdog = threading.Timer(max(0.0, deadline - time.monotonic()), stop_push)
        watchdog.daemon = True
        try:
            with self.motor_lock:
                if self.stop.is_set():
                    raise InterruptedError("Stop requested")
                if time.monotonic() >= deadline:
                    reason = "time_limit"
                    return reason
                watchdog.start()
                self.moving = True
                for name, value in zip(("IN1", "IN2", "IN3", "IN4"), (1, 0, 1, 0)):
                    self.gpio.output(PINS[name], value)
                self.left.ChangeDutyCycle(self.config.push_speed)
                self.right.ChangeDutyCycle(self.config.push_speed)
            while time.monotonic() < deadline:
                if self.ir_blocked():
                    reason = "ir_obstacle"
                    break
                distance = self.raw_distance(1, deadline=deadline)
                self.emit("push_distance", distance_cm=distance, valid=distance is not None)
                if time.monotonic() >= deadline:
                    reason = "time_limit"
                    break
                if distance is None:
                    reason = "invalid_distance"
                    break
                self.wait(min(0.02, max(0.0, deadline - time.monotonic())))
            else:
                reason = "time_limit"
        finally:
            try:
                self.brake("push_" + reason)
            finally:
                watchdog.cancel()
                if watchdog.ident is not None:
                    watchdog.join()
            self.emit("push_end", reason=reason, duration_actual_seconds=round(time.monotonic() - started, 6))
        if watchdog_errors:
            raise RuntimeError("Push watchdog brake failed") from watchdog_errors[0]
        return reason

    def brake(self, reason="stop"):
        with self.motor_lock:
            self.brake_locked(reason)

    def brake_locked(self, reason="stop"):
        if self.gpio is None:
            return
        errors = []
        for pwm in (self.left, self.right):
            if pwm is not None:
                try:
                    pwm.ChangeDutyCycle(0)
                except Exception as exc:
                    errors.append(str(exc))
        for name in ("IN1", "IN2", "IN3", "IN4"):
            try:
                self.gpio.output(PINS[name], self.gpio.LOW)
            except Exception as exc:
                errors.append(str(exc))
        self.moving = False
        started = self.cruising_since
        self.cruising_since = None
        elapsed = 0.0 if started is None else time.monotonic() - started
        self.forward_completed_seconds += elapsed
        self.emit("brake", reason=reason, errors=errors)
        if started is not None:
            self.emit("motor_stop", kind="forward", continuous=True, duration_actual_seconds=round(elapsed, 6), reason=reason)
        if errors:
            raise RuntimeError("Motor brake failed: " + "; ".join(errors))

    def forward_runtime_seconds(self) -> float:
        active = 0.0 if self.cruising_since is None else time.monotonic() - self.cruising_since
        return self.forward_completed_seconds + active

    def pulse(self, kind, seconds, speed, direction, angle=None, right_speed=None):
        right_speed = speed if right_speed is None else right_speed
        if not 0 < speed <= 100 or not 0 < right_speed <= 100 or seconds <= 0:
            raise ValueError("Invalid motor pulse")
        if self.stop.is_set():
            raise InterruptedError("Stop requested")
        if self.position != 90:
            raise RuntimeError("Sensor must be centred before moving")
        if self.ir_blocked():
            raise ObstacleDetected("IR obstacle before movement")
        self.emit("motor_start", kind=kind, duration_requested_seconds=seconds, pwm_left=speed, pwm_right=right_speed, angle_requested_deg=angle)
        started = time.monotonic()
        try:
            self.moving = True
            for name, value in zip(("IN1", "IN2", "IN3", "IN4"), direction):
                self.gpio.output(PINS[name], value)
            self.left.ChangeDutyCycle(speed)
            self.right.ChangeDutyCycle(right_speed)
            deadline = started + seconds
            while time.monotonic() < deadline:
                self.wait(min(0.02, deadline - time.monotonic()))
                if self.use_ir and self.ir_blocked():
                    raise ObstacleDetected("IR obstacle during movement", heading_uncertain=kind == "turn")
        finally:
            elapsed = time.monotonic() - started
            self.brake("pulse_finished")
            self.emit("motor_stop", kind=kind, duration_actual_seconds=round(elapsed, 6), angle_requested_deg=angle)
        self.wait(0.1)

    def forward(self, seconds):
        self.pulse("forward", seconds, self.config.move_speed + self.config.left_drive_trim_pwm, (1, 0, 1, 0), right_speed=self.config.move_speed + self.config.right_drive_trim_pwm)

    def advance(self, seconds):
        try:
            if self.stop.is_set():
                raise InterruptedError("Stop requested")
            if self.position != 90:
                raise RuntimeError("Sensor must be centred before moving")
            if not 0 < seconds <= 0.5:
                raise ValueError("Invalid forward monitoring interval")
            if self.ir_blocked():
                raise ObstacleDetected("IR obstacle before movement")
            if self.cruising_since is None:
                if self.moving:
                    raise RuntimeError("Another motor command is active")
                left = self.config.move_speed + self.config.left_drive_trim_pwm
                right = self.config.move_speed + self.config.right_drive_trim_pwm
                if not 0 < left <= 100 or not 0 < right <= 100:
                    raise ValueError("Invalid forward PWM")
                self.emit("motor_start", kind="forward", continuous=True, pwm_left=left, pwm_right=right, monitoring_interval_seconds=seconds)
                self.moving = True
                for name, value in zip(("IN1", "IN2", "IN3", "IN4"), (1, 0, 1, 0)):
                    self.gpio.output(PINS[name], value)
                self.left.ChangeDutyCycle(left)
                self.right.ChangeDutyCycle(right)
                self.cruising_since = time.monotonic()
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                self.wait(min(0.02, deadline - time.monotonic()))
                if self.use_ir and self.ir_blocked():
                    raise ObstacleDetected("IR obstacle during movement")
        except BaseException:
            self.brake("forward_interrupted")
            raise

    def front_distance(self):
        self.read_classification_ir()
        if not self.moving:
            value = self.measure(90)
            self.read_classification_ir()
            return value
        try:
            if self.stop.is_set():
                raise InterruptedError("Stop requested")
            if self.position != 90 or self.cruising_since is None:
                raise RuntimeError("Front monitoring requires centred sensor and forward motion")
            self.measurement_attempt = 1
            value = self.raw_distance(1)
            self.read_classification_ir()
            if value is None or value <= self.config.forward_threshold():
                self.brake("invalid_front_measurement" if value is None else "front_obstacle")
            self.emit("front_monitor", distance_cm=value, threshold_cm=self.config.forward_threshold(), motors_active=self.moving, reason="invalid_measurement" if value is None else "obstacle" if value <= self.config.forward_threshold() else "clear")
            return value
        except BaseException:
            self.brake("front_monitor_interrupted")
            raise

    def turn(self, angle):
        rate = self.config.turn_right_deg_per_sec if angle > 0 else self.config.turn_left_deg_per_sec
        direction = (1, 0, 0, 1) if angle > 0 else (0, 1, 1, 0)
        self.pulse("turn", abs(angle) / rate, self.config.turn_speed, direction, angle)

    def point(self, angle):
        if self.moving:
            raise RuntimeError("Cannot move sensor while motors are active")
        if not self.config.servo_min_deg <= angle <= self.config.servo_max_deg:
            self.emit("servo_rejected", angle_deg=angle, limits_deg=[self.config.servo_min_deg, self.config.servo_max_deg])
            raise ValueError("Servo angle outside configured mechanical limits")
        target = int(round(angle))
        self.emit("servo_start", previous_deg=self.position, target_deg=target)
        if self.position is not None and not self.config.servo_min_deg <= self.position <= self.config.servo_max_deg:
            self.emit("servo_position_reset", previous_deg=self.position)
            self.position = None
        if self.position is None:
            self.servo.ChangeDutyCycle(2.5 + 10 * target / 180)
            self.position = target
        else:
            step = self.config.servo_step_deg if target > self.position else -self.config.servo_step_deg
            for position in range(self.position, target, step):
                self.servo.ChangeDutyCycle(2.5 + 10 * position / 180)
                self.position = position
                self.wait(self.config.servo_step_seconds)
            self.servo.ChangeDutyCycle(2.5 + 10 * target / 180)
            self.position = target
        self.wait(self.config.servo_settle_seconds)
        self.emit("servo_ready", angle_deg=self.position)

    def raw_distance(self, sample, deadline=None):
        remaining = self.config.sample_interval_seconds - (time.monotonic() - self.last_trigger)
        self.wait(remaining if deadline is None else min(max(0.0, remaining), max(0.0, deadline - time.monotonic())))
        if deadline is not None and time.monotonic() >= deadline:
            self.emit("distance_raw", sample=sample, attempt=self.measurement_attempt, angle_deg=self.position, distance_cm=None, reason="push_deadline", echo_pulse_seconds=None)
            return None
        measurement_deadline = deadline
        gpio = self.gpio
        value = None
        reason = "ok"
        pulse_seconds = None
        if gpio.input(PINS["echo"]):
            reason = "echo_high_before_trigger"
            self.last_trigger = time.monotonic()
        else:
            gpio.output(PINS["trigger"], 0)
            time.sleep(0.000002)
            gpio.output(PINS["trigger"], 1)
            time.sleep(0.000015)
            gpio.output(PINS["trigger"], 0)
            self.last_trigger = time.monotonic()
            deadline = self.last_trigger + self.config.echo_timeout_seconds
            if measurement_deadline is not None:
                deadline = min(deadline, measurement_deadline)
            while not gpio.input(PINS["echo"]):
                if self.stop.is_set():
                    raise InterruptedError("Stop requested")
                if time.monotonic() >= deadline:
                    reason = "echo_rise_timeout"
                    break
            if reason == "ok":
                started = time.monotonic()
                deadline = started + self.config.echo_timeout_seconds
                if measurement_deadline is not None:
                    deadline = min(deadline, measurement_deadline)
                while gpio.input(PINS["echo"]):
                    if self.stop.is_set():
                        raise InterruptedError("Stop requested")
                    if time.monotonic() >= deadline:
                        reason = "echo_fall_timeout"
                        break
                if reason == "ok":
                    pulse_seconds = time.monotonic() - started
                    value = pulse_seconds * 17000
                    if not 0 < value <= self.config.max_distance_cm:
                        value = None
                        reason = "outside_range"
        self.emit("distance_raw", sample=sample, attempt=self.measurement_attempt, angle_deg=self.position, distance_cm=value, reason=reason, echo_pulse_seconds=pulse_seconds)
        return value

    def measure(self, angle=90):
        if self.moving:
            raise RuntimeError("Cannot measure while motors are active")
        if self.position != angle:
            self.point(angle)
        for attempt in range(self.config.measurement_retries + 1):
            self.measurement_attempt = attempt + 1
            self.emit("measurement_attempt", angle_deg=angle, attempt=attempt + 1)
            samples = [self.raw_distance(index + 1) for index in range(self.config.samples)]
            value = filtered_distance(samples, self.config)
            self.emit("distance_filtered", angle_deg=angle, distance_cm=value, samples_cm=samples, valid_count=sum(x is not None for x in samples), attempt=attempt + 1, reason="ok" if value is not None else "insufficient_valid_samples")
            self.ir_blocked()
            if value is not None:
                return value
            if attempt < self.config.measurement_retries:
                self.emit("measurement_retry", angle_deg=angle, next_attempt=attempt + 2)
        return None

    def scan(self, angles):
        angles = tuple(angles)
        if any(not self.config.servo_min_deg <= angle <= self.config.servo_max_deg for angle in angles):
            self.emit("scan_rejected", angles_deg=angles, limits_deg=[self.config.servo_min_deg, self.config.servo_max_deg])
            raise ValueError("Scan contains angles outside configured mechanical limits")
        self.brake("scan")
        self.emit("scan_start", angles_deg=list(angles))
        result = {}
        try:
            for angle in angles:
                result[angle] = self.measure(angle)
        finally:
            if not self.stop.is_set():
                self.point(90)
        self.emit("scan_complete", distances_cm=result)
        return result

    def close(self):
        if self.gpio is None:
            return
        try:
            self.brake("shutdown")
        finally:
            try:
                for pwm in (self.left, self.right, self.servo):
                    if pwm is not None:
                        try:
                            pwm.stop()
                        except Exception as exc:
                            self.emit("cleanup_error", component="pwm", error=str(exc))
            finally:
                self.gpio.cleanup(sorted(set(PINS.values()) | self.input_pins))
                self.emit("gpio_cleanup")


def parse_ir_pins(value):
    try:
        pins = tuple(int(item.strip()) for item in value.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError("IR pins must be comma-separated BCM integers")
    reserved = set(PINS.values()) - {PINS["ir_left"], PINS["ir_right"]}
    if not 1 <= len(pins) <= 8 or len(set(pins)) != len(pins) or any(pin < 2 or pin > 27 or pin in reserved for pin in pins):
        raise argparse.ArgumentTypeError("Choose 1..8 distinct BCM inputs; motor, servo, ultrasonic, button and LED pins are reserved")
    return pins


class InfraredMonitor:
    def __init__(self, pins, emit, stop):
        self.pins = pins
        self.emit = emit
        self.stop = stop
        self.gpio = None
        self.initialized = []
        self.previous = None
        self.transitions = dict.fromkeys(pins, 0)

    def initialize(self):
        import RPi.GPIO as GPIO
        self.gpio = GPIO
        GPIO.setmode(GPIO.BCM)
        for pin in self.pins:
            GPIO.setup(pin, GPIO.IN)
            self.initialized.append(pin)
        self.emit("ir_monitor_start", numbering="BCM", pins=self.pins)

    def poll(self):
        levels = {pin: int(self.gpio.input(pin)) for pin in self.pins}
        changed = [] if self.previous is None else [pin for pin in self.pins if levels[pin] != self.previous[pin]]
        for pin in changed:
            self.transitions[pin] += 1
        self.emit("ir_snapshot", levels=levels, changed_pins=changed, transitions=self.transitions.copy())
        self.previous = levels

    def close(self):
        if self.gpio is not None and self.initialized:
            try:
                self.emit("ir_monitor_summary", transitions=self.transitions.copy())
            finally:
                self.gpio.cleanup(self.initialized)


class Video:
    def __init__(self, log, port, color_config=None, session_stop=None):
        self.log = log
        self.port = port
        self.session_stop = session_stop if session_stop is not None else threading.Event()
        self.stop = threading.Event()
        self.camera = None
        self.start_failed = False
        self.condition = threading.Condition()
        self.frame = None
        self.frame_size = None
        self.number = 0
        self.thread = None
        self.server = None
        self.server_thread = None
        self.error = None
        self.ready = threading.Event()
        self.color_config = color_config
        self.detector = ColorDetector(color_config) if color_config is not None else None
        self.color_result = None
        self.color_processed = 0
        self.next_color_at = float("-inf")
        self.last_color_log = float("-inf")
        self.last_logged_color = None

    def update_color(self, frame, now):
        if self.detector is None or now < self.next_color_at:
            return
        started = time.perf_counter()
        self.color_result = self.detector.process(frame)
        self.color_processed += 1
        self.next_color_at = now + 1 / self.color_config.fps
        if self.color_result["label"] != self.last_logged_color or now - self.last_color_log >= 5:
            self.log.emit("color_result", **self.color_result, processed_frames=self.color_processed, processing_ms=round((time.perf_counter() - started) * 1000, 3))
            self.last_logged_color = self.color_result["label"]
            self.last_color_log = now

    def open_camera(self):
        self.log.emit("camera_opening", device=0)
        import cv2
        self.log.emit("camera_import_ready")
        self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.camera.isOpened():
            raise RuntimeError("Camera 0 could not be opened")
        self.log.emit("camera_open", device=0, backend="V4L2")

    def check_stop(self):
        if self.stop.is_set() or self.session_stop.is_set():
            raise InterruptedError("Stop requested during camera startup")

    def read_frame(self):
        success, frame = self.camera.read()
        if not success or frame is None:
            raise RuntimeError("Camera read failed")
        return frame

    def publish_frame(self, frame):
        import cv2
        if frame.shape[1] > 640:
            frame = cv2.resize(frame, (640, max(1, round(frame.shape[0] * 640 / frame.shape[1]))), interpolation=cv2.INTER_AREA)
        self.frame_size = [frame.shape[1], frame.shape[0]]
        now = time.monotonic()
        self.update_color(frame, now)
        if self.color_result is not None:
            result = self.color_result
            x, y, width, height = result["roi_box"]
            cv2.rectangle(frame, (x, y), (x + width, y + height), (160, 160, 160), 1)
            color = {"red": (0, 0, 255), "blue": (255, 0, 0), "unknown": (0, 255, 255)}[result["label"]]
            if result["box"] is not None:
                x, y, width, height = result["box"]
                cv2.rectangle(frame, (x, y), (x + width, y + height), color, 2)
            caption = result["label"].upper() + " / " + result["reason"]
            cv2.putText(frame, caption, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(frame, "red %.1f%% blue %.1f%%" % (100 * result["red_ratio"], 100 * result["blue_ratio"]), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        else:
            for index, text in enumerate((self.log.latest[:85], self.log.latest[85:170])):
                cv2.putText(frame, text, (10, 25 + index * 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        success, buffer = cv2.imencode(".jpg", frame)
        if not success:
            raise RuntimeError("JPEG encoding failed")
        with self.condition:
            self.frame = buffer.tobytes()
            self.number += 1
            self.condition.notify_all()
        self.ready.set()

    def close_capture(self):
        camera = self.camera
        self.camera = None
        try:
            if camera is not None:
                try:
                    camera.release()
                except Exception as exc:
                    self.error = self.error or exc
                    self.log.emit("camera_error", traceback=traceback.format_exc())
        finally:
            self.stop.set()
            self.ready.set()
            with self.condition:
                self.condition.notify_all()
            self.log.emit("camera_closed", total_frames=self.number)

    def start(self):
        try:
            self.check_stop()
            self.open_camera()
            self.check_stop()
            self.log.emit("camera_first_frame_start")
            frame = self.read_frame()
            self.check_stop()
            self.publish_frame(frame)
            self.check_stop()
            from flask import Flask, Response
            from werkzeug.serving import make_server
            app = Flask(__name__)

            @app.route("/")
            def index():
                description = "Распознавание цвета без команд моторам. RED — красный, BLUE — синий, UNKNOWN — цвет не определён. Рамка показывает область распознавания. Изображения не записываются." if self.detector is not None else "Видеонаблюдение. Остановка программы — Ctrl+C в терминале."
                return "<!doctype html><html lang='ru'><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Камера робота</title><h1>Камера робота</h1><p>" + description + "</p><img src='/video_feed' width='640' style='max-width:100%;height:auto' alt='Изображение с камеры робота'></html>"

            @app.route("/video_feed")
            def feed():
                return Response(self.frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

            self.server = make_server("0.0.0.0", self.port, app, threaded=True)
            self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
            self.thread = threading.Thread(target=self.capture, daemon=True)
            self.server_thread.start()
            self.thread.start()
            self.log.emit("video_server_start", port=self.server.server_port)
        except BaseException as exc:
            self.start_failed = True
            if isinstance(exc, Exception) and not isinstance(exc, InterruptedError):
                self.error = exc
                self.log.emit("camera_error", traceback=traceback.format_exc())
            if self.thread is None or not self.thread.is_alive():
                self.close_capture()
            raise

    def capture(self):
        try:
            if self.camera is None:
                self.open_camera()
            last_stats = time.monotonic()
            last_count = self.number
            while not self.stop.is_set() and not self.session_stop.is_set():
                try:
                    frame = self.read_frame()
                except RuntimeError:
                    if self.stop.is_set() or self.session_stop.is_set():
                        break
                    raise
                if self.stop.is_set() or self.session_stop.is_set():
                    break
                self.publish_frame(frame)
                now = time.monotonic()
                if now - last_stats >= 5:
                    self.log.emit("camera_stats", total_frames=self.number, fps=round((self.number - last_count) / (now - last_stats), 2), color_processed=self.color_processed, frame_size=self.frame_size)
                    last_stats = now
                    last_count = self.number
                self.stop.wait(1 / 15)
        except Exception as exc:
            self.error = exc
            self.log.emit("camera_error", traceback=traceback.format_exc())
        finally:
            self.close_capture()

    def frames(self):
        previous = -1
        while not self.stop.is_set():
            with self.condition:
                self.condition.wait_for(lambda: self.frame is not None and self.number != previous or self.stop.is_set(), timeout=1)
                if self.stop.is_set():
                    return
                if self.frame is None or self.number == previous:
                    continue
                data = self.frame
                previous = self.number
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"

    def close(self):
        self.stop.set()
        with self.condition:
            self.condition.notify_all()
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
        for thread in (self.thread, self.server_thread):
            if thread is not None:
                thread.join(timeout=2)
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError("Camera thread did not finish shutdown")
        if self.camera is not None:
            self.close_capture()
        self.log.emit("video_shutdown")
        if self.error is not None and not self.start_failed:
            raise RuntimeError("Camera failed: " + str(self.error)) from self.error


def load_config(path, auto=False):
    values = json.loads(Path(path).read_text(encoding="utf-8"))
    config = Config(**values)
    config.validate(require_geometry=auto)
    return config


def main(argv=None):
    parser = argparse.ArgumentParser(description="Yahboom 4WD diagnostics and experimental navigation")
    parser.add_argument("--mode", choices=("distance", "scan", "plan", "ir", "turn", "drive", "auto", "color"), default="scan")
    parser.add_argument("--config", default=str(Path(__file__).with_name("robot_config.json")))
    parser.add_argument("--log-dir", default=str(Path(__file__).with_name("logs")))
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--turn-deg", type=float, default=45.0)
    parser.add_argument("--drive-seconds", type=float, default=0.5)
    parser.add_argument("--max-seconds", type=float, default=30.0)
    parser.add_argument("--obstacle-mode", choices=("bypass", "ir"), default="bypass")
    parser.add_argument("--use-ir", action="store_true")
    parser.add_argument("--ir-pins", type=parse_ir_pins, default="12,17")
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--color-config", default=str(Path(__file__).with_name("color_config.json")))
    args = parser.parse_args(argv)
    if args.cycles <= 0 or not 0 < args.max_seconds <= 600 or not 0 < abs(args.turn_deg) <= 90 or not 0 < args.drive_seconds <= 2:
        parser.error("Use positive cycles, duration 0..600s, turn 0..90 degrees, drive 0..2s")
    if args.mode != "ir" and args.ir_pins != (12, 17):
        parser.error("Custom --ir-pins are available only in stationary ir mode")
    if args.mode == "ir" and (args.use_ir or args.video):
        parser.error("IR monitor logs raw inputs; --use-ir and --video are not used in this mode")
    if args.mode == "color" and args.use_ir:
        parser.error("Color mode does not initialize GPIO; --use-ir is unavailable")
    try:
        config = Config() if args.mode == "color" else load_config(args.config, args.mode in ("auto", "plan"))
        color_config = load_color_config(args.color_config) if args.mode == "color" else None
    except (OSError, ValueError, TypeError) as exc:
        parser.error(str(exc))
    try:
        log = SessionLog(args.log_dir)
    except OSError as exc:
        parser.error(f"Cannot create logs in {args.log_dir}: {exc}. Use --log-dir with a writable folder.")
    stop = threading.Event()
    stop_reason = {"reason": None}
    hardware = None if args.mode == "color" else InfraredMonitor(args.ir_pins, log.emit, stop) if args.mode == "ir" else Hardware(config, log.emit, stop, args.use_ir)
    video = Video(log, args.port, color_config=color_config, session_stop=stop) if args.mode == "color" else Video(log, args.port, session_stop=stop) if args.video else None
    timer = None
    old_handlers = {}
    exit_code = 0

    def request_stop(reason):
        if not stop.is_set():
            stop_reason["reason"] = reason
        stop.set()

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, lambda number, frame: request_stop(signal.Signals(number).name))
        hashes = {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest() for name in ("robot_navigation.py", "navigation.py", "trajectory.py", "color_detection.py")}
        log.emit("session_start", mode=args.mode, arguments=vars(args), config=asdict(config), color_config=asdict(color_config) if color_config is not None else None, source_sha256=hashes, python=sys.version, log_directory=str(log.directory))
        (log.directory / "config.json").write_text(json.dumps(asdict(config), indent=2) + "\n", encoding="utf-8")
        if color_config is not None:
            (log.directory / "color_config.json").write_text(json.dumps(asdict(color_config), indent=2) + "\n", encoding="utf-8")
        if hardware is not None:
            hardware.initialize()
        timer = threading.Timer(args.max_seconds, request_stop, args=("time_limit",))
        timer.daemon = True
        timer.start()
        if video is not None:
            video.start()
        if args.mode == "color":
            while not stop.is_set():
                if video.stop.is_set():
                    raise RuntimeError("Camera stopped: " + str(video.error or "unexpected shutdown"))
                stop.wait(0.1)
        elif args.mode == "ir":
            while not stop.is_set():
                hardware.poll()
                stop.wait(0.1)
        elif args.mode in ("distance", "scan", "plan"):
            for cycle in range(args.cycles):
                log.emit("diagnostic_cycle", cycle=cycle + 1)
                if args.mode == "distance":
                    hardware.measure(90)
                else:
                    scan = hardware.scan(scan_angles(config))
                    if args.mode == "plan":
                        decision = navigation_plan(scan, config)
                        if hardware.ir_blocked():
                            decision = {"action": "stop", "reason": "ir_obstacle", "angle_deg": None}
                        log.emit("navigation_plan", distances_cm=scan, **decision)
        elif args.mode == "turn":
            hardware.turn(args.turn_deg)
        elif args.mode == "drive":
            hardware.forward(args.drive_seconds)
        else:
            navigator = Navigator(config, hardware, log.emit, obstacle_mode=args.obstacle_mode)
            try:
                while not stop.is_set():
                    try:
                        if not navigator.tick():
                            break
                    except ObstacleDetected as exc:
                        navigator.recover_obstacle(str(exc), heading_uncertain=exc.heading_uncertain)
            finally:
                hardware.brake("auto_finished")
                navigator.update_progress()
                log.emit("navigation_final", heading_estimate_deg=navigator.heading, heading_known=navigator.heading_known, segments=navigator.path.segments, **navigator.path.details())
        log.emit("session_complete", stop_requested=stop.is_set(), stop_reason=stop_reason["reason"])
    except (KeyboardInterrupt, InterruptedError) as exc:
        log.emit("interrupted", reason=str(exc), stop_requested=stop.is_set(), stop_reason=stop_reason["reason"])
    except Exception:
        exit_code = 1
        log.emit("exception", traceback=traceback.format_exc())
    finally:
        stop.set()
        if timer is not None:
            timer.cancel()
        for component in (hardware, video):
            if component is not None:
                try:
                    component.close()
                except Exception:
                    exit_code = 1
                    log.emit("cleanup_exception", traceback=traceback.format_exc())
        for sig, old in old_handlers.items():
            signal.signal(sig, old)
        log.emit("session_end", exit_code=exit_code)
        log.close()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
