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

from navigation import Config, Navigator, filtered_distance


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

    def initialize(self):
        import RPi.GPIO as GPIO
        self.gpio = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for name in ("ENA", "ENB", "IN1", "IN2", "IN3", "IN4", "LED_R", "LED_G", "LED_B", "servo", "trigger"):
            GPIO.setup(PINS[name], GPIO.OUT, initial=GPIO.LOW)
        for name in ("key", "echo", "ir_left", "ir_right"):
            GPIO.setup(PINS[name], GPIO.IN)
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

    def ir_blocked(self):
        left = int(self.gpio.input(PINS["ir_left"]))
        right = int(self.gpio.input(PINS["ir_right"]))
        blocked = self.use_ir and self.config.ir_active_level in (left, right)
        self.emit("ir_reading", left_raw=left, right_raw=right, active_level=self.config.ir_active_level, protection_enabled=self.use_ir, blocked=blocked)
        return blocked

    def brake(self, reason="stop"):
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
        self.emit("brake", reason=reason, errors=errors)
        if errors:
            raise RuntimeError("Motor brake failed: " + "; ".join(errors))

    def pulse(self, kind, seconds, speed, direction, angle=None):
        if self.stop.is_set():
            raise InterruptedError("Stop requested")
        if self.position != 90:
            raise RuntimeError("Sensor must be centred before moving")
        if self.ir_blocked():
            raise InterruptedError("IR obstacle before movement")
        self.emit("motor_start", kind=kind, duration_requested_seconds=seconds, pwm_left=speed, pwm_right=speed, angle_requested_deg=angle)
        started = time.monotonic()
        try:
            self.moving = True
            for name, value in zip(("IN1", "IN2", "IN3", "IN4"), direction):
                self.gpio.output(PINS[name], value)
            self.left.ChangeDutyCycle(speed)
            self.right.ChangeDutyCycle(speed)
            deadline = started + seconds
            while time.monotonic() < deadline:
                self.wait(min(0.02, deadline - time.monotonic()))
                if self.use_ir and self.ir_blocked():
                    raise InterruptedError("IR obstacle during movement")
        finally:
            elapsed = time.monotonic() - started
            self.brake("pulse_finished")
            self.emit("motor_stop", kind=kind, duration_actual_seconds=round(elapsed, 6), angle_requested_deg=angle)
        self.wait(0.1)

    def forward(self, seconds):
        self.pulse("forward", seconds, self.config.move_speed, (1, 0, 1, 0))

    def turn(self, angle):
        rate = self.config.turn_right_deg_per_sec if angle > 0 else self.config.turn_left_deg_per_sec
        direction = (1, 0, 0, 1) if angle > 0 else (0, 1, 1, 0)
        self.pulse("turn", abs(angle) / rate, self.config.turn_speed, direction, angle)

    def point(self, angle):
        if self.moving:
            raise RuntimeError("Cannot move sensor while motors are active")
        if not 0 <= angle <= 180:
            raise ValueError("Servo angle outside 0..180")
        target = int(round(angle))
        self.emit("servo_start", previous_deg=self.position, target_deg=target)
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

    def raw_distance(self, sample):
        self.wait(self.config.sample_interval_seconds - (time.monotonic() - self.last_trigger))
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
            while not gpio.input(PINS["echo"]):
                if self.stop.is_set():
                    raise InterruptedError("Stop requested")
                if time.monotonic() >= deadline:
                    reason = "echo_rise_timeout"
                    break
            if reason == "ok":
                started = time.monotonic()
                deadline = started + self.config.echo_timeout_seconds
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
        self.emit("distance_raw", sample=sample, angle_deg=self.position, distance_cm=value, reason=reason, echo_pulse_seconds=pulse_seconds)
        return value

    def measure(self, angle=90):
        if self.moving:
            raise RuntimeError("Cannot measure while motors are active")
        if self.position != angle:
            self.point(angle)
        samples = [self.raw_distance(index + 1) for index in range(self.config.samples)]
        value = filtered_distance(samples, self.config)
        self.emit("distance_filtered", angle_deg=angle, distance_cm=value, samples_cm=samples, valid_count=sum(x is not None for x in samples), reason="ok" if value is not None else "insufficient_valid_samples")
        self.ir_blocked()
        return value

    def scan(self, angles):
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
                self.gpio.cleanup(list(PINS.values()))
                self.emit("gpio_cleanup")


class Video:
    def __init__(self, log, port):
        self.log = log
        self.port = port
        self.stop = threading.Event()
        self.condition = threading.Condition()
        self.frame = None
        self.number = 0
        self.thread = None
        self.server = None
        self.server_thread = None

    def start(self):
        from flask import Flask, Response
        from werkzeug.serving import make_server
        app = Flask(__name__)

        @app.route("/")
        def index():
            return "<h1>Yahboom 4WD</h1><p>Video is read-only. Stop from the terminal with Ctrl+C.</p><img src='/video_feed' width='640'>"

        @app.route("/video_feed")
        def feed():
            return Response(self.frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

        self.server = make_server("0.0.0.0", self.port, app, threaded=True)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread = threading.Thread(target=self.capture, daemon=True)
        self.server_thread.start()
        self.thread.start()
        self.log.emit("video_server_start", port=self.port)

    def capture(self):
        camera = None
        try:
            import cv2
            camera = cv2.VideoCapture(0)
            if not camera.isOpened():
                raise RuntimeError("Camera 0 could not be opened")
            self.log.emit("camera_open")
            last_stats = time.monotonic()
            while not self.stop.is_set():
                success, frame = camera.read()
                if not success:
                    raise RuntimeError("Camera read failed")
                for index, text in enumerate((self.log.latest[:85], self.log.latest[85:170])):
                    cv2.putText(frame, text, (10, 25 + index * 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                success, buffer = cv2.imencode(".jpg", frame)
                if not success:
                    raise RuntimeError("JPEG encoding failed")
                with self.condition:
                    self.frame = buffer.tobytes()
                    self.number += 1
                    self.condition.notify_all()
                if time.monotonic() - last_stats >= 5:
                    self.log.emit("camera_stats", total_frames=self.number)
                    last_stats = time.monotonic()
                self.stop.wait(0.03)
        except Exception:
            self.log.emit("camera_error", traceback=traceback.format_exc())
        finally:
            if camera is not None:
                camera.release()
            self.stop.set()
            with self.condition:
                self.condition.notify_all()
            self.log.emit("camera_closed", total_frames=self.number)

    def frames(self):
        previous = -1
        while not self.stop.is_set():
            with self.condition:
                self.condition.wait_for(lambda: self.number != previous or self.stop.is_set(), timeout=1)
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
        self.log.emit("video_shutdown")


def load_config(path, auto=False):
    values = json.loads(Path(path).read_text(encoding="utf-8"))
    config = Config(**values)
    config.validate(require_geometry=auto)
    return config


def main(argv=None):
    parser = argparse.ArgumentParser(description="Yahboom 4WD diagnostics and experimental navigation")
    parser.add_argument("--mode", choices=("distance", "scan", "turn", "drive", "auto"), default="scan")
    parser.add_argument("--config", default=str(Path(__file__).with_name("robot_config.json")))
    parser.add_argument("--log-dir", default=str(Path(__file__).with_name("logs")))
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--turn-deg", type=float, default=45.0)
    parser.add_argument("--drive-seconds", type=float, default=0.5)
    parser.add_argument("--max-seconds", type=float, default=30.0)
    parser.add_argument("--use-ir", action="store_true")
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args(argv)
    if args.cycles <= 0 or not 0 < args.max_seconds <= 600 or not 0 < abs(args.turn_deg) <= 90 or not 0 < args.drive_seconds <= 2:
        parser.error("Use positive cycles, duration 0..600s, turn 0..90 degrees, drive 0..2s")
    try:
        config = load_config(args.config, args.mode == "auto")
    except (OSError, ValueError, TypeError) as exc:
        parser.error(str(exc))
    log = SessionLog(args.log_dir)
    stop = threading.Event()
    stop_reason = {"reason": None}
    hardware = Hardware(config, log.emit, stop, args.use_ir)
    video = Video(log, args.port) if args.video else None
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
        hashes = {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest() for name in ("robot_navigation.py", "navigation.py")}
        log.emit("session_start", mode=args.mode, arguments=vars(args), config=asdict(config), source_sha256=hashes, python=sys.version, log_directory=str(log.directory))
        (log.directory / "config.json").write_text(json.dumps(asdict(config), indent=2) + "\n", encoding="utf-8")
        hardware.initialize()
        timer = threading.Timer(args.max_seconds, request_stop, args=("time_limit",))
        timer.daemon = True
        timer.start()
        if video is not None:
            video.start()
        if args.mode in ("distance", "scan"):
            for cycle in range(args.cycles):
                log.emit("diagnostic_cycle", cycle=cycle + 1)
                if args.mode == "distance":
                    hardware.measure(90)
                else:
                    hardware.scan((0, 45, 90, 135, 180))
        elif args.mode == "turn":
            hardware.turn(args.turn_deg)
        elif args.mode == "drive":
            hardware.forward(args.drive_seconds)
        else:
            navigator = Navigator(config, hardware, log.emit)
            while not stop.is_set() and navigator.tick():
                pass
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
