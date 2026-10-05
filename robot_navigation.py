from flask import Flask, Response
import threading
import RPi.GPIO as GPIO
import time
import cv2
import math

app = Flask(__name__)
camera = cv2.VideoCapture(0)

SAFE_DISTANCE = 20
heading_relative = 0.0
TURN_SPEED = 10
TURN_DEG_PER_SEC = 60.0
HEADING_TOLERANCE = 8.0
MOVE_SPEED = 25
STEP_TIME = 0.1
SERVO_STEP = 3
current_servo_pos = 90
SERVO_SPEED_DELAY = 0.02

IN1, IN2, IN3, IN4, ENA, ENB = 20, 21, 19, 26, 16, 13
key = 8
EchoPin, TrigPin = 0, 1
LED_R, LED_G, LED_B = 22, 27, 24
ServoPin = 23
AvoidSensorLeft, AvoidSensorRight = 12, 17

pwm_ENA = pwm_ENB = pwm_servo = None

overlay_lock = threading.Lock()
overlay_info = {
    "distances": "Scanning...",
    "decision": "Initializing..."
}


def init():
    global pwm_ENA, pwm_ENB, pwm_servo
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for p in [ENA, IN1, IN2, ENB, IN3, IN4, LED_R, LED_G, LED_B, ServoPin, TrigPin]:
        GPIO.setup(p, GPIO.OUT, initial=GPIO.HIGH if p in [ENA, ENB] else GPIO.LOW)
    for p in [key, EchoPin, AvoidSensorLeft, AvoidSensorRight]:
        GPIO.setup(p, GPIO.IN)

    pwm_ENA = GPIO.PWM(ENA, 2000);
    pwm_ENA.start(0)
    pwm_ENB = GPIO.PWM(ENB, 2000);
    pwm_ENB.start(0)
    pwm_servo = GPIO.PWM(ServoPin, 50);
    pwm_servo.start(0)


def run(ls, rs):
    GPIO.output(IN1, GPIO.HIGH);
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH);
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(ls);
    pwm_ENB.ChangeDutyCycle(rs)


def spin_left(ls, rs):
    GPIO.output(IN1, GPIO.LOW);
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH);
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(ls);
    pwm_ENB.ChangeDutyCycle(rs)


def spin_right(ls, rs):
    GPIO.output(IN1, GPIO.HIGH);
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW);
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(ls);
    pwm_ENB.ChangeDutyCycle(rs)


def brake():
    GPIO.output(IN1, GPIO.LOW);
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW);
    GPIO.output(IN4, GPIO.LOW)


def turn_relative(deg):
    global heading_relative
    if abs(deg) < 1.0: return
    dur = abs(deg) / TURN_DEG_PER_SEC
    (spin_right if deg > 0 else spin_left)(TURN_SPEED, TURN_SPEED)
    time.sleep(dur)
    brake()
    heading_relative += deg
    time.sleep(0.1)


def Distance():
    GPIO.output(TrigPin, GPIO.LOW);
    time.sleep(0.000002)
    GPIO.output(TrigPin, GPIO.HIGH);
    time.sleep(0.000015)
    GPIO.output(TrigPin, GPIO.LOW)
    t3 = time.time()
    while not GPIO.input(EchoPin):
        if time.time() - t3 > 0.03: return -1
    t1 = time.time()
    while GPIO.input(EchoPin):
        if time.time() - t1 > 0.03: return -1
    return ((time.time() - t1) * 340 / 2) * 100


def Distance_test():
    vals = [Distance() for _ in range(5) if Distance() not in [-1, 0]]
    if not vals: return -1
    vals.sort()
    return sum(vals[1:-1]) / len(vals[1:-1])


def servo_appointed_detection(pos):
    global current_servo_pos
    target_pos = max(0, min(180, int(pos)))

    if target_pos == current_servo_pos:
        pwm_servo.ChangeDutyCycle(2.5 + 10 * target_pos / 180)
        return

    step = SERVO_STEP if target_pos > current_servo_pos else -SERVO_STEP

    for p in range(int(current_servo_pos), target_pos, step):
        pwm_servo.ChangeDutyCycle(2.5 + 10 * p / 180)
        time.sleep(SERVO_SPEED_DELAY)

    pwm_servo.ChangeDutyCycle(2.5 + 10 * target_pos / 180)
    current_servo_pos = target_pos


def scan_all():
    res = {}
    for a in [0, 45, 90, 135, 180]:
        servo_appointed_detection(a)
        time.sleep(0.3)
        res[a] = Distance_test()
    servo_appointed_detection(90)
    return res


def log(text):
    ts = time.strftime('%H:%M:%S')
    msg = f"[{ts}]: {text}"
    print(msg)
    try:
        with open("logs.txt", "a") as f:
            f.write(f"{msg}\n")
    except:
        pass


def gen_frames():
    while True:
        success, frame = camera.read()
        if not success: break

        with overlay_lock:
            dist_txt = overlay_info["distances"]
            dec_txt = overlay_info["decision"]

        h, w = frame.shape[:2]
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (w - 10, 90), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        cv2.putText(frame, dist_txt, (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(frame, dec_txt, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)

        _, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    return "<h1>Robot Live Feed</h1><img src='/video_feed' width='640'>"


def main():
    global heading_relative
    log('System Start')
    init()
    time.sleep(1)
    servo_appointed_detection(90)
    heading_relative = 0.0
    turn_history = []

    try:
        while True:
            if Distance_test() >= SAFE_DISTANCE and abs(heading_relative) < HEADING_TOLERANCE:
                with overlay_lock: overlay_info["decision"] = "DECISION: GO FORWARD"
                run(MOVE_SPEED, MOVE_SPEED)
                time.sleep(STEP_TIME)
                continue

            brake()

            if Distance_test() >= SAFE_DISTANCE and turn_history:
                with overlay_lock: overlay_info["decision"] = "DECISION: RETURNING TO ROUTE"
                log("Obstacle bypassed. Returning to original route...")
                undo_deg = turn_history.pop(0)
                log(f"Undoing turn: {undo_deg}°")
                turn_relative(undo_deg)
                time.sleep(0.3)
                continue

            with overlay_lock:
                overlay_info["decision"] = "DECISION: SCANNING..."
            log("Obstacle detected or off-course. Full scanning...")
            scan = scan_all()
            dist_front = scan[90]

            dist_str = " | ".join([f"{a}°:{int(d) if d > 0 else '?'}cm" for a, d in sorted(scan.items())])
            with overlay_lock:
                overlay_info["distances"] = dist_str

            if dist_front < SAFE_DISTANCE:
                if scan[135] < SAFE_DISTANCE and scan[180] >= SAFE_DISTANCE:
                    turn_deg = -70
                elif scan[135] >= SAFE_DISTANCE and scan[180] < SAFE_DISTANCE:
                    turn_deg = -45
                elif scan[135] >= SAFE_DISTANCE and scan[180] >= SAFE_DISTANCE:
                    turn_deg = -45
                else:
                    if scan[45] < SAFE_DISTANCE and scan[0] >= SAFE_DISTANCE:
                        turn_deg = 70
                    elif scan[45] >= SAFE_DISTANCE and scan[0] < SAFE_DISTANCE:
                        turn_deg = 45
                    elif scan[45] >= SAFE_DISTANCE and scan[0] >= SAFE_DISTANCE:
                        turn_deg = 45

                '''
                left_free = scan[135] >= SAFE_DISTANCE or scan[180] >= SAFE_DISTANCE
                right_free = scan[45] >= SAFE_DISTANCE or scan[0] >= SAFE_DISTANCE

                if left_free and not right_free:
                    turn_deg = -45.0
                elif right_free and not left_free:
                    turn_deg = 45.0
                elif left_free:
                    turn_deg = -45.0
                else:
                    turn_deg = 45.0
                '''
                decision_msg = "TURN LEFT 45°" if turn_deg < 0 else "TURN RIGHT 45°"
                with overlay_lock:
                    overlay_info["decision"] = f"DECISION: {decision_msg}"
                log(f"Evasive maneuver: {decision_msg}")

                turn_relative(turn_deg)
                turn_history.append(-turn_deg)
                run(MOVE_SPEED, MOVE_SPEED)
                time.sleep(0.2)
                brake()
            else:
                if abs(heading_relative) >= HEADING_TOLERANCE:
                    with overlay_lock: overlay_info["decision"] = "DECISION: REALIGNING HEADING"
                    log(f"Realigning heading from {heading_relative}° to 0°")
                    turn_relative(-heading_relative)

            time.sleep(0.05)
    except KeyboardInterrupt:
        log("Interrupted by user")
    except Exception as e:
        log(f"Error: {e}")
    finally:
        brake()
        GPIO.cleanup()
        camera.release()


if __name__ == '__main__':
    time.sleep(2)
    flask_thread = threading.Thread(
        target=app.run,
        kwargs={'host': '0.0.0.0', 'port': 8080, 'debug': False, 'use_reloader': False}
    )
    flask_thread.daemon = True
    flask_thread.start()
    main()