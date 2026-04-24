from XRPLib.defaults import *
import time
import math

# Constants
dt = 0.05
MAX_EFFORT = 0.3 ## MAX IS 1.0
MIN_EFFORT = -MAX_EFFORT ## INVERSE OF MAX, CHANGE FOR SPECIFIC CONTROL IN TURNING 
deadzone = 0.18 ## CHANGE ACCORDING TO WEIGHT!!! DEFAULT IS 0.18 FOR NAKED BOT

YAW_BIAS = 0.00817 ## DERIVED FROM MEASUREMENTS, MAY BE INNACURATE. 
arounding = False  # (no longer used, left untouched)

# Drive constants
WHEEL_CIRCUMFERENCE = 21.0
TICKS_PER_REV = 24
##INACCURATE BUT SOMEWHAT TRANSLATES TO REAL DISTANCE

# ── Logging ──────────────────────────────────────────────────────────────────
LOG_FILE = "run_log.txt"

def log_open():
    """Call once at startup to create/clear the log file."""
    with open(LOG_FILE, "w") as f:
        f.write("=== RUN START: {} ms ===\n".format(time.ticks_ms()))

def log(msg):
    """Write a timestamped line to the log file and also print it."""
    line = "[{}] {}\n".format(time.ticks_ms(), msg)
    print(line, end="")
    with open(LOG_FILE, "a") as f:
        f.write(line)

# ── Utilities ─────────────────────────────────────────────────────────────────
def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)

def normalize_angle(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a

def accelerate(self, current, target):
    global ACCEL_FR
    global ACCEL_MG
    pass

def read_yaw(start_time_ms):
    elapsed_s = (time.ticks_ms() - start_time_ms) / 1000.0
    return imu.get_yaw() - YAW_BIAS * elapsed_s

# PI Controller
class PIController:
    def __init__(self, kp, ki, dt):
        self.kp = kp
        self.ki = ki
        self.dt = dt
        self.integral = 0.0

    def update(self, target, measurement):
        error = target - measurement

        if (abs(error) < 10.0) and (abs(error) > 3):
            self.integral += error * self.dt

        self.integral = clamp(self.integral, -1.0, 1.0)

        return self.kp * error + self.ki * self.integral

def go(target_cm): ## 1ft ~ 30.5cm
    log("go: target={} cm".format(target_cm))
    imu.reset_yaw()
    drivetrain.reset_encoder_position()

    driveController = PIController(kp=0.1, ki=0.01, dt=dt)
    headingController = PIController(kp=0.03, ki=0.01, dt=dt)

    target_heading = 0
    errorList = []
    distList = []

    while True:
        if (rangefinder.distance() > 40):
            ticks = (drivetrain.get_left_encoder_position() +
             drivetrain.get_right_encoder_position()) / 2
            distance = (ticks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE
            error = target_cm - distance
    
            errorList.append(error)
            distList.append(distance)
    
            if len(errorList) > 5:
                errorList.pop(0)
                distList.pop(0)
                
            avgError = sum(errorList) / len(errorList)
            
            if len(distList) >= 3:
                v1 = distList[-1] - distList[-2]
                v2 = distList[-2] - distList[-3]
                velocity = (v1 + v2) / 2
            else:
                velocity = 0
    
            forward = driveController.update(target_cm, distance)
            forward = clamp(forward, -0.8, 0.8)
            if abs(error) < 1.5 and abs(forward) < 0.05:
                log("go: target reached at distance={:.1f} cm".format(distance))
                break
            if abs(forward) > 0 and abs(forward) < 0.2:
                forward = 0.2 if forward > 0 else -0.2
    
            current_yaw = imu.get_yaw()
            turn = headingController.update(target_heading, current_yaw)
            turn = clamp(turn, -0.2, 0.2)
    
            left = forward - turn
            right = forward + turn
    
            left = clamp(left, -1.0, 1.0)
            right = clamp(right, -1.0, 1.0)
    
            drivetrain.set_effort(left, right)
    
            time.sleep(dt)
        else:
            log("go: obstacle detected at {:.1f} cm, distance traveled={:.1f} cm".format(
                rangefinder.distance(),
                (((drivetrain.get_left_encoder_position() +
                   drivetrain.get_right_encoder_position()) / 2)
                 / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE
            ))

            drivetrain.stop()

            avoided = around()

            imu.reset_yaw()
            drivetrain.reset_encoder_position()

            driveController.integral = 0.0
            headingController.integral = 0.0

            target_cm -= avoided
            log("go: resuming, adjusted target={:.1f} cm".format(target_cm))
            continue

    drivetrain.stop()


def around() -> float:
    """
    Peeks left 45°. If clear, goes around left.
    If blocked, turns 90° right (from the +45 peek position, net -45 from forward)
    and goes around right. Either way the robot is already facing the correct
    diagonal when avoidance begins — no extra turn needed.

    Diamond path in both cases returns to the original line.
      Left:  (already at +45) forward, -90, forward, +45
      Right: (already at -45) forward, +90, forward, -45
    Forward progress = dist * sqrt(2).
    """
    dist = 50

    log("around: peeking left")
    drivetrain.stop()

    imu.reset_yaw()
    turn(45)
    time.sleep(0.1)
    left_dist = rangefinder.distance()
    log("around: left peek distance={:.1f} cm".format(left_dist))

    if left_dist > 40:
        log("around: left clear, going around left")

        simple_forward(dist)
        log("around: leg 1 done")

        turn(-90)
        log("around: turned right 90")

        simple_forward(dist)
        log("around: leg 2 done")

        turn(45)
        log("around: back to original heading (left path complete)")

    else:
        log("around: left blocked, turning right 90 to face -45")
        turn(-90)
        log("around: now facing -45 from original forward")

        simple_forward(dist)
        log("around: leg 1 done")

        turn(90)
        log("around: turned left 90")

        simple_forward(dist)
        log("around: leg 2 done")

        turn(-45)
        log("around: back to original heading (right path complete)")

    avoided = math.sqrt(dist**2 + dist**2)
    log("around: complete, avoided={:.1f} cm".format(avoided))
    return avoided


def simple_forward(distance_cm):
    drivetrain.reset_encoder_position()
    imu.reset_yaw()

    headingController = PIController(kp=0.03, ki=0.01, dt=dt)
    log("simple_forward: target={} cm".format(distance_cm))

    while True:
        ticks = (drivetrain.get_left_encoder_position() +
                 drivetrain.get_right_encoder_position()) / 2
        distance = (ticks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE

        if distance >= distance_cm:
            log("simple_forward: done at {:.1f} cm".format(distance))
            break

        current_yaw = imu.get_yaw()
        turn_correction = headingController.update(0, current_yaw)
        turn_correction = clamp(turn_correction, -0.2, 0.2)

        left = clamp(0.3 - turn_correction, -1.0, 1.0)
        right = clamp(0.3 + turn_correction, -1.0, 1.0)

        drivetrain.set_effort(left, right)
        time.sleep(dt)

    drivetrain.stop()


def turn(target_deg, smooth=False, timeout_s=3.0):
    """
    PID turn to target_deg.
    timeout_s: max seconds to attempt the turn before giving up.
               Logs a warning and stops if exceeded.
    smooth: blends in forward creep during the final degrees of the turn.
    """
    imu.reset_yaw()
    start_ms = time.ticks_ms()
    log("turn: target={} deg, timeout={} s".format(target_deg, timeout_s))

    kp = 0.025
    ki = 0.0
    kd = 0.008

    integral = 0.0
    prev_error = 0.0

    MIN_TURN = 0.25
    MAX_TURN = 0.3
    STOP_ANGLE = 2.5

    BLEND_THRESHOLD = 20.0
    CREEP_SPEED = 0.15

    while True:
        # ── Timeout check ────────────────────────────────────────────────────
        elapsed_s = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed_s >= timeout_s:
            current = imu.get_yaw()
            log("turn: TIMEOUT after {:.2f} s — target={} deg, current={:.1f} deg, error={:.1f} deg".format(
                elapsed_s, target_deg, current, normalize_angle(target_deg - current)
            ))
            break

        current = imu.get_yaw()
        error = normalize_angle(target_deg - current)

        # PID
        integral += error * dt
        integral = clamp(integral, -1.0, 1.0)

        derivative = (error - prev_error) / dt
        prev_error = error

        output = kp * error + ki * integral + kd * derivative
        output = clamp(output, -MAX_TURN, MAX_TURN)

        if abs(output) < MIN_TURN and abs(error) > STOP_ANGLE:
            output = MIN_TURN if output > 0 else -MIN_TURN

        if smooth and abs(error) < BLEND_THRESHOLD:
            t = (abs(error) - STOP_ANGLE) / (BLEND_THRESHOLD - STOP_ANGLE)
            t = clamp(t, 0.0, 1.0)
            creep = CREEP_SPEED * (1.0 - t)
            left  = clamp(creep - output, -1.0, 1.0)
            right = clamp(creep + output, -1.0, 1.0)
        else:
            left  = -output
            right =  output

        drivetrain.set_effort(left, right)

        # Log every ~10 cycles (~0.5 s) to keep file size manageable
        if int(elapsed_s / dt) % 10 == 0:
            log("turn: t={:.2f} s  yaw={:.1f}  error={:.1f}  output={:.3f}".format(
                elapsed_s, current, error, output
            ))

        if abs(error) < STOP_ANGLE:
            log("turn: complete — target={} deg, final yaw={:.1f} deg, time={:.2f} s".format(
                target_deg, current, elapsed_s
            ))
            break

        time.sleep(dt)

    if not smooth:
        drivetrain.stop()
    imu.reset_yaw()


def main():
    log_open()
    log("main: starting")
    go(152)
    log("main: done")

if __name__ == "__main__":
    main()