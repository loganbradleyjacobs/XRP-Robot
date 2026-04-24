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

# Utilities
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
    print("go")
    imu.reset_yaw()
    drivetrain.reset_encoder_position()

    driveController = PIController(kp=0.1, ki=0.01, dt=dt)
    headingController = PIController(kp=0.03, ki=0.01, dt=dt)

    target_heading = 0
    errorList = []
    distList = []

    while True:
        if (rangefinder.distance() > 40):
            print("driving normally")
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
                break
            if abs(forward) > 0 and abs(forward) < 0.2:
                forward = 0.2 if forward > 0 else -0.2
    
            current_yaw = imu.get_yaw()
            turn = headingController.update(target_heading, current_yaw)
            
            #max_turn = 0.4 * abs(forward)
            turn = clamp(turn, -0.2, 0.2)
    
            left = forward - turn
            right = forward + turn
    
            left = clamp(left, -1.0, 1.0)
            right = clamp(right, -1.0, 1.0)
    
            drivetrain.set_effort(left, right)
    
            time.sleep(dt)
        else: 
            print("Diverting")

            drivetrain.stop()

            avoided = around()

            imu.reset_yaw()
            drivetrain.reset_encoder_position()

            driveController.integral = 0.0
            headingController.integral = 0.0

            target_cm -= avoided
            continue

    drivetrain.stop()


def peek_turn(target_deg):
    """
    A minimal PID turn that does NOT call imu.reset_yaw() at the start.
    Turns to target_deg relative to the CURRENT imu zero.
    Used by check_left_clear() so both the peek and the return-to-zero
    share the same IMU reference frame, avoiding the double-reset bug.
    Does reset yaw at the end to leave things clean.
    """
    kp = 0.025
    ki = 0.0
    kd = 0.008

    integral = 0.0
    prev_error = 0.0

    MIN_TURN = 0.25
    MAX_TURN = 0.3
    STOP_ANGLE = 2.5

    while True:
        current = imu.get_yaw()
        error = normalize_angle(target_deg - current)

        integral += error * dt
        integral = clamp(integral, -1.0, 1.0)

        derivative = (error - prev_error) / dt
        prev_error = error

        output = kp * error + ki * integral + kd * derivative
        output = clamp(output, -MAX_TURN, MAX_TURN)

        if abs(output) < MIN_TURN and abs(error) > STOP_ANGLE:
            output = MIN_TURN if output > 0 else -MIN_TURN

        drivetrain.set_effort(-output, output)

        if abs(error) < STOP_ANGLE:
            break

        time.sleep(dt)

    drivetrain.stop()


def check_left_clear():
    """
    Peeks 45 degrees left using a shared IMU frame (no reset between turns),
    reads the rangefinder, then returns to 0 degrees.
    Returns True if the left path is clear, False if blocked.
    """
    CLEAR_THRESHOLD = 40  # cm

    imu.reset_yaw()           # establish shared zero for both peek turns

    peek_turn(45)             # turn left 45 relative to that zero
    time.sleep(0.1)           # let rangefinder settle
    dist = rangefinder.distance()
    print("left peek distance:", dist)
    clear = dist > CLEAR_THRESHOLD

    peek_turn(0)              # return to zero in the same IMU frame
    drivetrain.stop()
    imu.reset_yaw()           # clean up for whoever calls next

    return clear


def around() -> float:
    """
    Checks left first. If clear, goes around left. Otherwise goes around right.
    Diamond path: returns to the original line after avoidance.
      Left:  +45, forward, -90, forward, +45
      Right: -45, forward, +90, forward, -45
    Net lateral displacement = 0. Forward progress = dist * sqrt(2).
    """
    dist = 50

    print("around")
    drivetrain.stop()

    if check_left_clear():
        print("going around left")
        turn(45)
        print("turned left 45")

        simple_forward(dist)
        print("went", dist, "cm")

        turn(-90)
        print("turned right 90")

        simple_forward(dist)
        print("went", dist, "cm")

        turn(45)
        print("turned left 45 - back to original heading")

    else:
        print("going around right")
        turn(-45)
        print("turned right 45")

        simple_forward(dist)
        print("went", dist, "cm")

        turn(90)
        print("turned left 90")

        simple_forward(dist)
        print("went", dist, "cm")

        turn(-45)
        print("turned right 45 - back to original heading")

    return math.sqrt(dist**2 + dist**2)


def simple_forward(distance_cm):
    drivetrain.reset_encoder_position()
    imu.reset_yaw()  # hold whatever heading we just turned to

    headingController = PIController(kp=0.03, ki=0.01, dt=dt)

    while True:
        ticks = (drivetrain.get_left_encoder_position() +
                 drivetrain.get_right_encoder_position()) / 2
        distance = (ticks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE

        if distance >= distance_cm:
            break

        current_yaw = imu.get_yaw()
        turn_correction = headingController.update(0, current_yaw)
        turn_correction = clamp(turn_correction, -0.2, 0.2)

        left = clamp(0.3 - turn_correction, -1.0, 1.0)
        right = clamp(0.3 + turn_correction, -1.0, 1.0)

        drivetrain.set_effort(left, right)
        time.sleep(dt)

    drivetrain.stop()


def turn(target_deg, smooth=False):
    """
    PID turn to target_deg.
    If smooth=True, once within BLEND_THRESHOLD degrees of the target,
    the robot begins blending in forward motion so the stop-start
    between turn() and simple_forward() is replaced by a gentle arc.
    The forward creep scales linearly from 0 -> CREEP_SPEED as the
    heading error closes, so the robot is already moving when it exits.
    """
    imu.reset_yaw()

    kp = 0.025
    ki = 0.0
    kd = 0.008

    integral = 0.0
    prev_error = 0.0

    MIN_TURN = 0.25
    MAX_TURN = 0.3
    STOP_ANGLE = 2.5

    BLEND_THRESHOLD = 20.0  # degrees: when to start blending forward
    CREEP_SPEED = 0.15      # max forward effort during blend phase

    while True:
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

        # Smooth blend: linearly ramp in forward creep as error shrinks
        if smooth and abs(error) < BLEND_THRESHOLD:
            # t goes 1.0 (at threshold) -> 0.0 (at STOP_ANGLE)
            t = (abs(error) - STOP_ANGLE) / (BLEND_THRESHOLD - STOP_ANGLE)
            t = clamp(t, 0.0, 1.0)
            creep = CREEP_SPEED * (1.0 - t)  # ramps UP as error shrinks
            left  = clamp(creep - output, -1.0, 1.0)
            right = clamp(creep + output, -1.0, 1.0)
        else:
            left  = -output
            right =  output

        drivetrain.set_effort(left, right)

        print("error:", round(error, 2), "output:", round(output, 3))

        if abs(error) < STOP_ANGLE:
            break

        time.sleep(dt)

    # No hard stop when smoothing — simple_forward takes over immediately
    if not smooth:
        drivetrain.stop()
    imu.reset_yaw()


def main():
    go(152)

if __name__ == "__main__":
    main()