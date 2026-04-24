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
    

def around() -> float:
    imu.reset_yaw()
    
    dist = 50
    
    print("around")
    drivetrain.stop()

    turn(-45)
    print("turned right 45")

    simple_forward(dist)
    print("went 50 cm")

    turn(90)
    print("turned left 90")
    simple_forward(dist)
    print('went 50 cm')

    turn(-45)
    print("turned right 45")

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


def turn(target_deg):
    #drivetrain.stop()
    imu.reset_yaw()

    kp = 0.025
    ki = 0.0
    kd = 0.008 #0.012

    integral = 0.0
    prev_error = 0.0

    MIN_TURN = 0.25   # your old deadzone (good value)
    MAX_TURN = 0.3
    STOP_ANGLE = 2.5

    while True:
        current = imu.get_yaw()
        error = normalize_angle(target_deg - current)

        # PID
        integral += error * dt
        integral = clamp(integral, -1.0, 1.0)

        derivative = (error - prev_error) / dt
        prev_error = error

        output = kp * error + ki * integral + kd * derivative

        # Clamp max (FIXED)
        output = clamp(output, -MAX_TURN, MAX_TURN)

        # ✅ SMART minimum output (ONLY when close)
        if abs(output) < MIN_TURN and abs(error) > STOP_ANGLE:
            output = MIN_TURN if output > 0 else -MIN_TURN
            
            
        drivetrain.set_effort(-output, output)

        print("error:", round(error, 2), "output:", round(output, 3))

        # ✅ Better exit condition
        if abs(error) < STOP_ANGLE:
            break

        time.sleep(dt)

    drivetrain.stop()
    imu.reset_yaw()


def main():
    go(152)

if __name__ == "__main__":
    main()