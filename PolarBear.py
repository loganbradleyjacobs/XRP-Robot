from XRPLib.defaults import *
import time
import math

# Constants
dt = 0.05
MAX_EFFORT = 0.3 ## MAX IS 1.0
MIN_EFFORT = -MAX_EFFORT ## INVERSE OF MAX, CHANGE FOR SPECIFIC CONTROL IN TURNING 

deadzone = 0.18 ## CHANGE ACCORDING TO WEIGHT!!! DEFAULT IS 0.18 FOR NAKED BOT

YAW_BIAS = 0.00817 ## DERIVED FROM MEASUREMENTS, MAY BE INNACURATE. 

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

def set_motor_speed(base_speed, correction):
    left = clamp(base_speed + correction, MIN_EFFORT, MAX_EFFORT)
    right = clamp(base_speed - correction, MIN_EFFORT, MAX_EFFORT)
    drivetrain.set_effort(left, right)

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

def turn_to_heading(target_deg):
    imu.reset_yaw()

    # Tuned starting values
    kp = 0.02
    ki = 0.0
    kd = 0.01

    integral = 0.0
    prev_error = 0.0

    while True:
        current = imu.get_yaw()
        error = normalize_angle(target_deg - current)

        # --- PID terms ---
        integral += error * dt
        integral = clamp(integral, -1.0, 1.0)

        derivative = (error - prev_error) / dt
        prev_error = error

        output = kp * error + ki * integral + kd * derivative
        output = clamp(output, -0.4, 0.4)

        # --- Apply turn ---
        drivetrain.set_effort(-output, output)

        print("error:", round(error, 2), "output:", round(output, 3))

        # --- Exit condition (tight + stable) ---
        if abs(error) < 2 and abs(derivative) < 5:
            break

        time.sleep(dt)

    drivetrain.stop()

def turn_to_heading(target_deg):
    imu.reset_yaw()
    turnController = PIController(kp=0.05, ki=0.01, dt=dt)
    errorList = []

    while True:
        current = imu.get_yaw()
        error = normalize_angle(target_deg - current)
        errorList.append(error)

        if len(errorList) > 0:
            avgError = sum(errorList) / len(errorList)
        else:
            avgError = 0

        if abs(avgError) < 2 and abs(output) < 0.1:
            break

        print("avgError:", avgError)

        output = turnController.update(target_deg, current)
        output = clamp(output, -0.4, 0.4)

        drivetrain.set_effort(-output, output)
        time.sleep(dt)

    drivetrain.stop()

def main():
    drive_distance_cm(5)
    turn_to_heading(90)
    drive_distance_cm(10)
if __name__ == "__main__":
    main()