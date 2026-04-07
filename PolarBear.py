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

def drive_distance_cm(target_cm): ## 1ft ~ 30.5cm
    imu.reset_yaw()
    drivetrain.reset_encoder_position()

    driveController = PIController(kp=0.1, ki=0.01, dt=dt)
    headingController = PIController(kp=0.03, ki=0.01, dt=dt)

    target_heading = 0
    errorList = []
    distList = []

    while rangefinder.distance()>10:
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
        
        max_turn = 0.4 * abs(forward)
        turn = clamp(turn, -max_turn, max_turn)

        left = forward - turn
        right = forward + turn

        left = clamp(left, -1.0, 1.0)
        right = clamp(right, -1.0, 1.0)

        drivetrain.set_effort(left, right)

        print("Dist:", round(distance, 2),
              "Err:", round(error, 2),
              "Vel:", round(velocity, 3))
        print(drivetrain.get_left_encoder_position())
        time.sleep(dt)

    drivetrain.stop()

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

def main():
    drive_distance_cm(5)
    turn_to_heading(90)
    drive_distance_cm(10)
if __name__ == "__main__":
    main()