from XRPLib.defaults import *
import time
import math

## -- Constants --
dt = 0.05
MAX_EFFORT = 1.0
MIN_EFFORT = -1.0
ACCEL_FR = 0.05 ## Frequency of additional effort
ACCEL_MG = 0.1 ## Amount of additional effort (1 is global max effort)

deadzone = 0.18

YAW_BIAS = 0.00817 ## deg/s (left) CURRENTLY UNUSED

## -- Utilities --
def clamp(val, min_val, max_val): ##Min/max clamp
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
    
    
    

## -- PI Controller --
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
        
    
    ## Read encoder docs, gearbox ratios, encoder return (ticks or degrees, wrapped -180,180? 0,360? 0, 1440?)
        
        
    ## ChatGPT Slop >>>>>>
        
## ------- Drive till        
WHEEL_CIRCUMFERENCE = 21.0   # cm (measure !!!!!!!!!!)
TICKS_PER_REV = 360          # confirm for XRP (might be more around 540 ticks)

def drive_distance_cm(target_cm):
    drivetrain.reset_left_encoder()
    imu.reset_yaw()

    driveController = PIController(kp=0.05, ki=0.01, dt=dt)
    headingController = PIController(kp=0.03, ki=0.0, dt=dt)

    target_heading = 0  # hold straight

    while True:
        # --- Distance ---
        ticks = drivetrain.get_left_encoder_position()
        distance = (ticks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE
        error = target_cm - distance

        if abs(error) < 0.5:  # 0.5 cm tolerance
            break

        forward = driveController.update(target_cm, distance)

        # --- Heading Correction ---
        current_yaw = imu.get_yaw()
        turn = headingController.update(target_heading, current_yaw)

        # --- Combine ---
        left = forward - turn
        right = forward + turn

        # Clamp motor output
        left = clamp(left, -1.0, 1.0)
        right = clamp(right, -1.0, 1.0)

        drivetrain.set_effort(left, right)

        time.sleep(dt)

    drivetrain.stop()
    
## ------- Turn till
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
        if avgError < 2:
            break
        print("avgError: ",avgError)
        

        output = turnController.update(target_deg, current)
        if (output > 0) and (output < deadzone):
            output = deadzone
        if (output < 0) and (output > -deadzone):
            output = -deadzone
        drivetrain.set_effort(-output, output)
        time.sleep(dt)

    drivetrain.stop()
        
        
def main():
    print('turning...')
    turn_to_heading(90)
    print("done turning...")
    time.sleep(3)
    print("driving...")
    drive_distance_cm(10)
    print("done driving...")
    
if __name__ == '__main__':
    main()

    

