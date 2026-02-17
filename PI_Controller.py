from XRPLib.defaults import *
import time

# ---------------------------
# Constants
# ---------------------------
dt = 0.05  # 50 ms loop
MAX_SPEED = 1.0   
MIN_SPEED = -1.0

YAW_BIAS_RATE = 0.00817  # deg/s (measured)

# ---------------------------
# Utilities
# ---------------------------
def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)

# ---------------------------
# PI Controller (speed-based)
# ---------------------------
class PIController:
    def __init__(self, kp, ki, dt):
        self.kp = kp
        self.ki = ki
        self.dt = dt
        self.integral = 0.0

    def update(self, target, measurement):
        error = target - measurement

        # Integrate only small errors (anti-windup)
        if abs(error) < 10.0:
            self.integral += error * self.dt
        
        self.integral = clamp(self.integral, -1.0, 1.0)

        return self.kp * error + self.ki * self.integral

# ---------------------------
# Yaw reading with bias correction
# ---------------------------
def read_yaw(start_time_ms):
    elapsed_s = (time.ticks_ms() - start_time_ms) / 1000.0
    return imu.get_yaw() - YAW_BIAS_RATE * elapsed_s

# ---------------------------
# Motor command (speed-based)
# ---------------------------
def set_motor_speed(base_speed, correction):
    left_speed = clamp(base_speed + correction, MIN_SPEED, MAX_SPEED)
    right_speed = clamp(base_speed - correction, MIN_SPEED, MAX_SPEED)
    drivetrain.set_effort(left_speed, right_speed)

# ---------------------------
# MAIN
# ---------------------------
if __name__ == "__main__":
    imu.calibrate()

    target_yaw = imu.get_yaw()
    base_speed = 0.4

    controller = PIController(
        kp=0.05 / (1 + base_speed),   
        ki=0.012 / (1 + base_speed),
        dt=dt
    )

    start_time_ms = time.ticks_ms()
    print("target yaw:",target_yaw)
    while rangefinder.distance() > 10:
        current_yaw = read_yaw(start_time_ms)
        
        correction = controller.update(target_yaw, current_yaw)
        ## new
        max_correction = 0.3 * abs(base_speed)
        correction = clamp(correction, -max_correction, max_correction)
        backwards = -1.0

        set_motor_speed(base_speed * backwards, correction * backwards)

        print(f"Yaw error: {current_yaw - target_yaw:.2f}")
        time.sleep(dt)

    drivetrain.stop()