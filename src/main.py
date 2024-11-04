# Library Imports ---------------------------------------------------
from vex import *
import math

# Define Constants --------------------------------------------------
DRIVE_GEAR_RATIO = 1/5
ARM_GEAR_RATIO = 1/5
WHEEL_RAD_MM = 50.8
TRACK_WIDTH_MM = 285
WHEEL_BASE_MM = 205

MIN_REFLECTIVITY = 166
MAX_REFLECTIVITY = 2645
LINE_FOLLOWING_GAIN = 0.6
GYRO_GAIN = 0.165


# Variable Setup ----------------------------------------------------
brain = Brain()
controller = Controller()
# timer = Timer()


# State Machine Definitions -----------------------------------------
# IDLE = "IDLE"
# LINE_FOLLOWING = "LINE_FOLLOWING"
# TURNING = "TURNING"
# state = IDLE
# count = 0

# def set_state(new_state):
#     global state
#     print("Current State: " + state + " | New State: " + new_state)
#     state = new_state


# Motor and Sensor Definitions --------------------------------------
left_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)
right_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
arm_motor = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
gyro = Inertial(Ports.PORT20)

sonar = Sonar(brain.three_wire_port.e)
left_line = Line(brain.three_wire_port.h)
right_line = Line(brain.three_wire_port.b)


# Motor and Sensor Setup --------------------------------------------
left_motor.reset_position()
right_motor.reset_position()
arm_motor.reset_position()

driveTrain = DriveTrain(left_motor, right_motor, 2*math.pi*WHEEL_RAD_MM, TRACK_WIDTH_MM, WHEEL_BASE_MM, MM, DRIVE_GEAR_RATIO)
# Use with gear ratio set to 1/5
# driveTrain.drive(FORWARD, 100*DRIVE_GEAR_RATIO, PERCENT)
# driveTrain.drive(FORWARD, 30, RPM)
# driveTrain.drive_for(FORWARD, 20, DistanceUnits.CM, 50*DRIVE_GEAR_RATIO, VelocityUnits.PERCENT, False)

driveTrain.set_stopping(BrakeType.HOLD)
arm_motor.set_stopping(BrakeType.HOLD)


# Function Definitions ----------------------------------------------
# Scale a value from a min->max to 0->1
def scale(val, min, max):
    return (val-min)/(max-min)

# Line follow at a given speed until the ultrasonic detects something
def line_follow_to_wall(dist_to_wall_mm, speed_percent):
    while (sonar.distance(MM) > dist_to_wall_mm):
        errorL = scale(left_line.value(), MIN_REFLECTIVITY, MAX_REFLECTIVITY)
        errorR = -scale(right_line.value(), MIN_REFLECTIVITY, MAX_REFLECTIVITY)
        sum_error = (errorL + errorR) * LINE_FOLLOWING_GAIN * speed_percent

        left_motor.spin(FORWARD, speed_percent + sum_error, PERCENT)
        right_motor.spin(FORWARD, speed_percent - sum_error, PERCENT)
        sleep(20)

    driveTrain.stop()

# Drive forward while maintaing the given target rotation
def gyro_drive_to_wall(target_rot_deg, dist_to_wall_mm, speed_percent):
    target = target_rot_deg/360
    current = gyro.heading()/360

    while (sonar.distance(MM) > dist_to_wall_mm):
        current = gyro.heading()/360
        error = ((target - current) - math.floor(target - current + 0.5))*360*GYRO_GAIN*speed_percent

        left_motor.spin(FORWARD, speed_percent + error, PERCENT)
        right_motor.spin(FORWARD, speed_percent - error, PERCENT)
        sleep(20)

    driveTrain.stop()

# Turn to a given gyro heading
def gyro_turn(target_rot_deg, dir, speed_percent):
    target = target_rot_deg/360
    driveTrain.turn(dir, speed_percent*DRIVE_GEAR_RATIO, PERCENT)
    current = gyro.heading()/360

    while (abs((target - current) - math.floor(target - current + 0.5)) > 1/360):
        current = gyro.heading()/360
        sleep(20)

    driveTrain.stop()

# Originally configured with a state machine, but reconfigured to use blocking sequential code as it 
# significantly increased readablilty and reliability for this use case


# Button Bindings ---------------------------------------------------
def start_routine():
    gyro.calibrate()
    while gyro.is_calibrating():
        sleep(50)
    
    gyro_drive_to_wall(0, 200, 100)
    gyro_turn(180, LEFT, 50)
    gyro_drive_to_wall(180, 200, 100)
    gyro_turn(0, RIGHT, 50)

controller.buttonB.pressed(start_routine)