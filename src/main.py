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
GYRO_GAIN = 50
GYRO_RPM = 300


# Variable Setup ----------------------------------------------------
brain = Brain()
controller = Controller()
timer = Timer()


# State Machine Definitions -----------------------------------------
IDLE = "IDLE"
LINE_FOLLOWING = "LINE_FOLLOWING"
TURNING = "TURNING"
state = IDLE
count = 0

def set_state(new_state):
    global state
    print("Current State: " + state + " | New State: " + new_state)
    state = new_state


# Motor and Sensor Definitions --------------------------------------
left_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
right_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)
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

driveTrain.set_stopping(BrakeType.HOLD)
arm_motor.set_stopping(BrakeType.HOLD)

gyro.calibrate()
while gyro.is_calibrating():
    sleep(50)


# Function Definitions ----------------------------------------------
def start_routine():
    set_state(LINE_FOLLOWING)

def scale(val, min, max):
    return (val-min)/(max-min)

def continuous_error_deg(target, current):
    target = target/360
    current = current/360
    return abs((target - current) - math.floor(target - current + 0.5))*360


# Button Bindings ---------------------------------------------------
controller.buttonB.pressed(start_routine)


# Main Loop ---------------------------------------------------------
while True:
    sleep(20)
    if (state == LINE_FOLLOWING):
        if (count % 2 == 0):
            target = 0
        elif (count % 2 != 0):
            target = 0.5
        current = gyro.heading()/360
        error = ((target - current) - math.floor(target - current + 0.5))*360*GYRO_GAIN

        left_motor.spin(FORWARD, GYRO_RPM - error, RPM)
        right_motor.spin(FORWARD, GYRO_RPM + error, RPM)

        if (sonar.distance(MM) < 200):
            driveTrain.stop()
            set_state(TURNING)
            count += 1
            print("New Count: " + str(count))
    elif (state == TURNING):
        current = gyro.heading()/360

        if (count % 2 != 0):
            target = 0.5
            driveTrain.turn(RIGHT, 20, RPM)
        elif (count % 2 == 0):
            target = 3/360
            driveTrain.turn(LEFT, 20, RPM)

        if (abs((target - current) - math.floor(target - current + 0.5)) < 3/360):
            driveTrain.stop()
            if (count % 2 != 0):
                set_state(LINE_FOLLOWING)
            else:
                set_state(IDLE)
                print("Heading: " + str(gyro.heading()))
    