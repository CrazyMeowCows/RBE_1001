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
LINE_FOLLOWING_GAIN = 0.8
LINE_FOLLOWING_RPM = 600


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
    global count
    set_state(LINE_FOLLOWING)
    count = 0

def scale(val, min, max):
    return (val-min)/(max-min)


# Button Bindings ---------------------------------------------------
controller.buttonB.pressed(start_routine)


# Main Loop ---------------------------------------------------------
while True:
    sleep(20)
    if (state == LINE_FOLLOWING):
        errorL = scale(left_line.value(), MIN_REFLECTIVITY, MAX_REFLECTIVITY)
        errorR = -scale(right_line.value(), MIN_REFLECTIVITY, MAX_REFLECTIVITY)
        sum_error = (errorL + errorR) * LINE_FOLLOWING_GAIN * LINE_FOLLOWING_RPM

        left_motor.spin(FORWARD, LINE_FOLLOWING_RPM - sum_error, RPM)
        right_motor.spin(FORWARD, LINE_FOLLOWING_RPM + sum_error, RPM)

        if (sonar.distance(MM) < 200):
            driveTrain.stop()
            set_state(TURNING)
            count += 1
            print("New Count: " + str(count))
    elif (state == TURNING):
        driveTrain.turn(LEFT, 20, RPM)
        current = gyro.heading()/360

        if (count == 1):
            target = 0.5
        elif (count == 2):
            target = 0

        if (abs((target - current) - math.floor(target - current + 0.5)) < 0.5/360):
            driveTrain.stop()
            if (count == 1):
                set_state(LINE_FOLLOWING)
            else:
                set_state(IDLE)
                print("Heading: " + str(gyro.heading()))
    