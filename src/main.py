# Library Imports ---------------------------------------------------
from vex import *
import math

# Define Constants --------------------------------------------------
DRIVE_GEAR_RATIO = 1/5
ARM_GEAR_RATIO = 1/5
WHEEL_RAD_MM = 50.8
TRACK_WIDTH_MM = 285
WHEEL_BASE_MM = 205


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
center_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False) #TODO: SET PORTS
effector_motor = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True) #TODO: SET PORTS


# Motor and Sensor Setup --------------------------------------------
left_motor.reset_position()
right_motor.reset_position()
center_motor.reset_position()
effector_motor.reset_position()

driveTrain = DriveTrain(left_motor, right_motor, 2*math.pi*WHEEL_RAD_MM, TRACK_WIDTH_MM, WHEEL_BASE_MM, MM, DRIVE_GEAR_RATIO)
# Use with gear ratio set to 1/5
# driveTrain.drive(FORWARD, 100*DRIVE_GEAR_RATIO, PERCENT)
# driveTrain.drive(FORWARD, 30, RPM)
# driveTrain.drive_for(FORWARD, 20, DistanceUnits.CM, 50*DRIVE_GEAR_RATIO, VelocityUnits.PERCENT, False)

driveTrain.set_stopping(BrakeType.HOLD)
center_motor.set_stopping(BrakeType.HOLD)
effector_motor.set_stopping(BrakeType.HOLD)


# Function Definitions ----------------------------------------------


# Main Loop  --------------------------------------------------------
while True:
    forward = controller.axis3.position()
    sideways = controller.axis4.position()
    rotation = controller.axis1.position()

    left_motor.spin(FORWARD, forward + rotation, PERCENT)
    right_motor.spin(FORWARD, forward - rotation, PERCENT)
    center_motor.spin(FORWARD, sideways, PERCENT)

    rButton = 0
    if (controller.buttonR1.pressing()):
        rButton += 100
    if (controller.buttonR2.pressing()):
        rButton += -100

    effector_motor.spin(FORWARD, rButton, PERCENT)
    
