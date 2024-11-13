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
center_motor = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
effector_motor = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)

Vision3__LEMON = Signature (3, 3267, 3959, 3613, -3503, -3121, -3312, 2.500, 0)
Vision3__LIME = Signature (2, -5553, -4205, -4879, -3549, -1999, -2774, 2.500, 0)
Vision3__ORANGE = Signature (1, 6345, 8529, 7437, -2349, -2033, -2191, 2.500, 0)
Vision3 = Vision (Ports.PORT3, 72, Vision3__LEMON, Vision3__LIME, Vision3__ORANGE)

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
def find_fruit():
    lime_objects = Vision3.take_snapshot(Vision3__LIME)
    lemon_objects = Vision3.take_snapshot(Vision3__LEMON)
    if lime_objects:
        print("Lime Detected")
    if lemon_objects:
        print("Lemon Detected")

controller.buttonB.pressed(find_fruit)

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
    sleep(20)
    
