# Library Imports ---------------------------------------------------
from vex import *
# import math


# Define Constants --------------------------------------------------
DRIVE_GEAR_RATIO = 1/5
ARM_GEAR_RATIO = 1/5
WHEEL_RAD_MM = 50.8
TRACK_WIDTH_MM = 285
WHEEL_BASE_MM = 205
RESOLUTION_WIDTH = 316
RESOLUTION_HEIGHT = 210
GAIN_X = 1
GAIN_Y = 1

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

Vision3__LEMON = Signature(3, 1335, 1737, 1536, -3855, -3589, -3722, 2.6, 0)
Vision3__LIME = Signature(2, -6813, -5985, -6400, -3439, -2829, -3134, 3.4, 0)
Vision3__ORANGE = Signature(1, 5939, 6607, 6273, -2463, -2145, -2304, 1.7, 0)
Vision3 = Vision(Ports.PORT3, 24, Vision3__LEMON, Vision3__LIME, Vision3__ORANGE)


# Motor and Sensor Setup --------------------------------------------
left_motor.reset_position()
right_motor.reset_position()
center_motor.reset_position()
effector_motor.reset_position()

left_motor.set_stopping(BrakeType.HOLD)
right_motor.set_stopping(BrakeType.HOLD)
center_motor.set_stopping(BrakeType.HOLD)
effector_motor.set_stopping(BrakeType.HOLD)


# Function Definitions ----------------------------------------------
def find_fruit():
    acc = 20

    while acc > 0:
        largest_object = Vision3.largest_object()
        if largest_object.exists:
            acc = 20
        else:
            acc = max(acc-1, 0)

        x_error = (largest_object.centerX - RESOLUTION_WIDTH/2)*GAIN_X #right is +
        y_error = (largest_object.centerY - RESOLUTION_HEIGHT/2)*GAIN_Y #down is +

        left_motor.spin(FORWARD, 20 + x_error, PERCENT)
        right_motor.spin(FORWARD, 20 - x_error, PERCENT)
        effector_motor.spin(REVERSE, y_error, PERCENT)

        sleep(20)


controller.buttonB.pressed(find_fruit)


# Main Loop  --------------------------------------------------------
while True:
    forward = controller.axis3.position()
    sideways = controller.axis4.position()
    rotation = controller.axis1.position()
    rButton = (controller.buttonR1.pressing()-controller.buttonR2.pressing())*100

    left_motor.spin(FORWARD, forward + rotation, PERCENT)
    right_motor.spin(FORWARD, forward - rotation, PERCENT)
    center_motor.spin(FORWARD, sideways * DRIVE_GEAR_RATIO, PERCENT)
    effector_motor.spin(FORWARD, rButton, PERCENT)

    sleep(20)
    
