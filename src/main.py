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

Vision3__ORANGE = Signature (1, 6149, 6747, 6448, -2375, -2061, -2218, 3.9, 0)
Vision3__LIME = Signature (2, -6089, -4819, -5454, -4097, -3219, -3658, 5.1, 0)
Vision3__LEMON = Signature (3, 1935, 2783, 2358, -3277, -2701, -2990, 3, 0)
Vision3 = Vision(Ports.PORT3, 50, Vision3__LEMON, Vision3__LIME, Vision3__ORANGE)


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
def append_objects(list, tuple, type):
    if tuple:
        for x in tuple:
            if x.width > 20 and x.height > 20:
                list.append((x, type))
                print(type)
    return list

def get_biggest_fruit():
    fruit = []
    fruit = append_objects(fruit, Vision3.take_snapshot(Vision3__LIME), "lime")
    fruit = append_objects(fruit, Vision3.take_snapshot(Vision3__LEMON), "lemon")
    fruit = append_objects(fruit, Vision3.take_snapshot(Vision3__ORANGE), "orange")

    biggest = None
    if len(fruit) > 0:
        biggest = fruit[0]
        for x in fruit:
            if x[0].height > biggest[0].height:
                biggest = x
    return biggest

def find_fruit():
    acc = 20
    while acc > 0:
        fruit_object = get_biggest_fruit()
        
        if fruit_object:
            fruit = fruit_object[0]
            x_error = (fruit.centerX - RESOLUTION_WIDTH/2)*GAIN_X #right is +
            y_error = (fruit.centerY - RESOLUTION_HEIGHT/2)*GAIN_Y #down is +

            left_motor.spin(REVERSE, 20 - x_error, PERCENT)
            right_motor.spin(REVERSE, 20 + x_error, PERCENT)
            effector_motor.spin(REVERSE, y_error, PERCENT)

            print("Targeting: " + fruit_object[1])
            acc = 20
        else:
            print("No Fruit Found")
            acc = max(acc-1, 0)

        sleep(20)

controller.buttonB.pressed(find_fruit)


# Main Loop  --------------------------------------------------------
# while True:
#     forward = controller.axis3.position()
#     sideways = controller.axis4.position()
#     rotation = controller.axis1.position()
#     rButton = (controller.buttonR1.pressing()-controller.buttonR2.pressing())*100

#     left_motor.spin(FORWARD, forward + rotation, PERCENT)
#     right_motor.spin(FORWARD, forward - rotation, PERCENT)
#     center_motor.spin(FORWARD, sideways * DRIVE_GEAR_RATIO, PERCENT)
#     effector_motor.spin(FORWARD, rButton, PERCENT)

#     sleep(20)
    
