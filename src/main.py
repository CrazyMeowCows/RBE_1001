# Library Imports ---------------------------------------------------
from vex import *
import math


# Define Constants --------------------------------------------------
DRIVE_GEAR_RATIO = 1/5
ARM_GEAR_RATIO = 1/5
WHEEL_RAD_CM = 5.08
TRACK_WIDTH_MM = 285
WHEEL_BASE_MM = 205
RESOLUTION_WIDTH = 316
RESOLUTION_HEIGHT = 210
GAIN_X = 1
GAIN_Y = 1
MIN_REFLECTIVITY = 166
MAX_REFLECTIVITY = 2645
LINE_FOLLOWING_GAIN = 0.6
GYRO_GAIN = 0.05


# Variable Setup ----------------------------------------------------
brain = Brain()
controller = Controller()


# Motor and Sensor Definitions --------------------------------------
left_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)
right_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
center_motor = Motor(Ports.PORT4, GearSetting.RATIO_18_1, True)
elbow_motor = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
effector_motor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)

gyro = Inertial(Ports.PORT11)

left_line = Line(brain.three_wire_port.a)
right_line = Line(brain.three_wire_port.b)

Vision3__LEMON = Signature(3, 1335, 1737, 1536, -3855, -3589, -3722, 2.6, 0)
Vision3__LIME = Signature(2, -6813, -5985, -6400, -3439, -2829, -3134, 3.4, 0)
Vision3__ORANGE = Signature(1, 5939, 6607, 6273, -2463, -2145, -2304, 1.7, 0)
Vision3 = Vision(Ports.PORT20, 24, Vision3__LEMON, Vision3__LIME, Vision3__ORANGE)


# Motor and Sensor Setup --------------------------------------------
left_motor.set_stopping(BrakeType.HOLD)
right_motor.set_stopping(BrakeType.HOLD)
center_motor.set_stopping(BrakeType.HOLD)
effector_motor.set_stopping(BrakeType.HOLD)


# Helper Functions -----------------------------------------------------------------
#Append a tuple of vision objects to the given list and return it
def append_objects(list, tuple, type):
    if tuple:
        for x in tuple:
            if x.width > 10 and x.height > 20:
                list.append((x, type))
                print(type)
    return list

#Returns the vision object of the biggest fruit
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

# Scale a value from a min->max to 0->1
def scale(val, min, max):
    return (val-min)/(max-min)

# Get continous angle error from discontinuous angles
def angle_error_deg(target, current):
    diff = math.radians(target) - math.radians(current)
    return math.degrees(math.atan2(math.sin(diff), math.cos(diff)))


# Function Definitions -------------------------------------------------------------
#Drive towards the biggest fruit the robot can currently see
def find_fruit():
    acc = 20
    while acc > 0 and not controller.buttonY.pressing():
        fruit_object = get_biggest_fruit()

        if fruit_object:
            fruit = fruit_object[0]
            x_error = -(fruit.centerX - RESOLUTION_WIDTH/2)*GAIN_X #right is +
            # y_error = -(fruit.centerY - RESOLUTION_HEIGHT/2 - 10)*GAIN_Y #down is +

            left_motor.spin(REVERSE, 20 - x_error, PERCENT)
            right_motor.spin(REVERSE, 20 + x_error, PERCENT)
            # effector_motor.spin(REVERSE, y_error, PERCENT)

            print("Targeting: " + fruit_object[1])
            acc = 20
        else:
            print("No Fruit Found")
            acc = max(acc-1, 0)

        sleep(20)

    left_motor.stop()
    right_motor.stop()
    effector_motor.stop()

# Turn with unspecified direction to target heading in degrees
def gyro_turn(target_rot_deg, speed_percent):
    current = gyro.heading()

    while (abs(angle_error_deg(target_rot_deg, current)) > 1):
        error = angle_error_deg(target_rot_deg, current) * GYRO_GAIN
        current = gyro.heading()

        left_motor.spin(FORWARD, speed_percent * error, PERCENT)
        right_motor.spin(FORWARD, speed_percent * -error, PERCENT)
        sleep(20)

    left_motor.stop()
    right_motor.stop()

# Line follow at a given speed for a given distance based off drivetrain encoders
def line_follow_dist_cm(dist_to_travel_cm, speed_percent):
    left_motor.reset_position()
    right_motor.reset_position()

    while ((left_motor.position(RotationUnits.REV)+right_motor.position(RotationUnits.REV))*math.pi*WHEEL_RAD_CM/DRIVE_GEAR_RATIO < dist_to_travel_cm):
        errorL = scale(left_line.value(), MIN_REFLECTIVITY, MAX_REFLECTIVITY)
        errorR = -scale(right_line.value(), MIN_REFLECTIVITY, MAX_REFLECTIVITY)
        sum_error = (errorL + errorR) * LINE_FOLLOWING_GAIN * speed_percent

        left_motor.spin(FORWARD, speed_percent + sum_error, PERCENT)
        right_motor.spin(FORWARD, speed_percent - sum_error, PERCENT)
        sleep(20)

    left_motor.stop()
    right_motor.stop()

# Raise the arm to a tree height and start intaking
ARM_LEVELS = { #TODO: Tune These
  "travel": 20,
  "tree_0": 15,
  "tree_1": 25,
  "tree_2": 40,
  "tree_3": 45
}
def set_arm(arm_level, intake_speed):
    elbow_motor.spin_to_position(ARM_LEVELS[arm_level]/ARM_GEAR_RATIO, DEGREES, 100, RPM, True)
    effector_motor.spin(FORWARD, intake_speed, PERCENT)

# Drive forward a set distance based purely off drivetrain encoders
def drive_forward(distance_cm, speed_percent):
    left_motor.reset_position()
    right_motor.reset_position()
    left_motor.spin(FORWARD, speed_percent, PERCENT)
    right_motor.spin(FORWARD, speed_percent, PERCENT)

    while ((left_motor.position(RotationUnits.REV)+right_motor.position(RotationUnits.REV))*math.pi*WHEEL_RAD_CM/DRIVE_GEAR_RATIO < distance_cm):
        sleep(20)

    left_motor.stop()
    right_motor.stop()

# Calibrate gyro and eliminate backlash
def calibrate_robot():
    elbow_motor.set_stopping(BrakeType.COAST)
    gyro.calibrate()
    while (gyro.is_calibrating()):
        sleep(50)
    left_motor.spin(REVERSE, 20, PERCENT)
    right_motor.spin(REVERSE, 20, PERCENT)

    sleep(0.5)

    left_motor.reset_position()
    right_motor.reset_position()
    center_motor.reset_position()
    elbow_motor.reset_position()
    effector_motor.reset_position()

    left_motor.stop()
    right_motor.stop()
    elbow_motor.set_stopping(BrakeType.HOLD)


# The routine to be executed when button is pressed --------------------------------
def auton_routine():
    calibrate_robot()
    set_arm("travel", 0)
    line_follow_dist_cm(27.5, 50)
    gyro_turn(90, 50)
    set_arm("TREE_2", 100)
    drive_forward(10, 40)
    set_arm("TREE_2", 0)
    # find_fruit()

controller.buttonB.pressed(auton_routine)
    
