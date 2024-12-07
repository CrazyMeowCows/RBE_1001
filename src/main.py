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
GAIN_X = 0.3
GAIN_Y = 1
MIN_REFLECTIVITY = 166
MAX_REFLECTIVITY = 2645
LINE_FOLLOWING_GAIN = 0.3
GYRO_GAIN = 0.05


# Variable Setup ----------------------------------------------------
brain = Brain()
controller = Controller()


# Motor and Sensor Definitions --------------------------------------
left_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)
right_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
center_motor = Motor(Ports.PORT4, GearSetting.RATIO_18_1, True)
elbow_motor = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
effector_motor = Motor(Ports.PORT5, GearSetting.RATIO_18_1, False)

gyro = Inertial(Ports.PORT11)

left_line = Line(brain.three_wire_port.e)
right_line = Line(brain.three_wire_port.f)
sonar = Sonar(brain.three_wire_port.c)

Vision5__ORANGE_FRUIT = Signature (1, 6719, 7239, 6978, -2555, -2215, -2384, 5.6, 0)
Vision5__LIME_FRUIT = Signature (2, -6089, -4819, -5454, -4097, -3219, -3658, 5.4, 0)
Vision5__LEMON_FRUIT = Signature (3, 2041, 2499, 2270, -3789, -3459, -3624, 6, 0)
Vision3 = Vision(Ports.PORT20, 38, Vision5__ORANGE_FRUIT, Vision5__LIME_FRUIT, Vision5__LEMON_FRUIT)


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
                # print(type)
    return list

#Returns the vision object of the biggest fruit
def get_biggest_fruit():
    fruit = []
    fruit = append_objects(fruit, Vision3.take_snapshot(Vision5__LIME_FRUIT), "lime")
    fruit = append_objects(fruit, Vision3.take_snapshot(Vision5__LEMON_FRUIT), "lemon")
    fruit = append_objects(fruit, Vision3.take_snapshot(Vision5__ORANGE_FRUIT), "orange")

    result = None
    if len(fruit) > 0:
        result = fruit[0]
        for x in fruit:
            if x[0].height > result[0].height:
                result = x
    return result

#Returns the vision object of the closest to center fruit
def get_centered_fruit():
    fruit = []
    fruit = append_objects(fruit, Vision3.take_snapshot(Vision5__LIME_FRUIT), "lime")
    fruit = append_objects(fruit, Vision3.take_snapshot(Vision5__LEMON_FRUIT), "lemon")
    fruit = append_objects(fruit, Vision3.take_snapshot(Vision5__ORANGE_FRUIT), "orange")

    result = None
    if len(fruit) > 0:
        result = fruit[0]
        for x in fruit:
            if abs(x[0].centerX - RESOLUTION_WIDTH/2) < abs(result[0].centerX - RESOLUTION_WIDTH/2):
                result = x
    return result


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
    acc = 200
    while acc > 0 and not controller.buttonY.pressing():
        fruit_object = get_biggest_fruit()

        if fruit_object:
            fruit = fruit_object[0]
            x_error = (fruit.centerX - RESOLUTION_WIDTH/2)*GAIN_X #right is +
            # y_error = -(fruit.centerY - RESOLUTION_HEIGHT/2 - 10)*GAIN_Y #down is +

            left_motor.spin(FORWARD, 30 - x_error, PERCENT)
            right_motor.spin(FORWARD, 30 + x_error, PERCENT)
            # effector_motor.spin(REVERSE, y_error, PERCENT)

            print("Targeting: " + fruit_object[1])
            print("Torque: " + str(effector_motor.torque()))
            if (effector_motor.torque() < 0.9):
                acc = 200
            else:
                acc = max(acc-10, 0)
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

    while ((left_motor.position(RotationUnits.REV)+right_motor.position(RotationUnits.REV))*math.pi*WHEEL_RAD_CM*DRIVE_GEAR_RATIO < dist_to_travel_cm):
        errorL = -scale(left_line.value(), MIN_REFLECTIVITY, MAX_REFLECTIVITY)
        errorR = scale(right_line.value(), MIN_REFLECTIVITY, MAX_REFLECTIVITY)
        line_error = (errorL + errorR) * LINE_FOLLOWING_GAIN * speed_percent

        left_motor.spin(FORWARD, speed_percent - line_error, PERCENT)
        right_motor.spin(FORWARD, speed_percent + line_error, PERCENT)
        sleep(20)

    left_motor.stop()
    right_motor.stop()

# Raise the arm to a tree height and start intaking
ARM_LEVELS = {
  "travel": 0,
  "tree_1": 13,
  "tree_2": 26,
  "tree_3": 41,
  "box": 23,
  "turn": 56
}
def set_arm(arm_level, intake_speed):
    elbow_motor.set_stopping(BrakeType.BRAKE)
    print("A")
    elbow_motor.spin_to_position(ARM_LEVELS[arm_level]/ARM_GEAR_RATIO, DEGREES, 70, RPM, True)
    if(arm_level == "travel"):
        elbow_motor.stop()
        elbow_motor.set_stopping(BrakeType.COAST)
    print("B")
    effector_motor.spin(FORWARD, intake_speed, PERCENT)

# Drive forward a set distance based purely off drivetrain encoders
def drive_forward(dist_to_travel_cm, speed_percent):
    left_motor.reset_position()
    right_motor.reset_position()
    left_motor.spin(FORWARD, speed_percent, PERCENT)
    right_motor.spin(FORWARD, speed_percent, PERCENT)

    while (abs((left_motor.position(RotationUnits.REV)+right_motor.position(RotationUnits.REV))*math.pi*WHEEL_RAD_CM*DRIVE_GEAR_RATIO) < dist_to_travel_cm):
        sleep(20)

    left_motor.stop()
    right_motor.stop()

# Reverse until the sonar reaches a distance threshold
def reverse_to_wall(dist_to_wall_cm, speed_percent):
    left_motor.spin(REVERSE, speed_percent, PERCENT)
    right_motor.spin(REVERSE, speed_percent, PERCENT)

    if (dist_to_wall_cm > 0):
        while (sonar.distance(DistanceUnits.CM) > dist_to_wall_cm):
            sleep(20)
    else:
        while (sonar.distance(DistanceUnits.CM) > 5):
            sleep(20)
        sleep(2000)

    left_motor.stop()
    right_motor.stop()

# Calibrate gyro and eliminate backlash
def calibrate_robot():
    elbow_motor.set_stopping(BrakeType.COAST)
    print("Elbow Motor Temp: " + str(elbow_motor.temperature()))
    gyro.calibrate()
    while (gyro.is_calibrating()):
        sleep(50)

    left_motor.reset_position()
    right_motor.reset_position()
    center_motor.reset_position()
    elbow_motor.reset_position()
    effector_motor.reset_position()


# The routine to be executed when button is pressed --------------------------------
def auton_routine():
    calibrate_robot()
    line_follow_dist_cm(80, 60)
    set_arm("turn", 0)
    gyro_turn(-90, 50)
    reverse_to_wall(8, 20)
    set_arm("tree_2", 100)
    sleep(500)
    find_fruit()
    gyro_turn(-90, 50)
    reverse_to_wall(10, 30)
    set_arm("travel", 0)
    gyro_turn(-180, 50)
    line_follow_dist_cm(69, 60)
    gyro_turn(-90, 50)
    set_arm("travel", -30)
    sleep(1000)
    set_arm("travel", 0)
    reverse_to_wall(10, 30)
    gyro_turn(-2, 50)
    reverse_to_wall(0, 10)

    calibrate_robot()
    line_follow_dist_cm(80+96, 60)
    set_arm("turn", 0)
    gyro_turn(-90, 50)
    reverse_to_wall(8, 20)
    set_arm("tree_2", 100)
    sleep(500)
    find_fruit()
    gyro_turn(-90, 50)
    reverse_to_wall(10, 30)
    set_arm("travel", 0)
    gyro_turn(-180, 50)
    line_follow_dist_cm(69+96, 60)
    gyro_turn(-90, 50)
    set_arm("travel", -30)
    sleep(1000)
    set_arm("travel", 0)
    reverse_to_wall(10, 30)
    gyro_turn(-2, 50)
    reverse_to_wall(0, 10)

controller.buttonB.pressed(auton_routine)

# follow line a set distance using line sensors and IMU to stay steady

# set height to proper level
    # locate fruit (within center range, which one is closest)

# move toward fruit using proportional control
# until fruit harvested(torque reaches certain threshold)

# return to line(rotate w IMU, drive to line, turn to proper direction)
# drive to base(detect wall with ultrasonic)

#drop fruit to side  
# recalibrate with wall
#    
