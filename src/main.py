# Library Imports
from vex import *

# Define Constants
DRIVE_GEAR_RATIO = 5/1
ARM_GEAR_RATIO = 5/1
WHEEL_RAD_MM = 50.8
TRACK_WIDTH_MM = 320
WHEEL_BASE_MM = 270

# Variable Setup
brain = Brain()
controller = Controller()

def set_state(new_state):
    global state
    brain.screen.print("Current State: " + state + " | New State: " + new_state)
    brain.screen.new_line()
    state = new_state

# Motor and Sensor Definitions
left_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
right_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
arm_motor = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
sonar = Sonar(brain.three_wire_port.e)
bumper = Bumper(brain.three_wire_port.d)

# Motor Setup
left_motor.reset_position()
right_motor.reset_position()
arm_motor.reset_position()

driveTrain = DriveTrain(left_motor, right_motor, 2*math.pi*WHEEL_RAD_MM, TRACK_WIDTH_MM, WHEEL_BASE_MM, MM, DRIVE_GEAR_RATIO)

driveTrain.set_stopping(BrakeType.HOLD)
arm_motor.set_stopping(BrakeType.HOLD)


# Function Definitions 
def sonar_detected(dist_thresh_mm):
    sleep(20)
    return sonar.distance(MM) < dist_thresh_mm

def bumper_pressed():
    driveTrain.drive(FORWARD, 100*DRIVE_GEAR_RATIO, VelocityUnits.PERCENT)
    while (not sonar_detected(75)):
        pass
    driveTrain.stop()
    arm_motor.spin_to_position(20*ARM_GEAR_RATIO, DEGREES, 100, VelocityUnits.PERCENT, True)
    sleep(5000)
    arm_motor.spin_to_position(0*ARM_GEAR_RATIO, DEGREES, 20, VelocityUnits.PERCENT, False)


# Button Bindings
bumper.pressed(bumper_pressed)