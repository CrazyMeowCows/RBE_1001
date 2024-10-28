# Library Imports
from vex import *

# Define Constants
DRIVE_GEAR_RATIO = 1/5
ARM_GEAR_RATIO = 1/5
WHEEL_RAD_MM = 50.8
TRACK_WIDTH_MM = 285
WHEEL_BASE_MM = 205

# Variable Setup
brain = Brain()
controller = Controller()
timer = Timer()

# State Machine Definitions
IDLE = "idle"
DRIVING = "driving"
ARM_MOVING_UP = "arm_moving_up"
ARM_MOVING_DOWN = "arm_moving_down"
state = IDLE

def set_state(new_state):
    global state
    brain.screen.new_line()
    brain.screen.print("Current State: " + state + " | New State: " + new_state)
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
def bumper_pressed():
    if (state == IDLE):
        # Use with gear ratio set to 1
        # driveTrain.drive(FORWARD, 100, PERCENT)
        # driveTrain.drive(FORWARD, 30*DRIVE_GEAR_RATIO, RPM)
        # driveTrain.drive_for(FORWARD, 20*DRIVE_GEAR_RATIO, DistanceUnits.CM, 50, VelocityUnits.PERCENT, False)
        
        # Use with gear ratio set to 1/5
        # driveTrain.drive(FORWARD, 100*DRIVE_GEAR_RATIO, PERCENT)
        driveTrain.drive(FORWARD, 30, RPM)
        # driveTrain.drive_for(FORWARD, 20, DistanceUnits.CM, 50*DRIVE_GEAR_RATIO, VelocityUnits.PERCENT, False)

        set_state(DRIVING)

def move_arm_to_home():
    arm_motor.spin_to_position(0/ARM_GEAR_RATIO, DEGREES, 20, VelocityUnits.PERCENT, False)
    set_state(ARM_MOVING_DOWN)

def sonar_detected(dist_thresh_mm):
    return sonar.distance(MM) <= dist_thresh_mm


# Button Bindings
controller.buttonB.pressed(bumper_pressed)


# State Machine
while True:
    if (state == DRIVING and sonar_detected(75)):
        driveTrain.stop()
        arm_motor.spin_to_position(45/ARM_GEAR_RATIO, DEGREES, 50, VelocityUnits.PERCENT, False)
        set_state(ARM_MOVING_UP)
    elif (state == ARM_MOVING_UP and arm_motor.is_done()):
        timer.event(move_arm_to_home, 2000)
        set_state(IDLE)
    elif (state == ARM_MOVING_DOWN and arm_motor.is_done()):
        set_state(IDLE)

    brain.screen.clear_line()
    brain.screen.print_at("Sonar: " + str(sonar.distance(MM)), x=300, y=15)
    brain.screen.print_at("Motor: " + str(arm_motor.command()), x=5, y=15)
    sleep(20)