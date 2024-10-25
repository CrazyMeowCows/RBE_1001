# Library imports
from vex import *

# Define Constants
DRIVE_GEAR_RATIO = 5/1
ARM_GEAR_RATIO = 5/1
WHEEL_RAD_MM = 50.8
TRACK_WIDTH_MM = 320
WHEEL_BASE_MM = 270

# Brain and controller setup
brain = Brain()
controller = Controller()

# State machine definitions
timer = Timer()
IDLE = "idle"
DRIVING = "driving"
ARM_MOVING_UP = "arm_moving_up"
ARM_MOVING_DOWN = "arm_moving_down"
state = IDLE

# Motor and sensor definitions
left_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
right_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
arm_motor = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)
sonar = Sonar(brain.three_wire_port.e)
bumper = Bumper(brain.three_wire_port.d)

# Drivetrain setup
left_motor.reset_position()
right_motor.reset_position()
arm_motor.reset_position()

driveTrain = DriveTrain(left_motor, right_motor, 2*math.pi*WHEEL_RAD_MM, TRACK_WIDTH_MM, WHEEL_BASE_MM, MM, DRIVE_GEAR_RATIO)

driveTrain.set_stopping(BrakeType.HOLD)
arm_motor.set_stopping(BrakeType.HOLD)

# Function Definitions
def set_state(new_state):
    global state
    brain.screen.print("Current State: " + state + " | New State: " + new_state)
    brain.screen.new_line()
    state = new_state

def button_b_pressed():
    driveTrain.drive_for(FORWARD, 50, DistanceUnits.CM, 30, VelocityUnits.PERCENT, False)
    set_state(DRIVING)

def move_arm_to_home():
    arm_motor.spin_to_position(0*ARM_GEAR_RATIO, DEGREES, 100, VelocityUnits.PERCENT, False)
    set_state(ARM_MOVING_DOWN)

# Button Bindings

controller.buttonB.pressed(button_b_pressed)

# The main loop
while True:
    if (state == DRIVING and driveTrain.is_done()):
        arm_motor.spin_to_position(20*ARM_GEAR_RATIO, DEGREES, 100, VelocityUnits.PERCENT, False)
        set_state(ARM_MOVING_UP)
    elif (state == ARM_MOVING_UP and arm_motor.is_done()):
        set_state(IDLE)
        timer.event(move_arm_to_home, 5000)
    elif (state == ARM_MOVING_DOWN and arm_motor.is_done()):
        set_state(IDLE)
