# Library Imports ---------------------------------------------------
from vex import *

# Define Constants --------------------------------------------------
DRIVE_GEAR_RATIO = 1/5
ARM_GEAR_RATIO = 1/5
WHEEL_RAD_MM = 50.8
TRACK_WIDTH_MM = 285
WHEEL_BASE_MM = 205


# Variable Setup ----------------------------------------------------
brain = Brain()
controller = Controller()
timer = Timer()


# State Machine Definitions -----------------------------------------
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


# Motor and Sensor Definitions --------------------------------------
left_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
right_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
arm_motor = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
gyro = Inertial(Ports.PORT20)

sonar = Sonar(brain.three_wire_port.e) #TODO: Set Ports
left_line = Line(brain.three_wire_port.a) #TODO: Set Ports
right_line = Line(brain.three_wire_port.b) #TODO: Set Ports


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
    pass


# Button Bindings ---------------------------------------------------
controller.buttonB.pressed(start_routine)


# Main Loop ---------------------------------------------------------
while True:
    sleep(20)