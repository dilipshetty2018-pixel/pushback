# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       sdumb                                                        #
# 	Created:      11/11/2025, 7:46:44 PM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

from vex import *

Kp = 0.5  # Proportional
Ki = 0.01  # Integral   
Kd = 0.1  # Derivative
HEADING_CORRECTION_GAIN = 0.2 #this is coeffient for error correction gain 
    #all 4 above values need TUNING

WHEEL_DIAMETER_MM = 101.6  
DISTANCE_PER_ROTATION_MM = 3.1415 * WHEEL_DIAMETER_MM
TRACK_WIDTH_MM = 228.6  # distance between left and right wheels center to center

DEADBAND = 5

CURRENT_LM_POWER = 0
CURRENT_RM_POWER = 0

RAMP_RATE = 9 #used for ramping (lower = smoother, higher = jumpy)

BRAKE_MODE = False  # False = coast, True = brake

# Solenoid state tracking
loader_piston_extended = False
a_button_last = False

descorer_piston_extended = False
b_button_last = False

#constants defined above

brain = Brain()
controller = Controller()

# Drive motors - update Ports to match wiring on robot.
left_motor1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
left_motor2 = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
right_motor1 = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
right_motor2 = Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)

input_motor = Motor(Ports.PORT3, False)  # reversed flag flipped so spin directions are swapped
output_motor = Motor(Ports.PORT13, False)

# Pneumatics solenoids (3-wire DigitalOut attached to Brain 3-wire ports)
loader_piston_solenoid = DigitalOut(brain.three_wire_port.a)
descorer_piston_solenoid = DigitalOut(brain.three_wire_port.b)
"""
    wheelTravel = the distance the robot travels with one full rotation of the wheels
    trackWidth = the distance between the left and right wheels (we are using for turns, and curves)
    wheelBase = the distance between the front and back wheels (we are using for turns, and curves)
"""
# DriveTrain object not used (we control all four motors directly). Removed to avoid unused variable.

def drive_frontback_pid(target_distance_mm):
    # Reset motor positions
    # reset encoder positions for all drive motors
    left_motor1.set_position(0, DEGREES)
    left_motor2.set_position(0, DEGREES)
    right_motor1.set_position(0, DEGREES)
    right_motor2.set_position(0, DEGREES)

    # PID variables
    error = 0
    previous_error = 0
    integral = 0
    derivative = 0

    # Convert mm to degrees to rotate using the configured wheel travel constant
    degrees_to_rotate = (target_distance_mm / DISTANCE_PER_ROTATION_MM) * 360

    while True:
        # average motor position per side, then average both sides for overall travel
        left_pos = (left_motor1.position(DEGREES) + left_motor2.position(DEGREES)) / 2
        right_pos = (right_motor1.position(DEGREES) + right_motor2.position(DEGREES)) / 2
        current_position = (left_pos + right_pos) / 2
        error = degrees_to_rotate - current_position
        integral += error
        derivative = error - previous_error

        # PID output
        power = (Kp * error) + (Ki * integral) + (Kd * derivative)

        # heading correction based on the average of both encoders on each side
        correction = (left_pos - right_pos) * HEADING_CORRECTION_GAIN

        # power with corrections
        # apply power to both motors on each side
        left_motor1.spin(FORWARD, power - correction, VOLT)
        left_motor2.spin(FORWARD, power - correction, VOLT)
        right_motor1.spin(FORWARD, power + correction, VOLT)
        right_motor2.spin(FORWARD, power + correction, VOLT)

        # exit condition
        if abs(error) < 5:
            break

        previous_error = error
        wait(20, MSEC)

    left_motor1.stop()
    left_motor2.stop()
    right_motor1.stop()
    right_motor2.stop()

def autonomous():
    #Route plan
    """
    drive forward
    turn left 90 degrees
    drive forward to vertical loading tube
    pick up 3 balls
    drive backwards to long goal
    place all balls in long goal
    """
    
    # NOTE: These values should be tuned by testing on the actual robot
    drive_frontback_pid(820)  # drive forward 820 mm
   
    left_motor1.stop(BRAKE)
    left_motor2.stop(BRAKE)
    right_motor1.stop(BRAKE)
    right_motor2.stop(BRAKE)
    


def driver_control():
    global CURRENT_LM_POWER, CURRENT_RM_POWER, BRAKE_MODE, RAMP_RATE
    global loader_piston_extended, a_button_last, descorer_piston_extended, b_button_last

    while True:
        left_power = controller.axis2.position()
        right_power = controller.axis3.position()

        if abs(left_power) < DEADBAND:
            left_power = 0
        if abs(right_power) < DEADBAND:
            right_power = 0
        #above to get rid of the deadband of the joysticks  
        # If both sticks are in the same direction (and non-zero), average them
        # to avoid a slight mismatch causing the robot to drift.
        if left_power * right_power > 0:
            avg = (left_power + right_power) / 2
            target_left_power = avg
            target_right_power = avg
        else:
            target_left_power = left_power
            target_right_power = right_power

        # toggle brake mode
        if controller.buttonX.pressing():
            BRAKE_MODE = not BRAKE_MODE
            controller.screen.clear_screen()
            controller.screen.print("Brake Mode: ", BRAKE_MODE)
            wait(200, MSEC)  #debounce delay

        # Loader Piston control
        a_button_pressed = controller.buttonA.pressing()
        if a_button_pressed and not a_button_last:
            loader_piston_extended = not loader_piston_extended
            if loader_piston_extended:
                loader_piston_solenoid.set(True)
            else:
                loader_piston_solenoid.set(False)
        a_button_last = a_button_pressed

        # Descorer Piston control
        b_button_pressed = controller.buttonB.pressing()
        if b_button_pressed and not b_button_last:
            descorer_piston_extended = not descorer_piston_extended
            if descorer_piston_extended:
                descorer_piston_solenoid.set(True)
            else:
                descorer_piston_solenoid.set(False)
        b_button_last = b_button_pressed

        # Ramping logic below
        # smoothly ramp left motor power toward target
        if CURRENT_LM_POWER < target_left_power: #if current is below target, increase
            CURRENT_LM_POWER += RAMP_RATE #prevent overshooting
            if CURRENT_LM_POWER > target_left_power:
                CURRENT_LM_POWER = target_left_power
        elif CURRENT_LM_POWER > target_left_power:#if current is above target, decrease
            CURRENT_LM_POWER -= RAMP_RATE# prevent undershooting
            if CURRENT_LM_POWER < target_left_power:
                CURRENT_LM_POWER = target_left_power

        # smoothly ramp right motor power toward target
        if CURRENT_RM_POWER < target_right_power:#if current is below target, increase
            CURRENT_RM_POWER += RAMP_RATE #prevent overshooting
            if CURRENT_RM_POWER > target_right_power:
                CURRENT_RM_POWER = target_right_power
        elif CURRENT_RM_POWER > target_right_power:#if current is above target, decrease
            CURRENT_RM_POWER -= RAMP_RATE# prevent undershooting
            if CURRENT_RM_POWER < target_right_power:
                CURRENT_RM_POWER = target_right_power

        # spin both motors on each side with the ramped power
        left_motor1.spin(FORWARD, CURRENT_LM_POWER, PERCENT)
        left_motor2.spin(FORWARD, CURRENT_LM_POWER, PERCENT)
        right_motor1.spin(FORWARD, CURRENT_RM_POWER, PERCENT)
        right_motor2.spin(FORWARD, CURRENT_RM_POWER, PERCENT)
        
        if target_left_power == 0 and target_right_power == 0:
            if BRAKE_MODE: #stop with brake
                left_motor1.stop(BRAKE)
                left_motor2.stop(BRAKE)
                right_motor1.stop(BRAKE)
                right_motor2.stop(BRAKE)
            else: #cruise with coast
                left_motor1.stop(COAST)
                left_motor2.stop(COAST)
                right_motor1.stop(COAST)
                right_motor2.stop(COAST)

        if controller.buttonR1.pressing():
            input_motor.spin(FORWARD, 100, PERCENT) #input motor forward
        elif controller.buttonR2.pressing():
            input_motor.spin(REVERSE, 100, PERCENT) #input motor reverse
        else:
            input_motor.stop() #stop input motor if neither button is pressed

        if controller.buttonL1.pressing():
            output_motor.spin(FORWARD, 100, PERCENT) #output motor forward
        elif controller.buttonL2.pressing():
            output_motor.spin(REVERSE, 100, PERCENT) #output motor reverse
        else:
            output_motor.stop() #stop output motor if neither button is pressed

        # Automatic intake/output
        if controller.buttonDown.pressing():
            input_motor.spin(FORWARD, 100, PERCENT)
            output_motor.spin(FORWARD, 100, PERCENT)
        elif controller.buttonUp.pressing():  
            input_motor.spin(REVERSE, 100, PERCENT)
            output_motor.spin(REVERSE, 100, PERCENT)

        # If you want to trigger autonomous from the controller, uncomment the block below.
        if controller.buttonY.pressing() and controller.buttonB.pressing() and controller.buttonA.pressing():
            autonomous()
            wait(200, MSEC)  #debounce delay

        wait(20, MSEC)

competition = Competition(driver_control, autonomous)