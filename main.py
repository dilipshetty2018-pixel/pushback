# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Shardul Patil Dumbre                                         #
# 	Created:      1/30/2026, 5:21:27 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

#THIS IS FINAL FILE

# Library imports
from vex import *

brain = Brain()
controller = Controller()

#pneumatics setup
loader_piston_solenoid = DigitalOut(brain.three_wire_port.a)
descorer_piston_solenoid = DigitalOut(brain.three_wire_port.b)

# Motor configurations
left_motor_front = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
left_motor_back = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
left_motor_group = MotorGroup(left_motor_front, left_motor_back)

right_motor_front = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
right_motor_back = Motor(Ports.PORT4, GearSetting.RATIO_18_1, True)
right_motor_group = MotorGroup(right_motor_front, right_motor_back)

front_intake_motor = Motor(Ports.PORT5, GearSetting.RATIO_6_1, False)
back_output_motor = Motor(Ports.PORT6, GearSetting.RATIO_6_1, False)

# --- PID Tuning Variables (Change these as needed) ---
# Distance PID
KP = 0.06      
KI = 0.01     
KD = 0.2

"""
Tune KP (proportional): Start low and gradually increase until the robot responds quickly but does not oscillate excessively.
Tune KD (derivative): Start at zero, then slowly increase to reduce overshoot and dampen oscillations.
Tune KI (integral): Start at zero, then increase very slowly if you notice steady-state error (robot doesn’t reach the exact target). Too much KI can cause instability.
"""
# power and error thresholds
MAX_POWER = 80    # max motor power (percent)
MIN_POWER = 10    # min motor power (percent)
ACCEPTABLE_ERROR = 2  # acceptable error to stop (degrees)

# --- Robot Physical Constants ---
WHEEL_DIAMETER_MM = 110.6  # Wheel diameter in mm
WHEEL_CIRCUMFERENCE_MM = 3.1416 * WHEEL_DIAMETER_MM
WHEELBASE_MM = 280  # Distance between left and right wheels (adjust to your robot)

# --- Other Constants ---
DEADBAND = 5  # controller deadband threshold
loader_piston_extended = False
a_button_last = False

descorer_piston_extended = False
b_button_last = False

# --- Ramping Variables ---
RAMP_RATE = 9  # used for ramping (lower = smoother, higher = jumpy)
CURRENT_LM_POWER = 0  # current left motor power
CURRENT_RM_POWER = 0  # current right motor power
BRAKE_MODE = False  # False = coast, True = brake

# --- Torque Display Variables ---
TORQUE_UPDATE_INTERVAL = 10  # update display every N loops (prevents flickering)
torque_update_counter = 0

def turn_pid(target_angle_deg):
    # positive angle = right turn, negative = left turn.

    # calc arc length 
    arc_length = (abs(target_angle_deg) / 360) * 3.1416 * WHEELBASE_MM
    # convert the length to wheel degrees
    wheel_degrees = (arc_length / WHEEL_CIRCUMFERENCE_MM) * 360
    if target_angle_deg < 0:
        wheel_degrees = -wheel_degrees

    # Reset sensors
    left_motor_group.reset_position()
    right_motor_group.reset_position()

    integral = 0
    prev_error = 0

    while True:
        left_pos = left_motor_group.position(DEGREES)
        right_pos = right_motor_group.position(DEGREES)
        # for turning in place right and left should be equal and opposite
        avg_pos = (left_pos - right_pos) / 2

        error = wheel_degrees - avg_pos
        integral += error
        derivative = error - prev_error
        prev_error = error

        drive_power = KP * error + KI * integral + KD * derivative
        drive_power = max(min(drive_power, MAX_POWER), -MAX_POWER)
        if abs(drive_power) < MIN_POWER:
            drive_power = MIN_POWER * (1 if drive_power > 0 else -1)    #have to check this logic


        left_power = drive_power
        right_power = -drive_power

        left_power = max(min(left_power, MAX_POWER), -MAX_POWER)
        right_power = max(min(right_power, MAX_POWER), -MAX_POWER)

        left_motor_group.spin(FORWARD, left_power, PERCENT)
        right_motor_group.spin(FORWARD, right_power, PERCENT)

        if abs(error) < ACCEPTABLE_ERROR:
            break

        wait(20, MSEC)

    left_motor_group.stop()
    right_motor_group.stop()

def straight_pid(target_distance_mm):

    # convert mm to degrees for motor encoders
    target_distance_deg = (target_distance_mm / WHEEL_CIRCUMFERENCE_MM) * 360

    # reset sensors
    left_motor_group.reset_position()
    right_motor_group.reset_position()

    integral = 0
    prev_error = 0

    while True:
        # get the motor encoder readings
        left_pos = left_motor_group.position(DEGREES)
        right_pos = right_motor_group.position(DEGREES)
        avg_pos = (left_pos + right_pos) / 2

        # distance PID
        error = target_distance_deg - avg_pos
        integral += error
        derivative = error - prev_error
        prev_error = error

        # PID calculations 
        drive_power = KP * error + KI * integral + KD * derivative
        drive_power = max(min(drive_power, MAX_POWER), -MAX_POWER)
        if abs(drive_power) < MIN_POWER:
            drive_power = MIN_POWER * (1 if drive_power > 0 else -1)

        # power
        left_power = drive_power
        right_power = drive_power

        # clamp power
        left_power = max(min(left_power, MAX_POWER), -MAX_POWER)
        right_power = max(min(right_power, MAX_POWER), -MAX_POWER)

        left_motor_group.spin(FORWARD, left_power, PERCENT)
        right_motor_group.spin(FORWARD, right_power, PERCENT)

        # exit if within reasonable error
        if abs(error) < ACCEPTABLE_ERROR:
            break

        wait(20, MSEC)

    left_motor_group.stop()
    right_motor_group.stop()

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
    
    straight_pid(820)  # drive forward 820 mm
    turn_pid(-90)
    straight_pid(700)  # drive forward 700 mm to loading tube
    front_intake_motor.spin_for(FORWARD, 5000, MSEC)  # pick up 3 balls (time may need adjustment) 
    back_output_motor.spin_for(FORWARD, 5000, MSEC)  # rotate balls into storage (time may need adjustment)
    straight_pid(-700)  # drive backward 600 mm to long goal
    back_output_motor.spin_for(FORWARD, 5000, MSEC)  # place all balls in long goal (time may need adjustment)
    
    left_motor_group.stop(BRAKE)
    right_motor_group.stop(BRAKE)

def user_control():
    global loader_piston_extended, a_button_last, descorer_piston_extended, b_button_last
    global CURRENT_LM_POWER, CURRENT_RM_POWER, BRAKE_MODE, RAMP_RATE
    global torque_update_counter
    while True:
        # Tank drive control with deadband
        left_speed = controller.axis3.position()
        right_speed = controller.axis2.position()

        if abs(left_speed) < DEADBAND:
            left_speed = 0
        if abs(right_speed) < DEADBAND:
            right_speed = 0

        # If both sticks are in the same direction (and non-zero), average them
        # to avoid a slight mismatch causing the robot to drift.
        if left_speed * right_speed > 0:
            avg = (left_speed + right_speed) / 2
            target_left_power = avg
            target_right_power = avg
        else:
            target_left_power = left_speed
            target_right_power = right_speed

        # Toggle brake mode with X button
        if controller.buttonX.pressing():
            BRAKE_MODE = not BRAKE_MODE
            controller.screen.clear_screen()
            controller.screen.print("Brake Mode: ", BRAKE_MODE)
            wait(200, MSEC)  # debounce delay

        # Toggle ramp rate with Y + Left (switches between smooth and responsive)
        if controller.buttonY.pressing() and controller.buttonLeft.pressing():
            RAMP_RATE = 10 - RAMP_RATE
            wait(200, MSEC)  # debounce delay

        # Ramping logic - smoothly ramp left motor power toward target
        if CURRENT_LM_POWER < target_left_power:  # if current is below target, increase
            CURRENT_LM_POWER += RAMP_RATE  # prevent overshooting
            if CURRENT_LM_POWER > target_left_power:
                CURRENT_LM_POWER = target_left_power
        elif CURRENT_LM_POWER > target_left_power:  # if current is above target, decrease
            CURRENT_LM_POWER -= RAMP_RATE  # prevent undershooting
            if CURRENT_LM_POWER < target_left_power:
                CURRENT_LM_POWER = target_left_power

        # Ramping logic - smoothly ramp right motor power toward target
        if CURRENT_RM_POWER < target_right_power:  # if current is below target, increase
            CURRENT_RM_POWER += RAMP_RATE  # prevent overshooting
            if CURRENT_RM_POWER > target_right_power:
                CURRENT_RM_POWER = target_right_power
        elif CURRENT_RM_POWER > target_right_power:  # if current is above target, decrease
            CURRENT_RM_POWER -= RAMP_RATE  # prevent undershooting
            if CURRENT_RM_POWER < target_right_power:
                CURRENT_RM_POWER = target_right_power

        # Spin motors with ramped power
        left_motor_group.spin(FORWARD, CURRENT_LM_POWER, PERCENT)
        right_motor_group.spin(FORWARD, CURRENT_RM_POWER, PERCENT)

        # Apply brake or coast when stopped
        if target_left_power == 0 and target_right_power == 0:
            if BRAKE_MODE:
                left_motor_group.stop(BRAKE)
                right_motor_group.stop(BRAKE)
            else:
                left_motor_group.stop(COAST)
                right_motor_group.stop(COAST)

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

        if controller.buttonR1.pressing():
            front_intake_motor.spin(FORWARD, 100, PERCENT) #input motor forward
        elif controller.buttonR2.pressing():
            front_intake_motor.spin(REVERSE, 100, PERCENT) #input motor reverse
        else:
            front_intake_motor.stop() #stop input motor if neither button is pressed
        if controller.buttonL1.pressing():
            back_output_motor.spin(FORWARD, 100, PERCENT) #output motor forward
        elif controller.buttonL2.pressing():
            back_output_motor.spin(REVERSE, 100, PERCENT) #output motor reverse
        else:
            back_output_motor.stop() #stop output motor if neither button is pressed

        # Automatic intake/output
        if controller.buttonDown.pressing():
            front_intake_motor.spin(FORWARD, 100, PERCENT)
            back_output_motor.spin(FORWARD, 100, PERCENT)
        elif controller.buttonUp.pressing():  
            front_intake_motor.spin(REVERSE, 100, PERCENT)
            back_output_motor.spin(REVERSE, 100, PERCENT)

        # Display drivetrain motor torque on controller screen (always on)
        torque_update_counter += 1
        if torque_update_counter >= TORQUE_UPDATE_INTERVAL:
            torque_update_counter = 0
            # Get torque values from each drivetrain motor (in Nm)
            lf_torque = left_motor_front.torque(TorqueUnits.NM)
            lb_torque = left_motor_back.torque(TorqueUnits.NM)
            rf_torque = right_motor_front.torque(TorqueUnits.NM)
            rb_torque = right_motor_back.torque(TorqueUnits.NM)
            
            # Display on controller screen (3 lines available)
            controller.screen.clear_screen()
            controller.screen.set_cursor(1, 1)
            controller.screen.print("LF:", round(lf_torque, 2), " LB:", round(lb_torque, 2))
            controller.screen.set_cursor(2, 1)
            controller.screen.print("RF:", round(rf_torque, 2), " RB:", round(rb_torque, 2))

        # If you want to trigger autonomous from the controller, uncomment the block below.
        if controller.buttonY.pressing() and controller.buttonB.pressing() and controller.buttonA.pressing():
            autonomous()
            wait(200, MSEC)
        wait(20, MSEC)

# create competition instance
competition = Competition(user_control, autonomous)

