from XRPLib.defaults import *

# available variables from defaults: left_motor, right_motor, drivetrain,
#      imu, rangefinder, reflectance, servo_one, board, webserver

# the optical sensor keeps track of pose automatically
# so we don't have to update the pose ourselves.

import qwiic_i2c
import qwiic_otos
import sys
import time
import math

from XRPLib.board import Board
from XRPLib.encoded_motor import EncodedMotor
from XRPLib.differential_drive import DifferentialDrive
from XRPLib.imu import IMU

# get refereneces to hardware.
board = Board.get_default_board()
motor_left = EncodedMotor.get_default_encoded_motor(1)
motor_right = EncodedMotor.get_default_encoded_motor(2)
drive = DifferentialDrive.get_default_differential_drive()

# geometry of differential drive.
# distances are in cm.
wheel_radius = 6.0
wheel_spacing = 15.5

# encoder resolution
counts_per_rev = 585.0
wheel_circumference = 2.0 * math.pi * wheel_radius

# correction constants
KP_DIST = 1
KP_ANGLE = 2.75

def getCurrentPose(myOtos):
    position = myOtos.getPosition()
    x = position.y
    y = -position.x
    h = position.h
    if (h < 0):
        h = h+360

    return qwiic_otos.Pose2D(x, y, h)

def printPose(pose):
    # Print measurement
    print()
    print("Position:")
    print("X (Inches): {}".format(pose.x))
    print("Y (Inches): {}".format(pose.y)) 
    print("Heading (Degrees): {}".format(pose.h))

def calculate_error(pose, waypoint):
    # calculate x and y error values.
    x_target = waypoint.x
    y_target = waypoint.y
    h_target = waypoint.h

    x_current = pose.x
    y_current = pose.y
    h_current = pose.h

    x_err = x_target - x_current
    y_err = y_target - y_current
    h_err = h_target - h_current
    
    return [x_err, y_err, h_err]

def calculate_offset(pose, waypoint):
    # calculate x and y error values.
    x_target = waypoint.x
    y_target = waypoint.y
    x_err = x_target - pose.x
    y_err = y_target - pose.y
    
    # calculate the angle from the robot to the target waypoint
    angle_to_target = math.atan2(y_err, x_err)

    # convert to angle to target from radians to degrees.
    angle_to_target = math.degrees(angle_to_target)
    
    # calculate the difference between the angle to the target and the pose angle.
    offset_angle = angle_to_target - pose.h
    
    offset_angle = (offset_angle + 180) % 360 - 180
    
    # calculate distance from bot to the target waypoint
    dist_to_tgt = math.sqrt(x_err ** 2 + y_err ** 2)
    
    return [dist_to_tgt, offset_angle]

def apply_correction(offset):
    # extract values from list.
    dist_to_tgt = offset[0]
    offset_angle = offset[1]

    # convert to radians.
    offset_angle = math.radians(offset_angle)
    
    # express the angular error as a ratio.
    error_angle = math.atan2(math.sin(offset_angle), math.cos(offset_angle))
    
    # calculate the correction velocity.
    if (dist_to_tgt > 8):
        error_vel = 20
    elif (dist_to_tgt > 4):
        error_vel = 10
    else:
        error_vel = 5
        
    # slow (scale) down the distance correction if robot is not pointed at target
    heading_scale = max(0.0, math.fabs(math.cos(error_angle)))
    vel_B = error_vel * heading_scale
        
    # calculate desired rotational and linear velocities of the robot.
    omega = KP_ANGLE * error_angle
    
    # turn only if the heading error is large enough.
    if error_angle > math.radians(25):
        vel_B = 0
    else:
        vel_B = KP_DIST * error_vel
    
    # convert to input motor velocities (angular).
    theta_dot_L = vel_B / wheel_radius - wheel_spacing / (2.0 * wheel_radius) * omega
    theta_dot_R = vel_B / wheel_radius + wheel_spacing / (2.0 * wheel_radius) * omega
    
    RAD_S_TO_RPM = 60.0 / (2.0 * math.pi)

    theta_dot_L_rpm = theta_dot_L * RAD_S_TO_RPM
    theta_dot_R_rpm = theta_dot_R * RAD_S_TO_RPM

    MAX_RPM = 60  # pick something reasonable (try 40–120)
    theta_dot_L_rpm = max(-MAX_RPM, min(MAX_RPM, theta_dot_L_rpm))
    theta_dot_R_rpm = max(-MAX_RPM, min(MAX_RPM, theta_dot_R_rpm))
        
    # print("theta_dot_R_rpm: {}".format(theta_dot_R_rpm))
    # print("theta_dot_L_rpm: {}".format(theta_dot_L_rpm))
    # print("vel_B: {}".format(vel_B))
    # print("omega: {}".format(omega))
    
    # apply correction.
    motor_left.set_speed(theta_dot_L_rpm)
    motor_right.set_speed(theta_dot_R_rpm)

def stop():
    motor_left.set_speed(0)
    motor_right.set_speed(0)

def run():
    print("\nAuto Optical\n")
   
    # Create instance of device
    my_bus = qwiic_i2c.get_i2c_driver(sda=18, scl=19, freq=100000)
    myOtos = qwiic_otos.QwiicOTOS(i2c_driver=my_bus)

    # Check if it's connected
    if myOtos.is_connected() == False:
        print("The device isn't connected to the system. Please check your connection", \
            file=sys.stderr)
        return

    # Initialize the device
    myOtos.begin()

    print("Ensure the OTOS is flat and stationary during calibration!")
    for i in range(5, 0, -1):
        print("Calibrating in %d seconds..." % i)
        time.sleep(1)

    print("Calibrating IMU...")

    # Calibrate the IMU, which removes the accelerometer and gyroscope offsets
    myOtos.calibrateImu()
    
    offset = qwiic_otos.Pose2D(2.25, 4.375, 0)
    myOtos.setOffset(offset)

    # Reset the tracking algorithm - this resets the position to the origin,
    # but can also be used to recover from some rare tracking errors
    myOtos.resetTracking()
    
    # wait a moment to let sensor settle down.
    print("waiting a moment...")
    time.sleep(3)
    print("done waiting!")
    
    currentPosition = qwiic_otos.Pose2D(0, 0, 0)
    myOtos.setPosition(currentPosition)

    # Main loop

    # test target position
    targetPosition = qwiic_otos.Pose2D(36, 0, 0)

    # wait until button pressed.
    # wait for button press.
    print("Flashing LED")
    board.led_blink(6)

    # # Wait for user to press button
    # print("Press user button to begin")
    # board.wait_for_button()
    # print("Running program...")
    # board.led_blink(2)

    prev_time = time.time()
    while True:
        # Get the latest position, which includes the x and y coordinates, plus
        # the heading angle
        currPose = getCurrentPose(myOtos)

        # calculate error to target
        error = calculate_error(currPose, targetPosition)

        #calculate offset
        offset = calculate_offset(currPose, targetPosition)
        
        if math.fabs(offset[0]) < 1.5:
            stop()
            print("at target")
            break
        else:
            # apply correction to move to target.
            apply_correction(offset)

        # is it time to print current position?
        # don't print too frequently, as it can spam the serial bus.
        current_time = time.time()
        if (current_time - prev_time) > 0.5:
            printPose(currPose)
            print("Error to Target:")
            print("  X Error (Inches): {}".format(error[0]))
            print("  Y Error (Inches): {}".format(error[1]))
            print("  Heading Error (Degrees): {}".format(error[2]))

            print("Offset:")
            print("  Dist to Tgt (Inches): {}".format(offset[0]))
            print("  Angle to Tgt (Degrees): {}".format(offset[1]))

            prev_time = current_time

if __name__ == '__main__':
    try:
        run()
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding Run")
        sys.exit(0)