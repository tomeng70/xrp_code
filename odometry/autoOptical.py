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

def printPosition(myOtos):
    myPosition = myOtos.getPosition()

    # Print measurement
    print()
    print("Position:")
    print("X (Inches): {}".format(myPosition.y))
    print("Y (Inches): {}".format(-myPosition.x)) # notice the negative sign to swap coordinate systems
    print("Heading (Degrees): {}".format(myPosition.h))

def calculate_error(pose, waypoint):
    # calculate x and y error values.
    x_target = waypoint.x
    y_target = waypoint.y
    h_target = waypoint.h

    # notice the transposed coordinate system between robot and optical sensor
    x_current = pose.y
    y_current = -pose.x
    h_current = pose.h

    x_err = x_target - x_current
    y_err = y_target - y_current
    h_err = h_target - h_current
    
    return [x_err, y_err, h_err]

# def calculate_offset(pose, waypoint):
#     # calculate x and y error values.
#     x_target = waypoint[0]
#     y_target = waypoint[1]
#     x_err = x_target - pose.y
#     y_err = y_target - pose.x
    
#     # calculate the angle from the robot to the target waypoint
#     angle_to_target = math.atan2(y_err, x_err)
    
#     # calculate the difference between the angle to the target and the pose angle.
#     offset_angle = angle_to_target - pose.theta
    
#     # calculate distance from bot to the target waypoint
#     dist_to_tgt = math.sqrt(x_err ** 2 + y_err ** 2)
    
#     return [dist_to_tgt, offset_angle]

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
    
    currentPosition = qwiic_otos.Pose2D(0, 0, 0)
    myOtos.setPosition(currentPosition)

    # Main loop

    # test target position
    targetPosition = qwiic_otos.Pose2D(12, 0, 0)

    prev_time = time.time()
    while True:
        # Get the latest position, which includes the x and y coordinates, plus
        # the heading angle
        myPosition = myOtos.getPosition()

        # calculate error to target
        error = calculate_error(myPosition, targetPosition)

        # is it time to print current position?
        # don't print too frequently, as it can spam the serial bus.
        current_time = time.time()
        if (current_time - prev_time) > 0.5:
            printPosition(myOtos)
            print("Error to Target:")
            print("X Error (Inches): {}".format(error[0]))
            print("Y Error (Inches): {}".format(error[1]))
            print("Heading Error (Degrees): {}".format(error[2]))
            prev_time = current_time

        # Alternatively, you can comment out the print and delay code above, and
        # instead use the following code to rapidly refresh the data
        # print("{}\t{}\t{}".format(myPosition.x, myPosition.y, myPosition.h))
        # time.sleep(0.01)

if __name__ == '__main__':
    try:
        run()
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding Run")
        sys.exit(0)