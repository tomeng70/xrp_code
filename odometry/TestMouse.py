from XRPLib.defaults import *

# available variables from defaults: left_motor, right_motor, drivetrain,
#      imu, rangefinder, reflectance, servo_one, board, webserver
# Write your code Here


import qwiic_i2c
import qwiic_otos
import sys
import time

def run():
    print("\nTest Mouse\n")
   
       # Create instance of device
    my_bus = qwiic_i2c.get_i2c_driver(sda=18, scl=19, freq=100000)
    myOtos = qwiic_otos.QwiicOTOS(i2c_driver=my_bus)

   # myOtos = qwiic_otos.QwiicOTOS()

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
    while True:
        # Get the latest position, which includes the x and y coordinates, plus
        # the heading angle
        myPosition = myOtos.getPosition()

        # Print measurement
        print()
        print("Position:")
        # print("X (Inches): {}".format(myPosition.x))
        # print("Y (Inches): {}".format(myPosition.y))
        print("X (Inches): {}".format(myPosition.y))
        print("Y (Inches): {}".format(-myPosition.x))
        print("Heading (Degrees): {}".format(myPosition.h))

        # Wait a bit so we don't spam the serial port
        time.sleep(0.5)

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