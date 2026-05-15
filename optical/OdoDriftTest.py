"""
OdoDriftTest.py - Stationary drift test for the SparkFun OTOS tracker.

Purpose
-------
The robot sits PERFECTLY STILL. The motors are never commanded. We just
read the OTOS pose and watch how far (x, y, h) wander away from zero over
time. If the heading h creeps steadily, that's a residual gyro bias left
over from calibrateImu() - the likely source of the run-to-run heading
disturbance seen while tuning SquarePID.

This program deliberately mirrors SquarePID.py:
    - same imports and OTOS init (so it tests the SAME calibration path)
    - same coordinate convention and get_pose()
    - same telemetry style, printed on a fixed interval

How to use
----------
1. Place the robot on the test surface and DO NOT TOUCH IT.
2. Run the program, press the button, hold still during IMU calibration.
3. Keep the robot motionless for the whole test window.
4. Read the summary at the end:
       - h drifts steadily   -> gyro bias (recalibrate / hold stiller / longer cal)
       - x or y drifts        -> optical-flow noise/bias
       - everything ~0        -> the OTOS is solid; the disturbance is mechanical

Coordinate convention (matches SquarePID.py / TestMouse.py):
    robot_x =  otos.y
    robot_y = -otos.x
    heading =  otos.h     (degrees, CCW positive)
"""

from XRPLib.defaults import *  # exposes drivetrain, etc.
import qwiic_i2c
import qwiic_otos
import sys
import time


# ---------------------------------------------------------------------------
# Test parameters
# ---------------------------------------------------------------------------
TEST_DURATION  = 30.0  # seconds to monitor drift
PRINT_INTERVAL = 1.0   # seconds between telemetry lines
LOOP_DT        = 0.02  # 50 Hz read loop (matches SquarePID)


# ---------------------------------------------------------------------------
# OTOS pose  (identical to SquarePID.py so we test the same path)
# ---------------------------------------------------------------------------
def init_otos():
    bus  = qwiic_i2c.get_i2c_driver(sda=18, scl=19, freq=100000)
    otos = qwiic_otos.QwiicOTOS(i2c_driver=bus)

    if not otos.is_connected():
        raise RuntimeError("OTOS sensor not detected on I2C bus.")

    otos.begin()

    print("Keep the robot perfectly still for IMU calibration.")
    for i in range(5, 0, -1):
        print("  Calibrating in {}...".format(i))
        time.sleep(1)
    otos.calibrateImu()

    # Sensor mounting offset on the chassis (inches), from TestMouse.py.
    otos.setOffset(qwiic_otos.Pose2D(2.25, 4.375, 0))

    otos.resetTracking()
    otos.setPosition(qwiic_otos.Pose2D(0, 0, 0))
    return otos


def get_pose(otos):
    """Return (x, y, heading_deg) in the robot's working frame."""
    p = otos.getPosition()
    return p.y, -p.x, p.h


def stop():
    # The motors are never driven in this test, but make doubly sure.
    drivetrain.set_effort(0, 0)


# ---------------------------------------------------------------------------
# Drift monitor
# ---------------------------------------------------------------------------
def drift_test(otos):
    """
    Sit still, read the OTOS, and report how far the pose has wandered from
    (0, 0, 0). Since init_otos() just zeroed the pose, the raw reading IS
    the accumulated drift.
    """
    stop()

    start_ms     = time.ticks_ms()
    next_print_s = 0.0

    # Track the worst-case excursion seen during the test.
    peak_x = peak_y = peak_h = 0.0

    print("\n--- Stationary drift test: monitoring for {:.0f} s ---"
          .format(TEST_DURATION))
    print("(robot must stay completely still)")

    while True:
        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        x, y, h = get_pose(otos)

        if abs(x) > peak_x:
            peak_x = abs(x)
        if abs(y) > peak_y:
            peak_y = abs(y)
        if abs(h) > peak_h:
            peak_h = abs(h)

        # Telemetry on a fixed interval. x, y, h ARE the drift; the avg
        # rates are the drift divided by elapsed time - a steady nonzero
        # h-rate is the gyro-bias signature.
        if elapsed >= next_print_s:
            if elapsed > 0.0:
                rx, ry, rh = x / elapsed, y / elapsed, h / elapsed
            else:
                rx = ry = rh = 0.0
            print("t={:5.1f}s  drift=({:7.3f},{:7.3f},{:7.2f})  "
                  "avg/s=({:7.4f},{:7.4f},{:7.4f})"
                  .format(elapsed, x, y, h, rx, ry, rh))
            next_print_s += PRINT_INTERVAL

        if elapsed >= TEST_DURATION:
            break

        time.sleep(LOOP_DT)

    # Final summary.
    x, y, h = get_pose(otos)
    print("\n--- Drift test complete ({:.1f} s) ---".format(elapsed))
    print("Total drift:  x={:7.3f} in   y={:7.3f} in   h={:7.2f} deg"
          .format(x, y, h))
    print("Avg rate:     x={:8.4f} in/s y={:8.4f} in/s h={:8.4f} deg/s"
          .format(x / elapsed, y / elapsed, h / elapsed))
    print("Peak |drift|: x={:7.3f} in   y={:7.3f} in   h={:7.2f} deg"
          .format(peak_x, peak_y, peak_h))
    print("\nInterpretation:")
    print("  - h drifts steadily      -> gyro bias from calibrateImu()")
    print("                              (hold stiller / recalibrate / longer cal)")
    print("  - x or y drifts          -> optical-flow bias or a vibrating surface")
    print("  - all ~0                 -> OTOS is solid; disturbance is mechanical")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def run():
    board = Board.get_default_board()

    # Make the monochrome LED start blinking.
    board.led_blink(1)

    print('press button to start')
    board.wait_for_button()

    otos = init_otos()
    drift_test(otos)
    stop()


if __name__ == "__main__":
    try:
        run()
    except (KeyboardInterrupt, SystemExit):
        print("\nInterrupted - stopping motors.")
        stop()
        sys.exit(0)
    except Exception as e:
        print("Error: {}".format(e))
        stop()
        raise
