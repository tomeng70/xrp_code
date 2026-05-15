"""
SquarePID.py - SIMPLIFIED: drive an XRP differential-drive robot in a
straight line from (0, 0, 0 deg) to (24 in, 0, 0 deg), using the SparkFun
OTOS optical tracker for pose feedback.

This is a stripped-down version for tuning the "drive straight" behavior
before going back to the full square. The key change from the square
version: instead of continuously chasing the *bearing to the target point*
(atan2 of the remaining offset), the robot just HOLDS A FIXED HEADING
reference (0 deg) while a distance controller drives it forward.

Why this fixes the left/right limit cycling:
    Chasing atan2(dy, dx) gets very twitchy as you approach the target -
    tiny lateral noise swings the commanded bearing by large amounts, so
    the heading controller hunts back and forth. Holding a fixed heading
    reference removes that noise source entirely, so you can tune the
    heading PID against a clean, constant setpoint.

Coordinate convention (matches TestMouse.py):
    robot_x =  otos.y
    robot_y = -otos.x
    heading =  otos.h     (degrees, CCW positive)

Robot starts at (0, 0) facing +x.
"""

from XRPLib.defaults import *  # exposes drivetrain, etc.
import qwiic_i2c
import qwiic_otos
import sys
import time
import math


# ---------------------------------------------------------------------------
# Mission parameters
# ---------------------------------------------------------------------------
TARGET_DISTANCE = 24.0  # inches to travel forward
HEADING_REF     = 0.0   # degrees - hold this heading the whole way

# ---------------------------------------------------------------------------
# PID gains - START HERE and tune on your robot
# ---------------------------------------------------------------------------
# Heading PID: input = heading error (degrees), output = turn effort [-1, 1]
#   KP_HEAD: 0.055 - the value found by hand-tuning. Reacts fast enough to
#            keep the early heading excursion small without limit cycling.
#   KI_HEAD: 0.010 - the P term alone leaves a steady-state heading error
#            against the constant (mechanical) yaw disturbance - a P
#            controller can only produce a correcting output if the error
#            is nonzero. The integral accumulates that persistent error and
#            builds the standing turn bias needed to cancel the disturbance,
#            which lets the error itself collapse toward zero. Started at
#            0.02, which overshot (heading swung past 0) and pushed `turn`
#            into saturation; 0.010 winds up smoothly and settles cleanly.
#            Tune SLOWLY - too much KI shows up as a slow overshoot/weave.
#   KD_HEAD: 0.002 - unchanged.
KP_HEAD, KI_HEAD, KD_HEAD = 0.075, 0.015, 0.002

# Integral clamp for the heading PID. The PID also has conditional-
# integration anti-windup (it won't keep winding up while the turn output
# is railed), but this is a hard backstop on how much turn authority the
# integral term can ever command:  KI_HEAD * I_MAX_HEAD = 0.010 * 15 = 0.15.
I_MAX_HEAD = 15.0

# Distance PID: input = remaining distance (inches), output = forward effort
KP_DIST, KI_DIST, KD_DIST = 0.060, 0.000, 0.005

# Cross-track steering: how hard to steer back toward the line when the
# robot drifts off it. Units = degrees of heading correction per inch of
# offset. The correction is clamped to +/- MAX_CT_CORRECTION so a big
# offset can't demand an absurd heading. Start gentle - too much gain here
# makes the robot weave across the line instead of converging on it.
KCT_HEADING       = 4.0   # deg per inch off-line
MAX_CT_CORRECTION = 20.0  # deg

# Tolerances for "arrived"
DIST_TOLERANCE = 0.5   # inches  (was 1.0 - robot was stopping ~0.8" short)
SETTLE_TIME    = 0.30  # seconds within tolerance before declaring done

# Deadband on heading error: inside this, the P and D terms are suppressed
# so OTOS heading noise near zero error doesn't make a twitchy turn command.
# IMPORTANT: this deadband is now applied INSIDE the PID and ONLY to the
# P/D terms - the integral still sees the true error, so it can null out a
# steady offset smaller than the deadband instead of being stuck with it.
HEAD_DEADBAND = 1.0    # degrees

# Motor stiction compensation. Below this effort the geared motors don't
# actually turn the wheels - so any nonzero command from the PIDs gets
# bumped up to at least this value (see deadband_ff / set_drive). This is
# why the last run stalled at rem=2.27 forever: the PID wanted 0.14 effort
# but the wheels need ~0.18 to break free. MEASURE THIS on your robot:
# slowly raise a constant effort until the wheels just start turning.
MOTOR_DEADBAND = 0.18

# Effort caps so we don't saturate motors
MAX_FORWARD = 0.6
MAX_TURN    = 0.5

# Overall speed cap, applied per-wheel in set_drive (range 0..1). This is
# the TRACTION limit, not just a "go slower" knob: on a slick surface
# (linoleum) the wheels break loose when commanded effort exceeds what the
# surface can transmit, and a slipping wheel gives you essentially zero
# control authority - PID tuning against a slipping plant is meaningless.
# Lower this until the wheels grip instead of spin. Must stay comfortably
# above MOTOR_DEADBAND or the robot won't move at all.
MAX_SPEED = 0.35

LOOP_DT      = 0.02   # 50 Hz control loop
MAX_RUN_TIME = 20.0   # seconds - safety timeout so a stall can't hang forever
                      # (bumped from 15: a lower MAX_SPEED means longer runs)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def normalize_angle_deg(a):
    """Wrap to (-180, 180]."""
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


class PID:
    def __init__(self, kp, ki, kd, out_min=-1.0, out_max=1.0, i_max=1.0,
                 deadband=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.i_max = i_max
        self.deadband = deadband   # |err| below this -> suppress P and D
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_err = None
        self.prev_t   = None

    def update(self, err):
        # NOTE: use time.ticks_ms(), NOT time.time(). On the XRP's RP2040
        # MicroPython, time.time() has 1-second resolution, so dt comes out
        # as 0 almost every loop. The old code then fell back to dt=1e-3,
        # which multiplied the derivative term by ~1000x and saturated the
        # motors every cycle (the shaky, bang-bang behavior).
        now = time.ticks_ms()
        if self.prev_t is None:
            dt = LOOP_DT
            d_err = 0.0
        else:
            dt = time.ticks_diff(now, self.prev_t) / 1000.0  # ms -> seconds
            if dt <= 0.0:
                dt = LOOP_DT
            d_err = (err - self.prev_err) / dt

        # Proportional + derivative terms. Inside the deadband these are
        # forced to zero so sensor noise near zero error doesn't produce a
        # twitchy command. The integral below still sees the TRUE error -
        # that is the "deadband fix": a small PERSISTENT offset (smaller
        # than the deadband) averages to a real value in the integral but
        # to nothing in P/D, so letting I keep working drives that residual
        # to zero instead of leaving it stuck inside the deadband.
        if abs(err) < self.deadband:
            p_term = 0.0
            d_term = 0.0
        else:
            p_term = self.kp * err
            d_term = self.kd * d_err

        # Tentative integral (accumulates the true error), then the output.
        new_integral = clamp(self.integral + err * dt, -self.i_max, self.i_max)
        out = p_term + self.ki * new_integral + d_term

        # Anti-windup by conditional integration: if the output is railed,
        # only accept the new integral when the current error would move
        # the output back INTO range. Otherwise hold the integral so it
        # can't keep winding up against a limit it cannot act on.
        if out > self.out_max:
            if err < 0.0:
                self.integral = new_integral
            out = self.out_max
        elif out < self.out_min:
            if err > 0.0:
                self.integral = new_integral
            out = self.out_min
        else:
            self.integral = new_integral

        self.prev_err = err
        self.prev_t   = now
        return out


# ---------------------------------------------------------------------------
# OTOS pose
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
    # Verify these numbers reflect where your OTOS is bolted.
    otos.setOffset(qwiic_otos.Pose2D(2.25, 4.375, 0))

    otos.resetTracking()
    otos.setPosition(qwiic_otos.Pose2D(0, 0, 0))
    return otos


def get_pose(otos):
    """Return (x, y, heading_deg) in the robot's working frame."""
    p = otos.getPosition()
    return p.y, -p.x, p.h


# ---------------------------------------------------------------------------
# Motion primitives
# ---------------------------------------------------------------------------
def stop():
    drivetrain.set_effort(0, 0)


def deadband_ff(effort):
    """
    Motor stiction feed-forward. The geared motors won't turn at all below
    ~MOTOR_DEADBAND effort, so a small command produces zero motion and the
    controller looks "dead." This remaps any nonzero command into the band
    [MOTOR_DEADBAND, MAX_SPEED] so it both (a) overcomes stiction and
    (b) NEVER exceeds the traction cap:

        (0, MAX_SPEED]  -> [MOTOR_DEADBAND, MAX_SPEED]
        exactly 0        -> 0   (so "stop" still means stop)

    IMPORTANT: set_drive calls this AFTER the MAX_SPEED scaling, so `effort`
    is already within +/-MAX_SPEED. The previous version remapped into
    [MOTOR_DEADBAND, 1.0], which took a speed-capped wheel (e.g. 0.25) and
    pushed it back UP to ~0.39 - blowing past the traction limit and
    spinning the wheel during hard heading corrections. Remapping into the
    capped band [MOTOR_DEADBAND, MAX_SPEED] fixes that: the stiction
    feed-forward and the speed cap no longer fight each other.
    """
    span = MAX_SPEED - MOTOR_DEADBAND
    if effort > 0.005:
        return MOTOR_DEADBAND + (effort / MAX_SPEED) * span
    elif effort < -0.005:
        return -MOTOR_DEADBAND + (effort / MAX_SPEED) * span
    return 0.0


def set_drive(forward, turn):
    """
    Mix forward + turn into per-wheel efforts, cap them at MAX_SPEED, then
    apply stiction feed-forward per wheel so small corrections still move
    the robot.
    Positive forward = drive +x in body frame.
    Positive turn    = rotate CCW (heading increasing).
    """
    left  = forward - turn
    right = forward + turn

    # Speed cap. Scale BOTH wheels by the same factor so the faster wheel
    # just touches MAX_SPEED. Scaling (rather than independently clamping)
    # preserves the forward:turn ratio - so we keep full steering authority
    # at the cap instead of clipping the turn component away.
    peak = max(abs(left), abs(right))
    if peak > MAX_SPEED:
        scale = MAX_SPEED / peak
        left  *= scale
        right *= scale

    left  = clamp(deadband_ff(left),  -1.0, 1.0)
    right = clamp(deadband_ff(right), -1.0, 1.0)
    drivetrain.set_effort(left, right)


# ---------------------------------------------------------------------------
# The one motion we care about right now: drive straight
# ---------------------------------------------------------------------------
def drive_straight(otos, target_distance, heading_ref=0.0):
    """
    Drive forward `target_distance` inches along the line that starts at the
    current pose and points in the `heading_ref` direction.

    Two feedback channels:
      - DISTANCE: measured as PROGRESS PROJECTED ONTO the heading_ref
        direction (not raw sqrt(dx^2+dy^2)), so cross-track drift doesn't
        corrupt the distance estimate, and overshoot produces a NEGATIVE
        remaining distance (gentle reverse to settle).
      - CROSS-TRACK: perpendicular offset from the line. This is fed into
        the heading setpoint so the robot actively steers back ONTO the
        line, instead of just driving parallel to it.

    The heading PID now has integral action (KI_HEAD), which is what
    cancels the constant mechanical yaw disturbance and drives the
    steady-state heading error to zero - something the P term structurally
    cannot do on its own.
    """
    x0, y0, _ = get_pose(otos)
    hr = math.radians(heading_ref)
    ux, uy = math.cos(hr), math.sin(hr)   # unit vector ALONG the path
    px, py = -uy, ux                      # unit vector PERPENDICULAR (left)

    dist_pid = PID(KP_DIST, KI_DIST, KD_DIST,
                   out_min=-MAX_FORWARD, out_max=MAX_FORWARD)
    head_pid = PID(KP_HEAD, KI_HEAD, KD_HEAD,
                   out_min=-MAX_TURN, out_max=MAX_TURN,
                   i_max=I_MAX_HEAD, deadband=HEAD_DEADBAND)
    in_tol_since = None
    loop_count   = 0
    start_ms     = time.ticks_ms()

    while True:
        x, y, h = get_pose(otos)

        # Progress along the path direction, and what's left to go.
        progress  = (x - x0) * ux + (y - y0) * uy
        remaining = target_distance - progress

        # Cross-track: signed perpendicular distance from the line.
        # Positive = robot has drifted to the LEFT of the line.
        cross_track = (x - x0) * px + (y - y0) * py

        # Steer back toward the line: drifting left -> bias heading right.
        ct_correction = clamp(-KCT_HEADING * cross_track,
                              -MAX_CT_CORRECTION, MAX_CT_CORRECTION)
        heading_target = heading_ref + ct_correction

        # Heading error against the (cross-track-adjusted) setpoint.
        # We pass the TRUE error to the PID now - the deadband is applied
        # INSIDE the PID and only to the P/D terms, so the integral can
        # still null out a sub-deadband steady offset.
        head_err = normalize_angle_deg(heading_target - h)

        arrived = abs(remaining) < DIST_TOLERANCE
        if arrived:
            # Close enough - cut motor commands and just let the settle
            # timer run. Prevents creeping/hunting around the target.
            forward, turn = 0.0, 0.0
        else:
            forward = dist_pid.update(remaining)
            turn    = head_pid.update(head_err)
        set_drive(forward, turn)

        # Telemetry for tuning - print ~5x/sec. `iterm` is the integral's
        # contribution to the turn command: watch it wind UP early and then
        # SETTLE at a steady value - that settled value is the controller's
        # learned estimate of the yaw disturbance. If iterm keeps growing
        # or oscillates, KI_HEAD is too high.
        loop_count += 1
        if loop_count % 10 == 0:
            iterm = KI_HEAD * head_pid.integral
            print("rem={:6.2f}  ct={:6.2f}  h_tgt={:6.2f}  head_err={:6.2f}  "
                  "iterm={:6.3f}  fwd={:5.2f}  turn={:5.2f}  "
                  "pose=({:6.2f},{:6.2f},{:6.1f})"
                  .format(remaining, cross_track, heading_target, head_err,
                          iterm, forward, turn, x, y, h))

        if arrived:
            if in_tol_since is None:
                in_tol_since = time.ticks_ms()
            elif time.ticks_diff(time.ticks_ms(), in_tol_since) >= SETTLE_TIME * 1000:
                break
        else:
            in_tol_since = None

        # Safety timeout: never hang forever if the robot stalls or the
        # tolerance is somehow unreachable.
        if time.ticks_diff(time.ticks_ms(), start_ms) >= MAX_RUN_TIME * 1000:
            print("WARNING: MAX_RUN_TIME hit - stopping. Target not reached.")
            break

        time.sleep(LOOP_DT)

    stop()


# ---------------------------------------------------------------------------
# Mission
# ---------------------------------------------------------------------------
def run():
    # Get a reference to the board
    board = Board.get_default_board()

    # get a reference to the differential drive
    differentialDrive = DifferentialDrive.get_default_differential_drive()

    # Make the monochrome LED start blinking
    board.led_blink(1)

    print('press button to start')
    board.wait_for_button()

    otos = init_otos()

    print("\nMAX_SPEED = ", f"{MAX_SPEED:.2f}")
    print("KP_HEAD = {:.3f}  KI_HEAD = {:.3f}  KD_HEAD = {:.3f}"
          .format(KP_HEAD, KI_HEAD, KD_HEAD))
    print("--- Straight-line test: drive {:.0f} in at heading {:.0f} deg ---"
          .format(TARGET_DISTANCE, HEADING_REF))
    drive_straight(otos, TARGET_DISTANCE, HEADING_REF)
    stop()

    x, y, h = get_pose(otos)
    print("Done. Final pose: x={:.2f} in, y={:.2f} in, h={:.1f} deg"
          .format(x, y, h))
    print("Target was x=24.00, y=0.00, h=0.0  ->  "
          "x error={:.2f} in, cross-track={:.2f} in, heading error={:.1f} deg"
          .format(x - TARGET_DISTANCE, y, normalize_angle_deg(h - HEADING_REF)))


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
