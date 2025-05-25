from XRPLib.board import Board
from XRPLib.encoded_motor import EncodedMotor
from XRPLib.differential_drive import DifferentialDrive
from XRPLib.imu import IMU
import time
import math

# The Pose class is used to represent the position and orientation of the bot.
class Pose:
    # construction.
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

# get refereneces to hardware.
board = Board.get_default_board()
motor_left = EncodedMotor.get_default_encoded_motor(1)
motor_right = EncodedMotor.get_default_encoded_motor(2)
drive = DifferentialDrive.get_default_differential_drive()

imu = IMU.get_default_imu()
imu.calibrate(1)
imu.reset_yaw()

# geometry of differential drive.
# distances are in cm.
wheel_radius = 6.0
wheel_spacing = 15.5

# encoder resolution
counts_per_rev = 585.0
wheel_circumference = 2.0 * math.pi * wheel_radius

# keep track of the previous motor encoder counts.
prev_count_left = motor_left.get_position_counts()
prev_count_right = motor_right.get_position_counts()

# robot states. Note MicroPython does not seem to support enums yet.
IDLE = 0
TURNING = 1
NAVIGATING = 2

def get_path():
    # dimensions (in cm)
    SIDE = 100.0
    
    # trace the edges of the path.
    path = [
        [SIDE, 0],
        [SIDE, SIDE],
        [0, SIDE],
        [0, 0],
        
        [SIDE - 20, 20],
        [SIDE - 20, SIDE - 20],
        [20, SIDE - 20],
        [20, 20],

        [SIDE - 40, 40],
        [SIDE - 40, SIDE - 40],
        [40, SIDE - 40],
        [40, 40]
        
        ]
        
    
    # # loop and generate columns
    # num_cols = math.floor(SIDE /  25 )
    # print(num_cols)
    # COLUMN = SIDE / num_cols 
    
    # for i in range (0, num_cols):
    #     # what was previous points y position.
    #     previous_y = path[len(path) - 1][1]
    #     new_x = (i + 1) *  COLUMN
    #     if (previous_y == 0):
    #         if (i == 0):
    #             # for the initial column, start at the same
    #             # position as the previous waypoint
    #             new_y = 0
    #         else:
    #             new_y = SIDE
    #     else:
    #         new_y = 0
    #     path.append([new_x, new_y])
        
    # return path
    return path
    
def update_pose():
    # global variables modified by this function.
    global prev_count_left, prev_count_right
    
    # get distance traveled by left and right wheels.
    curr_count_left = motor_left.get_position_counts()
    curr_count_right = motor_right.get_position_counts()
    
    # convert to revolutions then multiply by the circumference of the wheel.
    dist_left = (curr_count_left - prev_count_left) / counts_per_rev * wheel_circumference
    dist_right = (curr_count_right - prev_count_right) / counts_per_rev * wheel_circumference
    
    # calculate distance traveled by center point.
    dist_avg = (dist_left + dist_right) / 2.0
    
    # calculate change in angle.
    delta_theta = (dist_right - dist_left) / (2.0 * wheel_spacing)
    
     # calculate changes in position.
    delta_x = dist_avg * math.cos(pose.theta + delta_theta / 2.0) / 2.0
    delta_y = dist_avg * math.sin(pose.theta + delta_theta / 2.0) / 2.0
    
    # update position.
    pose.x += delta_x
    pose.y += delta_y
    
    # update angle.
    # pose.theta += delta_theta
    
    # use IMU for angle.
    pose.theta = math.radians(imu.get_heading())
    
    # update values for next iteration
    prev_count_left = curr_count_left
    prev_count_right = curr_count_right
    
# calculate the angle and distance to target waypoint.
def calculate_offset():
    # calculate x and y error values.
    x_target = waypoint[0]
    y_target = waypoint[1]
    x_err = x_target - pose.x
    y_err = y_target - pose.y
    
    # calculate the angle from the robot to the target waypoint
    angle_to_target = math.atan2(y_err, x_err)
    
    # calculate the difference between the angle to the target and the pose angle.
    offset_angle = angle_to_target - pose.theta
    
    # calculate distance from bot to the target waypoint
    dist_to_tgt = math.sqrt(x_err ** 2 + y_err ** 2)
    
    return [dist_to_tgt, offset_angle]
    
def apply_correction(offset):
    # extract values from list.
    dist_to_tgt = offset[0]
    offset_angle = offset[1]
    
    # express the angular error as a ratio.
    error_angle = math.atan2(math.sin(offset_angle), math.cos(offset_angle))
    
    # calculate the correction velocity.
    if (dist_to_tgt > 20):
        error_vel = 40
    elif (dist_to_tgt > 10):
        error_vel = 30
    else:
        error_vel = 20
        
    # calculate desired rotational and linear velocities of the robot.
    omega = KP_ANGLE * error_angle
    vel_B = KP_DIST * error_vel
    
    # convert to input motor velocities (angular).
    theta_dot_L = vel_B / wheel_radius - wheel_spacing / (2.0 * wheel_radius) * omega
    theta_dot_R = vel_B / wheel_radius + wheel_spacing / (2.0 * wheel_radius) * omega
    
    # limit motor speed
    MAX_SPEED = 90
    if theta_dot_L > MAX_SPEED:
        theta_dot_L = MAX_SPEED
    elif theta_dot_L < -MAX_SPEED:
        theta_dot_L = -MAX_SPEED
        
    if theta_dot_R > MAX_SPEED:
        theta_dot_R = MAX_SPEED
    elif theta_dot_R < -MAX_SPEED:
        theta_dot_R = -MAX_SPEED
    
    # apply correction.
    motor_left.set_speed(theta_dot_L)
    motor_right.set_speed(theta_dot_R)
    
def stop():
    motor_left.set_speed(0)
    motor_right.set_speed(0)
    
# main code.
# keep track of the state of the robot.
current_state = IDLE

# initial pose.
pose = Pose(0, 0, 0)

# path (x, y, coordinates are specified in cm)
index_path = 0
path = get_path()
print ("path:", path)

# correction constants
KP_DIST = 8
KP_ANGLE = 18
# wait for button press.
print("Flashing LED")
board.led_blink(6)

# Wait for user to press button
print("Press user button to begin")
board.wait_for_button()
print("Running program...")
board.led_blink(2)

imu.reset_yaw()

wait_flag = False
wait_count = 0
WAIT_ITERATIONS = 100

while True:
    # update the pose of the robot.
    update_pose()
    
    # what is our state?
    # print("debug: state =", current_state)
    if (current_state == IDLE):
        # should we wait?
        if(wait_flag):
            print("Waiting")
            wait_count += 1
            if (wait_count > WAIT_ITERATIONS):
                wait_flag = 0;
        else:
            # print("debug: in IDLE")
            # Are there any waypoints left in the path?
            if (index_path < len(path)):
                # get the next waypoint
                waypoint = path[index_path]
                print("navigating to", waypoint)
                current_state = NAVIGATING
            else:
                print("IDLE with no waypoints left.")
                board.led_blink(1)
    elif (current_state == NAVIGATING):
        # print("debug: in NAVIGATING")
         # how far are we from the target waypoint?
        offset = calculate_offset()
    
        # are we at the target waypoint?
        if (offset[0] < 10):
            # stop bot
            stop()
            
            # update index
            index_path += 1
            
            print("done!")
            current_state = IDLE
            
            # set wait flag to wait after reaching waypoint.
            wait_flag = True
            wait_count = 0
            
            # print("debug: sleep for a moment")
            # time.sleep(1)
            # print("debug: awake!")
        else:
            apply_correction(offset)
        