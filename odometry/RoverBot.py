from XRPLib.board import Board
from XRPLib.encoded_motor import EncodedMotor
from XRPLib.differential_drive import DifferentialDrive
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

# keep track of the state of the robot.
current_state = IDLE

# initial pose.
pose = Pose(0, 0, 0)

# path (x, y, coordinates are specified in cm)
# path = [[30, 0],
#         [30, 30],
#         [0, 30],
#         [0, 0]]
        
path = [[100,100]]

index_path = 0

# correction constants
KP_DIST = 8
KP_ANGLE = 22

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
    pose.theta += delta_theta
    
    # update values for next iteration
    prev_count_left = curr_count_left
    prev_count_right = curr_count_right
    
# main code.
# wait for button press.
print("Flashing LED")
board.led_blink(6)

# Wait for user to press button
print("Press user button to begin")
board.wait_for_button()
print("Running program...")
board.led_blink(2)

# get waypoint
waypoint = path[0]

while True:
    # update the pose of the robot.
    update_pose()
  
    # calculate x and y error values.
    x_target = waypoint[0]
    y_target = waypoint[1]
    x_err = x_target - pose.x
    y_err = y_target - pose.y
    
    # calculate the angle from the robot to the target waypoint
    angle_to_target = math.atan2(y_err, x_err)
    
    # calculate the difference between the angle to the target and the pose angle.
    theta_err = angle_to_target - pose.theta
    
    # express the error as a ratio.
    error_angle = math.atan2(math.sin(theta_err), math.cos(theta_err))
    
    # calculate distance from bot to the target waypoint
    dist_to_tgt = math.sqrt(x_err ** 2 + y_err ** 2)
    
    # calculate the correction velocity.
    if (abs(theta_err) > 0.25):
        # focus on correcting angle before correcting distance.
        error_vel = 0
    elif (dist_to_tgt > 20):
        error_vel = 40
    elif (dist_to_tgt > 10):
        error_vel = 30
    else:
        error_vel = 20
    
    # print(f"theta_err: {theta_err}, dist_to_tgt: {dist_to_tgt}")
    # print(f"error_angle: {error_angle}, error_vel: {error_vel}")
    # print(f"{pose.x: 5.1f} cm, {pose.y: 5.1f} cm, {pose.theta / math.pi * 180.0 : 5.1f} deg")
    
    #
    
    if (dist_to_tgt < 3):
        print("done")
        theta_dot_L = 0
        theta_dot_R = 0
        board.led_blink(0.5)
    else:
        omega = KP_ANGLE * error_angle
        vel_B = KP_DIST * error_vel
        theta_dot_L = vel_B / wheel_radius - wheel_spacing / (2.0 * wheel_radius) * omega
        theta_dot_R = vel_B / wheel_radius + wheel_spacing / (2.0 * wheel_radius) * omega
        
    # limit speed
    MAX_SPEED = 90
    if theta_dot_L > MAX_SPEED:
        theta_dot_L = MAX_SPEED
    elif theta_dot_L < -MAX_SPEED:
        theta_dot_L = -MAX_SPEED
        
    if theta_dot_R > MAX_SPEED:
        theta_dot_R = MAX_SPEED
    elif theta_dot_R < -MAX_SPEED:
        theta_dot_R = -MAX_SPEED
    
    motor_left.set_speed(theta_dot_L)
    motor_right.set_speed(theta_dot_R)
    
    # print(f"theta_dot_L, theta_dot_R: {theta_dot_L}, {theta_dot_R}")
    
    
    #     theta_err = angle_to_target - pose.theta

    
    # # what is our state?
    # if (current_state == IDLE):
    #     # Are there any waypoints left in the path?
    #     if (index_path < len(path)):
    #         waypoint = path[index_path]
    #         print("navigating to", waypoint)
    #         current_state = NAVIGATING
    #     else:
    #         print("IDLE with no waypoints left.")
    # elif (current_state == NAVIGATING):
    #     # calculate differences
    #     x_target = waypoint[0]
    #     y_target = waypoint[1]
    #     x_err = x_target - pose.x
    #     y_err = y_target - pose.y
        
    #     # what is the angle to the target?
    #     #angle_to_target = math.arctan2(y_err, x_err)
    #     angle_to_target = math.atan2(y_err, x_err)
    #     theta_err = angle_to_target - pose.theta
        
    #     # what is distance to target?
    #     distance_to_target = math.sqrt(x_err ** 2 + y_err ** 2)
        
    #     print("theta_err: ", theta_err)
    #     print("distance_to_target: ", distance_to_target);
    #     print("")
        
    #     # check to see if we are close enough to waypoint.
    #     if (distance_to_target < 5):
    #         # we navigated to waypoint.
    #         print("navigated to waypoint:", waypoint)
    #         index_path = index_path + 1
    #         motor_left.set_speed(0)
    #         motor_right.set_speed(0)
    #         current_state = IDLE
    #     else:
    #         # compute the correction values.
    #         correction_angle = KP_ANGLE * theta_err
    #         if (abs(theta_err) > 1000):
    #             # turn only if we're really misaligned.
    #             correction_distance = 0;
    #         else:
    #             # correction_distance = KP_DIST * distance_to_target
    #             if (distance_to_target > 20):
    #                 correction_distance = 0.6
    #             elif (distance_to_target > 10):
    #                 correction_distance = 0.4
    #             elif (distance_to_target > 5):
    #                 correction_distance = 0.3
    #             else:
    #                 correction_distance = 0.2
                    
    #         # phi_dot_L = correction_distance / wheel_radius - wheel_spacing * correction_angle / (2.0 * wheel_radius)
    #         # phi_dot_R = correction_distance / wheel_radius + wheel_spacing * correction_angle / (2.0 * wheel_radius)
            
    #         phi_dot_L = correction_distance
    #         phi_dot_R = correction_distance
            
    #         # todo: limit speed here.
    #         if (phi_dot_L > 1):
    #             phi_dot_L = 1
    #         elif (phi_dot_L < -1):
    #             phi_dot_L = -1
                
    #         if (phi_dot_R > 1):
    #             phi_dot_R = 1
    #         elif (phi_dot_R < -1):
    #             phi_dot_R = -1
            
            
    #         # adjust motor speeds
    #         motor_left.set_effort(phi_dot_L)
    #         motor_right.set_effort(phi_dot_R)
            
    #         print(f"{phi_dot_L}, {phi_dot_R}")
    # print(f"{pose.x: 5.1f} cm, {pose.y: 5.1f} cm, {pose.theta / math.pi * 180.0 : 5.1f} deg")