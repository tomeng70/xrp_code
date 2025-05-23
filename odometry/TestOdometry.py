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

start_time = 0
current_time = 0
delta_time = 0

prev_count_left = motor_left.get_position_counts()
prev_count_right = motor_right.get_position_counts()


# initial pose.
pose = Pose(0, 0, 0)

# main code.
start_time = time.time_ns()
while True:
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
    start_time = current_time
    prev_count_left = curr_count_left
    prev_count_right = curr_count_right
    
    print(f"{pose.x: 5.1f} cm, {pose.y: 5.1f} cm, {pose.theta / math.pi * 180.0 : 5.1f} deg")