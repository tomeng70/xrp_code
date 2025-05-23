from XRPLib.board import Board
from XRPLib.encoded_motor import EncodedMotor
from XRPLib.differential_drive import DifferentialDrive
import time

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

# initial pose.
pose = Pose(0, 0, 0)

# main method.
def main():
    print("main")
    
main()