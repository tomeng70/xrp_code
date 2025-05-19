from XRPLib.defaults import *

# this program uses the line plotter kit (which requires a standard sized servo)
# and tests raising and lowering the arm that holds the pen.
# See https://www.printables.com/model/924143-line-plotter-kit-for-xrp-robot
# for details on the line plotter kit.

# init global variables
previous_button_state = 0
UP_POS = 150
DOWN_POS = 180
pen_state = 0

# raise the pen, which should be controlled by servo #2.
def raise_pen():
    global pen_state
    servo_two.set_angle(UP_POS)
    pen_state = 1
    print("raise_pen(), UP_POS =", UP_POS)

# lower the pen, which should be controlled by servo #2.   
def lower_pen():
    global pen_state
    servo_two.set_angle(DOWN_POS)
    pen_state = 0
    print("lower_pen(), DOWN_POS =", DOWN_POS)
    
# toggle the position of the pen.
def toggle_pen():
    global pen_state
    if pen_state == 0:
        # raise the pen.
        raise_pen()
    else:
        # lower the pen.
        lower_pen()

# our main function
def main():
    global previous_button_state
    print("Running main() method...")
    print("Press the user button to toggle the pen arm position.")

    # loop and wait for button presses.
    while True:
        if (board.is_button_pressed() and previous_button_state == 0):
            # button was just pressed.
            print("toggling pen")
            toggle_pen()
            previous_button_state = 1
        elif (board.is_button_pressed() == False and previous_button_state == 1):
            # button was just released.
            previous_button_state = 0
    
main()