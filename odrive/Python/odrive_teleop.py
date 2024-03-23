import rospy
from geometry_msgs.msg import Twist
import odrive
from odrive.enums import *
import time
import math

# Constants
eStopMultiplier = 1
DAMPENING_THRESHOLD = 0.1
WHEEL_BASE = 0.62
WHEEL_DIAMETER = 0.3
CONTROL_TIMEOUT = 1000  # milliseconds
LEFT_POLARITY = 1
RIGHT_POLARITY = -1
VEL_TO_RPS = 1.0 / (WHEEL_DIAMETER * math.pi) * 98.0 / 3.0
RPS_LIMIT = 20
VEL_LIMIT = RPS_LIMIT / VEL_TO_RPS  # Speed limit

# State variables
is_autonomous = False
mode_change = True
left_vel = 0
right_vel = 0
left_vel_actual = 0
right_vel_actual = 0
dampening_on_l = False
dampening_on_r = False
wireless_stop = False
lastData = 0


# Initialize your ODrive
print("Finding an odrive...")
my_drive = odrive.find_any()
print("\tDone")

print("Starting calibration...")
my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while my_drive.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
print("\tDone")

my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL


def set_velocity(left_vel, right_vel):
    """
    Sets the velocity for the motors using ODrive.
    """
    my_drive.axis0.controller.input_vel = left_vel * VEL_TO_RPS * eStopMultiplier
    my_drive.axis1.controller.input_vel = right_vel * VEL_TO_RPS * eStopMultiplier
    time.sleep(0.01)

def vel_callback(twist_msg):
    global dampening_on_l, dampening_on_r
    left_vel = LEFT_POLARITY * (twist_msg.linear.x - WHEEL_BASE * twist_msg.angular.z / 2.0)
    right_vel = RIGHT_POLARITY * (twist_msg.linear.x + WHEEL_BASE * twist_msg.angular.z / 2.0)

    set_velocity(left_vel, right_vel)


def listener():
    rospy.init_node('motor_controller')
    rospy.Subscriber("/cmd_vel", Twist, vel_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
