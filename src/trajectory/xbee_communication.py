import math
import rospy
from geometry_msgs.msg import Pose2D

# Xbee imports (?)
import serial
#from xbee import XBee

# port, BAUD rate
PORT = '/dev/ttyUSB0'
BAUD = 9600
serial_port = serial.Serial(PORT, BAUD)

path = None
tolerance = 20

def get_theta(start,end):
    x = end[0] - start[0]
    y = end[1] - start[1]

    return math.atan2(y,x)

def get_distance(start,end):
    return math.sqrt(math.power(start[0] - end[0],2) + math.power(start[1] - end[1],2))

# get robot's current position and adjust angle to target
def update_robot(pos):
    global path
    current_pos = (pos.x,pos.y)

    distance_next = get_distance(current_pos, path[0])

    if distance_next < tolerance and len(path) > 1:
        path.pop(0)
    elif len(path) == 1 and distance_next < 5:
        path.pop(0)

    if len(path) > 0:
        new_theta = math.pi/2 - get_theta(current_pos, path[0])
    else:
        new_theta = 0

    send_signal(new_theta)

# send new angle via XBee
def send_signal(theta):
    global serial
    print 'Sending {} angle'.format(theta)

    serial.write(theta)


def run(final_path):
    global path

    # initialize path
    path = final_path[1:]

    rospy.init_node('xbee', anonymous=True)
    print "Node XBee initialized"
    #Robot position
    rospy.Subscriber("/y_r0", Pose2D, update_robot)
