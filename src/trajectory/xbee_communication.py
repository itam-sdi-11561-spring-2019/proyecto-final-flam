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

def get_distance(start,end):
    return math.sqrt(math.power(start[0] - end[0],2) + math.power(start[1] - end[1],2))

def get_vel(omega):
    r = 21
    v = 630
    matrix = np.matrix('1 -57.5; 1 57.5')

    if omega == -1:
        vel = np.array([0,0])
    else:
        vector = np.array([v, omega])
        vel = (1/r) * np.dot(matrix, omega)

    send_signal(vel)

# get robot's current position and adjust angle to target
def update_robot(pos):
    global path
    current_pos = (pos.x,pos.y)

    distance_next = get_distance(current_pos, path[0])

    if distance_next < tolerance: #and len(path) > 1:
        path.pop(0)
    '''
    elif len(path) == 1 and distance_next < 5:
        path.pop(0)
    '''

    if len(path) > 0:
        theta = pos.theta
    else:
        theta = -1

    get_vel(theta)

# send new angle via XBee
def send_signal(vel):
    global serial
    print 'Sending {} as vel'.format(vel)

    right = np.uint8(vel[1])
    dir_r = np.unit8(1 if vel[1] > 0 else 0)

    left = np.uint8(vel[0])
    dir_l = np.unit8(1 if vel[0] > 0 else 0)

    serial.write(bytearray([right, dir_r, left, dir_l]))

def run(final_path):
    global path

    # initialize path
    path = final_path[1:]

    rospy.init_node('xbee', anonymous=True)
    print "Node XBee initialized"
    #Robot position
    rospy.Subscriber("/y_r0", Pose2D, update_robot)
