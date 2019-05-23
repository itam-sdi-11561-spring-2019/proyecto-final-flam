#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose2D
from graphical_client.msg import Pose2D_Array
from AStar import run_astar
import numpy as np
import math
import serial
import pickle as pkl

#---------------------------------------------------------
#               AStar variables
#---------------------------------------------------------
obstacles = {}
pos = Pose2D()
end = Pose2D()
no_topics = 9
no_robots = 5
ready = False

#---------------------------------------------------------
#               XBee variables
#---------------------------------------------------------
PORT = '/dev/ttyUSB0'
BAUD = 9600
serial_port = serial.Serial(PORT, BAUD)
path = None
tolerance = 20

#---------------------------------------------------------
#                  Aux functions
#---------------------------------------------------------
'''
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

def send_signal(vel):
    global serial
    print 'Sending {} as vel'.format(vel)

    right = np.uint8(vel[1])
    dir_r = np.unit8(1 if vel[1] > 0 else 0)

    left = np.uint8(vel[0])
    dir_l = np.unit8(1 if vel[0] > 0 else 0)
'''

def publish(path):
    arr = Pose2D_Array()

    for p in path:
        pose = Pose2D()

        pose.x = p[0]
        pose.y = p[1]
        pose.theta = p[2]

        arr.poses.append(pos)

    pub.publish(arr)

    print 'Trajectory published'

#---------------------------------------------------------
#                    Logic functions
#---------------------------------------------------------
'''
def update_robot(pos):
    global path
    current_pos = (pos.x,pos.y)

    distance_next = get_distance(current_pos, path[0])

    if distance_next < tolerance: 
        path.pop(0)

    if len(path) > 0:
        theta = pos.theta
    else:
        theta = -1

    get_vel(theta)
'''

def calculate_trajectory():
    global path
    print "Running A* module"
    
    obstacles_pos = []

    for obs in obstacles:
        obstacles_pos.append((obstacles[obs].x,obstacles[obs].y))
    
    start = (pos.x, pos.y)
    destination = (end.x,end.y)

    if len(obstacles_pos) == 0:
        trajectory = [start,destination]
    else:
        trajectory = run_astar(obstacles_pos, start, destination)
    
    print trajectory

    publish(trajectory)

#---------------------------------------------------------
#                  Callbacks
#---------------------------------------------------------
def robot_position(msg):
    global pos

    pos.x = msg.x
    pos.y = msg.y
    pos.theta = msg.theta
    '''
    if not ready:
        pos.x = msg.x
        pos.y = msg.y
        pos.theta = msg.theta
    else:
        update_robot(msg)
    '''

def final_position(msg):
    global end

    end.x = msg.x
    end.y = msg.y
    end.theta = msg.theta

def obstacle_position(msg, args):
    global obstacles
    global ready

    robot_id = args
    obstacles[robot_id] = msg

    if not ready and len(obstacles.keys()) == no_robots:
        ready = True
        calculate_trajectory()

#---------------------------------------------------------
#               ROS communication
#---------------------------------------------------------

def receiver():
    rospy.init_node('receiver', anonymous=True)
    print "Node initialized"
    
    pub = rospy.Publisher('/trajectory', Pose2D_Array, queue_size=10)

    #Robot position
    rospy.Subscriber("/y_r0", Pose2D, robot_position)

    #Final destination
    rospy.Subscriber("/ball", Pose2D, final_position)
    
    _id = "/b_r{}"
    for i in range(no_topics):
        obstacle_id = _id.format(i)
        #Subscribe to n obstacles
        rospy.Subscriber(obstacle_id, Pose2D, obstacle_position,(obstacle_id))
    
    rate = rospy.Rate(1) # 10hz

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        receiver()
    except rospy.ROSInterruptException:
        pass
