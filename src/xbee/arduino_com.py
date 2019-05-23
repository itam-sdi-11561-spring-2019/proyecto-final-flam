#!/usr/bin/env python

#imports
import rospy
import serial 
import math
import numpy as np
from graphical_client.msg import Pose2D_Array
from geometry_msgs.msg import Pose2D

# GLOBAL VARIABLES
PORT = '/dev/ttyUSB0'
BAUD = 9600
#serial_port = serial.Serial(PORT, BAUD)

path = None
ready = False
tolerance = 20

def get_distance(start,end):
    return math.sqrt(math.pow(start[0] - end[0],2) + math.pow(start[1] - end[1],2))

def get_vel(omega):
    r = 21.0
    v = 630.0
    matrix = np.matrix('1 -57.5; 1 57.5')

    if omega == -1:
        vel = np.array([0,0])
    else:
        vector = np.array([v, omega])
        vel = (1.0/r) * np.dot(matrix, vector)

    print "Sending " + str(vel) + " as vel"

    #send_signal(vel)

# send new angle via XBee
def send_signal(vel):
    global serial

    right = np.uint8(vel[1])
    dir_r = np.unit8(1 if vel[1] > 0 else 0)

    left = np.uint8(vel[0])
    dir_l = np.unit8(1 if vel[0] > 0 else 0)

    serial.write(bytearray([right, dir_r, left, dir_l]))

#--------------------------------------------------------
#                   CALLBACKS
#-------------------------------------------------------

# get robot's current position and adjust angle to target
def update_robot(pos):
    global path
    global ready
    if ready and len(path) > 0:
        current_pos = (pos.x,pos.y)

        distance_next = get_distance(current_pos, path[0])

        if distance_next < tolerance:
            path.pop(0)

        if len(path) > 0:
            print "Robot angle " + str(pos.theta)
            print "Target angle" + str(path[0][2])
            theta = path[0][2] - pos.theta
        else:
            theta = -1

        print "New angle " + str(theta)

        get_vel(theta)

def get_path(trayectory):
    global path
    global ready
    path = []

    for node in trayectory.poses:
        path.append((node.x,node.y,node.theta))

    path = path[1:]

    ready = True

def run():
    print 'RUN function'
    rospy.init_node('xbee', anonymous=True)
    print "Node XBee initialized"
    #Robot position
    rospy.Subscriber("/y_r0", Pose2D, update_robot)
    # Get trajectory from AStar node
    rospy.Subscriber("/trajectory", Pose2D_Array, get_path)

    rate = rospy.Rate(2) # 10hz

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException: 
        pass
