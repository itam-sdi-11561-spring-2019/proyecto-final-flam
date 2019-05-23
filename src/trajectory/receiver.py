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
pos = None
end = Pose2D()
no_topics = 9
no_robots = 0
ready = False

pub = None

#---------------------------------------------------------
#                  Aux functions
#---------------------------------------------------------
def publish(path):
    global pub
    arr = Pose2D_Array()

    for p in path:
        pose = Pose2D()

        pose.x = p[0]
        pose.y = p[1]
        pose.theta = p[2]

        arr.poses.append(pose)

    pub.publish(arr)

    print 'Trajectory published'

#---------------------------------------------------------
#                    Logic functions
#---------------------------------------------------------
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
    pos = Pose2D()
    pos.x = msg.x
    pos.y = msg.y
    pos.theta = msg.theta


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

    if not ready and len(obstacles.keys()) == no_robots and not pos is None:
        ready = True
        calculate_trajectory()

#---------------------------------------------------------
#               ROS communication
#---------------------------------------------------------

def receiver():
    global pub
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
