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
end = None
no_topics = 9
no_robots = 0
ready = False

path = None

def publish(final_path):
    global pub
    global path

    arr = Pose2D_Array()

    for p in final_path:
        pose = Pose2D()

        pose.x = p[0]
        pose.y = p[1]
        pose.theta = p[2]

        arr.poses.append(pose)

    path = arr

    pub.publish(arr)

    print 'Trajectory published'

def get_theta(start, end):
    x = end[0] - start[0]
    y = end[1] - start[1]

    theta = 0

    if y >= 0:
        theta = math.atan2(x,y)
    else:
        theta = math.pi + math.atan2(x,y)
    
    theta = theta if theta <= 2*math.pi else 2*math.pi    

    return theta


#---------------------------------------------------------
#                    Logic functions
#---------------------------------------------------------
def calculate_trajectory():
    global path
    print "Running A* module"
    start = (pos.x,pos.y)
    destination = (end.x,pos.y)
    if no_robots == 0:
        start_0 = (start[0], start[1], 0)
        angle = get_theta(start,destination)
        dest_0 = (destination[0], destination[1],angle)
        trajectory = [start_0,dest_0]
    else:
        obstacles_pos = []

        for obs in obstacles:
            obstacles_pos.append((obstacles[obs].x,obstacles[obs].y))
        
        trajectory = run_astar(obstacles_pos, start, destination)
    
    print trajectory

    publish(trajectory)

#---------------------------------------------------------
#                  Callbacks
#---------------------------------------------------------
def robot_position(msg):
    global pos

    print "Recibi inicio"

    pos = Pose2D()
    pos.x = msg.x
    pos.y = msg.y
    pos.theta = msg.theta

    if no_robots == 0 and (not end is None):
        calculate_trajectory()

def final_position(msg):
    global end

    print "Recibi meta"

    end = Pose2D()
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
    global path
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
        if not path is None:
            pub.publish(path)
        rate.sleep()

if __name__ == '__main__':
    try:
        receiver()
    except rospy.ROSInterruptException:
        pass
