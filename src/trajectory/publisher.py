#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose2D
from graphical_client.msg import Pose2D_Array
from AStar import run_astar
import pickle as pkl

obstacles = {}

pos = Pose2D()
end = Pose2D()


no_topics = 9
no_robots = 5

ready = False

def robot_position(msg):
    global pos

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


    if not ready and len(obstacles.keys()) == no_robots:
        ready = True
        file = open("bag.pkl","wb")

        bag = {"obstacles": obstacles,
        "start": pos,
        "end": end}
        pkl.dump(bag,file)
        file.close
        calculate_trajectory()

def calculate_trajectory():
    print "Running A* module"
    
    obstacles_pos = []

    for obs in obstacles:
        obstacles_pos.append((obstacles[obs].x,obstacles[obs].y))
    
    start = (pos.x, pos.y)
    destination = (end.x,end.y)

    trajectory = run_astar(obstacles_pos, start, destination)
    
    print trajectory

def receiver():
    rospy.init_node('receiver', anonymous=True)
    print "Node initialized"
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
