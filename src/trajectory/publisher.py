#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose2D
from graphical_client.msg import Pose2D_Array
from AStar import run_astar

obstacles = {}

pos = Pose2D()
end = Pose2D()

no_robots = 9
ready = False

def robot_position(msg):
    global pos
    print "Robot Position Update"
    print msg.x
    print msg.y
    print msg.theta
    pos.x = msg.x
    pos.y = msg.y
    pos.theta = msg.theta

def destination_position(msg):
    global end
    print "Destination Position Update"
    print msg.x
    print msg.y
    end.x = msg.x
    end.y = msg.y
    end.theta = msg.theta

def obstacle_position(msg, args):
    global obstacles
    robot_id = args
    obstacles[robot_id] = msg

    print "Obstacle Position Update"
    print robot_id
    print obstacles

    if not ready and len(obstacles.keys()) == no_robots:
        ready = True
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
    
    #Robot position
    rospy.Subscriber("/y_r0", Pose2D, robot_position)

    #Final destination
    rospy.Subscriber("/ball", Pose2D, final_position)
    
    _id = "/b_r{}"
    for i in range(no_robots):
        obstacle_id = _id.format(i)
        #Subscribe to n obstacles
        rospy.Subscriber(obstacle_id, Pose2D, obstacle_position,(obstacle_id))
    
    rate = rospy.Rate(1) # 10hz

    while not rospy.is_shutdown():
        rate.sleep()

'''
def talker():
    #pub = rospy.Publisher('/trajectory', Pose2D_Array, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber("/y_r0", Pose2D, robot_position)
    rospy.Subscriber("/ball", Pose2D, destination_position)
    _id = "/b_r0"
    rospy.Subscriber(_id, Pose2D, obstacle_position,(_id))
    rate = rospy.Rate(1) # 10hz
    aux = 1
    while not rospy.is_shutdown():
        # arr = Pose2D_Array()
        # for i in range(10):
        #     pose = init_pose()
        #     pose.x = 100 * ( i + 1 ) 
        #     pose.y = 150 * ( i + 1 ) * aux
        #     pose.theta +=0.7853 * i
        #     arr.poses.append(pose)
        #     aux *= -1
        # print "The array is:", arr
	
	arr = Pose2D_Array()
	arr.poses.append(pos)        
        arr.poses.append(des)
        pub.publish(arr)
        
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        rate.sleep()
'''

if __name__ == '__main__':
    try:
        receiver()
    except rospy.ROSInterruptException:
        pass
