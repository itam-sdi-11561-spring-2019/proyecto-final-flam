#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose2D
from graphical_client.msg import Pose2D_Array

pos = Pose2D()
des = Pose2D()
obstacles = {}


def init_pose():
    pose = Pose2D()
    pose.x = 0
    pose.y = 0
    pose.theta = 0
    return pose

def robot_position(msg):
    print "Robot Position Update"
    print msg.x
    print msg.y
    print msg.theta
    pos.x = msg.x
    pos.y = msg.y
    pos.theta = msg.theta

def obstacle_position(msg, args):
    robot_id = args
    obstacles[robot_id] = msg
    print "Obstacle Position Update"
    print robot_id
    print obstacles

def destination_position(msg):
    print "Destination Position Update"
    print msg.x
    print msg.y
    des.x = msg.x
    des.y = msg.y
    des.theta = msg.theta

def talker():
    pub = rospy.Publisher('/trajectory', Pose2D_Array, queue_size=10)
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

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
