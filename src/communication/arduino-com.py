#!/usr/bin/env python


import rospy
from std_msgs.msg import Float64

def talker():
    pubL = rospy.Publisher('leftWheel', Float64)
    pubR = rospy.Publisher('rightWheel', Float64)
    rospy.init_node('communication node initialized')

    rate = rospy.Rate(1) # 10hz

    while not rospy.is_shutdown():
        commandL = input('Please enter a Left speed between 0.0 - 255.0:')
        commandR = input('Please enter a Right speed between 0.0 - 255.0:')
        pubL.publish(commandL)
        pubR.publish(commandR)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: 
        pass