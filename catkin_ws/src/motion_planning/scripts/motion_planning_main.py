#!/usr/bin/env python3 
# use python3 on noetic, python2 on melodic
import rospy
from sensor_msgs.msg import LaserScan # Datastructure
from rospy.numpy_msg import numpy_msg
# from numpy import array

# define callback function
def callback_laser_data(msg):

    rospy.loginfo("...")
    
    n = round(len(msg.ranges)/8)
    radius = .5

    # angle increment of measurement
    alpha = 30
    step = round((n*alpha)/360)

    # print length of msg.ranges (for c++ file)
    # rospy.loginfo(len(msg.ranges))

    # front
    for i in msg.ranges[0:n:step]:
        if i < radius:
            rospy.loginfo("front")

    for i in msg.ranges[(7*n):(8*n):step]:
        if i < radius:
            rospy.loginfo("front")
    
    # left
    for i in msg.ranges[n:(3*n):step]:
        if i < radius:
            rospy.loginfo("left")

    # back
    for i in msg.ranges[(3*n):(5*n):step]:
        if i < radius:
            rospy.loginfo("back")

    # right
    for i in msg.ranges[(5*n):(7*n):step]:
        if i < radius:
            rospy.loginfo("right")

if __name__ == '__main__':
    rospy.init_node("motion_planning_main") # make Anonymous with anonymous=True

    # obstacleVec = array([0,0])

    # create Subscriber object
    sub = rospy.Subscriber("/scan", LaserScan, callback_laser_data)

    rospy.spin() 

rospy.loginfo("Node shut down")