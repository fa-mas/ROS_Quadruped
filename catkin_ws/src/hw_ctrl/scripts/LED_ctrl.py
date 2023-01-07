#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import quad_pkg.config as config

# hw_motion_main.py needs to run before LED_ctrl

# TO DO:
# wire LEDs directly to adafruit driver GND -> cathode, PWM -> anode, no resistor
# publish new topic from "motion_planning_main.cpp" which contains obstacle position "obstacle_pos"
# subscribe to obstacle position and control LEDs accordingly

def callback_ctrl_LEDs(msg):
    # if obstacle there,
        # do this etc...
        # find LEDs in config

if __name__ == "__main__":
    rospy.init_node("LED_ctrl") 

    sub = rospy.Subscriber("obstacle_pos", String, callback_ctrl_LEDs)
    
    rospy.spin()
    rospy.loginfo("LED_ctrl shut down")