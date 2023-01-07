#!/usr/bin/env python3

import rospy
from motion_planning.srv import Direction
import quad_pkg.config as config

# TO DO:
# wire LEDs directly to adafruit driver
# publish new topic from "motion_planning_main.cpp" which contains obstacle position
# subscribe to obstacle position and control LEDs accordingly

if __name__ == "__main__":
    rospy.init_node("LED_ctrl") 

    # wait for service to be started
    rospy.wait_for_service("/direction") # will block until service is started

    rate = rospy.Rate(5)

    # setup LEDs
    led_channel_A = config.pca.channels[0]
    led_channel_B = config.pca.channels[1]
    led_channel_C = config.pca.channels[14]
    led_channel_D = config.pca.channels[15]

    try:# catch exception
        # create client
        direction = rospy.ServiceProxy("/direction",  # name of service
                                       Direction)   # Datatype

        while not rospy.is_shutdown():
            response = direction(True) # this function calls the service and returns the response

            if response.dir == "step": # no obstacle ahead
                # turn on LEDs
                led_channel_A.duty_cycle = 0xffff
                led_channel_B.duty_cycle = 0xffff
                led_channel_C.duty_cycle = 0xffff
                led_channel_D.duty_cycle = 0xffff

            elif (response.dir == "turn") and (response.ang > 0): # obstacle front & right, turn left
                # static on left
                led_channel_B.duty_cycle = 0xffff
                led_channel_C.duty_cycle = 0xffff
                # blink right
                for i in range(0, 0xffff, 200):
                    led_channe_A.duty_cycle = i
                    led_channe_D.duty_cycle = i
                for i in range(0xffff,0, -200):
                    led_channe_A.duty_cycle = i
                    led_channe_D.duty_cycle = i

            elif (response.dir == "turn") and (response.ang < 0): # obstacle front & left, turn right
                # static on right
                led_channel_A.duty_cycle = 0xffff
                led_channel_D.duty_cycle = 0xffff
                # blink left
                for i in range(0, 0xffff, 200):
                    led_channe_B.duty_cycle = i
                    led_channe_C.duty_cycle = i
                for i in range(0xffff,0, -200):
                    led_channe_B.duty_cycle = i
                    led_channe_C.duty_cycle = i
            else:
                pass
            
            rate.sleep()

    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))