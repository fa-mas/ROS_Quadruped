#!/usr/bin/env python3

import rospy
from motion_planning.srv import Direction

if __name__ == "__main__":
    rospy.init_node("hw_motion_main") 

    # wait for service to be started
    rospy.wait_for_service("/direction") # will block until service is started

    rate = rospy.Rate(10) # 10Hz loop

    try:# catch exception
        # create client
        direction = rospy.ServiceProxy("/direction",  # name of service
                                       Direction)   # Datatype

        while not rospy.is_shutdown():
            response = direction(True) # this function calls the service and returns the response
            rospy.loginfo("Response: " + str(response.dir) + str(response.ang) + str(response.vec))
            rate.sleep()

    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))