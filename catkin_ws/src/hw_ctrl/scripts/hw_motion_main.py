#!/usr/bin/env python3

import rospy
from motion_planning.srv import Direction

from numpy import array as arr
import RPi.GPIO as GPIO
import quad_pkg.motion as QM
import quad_pkg.config as config
import sys
import time

if __name__ == "__main__":
    rospy.init_node("hw_motion_main") 

    # wait for service to be started
    rospy.wait_for_service("/direction") # will block until service is started

    ### delete later
    rate = rospy.Rate(10) # 10Hz loop
    ###

    config.setup()

    # constants
    # reference vectors XY for alpha calc
    a0 =  arr([-1, 1]) # = [ax, ay]
    b0 =  arr([ 1, 1]) # = [bx, by]
    c0 =  arr([ 1, -1]) # etc...
    d0 =  arr([-1, -1])

    # reference vectors for Turn, points from center of body to hip
    aW = arr([-82.5, -64, 0]) 
    bW = arr([-82.5,  64, 0])
    cW = arr([ 82.5,  64, 0])
    dW = arr([ 82.5, -64, 0]) 

    legLen1 = 75  
    legLen2 = 55 # actually: 55  
    legLen3 = 200 # actually: 200

    # initial vectors
    xVal, yVal, zVal = 62, 112, 212
    vAPrev = arr([-xVal, -yVal, -zVal])
    vBPrev = arr([-xVal, yVal, -zVal])
    vCPrev = arr([xVal, yVal, -zVal])
    vDPrev = arr([xVal, -yVal, -zVal])

    # initial angles
    ACinitAlpha = 108
    BDinitAlpha = 72
    initBeta = 88
    initGamma = 90
    # because different Leglengths
    initBetaCD = initBeta # 61
    initGammaCD = initGamma #102

    # offsets (Correction)
    aAlphaOffset = 0
    bAlphaOffset = 0
    cAlphaOffset = 0
    dAlphaOffset = 0

    aBetaOffset = -6
    bBetaOffset = -6
    cBetaOffset = -9
    dBetaOffset = -10

    aGammaOffset = -55
    bGammaOffset = -53
    cGammaOffset = -55
    dGammaOffset = -45

    # instantiate Leg objects
    A = QM.Leg(vAPrev, a0, aW, ACinitAlpha, initBeta, initGamma, aAlphaOffset, aBetaOffset, aGammaOffset, legLen1, legLen2, legLen3)
    B = QM.Leg(vBPrev, b0, bW, BDinitAlpha, initBeta, initGamma, bAlphaOffset, bBetaOffset, bGammaOffset, legLen1, legLen2, legLen3)
    C = QM.Leg(vCPrev, c0, cW, ACinitAlpha, initBetaCD, initGammaCD, cAlphaOffset, cBetaOffset, cGammaOffset, legLen1, legLen2, legLen3)
    D = QM.Leg(vDPrev, d0, dW, BDinitAlpha, initBetaCD, initGammaCD, dAlphaOffset, dBetaOffset, dGammaOffset, legLen1, legLen2, legLen3)

    # instantiate Quadruped Object
    Quad = QM.Quadruped(A, B, C, D)
    # go to initial position 
    Quad.ServoFeed()

    """ 
    define input vectors 
    xLen = 75
    x1 = arr([xLen, 0, 0])
    x2 = arr([-xLen, 0, 0])
    x3 = arr([0, xLen, 0])
    x4 = arr([0, -xLen, 0])
    destLen = 1000
    dest1 = arr([destLen, 0, 0])
    dest2 = arr([-destLen, 0, 0])
    dest3 = arr([0, destLen, 0])
    dest4 = arr([0, -destLen, 0])
    """

    nInc = 50
    nInc1 = 30
    stepHeightCrawl = 50
    stepHeightTurn = 40
    stepHeightRun = 30

    try:# catch exception
        # create client
        direction = rospy.ServiceProxy("/direction",  # name of service
                                       Direction)   # Datatype

        while not rospy.is_shutdown():
            response = direction(True) # this function calls the service and returns the response
            rospy.loginfo("Response: " + str(response.dir) + str(response.ang) + str(response.vec))

            # if bla: Circle crawl usw...

            ### delete later
            rate.sleep()
            ###

    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))