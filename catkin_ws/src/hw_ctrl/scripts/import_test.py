#!/usr/bin/env python3

# 3rd party
import rospy
import numpy
import RPi.GPIO as GPIO
import time
import board
import busio
from adafruit_servokit import ServoKit

# custom
from quad_pkg.config import say_hi2
import quad_pkg.motion as motion

if __name__ == '__main__':
    rospy.init_node('import_test')

    # numpy
    numpy.array([5, 4, 3])

    # RPi
    GPIO.setmode(GPIO.BCM)

    # adafruit
    i2c = busio.I2C(board.SCL, board.SDA)
    kit = ServoKit(channels=16)

    # core
    time.sleep(1)



    rospy.loginfo("done")
