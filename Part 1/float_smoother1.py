#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32


target = 0.0

def smootherCallback(data):
    global target 
    target = data.data

def smoother():
    rospy.init_node("smoother", anonymous = True)
    rospy.Subscriber('float_command', Float32, smootherCallback)
    current = 0
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if math.fabs(target - current) > 0.0001:
            if target > current:
                current += .01
            else:
                current -= .01
        print(current)
        rate.sleep()


if __name__ == '__main__':
    smoother()



