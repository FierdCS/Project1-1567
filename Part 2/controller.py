#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty, Float32



def drive(speed, distance):
    rospy.init_node("controller", anonymous=True)
    pub1 = rospy.Publisher('velocity', Float32, queue_size = 1)

    rospy.init_node("controller", anonymous=True)
    pub2 = rospy.Publisher('reset_odometry', Float32, queue_size = 1)

    rospy.init_node("controller", anonymous = True)
    rospy.Subscriber('odom', Float32, smootherCallback)
    

def turn(speed, degrees):
    rospy.init_node("controller", anonymous=True)
    pub1 = rospy.Publisher('velocity', Float32, queue_size = 1)

    rospy.init_node("controller", anonymous=True)
    pub2 = rospy.Publisher('reset_odometry', Float32, queue_size = 1)


    rospy.init_node("controller", anonymous = True)
    rospy.Subscriber('odom', Float32, smootherCallback)

def controller():
    while not rospy.is_shutdown():
        print("Move Format t(turn)/d(drive), speed, amount. Ex. 'd, 0.8, 1.5'")
        move = input("Enter your move: ")
        parts = move.split(", ")

        if(parts[0] == 'd'):
            drive(float(parts[1]), float(parts[2]))
        elif (parts[0] == 't'):
            turn(float(parts[1]), float(parts[2]))
        else:
            print("invalid option, enter t or d")
        

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
