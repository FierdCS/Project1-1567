#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent, Led, Sound

# Publishers
velocityPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
Led1Pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
Led2Pub = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
soundPub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

# Globals
command = Twist()
target = Twist()
bumperActivated = True
emergency_brake = False
backwards = True
smootherMode = 1
leds = True
onlyBackwards = False

left = 0
right = 0
front = 0

def commandCallback(data):
    global bumperActivated, backwards, leds, emergency_brake, smootherMode
    bumperActivated = bool(data.data[0])
    backwards = bool(data.data[1])
    leds = bool(data.data[2])
    emergency_brake = bool(data.data[3])
    smootherMode = data.data[4]

def twistCallback(data):
    global target
    target = data

def bumperCallback(data):
    global left, right, front, onlyBackwards
    if not bumperActivated:
        return

    if data.bumper == 0:  # Left bumper
        if data.state == 1:
            left = 1
        else:
            left = 0
    elif data.bumper == 1:  # Front bumper
        if data.state == 1:
            front = 1
        else:
            front = 0
    elif data.bumper == 2:  # Right bumper
        if data.state == 1:
            right = 1
        else:
            right = 0


    if (left + right + front) > 0:
        emergencyBrake()
        onlyBackwards = True
    else:
        onlyBackwards = False

def emergencyBrake():
    global command
    command.linear.x = 0.0
    command.angular.z = 0.0
    velocityPub.publish(command)

def cliffCallback(data): #may be some errors in this callback need to fix
    global left, right, front, onlyBackwards

    if data.sensor == 0:  # Left bumper
        if data.state == 1:
            left = 1
        else:
            left = 0
    elif data.sensor == 1:  # Front bumper
        if data.state == 1:
            front = 1
        else:
            front = 0
    else:  # Right bumper
        if data.state == 1:
            right = 1
        else:
            right = 0


    if (left + right + front) > 0:
        emergencyBrake()
        onlyBackwards = True
    else:
        onlyBackwards = False
        
        
def wheelDropCallback(data):

def main():
    rospy.init_node("smoother", anonymous=True)

    rospy.Subscriber('/robot_twist', Twist, twistCallback)
    rospy.Subscriber('/robot_commands', Int32MultiArray, commandCallback)
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumperCallback)
    rospy.Subscriber('/mobile_base/commands/cliff', CliffEvent, cliffCallback)
    rospy.Subscriber('/mobile_base/commands/wheel_drop', WheelDropEvent, wheelDropCallback)

    rate = rospy.Rate(100)  # 10 ms
    while not rospy.is_shutdown():
        if target.linear.x > 0 and onlyBackwards and backwards:
            command.linear.x = 0
            command.angular.z = 0
        else:
            if smootherMode == 0:
                command.linear.x = target.linear.x
                command.angular.z = target.angular.z
            elif smootherMode == 1:
                if abs(target.linear.x - command.linear.x) > 0.0001:
                    if target.linear.x > command.linear.x:
                        command.linear.x += 0.03
                    else:
                        command.linear.x -= 0.03
                command.angular.z = target.angular.z
            elif smootherMode == 2:
                if abs(target.linear.x - command.linear.x) > 0.0001:
                    if target.linear.x > command.linear.x:
                        command.linear.x += 0.08
                    else:
                        command.linear.x -= 0.08
                command.angular.z = target.angular.z

        velocityPub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    main()
