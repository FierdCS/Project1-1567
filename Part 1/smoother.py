#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

velocityPub = rospy.publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
Led1Pub = rospy.publisher('/mobile_base/commands/led1', Led, queue_size=1)
Led2Pub = rospy.publisher('/mobile_base/commands/led2', Led, queue_size=1)
soundPub = rospy.publisher('/mobile_base/commands/sound', Sound, queue_size=1)


command = Twist()

emergency_brake = False
backwards = True


preventCollision = False


def commandCallback(data):#armon wang me 
    global bumperActivated, backwards, leds, emergency_brake, smootherMode, command

    bumperActivated = bool(data.data[0])
    backwards = bool(data.data[1])
    leds = bool(data.data[2])
    emergency_brake = bool(data.data[3])
    smootherMode = data.data[4]#0, 1, or 2


    #if not emergency brake

    
            

    #else


    

def twistCallback(data):#HAS A TWIST ITEM wang me


    if(data.linear.x >0 and preventCollision and backwards):
        command.linear.x = 0
        command.angular.z = 0


    #process angular and linear
    
    
    velocicityPub.publish(command)


def bumperCallback(data):
    global bumper_hit, front, left, right, preventCollision
    if not bumperActivated:
        return
    else:
        if data.bumper == 0:
            if(data.state== 1):
                left = 1
            else:
                left = 0
        elif data.bumper == 1:
            if(data.state == 1):
                front = 1
            else:
                front = 0
        elif data.bumper == 2:
            if(data.state == 1):
                right = 1
            else:
                right = 0

        if(left+right+front>0):
            emergency_brake()
            preventCollision = True #can only back up no forward movement


def emergencyBrake():#armon
    global emergency_brake
    command


target = 0.0

def driveMode():#ewang me, smoother mode
    global target #array
    target = data.data[4]

def cliffCallback(data): # wang me

def wheelDropEvent(data): #wang me armon


def main():
    rospy.init_node("smoother", anonymous = True)
    #subscribers
    rospy.Subscribe('/robot_twist', Twist, twistCallback)
    rospy.Subscribe('/robot_commands', Int32MultiArray, commandCallback)
    rospy.Subscribe('/mobile_base/events/bumper', BumperEvent, bumperCallback)
    rospy.Subscribe('/movile_base/events/cliff', CliffEvent, cliffCallback)
    rospy.Subscribe('/mobile_base/events/wheel_drop', WheelDropEvent, wheelDropCallback)
    rospu.spin()
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
    main()



