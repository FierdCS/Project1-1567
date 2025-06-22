#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent, Led, Sound, CliffEvent, WheelDropEvent

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
smootherMode = 1  # Eco mode
leds = True
onlyBackwards = False
received_command = False

# bumper checks
bumperLeft = 0
bumperRight = 0
bumperFront = 0

# cliff checks
cliffLeft = 0
cliffRight = 0
cliffFront = 0

# wheel drop checks
wheelDropLeft = 0
wheelDropRight = 0

def commandCallback(data):
    global bumperActivated, backwards, leds, emergency_brake, smootherMode, received_command
    bumperActivated = bool(data.data[0])
    backwards = bool(data.data[1])
    leds = bool(data.data[2])
    emergency_brake = bool(data.data[3])
    smootherMode = data.data[4]
    received_command = True

def twistCallback(data):
    global target, received_command
    target = data
    received_command = True

def bumperCallback(data):
    global bumperLeft, bumperRight, bumperFront, onlyBackwards
    if not bumperActivated:
        return

    if data.bumper == 0:
        
        if data.state == 1:
            bumperLeft = 1 
        else:
            bumperLeft = 0
        
    elif data.bumper == 1:
        if data.state == 1:
            bumperFront = 1 
        else:
            bumperFront = 0
      
    elif data.bumper == 2:
        if data.state == 1:
            bumperRight = 1 
        else:
            bumperRight = 0
        

    if (bumperLeft + bumperRight + bumperFront) > 0:
        emergencyBrake()
        onlyBackwards = True
    else:
        onlyBackwards = False

def cliffCallback(data):
    global cliffLeft, cliffRight, cliffFront, onlyBackwards

    if data.sensor == 0:
        if data.state == 0:
            cliffLeft = 0
        else:
            cliffLeft = 1
    elif data.sensor == 1:
        if data.state == 0:
            cliffFront = 0
        else:
            cliffFront = 1
    else:
        if data.state == 0:
            cliffRight = 0
        else:
            cliffRight = 1

    if (cliffLeft + cliffRight + cliffFront) > 0:
        emergencyBrake()
        onlyBackwards = True
    else:
        onlyBackwards = False

def wheelDropCallback(data):
    global wheelDropLeft, wheelDropRight, onlyBackwards

    if data.wheel == 0:
        if data.state == 1:
            wheelDropLeft = 1
        else:
            wheelDropLeft = 0
    else:
        if data.state == 1:
            wheelDropRight = 1
        else:
            wheelDropRight = 0

    if (wheelDropLeft + wheelDropRight) > 0:
        emergencyBrake()
        onlyBackwards = True
    else:
        onlyBackwards = False

def emergencyBrake():
    global command
    command.linear.x = 0.0
    command.angular.z = 0.0
    velocityPub.publish(command)

def main():
    global command, smootherMode, received_command, target, onlyBackwards

    rospy.init_node("smoother", anonymous=True)

    rospy.Subscriber('/robot_twist', Twist, twistCallback)
    rospy.Subscriber('/robot_commands', Int32MultiArray, commandCallback)
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumperCallback)
    rospy.Subscriber('/mobile_base/commands/cliff', CliffEvent, cliffCallback)
    rospy.Subscriber('/mobile_base/commands/wheel_drop', WheelDropEvent, wheelDropCallback)

    rospy.loginfo("Waiting for /mobile_base/commands/velocity subscribers...")
    stop_cmd = Twist()
    while velocityPub.get_num_connections() == 0 and not rospy.is_shutdown():
        velocityPub.publish(stop_cmd)
        rospy.sleep(0.1)

    

    # if not emergency_brake:
    #     sound_msg = Sound()
    #     sound_msg.value = Sound.ON
    #     soundPub.publish(sound_msg)
    
    
    # if leds:
    #     led_msg = Led()
    #     led_msg.value = Led.GREEN
    #     Led1Pub.publish(led_msg)
    #     Led2Pub.publish(led_msg)

    # #if not emergency_brake:
    #     sound_msg = Sound()
    #     sound_msg.value = Sound.ON
    #     soundPub.publish(sound_msg)

    rate = rospy.Rate(100)  # 10 ms
    while not rospy.is_shutdown():
        if not received_command:
            command.linear.x = 0.0
            command.angular.z = 0.0
            velocityPub.publish(command)
            rate.sleep()
            continue

        if target.linear.x > 0 and onlyBackwards and backwards:
            command.linear.x = 0
            command.angular.z = 0
        else:
            if smootherMode == 0:
                rospy.loginfo("OFF MODE")
                command.linear.x = target.linear.x
                command.angular.z = target.angular.z
            elif smootherMode == 1:
                rospy.loginfo("ECO MODE")
                if abs(target.linear.x - command.linear.x) > 0.0001:
                    if target.linear.x > command.linear.x:
                        command.linear.x += 0.03
                    else:
                        command.linear.x -= 0.03
                command.angular.z = target.angular.z
            elif smootherMode == 2:
                rospy.loginfo("SPORT MODE")
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
