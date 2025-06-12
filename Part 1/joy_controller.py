#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
command = Twist()

def joystickCallback(data):
    global pub, command
    
    
    RT = data.axes[5] # gas peddle
    a_button = data.buttons[0] #go backwards
    left_stick = data.axes[0] #left stick
    b_button = data.buttons[1] #b button
    LT = data.axes[2] #brake

    #new for project 1

    y_button = data.buttons[3] # for sport or eco
    



    if(b_button ==1):
        rospy.signal_shutdown("Emergency Stop!!!")
    if(a_button == 1):
        
            if(RT <0.2):
                command.linear.x = -0.8
            else:
                command.linear.x = -1+RT
       
        
    else :#a is not pressed
        
            if(RT < 0.2): #max speed
                command.linear.x = 0.8
            else:   #a is not pressed plus not pulled down 80%
                command.linear.x = 1-RT
        

    #if we hold a 
    
    
    
    command.angular.z = left_stick  #left is 1 , right is -1
    
    
    if(LT <=0.5):
        command.linear.x = 0.0
        command.angular.z = 0.0

    
    pub.publish(command)

def cleanUp():
    global pub, command
    command.linear.x = 0.0
    command.angular.z = 0.0
    pub.publish(command)
    rospy.sleep(1)

def remoteController():
    rospy.init_node("remoteControl", anonymous=True)
    rospy.Subscriber("joy", Joy, joystickCallback)
    rospy.on_shutdown(cleanUp)


    rospy.spin()

    

if __name__ == '__main__':
    remoteController()
