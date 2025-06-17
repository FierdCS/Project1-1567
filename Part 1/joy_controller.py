#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

robotpub = rospy.Publisher("/robot_twist", Twist, queue_size=10)
smootherPub = rospy.Publisher("/robot_commands", INT[],)#these means we are publishing DOWN to smoother

command = Twist()

def joystickCallback(data):
    global pub, command
    
    
    RT = data.axes[5] # gas peddle
    a_button = data.buttons[0] #go backwards
    left_stick = data.axes[0] #left stick
    LT = data.axes[2] #brake

    #new for project 1

    #bumper_button= data.buttons[] # whichever button for bumper
    #backward_button= data.buttons[] # whichever button for backward only
    
    #other button = data.buttons[] #for led
    #y_button = data.buttons[3] # for emergency brake
    b_button = data.buttons[1] #b button, smoother toggle

    

    eco_mode= 1 #eco is default 1, 0 is off, 2 is sport
    smoother_com = [1, 1, 1, 0, 1] # by default, bumper, backward, LEDS, emergency brake, smoothing mode
    
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

    
    robotpub.publish(command)#twist
        


    smootherPub.publish(smoother_com)#array
    

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
