#!/usr/bin/env python
import rospy
import math 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy


curX = 0
curY = 0
curDeg = 0
initX = 0
initY = 0
initDeg = 0

velocityPub = None
resetOdomPub = None

moveMode = 0




class move:
    def __init__(self, type, speed, dist):
        self.type = type
        self.speed = speed
        self.dist = dist


def resetOdom():
    global initX,initY,initDeg
    resetOdomPub.publish(Empty())
    rospy.sleep(0.5)  
    initX = curX
    initY  = curY
    initDeg = curDeg
        
def odomCallback(data):
    global curX,curY,curDeg
    q = [data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(q)

    curDeg = yaw * 180 / math.pi
    curX = data.pose.pose.position.x
    curY = data.pose.pose.position.y


def drive(speed, distance):
    resetOdom()
    global moveMode
    command = Twist()
    rate = rospy.Rate(10)
    current_speed = 0.0
    direction = 1 if speed > 0 else -1

    command.linear.x = 0.0
    command.angular.z = 0.0
    velocityPub.publish(command)

    while not rospy.is_shutdown():
        distTravelled = math.sqrt((curX - initX)**2 + (curY-initY)**2)
        distRemaining = distance - distTravelled
        if abs(distRemaining) <= 0.001: 
            command.linear.x = 0.0
            command.angular.z = 0.0
            rospy.sleep(0.5)  
            velocityPub.publish(command)
            moveMode = 0
            break
        
        if abs(current_speed) < abs(speed):
            current_speed += 0.04 * direction
            current_speed = min(abs(speed), abs(current_speed)) * direction

        if(distRemaining < abs(current_speed)):
            current_speed = direction * distRemaining
        
        command.linear.x = current_speed
        command.angular.z = 0.0
      
        velocityPub.publish(command)
        print("Current Position: X: {}, Y: {}, Degrees: {}".format(curX, curY, curDeg))
        rate.sleep()



def handleWrap(angle1, angle2):
    diff = angle2 - angle1
    if diff > 180:
        diff -= 360
    if diff < -180:
        diff += 360
    return diff


def turn(speed, degrees):
    resetOdom()
    global moveMode
    
    command = Twist()
    rate = rospy.Rate(10)
    current_speed = 0.0
    direction = 1 if speed > 0 else -1

    command.linear.x = 0.0
    command.angular.z = 0.0
    velocityPub.publish(command)

    target_angle = degrees
    lastAngle = curDeg
    curAngle = 0.0

    while not rospy.is_shutdown():
        Change = handleWrap(lastAngle, curDeg)
        curAngle += abs(Change)
        lastAngle = curDeg

        angleRemaining = target_angle - curAngle

        if abs(angleRemaining) <= .3: 
            command.linear.x = 0.0
            command.angular.z = 0.0
            velocityPub.publish(command)
            rospy.sleep(0.5)
            moveMode = 0
            break
        
        if abs(current_speed) < abs(speed):
            current_speed += 0.04 * direction
            current_speed = min(abs(speed), abs(current_speed), 0.7) * direction

        if(abs(angleRemaining) < 20):
            current_speed = max(abs(angleRemaining/100), 0.05)* direction
        
        command.linear.x = 0.0
        command.angular.z = current_speed
      
        velocityPub.publish(command)
        print("Current Position: X: {}, Y: {}, Degrees: {}".format(curX, curY, curDeg))
        print("current speed: {}, angle remaining: {}".format(current_speed, angleRemaining))
        rate.sleep()

def joystickCallback(data):
    global moveMode
    cross_horizontal = data.axes[6]  
    cross_vertical = data.axes[7] 
    print("cross_horizontal: {}, cross_vertical: {}".format(cross_horizontal, cross_vertical))
    if(cross_vertical > 0.7 and abs(cross_vertical) > abs(cross_horizontal)): #drive forward 1m
        moveMode =1 #move forward
    elif(cross_vertical < -0.7 and abs(cross_vertical) > abs(cross_horizontal)): #drive backward 1m
        moveMode = 2
    elif(cross_horizontal > 0.7 and abs(cross_vertical) < abs(cross_horizontal)): #turn left 90 degrees
        moveMode= 3
    elif(cross_horizontal < -0.7 and abs(cross_vertical) < abs(cross_horizontal)): #turn right 90 degrees
        moveMode = 4
    

def controller():
    global velocityPub, resetOdomPub, moveMode
    rospy.init_node("controller", anonymous=True)
    velocityPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    resetOdomPub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    rospy.Subscriber("/joy", Joy, joystickCallback)
    
    while not rospy.is_shutdown():
        if(moveMode==0):
            print("move mode 0")
            rospy.sleep(0.5)
        if(moveMode == 1):
            print("move mode 1")
            drive(0.5, 1.0)
        elif(moveMode == 2):
            print("move mode 2")
            drive(-0.5, 1.0)
        elif(moveMode== 3):
            print("move mode 3")
            turn(0.5, 90)
        elif(moveMode ==4):
            print("move mode 4")
            turn(-0.5, 90)         
    
    



if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
    