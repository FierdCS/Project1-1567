#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty, Float32
from std_msgs.msg import Empty


curX = 0
curY = 0
curDeg = 0
initX = 0
initY = 0
initDeg = 0

velocityPub = None
resetOdomPub = None


class move:
    def __init__(self, type, speed, dist):
        self.type = type
        self.speed = speed
        self.dist = dist

def executeMoves(moves):
    for move in moves:
        if move.type == "T":
            turn(move.speed, move.dist)
        else:
            drive(move.speed, move.dist)

def resetOdom():
    global initX,initY,initDeg
    resetOdomPub.publish(Empty())
    initX = curX
    initY = curY
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
        if abs(distRemaining) <= 0.01: #if close enough to target distance stop robot
            command.linear.x = 0.0
            command.angular.z = 0.0
            rospy.sleep(0.5)  # Allow time for the command to be processed
            velocityPub.publish(command)
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
       


def turn(speed, degrees):
    pass

def controller():
    global velocityPub, resetOdomPub


    rospy.init_node("controller", anonymous=True)
    velocityPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    resetOdomPub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odomCallback)


    moves = []
    print("Enter moves in format:'T, 0.5, 90'(turn) or 'D, 0.8, 3'(drive) '0.0'(end)")
    while True:
        curMove = raw_input("Enter Move: ").strip()
        if curMove == "0.0":
            break
        parts = [part for part in curMove.split(",")]
        curType = parts[0].strip().upper()
        curSpeed = float(parts[1].strip()) 
        curDist = float(parts[2].strip())
        newMove = move(curType,curSpeed,curDist)
        moves.append(newMove)

    if moves: 
        rospy.sleep(4)#time to set up the robot
        executeMoves(moves)   
    else:
        print("no moves to execute")   
    rospy.spin() 

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
    
