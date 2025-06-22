#!/usr/bin/env python
import rospy
from kobuki_msgs.msg import Led, Sound

# to implement:
# 1. flashing led/sound when moving backwards (condition is published by controller)
# 2. different led for each sports/eco/off mode

# note: code is currently incomplete, waiting for led callback data implementation
# and need to maybe rewrite publishing algo

# There are a couple way to create a flashing LED(s) and sound:
# Use a counter with the main loop (loop that constantly publishes the Twist command to kobuki_node)
# Use another node or nodes

# Should flash all lights and sound
def flashBackwards():
    rospy.init_node('backwards_sender', anonymous=True)
    led = LED()
    sound = Sound()
    pub_led1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=10)
    pub_led2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=10)
    pub_sound = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=10)
    for x in range(3,-1,-1):
        led.value = x
        pub_led1.publish(led)
        pub_led2.publish(led)
        rospy.sleep(1)
    for x in range (0,7):
        sound.value = x
        pub_sound.publish(sound)
        rospy.sleep(1.5)

# need a different LED for each mode
# data is likely an int
def changeLEDCallback(data):
    pub_led1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=10)
    pub_led2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=10)
    led = LED()
    if data == "eco":
        led.value = 1 # green
    elif data == "sport":
        led.value = 3 # red
    elif data == "off":
        led.value = 0 # black
    pub_led1.publish(led)
    pub_led2.publish(led)

def indicateSmoother():
    sub_smoothing = rospy.Subscriber("smoothing_setting", int, changeLEDCallback)