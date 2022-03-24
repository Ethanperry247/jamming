#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32

# RSS will be updated as much as controlled by the usrp publisher
rss = 0

# Publishes a movement to the linear movement topic.
def move_linear(distance):
    pub = rospy.Publisher("/create/linear", Int32, queue_size=2)
    msg = Int32()
    msg.data = distance
    pub.publish(msg)

# Publishes a movement to the angular movement topic.
def move_angular(angle):
    pub = rospy.Publisher("/create/angle", Int32, queue_size=2)
    msg = Int32()
    msg.data = angle
    pub.publish(msg)

def rss_callback(signal):
    global rss
    rss = signal.data
    
def algorithm():
    global rss
    rospy.init_node('simple', anonymous=True)
    sub = rospy.Subscriber('/rss', int, rss_callback)
    rate = rospy.Rate(0.5) # 10hz

    # Simply move every two seconds and sample the RSSI space in a grid-like manner.
    while not rospy.is_shutdown():
        if (forward == 20):
            move_angular(90)
            forward = 0
        elif (forward == 1):
            move_angular(90)
        else:
            move_linear(1)
            forward += 1
            
        rate.sleep()

if __name__ == '__main__':
    try:
        algorithm()
    except rospy.ROSInterruptException:
        pass