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
    rospy.init_node('algorithm', anonymous=True)
    sub = rospy.Subscriber('/rss', int, rss_callback)
    rate = rospy.Rate(0.5) # 10hz

    while not rospy.is_shutdown():

        # TODO: Move algorithm to ROS version
        
        rate.sleep()

if __name__ == '__main__':
    try:
        algorithm()
    except rospy.ROSInterruptException:
        pass