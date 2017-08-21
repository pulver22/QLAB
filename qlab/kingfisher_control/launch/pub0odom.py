#!/usr/bin/env python
'''
hack to publish some 0 odometry
'''

# Python
import sys
from math import pi

# ROS
import rospy
from nav_msgs.msg import Odometry

def talker():
    pub = rospy.Publisher('/odometry/filtered', Odometry, queue_size=10)
    rospy.init_node('odom0', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        omsg = Odometry()
        omsg.header.seq += 1
        omsg.header.stamp = rospy.get_rostime()
        omsg.header.frame_id = 'odom'
        rospy.loginfo("publishing odom")
        pub.publish(omsg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
