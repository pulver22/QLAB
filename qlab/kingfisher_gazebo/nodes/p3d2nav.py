#!/usr/bin/env python
'''
Node to emulate integrated imu/gps sensor (e.g., microstrain) using P3D plugin.

The P3D plugin provides an Odometry ground truth (with noise).  The position is provided relative to the the Gazebo origin.  We define this origin in lat/lon and then transform the position to a latitude/longitude message.

Uses the navsat_conversion module
'''

import copy
import sys
import numpy as np

import rospy
import tf
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

# ROS geonav_transform package
import geonav_transform.geonav_conversions as nc

def rotate_vector(q,v):
    '''
    Rotate vector v by quaternion q
    Assumes that q is given as a list in order x, y, z, w
    '''
    qv = list(v)
    qv.append(0.0)
    out = tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q, qv), 
        tf.transformations.quaternion_conjugate(q)
        )[:3]
    return out

class Node():
    def __init__(self,olat,olong):
        self.outmsg = Odometry()
        self.outpub = rospy.Publisher("nav_odom",Odometry,queue_size=10)
        self.sub = rospy.Subscriber("p3d_odom",Odometry,self.p3dcallback)
        rospy.loginfo("Subscribing to %s"%self.sub.name)
        self.rxodom = False
        self.originLat = olat
        self.originLong = olong
        utmy, utmx, utmzone = nc.LLtoUTM(self.originLat,self.originLong)
        self.originX = utmx
        self.originY = utmy
        self.originZone = utmzone
        rospy.loginfo("Origin at Lat/Lon %.6f/%.6f which is UTM X: %.4f Y: %.4f Zone: %s"%(self.originLat, self.originLong, self.originX, self.originY, self.originZone))
        self.seq = 0

    def p3dcallback(self,data):
        self.rxodom = True
        # copy the message 
        self.outmsg = copy.deepcopy(data)
        # Find lat/long based on origin
        utmx = self.originX + data.pose.pose.position.x
        utmy = self.originY + data.pose.pose.position.y
        lat, lon = nc.UTMtoLL(utmy,utmx,self.originZone)
        #print("Lat %.10f, Long %.10f"%(lat,lon))
        #print("X %.10f, Y %.10f"%(utmx,utmy))
        self.outmsg.pose.pose.position.x = lon
        self.outmsg.pose.pose.position.y = lat
        
        # Gazebo provides the twist.linear velocities relative to the 
        # Gazebo frame, we want them relative to the body-frame
        origv = [data.twist.twist.linear.x,
                 data.twist.twist.linear.y,
                 data.twist.twist.linear.z]
        q = [data.pose.pose.orientation.x,
             data.pose.pose.orientation.y,
             data.pose.pose.orientation.z,
             data.pose.pose.orientation.w]
        q = tf.transformations.quaternion_inverse(q)
        newv = rotate_vector(q,origv)
        self.outmsg.twist.twist.linear.x = newv[0]
        self.outmsg.twist.twist.linear.y = newv[1]
        self.outmsg.twist.twist.linear.z = newv[2]
        
    def publishodom(self):
        if self.rxodom:
            self.outmsg.header.stamp = rospy.get_rostime()
            self.outmsg.header.seq = self.seq
            self.seq += 1
            self.outpub.publish(self.outmsg)

        
if __name__ == '__main__':
    
    rospy.init_node('p3d2nav', anonymous=True)
    
    # Parameters
    olat = rospy.get_param('~gazebo_origin_lat',0.0)
    olong = rospy.get_param('~gazebo_origin_long',0.0)
    if (olat == 0.0):
        rospy.logerror('Must specify ~gazebo_origin_lat !')
        sys.exit(1)
    if (olong == 0.0):
        rospy.logerror('Must specify ~gazebo_origin_long !')
        sys.exit(1)

    # Initiate node object
    node=Node(olat,olong)
    r = rospy.Rate(10) # 10hz

    try:
        while not rospy.is_shutdown():
            node.publishodom()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
