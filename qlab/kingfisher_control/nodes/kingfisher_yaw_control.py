#!/usr/bin/env python
'''
Example of using the pypid module for heading control of a USV.

'''
# Python
import sys
from math import pi

# ROS
import rospy
import tf
from dynamic_reconfigure.server import Server
from kingfisher_control.cfg import YawDynamicConfig

from kingfisher_control.msg import PidDiagnose

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from kingfisher_msgs.msg import Drive
from sensor_msgs.msg import Imu

# BSB
import pypid


class Node():
    def __init__(self):
        Kp=0.0
        Ki=0.0
        Kd=0.1
        self.pid = pypid.Pid(Kp,Ki,Kd)
        self.pid.set_setpoint(-pi/2)
        self.pid.set_inputisangle(True,pi)
        self.pid.set_derivfeedback(True)  # D term in feedback look
        fc = 50;  # cutoff freq in hz
        wc = fc*(2.0*pi)  # cutoff freq. in rad/s
        self.pid.set_derivfilter(1,wc)
        self.drivemsg = None
        self.publisher = None
        self.lasttime = None
        # For diagnosing/tuning PID
        self.pubdebug = None
        
    def set_setpoint(self,msg):
        self.pid.set_setpoint(msg.data)

    def callback(self,msg):
        # Get quaternion from Imu message
        q = (msg.orientation.x,
             msg.orientation.y,
             msg.orientation.z,
             msg.orientation.w)
        euler = tf.transformations.euler_from_quaternion(q)
        yaw = euler[2]
        now = rospy.get_time()
        if self.lasttime is None:
            self.lasttime = now
            return
        dt = now-self.lasttime
        self.lasttime = now
        #print("dt: %.6f"%dt)
        out = self.pid.execute(dt,yaw)
        torque = out[0]
        self.drivemsg.left=-1*torque
        self.drivemsg.right=torque
        self.publisher.publish(self.drivemsg)

        if not (self.pubdebug is None):
            self.debugmsg.PID = out[0]
            self.debugmsg.P = out[1]
            self.debugmsg.I = out[2]
            self.debugmsg.D = out[3]
            self.debugmsg.Error = out[4]
            self.debugmsg.Setpoint = out[5]
            self.debugmsg.Derivative= out[6]
            self.debugmsg.Integral = out[7]
            self.pubdebug.publish(self.debugmsg)



    def dynamic_callback(self,config,level):
        #rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
        #  {str_param}, {bool_param}, {size}""".format(**config))
        rospy.loginfo("Reconfigure request...")
        #print config.keys()
        #print config['Kp']
        self.pid.Kp = config['Kp']
        self.pid.Ki = config['Ki']
        self.pid.Kd = config['Kd']
        return config
        

if __name__ == '__main__':
    
    rospy.init_node('kingfisher_yaw_pid', anonymous=True)
    
    # ROS Parameters
    in_topic = rospy.get_param('~input_topic','imu')
    out_topic = rospy.get_param('~output_topic','cmd_drive')
    Kp = rospy.get_param('~Kp',1.0)
    Kd = rospy.get_param('~Kd',0.0)
    Ki = rospy.get_param('~Ki',0.0)
    
    # Initiate node object - creates PID object
    node=Node()
    
    # Set initial gains from parameters
    node.pid.Kp = Kp
    node.pid.Kd = Kd
    node.pid.Ki = Ki

    # Setup outbound message
    node.drivemsg = Drive()

    #in_topic = "kingfisher_rpy"
    #in_type = Vector3

    in_type = Imu
    

    # Setup publisher
    rospy.loginfo("Subscribing to %s"%
                  (in_topic))
    rospy.loginfo("Publishing to %s"%
                  (out_topic))
    node.publisher = rospy.Publisher(out_topic,Drive,queue_size=10)
    node.pubdebug = rospy.Publisher("pid_debug",PidDiagnose,queue_size=10)
    node.debugmsg = PidDiagnose()
    # Setup subscribers
    rospy.Subscriber(in_topic,in_type,node.callback)
    rospy.Subscriber("set_setpoint",Float64,node.set_setpoint)
    
    # Dynamic configure
    srv = Server(YawDynamicConfig, node.dynamic_callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
