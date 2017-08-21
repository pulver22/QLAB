#!/usr/bin/env python
'''
Using instances of the pypid Pid class to control yaw and velocity
'''
# Python
import sys
from math import pi

# ROS
import rospy
import tf
from dynamic_reconfigure.server import Server
from kingfisher_control.cfg import YawDynamicConfig
from kingfisher_control.cfg import TwistDynamicConfig

from kingfisher_control.msg import PidDiagnose

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from kingfisher_msgs.msg import Drive
from kingfisher_msgs.msg import Course
from sensor_msgs.msg import Imu
#from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from std_srvs.srv import Empty, EmptyResponse

# BSB
import pypid


class Node():
    def __init__(self,engaged=False,yaw_cntrl=True,vel_cntrl=True):
        # Setup Yaw Pid
        self.engaged = engaged
        self.yaw_cntrl = yaw_cntrl
        self.vel_cntrl = vel_cntrl
        self.ypid = pypid.Pid(0.0, 0.0, 0.0)
        self.ypid.set_setpoint(0.0)
        #self.pid.set_inputisangle(True,pi)
        self.ypid.set_derivfeedback(True)  # D term in feedback look
        fc = 20;  # cutoff freq in hz
        wc = fc*(2.0*pi)  # cutoff freq. in rad/s
        self.ypid.set_derivfilter(1,wc)
        self.ypid.set_maxIout(1.0)
        # Setup Velocity Pid
        self.vpid = pypid.Pid(0.0, 0.0, 0.0)
        self.vpid.set_setpoint(0.0)
        self.vpid.set_maxIout(1.0)
        #self.pid.set_inputisangle(True,pi)
        self.vpid.set_derivfeedback(True)  # D term in feedback look
        fc = 20;  # cutoff freq in hz
        wc = fc*(2.0*pi)  # cutoff freq. in rad/s
        self.vpid.set_derivfilter(1,wc)
        
        # Initialize some bits as none - for now
        self.drivemsg = None
        self.publisher = None
        self.lasttime = None
        # For diagnosing/tuning PID
        self.vpubdebug = None
        self.ypubdebug = None

        # Type of yaw control
        self.yaw_type = 'yaw'
        
    '''
    def set_engaged_callback(self,req):
        result = SetBoolResponse()
        rospy.loginfo('Setting engaged to :')
        rospy.loginfo(req.data)
        self.engaged = req.data
        # response
        #req.success = True
        #req.message = "done"
        result.success = True
        result.message = 'done'
        return result
    '''
    def toggle_engaged_callback(self,req):
        self.engaged = not self.engaged
        rospy.loginfo('Toggling engaged status - new status is...')
        rospy.loginfo(self.engaged)
        
        # If engaging, zero the I terms
        if self.engaged:
            self.ypid.I = 0.0
            self.vpid.I = 0.0
        return EmptyResponse()

    def twist_callback(self,msg):
        self.ypid.set_setpoint(msg.angular.z)
        self.vpid.set_setpoint(msg.linear.x)

    def course_callback(self,msg):
        self.ypid.set_setpoint(msg.yaw)
        self.vpid.set_setpoint(msg.speed)

    def odom_callback(self,msg):
        # Calculate time step
        now = rospy.get_time()
        if self.lasttime is None:
            self.lasttime = now
            return
        dt = now-self.lasttime
        self.lasttime = now
        # Yaw Control
        if self.yaw_cntrl:
            if self.yaw_type=='yaw_rate':
                yaw_fdbk = msg.twist.twist.angular.z # measured rate (process var.)
            elif self.yaw_type=='yaw':
                euler = tf.transformations.euler_from_quaternion(
                    (msg.pose.pose.orientation.x,
                     msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z,
                     msg.pose.pose.orientation.w))
                yaw_fdbk = euler[2]  # yaw
            yout = self.ypid.execute(dt,yaw_fdbk)
            torque = yout[0]
        else:
            torque = 0.0


        # Velocity control
        if self.vel_cntrl:
            dx = msg.twist.twist.linear.x
            vout = self.vpid.execute(dt,dx)
            thrust = vout[0]
        else:
            thrust = 0.0

        # I believe drive messages are scaled to -1.0 to 1.0
        # Scale so that no one output saturates
        '''
        mag = abs(torque)+abs(thrust)
        if mag > 1.0:

            torque = torque/mag
            thrust = thrust/mag
        '''

        #rospy.loginfo('Torque: %.3f, Thrust: %.3f'%(torque,thrust))
        self.drivemsg.left=-1*torque + thrust
        self.drivemsg.right=torque + thrust
        
        # Only publish if engaged
        if (self.engaged):
            self.publisher.publish(self.drivemsg)

        if (not (self.ypubdebug is None)) and (self.yaw_cntrl):
            self.ydebugmsg.PID = yout[0]
            self.ydebugmsg.P = yout[1]
            self.ydebugmsg.I = yout[2]
            self.ydebugmsg.D = yout[3]
            self.ydebugmsg.Error = yout[4]
            self.ydebugmsg.Setpoint = yout[5]
            self.ydebugmsg.Derivative= yout[6]
            self.ydebugmsg.Integral = yout[7]
            self.ypubdebug.publish(self.ydebugmsg)
        if (not (self.vpubdebug is None)) and (self.vel_cntrl):
            self.vdebugmsg.PID = vout[0]
            self.vdebugmsg.P = vout[1]
            self.vdebugmsg.I = vout[2]
            self.vdebugmsg.D = vout[3]
            self.vdebugmsg.Error = vout[4]
            self.vdebugmsg.Setpoint = vout[5]
            self.vdebugmsg.Derivative= vout[6]
            self.vdebugmsg.Integral = vout[7]
            self.vpubdebug.publish(self.vdebugmsg)


    def dynamic_callback(self,config,level):
        #rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
        #  {str_param}, {bool_param}, {size}""".format(**config))
        rospy.loginfo("Reconfigure request...")
        #print config.keys()
        #print config['yawKp']
        self.ypid.Kp = config['yawKp']
        #self.ypid.Ki = config['yawKi']
        # Use method to zero the integrator
        Ki = config['yawKi']
        tol = 1e-6
        if abs(abs(Ki)-abs(self.ypid.Ki)) > tol:
            rospy.loginfo("Setting yaw Ki to %.3f"%Ki)
            self.ypid.set_Ki(Ki)
        self.ypid.Kd = config['yawKd']

        self.vpid.Kp = config['velKp']
        #self.vpid.Ki = config['velKi']
        Ki = config['velKi']
        if abs(abs(Ki)-abs(self.vpid.Ki)) > tol:
            rospy.loginfo("Setting vel Ki to %.3f"%Ki)
            self.vpid.set_Ki(Ki)
        self.vpid.Kd = config['velKd']
        return config
        

if __name__ == '__main__':
    
    rospy.init_node('kingfisher_yaw_pid', anonymous=True)
    
    # ROS Parameters
    yawKp = rospy.get_param('~yawKp',1.0)
    yawKd = rospy.get_param('~yawKd',0.0)
    yawKi = rospy.get_param('~yawKi',0.0)

    velKp = rospy.get_param('~velKp',1.0)
    velKd = rospy.get_param('~velKd',0.0)
    velKi = rospy.get_param('~velKi',0.0)

    engaged = rospy.get_param('~start_engaged',False)

    # Allow for combinations of yaw and vel control
    yaw_cntrl = rospy.get_param('~yaw_cntrl',True)
    vel_cntrl = rospy.get_param('~vel_cntrl',True)

    yaw_type = rospy.get_param('~yaw_type','yaw')  # 'yaw' or 'yaw_rate'
    rospy.loginfo('Setting control yaw_type to <%s>'%yaw_type)
    # validate
    if (not (yaw_type=='yaw' or yaw_type=='yaw_rate')):
        
        rospy.logerror('Must specify a known type of yaw control - '
                       '<%s> is unknown'%yaw_type)
        sys.exit()
        
        
    
    # Initiate node object - creates PID object
    node=Node(engaged=engaged, yaw_cntrl=yaw_cntrl, vel_cntrl=vel_cntrl)
    
    # Set initial gains from parameters
    node.ypid.Kp = yawKp
    node.ypid.Kd = yawKd
    node.ypid.Ki = yawKi
    node.vpid.Kp = velKp
    node.vpid.Kd = velKd
    node.vpid.Ki = velKi

    # Set to either 'yaw' or 'yaw_rate'
    node.yaw_type = yaw_type
    if node.yaw_type == 'yaw':
        node.ypid.set_inputisangle(True)

    # Setup outbound message
    node.drivemsg = Drive()

    # Setup publisher
    node.publisher = rospy.Publisher('cmd_drive',Drive,queue_size=10)
    rospy.loginfo("Publishing to %s"%
                  (node.publisher.name))
    node.ypubdebug = rospy.Publisher("yaw_pid_debug",PidDiagnose,queue_size=10)
    node.vpubdebug = rospy.Publisher("vel_pid_debug",PidDiagnose,queue_size=10)
    node.ydebugmsg = PidDiagnose()
    node.vdebugmsg = PidDiagnose()

    # Setup service
    #s = rospy.Service('set_engaged',SetBool,node.set_engaged_callback)
    s = rospy.Service('toggle_engaged',Empty,node.toggle_engaged_callback)

    # Setup subscribers
    s1 = rospy.Subscriber('odometry/nav',Odometry,node.odom_callback)
    rospy.loginfo("Subscribing to %s"%
                  (s1.name))
    if (yaw_type=='yaw_rate'):
        s2 = rospy.Subscriber("cmd_vel",Twist,node.twist_callback)
    elif (yaw_type=='yaw'):
        s2 = rospy.Subscriber("cmd_course",Course,node.course_callback)
    else:
        rospy.logerror("Don't know what to listen to for yaw_type <%s>"
                       %yaw_type)
        sys.exit()
    rospy.loginfo("Subscribing to setpoint commands at %s"%
                  (s2.name))

    # Dynamic configure
    srv = Server(TwistDynamicConfig, node.dynamic_callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
