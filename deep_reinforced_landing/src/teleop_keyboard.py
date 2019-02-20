#!/usr/bin/env python
# import roslib; roslib.load_manifest('teleop_ardrone')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from deep_reinforced_landing.srv import SendCommand

import sys
import select
import termios
import tty
import time

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
up/down:       move forward/backward
left/right:    move left/right
w/s:           increase/decrease altitude
a/d:           turn left/right
t/l:           takeoff/land
r:             reset (toggle emergency state)
anything else: stop
y/n : increase/decrease max speeds by 10%
u/m : increase/decrease only linear speed by 10%
i/, : increase/decrease only angular speed by 10%
anything else : stop
CTRL-C to quit
"""
# each button is associated with a 6-dim tuple: (x,th_x,y,th_y,z,th_z)

#(linear,angular) velocity on the three axis
moveBindingsAxis = {
    67: (0, 0, -1, 0, 0, 0, 0),  # left
    68: (0, 0, 1, 0, 0, 0, 0),  # right
    65: (1, 0, 0, 0, 0, 0, 0),  # forward
    66: (-1, 0, 0, 0, 0, 0),  # back
    'w': (0, 0, 0, 0, 1, 0),  # increase altitude
    's': (0, 0, 0, 0, -1, 0),  # decrease altitude
    'a': (0, 0, 0, 0, 0, 1),  # rotate left
    'd': (0, 0, 0, 0, 0, -1),  # rotate right
}

# increase/decrease velocity on X axis
speedBindingsAxis = {
    'y': (1.1, 1.1),  # increase linear and angular velocity
    'n': (.9, .9),  # decrease linear and angular velocity
    'u': (1.1, 1),  # increase only linear vel
    'm': (.9, 1),  # decrease only linear vel
    'i': (1, 1.1),  # increase only rot vel
    ',': (1, .9),  # decrease only rot vel
}

specialManoeuvre = {
    't': (0, 0),  # takeoff
    'l': (0, 0),  # land
    'r': (0, 0),  # reset
}


def send_action(action):
    """
    Send an action to the UAV.
    """
    rospy.wait_for_service('/drl/send_command')
    try:
        get_random_action_proxy = rospy.ServiceProxy(
            '/drl/send_command', SendCommand)
        # Call the service
        get_random_action_proxy(action)
    except rospy.ServiceException, ex:
        print "Service call get_random_action failed: %s" % ex


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


speed = .1
noop_time = 1.0 / speed  # sleep amount of seconds as 1 meter divided by speed
turn = 1


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    # NOTE: Simulator
    # pub_twist = rospy.Publisher('/quadrotor/cmd_vel', Twist, queue_size=1)
    # pub_empty_takeoff = rospy.Publisher(
    #     '/quadrotor/ardrone/takeoff', Empty, queue_size=1)
    # pub_empty_landing = rospy.Publisher(
    #     '/quadrotor/ardrone/land', Empty, queue_size=1)

    # NOTE: Real drone
    pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_empty_takeoff = rospy.Publisher(
        '/ardrone/takeoff', Empty, queue_size=1)
    pub_empty_landing = rospy.Publisher(
        '/ardrone/land', Empty, queue_size=1)

    rospy.init_node('teleop_ardrone_keyboard')

    command = "stop"
    try:
        print msg
        print vels(speed, turn)
        while(1):
            x = y = z = 0
            th_x = th_y = th_z = 0
            status = 0
            key = getKey()
            if ((ord(key) in moveBindingsAxis.keys()) or (key in moveBindingsAxis.keys())):
                # x is linear speed, th is the angular one

                if (ord(key) in moveBindingsAxis.keys()):
                    key = ord(key)

                x = moveBindingsAxis[key][0]
                th_x = moveBindingsAxis[key][1]
                y = moveBindingsAxis[key][2]
                th_y = moveBindingsAxis[key][3]
                z = moveBindingsAxis[key][4]
                th_z = moveBindingsAxis[key][5]

                # 67: (0, 0, -1, 0, 0, 0, 0),  # left
                # 68: (0, 0, 1, 0, 0, 0, 0),  # right
                # 65: (1, 0, 0, 0, 0, 0, 0),  # forward
                # 66: (-1, 0, 0, 0, 0, 0),  # back
                # 'w': (0, 0, 0, 0, 1, 0),  # increase altitude
                # 's': (0, 0, 0, 0, -1, 0),  # decrease altitude
                print key
                if (key == 's'):
                    command = "descend"
                elif (key == 'w'):
                    command = "ascend"
                elif (key == 67):
                    command = "right"
                elif (key == 68):
                    command = "left"
                elif (key == 65):
                    command = "forward"
                elif (key == 66):
                    command = "backward"
                else:
                    command = "stop"

                try:
                    while True:
                        send_action(command)
                        rospy.sleep(2.0)
                        send_action("stop")
                        continue
                except KeyboardInterrupt:
                    pass

            elif key in speedBindingsAxis.keys():
                # increase or decrease linear or angular speed
                speed = speed * speedBindingsAxis[key][0]
                turn = turn * speedBindingsAxis[key][1]

                print vels(speed, turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15

            elif key in specialManoeuvre.keys():

                if (key == 't'):
                    # publish mex to take off
                    pub_empty_takeoff.publish(Empty())
                    continue
                elif (key == 'l'):
                    # publish mex to landing
                    pub_empty_landing.publish(Empty())
                    # send_action("land")
                    # rospy.sleep(2.0)
                    # send_action("stop")
                    continue
                else:
                    # publish mex to reset velocity and hoovering
                    twist = Twist()
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.linear.z = 0
                    twist.angular.x = 0
                    twist.angular.y = 0
                    twist.angular.z = 0
                    # pub_twist.publish(twist)

                    try:
                        while True:
                            send_action("stop")
                            rospy.sleep(2.0)
                    except KeyboardInterrupt:
                        pass
            else:
                x = 0
                th = 0
                if (key == '\x03'):
                    break

            # twist = Twist()
            # twist.linear.x = x * speed
            # twist.linear.y = y * speed
            # twist.linear.z = z * speed
            # twist.angular.x = th_x * turn
            # twist.angular.y = th_y * turn
            # twist.angular.z = th_z * turn
            # pub_twist.publish(twist)
            # rospy.sleep(2.0)
            # send_action("stop")

    except Exception, e:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        # pub_twist.publish(twist)
        send_action("stop")

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
