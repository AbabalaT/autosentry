#! /usr/bin/env python3
# -*- coding: utf-8 -*  

import os
import sys
import tty, termios
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def teleop_key():

    thread_stop = False
    cmd = Twist()
    #roslib.load_manifest('smartcar_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist)
    rospy.init_node('key_rc_test_node')

    rate = rospy.Rate(rospy.get_param('~hz', 1))
    walk_vel_ = 0.6
    yaw_rate_ = 1

    max_tv = walk_vel_
    max_rv = yaw_rate_
    turn = 0
    sidewalk = 0
    speed = 0

    while (thread_stop == False):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try :
            tty.setraw( fd )
            ch = sys.stdin.read(1)
        finally :
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if ch == 'w':
            max_tv = walk_vel_
            speed = speed + 0.5
        elif ch == 's':
            max_tv = walk_vel_
            speed = speed - 0.5
        elif ch == 'a':
            max_rv = yaw_rate_
            turn = turn + 0.5
        elif ch == 'd':
            max_rv = yaw_rate_
            turn = turn - 0.5
        elif ch == 'q':
            max_rv = yaw_rate_
            sidewalk = sidewalk + 0.5
        elif ch == 'e':
            max_rv = yaw_rate_
            sidewalk = sidewalk - 0.5
        elif ch == 'p':
            turn = 0
            speed = 0
            thread_stop = True
        else:
            turn = 0
            sidewalk = 0
            speed = 0
            max_tv = walk_vel_
            max_rv = yaw_rate_

        cmd.linear.x = speed * max_tv
        cmd.linear.y = sidewalk * max_tv
        cmd.angular.z = turn * max_rv
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    teleop_key() 
