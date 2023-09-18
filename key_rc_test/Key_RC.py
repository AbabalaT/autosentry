#! /usr/bin/env python3
# -*- coding: utf-8 -*  

import os
import sys
import time
import tty, termios
import roslib
import rospy
from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def tele_key():
    thread_stop = False
    cmd = Twist()
    # roslib.load_manifest('smartcar_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist)
    rospy.init_node('key_rc_test_node')
    turn = 0
    sidewalk = 0
    speed = 0
    while (thread_stop == False):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if ch == 'w':
            speed = speed + 0.5
        elif ch == 's':
            speed = speed - 0.5
        elif ch == 'a':
            turn = turn + 0.785
        elif ch == 'd':
            turn = turn - 0.785
        elif ch == 'q':
            sidewalk = sidewalk + 0.5
        elif ch == 'e':
            sidewalk = sidewalk - 0.5
        elif ch == 'p':
            turn = 0
            speed = 0
            thread_stop = True
        else:
            turn = 0
            sidewalk = 0
            speed = 0

        cmd.linear.x = speed
        cmd.linear.y = sidewalk
        cmd.angular.z = turn
        print('rc speed:', cmd.linear.x, cmd.linear.y, cmd.angular.z)
        pub.publish(cmd)
        time.sleep(0.05)
        pub.publish(cmd)
        time.sleep(0.05)
        pub.publish(cmd)
        time.sleep(0.05)
        pub.publish(cmd)
        time.sleep(0.05)

if __name__ == '__main__':
    tele_key()
