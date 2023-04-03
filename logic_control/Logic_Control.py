#! /usr/bin/env python3
# -*- coding: utf-8 -*
import os
import sys
import tty, termios
import roslib
import rospy
import tf
import threading

from std_msgs.msg import String
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from serial_robot.msg import aim
from serial_robot.msg import spinning_control
from serial_referee.msg import message_game_command
from serial_referee.msg import message_game_HP
from serial_referee.msg import message_game_status
from serial_referee.msg import message_game_hurt

strategy_state = 0
'''
0:待机、 1：攻击、 2：防御
攻击：
    允许打击前哨站、基地、哨兵和干扰能量机关（将目标点设在指定区域）
    不攻击以上目标时前往目标点，如果发现目标则追杀直至目标丢失（逃跑、死亡）
防御：
    前往目标点原地驻守（包括开小陀螺+小范围躲避移动）
    路上发现目标会攻击但不改变运动策略
    到达后旋转扫描
'''
VIO_switch = 0
LIO_switch = 1

Pitch_Scan = 0
Yaw_Scan = 0
'''
订阅自瞄
当相机自瞄没有数据时：
    当在目标点附近时：
        开启Yaw轴扫描
        开启Pitch轴扫描
'''
current_chassis_angle = 0#底盘相对云台角度
target_spinning_speed = 0#期望小陀螺速度

self_aim_state = 0
location_target = 0
spinning_velocity = 0

def rps_spin_job():
    rospy.spin()
def game_staus_callback(ext_status):
    print(ext_status.game_type)
def game_HP_callback(ext_HP):
    print(ext_HP.game_type)
def game_hurt_callback(ext_hurt):
    print(ext_hurt.game_type)
def game_command_callback(ext_command):
    print(ext_command.game_type)
def chassis_angle_caalback(ext_chassis_angle):
    global target_spinning_speed
    target_spinning_speed= ext_chassis_angle.data

if __name__ == '__main__':
    rospy.init_node('logic_control_node')
    rate = rospy.Rate(0.1)

    location_listener = tf.TransformListener()
    loccation_target_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    chassis_angle_sub = rospy.Subscriber('/robot/chassis_angle', Int16, chassis_angle_caalback)

    threading.Thread(target = rospy.spin).start()
    while (rospy.is_shutdown() == 0):
        try:
            trans, rot = location_listener.lookupTransform('map', 'plane_base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print('position',trans)
        print(1)
        rate.sleep()
    print("Logic Control is shutting down!")
