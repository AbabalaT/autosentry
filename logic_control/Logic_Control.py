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
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from serial_robot.msg import aim
from serial_robot.msg import spinning_control
from serial_referee.msg import message_game_command
from serial_referee.msg import message_game_HP
from serial_referee.msg import message_game_status
from serial_referee.msg import message_game_hurt

from tf.transformations import quaternion_from_euler, euler_from_quaternion

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

pitch_scan = 0.0
pitch_state = 0 #0:上升 1: 下降
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
    print("Game Type:",ext_status.game_type)
def game_HP_callback(ext_HP):
    print("Blue7HP:",ext_HP.blue_7_robot_HP)
def game_hurt_callback(ext_hurt):
    print(ext_hurt.game_type)
def game_command_callback(ext_command):
    print(ext_command.game_type)
def chassis_angle_caalback(ext_chassis_angle):
    global target_spinning_speed
    target_spinning_speed= ext_chassis_angle.data


if __name__ == '__main__':
    rospy.init_node('logic_control_node')
    rate = rospy.Rate(10)

    location_listener = tf.TransformListener()
    loccation_target_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    recommend_pitch_publisher = rospy.Publisher('/robot/logic_recommend_angle', Float32, queue_size=1)
    spin_speed_pulisher = rospy.Publisher('/robot/spnning_speed', spinning_control, queue_size=1)
    chassis_angle_sub = rospy.Subscriber('/robot/chassis_angle', Int16, chassis_angle_caalback)

    ros_spin_thread = threading.Thread(target = rospy.spin, daemon=True)
    ros_spin_thread.start()

    while (rospy.is_shutdown() == 0):
        try:
            trans, rot = location_listener.lookupTransform('map', 'plane_base_link', rospy.Time(0))
            # print('position',trans,rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
            # print("logic no TF get !")

        spin_speed_msg = spinning_control()
        spin_speed_msg.spinning_speed = 9000
        spin_speed_pulisher.publish(spin_speed_msg)

        if pitch_state == 0:
            pitch_scan = pitch_scan + 6
            recommend_pitch_publisher.publish(pitch_scan)
            if pitch_scan > 10.0:
                pitch_state = 1
        if pitch_state == 1:
            pitch_scan = pitch_scan - 6
            recommend_pitch_publisher.publish(pitch_scan)
            if pitch_scan < -20.0:
                pitch_state = 0
        rate.sleep()
    print("Logic Control is shutting down!")
    exit()
