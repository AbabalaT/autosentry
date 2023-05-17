#! /usr/bin/env python3
# -*- coding: utf-8 -*
import os
import sys
import tty, termios
import roslib
import threading

import random
import rospy
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion

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

aim_distance = 0
lock_yaw_cnt = 0
remain_time = 300
require_add_HP = 0
robot_HP = 600
pre_base_HP = 1500
base_HP = 1500
base_HP_list = [1000, 1000]
self_color = 'blue'
save_base_time_cnt = 0  # 单位0.1秒
wait_attack_cnt = 1200
attack_cnt = 0  # 攻击模式计时器
strategy_state = 0
follow_enemy_cnt = 0
seq_1 = 0
seq_2 = 0

hurt_armor = 0
chassis_angle = 0.0

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
pitch_state = 0  # 0:上升 1: 下降

target_yaw = 0
game_status = 0

'''
订阅自瞄
当相机自瞄没有数据时：
    当在目标点附近时：
        开启Yaw轴扫描
        开启Pitch轴扫描
'''

target_spinning_speed = 16000  # 期望小陀螺速度

self_aim_state = 0
location_target = 0
spinning_velocity = 0


def autoaim_callback(ext_aim):
    global lock_yaw_cnt, aim_distance
    if ext_aim.target_number != 0:
        lock_yaw_cnt = 2
        # aim_distance = ext_aim.


def game_staus_callback(ext_status):
    global game_status
    global remain_time
    game_status = ext_status.game_progress
    remain_time = ext_status.stage_remain_time
    print("                                               Game Status:", game_status)


def game_HP_callback(ext_HP):
    # pass
    global robot_HP
    global base_HP
    if self_color == 'red':
        robot_HP = ext_HP.red_7_robot_HP
        base_HP = ext_HP.red_base_HP
    else:
        robot_HP = ext_HP.blue_7_robot_HP
        base_HP = ext_HP.blue_base_HP
    print("                               HP:", robot_HP)


def game_hurt_callback(ext_hurt):
    global target_spinning_speed
    if ext_hurt.hurt_type == 0:
        target_spinning_speed = 26000


def game_command_callback(ext_command):
    pass


def chassis_angle_callback(ext_chassis_angle):
    global chassis_angle
    # print("chassis angle:                          ",ext_chassis_angle)
    chassis_angle = ext_chassis_angle

def timer1_callback(event):
    global pre_base_HP, wait_attack_cnt, lock_yaw_cnt, target_yaw
    if robot_HP < 250:
        require_add_HP = 1
    if robot_HP > 550:
        require_add_HP = 0
    if base_HP != pre_base_HP:
        pre_base_HP = base_HP
        save_base_time_cnt = 400
    if save_base_time_cnt > 0:
        save_base_time_cnt = save_base_time_cnt - 1
    wait_attack_cnt = wait_attack_cnt - 1
    if wait_attack_cnt <= 0:
        wait_attack_cnt = 1200
        attack_cnt = 600
    if attack_cnt > 0:
        attack_cnt = attack_cnt - 1
    if lock_yaw_cnt > 0:
        lock_yaw_cnt = lock_yaw_cnt - 1

    target_yaw = target_yaw + 0.475
    target_pose = PoseStamped()
    target_pose.header.frame_id = "map"

    if (require_add_HP == 1):
        target_pose.pose.position.x = 0
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = 0
    elif (game_status == 4) and (save_base_time_cnt > 0):
        target_pose.pose.position.x = 0
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = 0
    elif (game_status == 4) and (base_HP > 800):
        if remain_time > 275:
            target_pose.pose.position.x = 0
            target_pose.pose.position.y = 0
        elif remain_time > 250:
            target_pose.pose.position.x = 0
            target_pose.pose.position.y = 0
        elif remain_time > 210:
            target_pose.pose.position.x = 0
            target_pose.pose.position.y = 0
        elif remain_time > 180:
            target_pose.pose.position.x = 0
            target_pose.pose.position.y = 0
        elif remain_time > 150:
            target_pose.pose.position.x = 0
            target_pose.pose.position.y = 0
        elif remain_time > 120:
            target_pose.pose.position.x = 0
            target_pose.pose.position.y = 0
        else:
            target_pose.pose.position.x = 0
            target_pose.pose.position.y = 0
        target_pose.pose.position.z = 0
    else:
        target_pose.pose.position.x = 0
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = 0

    target_quad = quaternion_from_euler(0, 0, target_yaw)
    target_pose.pose.orientation.x = target_quad[0]
    target_pose.pose.orientation.y = target_quad[1]
    target_pose.pose.orientation.z = target_quad[2]
    target_pose.pose.orientation.w = target_quad[3]
    if lock_yaw_cnt > 0:
        target_pose.pose.orientation = rot
        target_pose.pose.position = trans
    location_target_publisher.publish(target_pose)
    if target_yaw > 31.415:
        target_yaw = -31.415
        # print('OK!')


def timer2_callback(event):
    global target_spinning_speed
    spin_speed_msg = spinning_control()
    if target_spinning_speed > 16000:
        target_spinning_speed = target_spinning_speed - 2000
    spin_speed_msg.spinning_speed = target_spinning_speed + random.randint(0, 4000)
    # spin_speed_msg.spinning_speed = 0
    spin_speed_pulisher.publish(spin_speed_msg)


def timer3_callback(event):
    global pitch_state, pitch_scan
    try:
        global trans
        global rot
        trans, rot = location_listener.lookupTransform('/map', '/plane_base_link', rospy.Time(0))
        # print('position',trans,rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
        # print("logic no TF get !")

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


if __name__ == '__main__':
    rospy.init_node('logic_control_node')
    location_listener = tf.TransformListener()
    location_target_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    recommend_pitch_publisher = rospy.Publisher('/robot/logic_recommend_angle', Float32, queue_size=1)
    spin_speed_pulisher = rospy.Publisher('/robot/spnning_speed', spinning_control, queue_size=1)
    chassis_angle_sub = rospy.Subscriber('/robot/chassis_angle', Float32, chassis_angle_callback)
    referee_status_sub = rospy.Subscriber('/referee/status', message_game_status, game_staus_callback)
    referee_hurt_sub = rospy.Subscriber('/referee/hurt', message_game_hurt, game_hurt_callback)
    referee_HP_sub = rospy.Subscriber('/referee/HP', message_game_HP, game_HP_callback)
    aim_sub = rospy.Subscriber('/robot/auto_aim', aim, autoaim_callback)
    rospy.Timer(rospy.Duration(0.1), timer1_callback)
    rospy.Timer(rospy.Duration(0.1), timer2_callback)
    rospy.Timer(rospy.Duration(0.1), timer3_callback)
    rospy.spin()

    recommend_pitch_publisher.publish(0)

    spin_speed_msg = spinning_control()
    spin_speed_msg.spinning_speed = 0
    spin_speed_pulisher.publish(spin_speed_msg)

    print("Logic Control is shutting down!")
    exit()
