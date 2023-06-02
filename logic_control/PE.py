#! /usr/bin/env python3
# -*- coding: utf-8 -*
import os
import sys
import tty, termios
import roslib
import threading
import math

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
from serial_robot.msg import armor_select

from serial_referee.msg import message_game_command
from serial_referee.msg import message_game_HP
from serial_referee.msg import message_game_status
from serial_referee.msg import message_game_hurt

todo = 0

kill_enemy_engineer = False
invincible_detect = True
kill_sentry_first = True

aim_distance = 0

warning_cnt = 0

pitch_scan_low = -20.0
pitch_scan_up = 10.0

yaw_scan_low = 0.0
yaw_scan_up = 0.0
yaw_scan_state = 0  # 0：周扫描 1：扇正扫 2：扇反扫

target_publish_idle = 10

remain_time = 300
require_add_HP = 0
self_robot_HP = 600
self_outpost_HP = 1500

pre_base_HP = 1500

self_base_HP = 5000

enemy_robot_HP = 600
enemy_outpost_HP = 1500
enemy_base_HP = 5000

self_color = 'blue'

save_base_time_cnt = 0  # 单位0.1秒
wait_attack_cnt = 0
attack_cnt = 0  # 攻击模式计时器
hurt_lock_pos_cnt = 0
aim_lock_pos_cnt = 0
follow_enemy_cnt = 0

enemy_1_cnt = 0
enemy_2_cnt = 0
enemy_3_cnt = 0
enemy_4_cnt = 0
enemy_5_cnt = 0

gimbal_control_cnt = 0

strategy_state = 0  # 0： 防御模式 1： 攻击模式 2： 视觉拖曳

command_cnt = 0.0

hurt_armor = 0  # 受击装甲
chassis_angle = 0.0  # 底盘角度
armor0_angle = 0.0
hurt_angle = 0.0

commander_x = 0.0
commander_y = 0.0

strategy_target_x = 0.0
strategy_target_y = 0.0
strategy_target_yaw = 0.0

enemy_location_x = 11.1
enemy_location_y = 4.81

target_x = 0.0
target_y = 0.0
target_yaw = 0.0

pre_target_x = 0.0
pre_target_y = 0.0
pre_target_yaw = 0.0

current_x = 0.0
current_y = 0.0
current_yaw = 0.0

aim_switch = [0, 1, 1, 1, 1, 1, 1, 1, 1]

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

game_status = 0

'''
订阅自瞄
当相机自瞄没有数据时：
    当在目标点附近时：
        开启Yaw轴扫描
        开启Pitch轴扫描
'''

target_spinning_speed = 0  # 期望小陀螺速度
lowest_spinning_speed = 16000
force_spinning = False

self_aim_state = 0

trans = (0.0, 0.0, 0.0)
rot = (0.0, 0.0, 0.0, 0.0)


def armor_select_callback(event):
    global armor_select_publisher, aim_switch
    aim_select_msg = armor_select()
    if strategy_state == 0:
        aim_select_msg.aim_1 = 255
        if kill_enemy_engineer:
            aim_select_msg.aim_2 = 255
        else:
            aim_select_msg.aim_2 = 0
        aim_select_msg.aim_3 = 255
        aim_select_msg.aim_4 = 255
        aim_select_msg.aim_5 = 255
        aim_select_msg.aim_robot = 0
        aim_select_msg.aim_base = 0
        aim_select_msg.aim_outpost = 0
    elif strategy_state == 1:
        aim_select_msg.aim_1 = 0
        aim_select_msg.aim_2 = 0
        aim_select_msg.aim_3 = 0
        aim_select_msg.aim_4 = 0
        aim_select_msg.aim_5 = 0
        if enemy_outpost_HP > 0:
            aim_select_msg.aim_outpost = 1
            aim_select_msg.aim_robot = 0
            aim_select_msg.aim_base = 0
        elif enemy_robot_HP > 0 and enemy_base_HP == 5000:
            aim_select_msg.aim_outpost = 0
            aim_select_msg.aim_robot = 1
            aim_select_msg.aim_base = 0
        elif enemy_robot_HP > 0 and enemy_base_HP != 5000:
            if kill_sentry_first:
                aim_select_msg.aim_outpost = 0
                aim_select_msg.aim_robot = 1
                aim_select_msg.aim_base = 0
            else:
                aim_select_msg.aim_outpost = 0
                aim_select_msg.aim_robot = 0
                aim_select_msg.aim_base = 1
        else:
            aim_select_msg.aim_outpost = 0
            aim_select_msg.aim_robot = 0
            aim_select_msg.aim_base = 1
    elif strategy_state == 2:
        aim_select_msg.aim_1 = 0
        aim_select_msg.aim_2 = 0
        aim_select_msg.aim_3 = 0
        aim_select_msg.aim_4 = 0
        aim_select_msg.aim_5 = 0
        aim_select_msg.aim_robot = 0
        aim_select_msg.aim_base = 0
        aim_select_msg.aim_outpost = 0
    if invincible_detect:
        if enemy_1_cnt > 1:
            aim_select_msg.aim_1 = 0
        if enemy_2_cnt > 1:
            aim_select_msg.aim_2 = 0
        if enemy_3_cnt > 1:
            aim_select_msg.aim_3 = 0
        if enemy_4_cnt > 1:
            aim_select_msg.aim_4 = 0
        if enemy_5_cnt > 1:
            aim_select_msg.aim_5 = 0
    aim_switch[1] = aim_select_msg.aim_1
    aim_switch[2] = aim_select_msg.aim_2
    aim_switch[3] = aim_select_msg.aim_3
    aim_switch[4] = aim_select_msg.aim_4
    aim_switch[5] = aim_select_msg.aim_5
    aim_switch[6] = aim_select_msg.aim_robot
    aim_switch[7] = aim_select_msg.aim_outpost
    aim_switch[8] = aim_select_msg.aim_base

    armor_select_publisher.publish(aim_select_msg)


def strategy_callback(event):
    global warning_cnt, strategy_target_yaw, yaw_scan_state, command_cnt
    global strategy_target_x, strategy_target_y, strategy_state

    if warning_cnt > 0:
        warning_cnt = warning_cnt - 1
        vector_x = enemy_location_x - current_x
        vector_y = enemy_location_y - current_y
        vector_theta = math.atan(vector_y / vector_x)
        if vector_x < 0:
            if vector_y > 0:
                vector_theta = vector_theta + 3.1415926
            else:
                vector_theta = vector_theta - 3.1415926
        strategy_target_yaw = vector_theta
    if warning_cnt < 0:
        warning_cnt = 0
    if warning_cnt == 0:
        yaw_scan_state = 0

    if command_cnt > 0:
        command_cnt = command_cnt - 1
        strategy_target_x = commander_x
        strategy_target_y = commander_y
    elif game_status != 4:
        strategy_state = 0
        strategy_target_x = 0.0
        strategy_target_y = 0.0
    elif self_outpost_HP < 10:
        strategy_state = 0
        strategy_target_x = 0.0 + random.uniform(-0.5, 0.5)
        strategy_target_y = 0.0 + random.uniform(-0.5, 0.5)


def auto_aim_callback(ext_aim):
    global aim_lock_pos_cnt, aim_distance
    if aim_switch[ext_aim.target_number] != 0:
        aim_lock_pos_cnt = 0.8
        # aim_distance = ext_aim.


def game_status_callback(ext_status):
    global game_status, remain_time
    game_status = ext_status.game_progress
    remain_time = ext_status.stage_remain_time
    # print("                                               Game Status:", game_status)


def game_HP_callback(ext_HP):
    # pass
    global self_robot_HP, self_base_HP, self_outpost_HP, enemy_robot_HP, enemy_base_HP, enemy_outpost_HP
    global enemy_1_cnt, enemy_2_cnt, enemy_3_cnt, enemy_4_cnt, enemy_5_cnt
    if self_color == 'red':
        self_robot_HP = ext_HP.red_7_robot_HP
        self_base_HP = ext_HP.red_base_HP
        self_outpost_HP = ext_HP.red_outpost_HP
        enemy_robot_HP = ext_HP.blue_7_robot_HP
        enemy_base_HP = ext_HP.blue_base_HP
        enemy_outpost_HP = ext_HP.blue_outpost_HP

        if ext_HP.blue_1_robot_HP == 0:
            enemy_1_cnt = 10
        if ext_HP.blue_2_robot_HP == 0:
            enemy_2_cnt = 10
        if ext_HP.blue_3_robot_HP == 0:
            enemy_3_cnt = 10
        if ext_HP.blue_4_robot_HP == 0:
            enemy_4_cnt = 10
        if ext_HP.blue_5_robot_HP == 0:
            enemy_5_cnt = 10
    else:
        self_robot_HP = ext_HP.blue_7_robot_HP
        self_base_HP = ext_HP.blue_base_HP
        self_outpost_HP = ext_HP.blue_outpost_HP
        enemy_robot_HP = ext_HP.red_7_robot_HP
        enemy_base_HP = ext_HP.red_base_HP
        enemy_outpost_HP = ext_HP.red_outpost_HP

        if ext_HP.red_1_robot_HP == 0:
            enemy_1_cnt = 10
        if ext_HP.red_2_robot_HP == 0:
            enemy_2_cnt = 10
        if ext_HP.red_3_robot_HP == 0:
            enemy_3_cnt = 10
        if ext_HP.red_4_robot_HP == 0:
            enemy_4_cnt = 10
        if ext_HP.red_5_robot_HP == 0:
            enemy_5_cnt = 10
    # print("                               HP:", robot_HP)


def chassis_angle_callback(ext_chassis_angle):
    global chassis_angle, armor0_angle
    rad_angle = (ext_chassis_angle.data - 6446.00) / 8192.00 * 6.2831852
    if rad_angle < 0:
        rad_angle = rad_angle + 6.2831852
    chassis_angle = rad_angle
    armor0_angle = current_yaw - chassis_angle
    while armor0_angle < -3.1416:
        armor0_angle = armor0_angle + 6.28318
    while armor0_angle > 3.1416:
        armor0_angle = armor0_angle - 6.28318
    # print('         current_yaw:', current_yaw)
    # print('         chassis_angle', chassis_angle)
    # print('         armor0 yaw:', armor0_angle)


def cnt_timer_callback(event):
    global hurt_lock_pos_cnt, aim_lock_pos_cnt, command_cnt
    if hurt_lock_pos_cnt > 0:
        hurt_lock_pos_cnt = hurt_lock_pos_cnt - 0.1
    if aim_lock_pos_cnt > 0:
        aim_lock_pos_cnt = aim_lock_pos_cnt - 0.1
    if command_cnt > 0:
        command_cnt = command_cnt - 0.1


def death_robot_callback(event):
    global enemy_1_cnt, enemy_2_cnt, enemy_3_cnt, enemy_4_cnt, enemy_5_cnt

    if enemy_1_cnt > 0:
        enemy_1_cnt = enemy_1_cnt - 1
    else:
        enemy_1_cnt = 0
    if enemy_2_cnt > 0:
        enemy_2_cnt = enemy_2_cnt - 1
    else:
        enemy_2_cnt = 0
    if enemy_3_cnt > 0:
        enemy_3_cnt = enemy_3_cnt - 1
    else:
        enemy_3_cnt = 0
    if enemy_4_cnt > 0:
        enemy_4_cnt = enemy_4_cnt - 1
    else:
        enemy_4_cnt = 0
    if enemy_5_cnt > 0:
        enemy_5_cnt = enemy_5_cnt - 1
    else:
        enemy_5_cnt = 0


def target_xyz_callback():
    global self_base_HP, pre_base_HP, wait_attack_cnt, aim_lock_pos_cnt, target_yaw, current_yaw, target_x, target_y
    global pre_target_x, pre_target_y, pre_target_yaw, yaw_scan_state, target_publish_idle
    target_pose = PoseStamped()
    target_pose.header.frame_id = "map"
    frame_target_yaw = 0.0
    # if pow(current_x - target_x, 2) + pow(current_y - target_y, 2) > 1:
    #     return
    if aim_lock_pos_cnt > 0:
        target_pose.pose.position.x = current_x + random.uniform(-0.2, 0.2)
        target_pose.pose.position.y = current_y + random.uniform(-0.2, 0.2)
        frame_target_yaw = current_yaw
    elif hurt_lock_pos_cnt > 0:
        target_pose.pose.position.x = current_x + random.uniform(-0.5, 0.5)
        target_pose.pose.position.y = current_y + random.uniform(-0.5, 0.5)
        frame_target_yaw = hurt_angle
    else:
        target_pose.pose.position.x = strategy_target_x
        target_pose.pose.position.y = strategy_target_y
        if abs(target_yaw - current_yaw) <= 0.55:  # 0.349
            if yaw_scan_state == 0:
                target_yaw = target_yaw + 1.571
            elif yaw_scan_state == 1:
                target_yaw = strategy_target_yaw - 0.7
                yaw_scan_state = 2
            else:
                target_yaw = strategy_target_yaw + 0.7
                yaw_scan_state = 1

            if target_yaw > 3.14159:
                target_yaw = target_yaw - 6.2831852
            if target_yaw < -3.14159:
                target_yaw = target_yaw + 6.2881852
        frame_target_yaw = target_yaw

    # 发布
    if target_publish_idle > 0:
        if target_pose.pose.position.x == pre_target_x:
            if target_pose.pose.position.y == pre_target_y:
                if frame_target_yaw == pre_target_yaw:
                    target_publish_idle = target_publish_idle - 1
                    return
    pre_target_x = target_pose.pose.position.x
    pre_target_y = target_pose.pose.position.y
    pre_target_yaw = frame_target_yaw
    target_pose.pose.position.z = 0
    target_quad = quaternion_from_euler(0, 0, frame_target_yaw)
    target_pose.pose.orientation.x = target_quad[0]
    target_pose.pose.orientation.y = target_quad[1]
    target_pose.pose.orientation.z = target_quad[2]
    target_pose.pose.orientation.w = target_quad[3]
    location_target_publisher.publish(target_pose)
    target_publish_idle = 10
    # print('current pose:', current_yaw, 'target pose:', target_yaw)


def target_xyz_timer_callback(event):
    target_xyz_callback()


def game_command_callback(ext_command):
    global command_cnt, target_x, target_y, enemy_location_x, enemy_location_y, warning_cnt
    global strategy_state, yaw_scan_state, commander_x, commander_y
    global force_spinning, kill_sentry_first, kill_enemy_engineer, invincible_detect
    if ext_command.command_keyboard == 65:
        strategy_state = 1
        yaw_scan_state = 0
        command_cnt = 30
        command_cnt = 30
        if self_color == 'blue':
            commander_x = 22.1 - ext_command.target_position_x
            commander_y = 8.0 - ext_command.target_position_y
        else:
            commander_x = ext_command.target_position_x - 5.8
            commander_y = ext_command.target_position_y - 8.0
    elif ext_command.command_keyboard == 66:
        yaw_scan_state = 0
        strategy_state = 0
        command_cnt = 30
        if self_color == 'blue':
            commander_x = 22.1 - ext_command.target_position_x
            commander_y = 8.0 - ext_command.target_position_y
        else:
            commander_x = ext_command.target_position_x - 5.8
            commander_y = ext_command.target_position_y - 8.0
    elif ext_command.command_keyboard == 73:
        warning_cnt = 10
        yaw_scan_state = 1
        if self_color == 'blue':
            enemy_location_x = 22.1 - ext_command.target_position_x
            enemy_location_y = 8.0 - ext_command.target_position_y
        else:
            enemy_location_x = ext_command.target_position_x - 5.8
            enemy_location_y = ext_command.target_position_y - 8.0
    elif ext_command.command_keyboard == 67:
        command_cnt = 30
        strategy_state = 2
    elif ext_command.command_keyboard == 72:
        force_spinning = True
    elif ext_command.command_keyboard == 70:
        kill_enemy_engineer = True
    elif ext_command.command_keyboard == 71:
        kill_enemy_engineer = False
    elif ext_command.command_keyboard == 76:
        kill_sentry_first = False
    elif ext_command.command_keyboard == 78:
        kill_sentry_first = True
    elif ext_command.command_keyboard == 68:
        invincible_detect = True
    elif ext_command.command_keyboard == 69:
        invincible_detect = False
    target_xyz_callback()


def spin_timer_callback(event):
    global target_spinning_speed, force_spinning, lowest_spinning_speed
    spin_speed_msg = spinning_control()
    if force_spinning:
        lowest_spinning_speed = 16000
    elif self_outpost_HP > 0 and game_status == 4:
        lowest_spinning_speed = 0
    else:
        lowest_spinning_speed = 16000

    if target_spinning_speed > lowest_spinning_speed:
        target_spinning_speed = target_spinning_speed - 2000
    else:
        target_spinning_speed = lowest_spinning_speed

    if lowest_spinning_speed != 0:
        spin_speed_msg.spinning_speed = target_spinning_speed + random.randint(0, 4000)
    # spin_speed_msg.spinning_speed = 0
    spin_speed_publisher.publish(spin_speed_msg)
    print('     Debug:', strategy_target_x, strategy_target_y)


def game_hurt_callback(ext_hurt):
    global target_spinning_speed, hurt_angle, armor0_angle, hurt_lock_pos_cnt
    if hurt_lock_pos_cnt < 1.0:
        if ext_hurt.hurt_type == 0 and lowest_spinning_speed != 0:
            target_spinning_speed = 26000
            hurt_angle = armor0_angle + ext_hurt.armor_id * 1.571
            hurt_lock_pos_cnt = 1.5
            target_xyz_callback()


def tf_get_timer_callback(event):
    try:
        global trans, rot, current_x, current_y, current_yaw
        trans, rot = location_listener.lookupTransform('/map', '/plane_base_link', rospy.Time(0))
        current_x = trans[0]
        current_y = trans[1]
        current_yaw = (euler_from_quaternion(rot))[2]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
        # print("logic_node no TF get !")


def pitch_timer_callback(event):
    global pitch_state, pitch_scan, pitch_scan_low, pitch_scan_up, enemy_outpost_HP, strategy_state
    if enemy_outpost_HP > 0 and strategy_state == 1:
        pitch_scan_up = 25
        pitch_scan_low = 0
    else:
        pitch_scan_low = -20.0
        pitch_scan_up = 20

    if aim_lock_pos_cnt >= 0:
        pass
    else:
        if pitch_state == 0:
            pitch_scan = pitch_scan + 6
            if pitch_scan > pitch_scan_up:
                pitch_state = 1
        elif pitch_state == 1:
            pitch_scan = pitch_scan - 6
            if pitch_scan < pitch_scan_low:
                pitch_state = 0
        else:
            pass
    recommend_pitch_publisher.publish(pitch_scan)


if __name__ == '__main__':
    rospy.init_node('logic_control_node')
    location_listener = tf.TransformListener()
    location_target_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    recommend_pitch_publisher = rospy.Publisher('/robot/logic_recommend_angle', Float32, queue_size=1)
    spin_speed_publisher = rospy.Publisher('/robot/spinning_speed', spinning_control, queue_size=1)
    armor_select_publisher = rospy.Publisher('/robot/armor_select', armor_select, queue_size=1)

    chassis_angle_sub = rospy.Subscriber('/robot/chassis_angle', Float32, chassis_angle_callback)
    referee_status_sub = rospy.Subscriber('/referee/status', message_game_status, game_status_callback)
    referee_hurt_sub = rospy.Subscriber('/referee/hurt', message_game_hurt, game_hurt_callback)
    referee_HP_sub = rospy.Subscriber('/referee/HP', message_game_HP, game_HP_callback)
    aim_sub = rospy.Subscriber('/robot/auto_aim', aim, auto_aim_callback)
    command_sub = rospy.Subscriber('/referee/command', message_game_command, game_command_callback)

    timer_01 = rospy.Timer(rospy.Duration(0.5), target_xyz_timer_callback)
    timer_02 = rospy.Timer(rospy.Duration(0.1), pitch_timer_callback)
    timer_03 = rospy.Timer(rospy.Duration(0.1), tf_get_timer_callback)
    timer_04 = rospy.Timer(rospy.Duration(0.25), spin_timer_callback)
    timer_05 = rospy.Timer(rospy.Duration(1), strategy_callback)
    timer_06 = rospy.Timer(rospy.Duration(0.1), cnt_timer_callback)
    timer_07 = rospy.Timer(rospy.Duration(1), death_robot_callback)
    timer_08 = rospy.Timer(rospy.Duration(0.5), armor_select_callback)
    rospy.spin()

    recommend_pitch_publisher.publish(0)
    spin_speed_msg = spinning_control()
    spin_speed_msg.spinning_speed = 0
    spin_speed_publisher.publish(spin_speed_msg)

    print("Logic Control is shutting down!")
    exit()
