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

allow_out_cnt = 10.0    #回家倒计时，在该计时为0且前哨站已死亡时，目标点应该只出现在巡逻区内

kill_enemy_engineer = False # 是否打对方工程
invincible_detect = True    # 是否启用防无敌检测
kill_sentry_first = True    # 基地护甲以打开是否依然先打死哨兵

aim_distance = 0    #自瞄获得目标距离

warning_cnt = 0 #指向性警戒结束倒计时，不为0时云台指向境界点

pitch_scan_low = -20.0  #云台扫描下值
pitch_scan_up = 10.0    #云台扫描上值

yaw_scan_low = 0.0  
yaw_scan_up = 0.0
yaw_scan_state = 0  # 0：周扫描 1：扇正扫 2：扇反扫

target_publish_idle = 0 #不为0时此次不发送导航目标点，降低导航规划次数
yaw_refresh_idle = 0 #不更新导航目标航向

remain_time = 420   #距离比赛结束时间
require_add_HP = 0  #是否需要回补血区补血（省赛）

self_robot_HP = 1000    #自己的血量
self_outpost_HP = 1500  #大部分变量名是人话
self_base_HP = 5000

pre_base_HP = 5000  #上一时刻收到基地血量
pre_outpost_HP = 1500   #上一时刻收到前哨站血量

enemy_robot_HP = 600    #敌方哨兵血量
enemy_outpost_HP = 1500
enemy_base_HP = 5000

self_color = 'blue' #己方的颜色           ！！！一定要设置对！！！

hurt_lock_pos_cnt = 0   #收到攻击转向目标持续时间
aim_lock_pos_cnt = 0    #瞄准目标后在此航向停留时间

enemy_1_cnt = 0 #对方1号几秒内死过（10秒内死过，当前复活时为无敌状态，不攻击）
enemy_2_cnt = 0
enemy_3_cnt = 0
enemy_4_cnt = 0
enemy_5_cnt = 0

moving_cnt = -1
moving_direction = 0

strategy_state = 0  #   0： 防御模式 1： 攻击模式 2： 视觉拖曳

command_cnt = 0.0   #   云台手命令持续周期

hurt_armor = 0  # 受击装甲
chassis_angle = 0.0 # 底盘角度
armor0_angle = 0.0  # 0号装甲对世界系的角度，用于装甲感知
hurt_angle = 0.0    # 受击装甲板在世界系角度

commander_x = 0.0   #云台手点击位置
commander_y = 0.0

strategy_target_x = 0.0 #战略层目标位置
strategy_target_y = 0.0
strategy_target_yaw = 0.0

enemy_location_x = 11.1 #云台手警戒点/敌方位置（全向感知可以接入进来）
enemy_location_y = 4.81

target_x = 0.0  #战术层目标位置
target_y = 0.0
target_yaw = 0.0

pre_target_x = 0.0
pre_target_y = 0.0
pre_target_yaw = 0.0

force_standby_scanning = 0  #强制原地旋转扫描（哨兵疯了按一下原地转）

current_x = 0.0 #当前位置
current_y = 0.0
current_yaw = 0.0

aim_switch = [0, 1, 1, 1, 1, 1, 0, 0, 0]    #自瞄白名单

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

pitch_scan = 0.0
pitch_state = 0  # 0:上升 1: 下降

game_status = 0 #比赛阶段状态

'''
订阅自瞄
当相机自瞄没有数据时：
    当在目标点附近时：
        开启Yaw轴扫描
        开启Pitch轴扫描
'''

target_spinning_speed = 0  # 期望小陀螺速度
lowest_spinning_speed = 4000
force_spinning = False

self_aim_state = 0

trans = (0.0, 0.0, 0.0)
rot = (0.0, 0.0, 0.0, 0.0)


def at_patrol_check(x, y):
    return 0


def allow_out_timer_callback(event):    #允许出家计时-定时器回调
    global allow_out_cnt, self_outpost_HP
    if self_outpost_HP > 750:
        allow_out_cnt = 300.0
    elif self_outpost_HP > 450:
        allow_out_cnt = 15.0
    elif at_patrol_check(current_x, current_y) != 0:
        allow_out_cnt = 10.0
    else:
        allow_out_cnt = allow_out_cnt - 0.1


def armor_select_callback(event):   #白名单控制-定时器触发
    global armor_select_publisher, aim_switch
    aim_select_msg = armor_select()
    aim_select_msg.aim_1 = aim_switch[1]
    aim_select_msg.aim_2 = aim_switch[2]
    aim_select_msg.aim_3 = aim_switch[3]
    aim_select_msg.aim_4 = aim_switch[4]
    aim_select_msg.aim_5 = aim_switch[5]
    aim_select_msg.aim_robot = aim_switch[6]
    aim_select_msg.aim_outpost = aim_switch[7]
    aim_select_msg.aim_base = aim_switch[8]
    armor_select_publisher.publish(aim_select_msg)


def strategy_callback():    #战略层位置决策
    global warning_cnt, strategy_target_yaw, yaw_scan_state, command_cnt
    global strategy_target_x, strategy_target_y, strategy_state
    if command_cnt > 0:
        strategy_target_x = commander_x
        strategy_target_y = commander_y
    elif game_status != 4:
        strategy_target_x = -0.7
        strategy_target_y = 0.0
    else:
        strategy_target_x = -0.7
        strategy_target_y = 0.0
    target_xyz_callback()
    print('strategy_target:', strategy_target_x, strategy_target_y)


def auto_aim_callback(ext_aim): #自瞄回调
    global aim_lock_pos_cnt, aim_distance
    armor_number = ext_aim.target_number
    if armor_number > 8:
        armor_number = armor_number - 9
    if aim_switch[armor_number] != 0:
        aim_lock_pos_cnt = 0.8


def game_status_callback(ext_status):   #比赛阶段和结束时间信息回调
    global game_status, remain_time
    game_status = ext_status.game_progress
    remain_time = ext_status.stage_remain_time
    # print("                                               Game Status:", game_status)


def chassis_angle_callback(ext_chassis_angle):  #底盘0号装甲解算
    global chassis_angle, armor0_angle
    rad_angle = (ext_chassis_angle.data - 4804.00) / 8192.00 * 6.2831852    #4804.0是我们云台对0号装甲的绝对位置编码器数值
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


def cnt_timer_callback(event):  #几个不太重要的定时器一起计算
    global hurt_lock_pos_cnt, aim_lock_pos_cnt, command_cnt, warning_cnt
    if hurt_lock_pos_cnt > 0:
        hurt_lock_pos_cnt = hurt_lock_pos_cnt - 0.1

    if aim_lock_pos_cnt > 0:
        aim_lock_pos_cnt = aim_lock_pos_cnt - 0.1

    if command_cnt > 0:
        command_cnt = command_cnt - 0.1

    if warning_cnt > 0:
        warning_cnt = warning_cnt - 0.1


def death_robot_callback(event):#   已死亡机器人计时，死亡为10，活了之后过1秒减1秒，减到0时无敌解除
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


random_move_idle = 0.0


def target_xyz_callback():  #战术目标点选择
    global self_base_HP, pre_base_HP, wait_attack_cnt, aim_lock_pos_cnt
    global target_yaw, current_yaw, target_x, target_y, hurt_lock_pos_cnt
    global pre_target_x, pre_target_y, pre_target_yaw, yaw_scan_state, target_publish_idle, yaw_refresh_idle, random_move_idle
    target_pose = PoseStamped()
    target_pose.header.frame_id = "map"
    if random_move_idle > 0.0:
        random_move_idle = random_move_idle - 0.1
    frame_target_yaw = 0.0
    # if pow(current_x - target_x, 2) + pow(current_y - target_y, 2) > 1:
    #     return
    if aim_lock_pos_cnt > 0:
        if self_outpost_HP > 100:
            target_pose.pose.position.x = current_x
            target_pose.pose.position.y = current_y
        else:
            target_pose.pose.position.x = strategy_target_x
            target_pose.pose.position.y = strategy_target_y
        frame_target_yaw = current_yaw
    elif hurt_lock_pos_cnt > 0:
        if strategy_target_x == -0.7 and strategy_target_y == 0.0:
            if random_move_idle > 0.1:
                target_pose.pose.position.x = pre_target_x
                target_pose.pose.position.y = pre_target_y
                frame_target_yaw = hurt_angle
            else:
                target_pose.pose.position.x = strategy_target_x + random.uniform(-0.15, 0.15)
                target_pose.pose.position.y = strategy_target_y + random.uniform(-1.2, 1.2)
                frame_target_yaw = hurt_angle
                random_move_idle = 0.4
        else:
            target_pose.pose.position.x = strategy_target_x + random.uniform(-0.25, 0.25)
            target_pose.pose.position.y = strategy_target_y + random.uniform(-0.25, 0.25)
            frame_target_yaw = hurt_angle
    else:
        target_pose.pose.position.x = strategy_target_x
        target_pose.pose.position.y = strategy_target_y
        if yaw_refresh_idle > 0:
            yaw_refresh_idle = yaw_refresh_idle - 1
            if abs(target_yaw - current_yaw) <= 0.5:  # 0.349
                target_yaw = current_yaw + 1.0466
                yaw_refresh_idle = 5
        else:
            target_yaw = target_yaw + 1.0466
            yaw_refresh_idle = 5

        if target_yaw > 3.14159:
            target_yaw = target_yaw - 6.2831852
        if target_yaw < -3.14159:
            target_yaw = target_yaw + 6.2881852
        frame_target_yaw = target_yaw

    if warning_cnt > 0.1:
        delta_x = enemy_location_x - current_x
        delta_y = enemy_location_y - current_y
        theta = math.atan(delta_y / delta_x)
        if delta_x < 0:
            if delta_y > 0:
                theta += 3.1415926
            else:
                theta -= 3.1415926
        frame_target_yaw = theta
        print('warning information:', delta_x, delta_y, theta)

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
    target_publish_idle = 5
    # print('current pose:', current_yaw, 'target pose:', target_yaw)


def target_xyz_timer_callback(event):
    target_xyz_callback()


def game_command_callback(ext_command): #云台手命令回调
    global command_cnt, target_x, target_y, enemy_location_x, enemy_location_y, warning_cnt
    global strategy_state, yaw_scan_state, commander_x, commander_y, aim_switch
    global force_spinning, kill_sentry_first, kill_enemy_engineer, invincible_detect
    global moving_cnt, moving_direction, force_standby_scanning, enemy_outpost_HP
    signal_x = 0.0
    signal_y = 0.0

    if self_color == 'blue':    #红蓝方云台手按键位置转换到哨兵地图系坐标位置
        signal_x = (20.50 - ext_command.target_position_x) * 1.07
        signal_y = (8.12 - ext_command.target_position_y) * 1.07
    else:
        signal_x = (ext_command.target_position_x - 6.87) * 1.07
        signal_y = (ext_command.target_position_y - 8.12) * 1.07

    print('                 ', signal_x, signal_y, ext_command.target_position_x - 0.01,
          ext_command.target_position_y - 0.01)

    if 64 < ext_command.command_keyboard < 68:
        if self_outpost_HP > 200:
            command_cnt = 40
        else:
            command_cnt = 25
        commander_x = signal_x
        commander_y = signal_y

    if ext_command.command_keyboard == 65:
        aim_switch = [0, 1, 1, 1, 1, 1, 1, 0, 0]
    if ext_command.command_keyboard == 66:
        aim_switch = [0, 0, 0, 0, 0, 0, 0, 1, 0]
    if ext_command.command_keyboard == 67:
        aim_switch = [0, 0, 0, 0, 0, 0, 0, 0, 1]

    if ext_command.command_keyboard == 69:  #以下几个点都是哨兵常用位置，一键前往较为便捷
        commander_x = -0.685
        commander_y = -3.977
        command_cnt = 100

    if ext_command.command_keyboard == 70:
        commander_x = -0.627
        commander_y = 4.309
        command_cnt = 100

    if ext_command.command_keyboard == 72:
        commander_x = -0.7
        commander_y = 0.0
        command_cnt = 100

    if ext_command.command_keyboard == 81:
        commander_x = 6.42
        commander_y = -5.20
        command_cnt = 40

    if ext_command.command_keyboard == 82:
        commander_x = 9.92
        commander_y = -0.20
        command_cnt = 40

        enemy_location_x = 8.55
        enemy_location_y = 0.76
        warning_cnt = 25

    if ext_command.command_keyboard == 83:
        commander_x = 11.02
        commander_y = -6.57
        command_cnt = 180

    if ext_command.command_keyboard == 86:
        commander_x = 17.00
        commander_y = 1.80
        command_cnt = 180

    if ext_command.command_keyboard == 85:
        commander_x = 11.79
        commander_y = -1.83
        command_cnt = 180

    if enemy_outpost_HP > 5:  # turn off shooting unbeatable target
        aim_switch[6] = 0
        aim_switch[8] = 0
    if enemy_robot_HP > 10:
        aim_switch[8] = 0

    if ext_command.command_keyboard == 88:
        force_standby_scanning = 1

    if ext_command.command_keyboard == 84:  #点击左半面打开强制原地旋转扫描，右半屏幕关闭
        if ext_command.target_position_x > 14.0:
            force_spinning = True
        else:
            force_spinning = False

    if ext_command.command_keyboard == 71:
        enemy_location_x = signal_x
        enemy_location_y = signal_y
        warning_cnt = 10.0

    if ext_command.command_keyboard == 73:
        moving_cnt = 1.8
        moving_direction = 0
    if ext_command.command_keyboard == 74:
        moving_cnt = 1.8
        moving_direction = 1
    if ext_command.command_keyboard == 75:
        moving_cnt = 1.8
        moving_direction = 2
    if ext_command.command_keyboard == 76:
        moving_cnt = 1.8
        moving_direction = 3

    strategy_callback()


def game_HP_callback(ext_HP):   #裁判系统血量信号回调
    # pass
    global self_robot_HP, self_base_HP, self_outpost_HP, enemy_robot_HP, enemy_base_HP, enemy_outpost_HP
    global enemy_1_cnt, enemy_2_cnt, enemy_3_cnt, enemy_4_cnt, enemy_5_cnt, pre_outpost_HP
    global command_cnt, commander_x, commander_y
    global force_spinning, game_status
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

    if game_status == 4:
        if self_outpost_HP <= 200:
            if pre_outpost_HP > 200:
                commander_x = -0.627
                commander_y = 4.309
                command_cnt = 300
                strategy_callback()
        pre_outpost_HP = self_outpost_HP

    print("self outpost HP:", self_outpost_HP)


def force_moving_callback(event):   #强制挪车：万一导航卡住，按键挪一下
    global moving_cnt, moving_direction
    if moving_cnt >= 0:
        moving_cnt = moving_cnt - 0.04
        force_speed = Twist()
        if moving_cnt > 0.2:
            if moving_direction == 0:
                force_speed.linear.x = 0.4
                force_speed.linear.y = 0
                force_speed.angular.z = 0
            elif moving_direction == 1:
                force_speed.linear.x = 0
                force_speed.linear.y = 0.4
                force_speed.angular.z = 0
            elif moving_direction == 2:
                force_speed.linear.x = -0.4
                force_speed.linear.y = 0
                force_speed.angular.z = 0
            elif moving_direction == 3:
                force_speed.linear.x = 0
                force_speed.linear.y = -0.4
                force_speed.angular.z = 0
        else:
            force_speed.linear.x = 0
            force_speed.linear.y = 0
            force_speed.angular.z = 0
        force_moving_publisher.publish(force_speed)


def force_scanning_callback(event): #原地匀速旋转扫描
    force_flag = 0
    distance: float = math.sqrt(pow(pre_target_x - current_x, 2) + pow(pre_target_x - current_x, 2))
    if distance < 0.2:
        force_flag = 1
    if hurt_lock_pos_cnt > 0.01:
        force_flag = 0
    if aim_lock_pos_cnt > 0.01:
        force_flag = 0
    if warning_cnt > 0.01:
        force_flag = 0
    if moving_cnt > 0.1:
        force_flag = 0
    if force_standby_scanning == 1:
        force_flag = 1
    force_scanning_publisher.publish(force_flag)


def spin_timer_callback(event): #小陀螺速度决策-定时器触发
    global target_spinning_speed, force_spinning, lowest_spinning_speed
    spin_speed_msg = spinning_control()

    if force_spinning:
        lowest_spinning_speed = 23500
    elif game_status == 4:
        if self_outpost_HP > 100:
            lowest_spinning_speed = 0
        else:
            lowest_spinning_speed = 23500
    else:
        lowest_spinning_speed = 4000

    if target_spinning_speed > lowest_spinning_speed:
        target_spinning_speed = target_spinning_speed - 2000
    else:
        target_spinning_speed = lowest_spinning_speed

    if lowest_spinning_speed != 0:
        spin_speed_msg.spinning_speed = target_spinning_speed + random.randint(0, 4000)
    # spin_speed_msg.spinning_speed = 0
    spin_speed_publisher.publish(spin_speed_msg)
    print(strategy_target_x, strategy_target_y, 'lowest spinning:', lowest_spinning_speed, force_spinning)


def game_hurt_callback(ext_hurt):   #裁判系统伤害信号
    global target_spinning_speed, hurt_angle, armor0_angle, hurt_lock_pos_cnt, lowest_spinning_speed
    if hurt_lock_pos_cnt < 1.0:
        if ext_hurt.hurt_type == 0:
            hurt_angle = armor0_angle + ext_hurt.armor_id * 1.571
            if strategy_target_x == -0.7 and strategy_target_y == 0.0:
                hurt_lock_pos_cnt = 3.0
            else:
                hurt_lock_pos_cnt = 1.5
            if lowest_spinning_speed != 0:
                target_spinning_speed = 26000
            target_xyz_callback()


def strategy_timer_callback(event):
    strategy_callback()


def tf_get_timer_callback(event):   #位置获取
    try:
        global trans, rot, current_x, current_y, current_yaw
        trans, rot = location_listener.lookupTransform('/map', '/plane_base_link', rospy.Time(0))
        current_x = trans[0]
        current_y = trans[1]
        current_yaw = (euler_from_quaternion(rot))[2]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("logic_node no TF get !")


def pitch_timer_callback(event):    #俯仰值决策-定时器触发
    global pitch_state, pitch_scan, pitch_scan_low, pitch_scan_up, aim_switch
    if aim_switch[7] > 0:
        pitch_scan_up = 25.0
        pitch_scan_low = 0.0
    else:
        pitch_scan_low = -22.0
        pitch_scan_up = 2.0

    if aim_lock_pos_cnt > 0:
        pass
    else:
        if pitch_state == 0:
            pitch_scan = pitch_scan + 0.6
            if pitch_scan > pitch_scan_up:
                pitch_state = 1
        else:
            pitch_scan = pitch_scan - 0.6
            if pitch_scan < pitch_scan_low:
                pitch_state = 0
    recommend_pitch_publisher.publish(pitch_scan)


if __name__ == '__main__':
    rospy.init_node('logic_control_node')
    location_listener = tf.TransformListener()
    location_target_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    recommend_pitch_publisher = rospy.Publisher('/robot/logic_recommend_angle', Float32, queue_size=1)
    spin_speed_publisher = rospy.Publisher('/robot/spinning_speed', spinning_control, queue_size=1)
    armor_select_publisher = rospy.Publisher('/robot/armor_select', armor_select, queue_size=1)
    force_moving_publisher = rospy.Publisher('/cmd_vel', Twist)
    force_scanning_publisher = rospy.Publisher('/cmd_force_scanning', Int16, queue_size=1)

    chassis_angle_sub = rospy.Subscriber('/robot/chassis_angle', Float32, chassis_angle_callback)
    referee_status_sub = rospy.Subscriber('/referee/status', message_game_status, game_status_callback)
    referee_hurt_sub = rospy.Subscriber('/referee/hurt', message_game_hurt, game_hurt_callback)
    referee_HP_sub = rospy.Subscriber('/referee/HP', message_game_HP, game_HP_callback)
    aim_sub = rospy.Subscriber('/robot/auto_aim', aim, auto_aim_callback)
    command_sub = rospy.Subscriber('/referee/command', message_game_command, game_command_callback)
    # kalman_sub = rospy.Subscriber('/kalman_scope', kalman, kalman_scope_callback)

    timer_01 = rospy.Timer(rospy.Duration(0.2), target_xyz_timer_callback)
    timer_02 = rospy.Timer(rospy.Duration(0.0125), pitch_timer_callback)
    timer_03 = rospy.Timer(rospy.Duration(0.1), tf_get_timer_callback)
    timer_04 = rospy.Timer(rospy.Duration(0.25), spin_timer_callback)
    timer_05 = rospy.Timer(rospy.Duration(4.0), strategy_timer_callback)
    timer_06 = rospy.Timer(rospy.Duration(0.1), cnt_timer_callback)
    timer_07 = rospy.Timer(rospy.Duration(1), death_robot_callback)
    timer_08 = rospy.Timer(rospy.Duration(0.5), armor_select_callback)
    timer_09 = rospy.Timer(rospy.Duration(0.04), force_moving_callback)
    timer_10 = rospy.Timer(rospy.Duration(0.1), allow_out_timer_callback)
    timer_11 = rospy.Timer(rospy.Duration(0.1), force_scanning_callback)

    rospy.spin()

    recommend_pitch_publisher.publish(0)
    spin_speed_msg = spinning_control()
    spin_speed_msg.spinning_speed = 0
    spin_speed_publisher.publish(spin_speed_msg)

    print("Logic Control is shutting down!")
    exit()
