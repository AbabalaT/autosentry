#! /usr/bin/env python3
# -*- coding: utf-8 -*
import os
import sys
import tty, termios
import roslib
import random
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

robot_HP = 600

strategy_state = 0
seq_1 = 0
seq_2 = 0

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

target_yaw = 0
game_status = 0
'''
订阅自瞄
当相机自瞄没有数据时：
    当在目标点附近时：
        开启Yaw轴扫描
        开启Pitch轴扫描
'''
current_chassis_angle = 0 #底盘相对云台角度
target_spinning_speed = 15000 #期望小陀螺速度

self_aim_state = 0
location_target = 0
spinning_velocity = 0

def rps_spin_job():
    rospy.spin()
def game_staus_callback(ext_status):
    global game_status
    game_status = ext_status.game_progress
    print("                                               Game Status:", game_status)
def game_HP_callback(ext_HP):
    # pass
    global robot_HP
    robot_HP = ext_HP.red_7_robot_HP
    print("                               HP:",robot_HP)
def game_hurt_callback(ext_hurt):
    global target_spinning_speed
    if (ext_hurt.hurt_type == 0):
        target_spinning_speed = 20000
    # print(ext_hurt.game_type)
def game_command_callback(ext_command):
    pass
    # print(ext_command.game_type)
def chassis_angle_callback(ext_chassis_angle):
    pass

if __name__ == '__main__':
    rospy.init_node('logic_control_node')
    rate = rospy.Rate(10)

    location_listener = tf.TransformListener()
    location_target_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    recommend_pitch_publisher = rospy.Publisher('/robot/logic_recommend_angle', Float32, queue_size=1)
    spin_speed_pulisher = rospy.Publisher('/robot/spnning_speed', spinning_control, queue_size=1)
    chassis_angle_sub = rospy.Subscriber('/robot/chassis_angle', Int16, chassis_angle_callback)
    referee_status_sub = rospy.Subscriber('/referee/status', message_game_status, game_staus_callback)
    referee_hurt_sub = rospy.Subscriber('/referee/hurt', message_game_hurt, game_hurt_callback)
    referee_HP_sub = rospy.Subscriber('/referee/HP', message_game_HP, game_HP_callback)

    ros_spin_thread = threading.Thread(target = rospy.spin, daemon=True)
    ros_spin_thread.start()

    # while(game_status < 4):
    #     if rospy.is_shutdown():
    #         break
    #     print('Waiting For Match to start')
    #     rospy.sleep(0.25)

    while not rospy.is_shutdown():
        try:
            trans, rot = location_listener.lookupTransform('map', 'plane_base_link', rospy.Time(0))
            # print('position',trans,rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
            # print("logic no TF get !")

        seq_1 = seq_1 + 1
        if seq_1 >= 10:
            seq_1 = 0
            target_yaw = target_yaw + 0.95
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_quad = quaternion_from_euler(0, 0, target_yaw)
            target_pose.pose.orientation.x = target_quad[0]
            target_pose.pose.orientation.y = target_quad[1]
            target_pose.pose.orientation.z = target_quad[2]
            target_pose.pose.orientation.w = target_quad[3]

            target_pose.pose.position.x = 0
            target_pose.pose.position.y = 0
            target_pose.pose.position.z = 0

            location_target_publisher.publish(target_pose)
            if target_yaw > 31.415:
                target_yaw = -31.415
            # print('OK!')

        seq_2 = seq_2 + 1
        if seq_2 >= 10:
            seq_2 = 0
            spin_speed_msg = spinning_control()
            if target_spinning_speed > 15000:
                target_spinning_speed = target_spinning_speed - 2500
            spin_speed_msg.spinning_speed = target_spinning_speed + random.randint(0,4000)
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
    recommend_pitch_publisher.publish(0)
    spin_speed_msg = spinning_control()
    spin_speed_msg.spinning_speed = 0
    spin_speed_pulisher.publish(spin_speed_msg)
    print("Logic Control is shutting down!")
    exit()
