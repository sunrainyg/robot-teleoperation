#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Authors: Yulu Gan
# Email: yulugan1@gmail.com

import os, sys
import rospy
import threading
import numpy as np

import time
import math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray
from tf.transformations import *
from datamessage.msg import bend

from yumipy import YuMiRobot
from yumipy import YuMiRobotGripper
from yumipy import YuMiState
from yumipy import YuMiConstants as YMC
from autolab_core import RigidTransform


# All sizes in m!
hz = 100 # hz

## 定义订阅话题的回调函数
## //////////////////////////////////////////////////////////////////////////////////////////////////
# define the callback func for subcribing the data from neuron right hand
def neuron_r_callback(data):
    # 定义neuron数据全局变量，进行赋值
    global neuron_Rx, neuron_Ry, neuron_Rz, neuron_RQux, neuron_RQuy, neuron_RQuz, neuron_RQuw
    # 右手位置及姿态坐标值
    neuron_Rx = data.pose.position.x
    neuron_Ry = data.pose.position.y
    neuron_Rz = data.pose.position.z
    neuron_RQux = data.pose.orientation.x
    neuron_RQuy = data.pose.orientation.y
    neuron_RQuz = data.pose.orientation.z
    neuron_RQuw = data.pose.orientation.w

# define the callback func for subcribing the data from neuron left hand
def neuron_l_callback(data):
    # 定义neuron数据全局变量，进行赋值
    global neuron_Lx, neuron_Ly, neuron_Lz, neuron_LQux, neuron_LQuy, neuron_LQuz, neuron_LQuw
    # 左手位置及姿态坐标值
    neuron_Lx = data.pose.position.x
    neuron_Ly = data.pose.position.y
    neuron_Lz = data.pose.position.z
    neuron_LQux = data.pose.orientation.x
    neuron_LQuy = data.pose.orientation.y
    neuron_LQuz = data.pose.orientation.z
    neuron_LQuw = data.pose.orientation.w

# define the callback func for subcribing the data from dataglove
def glove_callback(data):
    global Rightfinger, Leftfinger, RightfingerT, LeftfingerT
    # 定义手套数据全局变量，进行赋值
    Rightfinger = round(data.RI,3)
    Leftfinger = round(data.LI,3)
    RightfingerT = round(data.RT,3)
    LeftfingerT = round(data.LT,3)

## 定义neuron增量计算函数
## //////////////////////////////////////////////////////////////////////////////////////////////////
# 计算右手手部位置position增量的函数    
def neuron_delta_R():
    global neuron_Rx_pre, neuron_Ry_pre, neuron_Rz_pre
    delta_Rx = 0.8*round(neuron_Rx - neuron_Rx_pre,10)
    delta_Ry = 0.8*round(neuron_Ry - neuron_Ry_pre,10)
    delta_Rz = 0.8*round(neuron_Rz - neuron_Rz_pre,10)
    neuron_Rx_pre = neuron_Rx
    neuron_Ry_pre = neuron_Ry
    neuron_Rz_pre = neuron_Rz
    delta_all_R = (delta_Rx,delta_Ry,delta_Rz)
    return delta_all_R

# 计算左手手部位置position增量的函数    
def neuron_delta_L():
    global neuron_Lx_pre, neuron_Ly_pre, neuron_Lz_pre
    delta_Lx = 0.8*round(neuron_Lx - neuron_Lx_pre,10)
    delta_Ly = 0.8*round(neuron_Ly - neuron_Ly_pre,10)
    delta_Lz = 0.8*round(neuron_Lz - neuron_Lz_pre,10)
    neuron_Lx_pre = neuron_Lx
    neuron_Ly_pre = neuron_Ly
    neuron_Lz_pre = neuron_Lz
    delta_all_L = (delta_Lx,delta_Ly,delta_Lz)
    return delta_all_L

## 定义手指抓握动作函数
## //////////////////////////////////////////////////////////////////////////////////////////////////

## 定义机器人的左右手左右臂的运动进程函数
## //////////////////////////////////////////////////////////////////////////////////////////////////
def right_arm():
    global Rightfinger, Leftfinger, RightfingerT, LeftfingerT, Rightfinger_pre, Leftfinger_pre, RightfingerT_pre, LeftfingerT_pre
    global neuron_Rx, neuron_Ry, neuron_Rz, neuron_RQux, neuron_RQuy, neuron_RQuz, neuron_RQuw
    global neuron_Rx_pre, neuron_Ry_pre, neuron_Rz_pre

    time.sleep(5)
    rate = rospy.Rate(hz)

    while (not rospy.is_shutdown()):
        if (RightfingerT < -80) & (RightfingerT_pre >= -80):
            RightfingerT_pre = RightfingerT
            print RightfingerT_pre #debug
            print "Rarm egm start"
            n = 0            
            # get current right arm pose
            (yumi_Rx_pre,yumi_Ry_pre,yumi_Rz_pre)=yg.right.get_pose().translation
            (yumi_ROrw_pre,yumi_ROrx_pre,yumi_ROry_pre,yumi_ROrz_pre)=yg.right.get_pose().quaternion
            print ("right arm pose:")
            print (yumi_Rx_pre,yumi_Ry_pre,yumi_Rz_pre)
            print (yumi_ROrw_pre,yumi_ROrx_pre,yumi_ROry_pre,yumi_ROrz_pre)
            # 计算机器人末端姿态与人手姿态的相对旋转矩阵
            q_rot_r = quaternion_multiply([yumi_ROrx_pre,yumi_ROry_pre,yumi_ROrz_pre,yumi_ROrw_pre], [neuron_RQuz, neuron_RQux, neuron_RQuy, -neuron_RQuw])
            # start right arm egm
            y.right.start_egm()

            # 更新当前neuron状态值为pre值
            neuron_Rx_pre = neuron_Rx
            neuron_Ry_pre = neuron_Ry
            neuron_Rz_pre = neuron_Rz

            while (not rospy.is_shutdown()):
                now = rospy.Time.now()
                pose = PoseStamped()
                pose.header = Header()
                pose.header.stamp = now
                pose.header.frame_id = "map"
                # Position in m 增量移动
                delta_all_R = neuron_delta_R() #得到穿戴设备的xyz方向增量
                pose.pose.position.x = yumi_Rx_pre + delta_all_R[2] #给机器人的x方向坐标加上增量，即得到下一次运动的位置
                pose.pose.position.y = yumi_Ry_pre + delta_all_R[0]
                pose.pose.position.z = yumi_Rz_pre + delta_all_R[1]
                # # 保持姿态不变化
                # pose.pose.orientation.x = yumi_ROrx_pre
                # pose.pose.orientation.y = yumi_ROry_pre
                # pose.pose.orientation.z = yumi_ROrz_pre
                # pose.pose.orientation.w = yumi_ROrw_pre
                # 跟随姿态变化
                # 优化目标四元数
                [neuron_RQux_opt, neuron_RQuy_opt, neuron_RQuz_opt, neuron_RQuw_opt] = quaternion_multiply(-q_rot_r, [neuron_RQuz, neuron_RQux, neuron_RQuy, neuron_RQuw])
                pose.pose.orientation.x = neuron_RQux_opt
                pose.pose.orientation.y = neuron_RQuy_opt
                pose.pose.orientation.z = neuron_RQuz_opt
                pose.pose.orientation.w = neuron_RQuw_opt
                command_pose_pub_r.publish(pose)
                # 更新机器人当前值
                yumi_Rx_pre = pose.pose.position.x
                yumi_Ry_pre = pose.pose.position.y
                yumi_Rz_pre = pose.pose.position.z 
                yumi_ROrx_pre = pose.pose.orientation.x
                yumi_ROry_pre = pose.pose.orientation.y
                yumi_ROrz_pre = pose.pose.orientation.z
                yumi_ROrw_pre = pose.pose.orientation.w
                # set break       
                if (RightfingerT >= -80) & (RightfingerT_pre < -80):
                    RightfingerT_pre = RightfingerT
                    print RightfingerT_pre #debug
                    print "Rarm egm end processing"
                    break
                rate.sleep()
            # stop right arm egm
            yg.left.stop_antithetic_egm()
            print "Rarm egm end finished"

def left_arm():
    global Rightfinger, Leftfinger, RightfingerT, LeftfingerT, Rightfinger_pre, Leftfinger_pre, RightfingerT_pre, LeftfingerT_pre
    global neuron_Lx, neuron_Ly, neuron_Lz, neuron_LQux, neuron_LQuy, neuron_LQuz, neuron_LQuw
    global neuron_Lx_pre, neuron_Ly_pre, neuron_Lz_pre

    time.sleep(5)
    rate = rospy.Rate(hz)    

    while (not rospy.is_shutdown()):
        if (LeftfingerT < -80) & (LeftfingerT_pre >= -80):
            LeftfingerT_pre = LeftfingerT
            print LeftfingerT_pre #debug
            print "Larm egm start"
            n = 0            
            # get current left arm pose
            (yumi_Lx_pre,yumi_Ly_pre,yumi_Lz_pre)=yg.left.get_pose().translation
            (yumi_LOrw_pre,yumi_LOrx_pre,yumi_LOry_pre,yumi_LOrz_pre)=yg.left.get_pose().quaternion
            print ("left arm pose:")
            print (yumi_Lx_pre,yumi_Ly_pre,yumi_Lz_pre)
            print (yumi_LOrw_pre,yumi_LOrx_pre,yumi_LOry_pre,yumi_LOrz_pre)
            # 计算机器人末端姿态与人手姿态的相对旋转矩阵
            q_rot_l = quaternion_multiply([yumi_LOrx_pre,yumi_LOry_pre,yumi_LOrz_pre,yumi_LOrw_pre], [neuron_LQuz, neuron_LQux, neuron_LQuy, -neuron_LQuw])
            # start left arm egm
            y.left.start_egm()   

            # 更新当前neuron状态值为pre值
            neuron_Lx_pre = neuron_Lx
            neuron_Ly_pre = neuron_Ly
            neuron_Lz_pre = neuron_Lz

            while (not rospy.is_shutdown()):
                now = rospy.Time.now()
                pose = PoseStamped()
                pose.header = Header()
                pose.header.stamp = now
                pose.header.frame_id = "map"
                # Position in m 增量移动
                delta_all_L = neuron_delta_L()
                pose.pose.position.x = yumi_Lx_pre + delta_all_L[2]
                pose.pose.position.y = yumi_Ly_pre + delta_all_L[0]
                pose.pose.position.z = yumi_Lz_pre + delta_all_L[1]
                # # 保持姿态不变化
                # pose.pose.orientation.x = yumi_LOrx_pre
                # pose.pose.orientation.y = yumi_LOry_pre
                # pose.pose.orientation.z = yumi_LOrz_pre
                # pose.pose.orientation.w = yumi_LOrw_pre
                # 跟随姿态变化
                # 优化目标四元数
                [neuron_LQux_opt, neuron_LQuy_opt, neuron_LQuz_opt, neuron_LQuw_opt] = quaternion_multiply(-q_rot_l, [neuron_LQuz, neuron_LQux, neuron_LQuy, neuron_LQuw])
                pose.pose.orientation.x = neuron_LQux_opt
                pose.pose.orientation.y = neuron_LQuy_opt
                pose.pose.orientation.z = neuron_LQuz_opt
                pose.pose.orientation.w = neuron_LQuw_opt
                command_pose_pub_l.publish(pose)
                # 更新机器人当前值
                yumi_Lx_pre = pose.pose.position.x
                yumi_Ly_pre = pose.pose.position.y
                yumi_Lz_pre = pose.pose.position.z 
                yumi_LOrx_pre = pose.pose.orientation.x
                yumi_LOry_pre = pose.pose.orientation.y
                yumi_LOrz_pre = pose.pose.orientation.z
                yumi_LOrw_pre = pose.pose.orientation.w     
                # set break
                if (LeftfingerT >= -80) & (LeftfingerT_pre < -80):
                    LeftfingerT_pre = LeftfingerT
                    print LeftfingerT_pre #debug
                    print "Larm egm end processing"
                    break
                rate.sleep()
            # stop left arm egm
            yg.right.stop_antithetic_egm()
            print "Larm egm end finished"

def right_hand():
    global Rightfinger, Leftfinger, RightfingerT, LeftfingerT, Rightfinger_pre, Leftfinger_pre, RightfingerT_pre, LeftfingerT_pre
    time.sleep(5)
    while (not rospy.is_shutdown()):
        if (Rightfinger < -80) & (Rightfinger_pre >= -80):
            Rightfinger_pre = Rightfinger
            # print Rightfinger_pre #debug
            print "Rhand close"
            yg.right.close_gripper()
        if (Rightfinger >= -80) & (Rightfinger_pre < -80):
            Rightfinger_pre = Rightfinger
            # print Rightfinger_pre #debug
            print "Rhand open"
            yg.right.open_gripper()

def left_hand():
    global Rightfinger, Leftfinger, RightfingerT, LeftfingerT, Rightfinger_pre, Leftfinger_pre, RightfingerT_pre, LeftfingerT_pre
    time.sleep(5)
    while (not rospy.is_shutdown()):
        if (Leftfinger < -80) & (Leftfinger_pre >= -80):
            Leftfinger_pre = Leftfinger
            # print Leftfinger_pre #debug
            print "Lhand close"
            yg.left.close_gripper()
        if (Leftfinger >= -80) & (Leftfinger_pre < -80):
            Leftfinger_pre = Leftfinger
            # print Leftfinger_pre #debug
            print "Lhand open"
            yg.left.open_gripper()
        

def run():
    # 全局变量定义
    global y, yg
    global command_pose_pub_r, command_pose_pub_l

    global yumi_Rx_pre,yumi_Ry_pre,yumi_Rz_pre, yumi_ROrw_pre,yumi_ROrx_pre,yumi_ROry_pre,yumi_ROrz_pre
    global yumi_Lx_pre,yumi_Ly_pre,yumi_Lz_pre, yumi_LOrw_pre,yumi_LOrx_pre,yumi_LOry_pre,yumi_LOrz_pre

    global Rightfinger, Leftfinger, RightfingerT, LeftfingerT, Rightfinger_pre, Leftfinger_pre, RightfingerT_pre, LeftfingerT_pre
    global neuron_Rx_pre, neuron_Ry_pre, neuron_Rz_pre, neuron_Lx_pre, neuron_Ly_pre, neuron_Lz_pre

    # 初始化节点及话题的接收发布
    rospy.init_node('egm_teleop')
    # Subcribe the topic from neuron
    rospy.Subscriber('neuron_tf_righthand', PoseStamped, neuron_r_callback)
    rospy.Subscriber('neuron_tf_lefthand', PoseStamped, neuron_l_callback)
    # Subcribe the topic from dataglove
    rospy.Subscriber('dataglove', bend, glove_callback)
    # define the publisher
    command_pose_pub_r = rospy.Publisher('/command_pose_right', PoseStamped, queue_size = 100, latch=True)
    command_pose_pub_l = rospy.Publisher('/command_pose_left', PoseStamped, queue_size = 100, latch=True)
    rospy.set_param('egm_mode', 'position')

    # 初始化机器人YuMi
    print "press enter to init the robot"
    # raw_input()
    y = YuMiRobot()
    y.right.set_tool(RigidTransform(translation=[0,0,0]))
    y.left.set_tool(RigidTransform(translation=[0,0,0]))
    # 设置TCP速度
    v_r = y.get_v(200)
    y.right.set_speed(v_r)
    v_l = y.get_v(200)
    y.left.set_speed(v_l)
    y.reset_home()
    # init yg
    yg = YuMiRobotGripper()
    yg.right.set_tool(RigidTransform(translation=[0,0,0]))
    yg.left.set_tool(RigidTransform(translation=[0,0,0]))
    # calibrate grippers    
    yg.calibrate_grippers()
    yg.right.set_gripper_max_speed(25)
    yg.left.set_gripper_max_speed(25)

    # # 安全初始关节状态归位
    # y.right.goto_state(YuMiState([22.25, -94.35, -17.27, -57.79, 48.15, 113.51, -80.27]))
    # y.left.goto_state(YuMiState([-22.25, -94.35, -17.27, 57.79, 48.15, -113.51, 80.27]))
    # y.right.goto_state(YuMiState([0.72, -98.86, 41.57, -114.62, 28.13, 172.57, -99.15]))
    # # y.right.goto_state(YuMiState([24.24, -75.82, 10.16, -16.57, 57.57, 15.23, -98.58]))
    # y.left.goto_state(YuMiState([-0.72, -98.86, 41.57, 114.62, 28.13, -172.57, 99.15]))
    y.right.goto_state(YMC.R_TELOP_READY_STATE)
    y.left.goto_state(YMC.L_TELOP_READY_STATE)
    
    # 获取当前neuron位姿
    print "press enter get current neuron pose"
    # raw_input()
    # # neuron right arm
    neuron_Rx_pre = neuron_Rx
    neuron_Ry_pre = neuron_Ry
    neuron_Rz_pre = neuron_Rz
    # neuron left arm
    neuron_Lx_pre = neuron_Lx
    neuron_Ly_pre = neuron_Ly
    neuron_Lz_pre = neuron_Lz
    
    # 获取当前手套位姿
    print "press enter get current finger pose data"
    # raw_input()
    Rightfinger_pre = Rightfinger
    Leftfinger_pre = Leftfinger  
    RightfingerT_pre = RightfingerT
    LeftfingerT_pre = LeftfingerT
    # print Rightfinger_pre #debug
    # print Leftfinger_pre #debug
    # print RightfingerT_pre #debug
    # print LeftfingerT_pre #debug

    # 设置EGM执行状态
    global current_egm_state
    current_egm_state = 0

    time.sleep(1)
      
    t_right_arm = threading.Thread(target = right_arm)
    t_left_arm = threading.Thread(target = left_arm)
    t_right_arm.start()
    t_left_arm.start()
    t_right_hand = threading.Thread(target = right_hand)
    t_left_hand = threading.Thread(target = left_hand)
    t_right_hand.start()
    t_left_hand.start()

    rospy.spin()

if __name__ == '__main__':
    try:
        run()
        print("##Program finished##")
        
    except rospy.ROSInterruptException:
        pass
