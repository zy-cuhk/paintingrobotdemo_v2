#!/usr/bin/env python
# -*- coding: utf_8 -*-
"""
id：1---->stand bar
id:2----->roation
id:3------>Upper and lower climbing pole
"""
import rospy
import time
import os

def flexbar_upwardsmotion_process():
    # upwards motion of flexbar mechanism 
    os.system('rosparam set /renov_up_level/write_flex_pole_motor_up 1')
    # rospy.logerr("upwards motion of flexbar mecahnism")
def flexbar_upwardsmotion_end():
    os.system('rosparam set /renov_up_level/write_flex_pole_motor_up 0')
    # rospy.logerr("upwards motion of flexbar mechanism is closed")

def flexbar_downwardsmotion_process():
    # downwards motion of flexbar mechanism
    os.system('rosparam set /renov_up_level/write_flex_pole_motor_down 2')
    # rospy.logerr("downwards motion of flexbar mechanism")   
def flexbar_downwardsmotion_end():
    os.system('rosparam set /renov_up_level/write_flex_pole_motor_down 0') 
    # rospy.logerr("downwards motion of flexbar mechanism is closed")   

def rotation_enable(target_rotation_angle):
    # motion of rotation mechanism 
    os.system('rosparam set /renov_up_level/enable_control_rotation 1')
    os.system('rosparam set /renov_up_level/enable_control_rotation 0')
    os.system('rosparam set /renov_up_level/open_rotation_flag 1')
    os.system('rosparam set /renov_up_level/rad_control_rotation '+str(target_rotation_angle))
def rotation_disable():
    os.system('rosparam set /renov_up_level/open_rotation_flag 0')
    # os.system('rosparam set /renov_up_level/enable_control_rotation 2')
    # os.system('rosparam set /renov_up_level/enable_control_rotation 0')

def climb_enable(target_distance):
    # the motion command of climb mechanism
    os.system('rosparam set /renov_up_level/enable_climb_control 1')
    os.system('rosparam set /renov_up_level/enable_climb_control 0')
    os.system('rosparam set /renov_up_level/open_climb_flag 1')
    os.system('rosparam set /renov_up_level/distance_climb_control '+str(target_distance))

def climb_disable():
    os.system('rosparam set /renov_up_level/open_climb_flag 0')
    # os.system('rosparam set /renov_up_level/enable_climb_control 2')
    # os.system('rosparam set /renov_up_level/enable_climb_control 0')

def standbar_motion_process(target_standbar_displacement):
    target_motion_distance=-target_standbar_displacement
    # motion of standbar mechanism
    os.system('rosparam set /renov_up_level/open_hold_flag 1')
    os.system('rosparam set /renov_up_level/enable_control_stand_bar 1')
    os.system('rosparam set /renov_up_level/enable_control_stand_bar 0')
    os.system('rosparam set /renov_up_level/distance_control_stand_bar '+str(target_motion_distance))
    # rospy.loginfo("the homing of standbar mechanism in process")

def standbar_motion_end():
    os.system('rosparam set /renov_up_level/open_hold_flag 0')
    # os.system('rosparam set /renov_up_level/enable_control_stand_bar 2')
    # os.system('rosparam set /renov_up_level/enable_control_stand_bar 0')
    # rospy.logerr("the motion of standbar mechanism is closed")

def standbar_homing_process(target_standbar_displacement):
    target_homing_distance=-target_standbar_displacement
    # homing of standbar mechanism
    os.system('rosparam set /renov_up_level/open_hold_flag 1')
    os.system('rosparam set /renov_up_level/enable_control_stand_bar 1')
    os.system('rosparam set /renov_up_level/enable_control_stand_bar 0')
    os.system('rosparam set /renov_up_level/enable_second_control_stand_bar 1')
    os.system('rosparam set /renov_up_level/distance_control_stand_bar '+str(target_homing_distance))
    # rospy.loginfo("the homing of standbar mechanism in process")

def standbar_homing_end():
    os.system('rosparam set /renov_up_level/open_hold_flag 0')
    # os.system('rosparam set /renov_up_level/enable_control_stand_bar 2')
    # os.system('rosparam set /renov_up_level/enable_control_stand_bar 0')
    os.system('rosparam set /renov_up_level/enable_second_control_stand_bar 0')
    # rospy.loginfo("the motion of standbar mechanism is closed")



