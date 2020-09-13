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
import math
from jackup_mechanism_basic_functions import *

def rodclimb_mechanism_motion(target_rotation_angle,target_distance,rate):
    motion_state_current2last()
    rospy.loginfo("target_rotation_angle is: %s,target_distance is: %s"%(str(target_rotation_angle),str(target_distance)))
    while not rospy.is_shutdown():
        last_motion_phase_over_flag = rospy.get_param("/renov_up_level/last_motion_phase_over_flag")
        #rospy.loginfo("%s is %s", rospy.resolve_name('last_motion_phase_over_flag'), last_motion_phase_over_flag)
        # manipulator_renovation_over_flag=rospy.get_param("/renov_up_level/manipulator_renovation_over_flag")
        # if rodmechanism_holding_over_flag==1  or manipulator_renovation_over_flag==1:
        if last_motion_phase_over_flag==1:
            # rotation_enable(target_rotation_angle)
            # rospy.loginfo("the motion of rotation mechanism in process")
            climb_enable(target_distance)
            # rospy.loginfo("the homing of climbing mechanism in process")
            rospy.loginfo("step 3: rod_climbing_mechanism_motion is in process")
            # os.system('rosparam set /renov_up_level/last_motion_phase_over_flag 1')

        # output tracking errors 
        rotation_joint_line_equation_k=rospy.get_param("/renov_up_level/rotation_joint_line_equation_k")
        rotation_joint_line_equation_b=rospy.get_param("/renov_up_level/rotation_joint_line_equation_b")
        pid_tolerance_error_rotation=rospy.get_param("/renov_up_level/pid_tolerance_error_rotation")
        # current_rotation_angle=rospy.get_param("/renov_up_level/rotation_abs_encode")*rotation_joint_line_equation_k+rotation_joint_line_equation_b
        # rotation_trackingerror= target_rotation_angle-current_rotation_angle
        # rospy.loginfo(("%s is %f"%("rotation target angle",target_rotation_angle)))
        # rospy.loginfo(("%s is %f"%("rotation current angle",current_rotation_angle)))
        # rospy.logerr(("%s is %f"%("rotation tracking error",rotation_trackingerror)))

        # output tracking errors 
        pid_tolerance_error_climb=rospy.get_param("/renov_up_level/pid_tolerance_error_climb")
        read_line_l0_encode=rospy.get_param("/renov_up_level/read_line_l0_encode")
        climb_tracking_error=read_line_l0_encode+target_distance-rospy.get_param("/renov_up_level/read_line_encode")
        rospy.logerr(("%s is %f"%("tracking error",climb_tracking_error)))

        if abs(climb_tracking_error)<=pid_tolerance_error_climb:
        # if abs(rotation_trackingerror)<=pid_tolerance_error_rotation and abs(climb_tracking_error)<=pid_tolerance_error_climb:
            rotation_disable()
            # rospy.loginfo("the motion of rotation mechanism is closed")
            climb_disable()
            # rospy.loginfo("the motion of climbing mechanism is closed")
            rospy.loginfo("step 3: rod_climbing_mechanism_motion is closed")
            os.system('rosparam set /renov_up_level/current_motion_phase_over_flag 1')
            os.system('rosparam set /renov_up_level/last_motion_phase_over_flag 0')
            # os.system('rosparam set /renov_up_level/rodmechanism_holding_over_flag 0')
            # os.system('rosparam set /renov_up_level/climbingmechanism_climbing_over_flag 1')
            break
        rate.sleep()

def rodclimb_mechanism_motion_simulation(target_rotation_angle,target_distance,rate):
    rospy.loginfo("target_rotation_angle is: %s,target_distance is: %s"%(str(target_rotation_angle),str(target_distance)))
    rodclimbing_tracking_error=0.1
    
    motion_state_current2last()
    while not rospy.is_shutdown():
        last_motion_phase_over_flag = rospy.get_param("/renov_up_level/last_motion_phase_over_flag")
        # manipulator_renovation_over_flag=rospy.get_param("/renov_up_level/manipulator_renovation_over_flag")
        # rospy.loginfo("%s is %s", rospy.resolve_name('last_motion_phase_over_flag'), last_motion_phase_over_flag)
        # if rodmechanism_holding_over_flag==1 or manipulator_renovation_over_flag==1:
        if last_motion_phase_over_flag==1:
            rospy.loginfo("step 3: rod_climbing_mechanism_motion is in process")
            os.system('rosparam set /renov_up_level/last_motion_phase_over_flag 0')

            rodclimbing_tracking_error=rodclimbing_tracking_error-0.1

            rodclimbing_tolerance_tracking_error=0.1
            if abs(rodclimbing_tracking_error)<rodclimbing_tolerance_tracking_error:
                rospy.loginfo("step 3: rod_climbing_mechanism_motion is closed")
                os.system('rosparam set /renov_up_level/current_motion_phase_over_flag 1')
                break
        rate.sleep()

def motion_state_current2last():
    current_motion_phase_over_flag=rospy.get_param("/renov_up_level/current_motion_phase_over_flag")
    if current_motion_phase_over_flag==1:
        os.system("rosparam set /renov_up_level/last_motion_phase_over_flag 1")

def main():
    nodename="rodclimbing_mechanism_motion"
    rospy.init_node(nodename)
    ratet=1
    rate = rospy.Rate(ratet)

    target_rotation_angle=-2 # -math.pi/2-0.7
    target_distance=0.0
    rodclimb_mechanism_motion(target_rotation_angle,target_distance,rate)    
    # rodclimb_mechanism_motion_simulation(target_rotation_angle,target_distance,rate)

if __name__=="__main__":
    main()
