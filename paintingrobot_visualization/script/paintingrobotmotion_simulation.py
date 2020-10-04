#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from std_msgs.msg import String,Float64,Bool
import time
import numpy
import os,sys
from sensor_msgs.msg import JointState
import yaml
import numpy as np
import numpy.matlib
import json
from scipy.io import loadmat

class aubo_state():
    def __init__(self):
        self.pub_state_ = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.aubo_joints_value=[0.0,0.0,0.0,0.0,0.0,0.0]

    def Init_node(self):
        rospy.init_node("aubo_jointvalue_viewpoints_generation")
    def deg_to_rad(self,tuplelist):
        dd = []
        for i in tuplelist:
            dd.append(i * pi / 180)
        return tuple(dd)
    def rad_to_degree(self,tuplelist):
        dd=[]
        for i in tuplelist:
            dd.append(i*180/math.pi)
        return dd
    def pub_state(self,robot_state):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = 'map'
        js.name = ["base_joint1", "base_joint2","mobilebase_joint","rodclimbing_joint1",'rodclimbing_joint2',"shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"]
        js.position = [robot_state[0],robot_state[1],robot_state[2],robot_state[3],robot_state[4],robot_state[5],robot_state[6],robot_state[7],robot_state[8],robot_state[9],robot_state[10]]
        js.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        js.effort = [0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub_state_.publish(js)


"the python file is to visualize the motion of mobile platform and manipulator in simulation environment"
def main():
    Aub=aubo_state()
    Aub.Init_node()
    ratet=1
    rate = rospy.Rate(ratet)
    temp1=[0.0, 0.0, 0.0, 0.0, 0.0]
    temp2=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    temp3=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    coverage_json_path=rospy.get_param("coverage_json_path")
    with open(coverage_json_path,'r') as f:
        planning_source_dict=json.load(f)
    plane_num_count=0
    mobile_base_point_count=0
    climb_base_count_num=0
    offset_distance=0.06
    while not rospy.is_shutdown():
        mobiledata=planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]["mobile_data_num_"+str(mobile_base_point_count)]     
        robot_q=[mobiledata[0], -mobiledata[1], mobiledata[5]]+temp2
        Aub.pub_state(robot_q)
        rate.sleep()

        while not rospy.is_shutdown():

            climb_data=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]["climb_num_"+ str(climb_base_count_num)]
            robot_q=[mobiledata[0], -mobiledata[1], mobiledata[5], climb_data[0]+offset_distance, climb_data[1]]+temp3
            Aub.pub_state(robot_q)
            print("robot_q is: ", robot_q)
            rate.sleep()

            aubo_q_list=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_aubojoint_num_"+str(mobile_base_point_count)]["aubo_planning_voxel_num_"+ str(climb_base_count_num)]
            for m in range(len(aubo_q_list)):
                aubo_q=aubo_q_list["aubo_data_num_"+str(m)]
                robot_q=[mobiledata[0], -mobiledata[1], mobiledata[5],climb_data[0]+offset_distance, climb_data[1]]+aubo_q
                Aub.pub_state(robot_q)
                print("robot_q is: ", robot_q)
                rate.sleep()
            
            climb_base_count_num+=1
            if climb_base_count_num>=len(planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]):
                mobile_base_point_count+=1
                climb_base_count_num=0
                break
        if mobile_base_point_count >= len(planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]):
            plane_num_count+=1 
            mobile_base_point_count=0            
        if plane_num_count>=len(planning_source_dict):
            plane_num_count=0
            mobile_base_point_count=0
            climb_base_count_num=0

            # break


if __name__ == '__main__':
    main()