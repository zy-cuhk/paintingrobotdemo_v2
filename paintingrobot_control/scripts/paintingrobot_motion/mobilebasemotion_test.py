#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from std_msgs.msg import String,Float64,Bool
import time
import commands
import os
import sys
from geometry_msgs.msg import PoseStamped,Quaternion, PoseArray, Pose
import tf

import json
from mobileplatform_motion import *
from jackup_mechanism_homing import *
from jackup_mechanism_holding import *
from jackup_mechanism_motion import *
from aubo_motion import *

class RenovationRobot():
    def __init__(self):
        self.time1_pub = rospy.Publisher('/renov_up_level/mobileplatform_motion_time', Float64, queue_size=1)
        self.time2_pub = rospy.Publisher('/renov_up_level/jackupmechanism_holding_time', Float64, queue_size=1)
        self.time3_pub = rospy.Publisher('/renov_up_level/jackupmechanism_motion_time', Float64, queue_size=1)
        self.time4_pub = rospy.Publisher('/renov_up_level/manipulator_motion_time', Float64, queue_size=1)
        self.time5_pub = rospy.Publisher('/renov_up_level/jackupmechanism_homing_time', Float64, queue_size=1)
        self.mobilebasepositions_pub = rospy.Publisher("/renov_up_level/mobilebase_positions",PoseArray, queue_size=1)

    def renovation_planning_source_dict_generation(self):
        coverage_json_path=rospy.get_param("coverage_json_path")
        with open(coverage_json_path,'r') as f:
            planning_source_dict=json.load(f)
        return planning_source_dict
    def renovationrobot_mobilebasepositions_pub(self, planning_source_dict,rate):
        plane_num_count=0
        mobile_base_point_count=0
        p_list = PoseArray()
        p_list.header.frame_id ="map"
        p_list.header.stamp = rospy.Time.now()

        points_num=0
        while not rospy.is_shutdown():
            mobiledata=planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]["mobile_data_num_"+str(mobile_base_point_count)]            
            
            "add into the poses list"
            p = Pose()
            p.position.x=mobiledata[0]
            p.position.y=mobiledata[1]
            p.position.z=mobiledata[2]
            q = tf.transformations.quaternion_from_euler(mobiledata[3],mobiledata[4],mobiledata[5])
            p.orientation = Quaternion(*q)

            p_list.poses.append(p)
            points_num+=1
            print("points number is: ", points_num)
            print("plane num is: ", plane_num_count)
            print("mobile base num is: ", mobile_base_point_count)
            print("mobile base pose is:", mobiledata)
            "exit condition"
            mobile_base_point_count+=1
            
            if mobile_base_point_count >= len(planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]):
                plane_num_count+=1 
                mobile_base_point_count=0
            if plane_num_count>=len(planning_source_dict):
                rospy.loginfo("painting operation of whole room is over")
                break
        while not rospy.is_shutdown():
            self.mobilebasepositions_pub.publish(p_list)
            rate.sleep()
    def renovationrobot_motion(self,planning_source_dict,rate):
        plane_num_count=0
        mobile_base_point_count=0
        climb_base_count_num=0
        list1=[]
        list2=[]
        while not rospy.is_shutdown():
            # "some selection for mobile base positions"
            # if plane_num_count==1 and mobile_base_point_count==0:
            #     plane_num_count=plane_num_count
            #     mobile_base_point_count=mobile_base_point_count+1
            
            if plane_num_count==1 and mobile_base_point_count==len(planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)])-1:
                plane_num_count=plane_num_count+1
                mobile_base_point_count=0

            if plane_num_count==2 and mobile_base_point_count==0:
                plane_num_count=plane_num_count+1
                mobile_base_point_count=mobile_base_point_count
            

            mobiledata=planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]["mobile_data_num_"+str(mobile_base_point_count)]            
            "some adjustment of mobile base positions"
            list1.append(mobiledata)
    
            "executing mobile platform motion"
            time1=time.time()
            renovation_mobileplatform=mobile_platform()
            renovation_mobileplatform.mobile_platform_motion(mobiledata,rate)
            time2=time.time()
            delta_time1=time2-time1
            self.time1_pub.publish(delta_time1)
            list2.append(delta_time1)

            rospy.loginfo("execute the %sth plane"%str(plane_num_count+1))
            rospy.loginfo("execute the %sth mobile base point"%str(mobile_base_point_count+1))

            "exit condition: all renovation surface is operated"
            mobile_base_point_count+=1
            if mobile_base_point_count >= len(planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]):
                plane_num_count+=1 
                mobile_base_point_count=0
            
            if plane_num_count>=len(planning_source_dict):
                rospy.loginfo("painting operation of whole room is over")
                break
            rate.sleep()

        print("list is:",list1)
        print("list is:",list2)

def main():
    begin_time=time.time()
    rospy.init_node("painting_opreating_node_with_bim_model")
    ratet=1
    rate = rospy.Rate(ratet)
    CUHK_renovationrobot=RenovationRobot()
    planning_source_dict=CUHK_renovationrobot.renovation_planning_source_dict_generation()
    # CUHK_renovationrobot.renovationrobot_mobilebasepositions_pub(planning_source_dict,rate)
    CUHK_renovationrobot.renovationrobot_motion(planning_source_dict,rate)
    end_time=time.time()
    print("the total time is %s"%(str(end_time-begin_time)))

if __name__ == '__main__':
    main()

    
