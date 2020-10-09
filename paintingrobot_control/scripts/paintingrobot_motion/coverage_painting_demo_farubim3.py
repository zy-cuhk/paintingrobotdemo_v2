#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from std_msgs.msg import String,Float64,Bool
import time
import commands
import os
import sys
from geometry_msgs.msg import PoseStamped,Quaternion
from visualization_msgs.msg import Marker, MarkerArray


import tf

import json
from mobileplatform_motion import *
from jackup_mechanism_homing import *
from jackup_mechanism_holding import *
from jackup_mechanism_motion import *
from aubo_motion import *
from paintingrobot_onlinepathposition_visualization import *

class RenovationRobot():
    def __init__(self):
        self.time1_pub = rospy.Publisher('/renov_up_level/mobileplatform_motion_time', Float64, queue_size=1)
        self.time2_pub = rospy.Publisher('/renov_up_level/jackupmechanism_holding_time', Float64, queue_size=1)
        self.time3_pub = rospy.Publisher('/renov_up_level/jackupmechanism_motion_time', Float64, queue_size=1)
        self.time4_pub = rospy.Publisher('/renov_up_level/manipulator_motion_time', Float64, queue_size=1)
        self.time5_pub = rospy.Publisher('/renov_up_level/jackupmechanism_homing_time', Float64, queue_size=1)
        self.pub_state = rospy.Publisher('/visualization_marker1', Marker, queue_size=10)


    def renovation_planning_source_dict_generation(self):
        coverage_json_path=rospy.get_param("coverage_json_path")
        with open(coverage_json_path,'r') as f:
            planning_source_dict=json.load(f)
        return planning_source_dict
    

    def renovationrobot_motion_and_visualization(self,planning_source_dict,rate):

        plane_num_count=3
        mobile_base_point_count=0
        climb_base_count_num=0

        list1=[]
        list2=[]
        while not rospy.is_shutdown():

            "executing mobile platform motion"            
            mobiledata=planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]["mobile_data_num_"+str(mobile_base_point_count)]            
            list1.append(mobiledata)    
            time1=time.time()
            renovation_mobileplatform=mobile_platform()
            # renovation_mobileplatform.mobile_platform_motion(mobiledata,rate)
            # renovation_mobileplatform.mobile_platform_motion_simulation(mobiledata,rate)
            time2=time.time()
            delta_time1=time2-time1
            self.time1_pub.publish(delta_time1)
            list2.append(delta_time1)

            # "executing rod mechanism holding operation when mobile platform motion is over"
            # time1=time.time()
            # target_standbar_displacement=holding_rod_mechanism_target_standbar_displacement_computation()
            # target_standbar_displacement=0.12
            # rod_mechanism_holding(target_standbar_displacement,rate)
            # rod_mechanism_holding_simulation(target_standbar_displacement,rate)
            # time2=time.time()
            # delta_time2=time2-time1
            # self.time2_pub.publish(delta_time2)

            rospy.loginfo("-----------------------------------------------------------------")                    
            rospy.loginfo("execute the %sth plane"%str(plane_num_count+1))
            rospy.loginfo("execute the %sth mobile base point"%str(mobile_base_point_count+1))            
            rospy.loginfo("-----------------------------------------------------------------")

            while not rospy.is_shutdown():
                if plane_num_count!=1 and plane_num_count!=3:
                    
                    rospy.loginfo("-----------------------------------------------------------------")
                    rospy.loginfo("execute the %sth climb base point"%str(climb_base_count_num+1))
                    rospy.loginfo("-----------------------------------------------------------------")

                    "visualize online aubo robot arm paths"
                    visualization_num=1
                    aubo_p_list=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_aubocartesian_num_"+str(mobile_base_point_count)]["aubo_planning_voxel_num_"+str(climb_base_count_num)]
                    manipulatorendeffector_targetpose_onecell=np.zeros((len(aubo_p_list),3))
                    for i in range(len(aubo_p_list)):
                        manipulatorendeffector_targetpose_onecell[i][0] = aubo_p_list["cartesianposition_num_"+str(i)][0]
                        manipulatorendeffector_targetpose_onecell[i][1] = aubo_p_list["cartesianposition_num_"+str(i)][1]
                        manipulatorendeffector_targetpose_onecell[i][2] = aubo_p_list["cartesianposition_num_"+str(i)][2]
                    visualization_num, manipulator_path=path_visualization(visualization_num,manipulatorendeffector_targetpose_onecell)
                    visualization_num=visualization_num+1
                    self.pub_state.publish(manipulator_path)
                    "visualize manipulator base coordinate frame"
                    rodclimbing_robot_targetjoints=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_manipulatorbase_num_"+str(mobile_base_point_count)]["manipulatorbase_num_"+str(climb_base_count_num)]
                    visualization_num, point, x_axis, y_axis, z_axis= coordinate_frame_visualization(visualization_num, rodclimbing_robot_targetjoints)
                    visualization_num=visualization_num+1
                    self.pub_state.publish(point)
                    self.pub_state.publish(x_axis)
                    self.pub_state.publish(y_axis)
                    self.pub_state.publish(z_axis)


                    "executing climbing motion of rod climbing mechanism when holding operation is over"
                    climb_data=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]["climb_num_"+ str(climb_base_count_num)]
                    climb_distance=climb_data[0]
                    if climb_distance<-0.10:
                        climb_distance+=0.08
                    climb_rotation_angle=climb_data[1]
                    # print("climb distance is:",climb_distance)
                    # print("climb rotation angle is",climb_rotation_angle)
                    time1=time.time()
                    rodclimb_mechanism_motion(climb_rotation_angle,climb_distance,rate)
                    # rodclimb_mechanism_motion_simulation(climb_rotation_angle,climb_distance,rate)
                    time2=time.time()
                    delta_time3=time2-time1
                    self.time3_pub.publish(delta_time3)

                    "exectuing painting operation of manipulator when climbing operation is over"
                    aubo_q_list=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_aubojoint_num_"+str(mobile_base_point_count)]["aubo_planning_voxel_num_"+ str(climb_base_count_num)]
                    # print("the number of aubo_q is:",len(aubo_q_list))

                    time1=time.time()
                    aubo5=Renovation_operation()
                    rate=rospy.Rate(5)
                    aubo5.aubo_motion1(aubo_q_list,climb_base_count_num,rate)
                    # aubo5.manipulator_motion_simulation(aubo_q_list,rate)
                    time2=time.time()
                    delta_time4=time2-time1
                    self.time4_pub.publish(delta_time4)

                    "termination condition: all climbing base positions are conversed"                
                    climb_base_count_num+=1
                    # print("climb_base_count_num is: ",climb_base_count_num)
                    # print("the total climbing base number is: ", len(planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]))
                    # print("-------------------------------------------------")
                    if climb_base_count_num>=len(planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]):
                        mobile_base_point_count+=1
                        climb_base_count_num=0
                        os.system('rosparam set /renov_up_level/one_mobilebase_operation_over_flag 1')
                        break
                    # break
                else:
                    mobile_base_point_count+=1
                    break

            "executing climgbing mechanism homing operation when manipulator operation on one mobile abse is over"
            time1=time.time()
            rodclimb_mechanism_motion(0.0,0.0,rate)
            # rodclimb_mechanism_motion_simulation(0.0,0.0,rate)
            time2=time.time()
            delta_time3=time2-time1
            self.time3_pub.publish(delta_time3)

            # "executing jackup motion of jackup mechanism when operation on one mobile base is over"
            # time1=time.time()
            # jackup_mechanism_homing(rate)
            # jackup_mechanism_homing_simulation(rate)
            # time2=time.time()
            # delta_time5=time2-time1
            # self.time5_pub.publish(delta_time5)

            "exit condition: all renovation surface is operated"
            if mobile_base_point_count >= len(planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]):
                mobile_base_point_count=0
                plane_num_count+=1

            print("plane_num_count is: ", plane_num_count)
            if plane_num_count>=6:
                rospy.loginfo("painting operation of the first part is over")
                break
            # break
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
    CUHK_renovationrobot.renovationrobot_motion_and_visualization(planning_source_dict,rate)
    end_time=time.time()
    print("the total time is %s"%(str(end_time-begin_time)))

if __name__ == '__main__':
    main()

    
