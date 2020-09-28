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
import tf


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

    def renovation_planning_source_dict_generation(self):
        coverage_json_path=rospy.get_param("coverage_json_path")
        with open(coverage_json_path,'r') as f:
            planning_source_dict=json.load(f)
        return planning_source_dict
        
    def renovationrobot_motion(self,planning_source_dict,rate):
        plane_num_count=0
        mobile_base_point_count=0
        climb_base_count_num=0
        list1=[]
        list2=[]
        while not rospy.is_shutdown():
            # "some selection for mobile base positions"
            # if plane_num_count==0 and mobile_base_point_count==0:
            #     mobile_base_point_count=mobile_base_point_count+1
            if plane_num_count==0 and mobile_base_point_count==0:
                mobile_base_point_count=mobile_base_point_count+1
            # if plane_num_count==1 and mobile_base_point_count==5:
            #     break
            
            mobiledata=planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]["mobile_data_num_"+str(mobile_base_point_count)]            
            "some adjustment of mobile base positions"
            list1.append(mobiledata)
            # if plane_num_count==0 and mobile_base_point_count==1:
            #     mobiledata[1]=mobiledata[1]+0.1
            # if plane_num_count==0 and mobile_base_point_count==2:
            #     mobiledata[1]=mobiledata[1]+0.15
    
            "executing mobile platform motion"
            time1=time.time()
            renovation_mobileplatform=mobile_platform()
            renovation_mobileplatform.mobile_platform_motion(mobiledata,rate)
            # renovation_mobileplatform.mobile_platform_motion_simulation(mobiledata,rate)
            time2=time.time()
            delta_time1=time2-time1
            self.time1_pub.publish(delta_time1)
            list2.append(delta_time1)

            "executing rod mechanism holding operation when mobile platform motion is over"
            # time1=time.time()
            
            # target_standbar_displacement=holding_rod_mechanism_target_standbar_displacement_computation()
            # target_standbar_displacement=0.12
            # rod_mechanism_holding(target_standbar_displacement,rate)

            # rod_mechanism_holding_simulation(target_standbar_displacement,rate)

            # time2=time.time()
            # delta_time2=time2-time1
            # self.time2_pub.publish(delta_time2)

            while not rospy.is_shutdown():
                rospy.loginfo("execute the %sth plane"%str(plane_num_count+1))
                rospy.loginfo("execute the %sth mobile base point"%str(mobile_base_point_count+1))
                rospy.loginfo("execute the %sth climb base point"%str(climb_base_count_num+1))

                climb_data=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]["climb_num_"+ str(climb_base_count_num)]
                climb_distance=climb_data[0]
                climb_rotation_angle=climb_data[1]
                print("climb distance is:",climb_distance)
                print("climb rotation angle is",climb_rotation_angle)

                "executing climbing motion of rod climbing mechanism when holding operation is over"
                time1=time.time()
                rodclimb_mechanism_motion(climb_rotation_angle,climb_distance,rate)
                # rodclimb_mechanism_motion_simulation(climb_rotation_angle,climb_distance,rate)
                time2=time.time()
                delta_time3=time2-time1
                self.time3_pub.publish(delta_time3)

                "exectuing painting operation of manipulator when climbing operation is over"
                aubo_q_list=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_aubo_num_"+str(mobile_base_point_count)]["aubo_planning_voxel_num_"+ str(climb_base_count_num)]
                # for i in range(len(aubo_q_list)):
                #     list1=aubo_q_list["aubo_data_num_"+str(i)]
                #     print(list1)
                print("the number of aubo_q is:",len(aubo_q_list))

                time1=time.time()
                aubo5=Renovation_operation()
                aubo5.aubo_motion1(aubo_q_list,rate)
                # aubo5.manipulator_motion_simulation(aubo_q_list,rate)
                time2=time.time()
                delta_time4=time2-time1
                self.time4_pub.publish(delta_time4)

                "termination condition: all climbing base positions are conversed"                
                climb_base_count_num+=1
                # if climb_base_count_num>=1:
                if climb_base_count_num>=len(planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]):
                    mobile_base_point_count+=1
                    climb_base_count_num=0
                    os.system('rosparam set /renov_up_level/one_mobilebase_operation_over_flag 1')
                    break
                # break
            "executing jackup motion of jackup mechanism when operation on one mobile base is over"
            time1=time.time()
            jackup_mechanism_homing(rate)
            # jackup_mechanism_homing_simulation(rate)
            time2=time.time()
            delta_time5=time2-time1
            self.time5_pub.publish(delta_time5)

            "exit condition: all renovation surface is operated"
            if mobile_base_point_count >= len(planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]):
                plane_num_count+=1 
                mobile_base_point_count=0
                # rospy.loginfo("plane_num_count is: %s",str(plane_num_count))
            # if plane_num_count==1 and mobile_base_point_count==3:
            #     break
            if plane_num_count>=len(planning_source_dict):
                rospy.loginfo("painting operation of whole room is over")
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
    CUHK_renovationrobot.renovationrobot_motion(planning_source_dict,rate)
    end_time=time.time()
    print("the total time is %s"%(str(end_time-begin_time)))

if __name__ == '__main__':
    main()

    
