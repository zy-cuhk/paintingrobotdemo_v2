#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, os
from math import *
import numpy as np
import numpy.matlib
import scipy.io as io
import time
from collections import defaultdict
import json 

from robotic_functions.aubo_kinematics import *
from manipulator_catersian_path_tspsolver import *

class multidict(dict):
    def __getitem__(self,item):
        try: 
            return dict.__getitem__(self,item)
        except KeyError: 
            value = self[item]=type(self)()
            return value


def manipulator_T_computation(paint_T, paintinggun_T):
    paintinggun_T_rot=paintinggun_T[0:3,0:3]
    paintinggun_T_tran=paintinggun_T[0:3,3]

    inv_paintinggun_T_rot=paintinggun_T_rot.T
    inv_paintinggun_T_tran=-np.dot(inv_paintinggun_T_rot, paintinggun_T_tran)

    inv_paintinggun_T = np.matlib.identity(4, dtype=float)
    inv_paintinggun_T[0:3, 0:3] = inv_paintinggun_T_rot
    for i in range(3):
        inv_paintinggun_T[i, 3] = inv_paintinggun_T_tran[i]
    
    manipulator_T=np.dot(paint_T,inv_paintinggun_T)
    # print("manipulator_T is:",manipulator_T)

    manipulator_T1=manipulator_T.tolist()
    manipulator_T_list=[]
    for i in range(len(manipulator_T1)):
        for j in range(len(manipulator_T1[0])):
            manipulator_T_list.append(manipulator_T1[i][j])
            
    return manipulator_T_list

def sample_climbing_joints(renovation_mobilebase_position_onecell):
    "obtain mobile platform position and some kinematic parameters"
    mobilebase_position=renovation_mobilebase_position_onecell
    parameterx=0.430725381079
    parametery=-0.00033063639818
    theta_z=renovation_mobilebase_position_onecell[5]
    deltax=parameterx*cos(theta_z)-parametery*sin(theta_z)
    deltay=parameterx*sin(theta_z)+parametery*cos(theta_z)

    "sampling manipulator base positions"
    candidate_manipulatorbase_num=2
    candidate_manipulatorbase_position=np.zeros((candidate_manipulatorbase_num,6))
    for i in range(candidate_manipulatorbase_num):
        candidate_manipulatorbase_position[i][0]=renovation_mobilebase_position_onecell[0]+deltax
        candidate_manipulatorbase_position[i][1]=renovation_mobilebase_position_onecell[1]+deltay
        candidate_manipulatorbase_position[i][2]=1.38-0.13*i  # the sampled point is: 1.38; 1.28 and 1.18
        candidate_manipulatorbase_position[i][3]=0
        candidate_manipulatorbase_position[i][4]=0
        candidate_manipulatorbase_position[i][5]=theta_z

    return candidate_manipulatorbase_position

def obtain_waypaths_insideclimbingworkspace(candidate_manipulatorbase_position,renovation_waypaths_onecell,paintinggun_T):
    rmax=1.7
    climbingjoints_coverage_number=np.zeros(len(candidate_manipulatorbase_position))
    cartersianwaypaths_incandidateclimbingjoints=defaultdict(defaultdict)
    cartersianwaypaths_outof_candidateclimbingjoints=defaultdict(defaultdict)

    for candidate_num in range(len(candidate_manipulatorbase_position)):
        coverage_waypaths_num=0
        uncoverage_waypaths_num=0
        for k in range(len(renovation_waypaths_onecell)):
            "obtain cartesian waypaths inside manipulator workspace" 
            delat_vector1=renovation_waypaths_onecell[k][0:3]-candidate_manipulatorbase_position[candidate_num][0:3]
            distance1=sqrt(delat_vector1[0]**2+delat_vector1[1]**2+delat_vector1[2]**2)
            delat_vector2=renovation_waypaths_onecell[k][3:6]-candidate_manipulatorbase_position[candidate_num][0:3]
            distance2=sqrt(delat_vector2[0]**2+delat_vector2[1]**2+delat_vector2[2]**2)
            if distance1<rmax and distance2<rmax:
                "remove waypaths without satisfied joint space solutions"
                flag_point=[0,0]
                for m in range(2):
                    xyz0=renovation_waypaths_onecell[k][3*m:3*m+3]-candidate_manipulatorbase_position[candidate_num][0:3]
                    manipulator_base_orientation=candidate_manipulatorbase_position[candidate_num][3:6]
                    rot=mat_computation.rpy2r(manipulator_base_orientation).T
                    xyz=np.dot(rot, xyz0).T

                    rpy1=[0,pi/2,0]
                    rpy2=[0,pi/2,pi]
                    paint_T1 = mat_computation.Tmat(xyz,rpy1)
                    manipulator_T_list1 = manipulator_T_computation(paint_T1, paintinggun_T)
                    flag1, q_dict1 = aubo_computation.GetInverseResult_withoutref(manipulator_T_list1)
                    paint_T2 = mat_computation.Tmat(xyz,rpy2)
                    manipulator_T_list2 = manipulator_T_computation(paint_T2, paintinggun_T)
                    flag2, q_dict2 = aubo_computation.GetInverseResult_withoutref(manipulator_T_list2)

                    if flag1==True or flag2==True:
                        flag_point[m] = 1

                if flag_point[0]==1 and flag_point[1]==1:
                    cartersianwaypaths_incandidateclimbingjoints[candidate_num][coverage_waypaths_num]=renovation_waypaths_onecell[k][0:6]  
                    coverage_waypaths_num+=1
                else:
                    cartersianwaypaths_outof_candidateclimbingjoints[candidate_num][uncoverage_waypaths_num]=renovation_waypaths_onecell[k][0:6]  
                    uncoverage_waypaths_num+=1
            else:
                cartersianwaypaths_outof_candidateclimbingjoints[candidate_num][uncoverage_waypaths_num]=renovation_waypaths_onecell[k][0:6]  
                uncoverage_waypaths_num+=1
        climbingjoints_coverage_number[candidate_num]=coverage_waypaths_num
        print("the position number and corresponding coverage and uncoverage state is:",candidate_num,coverage_waypaths_num,uncoverage_waypaths_num)

    return climbingjoints_coverage_number, cartersianwaypaths_incandidateclimbingjoints, cartersianwaypaths_outof_candidateclimbingjoints


def obtain_waypaths_insideclimbingworkspace1(candidate_manipulatorbase_position,renovation_waypaths_onecell,paintinggun_T):
    rmax=1.7
    climbingjoints_coverage_number=np.zeros(len(candidate_manipulatorbase_position))
    cartersianwaypaths_incandidateclimbingjoints=defaultdict(defaultdict)
    cartersianwaypaths_outof_candidateclimbingjoints=defaultdict(defaultdict)

    for candidate_num in range(len(candidate_manipulatorbase_position)):
        coverage_waypaths_num=0
        uncoverage_waypaths_num=0
        for k in range(len(renovation_waypaths_onecell)):
            "obtain cartesian waypaths inside manipulator workspace" 
            delat_vector1=renovation_waypaths_onecell[k][0:3]-candidate_manipulatorbase_position[candidate_num][0:3]
            distance1=sqrt(delat_vector1[0]**2+delat_vector1[1]**2+delat_vector1[2]**2)
            delat_vector2=renovation_waypaths_onecell[k][3:6]-candidate_manipulatorbase_position[candidate_num][0:3]
            distance2=sqrt(delat_vector2[0]**2+delat_vector2[1]**2+delat_vector2[2]**2)
            if distance1<rmax and distance2<rmax:
                "remove waypaths without satisfied joint space solutions"
                flag_point=[0,0]
                for m in range(2):
                    xyz0=renovation_waypaths_onecell[k][3*m:3*m+3]-candidate_manipulatorbase_position[candidate_num][0:3]
                    manipulator_base_orientation=candidate_manipulatorbase_position[candidate_num][3:6]
                    rot=mat_computation.rpy2r(manipulator_base_orientation).T
                    xyz=np.dot(rot, xyz0).T

                    rpy1=[0,pi/2,0]
                    rpy2=[0,pi/2,pi]
                    paint_T1 = mat_computation.Tmat(xyz,rpy1)
                    manipulator_T_list1 = manipulator_T_computation(paint_T1, paintinggun_T)
                    flag1, q_dict1 = aubo_computation.GetInverseResult_withoutref(manipulator_T_list1)
                    paint_T2 = mat_computation.Tmat(xyz,rpy2)
                    manipulator_T_list2 = manipulator_T_computation(paint_T2, paintinggun_T)
                    flag2, q_dict2 = aubo_computation.GetInverseResult_withoutref(manipulator_T_list2)

                    if flag1==True or flag2==True:
                        flag_point[m] = 1

                    if candidate_num==0 and renovation_waypaths_onecell[k][2]<=1.3 and len(candidate_manipulatorbase_position)==2: 
                        flag_point[m] = 0

                if flag_point[0]==1 and flag_point[1]==1:
                    cartersianwaypaths_incandidateclimbingjoints[candidate_num][coverage_waypaths_num]=renovation_waypaths_onecell[k][0:6]  
                    coverage_waypaths_num+=1
                else:
                    cartersianwaypaths_outof_candidateclimbingjoints[candidate_num][uncoverage_waypaths_num]=renovation_waypaths_onecell[k][0:6]  
                    uncoverage_waypaths_num+=1
            else:
                cartersianwaypaths_outof_candidateclimbingjoints[candidate_num][uncoverage_waypaths_num]=renovation_waypaths_onecell[k][0:6]  
                uncoverage_waypaths_num+=1
        climbingjoints_coverage_number[candidate_num]=coverage_waypaths_num
        print("the position number and corresponding coverage and uncoverage state is:",candidate_num,coverage_waypaths_num,uncoverage_waypaths_num)

    return climbingjoints_coverage_number, cartersianwaypaths_incandidateclimbingjoints, cartersianwaypaths_outof_candidateclimbingjoints


def select_climbingjoints(manipulatorbaseheight_now, candidate_manipulatorbase_position,climbingjoints_coverage_number, cartersianwaypaths_incandidateclimbingjoints,cartersianwaypaths_outof_candidateclimbingjoints):
    "obtain the selected climbing position and waypaths contained inside the workspace of selected climbing position"

    sampled_motiondistance=np.zeros(len(candidate_manipulatorbase_position))
    for i in range(len(candidate_manipulatorbase_position)):
        sampled_manipulatorbase_height=candidate_manipulatorbase_position[i][2]
        sampled_motiondistance[i]=abs(sampled_manipulatorbase_height-manipulatorbaseheight_now)
    # print("the sampled motion distance is: ", sampled_motiondistance)

    sampled_coveragepaths_number=np.zeros(len(candidate_manipulatorbase_position))
    for i in range(len(candidate_manipulatorbase_position)):
        sampled_coveragepaths_number[i]=climbingjoints_coverage_number[i]

    weight=10
    sampled_coveragepaths_evaluator=np.zeros(len(candidate_manipulatorbase_position))
    for i in range(len(candidate_manipulatorbase_position)):
        sampled_coveragepaths_evaluator[i]=sampled_coveragepaths_number[i]/(1+weight*sampled_motiondistance[i])
    print("the sampled coverage paths evaluator is: ", sampled_coveragepaths_evaluator)

    max_coverage_paths_number=0
    for i in range(len(candidate_manipulatorbase_position)):
        if sampled_coveragepaths_evaluator[i]>=max_coverage_paths_number:
            max_coverage_paths_number=sampled_coveragepaths_evaluator[i]
            max_coverage_index=i
            print("max coverage paths evaluator is:",max_coverage_paths_number)
            print("max_coverage_index is:",max_coverage_index)

    selected_manipulatorbase_position=candidate_manipulatorbase_position[max_coverage_index]
    selected_cartersian_waypaths=[]
    for i in range(len(cartersianwaypaths_incandidateclimbingjoints[max_coverage_index])):
        list1=cartersianwaypaths_incandidateclimbingjoints[max_coverage_index][i]
        selected_cartersian_waypaths.append(list1)
    print("the selected manipualator position height is:",selected_manipulatorbase_position[2])

    "obtain remaining unselected manipulator positions"
    new_candidate_manipulatorbase_position=[]
    for i in range(len(candidate_manipulatorbase_position)):
        if i!=max_coverage_index:
            new_candidate_manipulatorbase_position.append(candidate_manipulatorbase_position[i])
    for i in range(len(new_candidate_manipulatorbase_position)):
        print("the unselected manipulator position height is:",new_candidate_manipulatorbase_position[i][2])

    "obtain remaining uncovered manipulator painting paths"
    renovation_waypaths_onecell=[]

    for i in range(len(cartersianwaypaths_outof_candidateclimbingjoints[max_coverage_index])):
        list1=cartersianwaypaths_outof_candidateclimbingjoints[max_coverage_index][i]
        renovation_waypaths_onecell.append(list1)

    return selected_manipulatorbase_position, selected_cartersian_waypaths, new_candidate_manipulatorbase_position,renovation_waypaths_onecell

def manipulator_jointspace_tspsolver(selected_manipulatorbase_position,scheduled_selected_strokes_dict,paintinggun_T):
    "step 1: obtain scheduled waypoints list based on the selected strokes"
    scheduled_selected_waypoints_list=[]
    manipulator_strokes_number=0
    for i in range(len(scheduled_selected_strokes_dict)):
        for j in range(len(scheduled_selected_strokes_dict[i])):
            manipulator_strokes_number+=1
            scheduled_selected_waypoints_list.append(scheduled_selected_strokes_dict[i][j][0:3])
            if j==len(scheduled_selected_strokes_dict[i])-1:
                scheduled_selected_waypoints_list.append(scheduled_selected_strokes_dict[i][j][3:6])    
    "step 2: obtain the joint space solutions for each waypoint"
    waypoints_candidate_joints_dict=defaultdict(defaultdict)
    scheduled_selectedjoints_dict=defaultdict(defaultdict)
    aubo_q_ref=np.array([0.0,-0.24435,2.7524,-0.3,-1.4835,-1.57])
    
    for i in range(len(scheduled_selected_waypoints_list)):
    # for i in range(1):
        onewaypoint_candidate_joints_dict=defaultdict(defaultdict)

        xyz0=scheduled_selected_waypoints_list[i][0:3]-selected_manipulatorbase_position[0:3]
        manipulator_base_orientation=selected_manipulatorbase_position[3:6]
        rot=mat_computation.rpy2r(manipulator_base_orientation).T
        xyz=np.dot(rot, xyz0).T

        rpy1=[0,pi/2,0]
        rpy2=[0,pi/2,pi]
        paint_T1 = mat_computation.Tmat(xyz,rpy1)
        manipulator_T_list1 = manipulator_T_computation(paint_T1, paintinggun_T)
        flag1, q_dict1 = aubo_computation.GetInverseResult_withoutref(manipulator_T_list1)
        paint_T2 = mat_computation.Tmat(xyz,rpy2)
        manipulator_T_list2 = manipulator_T_computation(paint_T2, paintinggun_T)
        flag2, q_dict2 = aubo_computation.GetInverseResult_withoutref(manipulator_T_list2)
        num=0
        for num1 in range(len(q_dict1)):
            if q_dict1[num1][0]<2.2:
                onewaypoint_candidate_joints_dict[num]=q_dict1[num1]
                waypoints_candidate_joints_dict[i][num]=q_dict1[num1]
                num+=1
        for num2 in range(len(q_dict2)):
            if q_dict2[num2][0]<2.2:
                onewaypoint_candidate_joints_dict[num]=q_dict2[num2]
                waypoints_candidate_joints_dict[i][num]=q_dict2[num2]
                num+=1    

        flag, selected_qlist= chooseIKonRefJoint(onewaypoint_candidate_joints_dict, aubo_q_ref)
        scheduled_selectedjoints_dict[i]=selected_qlist
        aubo_q_ref=selected_qlist


    # for i in range(len(scheduled_selectedjoints_dict)):
    #     print("the selected joints list is: ",scheduled_selectedjoints_dict[i])
    # print("the waypoints number is:",len(scheduled_selectedjoints_dict))
    return scheduled_selectedjoints_dict,scheduled_selected_waypoints_list

def chooseIKonRefJoint(q_sols, q_ref):
    sum_data = List_Frobenius_Norm(q_ref, q_sols[0])
    err = 0
    index1 = 0
    for i in range(len(q_sols)):
        err = List_Frobenius_Norm(q_ref, q_sols[i])
        if (err < sum_data):
            index1 = i
            sum_data = err
    
    q_choose = q_sols[index1]
    return True, q_choose

def List_Frobenius_Norm(list_a, list_b):
    new_list = []
    w=1
    for i in range(len(list_a)):
        if i==5:
            new_list.append(w*abs(list_a[i] - list_b[i]) ** 2)
        else:
            new_list.append(abs(list_a[i] - list_b[i]) ** 2)
    return sqrt(sum_list(new_list))

def sum_list(list_data):
    sum_data = 0
    for i in range(len(list_data)):
        sum_data += list_data[i]
    return sum_data


if __name__ == "__main__":
    
    mat_path="/data/ros/renov_robot_ws/src/paintingrobotdemo_v2/paintingrobotdemo_data/scan_guangtian/data/second_scan_data3.mat"
    json_path="/data/ros/renov_robot_ws/src/paintingrobotdemo_v2/paintingrobotdemo_data/scan_guangtian/data/coverageplanningresults_dict.json"

    "input: renovation_cells_mobilebase_positions and renovation_cells_waypaths" 
    data = io.loadmat(mat_path)
    renovation_cells_mobilebase_positions=data['renovation_cells_mobilebase_positions1']
    print("the best is:", renovation_cells_mobilebase_positions[0][0][0])

    coverageplanningresults_dict=multidict()
    for i in range(len(renovation_cells_mobilebase_positions[0])):
        for j in range(len(renovation_cells_mobilebase_positions[0][i])):
            mobileplatform_targetjoints=renovation_cells_mobilebase_positions[0][i][j].tolist()
            coverageplanningresults_dict["plane_num_"+str(i)]["moible_way_num_"+str(i)]["mobile_data_num_"+str(j)]=mobileplatform_targetjoints

    with open(json_path,'w') as f:
        json.dump(coverageplanningresults_dict, f, sort_keys=True, indent=4, separators=(', ', ': '), ensure_ascii=False)




    
