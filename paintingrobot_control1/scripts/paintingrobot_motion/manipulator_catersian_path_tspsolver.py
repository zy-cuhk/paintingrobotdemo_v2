#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, os
import scipy.io as io
from math import *
import numpy as np
from robotic_functions.aubo_kinematics import *
import numpy.matlib
from collections import defaultdict


def length(point1, point2):
    return sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 + (point1[2] - point2[2]) ** 2)

def connect_waypaths(unconnected_waypaths):
    "select one waypath"
    connected_waypaths=[]
    connected_waypaths.append(unconnected_waypaths[0][0:6])
    flag = np.zeros(len(unconnected_waypaths),dtype=int) 
    flag[0] = 1
    "left connect"
    index1 = 0
    while(1):
        index1_old=index1
        p1=connected_waypaths[index1][0:3]
        p2=connected_waypaths[index1][3:6]
        for i in range(len(unconnected_waypaths)):
            if flag[i]!=1:
                p3=unconnected_waypaths[i][0:3]
                p4=unconnected_waypaths[i][3:6]
                list1=unconnected_waypaths[i][0:6]
                if length(p1,p3)==0:
                    list1.reverse()
                    connected_waypaths.append(list1)
                    index1+=1
                    flag[i]=1
                if length(p1,p4)==0:
                    connected_waypaths.append(list1)
                    index1+=1
                    flag[i]=1
        if index1==index1_old:
            break
    connected_waypaths.reverse()
    "add right connect"
    index2=len(connected_waypaths)-1
    # print("index2 is:",connected_waypaths[index2][0:3])
    while(1):
        index2_old=index2
        # print("index2 is:",index2)

        p1=connected_waypaths[index2][0:3]
        p2=connected_waypaths[index2][3:6]
        for i in range(len(unconnected_waypaths)):
            if flag[i]!=1:
                p3=unconnected_waypaths[i][0:3]
                p4=unconnected_waypaths[i][3:6]
                list1=unconnected_waypaths[i][0:6]
                if length(p2,p3)==0:
                    connected_waypaths.append(list1)
                    index2+=1
                    flag[i]=1
                if length(p2,p4)==0:
                    list1.reverse()
                    connected_waypaths.append(list1)
                    index2+=1
                    flag[i]=1
        if index2==index2_old:
            break
    unconnected_waypaths1=[]
    for i in range(len(unconnected_waypaths)):
        if flag[i]!=1:
            unconnected_waypaths1.append(unconnected_waypaths[i][0:6])
    #print("connected_waypaths is:",connected_waypaths)
    return connected_waypaths, unconnected_waypaths1

"the painting strokes should be reschedured again based on Cartesian-space tsp solver"
def schedule_renovationstrokes(renovation_strokes_dict):
    "setup stroke end points"
    strokes_num=len(renovation_strokes_dict)
    points=np.zeros((2*strokes_num,3))
    for i in range(strokes_num):
        points[2*i,0:3]=renovation_strokes_dict[i][0][0:3]
        points[2*i+1,0:3]=renovation_strokes_dict[i][len(renovation_strokes_dict[i])-1][3:6]

    "schedule these strokes"
    cost = np.zeros(2*strokes_num)
    solution = np.zeros((2*strokes_num, 2*strokes_num))
    for point_index in range(2*strokes_num):
        index = point_index
        flag = [0]*2*strokes_num  
        flag[index] = 1
        i = 0
        cost[point_index]=0
        solution[point_index][0]=point_index
        while i < 2*strokes_num-1:
            dis_min = float("inf")
            for j in range(0, 2*strokes_num):
                if flag[j] == 0 and j != index:
                    dis = length(points[index], points[j])
                    if index % 2 == 1:
                        if j==index-1:
                            dis=0
                    else:
                        if j==index+1:
                            dis=0
                    if dis < dis_min:
                        dis_min = dis
                        index1 = j
            # print("the choosen point is:",index1)
            solution[point_index][i+1] = index1
            flag[index1] = 1
            index=index1
            i += 1
            cost[point_index]=cost[point_index]+dis_min
    cost=cost.tolist()
    min_point_index=cost.index(min(cost))
    solution1=np.zeros(2*strokes_num)
    solution1[:]=solution[min_point_index][:]
    
    "fill into new dict"
    scheduled_strokes_dict=defaultdict(defaultdict)
    for i in range(strokes_num):
        stroke_index=int(solution1[2*i]/2)
        if int(solution1[2*i]%2)==0:
            for j in range(len(renovation_strokes_dict[stroke_index])):
                scheduled_strokes_dict[i][j]=renovation_strokes_dict[stroke_index][j]        
        else:
            for j in range(len(renovation_strokes_dict[stroke_index])):           
                list1=renovation_strokes_dict[stroke_index][len(renovation_strokes_dict[stroke_index])-1-j]
                scheduled_strokes_dict[i][j]=[list1[3],list1[4],list1[5],list1[0],list1[1],list1[2]]
    return solution1, scheduled_strokes_dict

def manipulator_catersian_path_tspsolver(unconnected_waypaths):
    renovation_strokes_dict=defaultdict(defaultdict)

    strokes_index=0

    while(1):
        connected_waypaths, unconnected_waypaths=connect_waypaths(unconnected_waypaths)
        renovation_strokes_dict[strokes_index][0]=[connected_waypaths[0][0],connected_waypaths[0][1],connected_waypaths[0][2],connected_waypaths[len(connected_waypaths)-1][3],connected_waypaths[len(connected_waypaths)-1][4],connected_waypaths[len(connected_waypaths)-1][5]]
        # for i in range(len(connected_waypaths)):
        #     renovation_strokes_dict[strokes_index][i]=connected_waypaths[i][0:6]
        strokes_index+=1
        if len(unconnected_waypaths)==0:
            break
    
    solution, scheduled_strokes_dict=schedule_renovationstrokes(renovation_strokes_dict)
    # print("the solution is: ",solution)

    scheduled_strokes_dict1=defaultdict(defaultdict)
    if scheduled_strokes_dict[0][0][2]>=scheduled_strokes_dict[len(scheduled_strokes_dict)-1][0][2]:
        for i in range(len(scheduled_strokes_dict)):
            scheduled_strokes_dict1[i][0]=scheduled_strokes_dict[i][0]
    else:
        for i in range(len(renovation_strokes_dict)):
            scheduled_strokes_dict1[i][0]=scheduled_strokes_dict[len(scheduled_strokes_dict)-1-i][0]

    return scheduled_strokes_dict1



if __name__ == "__main__":
    mat_path="/data/ros/renov_robot_ws/src/paintingrobot_zy/paintingrobotdemo_v2/paintingrobotdemo_data/second_scan_2/data/second_scan_data3.mat"
    data = io.loadmat(mat_path)
    renovation_cells_waypaths=data['renovation_cells_waypaths']    
    unconnected_waypaths=renovation_cells_waypaths[0][0][0][0].tolist()
    scheduled_strokes_dict=manipulator_catersian_path_tspsolver(unconnected_waypaths)


    selected_waypoints_list=[]
    manipulator_strokes_number=0
    for i in range(len(scheduled_strokes_dict)):
        for j in range(len(scheduled_strokes_dict[i])):
            manipulator_strokes_number+=1
            selected_waypoints_list.append(scheduled_strokes_dict[i][j][0:3])
            if j==len(scheduled_strokes_dict[i])-1:
                selected_waypoints_list.append(scheduled_strokes_dict[i][j][3:6])
    # io.savemat('/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_underusing/paintingrobot_planning_tmech/scripts/data1.mat',{'selected_waypoints_list':selected_waypoints_list})  




    # print("manipulator paths number is: ", len(scheduled_strokes_dict))
    # manipulator_strokes_number=0
    # for i in range(len(scheduled_strokes_dict)):
    #     for j in range(len(scheduled_strokes_dict[i])):
    #         print(scheduled_strokes_dict[i][j])
    #         manipulator_strokes_number+=1
    # print("manipulator strokes number is: ",manipulator_strokes_number)

    # print("unconnected_waypaths is:",unconnected_waypaths[0][0:6])
    # renovation_strokes_dict=defaultdict(defaultdict)
    # time1=time.time()
    # strokes_index=0
    # while(1):
    #     connected_waypaths, unconnected_waypaths=connect_waypaths(unconnected_waypaths)
    #     for i in range(len(connected_waypaths)):
    #         renovation_strokes_dict[strokes_index][i]=connected_waypaths[i][0:6]
    #     print("len(connected_waypaths) is:",len(connected_waypaths))
    #     print("the starting and ending points are:",renovation_strokes_dict[strokes_index][0])#,renovation_strokes_dict[strokes_index][])
    #     strokes_index+=1
    #     if len(unconnected_waypaths)==0:
    #         break    
    # for i in range(len(renovation_strokes_dict)):
    #     if i==1:
    #         print(renovation_strokes_dict[i])
    #         print("len(renovation_strokes_dict[i]",len(renovation_strokes_dict[i]))
    #     print("the starting point is: ", renovation_strokes_dict[i][0][2])
    #     print("the ending point is: ", renovation_strokes_dict[i][len(renovation_strokes_dict[i])-1][5])
    #     print("*************************************************************")

    # solution, scheduled_strokes_dict=schedule_renovationstrokes(renovation_strokes_dict)
    # print("solution is:",solution)
    # for i in range(len(scheduled_strokes_dict)):
    #     if i==1:
    #         print(scheduled_strokes_dict[i])
    #         print("len(scheduled_strokes_dict[i]) is:",len(scheduled_strokes_dict[i]))
    #     print("the starting point is: ", scheduled_strokes_dict[i][0][2])
    #     print("the ending point is: ", scheduled_strokes_dict[i][len(scheduled_strokes_dict[i])-1][5])
    #     print("------------------------------------------------------------")
    # time2=time.time()
    # print("delta time is:",time2-time1)