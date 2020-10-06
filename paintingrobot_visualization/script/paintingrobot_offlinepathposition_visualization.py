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

from nav_msgs.msg import Path
from geometry_msgs.msg import Quaternion, PoseStamped, Point
import tf
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

def rotx(q):
    cq=cos(q)
    sq=sin(q)
    R =[[1, 0, 0],
        [0, cq, -sq],
        [0, sq, cq]]
    return R
def roty(q):
    cq = cos(q)
    sq = sin(q)
    R=[[cq, 0, sq],
        [0, 1, 0],
        [-sq,0,cq]]
    return  R
def rotz(q):
    cq=cos(q)
    sq=sin(q)
    R=[[cq, -sq, 0],
        [sq,  cq, 0],
        [ 0,   0, 1]]
    return R
def rpy2r(q):
    roll=q[0]
    pitch=q[1]
    yaw=q[2]
    Rx = rotx(roll)
    Ry = roty(pitch)
    Rz = rotz(yaw)
    R1=np.dot(Rx, Ry)
    R=np.dot(R1,Rz)
    return R

def coordinate_frame_visualization(visualization_num, robotplacement_pose):
    points=Marker()
    line_strip=Marker()
    x_axis=Marker()
    y_axis=Marker()
    z_axis=Marker()
    points.header.frame_id = line_strip.header.frame_id = x_axis.header.frame_id = y_axis.header.frame_id =z_axis.header.frame_id  = "/map"
    points.header.stamp = line_strip.header.stamp = x_axis.header.stamp= y_axis.header.stamp = z_axis.header.stamp= rospy.Time.now()
    points.ns = line_strip.ns = x_axis.ns= y_axis.ns= z_axis.ns = "points_and_lines"
    points.action = line_strip.action = x_axis.action = y_axis.action =z_axis.action = Marker.ADD
    points.pose.orientation.w = line_strip.pose.orientation.w = x_axis.pose.orientation.w =y_axis.pose.orientation.w =z_axis.pose.orientation.w = 1.0

    points.id = visualization_num
    line_strip.id = visualization_num+1
    x_axis.id = visualization_num+2
    y_axis.id = visualization_num+3
    z_axis.id = visualization_num+4     

    points.type = Marker.POINTS
    line_strip.type = Marker.LINE_STRIP
    x_axis.type = Marker.LINE_STRIP
    y_axis.type = Marker.LINE_STRIP
    z_axis.type = Marker.LINE_STRIP

    "POINTS markers use x and y scale for width/height respectively" 
    points.scale.x = 0.1
    points.scale.y = 0.1
    "LINE_STRIP markers use only the x component of scale, for the line width"
    x_axis.scale.x = 0.04
    y_axis.scale.x = 0.04
    z_axis.scale.x = 0.04

    points.color.r = 0.1
    points.color.g = 0.1
    points.color.b = 0.1
    points.color.a = 1.0

    x_axis.color.r = 1.0
    x_axis.color.a = 1.0
    y_axis.color.g = 1.0
    y_axis.color.a = 1.0
    z_axis.color.b = 1.0
    z_axis.color.a = 1.0

    "Create the vertices for the points and lines"
    p0=Point()
    p1=Point()
    p2=Point()
    p3=Point()
    p0.x=robotplacement_pose[0]
    p0.y=robotplacement_pose[1]
    p0.z=robotplacement_pose[2]

    q=[robotplacement_pose[3], robotplacement_pose[4], robotplacement_pose[5]]
    rot_mat=rpy2r(q)

    scale=0.5

    p1.x=robotplacement_pose[0]+rot_mat[0,0]*scale
    p1.y=robotplacement_pose[1]+rot_mat[1,0]*scale
    p1.z=robotplacement_pose[2]+rot_mat[2,0]*scale
    
    p2.x=robotplacement_pose[0]+rot_mat[0,1]*scale
    p2.y=robotplacement_pose[1]+rot_mat[1,1]*scale
    p2.z=robotplacement_pose[2]+rot_mat[2,1]*scale

    p3.x=robotplacement_pose[0]+rot_mat[0,2]*scale
    p3.y=robotplacement_pose[1]+rot_mat[1,2]*scale
    p3.z=robotplacement_pose[2]+rot_mat[2,2]*scale

    points.points.append(p0)
    x_axis.points.append(p0)
    x_axis.points.append(p1)
    y_axis.points.append(p0)
    y_axis.points.append(p2)
    z_axis.points.append(p0)
    z_axis.points.append(p3)

    visualization_num=visualization_num+4
    return visualization_num, points, x_axis, y_axis, z_axis

def path_visualization(num1, waypoints):
    marker1 = Marker()
    marker1.header.frame_id = '/map'
    marker1.type = Marker.LINE_STRIP
    marker1.action = Marker.ADD
    marker1.pose.orientation.w = 1.0
    marker1.scale.x = 0.05
    marker1.ns = 'paths and waypoints'
    marker1.id = num1
    marker1.color.r = 0.1
    marker1.color.g = 0.1
    marker1.color.b = 0.1
    marker1.color.a = 1.0
    marker1.lifetime = rospy.Duration()
    for i in range(len(waypoints)):
        pathpoints=Point()
        pathpoints.x = waypoints[i][0]
        pathpoints.y = waypoints[i][1]
        pathpoints.z = waypoints[i][2]
        marker1.points.append(pathpoints)
        # marker2.points.append(pathpoints)
    return num1,marker1


if __name__ == "__main__":
    # sparse the data
    coverage_json_path=rospy.get_param("coverage_json_path")
    with open(coverage_json_path,'r') as f:
        coverageplanningresults_dict=json.load(f)

    rospy.init_node("visualization_test")
    pub_state = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    ratet=5
    rate = rospy.Rate(ratet)

    plane_num_count=0
    mobile_base_point_count=0
    climb_base_count_num=0
    visualization_num=1
    while not rospy.is_shutdown():

        "visualize aubo offline path"
        renovation_waypaths_onecell=coverageplanningresults_dict["plane_num_"+str(plane_num_count)]["plane_renovationcells_num_"+str(plane_num_count)]["renovationcells_num_"+str(mobile_base_point_count)]
        renovation_onewaypath_onecell=np.zeros((2,3))
        for i in range(len(renovation_waypaths_onecell)):
            renovation_onewaypath_onecell[0][0] = renovation_waypaths_onecell[i][0]
            renovation_onewaypath_onecell[0][1] = renovation_waypaths_onecell[i][1]
            renovation_onewaypath_onecell[0][2] = renovation_waypaths_onecell[i][2]

            renovation_onewaypath_onecell[1][0] = renovation_waypaths_onecell[i][3]
            renovation_onewaypath_onecell[1][1] = renovation_waypaths_onecell[i][4]
            renovation_onewaypath_onecell[1][2] = renovation_waypaths_onecell[i][5]

            visualization_num, manipulator_path=path_visualization(visualization_num,renovation_onewaypath_onecell)
            visualization_num=visualization_num+1
            pub_state.publish(manipulator_path)
            # rate.sleep()

        while not rospy.is_shutdown():
            climb_base_count_num+=1
            if climb_base_count_num>=len(coverageplanningresults_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]):
                mobile_base_point_count+=1
                climb_base_count_num=0
                break


        if mobile_base_point_count >= len(coverageplanningresults_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]):
            if plane_num_count==0:
                plane_num_count+=2
            elif plane_num_count==2: 
                plane_num_count+=2
            elif plane_num_count==4:
                plane_num_count+=1
            elif plane_num_count==5:
                plane_num_count+=1
            mobile_base_point_count=0            


        if plane_num_count>=6:
            plane_num_count=0
            mobile_base_point_count=0
            climb_base_count_num=0
            visualization_num=1
            # break


        "visualize mobile platform coordinate frame"
        mobileplatform_targetjoints=coverageplanningresults_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]["mobile_data_num_"+str(mobile_base_point_count)]
        visualization_num, point, x_axis, y_axis, z_axis= coordinate_frame_visualization(visualization_num, mobileplatform_targetjoints)
        visualization_num=visualization_num+1
        pub_state.publish(point)
        pub_state.publish(x_axis)
        pub_state.publish(y_axis)
        pub_state.publish(z_axis)



