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
from geometry_msgs.msg import PoseStamped,Quaternion
import tf

class mobile_platform():
    def __init__(self):
        self.rate=rospy.Rate(1)
        self.mobile_go_point_pub = rospy.Publisher('/renov_down_mobile/mobile_go_to_point', PoseStamped, queue_size=1)
    def euler_to_quaternion(self,euler_data):
        return tf.transformations.quaternion_from_euler(euler_data[0],euler_data[1],euler_data[2])
    def pub_posestamped(self,frame_id,posedata,euler_data):
        p = PoseStamped()
        p.header.frame_id = frame_id
        p.header.stamp = rospy.Time.now()
        p.pose.position.x=posedata[0]
        p.pose.position.y=posedata[1]
        p.pose.position.z=0
        q = self.euler_to_quaternion(euler_data)
        p.pose.orientation = Quaternion(*q)
        self.mobile_go_point_pub.publish(p)
    def mobile_platform_motion(self,mobiledata,rate):
        self.motion_state_current2last()
        while not rospy.is_shutdown():
            last_motion_phase_over_flag=rospy.get_param("/renov_up_level/last_motion_phase_over_flag")
            rospy.loginfo("%s is %s", rospy.resolve_name('last_motion_phase_over_flag'), last_motion_phase_over_flag)
            if last_motion_phase_over_flag==1:
                rospy.logerr("step 1: mobile_platform_motion is in process")
                os.system("rosparam set /renov_up_level/last_motion_phase_over_flag 0")
                time.sleep(3.5)
                self.pub_posestamped("map",[mobiledata[0],mobiledata[1],0],[0,0,mobiledata[2]])
            mobile_platform_tracking_over_flag=rospy.get_param("/renov_up_level/mobile_platform_tracking_over_flag")
            if mobile_platform_tracking_over_flag==1:
                rospy.logerr("step 1: mobile_platform_motion is over")
                os.system("rosparam set /renov_up_level/mobile_platform_tracking_over_flag 0")
                os.system("rosparam set /renov_up_level/current_motion_phase_over_flag 1")
                break
            # mobileplatform_tracking_error=rospy.get_param("/renov_up_level/mobileplatform_tracking_error")
            # mobileplatfomr_tolerance_tracking_error=0.0
            # if abs(mobileplatform_tracking_error)<=mobileplatfomr_tolerance_tracking_error:
            #     rospy.logerr("step 1: mobile_platform_motion is closed")
            #     os.system("rosparam set /renov_up_level/mobile_platform_tracking_over_flag 1")
            #     break
            rate.sleep()
    def mobile_platform_motion_simulation(self,mobiledata,rate):
        self.motion_state_current2last()
        mobileplatform_tracking_error=0.1
        while not rospy.is_shutdown():
            last_motion_phase_over_flag=rospy.get_param("/renov_up_level/last_motion_phase_over_flag")
            # rospy.loginfo("%s is %s", rospy.resolve_name('last_motion_phase_over_flag'), last_motion_phase_over_flag)
            if last_motion_phase_over_flag==1:
                rospy.logerr("step 1: mobile_platform_motion is in process")
                os.system("rosparam set /renov_up_level/last_motion_phase_over_flag 0")
                self.pub_posestamped("mobile_base_link",[mobiledata[0],mobiledata[1],0],[0,0,mobiledata[2]])
                mobileplatform_tracking_error=mobileplatform_tracking_error-0.1
                mobileplatfomr_tolerance_tracking_error=0.1
                if abs(mobileplatform_tracking_error)<=mobileplatfomr_tolerance_tracking_error:
                    rospy.logerr("step 1: mobile_platform_motion is closed")
                    os.system("rosparam set /renov_up_level/current_motion_phase_over_flag 1")
                    break
            rate.sleep()

    def motion_state_current2last(self):
        current_motion_phase_over_flag=rospy.get_param("/renov_up_level/current_motion_phase_over_flag")
        if current_motion_phase_over_flag==1:
            os.system("rosparam set /renov_up_level/last_motion_phase_over_flag 1")

def euler_to_quaternion_function(euler_data):
    angles=tf.transformations.quaternion_from_euler(euler_data[0],euler_data[1],euler_data[2])
    print("quaternion angles is:",angles)
    

def main():
    nodename="mobile platform motion"
    rospy.init_node(nodename)
    ratet=1
    rate = rospy.Rate(ratet)
    
    mobiledata=[0.0,0.0,0.0]
    # renovation_mobileplatform=mobile_platform()
    # renovation_mobileplatform.mobile_platform_motion(mobiledata,rate)
    # renovation_mobileplatform.mobile_platform_motion_simulation(mobiledata,rate)

    data1=[0,0,-0.3584438970917431]
    data2=[0,0, -1.8953594448649913]
    data3=[0,0,0]
    euler_to_quaternion_function(data1)
    euler_to_quaternion_function(data2)
    euler_to_quaternion_function(data3)

if __name__=="__main__":
    main()
