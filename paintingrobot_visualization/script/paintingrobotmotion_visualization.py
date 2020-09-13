#!/usr/bin/env python

import rospy, sys
import math
import tf
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

from sensor_msgs.msg import JointState
from tf_conversions import transformations
from math import pi

class Renovationrobot_joints_pub():
    def __init__(self):
        self.mobile_platform_joints_value=[0.0,0.0,0.0]
        self.jackup_mechanism_joints_value=[0.0,0.0]
        self.aubo_joints_value=[0.0,0.0,0.0,0.0,0.0,0.0]
        self.rodmechanism2ground_offsetlength1=0.86
        self.rodmechanism2ground_offsetlength2=-1.32

        self.aubo_joints_sub=rospy.Subscriber('/renov_up_level/aubo_joints', JointState, self.obtain_aubo_joints, queue_size=10)
        self.paintingrobot_joints_pub=rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.tf_listener = tf.TransformListener()


    def obtain_aubo_joints(self,msg):
        self.aubo_joints_value[0]=msg.position[0]
        self.aubo_joints_value[1]=msg.position[1]
        self.aubo_joints_value[2]=msg.position[2]
        self.aubo_joints_value[3]=msg.position[3]
        self.aubo_joints_value[4]=msg.position[4]
        self.aubo_joints_value[5]=msg.position[5]

    def obtain_mobileplatform_states(self):
        try:
            (trans,rot) = self.tf_listener.lookupTransform( '/map','/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            return None
        euler = transformations.euler_from_quaternion(rot)
        self.mobile_platform_joints_value[0] = trans[0]
        self.mobile_platform_joints_value[1] = trans[1]
        self.mobile_platform_joints_value[2] = euler[2]
        # print("mobile platform x is:", self.mobile_platform_joints_value[0])
        # print("mobile platform y is:", self.mobile_platform_joints_value[1])
        print("the angle is:", self.mobile_platform_joints_value[2])


    def obtain_jackupmechanism_states(self):
        line_encoder_data=rospy.get_param('/renov_up_level/read_line_encode')
        self.jackup_mechanism_joints_value[0]=line_encoder_data+self.rodmechanism2ground_offsetlength1+self.rodmechanism2ground_offsetlength2
        self.jackup_mechanism_joints_value[1]=0.0


    def obtain_paintingrobot_states(self):
        paintingrobot_joints=JointState()
        paintingrobot_joints.header.stamp=rospy.Time.now()
        paintingrobot_joints.name = []
        paintingrobot_joints.position = []
        paintingrobot_joints.velocity = []
        paintingrobot_joints.effort = []

        paintingrobot_joints.name.append('base_joint1')
        paintingrobot_joints.position.append(0.0)
        # print(paintingrobot_joints.position)
        # paintingrobot_joints.name[0]='base_joint1'
        # paintingrobot_joints.position[0]=self.mobile_platform_joints_value[0]
        paintingrobot_joints.name.append('base_joint2')
        paintingrobot_joints.position.append(0.0)
        paintingrobot_joints.name.append('mobilebase_joint')
        paintingrobot_joints.position.append(0.0)
        print("the received angle is:",self.mobile_platform_joints_value[2])

        paintingrobot_joints.name.append('rodclimbing_joint1')
        paintingrobot_joints.position.append(self.jackup_mechanism_joints_value[0])
        paintingrobot_joints.name.append('rodclimbing_joint2')
        paintingrobot_joints.position.append(self.jackup_mechanism_joints_value[1])

        paintingrobot_joints.name.append('shoulder_joint')
        paintingrobot_joints.position.append(self.aubo_joints_value[0])
        paintingrobot_joints.name.append('upperArm_joint')
        paintingrobot_joints.position.append(self.aubo_joints_value[1])
        paintingrobot_joints.name.append('foreArm_joint')
        paintingrobot_joints.position.append(self.aubo_joints_value[2])
        paintingrobot_joints.name.append('wrist1_joint')
        paintingrobot_joints.position.append(self.aubo_joints_value[3])
        paintingrobot_joints.name.append('wrist2_joint')
        paintingrobot_joints.position.append(self.aubo_joints_value[4])
        paintingrobot_joints.name.append('wrist3_joint')
        paintingrobot_joints.position.append(self.aubo_joints_value[5])

        self.paintingrobot_joints_pub.publish(paintingrobot_joints)
        rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node('paintingrobot_jointsvalue_publish', anonymous=True)
    rate = rospy.Rate(10.0)
    Renovationrobot=Renovationrobot_joints_pub()
    while not rospy.is_shutdown():
        Renovationrobot.obtain_mobileplatform_states()
        Renovationrobot.obtain_jackupmechanism_states()
        Renovationrobot.obtain_paintingrobot_states()
        # rate.sleep()





















# class robot_pose_computation():
#     def __init__(self):
#         self.T1=np.eye(4)
#         self.T2=np.eye(4)
#         self.T3=np.eye(4)
#         self.wholerobot_pose=np.array([0.0,0.0,0.0,0.0,0.0,0.0])
#         self.aubo_pose_sub = rospy.Subscriber('aubo_pose',Pose, manipulator_computation)

#     # part one: mobile platform frame to map frame
#     def tf_computation(self):
#         try:
#             (mobile_trans, mobile_quaternion) = listener.lookupTransform('/base_link', '/aubo_baselink', rospy.Time(0))
#         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             continue
#         # obtain trans
#         mobile_trans
#         # quternion to euler

#         # final: obtain T1 and z-yaw angle

#     # part two: aubo manipulator base to mobile platform frame: height/angle is obtained from the linear/rotation encoder respectively
#     def rodclimbing_computation(self):
#         constantvalue=0.5
#         rotation_length=0.6
#         linear_encoder_length=rospy.get_param("")
#         rotation_encoder_angle=rospy.get_param("")
#         tran_x=rotation_length*math.cos(rotation_encoder_angle)
#         tran_y=rotation_length*math.sin(rotation_encoder_angle)
#         tran_z=linear_encoder_length+constantvalue

#         rot_mat=rotz(rotation_encoder_angle)
#         # obtain T2 and z-yaw angle

#     # part three: aubo manipulator end effector to base frame
#     def manipulator_computation(self,data):
#         p = np.array([data.position.x, data.position.y, data.position.z])
#         q = np.array([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
#         # obtain T3 and rpy angles
#         mat_generation = pose2mat()
#         self.T3 = mat_generation.mat4x4(p, q)

#     def wholerobot_computation(self):
#         wholerobot_T=np.array(self.T1*self.T2*self.T3)
#         tran_x=wholerobot_T[0][3]
#         tran_y=wholerobot_T[1][3]
#         tran_z=wholerobot_T[2][3]
#         # obtain rpy angles and T matrix
#     listener = tf.TransformListener()








