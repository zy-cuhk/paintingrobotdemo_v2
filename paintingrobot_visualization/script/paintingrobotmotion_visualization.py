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
        self.rodmechanism2ground_offsetlength1=0.76
        self.rodmechanism2ground_offsetlength2=-1.32

        self.aubo_joints_sub=rospy.Subscriber('/renov_up_level/aubo_joints', JointState, self.obtain_aubo_joints, queue_size=10)
        self.paintingrobot_joints_pub=rospy.Publisher('/joint_states1', JointState, queue_size=10)
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
        paintingrobot_joints.name.append('base_joint2')
        paintingrobot_joints.position.append(0.0)
        paintingrobot_joints.name.append('mobilebase_joint')
        paintingrobot_joints.position.append(0.0)


        # paintingrobot_joints.name.append('base_joint1')
        # paintingrobot_joints.position.append(self.mobile_platform_joints_value[0])
        # paintingrobot_joints.name.append('base_joint2')
        # paintingrobot_joints.position.append(self.mobile_platform_joints_value[1])
        # paintingrobot_joints.name.append('mobilebase_joint')
        # paintingrobot_joints.position.append(self.mobile_platform_joints_value[2])
        # print("the received angle is:",self.mobile_platform_joints_value[2])

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








