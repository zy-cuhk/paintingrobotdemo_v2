#!/usr/bin/env python
# -*- coding: utf_8 -*-
import rospy
import sys
import binascii
import inspect
import serial
import time
from CRC_16_Check_CLB import *
from clb_command import *
from textwrap import wrap
from std_msgs.msg import String,UInt64MultiArray
import math
import os
class CLMBPKG:
    def __init__(self,nodename):
        self.readstringlength=7

        self.crc16=RobotCRC16()
        self.nodename=nodename
        self.plccmd=CLBDriverCommands() 
        self.Openmodbus_ok_flag=0
        self.buf = bytearray()
        self.climb_close_loop_inital()
        self.rotation_close_loop_inital()
        self.hold_close_loop_inital()
    def rotation_close_loop_inital(self,):
        self.rotation_joint_line_equation_k=rospy.get_param("rotation_joint_line_equation_k")
        self.rotation_joint_line_equation_b=rospy.get_param("rotation_joint_line_equation_b")
        self.pid_tolerance_error_rotation=rospy.get_param("pid_tolerance_error_rotation")
        self.Kp_rotation = rospy.get_param("rotation_kp")
        self.Ki_rotation = rospy.get_param("rotation_ki")
        self.Kd_rotation = rospy.get_param("rotation_kd")
        self.current_time_rotation=0
        self.last_time_rotation=0
        self.last_error_rotation=0
        self.sample_time_rotation=0.00
        self.PTerm_rotation=0
        self.ITerm_rotation=0
        self.DTerm_rotation=0
        self.int_rotation_error = 0.0
        self.windup_guard_rotation = 0.1
        self.output_rotation = 0.0
    def hold_close_loop_inital(self,):
        self.pid_tolerance_error_standbar=rospy.get_param("pid_tolerance_error_standbar")
        self.Kp_hold = rospy.get_param("hold_kp")
        self.Ki_hold = rospy.get_param("hold_ki")
        self.Kd_hold = rospy.get_param("hold_kd")
        self.current_time_hold=0
        self.last_time_hold=0
        self.last_error_hold=0
        self.sample_time_hold=0.00
        self.PTerm_hold=0
        self.ITerm_hold=0
        self.DTerm_hold=0
        self.int_hold_error = 0.0
        self.windup_guard_hold = 0.1
        self.output_hold = 0.0
        self.read_line_l0_encode_bottom=rospy.get_param("read_line_l0_encode_bottom")        
    def climb_close_loop_inital(self,):
        self.pid_tolerance_error_climb=rospy.get_param("pid_tolerance_error_climb")
        self.Kp_climb = rospy.get_param("climb_kp")
        self.Ki_climb = rospy.get_param("climb_ki")
        self.Kd_climb = rospy.get_param("climb_kd")
        self.current_time_climb=0
        self.last_time_climb=0
        self.last_error_climb=0
        self.sample_time_climb=0.00
        self.PTerm_climb=0
        self.ITerm_climb=0
        self.DTerm_climb=0
        self.int_climb_error = 0.0
        self.windup_guard_climb = 0.1
        self.output_climb = 0.0
    def Init_node(self):
        rospy.init_node(self.nodename)

    def Send_message_to_port(self,ser,message):
        """
        :param message: String "010600000001480A" or "01 06 00 00 00 01 48 0A"
        :return: int list
        """
        # print("message",message)
        message_bytes = message.replace(" ",'').decode('hex')
        # print("message_bytes",message_bytes)
        # print str(hex(message_bytes))
        
        ser.write(message_bytes)
        ser.flushInput()
        ser.flushOutput()

        time.sleep(0.02)
        strt = ser.read(self.readstringlength).encode('hex')
        # endtime=time.time()
        # rospy.logerr("---Send_message_to_port--spend time -----$%s",endtime-starttime)
        readbuffer=[int(int(i, 16)) for i in wrap(strt, 2)]

        return readbuffer,message
    def Get_crc_16_str(self,cmd):
        return self.crc16.Combining_CRC_and_info(cmd)
    def Get_hex_list_from_OCT(self,data):
        if data<0:
            datahexstr=hex(int(data) & (2**16-1)).replace('0x','')
            datahexstr=datahexstr.zfill(len(datahexstr)+4-len(datahexstr))
            return [int(datahexstr[:2],16),int(datahexstr[2:],16)]
        elif data==0:
            return [0x00,0x00]
        else:
            # print data,type(data)
            datahexstr=hex(int(data)).replace('0x','')
            datahexstr=datahexstr.zfill(len(datahexstr)+4-len(datahexstr))
            return [int(datahexstr[:2],16),int(datahexstr[2:],16)]
    def Control_3DOF_Robot_Velocity(self, ser, control_id, velocity):  # velocity control
                """
                :param master:
                :param control_id: 1-stand,2-rotation,3-climber
                :param velocity: 0-2500
                :param outputPulse: High 32位
                :return:
                """
                if control_id==1:
                    #p001
                    self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.HOLD_DRIVER_MODEL_VELOCITY))#seting vel model
                    #p324
                    send324DataHEXlist=self.plccmd.HOLDING_INITIAL_P324_BASE_VELOCITY_DATA[:4]+self.Get_hex_list_from_OCT(velocity)
                    self.Send_message_to_port(ser,self.Get_crc_16_str(send324DataHEXlist))#seting pos model
                if control_id==3:
                    #p001
                    self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.CLIMB_DRIVER_MODEL_VELOCITY))#seting pos model
                    #p290
                    send324DataHEXlist=self.plccmd.CLIMB_INITIAL_P324_BASE_VELOCITY_DATA[:4]+self.Get_hex_list_from_OCT(velocity)
                    self.Send_message_to_port(ser,self.Get_crc_16_str(send324DataHEXlist))#seting pos model
                if control_id==2:
                    #p001
                    self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.ROTATION_DRIVER_MODEL_VELOCITY))#seting pos model
                    #p290
                    send324DataHEXlist=self.plccmd.ROTATION_INITIAL_P324_BASE_VELOCITY_DATA[:4]+self.Get_hex_list_from_OCT(velocity)
                    self.Send_message_to_port(ser,self.Get_crc_16_str(send324DataHEXlist))#seting pos model
    def Hold_Robot_close_loop_control(self, ser, DesireDistance, feedback_distance,control_id=1):  # velocity control
            """
            Integral windup, also known as integrator windup or reset windup,
            refers to the situation in a PID feedback controller where
            a large change in setpoint occurs (say a positive change)
            and the integral terms accumulates a significant error
            during the rise (windup), thus overshooting and continuing
            to increase as this accumulated error is unwound
            (offset by errors in the other direction).
            The specific problem is the excess overshooting.
            :param ser:
            :param velocity: +-2500
            :param DesireDistance: 0.07-0.22
            :param control_id:
            :return:
            """
            Desiredisplacement=self.read_line_l0_encode_bottom-DesireDistance
            feedback_displacement=self.read_line_l0_encode_bottom-feedback_distance
            displacement_error=Desiredisplacement-feedback_displacement
            # rospy.loginfo("standbar----target displacement is:%s",str(Desiredisplacement))
            # rospy.loginfo("standbar----feedback displacement is:%s",str(feedback_displacement))
            # rospy.logerr("standbar----displacement error is:%s",str(displacement_error))
            Distance_error=DesireDistance-feedback_distance
            self.current_time_hold=time.time()
            deltatime=self.current_time_hold-self.last_time_hold
            
            deltaerror=Distance_error-self.last_error_hold
            if(deltatime>=self.sample_time_hold):
                self.PTerm_hold=self.Kp_hold*Distance_error
                self.ITerm_hold+=Distance_error*deltatime
                if (self.ITerm_hold < -self.windup_guard_hold):
                    self.ITerm_hold = -self.windup_guard_hold
                elif (self.ITerm_hold > self.windup_guard_hold):
                    self.ITerm_hold = self.windup_guard_hold
                self.DTerm_hold=0.0
                if deltatime>0:
                    self.DTerm_hold=deltaerror/deltatime
                self.last_time_hold=self.current_time_hold
                self.last_error_hold=Distance_error
                self.output_hold=self.PTerm_hold + (self.Ki_hold *self.ITerm_hold) + (self.Kd_hold * self.DTerm_hold)

                velocity = self.output_hold 
                # rospy.logerr("hold----velocity---before:  %s",str(velocity))
                if velocity<0 and abs(velocity)>1300:
                    velocity=-1300.0
                elif velocity>0 and abs(velocity)>1300:
                    velocity=1300.0
                else:
                    pass
                # rospy.loginfo("hold----velocity:  %s",str(velocity))
                if abs(Distance_error)<=self.pid_tolerance_error_standbar:
                    os.system('rosparam set /renov_up_level/hold_distance_tracking_over 1')
                    velocity=0
                self.Control_3DOF_Robot_Velocity(ser, control_id, velocity)
    def Rotation_Robot_close_loop_control(self, ser,DesireEncodeData,feedback_EncodeData,control_id=2):  # velocity control
        """
        Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        :param master:
        :param velocity: 0-2500
        :param DesireDistance: 0-3m
        :param control_id:
        :return:
        """
        encodedata_error=DesireEncodeData-feedback_EncodeData
        target_angle=self.rotation_joint_line_equation_k*DesireEncodeData+self.rotation_joint_line_equation_b
        current_angle=self.rotation_joint_line_equation_k*feedback_EncodeData+self.rotation_joint_line_equation_b
        angle_error=encodedata_error*self.rotation_joint_line_equation_k
        # rospy.loginfo("---target_angle---%s",str(target_angle))
        # rospy.loginfo("---current_angle---%s",str(current_angle))        
        # rospy.logerr("------neg anticlockwise, pos clockwise---angle_error---%s",str(angle_error))
        self.current_time_rotation=time.time()
        deltatime=self.current_time_rotation-self.last_time_rotation
        
        deltaerror=encodedata_error-self.last_error_rotation
        if(deltatime>=self.sample_time_rotation):
            self.PTerm_rotation=self.Kp_rotation*encodedata_error
            self.ITerm_rotation+=encodedata_error*deltatime
            if (self.ITerm_rotation < -self.windup_guard_rotation):
                self.ITerm_rotation = -self.windup_guard_rotation
            elif (self.ITerm_rotation > self.windup_guard_rotation):
                self.ITerm_rotation = self.windup_guard_rotation
            self.DTerm_rotation=0.0
            if deltatime>0:
                self.DTerm_rotation=deltaerror/deltatime
            self.last_time_rotation=self.current_time_climb
            self.last_error_rotation=encodedata_error
            self.output_rotation=self.PTerm_rotation + (self.Ki_rotation *self.ITerm_rotation) + (self.Kd_climb * self.DTerm_climb)

            velocity = -1.0*self.output_rotation
            # rospy.loginfo("-------roation pid control velocity------%s",velocity) 
            if velocity<0 and abs(velocity)>1000:
                velocity=-1000.0
            elif velocity>0 and abs(velocity)>1000:
                velocity=1000.0
            else:
                pass
            if abs(angle_error)<=self.pid_tolerance_error_rotation:
                velocity=0
                os.system('rosparam set /renov_up_level/rotation_distance_tracking_over 1')
            self.Control_3DOF_Robot_Velocity(ser, control_id, velocity)

                
    def Climbing_Robot_close_loop_control(self, ser,DesireDistance, feedback_distance,control_id=3):  # velocity control
        """
        Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        :param master:
        :param velocity: 0-2500
        :param DesireDistance: 0-3m
        :param control_id:
        :return:
        """
        Distance_error=DesireDistance-feedback_distance
        # rospy.loginfo("-------climb pid control Desired Distance-:  %s",str(DesireDistance))
        # rospy.loginfo("-------climb pid control feedback Distance-:  %s",str(feedback_distance))     
        # rospy.logerr("-------climb pid control Distance_error-:  %s",str(Distance_error))

        self.current_time_climb=time.time()
        deltatime=self.current_time_climb-self.last_time_climb
        deltaerror=Distance_error-self.last_error_climb
        if(deltatime>=self.sample_time_climb):
            self.PTerm_climb=self.Kp_climb*Distance_error
            self.ITerm_climb+=Distance_error*deltatime
            if (self.ITerm_climb < -self.windup_guard_climb):
                self.ITerm_climb = -self.windup_guard_climb
            elif (self.ITerm_climb > self.windup_guard_climb):
                self.ITerm_climb = self.windup_guard_climb
            self.DTerm_climb=0.0
            if deltatime>0:
                self.DTerm_climb=deltaerror/deltatime
            self.last_time_climb=self.current_time_climb
            self.last_error_climb=Distance_error
            self.output_climb=self.PTerm_climb + (self.Ki_climb *self.ITerm_climb) + (self.Kd_climb * self.DTerm_climb)

            velocity = -1.0*self.output_climb #*42.5
            # rospy.loginfo("-------climb pid control velocity------before:  %s",str(velocity))
            if velocity<0 and abs(velocity)>1000:
                velocity=-1000.0
            elif velocity>0 and abs(velocity)>1000:
                velocity=1000.0
            else:
                pass
            # rospy.loginfo("-------climb pid control velocity------:  %s",str(velocity))       
            if abs(Distance_error)<=self.pid_tolerance_error_climb:
                velocity=0
                os.system('rosparam set /renov_up_level/climb_distance_tracking_over 1')
            self.Control_3DOF_Robot_Velocity(ser, control_id, velocity)  
    def Open_Stop_Enable_Driver(self, ser, control_id,stop_open_flag):
        if control_id==1:
            #p282
            if stop_open_flag==1:
                rospy.loginfo(self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.HOLDING_DRIVER_P282_ENABALE)))#seting pos model
            if stop_open_flag==0:
                rospy.loginfo(self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.HOLDING_DRIVER_P282_DISENABALE)))#seting pos model      
        if control_id==3:
            #p282
            if stop_open_flag==1:
                rospy.loginfo(self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.CLIMB_DRIVER_P282_ENABALE)))#seting pos model
            if stop_open_flag==0:
                rospy.loginfo(self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.CLIMB_DRIVER_P282_DISENABALE)))#seting pos model      
        if control_id==2:
            #p282
            if stop_open_flag==1:
                rospy.loginfo(self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.ROTATION_DRIVER_P282_ENABALE)))#seting pos model
            if stop_open_flag==0:
                rospy.loginfo(self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.ROTATION_DRIVER_P282_DISENABALE)))#seting pos model 
                        
  
def main():
    clbpkg=CLMBPKG("tridof_pkg_node")
    open_serial_port_again_flag=0
    clbpkg.Init_node()

    climb_port = rospy.get_param("climb_port")
    rospy.loginfo("%s is %s", rospy.resolve_name('climb_port'), climb_port)
    # fetch the utterance parameter from our parent namespace
    climb_port_baudrate = rospy.get_param('climb_port_baudrate')
    rospy.loginfo("%s is %s", rospy.resolve_name('climb_port_baudrate'), climb_port_baudrate)
    stand_bar_flex_distance=rospy.get_param("stand_bar_flex_distance")
    light_scan_to_top_distance=rospy.get_param("light_scan_to_top_distance")
    rotation_homing_abs_encode_data=rospy.get_param("rotation_homing_abs_encode_data")
    read_line_l0_encode = rospy.get_param("read_line_l0_encode")
    rotation_joint_line_equation_k = rospy.get_param("rotation_joint_line_equation_k")
    rotation_joint_line_equation_b = rospy.get_param("rotation_joint_line_equation_b")

    try:
        ser = serial.Serial(port=climb_port, baudrate=climb_port_baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN, stopbits=1,timeout=0.3, xonxoff=0,rtscts=False,dsrdtr=False)
    except:
        rospy.logerr("Please check TRIDOF robot Usb port----")
        open_serial_port_again_flag=1

    count=0
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        read_line_encode = rospy.get_param("read_line_encode")
        read_line_encode_bottom = rospy.get_param("read_line_encode_bottom")
        read_line_l0_encode_bottom = rospy.get_param("read_line_l0_encode_bottom")
        # rospy.loginfo("%s is %s", rospy.resolve_name('read_line_encode'), read_line_encode)
        rotation_abs_encode= rospy.get_param("rotation_abs_encode")
        climb_port_ok_flag = rospy.get_param("climb_port_ok_flag")
        light_scan_to_ceil_distance = rospy.get_param("light_scan_to_ceil_distance")
        close_all_3dof_climb_driver_flag = rospy.get_param("close_all_3dof_climb_driver_flag")
        # rospy.loginfo("%s is %s", rospy.resolve_name('close_all_3dof_climb_driver_flag'), close_all_3dof_climb_driver_flag)
        enable_control_stand_bar = rospy.get_param("enable_control_stand_bar")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_control_stand_bar'), enable_control_stand_bar)
        enable_control_rotation = rospy.get_param("enable_control_rotation")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_control_rotation'), enable_control_rotation)
        enable_climb_control = rospy.get_param("enable_climb_control")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_climb_control'), enable_climb_control)
        velocity_control_stand_bar = rospy.get_param("velocity_control_stand_bar")
        # rospy.loginfo("%s is %s", rospy.resolve_name('velocity_control_stand_bar'), velocity_control_stand_bar)
        velocity_control_rotation = rospy.get_param("velocity_control_rotation")
        # rospy.loginfo("%s is %s", rospy.resolve_name('velocity_control_rotation'), velocity_control_rotation)
        velocity_climb_control = rospy.get_param("velocity_climb_control")
        # rospy.loginfo("%s is %s", rospy.resolve_name('velocity_climb_control'), velocity_climb_control)
        distance_control_stand_bar = rospy.get_param("distance_control_stand_bar")
        # rospy.loginfo("%s is %s", rospy.resolve_name('distance_control_stand_bar'), distance_control_stand_bar)
        rad_control_rotation = rospy.get_param("rad_control_rotation")
        # rospy.loginfo("%s is %s", rospy.resolve_name('rad_control_rotation'), rad_control_rotation)
        distance_climb_control = rospy.get_param("distance_climb_control")
        # rospy.loginfo("%s is %s", rospy.resolve_name('distance_climb_control'), distance_climb_control)
        top_limit_switch_status = rospy.get_param("top_limit_switch_status")
        # rospy.loginfo("%s is %s", rospy.resolve_name('top_limit_switch_status'), top_limit_switch_status)
        mid_limit_switch_status = rospy.get_param("mid_limit_switch_status")
        # rospy.loginfo("%s is %s", rospy.resolve_name('mid_limit_switch_status'), mid_limit_switch_status)
        bottom_limit_switch_status = rospy.get_param("bottom_limit_switch_status")
        # rospy.loginfo("%s is %s", rospy.resolve_name('bottom_limit_switch_status'), bottom_limit_switch_status)
        enable_second_control_stand_bar = rospy.get_param("enable_second_control_stand_bar")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_second_control_stand_bar'), enable_second_control_stand_bar)
        enable_second_climb_control = rospy.get_param("enable_second_climb_control")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_second_climb_control'), enable_second_climb_control)
        enable_third_stand_bar = rospy.get_param("enable_third_stand_bar")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_third_stand_bar'), enable_third_stand_bar)
        open_climb_flag = rospy.get_param("open_climb_flag")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_second_control_stand_bar'), enable_second_control_stand_bar)
        open_hold_flag = rospy.get_param("open_hold_flag")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_second_climb_control'), enable_second_climb_control)
        open_rotation_flag = rospy.get_param("open_rotation_flag")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_third_stand_bar'), enable_third_stand_bar)
        open_hold_to_ceil_flag = rospy.get_param("open_hold_to_ceil_flag")

        if clbpkg.Openmodbus_ok_flag!=1 and climb_port_ok_flag==1:
            rospy.loginfo("climb usb port is ok!")
            # the collision of top limit switch triggers stopping stand bar 
            if top_limit_switch_status==1:
                if enable_second_control_stand_bar==0:
                    try:
                        clbpkg.Control_3DOF_Robot_Velocity(ser, 1, 0)
                    except:
                        rospy.logerr("some errors with top_limit_switch_status --- hold--0")
                    rospy.set_param('open_hold_flag',0)
            # the collision of middle limit switch triggers stopping climb mechanism 
            if mid_limit_switch_status==1:
                if enable_second_climb_control==0:
                    try:
                        clbpkg.Control_3DOF_Robot_Velocity(ser, 3, 0)
                    except:
                        rospy.logerr("some errors with mid_limit_switch_status--- climb--0")
                    rospy.set_param('open_climb_flag',0)
            # the collision of middle limit switch triggers stopping stand bar mechanism 
            if bottom_limit_switch_status==1:
                if enable_third_stand_bar==0:
                    try:
                        clbpkg.Control_3DOF_Robot_Velocity(ser, 1, 0)
                    except:
                        rospy.logerr("some errors with bottom_limit_switch_status--- hold")
                    rospy.set_param('open_hold_flag',0)


            # enabling and disabling operation for stand bar 
            if enable_control_stand_bar==1:
                try:
                    clbpkg.Open_Stop_Enable_Driver(ser,1,1)
                except:
                    rospy.logerr("some errors with enable_control_stand_bar--- hold--1")
                rospy.set_param('enable_control_stand_bar',0)
            if enable_control_stand_bar==2:
                try:
                    clbpkg.Open_Stop_Enable_Driver(ser,1,0)
                except:
                    rospy.logerr("some errors with enable_control_stand_bar--- hold--0")
                rospy.set_param("enable_control_stand_bar",0)
                open_hold_flag=0
                rospy.set_param('open_hold_flag',0)

            # enabling and disabling operation for climbing mechanism 
            if enable_climb_control==1:
                try:
                    clbpkg.Open_Stop_Enable_Driver(ser,3,1)
                except:
                    rospy.logerr("some errors with enable_climb_control--- climb--1") 
                rospy.set_param('enable_climb_control',0)
                # open_climb_flag=1
            if enable_climb_control==2:
                try:
                    clbpkg.Open_Stop_Enable_Driver(ser,3,0)
                except:
                    rospy.logerr("some errors with enable_climb_control==2--- climb--0")                     
                rospy.set_param('enable_climb_control',0)
                open_climb_flag=0
                rospy.set_param('open_climb_flag',0)

            # enabling and disabling operation for rotation mechanism 
            if enable_control_rotation==1:
                try:
                    clbpkg.Open_Stop_Enable_Driver(ser,2,1)
                except:
                    rospy.logerr("some errors with enable_control_rotation==1")
                rospy.set_param('enable_control_rotation',0)
                # open_rotation_flag=1
            if enable_control_rotation==2:
                try:
                    clbpkg.Open_Stop_Enable_Driver(ser,2,0)
                except:
                    rospy.logerr("some errors with enable_control_rotation--- rotation--0")                
                rospy.set_param('enable_control_rotation',0)
                open_rotation_flag=0
                rospy.set_param('open_rotation_flag',0)

            # open pid control for stand bar, climbing mechanism and rotation mechanism
            if open_hold_flag==1:
                try:
                    distance_control_stand_bar = rospy.get_param("distance_control_stand_bar")
                    min_holding_distance = rospy.get_param("min_holding_distance")
                    max_holding_distance = rospy.get_param("max_holding_distance")
                    if distance_control_stand_bar>=min_holding_distance and distance_control_stand_bar<=max_holding_distance:
                        # rospy.logerr("you can not more than 0.2----")
                        target_hold_distance=read_line_l0_encode_bottom+distance_control_stand_bar#-light_scan_to_top_distance
                        # rospy.logerr("read_line_encode_bottom-----inclimb modbus%s  target_hold_distance%s",read_line_encode_bottom,target_hold_distance)
                        clbpkg.Hold_Robot_close_loop_control(ser,target_hold_distance, read_line_encode_bottom)
                    elif distance_control_stand_bar<min_holding_distance:
                        rospy.loginfo("distance_control_stand_bar is %s"%(str(distance_control_stand_bar)))
                        rospy.loginfo("min_holding_distance is %s"%(str(min_holding_distance)))
                        rospy.logerr("the holding distance is smaller than minimum holding distance")
                    elif distance_control_stand_bar>max_holding_distance:
                        rospy.loginfo("distance_control_stand_bar is %s"%(str(distance_control_stand_bar)))
                        rospy.loginfo("max_holding_distance is %s"%(str(max_holding_distance)))
                        rospy.logerr("the holding distance is larger than maximum holding distance")

                except:
                    rospy.logerr("something error with open_hold_flag----")

            if open_climb_flag==1:
                try:
                    distance_climb_control = rospy.get_param("distance_climb_control")
                    min_climbing_distance = rospy.get_param("min_climbing_distance")
                    max_climbing_distance = rospy.get_param("max_climbing_distance")
                    if distance_climb_control>=min_climbing_distance and distance_climb_control<=max_climbing_distance:
                        target_distance=distance_climb_control+read_line_l0_encode
                        clbpkg.Climbing_Robot_close_loop_control(ser,target_distance,read_line_encode)
                    elif distance_climb_control<min_climbing_distance:
                        rospy.loginfo("distance_climb_control is: %s"%str(distance_climb_control))
                        rospy.loginfo("min_climbing_distance is: %s"%str(min_climbing_distance))
                        rospy.logerr("climbing distance is smaller than the minimum climb distance")
                    elif distance_climb_control>max_climbing_distance:
                        rospy.loginfo("distance_climb_control is: %s"%str(distance_climb_control))
                        rospy.loginfo("max_climbing_distance is: %s"%str(max_climbing_distance))
                        rospy.logerr("climbing distance is larger than the maximum climb distance")
                except:
                    rospy.logerr("something error with open_climb_flag----")

            if open_rotation_flag==1:
                try:
                    rad_control_rotation = rospy.get_param("rad_control_rotation")
                    min_rotation_distance = rospy.get_param("min_rotation_distance")
                    max_rotation_distance = rospy.get_param("max_rotation_distance")
                    if rad_control_rotation>=min_rotation_distance and rad_control_rotation<=max_rotation_distance:
                        target_abs_encode_data=(rad_control_rotation-rotation_joint_line_equation_b)/rotation_joint_line_equation_k
                        clbpkg.Rotation_Robot_close_loop_control(ser,target_abs_encode_data,rotation_abs_encode) 
                    elif rad_control_rotation<min_rotation_distance:
                        rospy.loginfo("rad_control_rotation is: %s"%str(rad_control_rotation))
                        rospy.loginfo("min_rotation_distance is: %s"%str(min_rotation_distance))
                        rospy.logerr("the rotation angle is smaller than the minimum rotation distance")
                    elif rad_control_rotation>max_rotation_distance:
                        rospy.loginfo("rad_control_rotation is: %s"%str(rad_control_rotation))
                        rospy.loginfo("max_rotation_distance is: %s"%str(max_rotation_distance))
                        rospy.logerr("the rotation angle is larger than the maximum rotation distance")

                except:
                    rospy.logerr("something error with open_rotation_flag----")     

        else:
            try:
                ser = serial.Serial(port=climb_port, baudrate=climb_port_baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN, stopbits=1,timeout=0.3, xonxoff=0,rtscts=False,dsrdtr=False)
            except:
                rospy.logerr("Please check TRIDOF ROBOT Usb port----,I will reconnect after three seconds-----")
                open_serial_port_again_flag=1
                time.sleep(3)
            
        rate.sleep()
if __name__ == "__main__":
    main()