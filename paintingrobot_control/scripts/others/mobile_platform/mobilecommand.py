 #! /usr/bin/env python
# coding=utf-8
class MobileDriverCommands:
    def __init__(self):
        self.OPEN_CAN_NODE_DRIVER=(0x01,0x00)
        self.CLOSE_CAN_NODE_DRIVER=(0x02,0x00)
        self.CLEAR_ERROR_WITHOUT_DISABLE=(0xfd,0x86,0x00)
        self.CLEAR_ERROR_WITH_DISABLE=(0x01,0x80,0x00)
        self.ENABLE_DRIVER_SET_VELOCITY=(0xfd,0x0f,0x00)
        self.ENABLE_DRIVER_SET_VELOCITY_SLOW=(0x03,0x0f,0x00)
        self.ENABLE_DRIVER_SET_POSITION=(0x01,0x3f,0x10)
        self.ENABLE_DRIVER_SET_HOMING=(0x06,0x1f,0x00)
        self.DISABLE_DRIVER=(0x01,0x06,0x00)
        self.ZERO_VELOCITY=(0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00)
        self.TEST_VELOCITY_STEERING=(0x11,0x11,0x11,0x00,0x00,0x00,0x00,0x00)
        self.TEST_VELOCITY_WALKING=(0xab,0xaa,0x29,0x00,0x00,0x00,0x00,0x00)
        self.BASIC_TARGET_COMMAND=(0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00)
        self.TEST_POSITION_2PI=(0x00,0x0c,0xd0,0x00,0x00,0x00,0x00,0x00)
        self.ZERO_COMMAND=()
        # self.ENABLE_COMMAND_1 = (0x2b, 0x40, 0x60, 0x00, 0x06, 0x00)
        # self.ENABLE_COMMAND_2 = (0x2b, 0x40, 0x60, 0x00, 0x07, 0x00)
        # self.ENABLE_COMMAND_3 = (0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00)
        # self.DISENABLE_COMMAND = (0x2b, 0x40, 0x60, 0x00, 0x00, 0x00)

        # self.SET_MODE_VELOCITY = (0x2f, 0x60, 0x60, 0x00, 0x03)
        # self.SET_MODE_POSITION = (0x2f, 0x60, 0x60, 0x00, 0x08)
        # self.SET_MODE_ELECTRIC = (0x2f, 0x60, 0x60, 0x00, 0x04)

        # self.SET_PROFILE_VELOCITY = (0x23, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)

        # self.BASIC_RIGHT_TARGET_VELOCITY_COMMAND=(0x23, 0xff, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00)
        # self.BASIC_LEFT_TARGET_VELOCITY_COMMAND=(0x23, 0xff, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00)
        # #正速度
        # self.BASIC_LEFT_TEST_200_VELOCITY_COMMAND=(0x23, 0xff, 0x60, 0x00, 0xc8, 0x00, 0x00, 0x00)#200
        # self.BASIC_LEFT_TEST_600_VELOCITY_COMMAND=(0x23, 0xff, 0x60, 0x00, 0x58, 0x02, 0x00, 0x00)#600
        # self.BASIC_LEFT_TEST_1000_VELOCITY_COMMAND=(0x23, 0xff, 0x60, 0x00, 0xe8, 0x03, 0x00, 0x00)#1000
        # self.BASIC_LEFT_TEST_450_VELOCITY_COMMAND=(0x23, 0xff, 0x60, 0x00, 0xc2, 0x01, 0x00, 0x00)#450

        # self.BASIC_RIGHT_TEST_200_VELOCITY_COMMAND=(0x23, 0xff, 0x68, 0x00, 0xc8, 0x00, 0x00, 0x00)#200
        # self.BASIC_RIGHT_TEST_600_VELOCITY_COMMAND=(0x23, 0xff, 0x68, 0x00, 0x58, 0x02, 0x00, 0x00)#600
        # self.BASIC_RIGHT_TEST_1000_VELOCITY_COMMAND=(0x23, 0xff, 0x68, 0x00, 0xe8, 0x03, 0x00, 0x00)#1000
        # self.BASIC_RIGHT_TEST_450_VELOCITY_COMMAND=(0x23, 0xff, 0x68, 0x00, 0xc2, 0x01, 0x00, 0x00)#600
        # #负速度
        # self.BASIC_LEFT_TEST_NEG_200_VELOCITY_COMMAND=(0x23, 0xff, 0x60, 0x00, 0x38, 0xff, 0xff, 0xff)#-200
        # self.BASIC_LEFT_TEST_NEG_600_VELOCITY_COMMAND=(0x23, 0xff, 0x60, 0x00, 0xa8, 0xfd, 0xff, 0xff)#-600
        # self.BASIC_LEFT_TEST_NEG_1000_VELOCITY_COMMAND=(0x23, 0xff, 0x60, 0x00, 0x18, 0xfc, 0xff, 0xff)#1000
        # self.BASIC_LEFT_TEST_NEG_450_VELOCITY_COMMAND=(0x23, 0xff, 0x60, 0x00, 0x3e, 0xfe, 0xff, 0xff)#450

        # self.BASIC_RIGHT_TEST_NEG_200_VELOCITY_COMMAND=(0x23, 0xff, 0x68, 0x00, 0x38, 0xff, 0xff, 0xff)#-200
        # self.BASIC_RIGHT_TEST_NEG_600_VELOCITY_COMMAND=(0x23, 0xff, 0x68, 0x00, 0xa8, 0xfd, 0xff, 0xff)#-600
        # self.BASIC_RIGHT_TEST_NEG_1000_VELOCITY_COMMAND=(0x23, 0xff, 0x68, 0x00, 0x18, 0xfc, 0xff, 0xff)#1000
        # self.BASIC_RIGHT_TEST_NEG_450_VELOCITY_COMMAND=(0x23, 0xff, 0x68, 0x00, 0x3e, 0xfe, 0xff, 0xff)#-450
        # #
        # self.BASIC_RIGHT_TARGET_POSITION_COMMAND=(0x23, 0x7a, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00)
        # self.BASIC_LEFT_TARGET_POSITION_COMMAND=(0x23, 0x7a, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00)

        # self.BASIC_ELECTRIC_RIGHT_TARGET_TORQUE_COMMAND = (0x2b, 0x71, 0x68, 0x00, 0x00, 0x00)
        # self.BASIC_ELECTRIC_LEFT_TARGET_TORQUE_COMMAND = (0x2b, 0x71, 0x60, 0x00, 0x00, 0x00)

        # self.BASIC_LEFT_VELOCITY_FEEDBACK = (0x40, 0x69, 0x60, 0x00)
        # self.BASIC_RIGHT_VELOCITY_FEEDBACK = (0x40, 0x69, 0x68, 0x00)

        # self.BASIC_LEFT_POSITION_FEEDBACK = (0x40, 0x64, 0x60, 0x00)
        # self.BASIC_RIGHT_POSITION_FEEDBACK = (0x40, 0x64, 0x68, 0x00)

        # self.BASIC_LEFT_ELECTRIC_FEEDBACK = (0x40, 0x78, 0x00, 0x00)
        # self.BASIC_RIGHT_ELECTRIC_FEEDBACK = (0x40, 0x78, 0x00, 0x00)

        # self.BASIC_LEFT_FORCE_FEEDBACK = (0x40, 0x77, 0x60, 0x00)
        # self.BASIC_RIGHT_FORCE_FEEDBACK = (0x40, 0x77, 0x68, 0x00)
        # #VelocityDemandValue_R
        # self.BASIC_LEFT_VELOCITY_TARGET = (0x40, 0x6b, 0x60, 0x00)
        # self.BASIC_RIGHT_VELOCITY_TARGET = (0x40, 0x6b, 0x68, 0x00)

        # self.SAVE_PARAMETERS = (0x23, 0x10, 0x10, 0x01, 0x73, 0x61, 0x76, 0x65)
        # Four Abs_encoder request command

        self.REQUEST_ENCODER_1 = (0x04, 0x01, 0x01, 0x00)
        self.REQUEST_ENCODER_2 = (0x04, 0x02, 0x01, 0x00)
        self.REQUEST_ENCODER_3 = (0x04, 0x03, 0x01, 0x00)
        self.REQUEST_ENCODER_4 = (0x04, 0x04, 0x01, 0x00)