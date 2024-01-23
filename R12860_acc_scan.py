#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Move Joint
"""

import os
import sys
import time
import math

sys.path.append('/mnt/c/Users/Belle II Melbourne/Desktop/HyperK/wihann')
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI
from ArmPMT_class_fixed2 import ArmPMT


#######################################################
"""
Just for test example
"""
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        working_dir = "/mnt/c/Users/Belle II Melbourne/Desktop/HyperK"
        parser = ConfigParser()
        parser.read(f'{working_dir}/wihann/xArm-Python-SDK-master/example/wrapper/robot.conf')
        # parser.read(f'../robot.conf')
        ip = parser.get('xArm', 'ip')
        print(ip)
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)
########################################################


arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0) #0
arm.set_state(state=0)

xArmPMT = ArmPMT()

arm.reset(wait=True)

tcp_speed = 40
offset = [0, 0, 0, 0, 0, 0]
# arm.set_tcp_offset(offset=offset)
print(arm.tcp_offset)

azimuthals = [0, 90, 180, 270]
# azimuthals = [0]
zeniths = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60]
# zeniths = [0, 10.276, 20.547, 30.810, 41.132]


# Joint angles
HomeTrue = [0, 0, 0, 0, 0]
Home = [0, -117, 5, 0, 0]

# - Buffer Joint angles
B0 = [0.0, -95.0, -127.1, 150.5, 0.0]
B1 = [0.0, -117.0, 5.0, 0.0, 0.0]
B2 = [180.0, -117.0, 5.0, 0.0, 0.0]
B3 = [180.0, 0.0, 0.0, 0.0, 0.0]
B4 = [180.0, 30.0, -50.0, -2.41, 0.0]
B5 = [180.0, 30.0, -115.0, -2.41, 0.0]
# B6 = [180, 68.00, -223.00, -2.10, 0]

Buffers = [B0, B1, B2, B3, B4, B5]

arm.set_servo_angle(angle=Home, speed=tcp_speed, wait=True)

print(" ")
print("Joint angles: ", arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
print("TCP position: ", arm.get_position(), arm.get_position_aa())

passed_rob_change = False

for theta in zeniths:
    for phi in azimuthals:
        target_name = f'phi{phi}_theta{theta}'

        print(" ")
        print(f'{target_name}')

        if math.radians(theta) >= xArmPMT.Urob_change and not passed_rob_change:
            passed_rob_change = True
            print('passed_rob_change')
            for buffer in Buffers:
                arm.set_servo_angle(angle=buffer, speed=tcp_speed, wait=True)

        cart_target = xArmPMT.get_cart(theta)
        print(f"cart_target: {cart_target}")

        joint_target = arm.get_inverse_kinematics(cart_target)
        print(f"inverse_kinematics: {joint_target}")
        
        try:
            joint_target = xArmPMT.rotate_base(phi, joint_target[1])
            print(f"joint_target: {joint_target}")
        except IndexError:
            print("Nope")


        arm.set_servo_angle(angle=joint_target, speed=tcp_speed, wait=True)

        print("Joint angles: ", arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
        print("TCP position: ", arm.get_position(), arm.get_position_aa())
        time.sleep(2)


for buffer in reversed(Buffers[4:]):
    arm.set_servo_angle(angle=buffer, speed=tcp_speed, wait=True)

arm.reset(wait=True)
arm.disconnect()