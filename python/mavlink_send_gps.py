#! /usr/bin/python3

# -*- coding: utf-8 -*-
from __future__ import print_function 
import time
import math
# Force mavlink2 for yaw in GPS_INPUT...
import os
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink1 
#from pymavlink.dialects.v20 import ardupilotmega as mavlink2 

# DO NOT USE THIS CODE DIRECTLY ON A REAL ROBOT WITHOUT CHECKING WHAT IT MIGHT DO!

# See https://code.ihub.org.cn/projects/316/repository/revisions/master/entry/Tools/Vicon/vicon_mavlink.py and https://mavlink.io/en/messages/common.html

# Injecting GPS data using GPS_INPUT MAVLink message : need parameter GPS_TYPE = 14, might need ArduPilot >= 3.4.0
# If yaw support needed, need parameters AHRS_EKF_TYPE = 3 (to use EKF3), EK2_ENABLE = 0 (to disable EKF2), EK3_ENABLE = 1 (to enable EKF3), EK3_MAG_CAL = 5 (use external yaw sensor) or 6 (external yaw sensor with compass fallback), might need parameters COMPASS_USE = 0, COMPASS_USE1 = 0, COMPASS_USE2 = 0, EK3_SRC1_YAW = 2, EK3_GPS_TYPE = 0, MAVLink 2 and ArduPilot >= 4.0.0.
#
# Inject GPHDT, GPRMC via TCP->does not seem directly possible, needs to be through serial port, use MAVLink message GPS_INPUT instead...?
#
# Inject full external AHRS/INS data->EAHRS_TYPE=1 only for VectorNav AHRS (see https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_ExternalAHRS/AP_ExternalAHRS_VectorNav.cpp, https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/SIM_VectorNav.cpp), might need parameters SERIAL5_PROTOCOL = 36, SERIAL5_BAUD = 230400, AHRS_EKF_TYPE = 11, GPS_TYPE = 21, INS_GYR_CAL = 1, ArduPilot >= 4.1.0-dev...

#autopilot = mavutil.mavlink_connection("COM12", 115200)
autopilot = mavutil.mavlink_connection('tcp:127.0.0.1:5760')  
#autopilot = mavutil.mavlink_connection('udp:0.0.0.0:14550')
autopilot.wait_heartbeat()

print(autopilot.mav.gps_input_send.__doc__)

# n = 4; dt = 0.1 # To send repeatedly the messages
# t = 1000
# for i in range(0,int(t/dt)):
#     time_us = 0; gps_week_ms = 0; gps_week = 0; fix_type = 5; gps_lat = 48.41792280; gps_lon = -4.47267279; gps_alt = 150.749; gps_vel = [0,0,0]; gps_nsats = 15; yaw = 270
#     autopilot.mav.gps_input_send(time_us, 0, 0, gps_week_ms, gps_week, fix_type,
#                                int(gps_lat*1.0e7), int(gps_lon*1.0e7), gps_alt,
#                                1.0, 1.0,
#                                gps_vel[0], gps_vel[1], gps_vel[2],
#                                0.2, 1.0, 1.0,
#                                gps_nsats,
#                                yaw*100)
#     time.sleep(dt)
# autopilot.close()


