#! /usr/bin/python3

import time
import math
import os
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil

import rospy
from hemisphere_v500.msg import StampedString

# DO NOT USE THIS CODE DIRECTLY ON A REAL ROBOT WITHOUT CHECKING WHAT IT MIGHT
# DO!

# See
# https://code.ihub.org.cn/projects/316/repository/revisions/master/entry/Tools/Vicon/vicon_mavlink.py
# and https://mavlink.io/en/messages/common.html

# Injecting GPS data using GPS_INPUT MAVLink message : need parameter GPS_TYPE
# = 14, might need ArduPilot >= 3.4.0 If yaw support needed, need parameters
# AHRS_EKF_TYPE = 3 (to use EKF3), EK2_ENABLE = 0 (to disable EKF2), EK3_ENABLE
# = 1 (to enable EKF3), EK3_MAG_CAL = 5 (use external yaw sensor) or 6
# (external yaw sensor with compass fallback), might need parameters
# COMPASS_USE = 0, COMPASS_USE1 = 0, COMPASS_USE2 = 0, EK3_SRC1_YAW = 2,
# EK3_GPS_TYPE = 0, MAVLink 2 and ArduPilot >= 4.0.0.
#
# Inject GPHDT, GPRMC via TCP->does not seem directly possible, needs to be
# through serial port, use MAVLink message GPS_INPUT instead...?
#
# Inject full external AHRS/INS data->EAHRS_TYPE=1 only for VectorNav AHRS (see
# https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_ExternalAHRS/AP_ExternalAHRS_VectorNav.cpp,
# https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/SIM_VectorNav.cpp),
# might need parameters SERIAL5_PROTOCOL = 36, SERIAL5_BAUD = 230400,
# AHRS_EKF_TYPE = 11, GPS_TYPE = 21, INS_GYR_CAL = 1, ArduPilot >= 4.1.0-dev...

class NmeaGPStoMavlink:
    def __init__(self, autopilot, sourceTopic):
        self.autopilot = autopilot
        self.sub       = rospy.Subscriber(sourceTopic, StampedString,
                                          self.handle_message)
        self.msgs = {'GPGGA': None,
                     'GPVTG': None,
                     'PSAT' : None}

rospy.init_node('gps_pixhawk_bridge')

gpsTopic = rospy.get_param('gps_source_topic', '/gnss_v500/nmea')

# autopilot = mavutil.mavlink_connection('tcp:127.0.0.1:5760')  
# autopilot.wait_heartbeat()

def handle_nmea(autopilot, msg, msgsDict):
    print(msg.data, end='', flush=True)

sub = rospy.Subscriber(gpsTopic, StampedString, lambda msg: handle_nmea(None, msg))

rospy.sleep()

# autopilot.close()
