#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

from ak_param import *
from gps_robo_car.msg import AkRawData, PWM

from numpy import sin,cos,arctan2,arccos,sqrt,pi, rad2deg,deg2rad

import serial

#SERIAL_PORT = '/dev/ttyUSB0'
#rospy.init_node("ak_driver", log_level=rospy.DEBUG)
rospy.init_node("ak_driver", log_level=rospy.INFO)
SERIAL_PORT=rospy.get_param('~port', "/dev/ttyUSB0")
#s = serial.Serial(SERIAL_PORT, 115200, timeout=1)
s = serial.Serial(SERIAL_PORT, 9600, timeout=1)

joy_override=False
def joy_cb(joy_msg):
    global joy_override
    for i in range(len(joy_msg.buttons)):
        if joy_msg.buttons[i]!=0:
            joy_override=True
            #some button(s) pressed -> send motor command manually
            pwmL=int(joy_msg.axes[1]*255.)
            pwmR=int(joy_msg.axes[3]*255.)
            cmd_str="%d,%d\n"%(pwmL,pwmR)
            rospy.logdebug("#PWM(joy override), %s",cmd_str)
            s.write(cmd_str)
            return
    joy_override=False
sub_joy=rospy.Subscriber("joy", Joy, joy_cb, queue_size=3)


def pwm_cmd_cb(cmd):
    if joy_override==True:
        return
    def omit255(x):
        if x<-255:
            return -255
        if x>255:
            return 255
        return x
    pwmL=omit255(cmd.pwm[0])
    pwmR=omit255(cmd.pwm[1])
    cmd_str="%d,%d\n"%(pwmL,pwmR)
    rospy.logdebug("#PWM(cmd by topic), %s",cmd_str)
    s.write(cmd_str)
    return
sub_pwmcmd=rospy.Subscriber("ak_pwm_cmd", PWM, pwm_cmd_cb, queue_size=10)

pub_raw_data=rospy.Publisher("ak_raw_data", AkRawData, queue_size=10)

pub_aqtalk=rospy.Publisher("aqtalk_str", String, queue_size=3)


last_responce_t=rospy.Time.now()
last_warn_t=rospy.Time.now()
def res_chk(event):
    global last_responce_t, last_warn_t
    t = rospy.Time.now()
    if t-last_responce_t > rospy.Duration(1.5):
        if t-last_warn_t > rospy.Duration(7.5):
            pub_aqtalk.publish("    コントローラ応答していません")
            last_warn_t=rospy.Time.now()

with s:
    ak_raw_data=AkRawData()
    pls_ready=False
    cur_ready=False
    hdeg_ready=False
    x9stat_ready=False

    rospy.Timer(rospy.Duration(1), res_chk)

    while not rospy.is_shutdown():
        try:
            line_str=s.readline()
            words = line_str.split(',')
#            print words
            if words[0]=="#PLS" and len(words)==3:
                plsL=int(words[1])
                plsR=int(words[2])
                #print words[0],plsL,plsR
                ak_raw_data.pulses=(plsL,plsR)
                pls_ready=True
            elif words[0]=="#CUR" and len(words)==3:
                curL=float(words[1])
                curR=float(words[2])
                #print words[0],curL,curR
                ak_raw_data.currents=(curL,curR)
                cur_ready=True
            elif words[0]=="#HDEG" and len(words)==2:
                hdeg=float(words[1])
#                if hdeg>180.:
#                  hdeg-=360.
                #print words[0],hdeg
                ak_raw_data.hdeg=hdeg
                ak_raw_data.yawrate=0. #
                hdeg_ready=True
            elif words[0]=="#CLB_AMGS" and len(words)==5:
                calib_A=int(words[1])
                calib_M=int(words[2])
                calib_G=int(words[3])
                calib_S=int(words[4])
                #print("#A%d/M%d/G%d/S%d"%(calib_A,calib_M,calib_G,calib_S))
                ak_raw_data.accel_stat = calib_A
                ak_raw_data.mag_stat = calib_M
                ak_raw_data.gyro_stat = calib_G
                ak_raw_data.sys_stat = calib_S
                x9stat_ready=True
            else:
                print("bad packet?: %s"%(line_str))
                continue
        except Exception as e: #python3?
        #except Exception,e: #python2?
                rospy.logerr(e)
                rospy.logerr(words)
                rospy.logerr(len(words))
                rospy.logerr("COM ERROR ???")

        if pls_ready==True and cur_ready==True and hdeg_ready==True and x9stat_ready==True:
            global last_responce_t
            last_responce_t = rospy.Time.now()
            ak_raw_data.stamp = last_responce_t
            pub_raw_data.publish(ak_raw_data)
            pls_ready=False
            cur_ready=False
            hdeg_ready=False
            x9stat_ready=False

