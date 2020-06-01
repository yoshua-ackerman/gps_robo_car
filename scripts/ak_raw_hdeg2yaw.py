#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ak_raw_data から hdeg(=azimuth、時計回りが正、北が0度) を抽出して
# yaw(半時計回りが正、東が0度)に変換する
# rtk_compass に subさせる。rtk_compass を ak_raw_data 非依存にする
# ak_driverに組み込んでもいいが、とりあえずbag再生時にyawトピックを出したいので

import rospy
from std_msgs.msg import Float64
from gps_robo_car.msg import AkRawData

from numpy import pi, deg2rad

def omit_pi_pi(rad): #range -pi~+pi
    if rad < -pi or rad > +pi:
        rad = rad%(2.0*pi)
        if rad >= pi:
            rad = rad-(2.0*pi)
    return rad

rospy.init_node("ak_azimuth2yaw", log_level=rospy.INFO)
pub = rospy.Publisher("yaw_raw" ,Float64, queue_size=10)
def az2yaw(ak_raw_msg):
    hdeg=ak_raw_msg.hdeg
    yaw_rad = omit_pi_pi(-deg2rad(hdeg) + pi/2.0)
    rospy.logdebug("%5.1f -> %+.3f"%(hdeg,yaw_rad))
    pub.publish(Float64(yaw_rad))
sub = rospy.Subscriber("ak_raw_data", AkRawData, az2yaw, queue_size=10)
rospy.spin()

