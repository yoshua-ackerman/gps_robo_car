#!/usr/bin/python
# -*- coding: utf-8 -*-

from pprint import pprint

import rospy
import tf_conversions
#import tf2_ros
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Twist, Vector3, TransformStamped
#from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped

from numpy import pi,cos,sin,rad2deg,hypot

from ak_param import *
## PPR #Pulse Per Rotation
## WHEEL_DIA=0.07 #[m]
## TREAD=0.44 #[m]
## MAX_V=RPS_MAX_NOGEAR/GEAR_RATIO*WHEEL_DIA*pi #[m/sec]
from gps_robo_car.msg import PWM

from path2d import Arc2D, omit_pi_pi
from gen_path2019 import gen_1lap_path

#この距離より離れるとパスに直交するような向きになりながらパス上の最短点を目指す。
#この距離よ短い外れ方をしているときは弧を描くかどうかは分からないがそれっぽい動きでパスに沿うのを目指す
BALANCE_DIST=0.75
#BALANCE_DIST=1.
def judge_reduce(dist_diff, direc_diff):
#コースに沿わすための片輪の減速量を決める
#正ならyaw正転=CCW=左輪減速
#負ならyaw逆転=CW=右輪減速
    l=dist_diff
    dth=direc_diff

    x=abs(l/BALANCE_DIST)
    if x>1.0:
        x=1.0
    xxx = (1.0-x)*omit_pi_pi(dth) + x*( omit_pi_pi(dth)+(-pi/2.0 if l>0 else +pi/2.0) )
    xxx = xxx/pi # -1.5~+1.5
    #xxx>0:CCW,正回転(左減速)
    #xxx<0;CW,負回転(右減速)
    return xxx

p1, center, p2, path = gen_1lap_path()

rospy.init_node('path_tracer')
pub_pwm_cmd=rospy.Publisher("ak_pwm_cmd", PWM, queue_size=10)
pwm_cmd=PWM(pwm=[0,0])

last_talk_t=rospy.get_time()
pub_aqtalk=rospy.Publisher("aqtalk_str", String, queue_size=3)

#pub_aqtalk.publish(String(data="スタート"))
pub_aqtalk.publish("      スタートhogehogehogehogehogehogehgoehogehogehoge")

current=0 # path[current]

start_t = rospy.get_time()
lap_begin_t = start_t
last_lap_time=0
STOP=False
def go_on_path(odm_msg):
    global current, start_t,lap_begin_t, last_lap_time, STOP
    next=(current+1)%len(path)

    if STOP==True:
        return

    pose = odm_msg.pose.pose
    x=pose.position.x
    y=pose.position.y

    in_phase, dist, dth = path[current].pose_diff(pose)
    in_phase_next, dist_next, dth_next = path[next].pose_diff(pose)
#    reduc = judge_reduce(dist, dth)
    to_be_next=False
    if in_phase==False and in_phase_next==True and path[current].end_p_dist(x,y) < path[current].start_p_dist(x,y):
        #次のパスに入っている
        to_be_next=True
        reduc = judge_reduce(dist_next, dth_next)
    else:
        reduc = judge_reduce(dist, dth)
    rospy.loginfo("%+.3f, current=%d,next=%d"%(reduc,current,next))
#    print in_phase,dist,rad2deg(dth),reduc
#    print(path[current].R,path[current].cx,path[current].cy,
#   path[current].rad_start, path[current].crad)

#    if path[current].end_p_dist(x,y)<1.00: #0.5 
    if path[current].end_p_dist(x,y)<0.25 or to_be_next==True:
        if current==1 or current==5:
            pub_aqtalk.publish("パス%dクリア、パス%dスタート、令和ポイントゲット"%(current,next))
        elif current==3 or current==7:
            lap_end_t = rospy.get_time()
            lap_time_new = lap_end_t - lap_begin_t
            pub_aqtalk.publish("センター通過、ラップタイム%.1f秒"%(lap_time_new))
            if lap_end_t - start_t > 180:
                pub_aqtalk.publish("3分経過しています")
            if lap_end_t + lap_time_new - start_t > 180:
                pub_aqtalk.publish("止まります。トータルタイム%.1f秒"%(lap_end_t-start_t))
                STOP=True
                pwm_cmd.pwm=[0,0]
                pwm_cmd.stamp=rospy.Time.now()
                pub_pwm_cmd.publish(pwm_cmd)
                rospy.signal_shutdown("GPS ROBO CAR REACHED GOAL")
                #return
            elif lap_end_t + 2*lap_time_new - start_t > 180:
                pub_aqtalk.publish("ファイナルラップ行きます")
            lap_begin_t = lap_end_t
            last_lap_time = lap_time_new
        else:
            pub_aqtalk.publish("パス%dクリア、パス%dスタート"%(current,next))
        current = next

    R=path[current].R
    T=TREAD
    inner_ratio=(2.0*R-T)/(2.0*R+T)
    rospy.loginfo("R=%.1f, inner_ratio=%.2f"%(R, inner_ratio))

    l=1.0
    r=1.0
    k=2.0
    MAX_PWM=255.
#    MAX_PWM=160.
    if reduc>0:
        l=l-reduc*k
        inner_ratio2 = 1.0-(1.0-inner_ratio)/(1.0+reduc*k)
        rospy.loginfo("inner_ratio2=%.2f"%(inner_ratio2))
        l = l*inner_ratio2
        if l <-1.0:
            l=-1.0
    elif reduc<0:
        r=r+reduc*k
        inner_ratio2 = 1.0-(1.0-inner_ratio)/(1.0-reduc*k)
        rospy.loginfo("inner_ratio2=%.2f"%(inner_ratio2))
        r = r*inner_ratio2
        if r <-1.0:
            r=-1.0
    pwm_cmd.pwm[0]=int(l*MAX_PWM)
    pwm_cmd.pwm[1]=int(r*MAX_PWM)
    pwm_cmd.stamp=rospy.Time.now()
    print pwm_cmd.pwm
    pub_pwm_cmd.publish(pwm_cmd)


sub=rospy.Subscriber("odom", Odometry, go_on_path, queue_size=10)

rospy.spin()
