#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from ak_param import *
from gps_robo_car.msg import AkRawData, PWM

from numpy import sin,cos,arctan2,arccos,sqrt,pi, rad2deg,deg2rad


#rospy.init_node("ak_twist", log_level=rospy.DEBUG)
rospy.init_node("ak_twist", log_level=rospy.INFO)
pub_pwm = rospy.Publisher("ak_pwm_cmd", PWM, queue_size=10)
maxpwm = int(rospy.get_param("~maxpwm", 255))
if maxpwm<0 or maxpwm>255:
    maxpwm=255
rot_tgt = [0.0, 0.0]


K_p2dist = WHEEL_DIA * pi / PPR #パルス数にかけて移動距離になる係数


last_twist_receive_t = rospy.Time.now()
last_ak_raw_data = AkRawData()
last_ak_raw_data.stamp = rospy.Time.now()

#equivarent
#x=rospy.get_time()
#x=rospy.Time.now().to_sec()

def twist2pwm(twist):
    global last_twist_receive_t
    t = rospy.Time.now()
#    print (t-last_twist_receive_t).to_sec()
    #Arduinoがパンクするみたいなことはなさそうだが、あまり送信頻度を上げすぎない
    #teleop_twist_keyboard でキー押しっぱなしだと秒間30コマンド送ってくる
    if t - last_twist_receive_t < rospy.Duration(0.075):
        return
    last_twist_receive_t = t

#    print twist

    v = twist.linear.x
    omega = twist.angular.z
#    R = v / omega
    #x+を前x-を後ろとして
    #R:+:Rの中心が左側(y+)
    #R:-:Rの中心が右側(y-)
    #R/omega/v
    #+/+/+
    #-/+/-
    #+/-/-
    #-/-/+
    #Inf/0/*
    #0/*/0

#    vl = (R - TREAD/2)*omega = v -TREAD/2*omega
#    vr = (R + TREAD/2)*omega = v +TREAD/2*omega
    v_l = v - TREAD/2*omega
    v_r = v + TREAD/2*omega
    #v_r,v_lどちらかが最大速度(仮)を超えていたら
    #Rを維持してv,omegaを調整
    #   ※フィールドの傾斜や摩擦を一定としておそらく最大速度は一定でない
    #   直進時は上がり、片輪旋回のときもっとも抵抗を受けて下がる

    # ていうかパスに沿わせたくかつ最高速度にしたいときは
    # 片輪PWMMAXみたいなモードで制御するという初期の発想でよさそう
    # 片輪PWMMAXで動いているパルス数をフィードバック受けつつ
    # Rを実現するように反対輪をフィードバック制御

    #バッテリの電圧変えるだけでも最高速度変わる

    rot_l = v_l / (WHEEL_DIA / 2.0)
    rot_r = v_r / (WHEEL_DIA / 2.0)

    K_p2r = 1.0/PPR
    K_r2p = PPR

    v_max_interim = RPS_MAX * WHEEL_DIA * pi
    omega_max_interim = 2.0 * v_max_interim / TREAD

    pwm_cmd=PWM(pwm=[0,0])
    #キーボードで動かせれば目標値とかとりあえずどうでもいいんだよ？！
    if v==0 and omega==0:
        pwm_cmd.pwm[0] = 0
        pwm_cmd.pwm[1] = 0
    elif v>0 and omega==0:
        pwm_cmd.pwm[0] = +maxpwm
        pwm_cmd.pwm[1] = +maxpwm
    elif v<0 and omega==0:
        pwm_cmd.pwm[0] = -maxpwm
        pwm_cmd.pwm[1] = -maxpwm
    elif v==0 and omega>0:
        pwm_cmd.pwm[0] = -maxpwm
        pwm_cmd.pwm[1] = +maxpwm
    elif v==0 and omega<0:
        pwm_cmd.pwm[0] = +maxpwm
        pwm_cmd.pwm[1] = -maxpwm
    elif v>0 and omega>0:
        pwm_cmd.pwm[0] = 0
        pwm_cmd.pwm[1] = +maxpwm
    elif v>0 and omega<0:
        pwm_cmd.pwm[0] = +maxpwm
        pwm_cmd.pwm[1] = 0
    elif v<0 and omega>0:
        pwm_cmd.pwm[0] = -maxpwm
        pwm_cmd.pwm[1] = 0
    elif v<0 and omega<0:
        pwm_cmd.pwm[0] = 0
        pwm_cmd.pwm[1] = -maxpwm

    pwm_cmd.stamp=rospy.Time.now()
    pub_pwm.publish(pwm_cmd)


def update_v(ak_raw_data):
#bno055によるヨーレート表示したい
#キャリブレーションステータスも1行で表示したい
    global last_ak_raw_data
    new_pulses = ak_raw_data.pulses
    dt = (ak_raw_data.stamp - last_ak_raw_data.stamp).to_sec()

#    print "dt=%f, pulses="%dt,new_pulses


    vl = new_pulses[0]*K_p2dist/dt
    vr = new_pulses[1]*K_p2dist/dt
    v = (vl+vr)/2.0
    omega = (vr-vl) / TREAD
    omega_deg = rad2deg(omega)
    if omega!=0:
        print "v,omega=%+5.2f,%+6.1f, R=%+.1f"%(v,omega_deg,v/omega)
    else:
        print "v,omega=%+5.2f,%+6.1f"%(v,omega_deg)

    last_ak_raw_data = ak_raw_data
#PID
#目標値が大きく変わったらI項をリセットする要素を入れる


sub_twist = rospy.Subscriber("cmd_vel", Twist, twist2pwm, queue_size=10)
sub_raw_data = rospy.Subscriber("ak_raw_data", AkRawData, update_v, queue_size=10)

v_max_interim = RPS_MAX * WHEEL_DIA * pi
omega_max_interim = 2.0 * v_max_interim / TREAD
print v_max_interim,omega_max_interim

rospy.spin()

