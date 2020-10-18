#!/usr/bin/python
# -*- coding: utf-8 -*-

#from pprint import pprint
from time import time
if __name__ == '__main__':
    start_t=time()

import rospy
#import tf_conversions
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

from path2d import Line2D,Arc2D, omit_pi_pi, \
    path_transform2D, path_align_dir, path_sort, path_total_len, path_rosvis
from gen_path_dpreiwa import gen_1lap_path

#この距離より離れるとパスに直交するような向きになりながらパス上の最短点を目指す。
#この距離よ短い外れ方をしているときは弧を描くかどうかは分からないがそれっぽい動きでパスに沿うのを目指す
BALANCE_DIST=0.75
#BALANCE_DIST=1.
def judge_reduce(dist_diff, direc_diff):
#コースに沿わすための片輪の減速量を決める
#正ならyaw正転=CCW=左輪減速
# 0ならば両輪全速(コースにきっちり沿っている)
#負ならyaw逆転=CW=右輪減速
    l=dist_diff
    dth=direc_diff

    x=abs(l/BALANCE_DIST)
    if x>1.0:
        x=1.0
    xxx = (1.0-x)*omit_pi_pi(dth) + x*( omit_pi_pi(dth)+(-pi/2.0 if l>0 else +pi/2.0) )
    xxx = xxx/pi # -1.5~+1.5
    #xxx>0:CCW,正回転(左輪減速)
    #xxx=0:両輪全速の前進
    #xxx<0;CW,負回転(右輪減速)
    return xxx


class PathTracer(object):
    def __init__(self, path, end_closeness=0.25):
        self.path = path
        self.end_closeness = end_closeness
        self.start_t = None
        self.lap_begin_t = None
        self.last_lap_time = None #使ってなくない？
        self.STOP=True
        self.current=0
        self.next=1 # nextという組み込み関数があるのでメソッド内ローカル変数名などにnextを使わないよう注意

        self.pub_pwm_cmd=rospy.Publisher("ak_pwm_cmd", PWM, queue_size=10)
        self.pwm_cmd=PWM(pwm=[0,0])

        self.last_talk_t=rospy.get_time()
        self.pub_aqtalk=rospy.Publisher("aqtalk_str", String, queue_size=3)

        self.sub=rospy.Subscriber("odom", Odometry, \
                                  self.go_on_path, queue_size=10)

    def prompt(self, node_start_time=None):
        if node_start_time != None:
            #モジュールインポートやクラス定義などの初期化に時間がかかるようなのでその計測用
            #6-7秒かかったり、4-5秒で済んだり？
            node_ready_time = time() - node_start_time
        rospy.sleep(0.25)
        #ある程度sleepしないとなぜか無視される。また、はじめの方の音は出ない(低レベルドライバの遅れ？)
        self.pub_aqtalk.publish("ready, スタート準備できました") 
        #for python3, use input()
        raw_input("Ready%s hit enter:"%("(%.1fsec)"%(node_ready_time) if node_start_time!=None else "" ))
        #self.pub_aqtalk.publish(String(data="スタート"))
        self.pub_aqtalk.publish("スタートォッ！")
        self.start_t = rospy.get_time()
        self.lap_begin_t = self.start_t
        self.last_lap_time=0
        self.STOP=False

    def go_on_path(self, odm_msg):
        self.next=(self.current+1)%len(self.path)

        if self.STOP==True:
            return

        pose = odm_msg.pose.pose
        x=pose.position.x
        y=pose.position.y

        in_phase, dist, dth = self.path[self.current].pose_diff(pose)
        in_phase_next, dist_next, dth_next = self.path[self.next].pose_diff(pose)
#       reduc = judge_reduce(dist, dth)
        to_be_next=False
        if in_phase==False and in_phase_next==True and \
            self.path[self.current].end_p_dist(x,y) < \
            self.path[self.current].start_p_dist(x,y):
            #次のパスに入っている
            to_be_next=True
            reduc = judge_reduce(dist_next, dth_next)
        else:
            reduc = judge_reduce(dist, dth)
        rospy.loginfo("reduc:%+.3f(%s), cur=%d,next=%d"%(reduc,("CCW" if reduc >0 else "CW"),self.current,self.next))
        rospy.loginfo("in_pahse:%s, dist:%+.02f, deg_diff:%+.1f, -> reduc:%+.2f"%(in_phase,dist,rad2deg(dth),reduc))
#       print(self.path[current].R, self.path[current].cx, self.path[current].cy,
#       self.path[current].rad_start, self.path[current].crad)

        if self.path[self.current].end_p_dist(x,y)<self.end_closeness or \
            to_be_next==True:

            # stop, sound output and so on
            self.segment_end_act()

            # next segment
            self.current = self.next

        if type(self.path[self.current])==Arc2D:
            R=self.path[self.current].R
            T=TREAD
            inner_ratio=(2.0*R-T)/(2.0*R+T)
            rospy.loginfo("R=%.1f, inner_ratio=%.2f"%(R, inner_ratio))

        #inner_ratio: 円弧パス上で位置も向きも沿ってる場合の内輪の外輪(常に速度MAX)に対する速度比
        #inner_ratio2:パスに沿ってないときに沿わすための減速との合成比
        #現状のやり方:パスへの位置と方向を合わすためにどちらの車輪を減速するか決めているが
        #それがRに沿わすための減速すべき車輪と反対であってもRに沿わすための減速比を適用しているのはおかしいかもしんない
        #大きく破綻するような動きはないかもしれないが過修正な動きをもたらしてふらついたりしているかも

        l=1.0
        r=1.0
        k=2.0
        k=2.25
#        k=3.0 #その場で旋回するなど向きを直す動きが強まる、…といいな -> ふらふらするような？
        MAX_PWM=255.
        #MAX_PWM=160.
        if reduc>0:
            l=l-reduc*k
            if type(self.path[self.current])==Arc2D:
                inner_ratio2 = 1.0-(1.0-inner_ratio)/(1.0+reduc*k)
                rospy.loginfo("inner_ratio2=%.2f"%(inner_ratio2))
                l = l*inner_ratio2
            if l <-1.0:
                l=-1.0
        elif reduc<0:
            r=r+reduc*k
            if type(self.path[self.current])==Arc2D:
                inner_ratio2 = 1.0-(1.0-inner_ratio)/(1.0-reduc*k)
                rospy.loginfo("inner_ratio2=%.2f"%(inner_ratio2))
                r = r*inner_ratio2
            if r <-1.0:
                r=-1.0
        self.pwm_cmd.pwm[0]=int(l*MAX_PWM)
        self.pwm_cmd.pwm[1]=int(r*MAX_PWM)
        self.pwm_cmd.stamp=rospy.Time.now()
        rospy.loginfo("pwm=[L:%+4d, R:%+4d]"%(self.pwm_cmd.pwm[0], self.pwm_cmd.pwm[1]))
        self.pub_pwm_cmd.publish(self.pwm_cmd)
    def segment_end_act(self):
        if self.current>=len(self.path)-1:
            end_t = rospy.get_time()
            t_all = end_t - self.start_t
            self.pub_aqtalk.publish("止まります。トータルタイム%.1f秒"%(t_all))
            self.stop_and_shutdown()
    def stop_and_shutdown(self):
        self.STOP=True
        self.pwm_cmd.pwm=[0,0]
        self.pwm_cmd.stamp=rospy.Time.now()
        self.pub_pwm_cmd.publish(self.pwm_cmd)
        rospy.signal_shutdown("GPS ROBO CAR REACHED GOAL")
        #return


class PathTracerDPReiwa(PathTracer):
    def __init__(self, path, end_closeness, \
                 reiwa_pt_check_indices, lap_end_check_indices):
        super(PathTracerDPReiwa, self).__init__(path, end_closeness) # python2.7
        #super().__init__(path, end_closeness) # python3.x
        self.reiwa_pt_check_indices = reiwa_pt_check_indices
        self.lap_end_check_indices = lap_end_check_indices
    def segment_end_act(self):
        if self.current in self.reiwa_pt_check_indices:
            self.pub_aqtalk.publish("パス%dクリア、パス%dスタート、令和ポイント獲得"%(self.current,self.next))
        elif self.current in self.lap_end_check_indices:
            self.lap_end_t = rospy.get_time()
            lap_time_new = self.lap_end_t - self.lap_begin_t
            self.pub_aqtalk.publish("センター通過、ラップタイム%.1f秒"%(lap_time_new))
            if self.lap_end_t - self.start_t > 180:
                self.pub_aqtalk.publish("3分経過しています")
            if self.lap_end_t + lap_time_new - self.start_t > 180:
                #もう1周行くとタイムオーバーなので止まる
                self.pub_aqtalk.publish("止まります。トータルタイム%.1f秒"%(self.lap_end_t-self.start_t))
                self.stop_and_shutdown()
            elif self.lap_end_t + 2*lap_time_new - self.start_t > 180:
                #あと1周は行けるが2周行くとタイムオーバーになってしまう　-> ファイナルラップ
                self.pub_aqtalk.publish("ファイナルラップ行きます")
            self.lap_begin_t = self.lap_end_t
            self.last_lap_time = lap_time_new #いちおう値をとっているが特に使ってない
        else:
            self.pub_aqtalk.publish("パス%dクリア、パス%dスタート"%(self.current,self.next))

if __name__ == '__main__':
    #start_t=time()
    p1, center, p2, path = gen_1lap_path()
    print "path total length = %.1f[m]"%(path_total_len(path))

    rospy.init_node('path_tracer')

    path_rosvis(path, "static_path")

    path_tracer = PathTracerDPReiwa(path, 0.125,(1,5),(3,7))
    path_tracer.prompt(start_t)
    rospy.spin()

