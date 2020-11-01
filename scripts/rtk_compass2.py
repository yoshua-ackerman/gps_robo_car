#!/usr/bin/python
# -*- coding: utf-8 -*-


#DONE:(AkRawData->)yaw_raw をサブスクリプトして yaw_offset_raw2rtkcmps をパブリッシュする
#DONE:LINE_STRIPのマーカ表示をする。1点ごとにsingle/float/fixの色付けをする 
#DONE:  -> LINE LISTでいいんじゃないか。点毎に色付け、点のペアで色が違う場合グラデーション
#TODO:poseのマーカををfix度合に応じて色付けする -> LINE_STRIP のマーカにするかもしくはもう不要
#TODO:
#TODO:fix率、float率、single率、無応答率を見れるようにする


import ak_param


from numpy import cos,sin,arctan2,hypot,rad2deg,pi
from pprint import pprint

from utm_coordinate import GEOD,ll2xy,xy2ll,\
                    UTM_FRAME_NAME,\
                    WORK_ORIGIN_TOPIC_NAME,\
                    WORK_ORIGIN_FRAME_NAME

import rospy
from std_msgs.msg import Header,Float64
from geometry_msgs.msg import Pose2D, Point, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path #, Odometry
# pos_estimator には PoseWithCovarianceStamped で位置を伝える
# はじめは Odometry を使ってrvizでの表示も兼ねて伝達しようと考えていたが、
# Markerでより詳細なrvizの表示を実装したので、Odometry でなくてよい

import tf2_ros
import tf_conversions

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

#応答すべき時間、無応答時間の計測に使う
#受信機設定が1Hzならば1.0、5Hzならば0.2
PERIOD_ASSUMED=1.0

def omit_pi_pi(rad): #range -pi~+pi
    if rad < -pi or rad > +pi:
        rad = rad%(2.0*pi)
        if rad >= pi:
            rad = rad-(2.0*pi)
    return rad

class RtkCompass(object):

    fix_str=("------","single","float ","*FIX* ")
    single_color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
    float_color  = ColorRGBA(0.8, 0.5, 0.0, 1.0)
    fix_color    = ColorRGBA(0.0, 0.5, 0.0, 1.0)
    unknown_color= ColorRGBA(0.5, 0.5, 0.5, 0.5)

    def __init__(self, period_assumed=PERIOD_ASSUMED):
        rospy.init_node('Rtk_compass', anonymous=True)

        self.period_assumed=period_assumed

        self.work_origin=Pose2D(0.0, 0.0, 0.0)
        self.t0=rospy.get_time()

        self.L_fix_time=0.0
        self.L_float_time=0.0
        self.L_single_time=0.0
        self.R_fix_time=0.0
        self.R_float_time=0.0
        self.R_single_time=0.0

        self.dual_fix_time=0.0 # include miss-fix
        self.perfect_dual_fix_time=0.0 # reliable: range of 2 antena is good

        self.L_fix_stat = None
        self.R_fix_stat = None

        self.L_old=None
        self.L_new=None
        self.R_old=None
        self.R_new=None

        self.L_last_t=None
        self.R_last_t=None

        self.Lx=None
        self.Ly=None
        self.Rx=None
        self.Ry=None
        self.Cx=None
        self.Cy=None

        self.theta=None
        self.distance=None #between 2 antenas
        self.lr_real_dist = ak_param.LR_DIST
        self.quat=None
        self.variance=0.0
        #1. Fix and Fix
        #1-1. variance = 1.0 - abs(distance-lr_real_dist)/lr_real_dist
        #(1-2. if variance<0 in 1-1. L,R 多分どちらかは正しい。ホイールオドメトリと合わせて推測したいが、現状難しいのでやらない)
        #
        #2. Fix and Float
        # Fix しているほうを信じてyaw_raw値とFixしている位置から中心位置を算出
        # ただし variance = 0.5
        # 向きはセンサ+fix時に得た補正、センサ情報が古くなければ向きに関するvariance は高くする
        #
        #3. Float and Float
        # とりあえず２つのアンテナ位置から1-1と同様に算出するが、max variance = 0.25
        #
        #(ALL) varianceが低い場合、向きはセンサ値+fix時に得た補正、センサ情報が古くなければ向きに関するvariance は高くする

        self.yaw_raw=None
        self.yaw_raw_offset=0.0
        self.yaw_raw_last_t=0.0

        self.seq=0

        self.path=Path()
        self.poses=[]
        #for visualize by LINE_LIST
        self.point_pairs=[] # L,R antena pos
        self.color_pairs=[]
        #self.dirs_rtk=[]
        self.dirs=[] # directions by field-magnetic sensor(yaw_raw + yaw_raw_offset)

        self.sub_worig = rospy.Subscriber(WORK_ORIGIN_TOPIC_NAME, Pose2D, self.cb_worigin, queue_size=2)
        self.sub_L = rospy.Subscriber('gps_sol_L', NavSatFix, self.cb_L, queue_size=10)
        self.sub_R = rospy.Subscriber('gps_sol_R', NavSatFix, self.cb_R, queue_size=10)
        self.sub_yaw_raw = rospy.Subscriber('yaw_raw', Float64, self.cb_yaw_raw, queue_size=10)

        self.pub_yaw_raw_offset = rospy.Publisher('yaw_raw_offset', Float64, queue_size=10)
        self.pub_path = rospy.Publisher('point_log', Path, queue_size=10)
        self.pub_rtk_odom = rospy.Publisher('rtk_odom', PoseWithCovarianceStamped, queue_size=10)
        self.pub_mkarr = rospy.Publisher('rtk_markers', MarkerArray, queue_size=10)

        self.tf_br = tf2_ros.TransformBroadcaster()

    def cb_yaw_raw(self, yaw_raw_msg):
        self.yaw_raw = yaw_raw_msg.data
        self.yaw_raw_last_t = rospy.get_time()

    def calc_pose(self):
        t_now = rospy.get_time()
        t = t_now-self.t0

        self.theta=arctan2(self.Rx-self.Lx,-(self.Ry-self.Ly))
        self.distance=hypot(self.Rx-self.Lx, self.Ry-self.Ly)

        fix_variance = 1.0 - abs(self.lr_real_dist - self.distance)/self.lr_real_dist
        if fix_variance < 0.0:
            fix_variance = 0.0

        if self.L_fix_stat==3 and self.R_fix_stat==3:
            self.dual_fix_time += self.period_assumed
            self.Cx=(self.Lx+self.Rx)/2.0
            self.Cy=(self.Ly+self.Ry)/2.0
            if fix_variance >= 0.75:
                self.perfect_dual_fix_time += self.period_assumed
                if self.yaw_raw != None and t_now - self.yaw_raw_last_t < 0.2:
                # yaw_raw 取得と rtk の解が得られるタイミングは同期せずにずれるので
                # yaw_raw の値を補完するか、
                # yaw_raw取得->rtk解 or rtk解->yaw_raw取得 のうち時間差が少ないほうでyaw_raw_offsetを更新すべきだが、
                # 今はとりあえずこのタイミングで yaw_raw_offset を更新する
                    self.yaw_raw_offset = self.theta - self.yaw_raw
                    self.pub_yaw_raw_offset.publish(Float64(self.yaw_raw_offset))
            #elif fix_variance <0.75 and fix_variance >=0.5:
                # 何もしない。向きは2アンテナの位置から算出したものをとりあえず信用するが
                # yaw_raw の補正値更新はしない。もっと良かった状態のを使う
                #
                # 補正値なしのyaw_raw値が存在しうるがどうする？
                #　とりあえず実験走行は必ずperfect_fixさせてから
            elif fix_variance < 0.5 and t_now - self.yaw_raw_last_t < 0.2:
                self.theta = self.yaw_raw + self.yaw_raw_offset
                if fix_variance <0.25:
                    fix_variance=0.25
        elif self.L_fix_stat==3 and t_now - self.yaw_raw_last_t < 0.2:
            self.theta = self.yaw_raw + self.yaw_raw_offset
            self.Cx = self.Lx - self.lr_real_dist/2.0 * cos(self.theta + pi/2.0)
            self.Cy = self.Ly - self.lr_real_dist/2.0 * sin(self.theta + pi/2.0)
            fix_variance = fix_variance *0.75
            if fix_variance <0.5:
                fix_variance=0.5
        elif self.R_fix_stat==3 and t_now - self.yaw_raw_last_t < 0.2:
            self.theta = self.yaw_raw + self.yaw_raw_offset
            self.Cx = self.Rx - self.lr_real_dist/2.0 * cos(self.theta - pi/2.0)
            self.Cy = self.Ry - self.lr_real_dist/2.0 * sin(self.theta - pi/2.0)
            fix_variance = fix_variance *0.75
            if fix_variance <0.5:
                fix_variance=0.5
        else:
            if t_now - self.yaw_raw_last_t < 0.2:
                self.theta = self.yaw_raw + self.yaw_raw_offset
            self.Cx=(self.Lx+self.Rx)/2.0
            self.Cy=(self.Ly+self.Ry)/2.0
            fix_variance = fix_variance *0.5
            if fix_variance >0.25:
                fix_variance=0.25

        self.quat = tf_conversions.transformations.quaternion_from_euler(0,0,self.theta)

        L_fix_str = RtkCompass.fix_str[self.L_fix_stat]
        R_fix_str = RtkCompass.fix_str[self.R_fix_stat]

        print L_fix_str, R_fix_str, "%5.3f[m],%.2f[deg], (Cx,Cy)=(%+.2f,%+.2f), fix_reliable=%.2f"%(self.distance, rad2deg(self.theta), self.Cx,self.Cy, fix_variance)

        self.seq = self.seq+1
        header=Header()
        header.seq=self.seq
        header.stamp=rospy.Time.now()
        if self.work_origin.x==0.0 and self.work_origin.y==0.0:
            header.frame_id = UTM_FRAME_NAME
        else:
            header.frame_id = WORK_ORIGIN_FRAME_NAME

        self.marker(header)

        #odom型じゃないけどodomみたいなもんなんで
        rtk_odom = PoseWithCovarianceStamped()
        rtk_odom.header=header
        rtk_odom.pose.pose.position.x = self.Cx
        rtk_odom.pose.pose.position.y = self.Cy
        rtk_odom.pose.pose.position.z = 0.0
        rtk_odom.pose.pose.orientation.x = self.quat[0]
        rtk_odom.pose.pose.orientation.y = self.quat[1]
        rtk_odom.pose.pose.orientation.z = self.quat[2]
        rtk_odom.pose.pose.orientation.w = self.quat[3]
        rtk_odom.pose.covariance[0] = fix_variance #x,y
        if t_now - self.yaw_raw_last_t < 0.2:
        #theta for convenience
            rtk_odom.pose.covariance[1] = fix_variance
        else:
            rtk_odom.pose.covariance[1] = 0
        self.pub_rtk_odom.publish(rtk_odom)

        p_c=PoseStamped()
        p_c.header=header
        p_c.pose.position.x = self.Cx
        p_c.pose.position.y = self.Cy
        p_c.pose.position.z = 0.0
        p_c.pose.orientation.x = self.quat[0]
        p_c.pose.orientation.y = self.quat[1]
        p_c.pose.orientation.z = self.quat[2]
        p_c.pose.orientation.w = self.quat[3]

        self.path.header=header
        self.poses.append(p_c)
        if len(self.poses)>200:
            self.poses.pop(0)
        self.path.poses = self.poses
        self.pub_path.publish(self.path)

        t = TransformStamped()
        t.header = header
        t.child_frame_id = "base_footprint_rtk"
        t.transform.translation.x = self.Cx
        t.transform.translation.y = self.Cy
        t.transform.translation.z = 0.0
        t.transform.rotation.x = self.quat[0]
        t.transform.rotation.y = self.quat[1]
        t.transform.rotation.z = self.quat[2]
        t.transform.rotation.w = self.quat[3]
        self.tf_br.sendTransform(t)

    def cb_L(self, sol_L):
        self.L_last_t=rospy.get_time()
        self.L_old = self.L_new
        self.L_new = sol_L
        self.L_fix_stat = self.L_new.position_covariance_type
        if self.L_fix_stat==1:
            self.L_single_time += self.period_assumed
        elif self.L_fix_stat==2:
            self.L_float_time += self.period_assumed
        elif self.L_fix_stat==3:
            self.L_fix_time += self.period_assumed
        self.Lx, self.Ly = ll2xy(sol_L.longitude, sol_L.latitude)
        # if work_origin was loaded, offset to origin
        self.Lx = self.Lx - self.work_origin.x
        self.Ly = self.Ly - self.work_origin.y
        #pprint(sol_L)
        if self.R_new==None:
            return
        if self.L_last_t - self.R_last_t<0.1 and self.L_last_t > self.R_last_t:
#            rospy.logdebug("L %.1f"%(self.L_last_t - self.t0))
            self.calc_pose()

    def cb_R(self, sol_R):
        self.R_last_t = rospy.get_time()
        self.R_old = self.R_new
        self.R_new = sol_R
        self.R_fix_stat = self.R_new.position_covariance_type
        if self.R_fix_stat==1:
            self.R_single_time += self.period_assumed
        elif self.R_fix_stat==2:
            self.R_float_time += self.period_assumed
        elif self.R_fix_stat==3:
            self.R_fix_time += self.period_assumed
        self.Rx, self.Ry = ll2xy(sol_R.longitude, sol_R.latitude)
        # if work_origin was loaded, offset to origin
        self.Rx = self.Rx - self.work_origin.x
        self.Ry = self.Ry - self.work_origin.y
        #pprint(sol_R)
        if self.L_new==None:
            return
        if self.R_last_t - self.L_last_t<0.1 and self.R_last_t > self.L_last_t:
#            rospy.logdebug("R %.1f"%(self.R_last_t - self.t0))
            self.calc_pose()

    def cb_worigin(self, worigin):
        self.work_origin=worigin

    def marker(self, header):
        mkarr = MarkerArray()
        markers=[Marker(type=Marker.ARROW),\
                Marker(type=Marker.CYLINDER),Marker(type=Marker.CYLINDER),\
                Marker(type=Marker.TEXT_VIEW_FACING),Marker(type=Marker.TEXT_VIEW_FACING),\
                Marker(type=Marker.LINE_LIST), \
                Marker(type=Marker.LINE_LIST) ]
        for i in range(7):
            markers[i].header=header
            markers[i].ns="RTK-info-marker"
            markers[i].id =i
            markers[i].action = Marker.ADD
            markers[i].pose.position.z = 0.0
            markers[i].pose.orientation.x=0.0
            markers[i].pose.orientation.y=0.0
            markers[i].pose.orientation.z=0.0
            markers[i].pose.orientation.w=1.0
            markers[i].lifetime = rospy.Duration()

        markers[0].color.r = 1.0
        markers[0].color.g = 1.0
        markers[0].color.b = 1.0
        markers[0].color.a = 0.5
        fix_stats=[None, self.L_fix_stat, self.R_fix_stat]
        for i in [1,2]:
            markers[i].scale.x = 0.15
            markers[i].scale.y = 0.15
            markers[i].scale.z = 0.03
            if fix_stats[i]==1:
                markers[i].color = RtkCompass.single_color
            elif fix_stats[i]==2:
                markers[i].color = RtkCompass.float_color
            elif fix_stats[i]==3:
                markers[i].color = RtkCompass.fix_color
            else: #????
                markers[i].color = RtkCompass.unknown_color
            markers[i+2].color.r = 1.0
            markers[i+2].color.g = 1.0
            markers[i+2].color.b = 1.0
            markers[i+2].color.a = 1.0
            markers[i+2].scale.z = 1.0

        markers[0].pose.position.x = self.Cx
        markers[0].pose.position.y = self.Cy
        markers[0].pose.orientation.x=self.quat[0]
        markers[0].pose.orientation.y=self.quat[1]
        markers[0].pose.orientation.z=self.quat[2]
        markers[0].pose.orientation.w=self.quat[3]
        markers[0].scale.x=0.25
        markers[0].scale.y=0.05
        markers[0].scale.z=0.05
        markers[1].pose.position.x = self.Lx
        markers[1].pose.position.y = self.Ly
        markers[2].pose.position.x = self.Rx
        markers[2].pose.position.y = self.Ry
        markers[3].pose.position.x = self.Lx
        markers[3].pose.position.y = self.Ly
        markers[4].pose.position.x = self.Rx
        markers[4].pose.position.y = self.Ry
        #tf の座標名もそうだがなぜか文字は表示されない
        markers[3].text="L"
        markers[4].text="R"

        markers[5].scale.x=0.1
        self.point_pairs.append(Point(self.Lx, self.Ly, 0.0))
        self.point_pairs.append(Point(self.Rx, self.Ry, 0.0))
        for fix_stat in [self.L_fix_stat, self.R_fix_stat]:
            if fix_stat==1:
                self.color_pairs.append(RtkCompass.single_color)
            elif fix_stat==2:
                self.color_pairs.append(RtkCompass.float_color)
            elif fix_stat==3:
                self.color_pairs.append(RtkCompass.fix_color)
            else: #????
                self.color_pairs.append(RtkCompass.unknown_color)

        markers[6].scale.x=0.025
        self.dirs.append(Point(self.Cx, self.Cy, 0.0))
        if rospy.get_time() - self.yaw_raw_last_t < 0.2:
            dir_len=0.25
            theta = self.yaw_raw + self.yaw_raw_offset
            self.dirs.append(Point(self.Cx+dir_len*cos(theta), self.Cy+dir_len*sin(theta), 0.0))
        else:
            self.dirs.append(Point(self.Cx, self.Cy, 0.0))

        markers[5].points = self.point_pairs
        markers[5].colors = self.color_pairs
        markers[6].points = self.dirs
        markers[6].colors = self.color_pairs

#        if len(self.point_pairs)>400:
        if len(self.point_pairs)>40:
            self.point_pairs.pop(0)
            self.point_pairs.pop(0)
            self.color_pairs.pop(0)
            self.color_pairs.pop(0)
            self.dirs.pop(0)
            self.dirs.pop(0)

        mkarr.markers=markers
        self.pub_mkarr.publish(mkarr)

if __name__=="__main__":
    rtk_compass = RtkCompass()
    rospy.spin()

