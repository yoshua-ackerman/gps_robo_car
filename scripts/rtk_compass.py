#!/usr/bin/python
# -*- coding: utf-8 -*-


#TODO:(AkRawData->)yaw_raw をサブスクリプトして yaw_offset_raw2rtkcmps をパブリッシュする
#TODO:LINE_STRIPのマーカ表示をする。1点ごとにsingle/float/fixの色付けをする
#TODO:poseのマーカををfix度合に応じて色付けする
#TODO:
#TODO:fix率、float率、single率、無応答率を見れるようにする


from ak_param import *


from numpy import arctan2,hypot,rad2deg,pi
from pprint import pprint

from utm_coordinate import GEOD,ll2xy,xy2ll,\
                    UTM_FRAME_NAME,\
                    WORK_ORIGIN_TOPIC_NAME,\
                    WORK_ORIGIN_FRAME_NAME

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D, PointStamped, PoseStamped, TransformStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path

import tf2_ros
import tf_conversions

from visualization_msgs.msg import Marker, MarkerArray

#応答すべき時間、無応答時間の計測に使う
#受信機設定が1Hzならば1.0、5Hzならば0.2
PERIOD_ASSUMED=1.0

L_old=None
L_new=None
R_old=None
R_new=None

L_last_t=None
R_last_t=None

work_origin=Pose2D(0,0,0)

Lx=None
Ly=None
Rx=None
Ry=None
Cx=None
Cy=None

theta=None
distance=None
path=Path()
poses=[]

#pub_pos=None
#pub_ant_pos_l=None
#pub_ant_pos_r=None
#pub_path=None
#t0=None
seq=0

fix_str=("------","single","float ","*FIX* ")

def calc_pose():
    global seq
    theta=arctan2(Rx-Lx,-(Ry-Ly))
    distance=hypot(Rx-Lx, Ry-Ly)
    Cx=(Lx+Rx)/2.0
    Cy=(Ly+Ry)/2.0
    L_fix_stat = L_new.position_covariance_type
    R_fix_stat = R_new.position_covariance_type
    L_fix_str = fix_str[L_fix_stat]
    R_fix_str = fix_str[R_fix_stat]
    q=tf_conversions.transformations.quaternion_from_euler(0,0,theta)

    print L_fix_str, R_fix_str, "%5.3f[m],%.2f[deg], (Cx,Cy)=(%+.2f,%+.2f)"%(distance, rad2deg(theta), Cx,Cy)
#    print Lx,Ly,Rx,Ry
#    print Lx-Cx,Ly-Cy,Rx-Cx,Ry-Cy
#    print hypot(Lx-Cx,Ly-Cy), hypot(Rx-Cx,Ry-Cy)

    seq=seq+1
    header=Header()
    header.seq=seq
    header.stamp=rospy.Time.now()
    header.frame_id = WORK_ORIGIN_FRAME_NAME #UTM_FRAME_NAME

    mkarr = MarkerArray()
    markers=[Marker(type=Marker.ARROW),\
            Marker(type=Marker.CYLINDER),Marker(type=Marker.CYLINDER),\
            Marker(type=Marker.TEXT_VIEW_FACING),Marker(type=Marker.TEXT_VIEW_FACING)]
    for i in range(5):
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
    markers[0].color.a = 1.0
    fix_stats=[None, L_fix_stat, R_fix_stat]
    for i in [1,2]:
        markers[i].scale.x = 0.15
        markers[i].scale.y = 0.15
        markers[i].scale.z = 0.03
        if fix_stats[i]==1:
            markers[i].color.r = 1.0
            markers[i].color.g = 0.0
            markers[i].color.b = 0.0
            markers[i].color.a = 1.0
        elif fix_stats[i]==2:
            markers[i].color.r = 0.5
            markers[i].color.g = 0.5
            markers[i].color.b = 0.0
            markers[i].color.a = 1.0
        elif fix_stats[i]==3:
            markers[i].color.r = 0.0
            markers[i].color.g = 0.5
            markers[i].color.b = 0.0
            markers[i].color.a = 1.0
        else: #????
            markers[i].color.r = 0.5
            markers[i].color.g = 0.5
            markers[i].color.b = 0.5
            markers[i].color.a = 1.0
        markers[i+2].color.r = 1.0
        markers[i+2].color.g = 1.0
        markers[i+2].color.b = 1.0
        markers[i+2].color.a = 1.0
        markers[i+2].scale.z = 1.0

    markers[0].pose.position.x=Cx
    markers[0].pose.position.y=Cy
    markers[0].pose.orientation.x=q[0]
    markers[0].pose.orientation.y=q[1]
    markers[0].pose.orientation.z=q[2]
    markers[0].pose.orientation.w=q[3]
    markers[0].scale.x=0.25
    markers[0].scale.y=0.05
    markers[0].scale.z=0.05
    markers[1].pose.position.x = Lx
    markers[1].pose.position.y = Ly
    markers[2].pose.position.x = Rx
    markers[2].pose.position.y = Ry
    markers[3].pose.position.x = Lx
    markers[3].pose.position.y = Ly
    markers[4].pose.position.x = Rx
    markers[4].pose.position.y = Ry
    #tf の座標名もそうだがなぜか文字は表示されない
    markers[3].text="L"
    markers[4].text="R"

    mkarr.markers=markers
    pub_mkarr.publish(mkarr)

    #ポイントではなくマーカーにしたほうがいいかも
    #マーカーはプログラムから色を変えられるから、色でsingle/float/fixを表示できる
 
    p_c=PoseStamped()
    p_c.header=header
    p_c.pose.position.x=Cx
    p_c.pose.position.y=Cy
    p_c.pose.position.z=0
    p_c.pose.orientation.x=q[0]
    p_c.pose.orientation.y=q[1]
    p_c.pose.orientation.z=q[2]
    p_c.pose.orientation.w=q[3]
 #   pub_pos.publish(p_c)

#    p_l=PointStamped()
#    p_l.header=header
#    p_l.point.x=Lx
#    p_l.point.y=Ly
#    p_l.point.z=0
#    pub_ant_pos_l.publish(p_l)
#    p_r=PointStamped()
#    p_r.header=header
#    p_r.point.x=Rx
#    p_r.point.y=Ry
#    p_r.point.z=0
#    pub_ant_pos_r.publish(p_r)

    path.header=header
    poses.append(p_c)
    if len(poses)>200:
        poses.pop(0)
    path.poses=poses
    pub_path.publish(path)

    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header=p_c.header
    t.child_frame_id = "base_footprint_rtk"
    t.transform.translation.x = Cx
    t.transform.translation.y = Cy
    t.transform.translation.z = 0.0
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

def cb_L(sol_L):
    global L_last_t,Lx,Ly, L_old,L_new
    L_last_t=rospy.get_time()
    L_old=L_new
    L_new=sol_L
    Lx,Ly=ll2xy(sol_L.longitude, sol_L.latitude)
    # if work_origin was loaded, offset to origin
    Lx = Lx - work_origin.x
    Ly = Ly - work_origin.y
#    pprint(sol_L)
    if R_new==None:
        return
    if L_last_t-R_last_t<0.1 and L_last_t>R_last_t:
        rospy.logdebug("L %.1f"%(L_last_t-t0))
        calc_pose()

def cb_R(sol_R):
    global R_last_t,Rx,Ry, R_old,R_new
    R_last_t=rospy.get_time()
    R_old=R_new
    R_new=sol_R
    Rx,Ry=ll2xy(sol_R.longitude, sol_R.latitude)
    # if work_origin was loaded, offset to origin
    Rx = Rx - work_origin.x
    Ry = Ry - work_origin.y
#    pprint(sol_R)
    if L_new==None:
        return
    if R_last_t-L_last_t<0.1 and R_last_t>L_last_t:
        rospy.logdebug("R %.1f"%(R_last_t-t0))
        calc_pose()

def cb_worigin(worigin):
    global work_origin
    work_origin=worigin

if __name__=="__main__":
    rospy.init_node('Rtk_compass', anonymous=True)

    global pub_path, pub_mkarr, t0
#    global pub_pos, pub_ant_pos_l, pub_ant_pos_r, pub_path, pub_mkarr, t0
    t0=rospy.get_time()
#    pub_pos = rospy.Publisher('/pose', PoseStamped, queue_size=10)
#    pub_ant_pos_l = rospy.Publisher('/ant_pos_L', PointStamped, queue_size=10)
#    pub_ant_pos_r = rospy.Publisher('/ant_pos_R', PointStamped, queue_size=10)
    pub_path = rospy.Publisher('/point_log', Path, queue_size=10)
    sub_L = rospy.Subscriber('/gps_sol_L', NavSatFix, cb_L, queue_size=10)
    sub_R = rospy.Subscriber('/gps_sol_R', NavSatFix, cb_R, queue_size=10)

    sub_worig = rospy.Subscriber(WORK_ORIGIN_TOPIC_NAME, Pose2D, cb_worigin, queue_size=2)

    pub_mkarr = rospy.Publisher('/rtk_markers', MarkerArray, queue_size=10)

    rospy.spin()

