#!/usr/bin/env python
# -*- coding: utf-8 -*-

#このプログラムでやっていること
#1. GPSロボカーのパス図形を生成する。ROSのPolygon,PoseArray,Pathとしてtopic出力して可視化
#2-1. パスの中心のUTM座標系内での位置を原点オフセットとみなしPose2D型のtopicとして出力
#       単純なシフト量なのでTFを使うより確実高速そうだから
#2-2. UTM座標系とパス中心座標系の関係をTF出力
#       単純にUTM座標系のみで表しているとrviz内でfloat32使ってる疑惑によってメートル以下の可視化精度が失われる

#from pprint import pprint
from utm_coordinate import GEOD,ll2xy,xy2ll,\
                    UTM_FRAME_NAME,\
                    WORK_ORIGIN_TOPIC_NAME,\
                    WORK_ORIGIN_FRAME_NAME
from gen_path2019 import gen_1lap_path

import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_conversions
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D, Point32,PolygonStamped, Pose,PoseStamped,PoseArray
from nav_msgs.msg import Path

if __name__ == '__main__':

    p1, center, p2, path = gen_1lap_path()

    # ROS ノードの初期化処理
    rospy.init_node('static_path_broadcaster')

    # ブロードキャスタ、Transform
    br = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()

    # Transform の時刻情報、Base となる座標系、world を Base とする座標系
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = UTM_FRAME_NAME
    t.child_frame_id = WORK_ORIGIN_FRAME_NAME

    # 6D pose (位置 translation、姿勢 rotation)
    t.transform.translation.x = center[0]
    t.transform.translation.y = center[1]
    t.transform.translation.z = 0

#    from numpy import arctan2
#    slant_rad = arctan2(p2[1]-p1[1], p2[0]-p1[0])
#    quat = quaternion_from_euler(0, 0, slant_rad)
    #とりあえず回転なし、シフトのみで
    quat = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    br.sendTransform(t)

    work_origin2D=Pose2D(center[0],center[1],0.0)
    pub_work_origin2D = rospy.Publisher(WORK_ORIGIN_TOPIC_NAME, \
                                        Pose2D, queue_size=2, latch=True)
    pub_work_origin2D.publish(work_origin2D)

    pts_all=[]
    pose_all=[]
    path_all=[]
    for i in range(len(path)):
        pts_1arc = path[i].div_len(0.5, output_type=Point32)
        pts_all = pts_all + pts_1arc
        poses_1arc = path[i].div_len(0.5, output_type=Pose)
        pose_all = pose_all + poses_1arc
        path_1arc = path[i].div_len(0.5)
        path_all = path_all + path_1arc
#    pprint(path_all)
#    pprint(pts_all)

    header=Header()
    header.seq=0
    header.stamp=rospy.Time.now()
# rviz内部でfloat32を使っているらしくUTM座標系の表示は桁落ちでパスがガタガタになる
# 32bitのraspi用の ubuntu Mate だけでなく64bitPC版のUbuntuでも同様の現象になる
#    header.frame_id = UTM_FRAME_NAME
    header.frame_id = WORK_ORIGIN_FRAME_NAME

    static_path_poly = PolygonStamped()
    static_path_poly.header = header
    static_path_poly.polygon.points = pts_all
    pub_poly=rospy.Publisher("/static_path_2019_poly", \
                            PolygonStamped, queue_size=2, latch=True)
    pub_poly.publish(static_path_poly)

    static_path_parr = PoseArray()
    static_path_parr.header = header
    static_path_parr.poses = pose_all
    pub_parr=rospy.Publisher("/static_path_2019_parr", \
                            PoseArray, queue_size=2, latch=True)
    pub_parr.publish(static_path_parr)

    static_path = Path()
    static_path.header=header
    static_path.poses=path_all
    pub_path=rospy.Publisher('/static_path_2019', Path, queue_size=2, latch=True)
    pub_path.publish(static_path)

    rospy.spin()

