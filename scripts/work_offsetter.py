#!/usr/bin/env python
# -*- coding: utf-8 -*-

#このプログラムでやっていること
# 1. UTM座標系内でのパスの原点オフセットをPose2D型のtopicとして出力
#       単純なシフト量なのでTFを使うより確実高速そうだから
# 2. UTM座標系とパス座標系の関係をTF出力
#       単純にUTM座標系のみで表しているとrviz内でfloat32使ってる疑惑によってメートル以下の可視化精度が失われる

from utm_coordinate import ll2xy,\
                    UTM_FRAME_NAME,\
                    WORK_ORIGIN_TOPIC_NAME,\
                    WORK_ORIGIN_FRAME_NAME
from course_location import LON0,LAT0

import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_conversions
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose2D

if __name__ == '__main__':

    center = ll2xy(LON0,LAT0)

    # ROS ノードの初期化処理
    rospy.init_node('work_offset_broadcaster')

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

    rospy.spin()
