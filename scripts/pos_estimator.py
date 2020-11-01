#!/usr/bin/python
# -*- coding: utf-8 -*-

# TODO: rtk_odom で向きが出ていないとき？変な方向に進みはじめる

from pprint import pprint

import rospy
import tf_conversions
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion, Twist, Vector3, TransformStamped
from std_msgs.msg import String

from numpy import pi,cos,sin

from ak_param import *
# PPR #Pulse Per Rotation
# WHEEL_DIA=0.07 #[m]
# TREAD=0.44 #[m]
# MAX_V=RPS_MAX_NOGEAR/GEAR_RATIO*WHEEL_DIA*pi #[m/sec]
from gps_robo_car.msg import AkRawData


rospy.init_node('postion_estimator')
pub_aqtalk=rospy.Publisher("aqtalk_str", String, queue_size=3)
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf2_ros.TransformBroadcaster()

x = 0.0
y = 0.0
th = 0.0
#v_l=0.0
#v_r=0.0
#v = 0.0
#vth = 0.0

last_time = None

def update_pos_by_odom(od_msg):
    global x,y,th,last_time
    current_time = od_msg.stamp
    if last_time==None:
        last_time=current_time
        return

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    d_l = od_msg.pulses[0]/PPR*WHEEL_DIA*pi
    d_r = od_msg.pulses[1]/PPR*WHEEL_DIA*pi
    v_l=d_l/dt
    v_r=d_r/dt

    v=(v_l+v_r)/2.0
#   odom座標系での速度成分は、
#    vx=v*cos(th)
#    vy=v*sin(th)
#   こうなるが「速度情報は "base_link" 座標系に対して送る」ので、
#   http://wiki.ros.org/ja/navigation/Tutorials/RobotSetup/Odom
#   下記でいいらしい。poseはodom、twistはbase_link
    vx=v
    vy=0
    vth=(v_r-v_l)/TREAD #omega

    if d_l==d_r:
        x += v*dt*cos(th)
        y += v*dt*sin(th)
    else:
        delta_th = vth * dt
        R=v/vth
        rospy.logdebug("R=%.1f",R)
#        print "R=",R
        #   第1項:旋回開始点から旋回中心へ、第2項:旋回中心から旋回終了点へ
        x += R*cos(th+pi/2.0)+R*cos(th-pi/2.0+delta_th)
        y += R*sin(th+pi/2.0)+R*sin(th-pi/2.0+delta_th)
#        if v>=0:
#            x += R*cos(th+pi/2.0)+R*cos(th-pi/2.0+delta_th)
#            y += R*sin(th+pi/2.0)+R*sin(th-pi/2.0+delta_th)
#        else:
#            x += R*cos(th-pi/2.0)+R*cos(th+pi/2.0+delta_th)
#            y += R*sin(th-pi/2.0)+R*sin(th+pi/2.0+delta_th)
        th += delta_th

    t = TransformStamped()
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, th)
    t.transform.rotation.x = odom_quat[0]
    t.transform.rotation.y = odom_quat[1]
    t.transform.rotation.z = odom_quat[2]
    t.transform.rotation.w = odom_quat[3]

    # first, we'll publish the transform over tf
    t.header.stamp = current_time
    t.header.frame_id = "odom"
    t.child_frame_id = "base_footprint_odom"
    odom_broadcaster.sendTransform(t)

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_footprint_odom"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time


last_t_reliable=None
last_warn_t=None

def update_pos_by_rtk(pose_by_rtk):
    global x,y,th, last_t_reliable,last_warn_t

    if pose_by_rtk.pose.covariance[0]>=0.75:
        last_t_reliable = rospy.get_time()
    if pose_by_rtk.pose.covariance[0]<0.25 and last_t_reliable!=None:
        t_now = rospy.get_time()
        if t_now - last_t_reliable > 10:
            if last_warn_t == None:
                pub_aqtalk.publish("位置情報来てません")
                last_warn_t = rospy.get_time()
            if last_warn_t - t_now>10:
                pub_aqtalk.publish("位置情報来てません")
                last_warn_t = rospy.get_time()

    if pose_by_rtk.pose.covariance[0]>=0.25:
#    if True:
        x=pose_by_rtk.pose.pose.position.x
        y=pose_by_rtk.pose.pose.position.y
        orientation_q = pose_by_rtk.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
#        print orientation_list
        (roll, pitch, yaw) = tf_conversions.transformations.euler_from_quaternion (orientation_list)
        print (roll, pitch, yaw)
        th = yaw

sub_wheel_odom = rospy.Subscriber("ak_raw_data", AkRawData, update_pos_by_odom, queue_size=10)
sub_rtk_odom = rospy.Subscriber("rtk_odom", PoseWithCovarianceStamped, update_pos_by_rtk, queue_size=10)
rospy.spin()
