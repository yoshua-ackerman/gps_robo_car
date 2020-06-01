# -*- coding: utf-8 -*-

from pprint import pprint

import rospy
import tf_conversions
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point32, Pose2D, PolygonStamped, Pose,PoseArray,PoseStamped
from nav_msgs.msg import Path

from numpy import pi,cos,sin,arctan2,hypot

#from utm_coordinate import UTM_FRAME_NAME,GEOD,ll2xy,xy2ll,WORK_ORIGIN_FRAME_NAME
from utm_coordinate import WORK_ORIGIN_FRAME_NAME

def omit_pi_pi(rad): #range -pi~+pi
    if rad < -pi or rad > +pi:
        rad = rad%(2.0*pi)
        if rad >= pi:
            rad = rad-(2.0*pi)
    return rad

class Arc2D(object):
    def __init__(self, center, R, rad_start, crad):
        self.cx=center[0]
        self.cy=center[1]
        self.R=R
        self.rad_start=rad_start
        self.crad=crad
        self.arc_len=self.R*self.crad
        self.start_p()
        self.end_p()
    def start_p(self):
        self.start_x = self.cx + self.R * cos(self.rad_start)
        self.start_y = self.cy + self.R * sin(self.rad_start)
        return self.start_x, self.start_y
    def end_p(self):
        self.end_x = self.cx + self.R * cos(self.rad_start + self.crad)
        self.end_y = self.cy + self.R * sin(self.rad_start + self.crad)
        return self.end_x, self.end_y
    def start_p_dist(self, x,y):
        return hypot(x-self.start_x, y-self.start_y)
    def end_p_dist(self, x,y):
        return hypot(x-self.end_x, y-self.end_y)
#    def rot_and_shift(self, yaw, x,y):
#        pass
    def div_N(self, n, output_type=PoseStamped, frame_id=WORK_ORIGIN_FRAME_NAME):
    #output_type:
    #   Point32 (for geometry_msg/PolygonStamped)
    #   Pose (for geometry_msgs/PoseArray)
    #   PoseStamped (for nav_msgs/Path)
        if n<=0:
            n=1
        drad = self.crad / n
        plist=[]
        for i in range(n+1):
            if i==n:
                theta = self.rad_start + self.crad
            else:
                theta = self.rad_start + drad*i
            x=self.cx + self.R * cos(theta)
            y=self.cy + self.R * sin(theta)
            if self.crad>0: #direction of tangent line
                direc=theta+pi/2.0
            else:
                direc=theta-pi/2.0
            if output_type==Point32:
                p=Point32(x=x,y=y,z=0)
            elif output_type==Pose:
                p=Pose()
                p.position.x=x
                p.position.y=y
                p.position.z=0
                q = tf_conversions.transformations.quaternion_from_euler(0,0,direc)
                p.orientation.x=q[0]
                p.orientation.y=q[1]
                p.orientation.z=q[2]
                p.orientation.w=q[3]
            elif output_type==PoseStamped:
                p=PoseStamped()
                p.header.seq=0
                p.header.stamp=rospy.Time(0)
                p.header.frame_id=frame_id
                p.pose.position.x=x
                p.pose.position.y=y
                p.pose.position.z=0
                q = tf_conversions.transformations.quaternion_from_euler(0,0,direc)
                p.pose.orientation.x=q[0]
                p.pose.orientation.y=q[1]
                p.pose.orientation.z=q[2]
                p.pose.orientation.w=q[3]
            plist.append(p)
        return plist
    def div_deg(self, deg, output_type=PoseStamped, frame_id=WORK_ORIGIN_FRAME_NAME):
        drad = deg2rad(deg)
        n = abs(int(self.crad // drad))
        if n==0: #too large div_deg
            n=1
        return self.div_N(n, output_type=output_type, frame_id=frame_id)
    def div_len(self, l, output_type=PoseStamped, frame_id=WORK_ORIGIN_FRAME_NAME):
        n = abs(int(self.arc_len // l))
        if n==0: #too large div_length
            n=1
        return self.div_N(n, output_type=output_type, frame_id=frame_id)
    def calc_thp(self,x,y):
        #中心から点x,yへの絶対角度
        dx=x-self.cx
        dy=y-self.cy
        self.thp_abs = arctan2(dy,dx)
    def in_phase(self,x,y):
        self.calc_thp(x,y)
        thp_rel = self.thp_abs-self.rad_start
        #print self.rad_start,self.thp_abs,thp_rel
        if self.crad>=0: #CCW
            if thp_rel>2.0*pi or thp_rel <=0:
                thp_rel=thp_rel%(2.0*pi)
            if thp_rel < self.crad:
                return True
        else: #CW
            if thp_rel<-2.0*pi or thp_rel >=0:
                thp_rel=thp_rel%(-2.0*pi)
            if self.crad < thp_rel:
                return True
        return False
    def pose2d_diff(self, pose2d):
        x=pose2d.x
        y=pose2d.y
        theta=pose2d.theta
        if self.in_phase(x,y): #and calc thp_abs
        #中心角180度以上なら始点・中心点・終点の三角形の中もありにしてもいいかも？
            dist = hypot(x-self.cx, y-self.cy)-abs(self.R)
            #円弧パスに対して進行方向左なら+、右なら-になるように距離値を返す
            if self.crad>=0: #CCW
                dist = -dist
                direction_diff = self.thp_abs +pi/2.0 - theta
            else: #CW
                #dist = +dist
                direction_diff = self.thp_abs -pi/2.0 - theta
            return True, dist, omit_pi_pi(direction_diff)
        else:
            dist2st = hypot(x-self.start_x, y-self.start_y)
            dist2end = hypot(x-self.end_x, y-self.end_y)
            #とりあえず接線方向とか難しいことは考えずに始点か終点の近いほうにまっしぐら
            if dist2st < dist2end:
                #go to startpoint
                direction_diff = arctan2(self.start_y-y, self.start_x-x) - theta
            else:
                #go to endpoint
                direction_diff = arctan2(self.end_y-y, self.end_x-x) - theta
            return False, 0.0, omit_pi_pi(direction_diff)
    def pose_diff(self, pose):
        pose2d=Pose2D()
        pose2d.x=pose.position.x
        pose2d.y=pose.position.y
        q=(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
        angles=tf_conversions.transformations.euler_from_quaternion(q)
        pose2d.theta=angles[2]
        return self.pose2d_diff(pose2d)

if __name__ == '__main__':
    pass
