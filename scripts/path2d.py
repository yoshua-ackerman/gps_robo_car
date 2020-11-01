# -*- coding: utf-8 -*-

from pprint import pprint

import rospy
import tf_conversions
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header
from geometry_msgs.msg import Point32, Pose2D, PolygonStamped, Pose,PoseArray,PoseStamped
from nav_msgs.msg import Path

from numpy import pi,cos,sin,arctan2,hypot, rad2deg

#from utm_coordinate import UTM_FRAME_NAME,GEOD,ll2xy,xy2ll,WORK_ORIGIN_FRAME_NAME
from utm_coordinate import WORK_ORIGIN_FRAME_NAME

def omit_pi_pi(rad): #range -pi~+pi
    if rad < -pi or rad > +pi:
        rad = rad%(2.0*pi)
        if rad >= pi:
            rad = rad-(2.0*pi)
    return rad

def omit_2pi_2pi(rad): #range -2*pi~+2*pi
    if rad < -2*pi or rad > +2*pi:
        rad = rad%(4.0*pi)
        if rad >= 2*pi:
            rad = rad-(4.0*pi)
    return rad

class Line2D(object):
    def __init__(self, x1,y1, x2,y2):
        self.start_x = x1
        self.start_y = y1
        self.end_x = x2
        self.end_y = y2
        self.len = hypot(x2-x1,y2-y1)
        self.direction = arctan2(y2-y1, x2-x1)
        # ax+by+c=0, a**2+b**2=1
        # 直線の向き:(b,-a)
        # 法線ベクトル+(左側):(+a,+b)
        # 法線ベクトル-(右側):(-a,-b)
        a = -(y2-y1)
        b = x2-x1
        c = x1*y2 - x2*y1
        hyp_a_b = hypot(a,b)
        self.a = a/hyp_a_b
        self.b = b/hyp_a_b
        self.c = c/hyp_a_b
    def start_p(self):
        return self.start_x, self.start_y
    def end_p(self):
        return self.end_x, self.end_y
    def start_p_dist(self, x,y):
        return hypot(x-self.start_x, y-self.start_y)
    def end_p_dist(self, x,y):
        return hypot(x-self.end_x, y-self.end_y)
    def transform2D(self, scale=1.0, rotation=0.0, xoff=0.0,yoff=0.0):
        cos_r=cos(rotation)
        sin_r=sin(rotation)
        x1 = scale*(cos_r*self.start_x - sin_r*self.start_y)+xoff
        y1 = scale*(sin_r*self.start_x + cos_r*self.start_y)+yoff
        x2 = scale*(cos_r*self.end_x - sin_r*self.end_y)+xoff
        y2 = scale*(sin_r*self.end_x + cos_r*self.end_y)+yoff
        self.__init__(x1,y1, x2,y2)
    def invert(self):
        x2=self.start_x
        y2=self.start_y
        x1=self.end_x
        y1=self.end_y
        self.__init__(x1,y1, x2,y2)
    def to_ros(self, output_type=PoseStamped, frame_id=WORK_ORIGIN_FRAME_NAME):
    #output_type:
    #   Point32 (for geometry_msg/PolygonStamped)
    #   Pose (for geometry_msgs/PoseArray)
    #   PoseStamped (for nav_msgs/Path)
        x=(self.start_x, self.end_x)
        y=(self.start_y, self.end_y)
        plist=[]
        for i in (0,1):
            if output_type==Point32:
                p=Point32(x=x[i],y=y[i],z=0)
            elif output_type==Pose:
                p=Pose()
                p.position.x=x[i]
                p.position.y=y[i]
                p.position.z=0
                q = tf_conversions.transformations.quaternion_from_euler(0,0,self.direction)
                p.orientation.x=q[0]
                p.orientation.y=q[1]
                p.orientation.z=q[2]
                p.orientation.w=q[3]
            elif output_type==PoseStamped:
                p=PoseStamped()
                p.header.seq=0
                p.header.stamp=rospy.Time(0)
                p.header.frame_id=frame_id
                p.pose.position.x=x[i]
                p.pose.position.y=y[i]
                p.pose.position.z=0
                q = tf_conversions.transformations.quaternion_from_euler(0,0,self.direction)
                p.pose.orientation.x=q[0]
                p.pose.orientation.y=q[1]
                p.pose.orientation.z=q[2]
                p.pose.orientation.w=q[3]
            plist.append(p)
        return plist
    def in_phase(self,x,y):
        ##点を直線に投影したら範囲内(線分上)にある
        #1.投影点の座標を求める
        #2-1.始点->投影点の向きが線分の向きと合っていなければFalse
        #2-2.向きが同じならば始点->投影点の長さが線分の長さ以下
        dist = self.a*x + self.b*y +self.c
        x_online = x - dist*self.a
        y_online = y - dist*self.b
        if self.b*(x_online-self.start_x)-self.a*(y_online-self.start_y)<0:
            #内積正:おおむね同方向、内積負:おおむね逆方向(、内積ゼロ:直交)
            return False
        l = hypot(x_online-self.start_x, y_online-self.start_y)
        if l<=self.len:
            return True
        return False
    def pose2d_diff(self, pose2d):
    ##距離差と向きの差
        x=pose2d.x
        y=pose2d.y
        theta=pose2d.theta
        if self.in_phase(x,y):
            dist = self.a*x + self.b*y +self.c
            direction_diff = self.direction - theta
            return True, dist, omit_pi_pi(direction_diff)
        else:
            return self.pose2d_diff_start_or_end(x,y,theta)
        #線分や円弧の範囲から外れていたら始点か終点の近いほうにまっしぐらに向かう
        #接線方向になめらかに接続みたいなことはとりあえず考えない
    def pose2d_diff_start_or_end(self, x,y,theta):
        dist2st = self.start_p_dist(x, y)
        dist2end = self.end_p_dist(x, y)
        if dist2st < dist2end:
            #go to startpoint
            direction_diff = arctan2(self.start_y-y, self.start_x-x) - theta
            dist=dist2st
        else:
            #go to endpoint
            direction_diff = arctan2(self.end_y-y, self.end_x-x) - theta
            dist=dist2end
        return False, 0.0, omit_pi_pi(direction_diff)
#        return False, dist, omit_pi_pi(direction_diff) #変な円弧を描いて遠回りに目標地点(start or end)に向かってしまう
    def pose_diff(self, pose):
        pose2d=Pose2D()
        pose2d.x=pose.position.x
        pose2d.y=pose.position.y
        q=(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
        angles=tf_conversions.transformations.euler_from_quaternion(q)
        pose2d.theta=angles[2]
        return self.pose2d_diff(pose2d)

class Arc2D(Line2D):
    def __init__(self, center, R, rad_start, crad):
        self.cx=center[0]
        self.cy=center[1]
        self.R=R
        self.rad_start=rad_start
        self.crad=crad
        self.len=abs(self.R*self.crad)
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
    def transform2D(self, scale=1.0, rotation=0.0, xoff=0.0,yoff=0.0):
        cos_r=cos(rotation)
        sin_r=sin(rotation)
        x_center = scale*(cos_r*self.cx - sin_r*self.cy)+xoff
        y_center = scale*(sin_r*self.cx + cos_r*self.cy)+yoff
        R = scale*self.R
        rad_start = omit_pi_pi(self.rad_start + rotation)
        crad=self.crad
        self.__init__((x_center,y_center),R,rad_start,crad)
    def invert(self):
        x_center = self.cx
        y_center = self.cy
        R = self.R
        rad_start = omit_pi_pi(self.rad_start + self.crad)
        crad = - self.crad
        self.__init__((x_center,y_center),R,rad_start,crad)
    def to_ros_div_N(self, n, output_type=PoseStamped, frame_id=WORK_ORIGIN_FRAME_NAME):
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
    def to_ros_div_deg(self, deg, output_type=PoseStamped, frame_id=WORK_ORIGIN_FRAME_NAME):
        drad = deg2rad(deg)
        n = abs(int(self.crad // drad))
        if n==0: #too large div_deg
            n=1
        return self.to_ros_div_N(n, output_type=output_type, frame_id=frame_id)
    def to_ros_div_len(self, l, output_type=PoseStamped, frame_id=WORK_ORIGIN_FRAME_NAME):
        n = abs(int(self.len // l))
        if n==0: #too large div_length
            n=1
        return self.to_ros_div_N(n, output_type=output_type, frame_id=frame_id)
#    def to_ros_div_err(self, err, output_type=PoseStamped, frame_id=WORK_ORIGIN_FRAME_NAME):
#        #円弧との最大誤差で直線に分割
#        pass
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
            return self.pose2d_diff_start_or_end(x,y,theta)


def path_transform2D(path, scale=1.0, rotation=0.0, xoff=0.0,yoff=0.0):
    for segment in path:
        segment.transform2D(scale,rotation,xoff,yoff)

#パスの方向のみなるべくつながるように変更する。ソートはしない
def path_align_dir(path):
    x_cur,y_cur = path[0].end_p()
    for segment in path[1:]:
        if segment.start_p_dist(x_cur,y_cur)>segment.end_p_dist(x_cur,y_cur):
            segment.invert()
        x_cur,y_cur=segment.end_p()

#パスの近接順ソート。方向もなるべくつながるように変更する。
#特定のパスからはじめる・左上等特定の方向からはじめるなどの考えもあるだろうけど、開始点を指定する
#開始点に最も近いパスから近接順にソートする
def path_sort(unsorted_path, x=0.0,y=0.0, sorted_path=None):
    if sorted_path==None:
        sorted_path=[]
    min_dist=None
    min_idx=0
    for i in range(len(unsorted_path)):
        dist_st = unsorted_path[i].start_p_dist(x,y)
        dist_end = unsorted_path[i].end_p_dist(x,y)
        dist=dist_st if dist_st<=dist_end else dist_end
        if min_dist==None or dist < min_dist:
            min_dist=dist
            min_idx = i
            if min_dist==0:
                break
    segment = unsorted_path.pop(min_idx)
    dist_st = segment.start_p_dist(x,y)
    dist_end = segment.end_p_dist(x,y)
    if dist_end<dist_st:
        segment.invert()
    sorted_path.append(segment)
    x,y=segment.end_p()
    if len(unsorted_path)==0:
        return sorted_path
    else:
        return path_sort(unsorted_path, x,y, sorted_path)        

def path_total_len(path):
    total_len=0.0
    for segment in path:
        total_len = total_len + segment.len
    return total_len

#パスをrvizで可視化するためにトピック出力
#Pointの集まりであるPolygon
#Poseの集まりであるPoseArray
#Posestampedの集まりであるPath
#それぞれ違いや役割があって3種出力してた気がするがよく覚えていない
def path_rosvis(path, topic_base_name="static_path_"):
    pts_all=[]
    pose_all=[]
    path_all=[]
    for segment in path:
        if type(segment)==Line2D:
            pts_1seg = segment.to_ros(output_type=Point32)
            poses_1seg = segment.to_ros(output_type=Pose)
            path_1seg = segment.to_ros(output_type=PoseStamped)
        elif type(segment)==Arc2D:
            pts_1seg = segment.to_ros_div_len(0.5, output_type=Point32)
            poses_1seg = segment.to_ros_div_len(0.5, output_type=Pose)
            path_1seg = segment.to_ros_div_len(0.5, output_type=PoseStamped)

        pts_all = pts_all + pts_1seg
        pose_all = pose_all + poses_1seg
        path_all = path_all + path_1seg

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
    pub_poly=rospy.Publisher(topic_base_name+"_poly", \
                            PolygonStamped, queue_size=2, latch=True)
    pub_poly.publish(static_path_poly)

    static_path_parr = PoseArray()
    static_path_parr.header = header
    static_path_parr.poses = pose_all
    pub_parr=rospy.Publisher(topic_base_name+"_parr", \
                            PoseArray, queue_size=2, latch=True)
    pub_parr.publish(static_path_parr)

    static_path = Path()
    static_path.header=header
    static_path.poses=path_all
    pub_path=rospy.Publisher(topic_base_name, Path, queue_size=2, latch=True)
    pub_path.publish(static_path)



if __name__ == '__main__':
    pass
