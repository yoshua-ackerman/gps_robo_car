# -*- coding: utf-8 -*-

from pprint import pprint

from utm_coordinate import GEOD,ll2xy,xy2ll,\
                    UTM_FRAME_NAME,\
                    WORK_ORIGIN_TOPIC_NAME,\
                    WORK_ORIGIN_FRAME_NAME
from path2d import Arc2D


from course_location import *
#LON0, LAT0, SLANT_DEG, REIWA_DIST, TURN_R


from numpy import cos,sin,arctan2,deg2rad,hypot,pi

def gen_1lap_path_3pt(x0,y0, x1,y1, x2,y2, turn_r=1.5):
    #x0,y0:center
    #x1,y1, x2,y2: sub-pylon or point between pylon and sub-pylon
    l1=hypot(x1-x0,y1-y0)
    l2=hypot(x2-x0,y2-y0)
    vec1=[ (x1-x0)/l1, (y1-y0)/l1 ]
    vec2=[ (x2-x0)/l2, (y2-y0)/l2 ]

    r = turn_r

    ######
    #path1
    d1=l1+r
    #(R1+d1)**2==R1**2+d1**2
    R1=(d1*d1-r*r)/(2.0*r)
    cx1 = x0+R1*vec1[1]
    cy1 = y0-R1*vec1[0]
    rad1_start = arctan2(vec1[0],-vec1[1])
    rad1 = - arctan2(d1,R1)

    #path2
    R2=r
    cx2=x1+R2*vec1[0]
    cy2=y1+R2*vec1[1]
    rad2_start = arctan2(cy1-cy2, cx1-cx2)
    rad2 = 2.0*pi - (0.5*pi - abs(rad1))

    #path3 and path4
    R3=r
    cx3=x1-R3*vec1[0]
    cy3=y1-R3*vec1[1]
    rad3_start = arctan2(vec1[1],vec1[0])

    d4=l1-r
    #(R4+d4)**2==R4**2+d4**2
    R4=(d4*d4-r*r)/(2.0*r)
    cx4= x0+R4*vec1[1]
    cy4= y0-R4*vec1[0]
    rad4_start = arctan2(cy3-cy4, cx3-cx4)
    rad4 = arctan2(d4,R4) #サブパイロン1がセンターの右側だと符号逆かも? -> そんなことはないようだ
    rad3 = -(pi-(0.5*pi-abs(rad4)))

    ######
    #path5
    d5=l2+r
    R5=(d5*d5-r*r)/(2.0*r)
    cx5 = x0-R5*vec2[1]
    cy5 = y0+R5*vec2[0]
    rad5_start = arctan2(-vec2[0],vec2[1])
    rad5 = arctan2(d5,R5)

    #path6
    R6=r
    cx6=x2+R6*vec2[0]
    cy6=y2+R6*vec2[1]
    rad6_start = arctan2(cy5-cy6, cx5-cx6)
    rad6 = -(2.0*pi - (0.5*pi - abs(rad5)))

    #path7 and path8
    R7=r
    cx7=x2-R7*vec2[0]
    cy7=y2-R7*vec2[1]
    rad7_start= arctan2(vec2[1],vec2[0])

    d8=l2-r
    R8=(d8*d8-r*r)/(2.0*r)
    cx8= x0-R8*vec2[1]
    cy8= y0+R8*vec2[0]
    rad8_start = arctan2(cy7-cy8, cx7-cx8)
    rad8 = -arctan2(d8,R8) #サブパイロン2がセンターの左側だと符号逆かも? -> そんなことはないようだ
    rad7 = pi-(0.5*pi-abs(rad8))

    lap_path =  [
                Arc2D((cx1-x0,cy1-y0),R1,rad1_start,rad1),
                Arc2D((cx2-x0,cy2-y0),R2,rad2_start,rad2),
                Arc2D((cx3-x0,cy3-y0),R3,rad3_start,rad3),
                Arc2D((cx4-x0,cy4-y0),R4,rad4_start,rad4),
                Arc2D((cx5-x0,cy5-y0),R5,rad5_start,rad5),
                Arc2D((cx6-x0,cy6-y0),R6,rad6_start,rad6),
                Arc2D((cx7-x0,cy7-y0),R7,rad7_start,rad7),
                Arc2D((cx8-x0,cy8-y0),R8,rad8_start,rad8)
                ]
    return (x1,y1),(x0,y0),(x2,y2), lap_path


def gen_1lap_path_3ll(lon0,lat0, lon1,lat1, lon2,lat2, turn_r=1.5):
    x0,y0=ll2xy(lon0,lat0)
    x1,y1=ll2xy(lon1,lat1)
    x2,y2=ll2xy(lon2,lat2)
    return gen_1lap_path_3pt(x0,y0, x1,y1, x2,y2, turn_r=turn_r)

def gen_1lap_path_2ll(lon1,lat1, lon2,lat2, turn_r=1.5):
    x1,y1=ll2xy(lon1,lat1)
    x2,y2=ll2xy(lon2,lat2)
    x0=(x1+x2)/2.
    y0=(y1+y2)/2.
    return gen_1lap_path_3pt(x0,y0, x1,y1, x2,y2, turn_r=turn_r)

def gen_1lap_path_1ll(lon0,lat0, deg,dist, turn_r=1.5):
    x0,y0=ll2xy(lon0,lat0)
    rad1=deg2rad(deg)
    rad2=rad1+pi
    x1=x0+dist*cos(rad1)
    y1=y0+dist*sin(rad1)
    x2=x0+dist*cos(rad2)
    y2=y0+dist*sin(rad2)
    return gen_1lap_path_3pt(x0,y0, x1,y1, x2,y2, turn_r=turn_r)


def gen_1lap_path():
    if True:
        #1点と距離・傾きからパス生成
        return gen_1lap_path_1ll(LON0,LAT0, SLANT_DEG, REIWA_PT_DIST, turn_r=TURN_R)
    elif True:
        #3点からパス生成(本番形式)
        return gen_1lap_path_3ll(LON0,LAT0, LON1,LAT1, LON2,LAT2, turn_r=TURN_R)
    else:
        #2点からパス生成。いちおうつくったけどあんま使わん気ガス
        return gen_1lap_path_2ll(LON1,LAT1, LON2,LAT2, turn_r=TURN_R)

