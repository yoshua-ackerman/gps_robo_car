#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Nov  2 20:26:12 2019

@author: yoshua
"""

pts1=[[35.66549019,139.79340376], #main pylon 1
      [35.66549647,139.79341137], #sub pylon 1
      [35.66555540,139.79347977], #center
      [35.66561394,139.79354856], #sub pylon 2
      [35.66562049,139.79355609]] #main pylon 2
pts2=[[35.66536847,139.79357178],
      [35.66537526,139.79357928],
      [35.66543449,139.79364659],
      [35.66549409,139.79371440],
      [35.66550084,139.79372173]]

UTM_BASE_LON=139 #home:138.84, KAIYO-Univ:139.79 -> UTM_BASE_LON=54
from pyproj import Proj,Geod
def lon2utmzone(lon):
    #W180-W174:1
    #W6-0:30
    #0-E6:31
    #E174-E180:60
    return ((lon+180)%360)//6+1
PROJCONV=Proj(proj='utm', zone=lon2utmzone(UTM_BASE_LON), ellps='WGS84')
GEOD = Geod(ellps='WGS84')
def ll2xy(lon,lat):
    x,y=PROJCONV(lon,lat)
    return x,y
def xy2ll(x,y):
    lat,lon=PROJCONV(x,y,inverse=True)
    return lat,lon

#print lon2utmzone(UTM_BASE_LON)


print("# point2point azimuth and distance (course 1)")
for i in range(4):
    azimuth,back_azimuth,dist = GEOD.inv(pts1[i][1],pts1[i][0],pts1[i+1][1],pts1[i+1][0])
    print("({0:+8.3f}, {1:+8.3f}, {2:.3f})".format(azimuth,back_azimuth,dist))

print("# point2point azimuth and distance (course 2)")
for i in range(4):
    azimuth,back_azimuth,dist = GEOD.inv(pts2[i][1],pts2[i][0],pts2[i+1][1],pts2[i+1][0])
    print("({0:+8.3f}, {1:+8.3f}, {2:.3f})".format(azimuth,back_azimuth,dist))

print("\n")
#from pprint import pprint
subp1_1_xy=ll2xy(pts1[1][1],pts1[1][0])
center1_xy=ll2xy(pts1[2][1],pts1[2][0])
subp1_2_xy=ll2xy(pts1[3][1],pts1[3][0])
subp2_1_xy=ll2xy(pts2[1][1],pts2[1][0])
center2_xy=ll2xy(pts2[2][1],pts2[2][0])
subp2_2_xy=ll2xy(pts2[3][1],pts2[3][0])

def pdiff(p1,p2):
    return (p2[0]-p1[0],p2[1]-p1[1])

#https://gammasoft.jp/blog/python-string-format/
#https://docs.python.org/ja/3/library/string.html
format_str1="({0[0]:+012.3f},{0[1]:+012.3f}), ({1[0]:+012.3f},{1[1]:+012.3f}), ({2[0]:+012.3f},{2[1]:+012.3f})"
format_str2="({0[0]:+.3f},{0[1]:+.3f}), ({1[0]:+.3f},{1[1]:+.3f})"
print("## apporoximated orthogonal coodinate")
print("# absolute/relative 2D positions  of subpylon1, center, subpylon2")
print(format_str1.format(subp1_1_xy, center1_xy, subp1_2_xy))
print(format_str2.format(pdiff(center1_xy, subp1_1_xy), pdiff(center1_xy, subp1_2_xy)))
print(format_str1.format(subp2_1_xy, center2_xy, subp2_2_xy))
print(format_str2.format(pdiff(center2_xy, subp2_1_xy), pdiff(center2_xy, subp2_2_xy)))

print("\n# point2point deg and distance")

#subp1_1_ll = xy2ll(subp1_1_xy[0],subp1_1_xy[1])
#center1_ll = xy2ll(center1_xy[0],center1_xy[1])
#subp1_2_ll = xy2ll(subp1_2_xy[0],subp1_2_xy[1])
#
#print(subp1_1_ll,center1_ll,subp1_2_ll)


from numpy import hypot,arctan2,rad2deg

pts=[pts1,pts2]
x=[[0,0,0,0,0],[0,0,0,0,0]]
y=[[0,0,0,0,0],[0,0,0,0,0]]

for i in range(2):
    for j in range(5):
        x[i][j],y[i][j] = ll2xy(pts[i][j][1],pts[i][j][0])

for i in range(2):
    for j in range(4):
        dx=x[i][j+1]-x[i][j]
        dy=y[i][j+1]-y[i][j]
        dist=hypot(dx,dy)
        deg=rad2deg(arctan2(dy,dx))
        print "{0:+8.3f}, {1:.3f}".format(deg,dist)
    print " --- "

#会場1の「サブパイロン1-センター間」と「センター-サブパイロン2間」の直線角度は
#0.4度差があり、２つのサブパイロン間の直線とセンターポイントで0.07m=7cmほどの差が出る
#会場2はほぼ一直線

print("\n# degree and distance from course 1 to 2")
dx=x[1][2]-x[0][2]
dy=y[1][2]-y[0][2]
dist=hypot(dx,dy)
deg=rad2deg(arctan2(dy,dx))
print "{0:+8.3f}, {1:.3f}".format(deg,dist)
