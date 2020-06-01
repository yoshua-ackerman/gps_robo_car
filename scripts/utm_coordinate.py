# -*- coding: utf-8 -*-

from pyproj import Proj,Geod


UTM_BASE_LON=139 #home:138.84, KAIYO-Univ:139.79 -> UTM zone number 54

def lon2utmzone(lon):
    # W180 - W174 :  1
    # W006 -  000 : 30
    #  000 - E006 : 31
    # E174 - E180 : 60
    round_int= lambda x:int((x*2+1)//2)
    return ((round_int(lon)+180)%360)//6+1

UTM_ZONE=lon2utmzone(UTM_BASE_LON)

PROJCONV=Proj(proj='utm', zone=UTM_ZONE, ellps='WGS84')
GEOD = Geod(ellps='WGS84')
#generate other point(lon,lat)
#lon,lat,baz=GEOD.fwd(lon,lat,azimuth,meter)

def ll2xy(lon,lat):
    x,y=PROJCONV(lon,lat)
    return x,y

def xy2ll(x,y):
    lat,lon=PROJCONV(x,y,inverse=True)

UTM_FRAME_NAME="utm"+str(UTM_ZONE)
#UTM座標系で管理しているとrvizの表示が[m](メートル)以下の精度が失われている
#64bit PC の Kinetic でも現象を確認
#プログラム内でtfを使って変換すればそのようなことは起きないのでrvizの問題
WORK_ORIGIN_FRAME_NAME="work_origin_in_"+UTM_FRAME_NAME
WORK_ORIGIN_TOPIC_NAME=WORK_ORIGIN_FRAME_NAME
