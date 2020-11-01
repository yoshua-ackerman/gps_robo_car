# -*- coding: utf-8 -*-
"""
Created on Thu Oct 31 11:24:55 2019

@author: 
"""

from numpy import pi

#モーター公称スペック
#https://ja.aliexpress.com/item/33001695040.html?spm=a2g0s.9042311.0.0.27424c4d7WvtUh
#12V
#NoLoad:       280RPM, 0.38A
#RatedValue:   224RPM, 2.3A,  4.6Kgcm, 28W
#STALL:                3.8A,  9.6Kgcm
#減速比: 1:18
#1回転あたりパルス: 216(モーター単体12*ギヤ比18)

#Motor and encoder
#RPS_MAX_NOGEAR=84 #Noload#84RPS=5040RPM~=5000RPM  #[(rotation)/sec]
RPS_MAX_NOGEAR=67.2 # RPS:Rotation Per Second, Rated, 4032=224*18RPM
#PPR_NOGEAR=12       # PPR:Pulse per Rotation [(pulse)/sec]
PPR_NOGEAR=16       # PPR:Pulse per Rotation [(pulse)/sec]
GEAR_RATIO=18
PPR=float(PPR_NOGEAR*GEAR_RATIO)
RPS_MAX=RPS_MAX_NOGEAR/GEAR_RATIO

#machine body
WHEEL_DIA=0.07 #[m]
TREAD=0.44 #[m]


MAX_V=RPS_MAX*WHEEL_DIA*pi #[m/sec]


LR_DIST=0.35 #アンテナ間距離[m]
#LR_DIST_TOLE=0.08 #アンテナ間距離への許容誤差[m] #この値使ってない
