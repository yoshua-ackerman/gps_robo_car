# -*- coding: utf-8 -*-
"""
Created on Thu Jun 29 12:45:58 2017

@author: mate-go
"""

import sys
import dxfgrabber as dg
import turtle as tut

from numpy import pi,cos,sin,arctan2,sqrt


fname=sys.argv[1]
dxf=dg.readfile(fname)
print(dxf.header)
all_lines = [entity for entity in dxf.entities if entity.dxftype == 'LINE']
all_cirs = [entity for entity in dxf.entities if entity.dxftype == 'CIRCLE']
all_arcs = [entity for entity in dxf.entities if entity.dxftype == 'ARC']
print("line=%d,circle=%d,arc=%d"%(len(all_lines),len(all_cirs),len(all_arcs)))


def tut_dxfline(l):
    x0=l.start[0]
    y0=l.start[1]
    x1=l.end[0]
    y1=l.end[1]
    deg = arctan2(y1-y0,x1-x0) / pi*180
    dist = sqrt((x1-x0)**2+(y1-y0)**2)
    
    tut.penup()
    tut.setpos(x0,y0)
    tut.setheading(deg)
    tut.pendown()
    tut.forward(dist)
    
def tut_dxfarc(a):
    deg0=a.start_angle
    deg1=a.end_angle
    r=a.radius
    x0=a.center[0]
    y0=a.center[1]
    if deg0 > deg1:
        deg0 = deg0-360

#    print(x0,y0,r,deg0,deg1)

    tut.penup()
    tut.setpos(x0+r*cos(deg0/180.*pi),y0+r*sin(deg0/180.*pi))
    tut.setheading(deg0+90)
    tut.pendown()
    tut.circle(r,extent=(deg1-deg0))

def tut_dxfcir(c):
    r=c.radius
    x0=c.center[0]
    y0=c.center[1]

    tut.penup()
    tut.setpos(x0+r,y0)
    tut.setheading(90)
    tut.pendown()
    tut.circle(r)

#tut.speed("fast")
tut.speed("fastest")

#for i in range(len(all_cirs)):
#    tut_dxfcir(all_cirs[i])
#for i in range(len(all_arcs)):
#    tut_dxfarc(all_arcs[i])
#for i in range(len(all_lines)):
#    tut_dxfline(all_lines[i])

for entity in dxf.entities:
    if entity.dxftype == 'LINE':
        tut_dxfline(entity)
    elif entity.dxftype == 'CIRCLE':
        tut_dxfcir(entity)
    elif entity.dxftype == 'ARC':
        tut_dxfarc(entity)
    else:
        print(entity.dxftype)

print("Draw End")            
tut.mainloop()
    
              