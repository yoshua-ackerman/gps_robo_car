#!/usr/bin/env python
## -*- coding: utf-8 -*-

import dxfgrabber
from numpy import pi, deg2rad
from path2d import Line2D,Arc2D, omit_pi_pi, omit_2pi_2pi, \
    path_transform2D, path_align_dir, path_sort, path_total_len


def dxf2path(filename, verbose=False):
    dxf = dxfgrabber.readfile(filename)
    if verbose:
        print dxf.header

    path=[]
    segment=None

    i=0
    for entity in dxf.entities:
        if entity.dxftype == 'LINE':
            if verbose:
                print "%4d:LINE:(x1,y1)=(%.1f,%.1f),(x2,y2)=(%.1f,%.1f)"% \
                (i, entity.start[0],entity.start[1], entity.end[0],entity.end[1])
            segment=Line2D(entity.start[0],entity.start[1],entity.end[0],entity.end[1])
            path.append(segment)
        elif entity.dxftype == 'ARC':
            if verbose:
                print "%4d:ARC :(start,end)=(%.1f,%.1f), R=%.1f, (cx,cy)=(%.1f,%.1f)"% \
                (i, entity.start_angle, entity.end_angle, entity.radius,\
            entity.center[0], entity.center[1])
            rad_start = omit_pi_pi(deg2rad(entity.start_angle))
            #DXF内で記述される円弧はCCWのみのよう。CWの可能性は考えない
            if entity.start_angle>entity.end_angle:
                crad = omit_2pi_2pi(deg2rad(entity.end_angle +360. -entity.start_angle))
            else:
                crad = omit_2pi_2pi(deg2rad(entity.end_angle - entity.start_angle))
            segment=Arc2D(entity.center, entity.radius, rad_start, crad)
            path.append(segment)
        elif entity.dxftype == 'CIRCLE':
            print i,"CIRCLE"
        else:
            print i,entity.dxftype
        i=i+1

    return path


if __name__ == '__main__':
    import argparse
    import turtle
    from numpy import rad2deg,deg2rad

    parser = argparse.ArgumentParser(description='confirm DXF input by visualisation with turtle')
    parser.add_argument('fname', help='path and name of DXF file')
    parser.add_argument('-s','--scale', type=float, default=1.0)
    parser.add_argument('-r','--rotate', type=float, default=0.0, help='rotation[deg]')
    parser.add_argument('-x','--xoffset', type=float, default=0.0)
    parser.add_argument('-y','--yoffset', type=float, default=0.0)
    parser.add_argument('--speed', type=int, default=5, help='0:no animation(fastest), or 1-10')
    parser.add_argument('--sort', action='store_true')

    args = parser.parse_args()

    path = dxf2path(args.fname, True)

    if args.scale!=1.0 or args.rotate!=0.0 or args.xoffset!=0.0 or args.yoffset!=0.0:
        path_transform2D(path, args.scale, deg2rad(args.rotate), args.xoffset, args.yoffset)

    if args.sort:
        path=path_sort(path)
    else:
        path_align_dir(path)

    #turtle.speed("fast")
    #turtle.speed("fastest")
    turtle.speed(args.speed)

    print "path total length = %.1f[m]"%(path_total_len(path))

    def turtle_line(l):
        turtle.penup()
        turtle.setpos(l.start_p())
        turtle.setheading(rad2deg(l.direction))
        turtle.pendown()
        turtle.forward(l.len)
    def turtle_arc(a):
        turtle.penup()
        turtle.setpos(a.start_p())
        turtle.pendown()
        if a.crad>=0:
            turtle.setheading(rad2deg(a.rad_start)+90.)
            turtle.circle(a.R, extent=rad2deg(a.crad))
        else:
        #この分岐なしでも正しい図は描けるが、後退しながら描く
        #Rの正負と角度の正負の取扱はturtle流と自分流、どちらが正統なのか不明
            turtle.setheading(rad2deg(a.rad_start)-90.)
            turtle.circle(-a.R, extent=rad2deg(-a.crad))

    for segment in path:
        if type(segment)==Line2D:
            turtle_line(segment)
        elif type(segment)==Arc2D:
            turtle_arc(segment)

    turtle.mainloop()
