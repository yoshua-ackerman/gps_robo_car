#!/usr/bin/python
# -*- coding: utf-8 -*-

if __name__ == '__main__':
    from time import time
    start_t=time()

import rospy

from path_tracer import PathTracer,PathTracerDPReiwa
from path2d import path_transform2D, path_align_dir, path_sort, path_total_len, path_rosvis
from dxf2path import dxf2path

from course_location import SLANT_DEG
from numpy import deg2rad

if __name__ == '__main__':

    rospy.init_node('geoglypher')

    fname=rospy.get_param('~fname', "Geoglyph/nasca-hummingbird-50x50.dxf")
    scale=rospy.get_param('~scale', 1.0)
    rotate=rospy.get_param('~rotate', SLANT_DEG)
    xoffset=rospy.get_param('~xoffset', 0.0)
    yoffset=rospy.get_param('~yoffset', 0.0)

    path = dxf2path(fname)
#    path = dxf2path("Geoglyph/dpreiwa-arcs.dxf")
#    path = dxf2path("Geoglyph/dpreiwa-linepathtest.dxf")
#    path = dxf2path("Geoglyph/dpreiwa-linepathtest2.dxf")
#    path = dxf2path("Geoglyph/helloworld.dxf")
#    path = dxf2path("Geoglyph/OvercomeCOVID-19.dxf")
#    path = dxf2path("Geoglyph/nasca-hummingbird-50x50.dxf")
#    path = dxf2path("Geoglyph/nasca-hummingbird-100x100.dxf")
#    path = dxf2path("Geoglyph/path_sort_test.dxf")
    path_align_dir(path)
#    path=path_sort(path,0,0)
    if scale!=1.0 or rotate!=0.0 or xoffset!=0.0 or yoffset!=0.0:
        path_transform2D(path, scale=scale, rotation=deg2rad(rotate), xoff=xoffset, yoff=yoffset)
    print "path total length = %.1f[m]"%(path_total_len(path))

    path_rosvis(path, "static_path")

    path_tracer = PathTracer(path, 0.125)
#    path_tracer = PathTracerDPReiwa(path, 0.125,(1,4),(2,5))
    path_tracer.prompt(start_t)
    rospy.spin()

