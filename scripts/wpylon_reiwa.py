#!/usr/bin/python
# -*- coding: utf-8 -*-

if __name__ == '__main__':
    from time import time
    start_t=time()

import rospy

from path_tracer import PathTracerDPReiwa
from path2d import path_transform2D, path_align_dir, path_total_len, path_rosvis
from dxf2path import dxf2path

from course_location import SLANT_DEG
from numpy import deg2rad

if __name__ == '__main__':

#    path = dxf2path("Geoglyph/dpreiwa-arcs.dxf")
#    path = dxf2path("Geoglyph/dpreiwa-linepathtest.dxf")
    path = dxf2path("Geoglyph/dpreiwa-linepathtest2.dxf")

    path_align_dir(path)
    path_transform2D(path, scale=1., rotation=deg2rad(SLANT_DEG), xoff=0., yoff=0.)
    print "path total length = %.1f[m]"%(path_total_len(path))

    rospy.init_node('wpylon_reiwa')

    path_rosvis(path, "static_path")

    path_endlimit=rospy.get_param('~path_endlimit', 0.125) #0.25?
    if len(path)==6:
        path_tracer = PathTracerDPReiwa(path, path_endlimit, (1,4),(2,5))
    elif len(path)==8:
        path_tracer = PathTracerDPReiwa(path, path_endlimit, (1,5),(3,7))
    else: #unknown path
        import sys
        sys.exit()
    path_tracer.prompt(start_t)
    rospy.spin()
