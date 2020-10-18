#!/usr/bin/env python

###test command
# % rostopic pub -1 /aqtalk_str std_msgs/String "gogogogoal"

import rospy
from std_msgs.msg import String

import subprocess
def aqtalk_cb(str):
    print str.data
    popen=subprocess.Popen("$HOME/Downloads/aquestalkpi/AquesTalkPi %s | aplay"%(str.data), \
                           shell=True)
    popen.wait() #wait for speak


rospy.init_node("aquestalkpi")
sub=rospy.Subscriber("aqtalk_str", String, aqtalk_cb, queue_size=3)
rospy.spin()

#from time import sleep
#if __name__ == '__main__':
#    while not rospy.is_shutdown():
#        sleep(180)

