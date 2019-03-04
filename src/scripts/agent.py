#!/usr/bin/env python

# Publishes `position` of agent to the `/agent/position` topic.

import rospy
from geometry_msgs.msg import PointStamped
from osc4py3.as_eventloop import (osc_startup, osc_udp_server, osc_method,
    osc_process, osc_terminate)
from osc4py3 import oscmethod as osm

def handleOSCMessage(x, y, z):
    positionStr = "Position(%s, %s, %s)"%(x, y, z)
    rospy.loginfo(positionStr)
    position = PointStamped()
    position.header.stamp = rospy.Time.now()
    position.header.frame_id = "unreal"
    position.point.x = float(x)/100
    position.point.y = float(y)/100
    position.point.z = float(z)/100
    pub.publish(position)
    rate.sleep()

osc_startup()
osc_udp_server("0.0.0.0", 7000, "UnrealGAMS")

osc_method("/agent/0/pos", handleOSCMessage)

pub = rospy.Publisher('agent/position', PointStamped, queue_size=10)
rospy.init_node('agent', anonymous=True)
rate = rospy.Rate(10) # 10hz
 
if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            osc_process()
        osc_terminate()
    except rospy.ROSInterruptException:
        pass
