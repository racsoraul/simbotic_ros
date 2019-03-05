#!/usr/bin/env python

# Publishes `pose` of agent to the `/agent/pose` topic.

import rospy
from geometry_msgs.msg import PoseStamped
from osc4py3.as_eventloop import (osc_startup, osc_udp_server, osc_method,
    osc_process, osc_terminate)
from osc4py3 import oscmethod as osm

def handleOSCPoseMessage(px, py, pz, qx, qy, qz, qw):
    poseStr = "Position(%s, %s, %s), Quaternion(%s, %s, %s, %s)" % (
        px, py, pz, qx, qy, qz, qw)
    rospy.loginfo(poseStr)
    agentP = PoseStamped()
    agentP.header.stamp = rospy.Time.now()
    agentP.header.frame_id = "unreal"
    agentP.pose.position.x = float(px)/100
    agentP.pose.position.y = -float(py)/100
    agentP.pose.position.z = float(pz)/100
    agentP.pose.orientation.x = float(qx)/100
    agentP.pose.orientation.y = float(qy)/100
    agentP.pose.orientation.z = -float(qz)/100
    agentP.pose.orientation.w = float(qw)/100
    pub.publish(agentP)
    rate.sleep()

osc_startup()
osc_udp_server("0.0.0.0", 7000, "UnrealGAMS")

osc_method("/agent/0/pose", handleOSCPoseMessage)

pub = rospy.Publisher('agent/pose', PoseStamped, queue_size=10)
rospy.init_node('agent', anonymous=True)
rate = rospy.Rate(10) # 10hz
 
if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            osc_process()
        osc_terminate()
    except rospy.ROSInterruptException:
        pass
