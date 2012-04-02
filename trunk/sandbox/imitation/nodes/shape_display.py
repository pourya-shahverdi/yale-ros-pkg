#!/usr/bin/env python  
import math
import time

import roslib
roslib.load_manifest('visualization_msgs')
import rospy

from visualization_msgs.msg import Marker

if __name__ == '__main__':
    rospy.init_node('shape_display')
    
    shape_pub = rospy.Publisher('imitation_shapes', Marker)
    
    m = Marker()
    m.header.frame_id = '/openni_rgb_frame'
    m.header.stamp = rospy.Time(0)
    m.ns = 'imitation_robo'
    m.id = 0
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = 2
    m.pose.position.y = 0
    m.pose.position.z = 1
    m.pose.orientation.x = 0.0
    m.pose.orientation.y = 0.0
    m.pose.orientation.z = 0.0
    m.pose.orientation.w = 1.0
    m.scale.x = 1
    m.scale.y = 1
    m.scale.z = 0.05
    m.color.a = 1.0
    m.color.r = 0.0
    m.color.g = 1.0
    m.color.b = 0.0
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        shape_pub.publish(m)
        rate.sleep()