#!/usr/bin/env python  
import math
import time

import roslib
roslib.load_manifest('imitation')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('visualization_msgs')
roslib.load_manifest('nao_msgs')
import rospy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nao_msgs.msg import JointAnglesWithSpeed

FIXED_FRAME = 'openni_rgb_frame'
IM_NAMESPACE = 'imitation_robo'

def handle_joint_angles(data):
    """
    Given a JointAnglesWithSpeed message,
    Create and display the Marker messages to visualize
    all the joint angles
    """
    msgs = []
    for angle_type, angle_val in zip(data.joint_names, data.joint_angles):
        msgs.append(create_msg(angle_val, angle_type))  
    
    for msg in msgs:
        if msg:
            vis_pub.publish(msg)

def create_msg(val, angle_type):
    """
    Given an angle value (e.g. math.pi/2)
    and an angle type (e.g. 'ls_pitch'),
    Create a marker message to display the appropriate visualization
    """
    m = Marker()
    m.header.frame_id = FIXED_FRAME
    m.header.stamp = rospy.Time(0)
    m.ns = IM_NAMESPACE
    m.id = 0
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.01
    m.color.a = 1.0
    m.color.r = 1.0
    
    SIDE_LEN = 1
    
    if angle_type == 'LShoulderPitch':
        #create 3 points
        #      C
        #     /
        #   B  --- A
        
        A = Point(0, 1 + SIDE_LEN, 0)
        B = Point(0, 1, 0)
        C = Point(0, SIDE_LEN * math.cos(val), SIDE_LEN * math.sin(val))
        for pt in [A,B,C]:
            m.points.append(pt)
        return m
    else:
        return None

if __name__ == '__main__':
    rospy.init_node('im_vis')
    
    vis_pub = rospy.Publisher('imitation_visualization', Marker)
    rospy.Subscriber('joint_angles', JointAnglesWithSpeed, handle_joint_angles)
    
    rospy.spin()
