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

BAR_LEN = 0.7
BAR_HEIGHT = 0.1
LINE_WIDTH = 0.01
SLIDER_WIDTH = 0.03
TEXT_HEIGHT = 0.08
BAR_SPACING = 0.05

def handle_joint_angles(data):
    """
    Given a JointAnglesWithSpeed message,
    Create and display the Marker messages to visualize
    all the joint angles
    """
    markers_msgs = []
    for angle_type, angle_val in zip(data.joint_names, data.joint_angles):
        markers_msgs.append(create_msgs(angle_val, angle_type))  
    
    for msgs in markers_msgs:
        if msgs:
            print msgs
            for msg in msgs:
                vis_pub.publish(msg)

def create_blank_msgs():
    """
    Create the basic box, slider, min_text, max_text, and joint_text Markers.
    Return the markers in an array.
    """
    box = Marker()
    box.header.frame_id = FIXED_FRAME
    box.header.stamp = rospy.Time(0)
    box.ns = IM_NAMESPACE
    box.type = Marker.LINE_STRIP
    box.action = Marker.ADD
    box.pose.orientation.w = 1.0
    box.scale.x = LINE_WIDTH
    box.color.a = 1.0
    box.color.r = 1.0
    
    slider = Marker()
    slider.header.frame_id = FIXED_FRAME
    slider.header.stamp = rospy.Time(0)
    slider.ns = IM_NAMESPACE
    slider.type = Marker.LINE_STRIP
    slider.action = Marker.ADD
    slider.pose.orientation.w = 1.0
    slider.scale.x = SLIDER_WIDTH
    slider.color.a = 1.0
    slider.color.r = 1.0
    
    min_text = Marker()
    min_text.header.frame_id = FIXED_FRAME
    min_text.header.stamp = rospy.Time(0)
    min_text.ns = IM_NAMESPACE
    min_text.type = Marker.TEXT_VIEW_FACING
    min_text.action = Marker.ADD
    min_text.scale.z = TEXT_HEIGHT
    min_text.color.a = 1.0
    min_text.color.r = 1.0
    min_text.color.g = 1.0
    min_text.color.b = 1.0
    
    max_text = Marker()
    max_text.header.frame_id = FIXED_FRAME
    max_text.header.stamp = rospy.Time(0)
    max_text.ns = IM_NAMESPACE
    max_text.type = Marker.TEXT_VIEW_FACING
    max_text.action = Marker.ADD
    max_text.scale.z = TEXT_HEIGHT
    max_text.color.a = 1.0
    max_text.color.r = 1.0
    max_text.color.g = 1.0
    max_text.color.b = 1.0
    
    joint_text = Marker()
    joint_text.header.frame_id = FIXED_FRAME
    joint_text.header.stamp = rospy.Time(0)
    joint_text.ns = IM_NAMESPACE
    joint_text.type = Marker.TEXT_VIEW_FACING
    joint_text.action = Marker.ADD
    joint_text.scale.z = TEXT_HEIGHT
    joint_text.color.a = 1.0
    joint_text.color.r = 1.0
    joint_text.color.g = 1.0
    joint_text.color.b = 1.0
    
    return [box, slider, min_text, max_text, joint_text]

def customize_msgs(val, angle_type, box, slider, min_text, max_text, joint_text, min_angle, max_angle, x, y, z, id_start):
    """
    Takes in the val, angle_type, the basic markers, 
    the min and max angle (in radians)
    the x,y,z of the anchor coords
    and the id to start numbering at
    """
    min_text.id = id_start
    min_text.text = '%s' % min_angle
    min_text.pose.position.x = x
    min_text.pose.position.y = y + BAR_LEN
    min_text.pose.position.z = z + BAR_HEIGHT + 2*LINE_WIDTH
    
    max_text.id = id_start + 1
    max_text.text = '%s' % max_angle
    max_text.pose.position.x = x
    max_text.pose.position.y = y
    max_text.pose.position.z = z + BAR_HEIGHT + 2*LINE_WIDTH
    
    joint_text.id = id_start + 2
    joint_text.text = '%s: %.2f' % (angle_type, val)
    joint_text.pose.position.x = x
    joint_text.pose.position.y = y + (BAR_LEN / 2)
    joint_text.pose.position.z = z + LINE_WIDTH*2
    
    slider.id = id_start + 3
    percentage = (val - min_angle) / (max_angle - min_angle)
    slider_y = y + (1-percentage)*BAR_LEN
    S1 = Point(x, slider_y, z)
    S2 = Point(x, slider_y, z + BAR_HEIGHT)
    for pt in [S1, S2]:
        slider.points.append(pt)
    if not (0 <= percentage <= 1): #make slider blue if out of parameter
        slider.color.b = 1
        slider.color.r = 0
    
    A = Point(x, y, z)
    B = Point(x, y + BAR_LEN, z)
    C = Point(x, y + BAR_LEN, z + BAR_HEIGHT)
    D = Point(x, y, z + BAR_HEIGHT)

    box.id = id_start + 4
    for pt in [A,B,C, D, A]:
        box.points.append(pt)
    
    return [box, slider, min_text, max_text, joint_text]

BAR_OFFSET = BAR_HEIGHT + LINE_WIDTH*2 + TEXT_HEIGHT + BAR_SPACING
JOINT_INFO = { #min/max angles from http://developer.aldebaran-robotics.com/doc/1-12/nao/hardware/kinematics/nao-joints-33.html
'LShoulderPitch': {'id_start':0, 'min_angle':-2.0857, 'max_angle':2.0857, 'coords':[1,1.5,0.5]},
'LShoulderRoll': {'id_start':5, 'min_angle':-0.3142, 'max_angle':1.3265, 'coords':[1,1.5,0.5 - 1*BAR_OFFSET]},
'LElbowYaw': {'id_start':10, 'min_angle':-2.0857, 'max_angle':2.0857, 'coords':[1,1.5,0.5 - 2*BAR_OFFSET]},
'LElbowRoll': {'id_start':15, 'min_angle':-1.5446, 'max_angle':-.0349, 'coords':[1,1.5,0.5 - 3*BAR_OFFSET]},
'RShoulderPitch': {'id_start':20, 'min_angle':-2.0857, 'max_angle':2.0857, 'coords':[1,0,0.5]},
'RShoulderRoll': {'id_start':25, 'min_angle':-1.3265, 'max_angle':0.3142, 'coords':[1,0,0.5 - 1*BAR_OFFSET]},
'RElbowYaw': {'id_start':30, 'min_angle':-2.0857, 'max_angle':2.0857, 'coords':[1,0,0.5 - 2*BAR_OFFSET]},
'RElbowRoll': {'id_start':35, 'min_angle':.0349, 'max_angle':1.5446, 'coords':[1,0,0.5 - 3*BAR_OFFSET]},
}

def create_msgs(val, angle_type):
    """
    Given an angle value (e.g. math.pi/2)
    and an angle type (e.g. 'ls_pitch'),
    Create a marker message to display the appropriate visualization
    """
    
    box, slider, min_text, max_text, joint_text = create_blank_msgs()
    
    joint_info = JOINT_INFO.get(angle_type)
    if not joint_info:
        return None
    
    x,y,z = joint_info['coords']
    return customize_msgs(
        val, angle_type, box, slider, min_text, max_text, joint_text, 
        joint_info['min_angle'],
        joint_info['max_angle'], 
        x, y, z, 
        joint_info['id_start']
    )

if __name__ == '__main__':
    rospy.init_node('im_vis')
    
    vis_pub = rospy.Publisher('imitation_visualization', Marker)
    rospy.Subscriber('joint_angles', JointAnglesWithSpeed, handle_joint_angles)
    
    rospy.spin()
