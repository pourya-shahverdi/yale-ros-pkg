#!/usr/bin/env python  
import math
import time

import roslib
roslib.load_manifest('imitation')
roslib.load_manifest('nao_msgs')
import rospy
import tf

from std_msgs.msg import Header
from nao_msgs.msg import JointAnglesWithSpeed
from imitation.msg import Coords

FIXED_FRAME = 'openni_rgb_frame'

def get_coords():
    """
    Find all the coordinates of the left/right shoulder, elbow, and wrist
    Return a tuple of (timestamp, ls, le, lw, rs, re, rw) 
    """
    timestamp = rospy.Time.now()
    #get coordinates relative to the fixed frame 
    left_shoulder, _ = listener.lookupTransform('left_shoulder', FIXED_FRAME, rospy.Time(0))
    left_elbow, _ = listener.lookupTransform('left_elbow', FIXED_FRAME, rospy.Time(0))
    left_wrist, _ = listener.lookupTransform('left_hand', FIXED_FRAME, rospy.Time(0))
    right_shoulder, _ = listener.lookupTransform('right_shoulder', FIXED_FRAME, rospy.Time(0))
    right_elbow, _ = listener.lookupTransform('right_elbow', FIXED_FRAME, rospy.Time(0))
    right_wrist, _ = listener.lookupTransform('right_hand', FIXED_FRAME, rospy.Time(0))

    return (timestamp, left_shoulder, left_elbow, left_wrist, right_shoulder, right_elbow, right_wrist)

PITCH_OFFSET = 0
def shoulder_pitch_roll(shoulder_coords, elbow_coords):
    """
    Given the shoulder and elbow coordinates
    (relative to the opposite shoulder at 0,0,0)
    find the pitch and roll of the shoulder joint.
    Return a tuple (pitch, roll) in radians
    """
    #distance from opposite shoulder to elbow
    os2e = math.sqrt(sum([x**2 for x in elbow_coords]))
    #distance from opposite shoulder to shoulder
    os2s = math.sqrt(sum([x**2 for x in shoulder_coords])) #TODO this should be constant
    #distance from shoulder to elbow
    s2e = math.sqrt(sum([(e-s)**2 for e,s in zip(elbow_coords, shoulder_coords)])) #TODO this should be constant
    
    #ROLL
    #law of cosines
    roll = math.acos((os2s**2 + s2e**2 - os2e**2) / (2 * os2s * s2e))
    
    #PITCH
    #project elbow onto shoulder plane.  This assumes that the shoulders are parallel to the ground
    projected_elbow_coords = [elbow_coords[0], elbow_coords[1], shoulder_coords[2]]
    elbow_proj_dist = elbow_coords[2] - shoulder_coords[2] #straight drop to plane
    shoulder_proj_dist = math.sqrt(sum([(p-s)**2 for p,s in zip(projected_elbow_coords, shoulder_coords)])) #shoulder to projected elbow point
    pitch = math.atan2(elbow_proj_dist, shoulder_proj_dist)
    pitch += PITCH_OFFSET
    
    coords = Coords(elbow_proj_dist, shoulder_proj_dist, 0)
    
    return (pitch, roll, coords)

def elbow_yaw_roll(shoulder_coords, elbow_coords, wrist_coords):
    """
    Given the shoulder, elbow, and wrist coordinates
    (relative to the opposite shoulder at 0,0,0)
    find the yaw and roll of the elbow joint.
    Return a tuple (yaw, roll) in radians
    """
    #distance from shoulder to elbow
    s2e = math.sqrt(sum([(e-s)**2 for e,s in zip(elbow_coords, shoulder_coords)])) #TODO this should be constant
    #distance from elbow to wrist
    e2w = math.sqrt(sum([(w-e)**2 for w,e in zip(wrist_coords, elbow_coords)])) #TODO this should be constant
    #distance from shoulder to wrist
    s2w = math.sqrt(sum([(w-s)**2 for w,s in zip(wrist_coords, shoulder_coords)]))
    
    #ROLL
    #law of cosines
    roll = math.acos((s2e**2 + e2w**2 - s2w**2) / (2 * s2e * e2w))
    
    #YAW
    yaw, roll = 0,0
    return (yaw, roll)
    
if __name__ == '__main__':
    rospy.init_node('angle_calculator')
    listener = tf.TransformListener()
    publisher = rospy.Publisher('joint_angles', JointAnglesWithSpeed)
    
    #for debugging
    ls_pub = rospy.Publisher('left_shoulder_coords', Coords)
    rs_pub = rospy.Publisher('right_shoulder_coords', Coords)
    le_pub = rospy.Publisher('left_elbow_coords', Coords)
    re_pub = rospy.Publisher('right_elbow_coords', Coords)
    lw_pub = rospy.Publisher('left_wrist_coords', Coords)
    rw_pub = rospy.Publisher('right_wrist_coords', Coords)
    
    cd_pub = rospy.Publisher('coord_data', Coords)
    
    rate = rospy.Rate(10)
    FRAME_NUM = 1;
    ASSOCIATED_FRAME = '0' #TODO? 0: no frame | 1: global frame
    while not rospy.is_shutdown():
        try:
            timestamp, ls, le, lw, rs, re, rw = get_coords()
            ls_pub.publish(Coords(ls[0], ls[1], ls[2]))
            le_pub.publish(Coords(le[0], le[1], le[2]))
            lw_pub.publish(Coords(lw[0], lw[1], lw[2]))
            ls_pitch, ls_roll, info = shoulder_pitch_roll(ls, le)
            le_yaw, le_roll = elbow_yaw_roll(ls, le, lw)
            cd_pub.publish(info)
            
            rs_pub.publish(Coords(rs[0], rs[1], rs[2]))
            re_pub.publish(Coords(re[0], re[1], re[2]))
            rw_pub.publish(Coords(rw[0], rw[1], rw[2]))
            rs_pitch, rs_roll, info = shoulder_pitch_roll(rs, re)
            re_yaw, re_roll = elbow_yaw_roll(rs, re, rw)
            
            header = Header(FRAME_NUM, timestamp, ASSOCIATED_FRAME)
            FRAME_NUM += 1
            joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
                           'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
            joint_angles = [0,0,0,0,0,0,0,0]
            #joint_angles = [ls_pitch, ls_roll, le_yaw, le_roll, rs_pitch, rs_roll, re_yaw, re_roll]
            speed = 1
            relative = 0 #absolute angle
            publisher.publish(JointAnglesWithSpeed(header, joint_names, joint_angles, speed, relative))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
