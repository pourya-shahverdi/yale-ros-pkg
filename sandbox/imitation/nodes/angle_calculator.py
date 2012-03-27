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

def get_coords(side_name):
    """
    Given a side ("left" or "right"),
    Find all the coordinates of the shoulder, elbow, and wrist
    in terms of the opposite shoulder.
    Return a tuple of (timestamp, shoulder_coords, elbow_coords, wrist_coords) 
    """
    if side_name == 'right':
        opposite_shoulder = '/left_shoulder'
    elif side_name == 'left':
        opposite_shoulder = '/right_shoulder'

    timestamp = rospy.Time.now()
    #get coordinates relative to the fixed frame (can't do relative to opposite shoulder - coord frame is rotating over time!) 
    opp_shoulder_trans, opp_shoulder_rot = listener.lookupTransform(opposite_shoulder, FIXED_FRAME, rospy.Time(0))
    shoulder_trans, shoulder_rot = listener.lookupTransform('%s_shoulder' % side_name, FIXED_FRAME, rospy.Time(0))
    elbow_trans, elbow_rot = listener.lookupTransform('%s_elbow' % side_name, FIXED_FRAME, rospy.Time(0))
    wrist_trans, wrist_rot = listener.lookupTransform('%s_hand' % side_name, FIXED_FRAME, rospy.Time(0))
    
    #normalize coords relative to opposite shoulder translation
    shoulder_trans = [s - os for s, os in zip(shoulder_trans, opp_shoulder_trans)]
    elbow_trans = [e - os for e, os in zip(elbow_trans, opp_shoulder_trans)]
    wrist_trans = [w - os for w, os in zip(wrist_trans, opp_shoulder_trans)]
    
    return (timestamp, shoulder_trans, elbow_trans, wrist_trans)

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
    #subtract shoulder-shoulder vector from shoulder-elbow vector to
    #normalize relative to camera
    norm_elbow_coords = [ec - sc for ec, sc in zip(elbow_coords, shoulder_coords)]
    y, z = norm_elbow_coords[1:3]
    pitch = math.atan2(y, z)
    
    le_dist_pub.publish(Coords(pitch, roll, 0))
    
    pitch = 0
    return (pitch, roll)

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
    
    le_dist_pub = rospy.Publisher('left_elbow_dist', Coords)
    
    rate = rospy.Rate(10)
    FRAME_NUM = 1;
    ASSOCIATED_FRAME = '0' #TODO? 0: no frame | 1: global frame
    while not rospy.is_shutdown():
        try:
            for side in ('left', 'right'):
                timestamp, shoulder_coords, elbow_coords, wrist_coords = get_coords(side)
                if side == 'left':
                    ls_pub.publish(Coords(shoulder_coords[0], shoulder_coords[1], shoulder_coords[2]))
                    le_pub.publish(Coords(elbow_coords[0], elbow_coords[1], elbow_coords[2]))
                    lw_pub.publish(Coords(wrist_coords[0], wrist_coords[1], wrist_coords[2]))
                    ls_pitch, ls_roll = shoulder_pitch_roll(shoulder_coords, elbow_coords)
                    le_yaw, le_roll = elbow_yaw_roll(shoulder_coords, elbow_coords, wrist_coords)
                elif side == 'right':
                    rs_pub.publish(Coords(shoulder_coords[0], shoulder_coords[1], shoulder_coords[2]))
                    re_pub.publish(Coords(elbow_coords[0], elbow_coords[1], elbow_coords[2]))
                    rw_pub.publish(Coords(wrist_coords[0], wrist_coords[1], wrist_coords[2]))
                    rs_pitch, rs_roll = shoulder_pitch_roll(shoulder_coords, elbow_coords)
                    re_yaw, re_roll = elbow_yaw_roll(shoulder_coords, elbow_coords, wrist_coords)
            
            header = Header(FRAME_NUM, timestamp, ASSOCIATED_FRAME)
            FRAME_NUM += 1
            joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
                           'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
            joint_angles = [ls_pitch, ls_roll, le_yaw, le_roll, rs_pitch, rs_roll, re_yaw, re_roll]
            speed = 0.5 #half of max velocity
            relative = 0 #absolute angle
            publisher.publish(JointAnglesWithSpeed(header, joint_names, joint_angles, speed, relative))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
