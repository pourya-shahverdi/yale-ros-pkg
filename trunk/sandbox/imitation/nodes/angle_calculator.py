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
    #TODO better job of getting time actually associated w/ transforms
    timestamp = rospy.Time.now()
    shoulder_trans, shoulder_rot = listener.lookupTransform('%s_shoulder' % side_name, opposite_shoulder, rospy.Time(0))
    elbow_trans, elbow_rot = listener.lookupTransform('%s_elbow' % side_name, opposite_shoulder, rospy.Time(0))
    wrist_trans, wrist_rot = listener.lookupTransform('%s_hand' % side_name, opposite_shoulder, rospy.Time(0))
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
    
    #find equation of plane through shoulder, perpendicular to line through shoulders
        #shoulder coords of (A,B,C)  and opposite shoulder at (0,0,0) 
        #yield plane of Ax + By + Cz + (A^2 + B^2 + C^2) = 0
    #project elbow onto plane - http://www.9math.com/book/projection-point-plane
    #project neck onto plane (assumes that this is always "up" perpendicular to shoulders)
    #Find elbow-shoulder-neck angle in this plane using law of cosines
    #Somehow determine whether the angle is positive or negative
    
    pitch = 3
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
    #find equation of plane through elbow, perpendicular to line from shoulder to elbow
        #elbow coords of (A,B,C) and shoulder coords of (D,E,F) 
        #yield plane of (A-D)x + (B-E)y + (C-F)z + (AD + BE + CF - A^2 - B^2 - C^2) = 0
    #project wrist onto plane - http://www.9math.com/book/projection-point-plane
    #... how to orient this angle?!
    yaw = 3
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
    
    rate = rospy.Rate(5) #TODO 10.0
    FRAME_NUM = 1;
    ASSOCIATED_FRAME = '0' #TODO? 0: no frame | 1: global frame
    while not rospy.is_shutdown():
        try:
            for side in ('left', ): #TODO 'right'):
                timestamp, shoulder_coords, elbow_coords, wrist_coords = get_coords(side)
                if side == 'left':
                    ls_pub.publish(Coords(shoulder_coords[0], shoulder_coords[1], shoulder_coords[2]))
                    le_pub.publish(Coords(elbow_coords[0], elbow_coords[1], elbow_coords[2]))
                elif side == 'right':
                    rs_pub.publish(Coords(shoulder_coords[0], shoulder_coords[1], shoulder_coords[2]))
                    re_pub.publish(Coords(elbow_coords[0], elbow_coords[1], elbow_coords[2]))
                s_pitch, s_roll = shoulder_pitch_roll(shoulder_coords, elbow_coords)
                e_yaw, e_roll = elbow_yaw_roll(shoulder_coords, elbow_coords, wrist_coords)
                header = Header(FRAME_NUM, timestamp, ASSOCIATED_FRAME)
                FRAME_NUM += 1
                joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll']
                joint_angles = [s_pitch, s_roll, e_yaw, e_roll]
                speed = 0.5 #half of max velocity
                relative = 0 #absolute angle
                publisher.publish(JointAnglesWithSpeed(header, joint_names, joint_angles, speed, relative))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
