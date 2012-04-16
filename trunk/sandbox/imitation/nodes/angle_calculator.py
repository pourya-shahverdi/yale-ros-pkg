#!/usr/bin/env python  
import math
import numpy
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
    left_shoulder, _ = listener.lookupTransform(FIXED_FRAME, 'left_shoulder', rospy.Time(0))
    left_elbow, _ = listener.lookupTransform(FIXED_FRAME, 'left_elbow', rospy.Time(0))
    left_wrist, _ = listener.lookupTransform(FIXED_FRAME, 'left_hand', rospy.Time(0))

    right_shoulder, _ = listener.lookupTransform(FIXED_FRAME, 'right_shoulder', rospy.Time(0))
    right_elbow, _ = listener.lookupTransform(FIXED_FRAME, 'right_elbow', rospy.Time(0))
    right_wrist, _ = listener.lookupTransform(FIXED_FRAME, 'right_hand', rospy.Time(0))
    
    return (timestamp, left_shoulder, left_elbow, left_wrist, right_shoulder, right_elbow, right_wrist)

PITCH_OFFSET = 0
PITCH_FACTOR = -1
ROLL_OFFSET = math.pi / 2
def shoulder_pitch_roll(side, opp_shoulder_coords, shoulder_coords, elbow_coords): 
    """
    Given the opp shoulder, shoulder, and elbow coordinates
    find the pitch and roll of the shoulder joint.
    Return a tuple (pitch, roll) in radians
    """
    #distance from opposite shoulder to elbow
    os2e = math.sqrt(sum([(o-e)**2 for o,e in zip(opp_shoulder_coords, elbow_coords)]))
    #distance from opposite shoulder to shoulder
    os2s = math.sqrt(sum([(o-s)**2 for o,s in zip(opp_shoulder_coords, shoulder_coords)])) #TODO this should be constant
    #distance from shoulder to elbow
    s2e = math.sqrt(sum([(e-s)**2 for e,s in zip(elbow_coords, shoulder_coords)])) #TODO this should be constant
    
    #ROLL
    #law of cosines
    roll = math.acos((os2s**2 + s2e**2 - os2e**2) / (2 * os2s * s2e))
    roll -= ROLL_OFFSET
    if side == 'right':
        roll *= -1
    
    #PITCH
    #project elbow onto shoulder plane.  This assumes that the shoulders are parallel to the ground
    projected_elbow_coords = [elbow_coords[0], elbow_coords[1], shoulder_coords[2]]
    elbow_proj_dist = elbow_coords[2] - shoulder_coords[2] #straight drop to plane
    shoulder_proj_dist = math.sqrt(sum([(p-s)**2 for p,s in zip(projected_elbow_coords, shoulder_coords)])) #shoulder to projected elbow point
    pitch = math.atan2(elbow_proj_dist, shoulder_proj_dist)
    pitch += PITCH_OFFSET
    pitch *= PITCH_FACTOR


#    norm_shoulder = normalize([(s-o) for s,o in zip(shoulder_coords, opp_shoulder_coords)])
#    norm_elbow = normalize([(e-s) for e,s in zip(elbow_coords, shoulder_coords)])
#    angle_vector = numpy.subtract(norm_elbow, norm_shoulder)
#    
#    pitch = math.atan2(angle_vector[2], angle_vector[0])
#    pitch += PITCH_OFFSET
#    pitch *= PITCH_FACTOR
    
    coords = Coords(0,0,0)
    
    return (pitch, roll, coords)

ELBOW_ROLL_OFFSET = math.pi
def elbow_yaw_roll(side, opp_shoulder_coords, shoulder_coords, elbow_coords, wrist_coords):
    """
    Given the opposite shoulder, shoulder, elbow, and wrist coordinates
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
    roll += ELBOW_ROLL_OFFSET
    if side == 'right':
        roll *= -1
    
    #YAW
    norm_elbow = normalize([(e-s) for e,s in zip(elbow_coords, shoulder_coords)])
    norm_wrist = normalize([(w-e) for w,e in zip(wrist_coords, elbow_coords)])
    angle_vector = numpy.subtract(norm_wrist, norm_elbow)
    yaw = math.atan2(angle_vector[0], angle_vector[1])
    
    yaw += math.pi / 2

    return (yaw, roll)

def normalize(vector):
    """
    Given a vector, normalize it
    """
    mag = numpy.sqrt(numpy.square(vector).sum())
    return numpy.divide(vector, mag)
    
if __name__ == '__main__':
    rospy.init_node('angle_calculator')
    listener = tf.TransformListener()
    publisher = rospy.Publisher('joint_angles', JointAnglesWithSpeed)
    
    cd_pub = rospy.Publisher('coord_data', Coords)
    
    rate = rospy.Rate(10)
    FRAME_NUM = 1;
    ASSOCIATED_FRAME = '0' #no frame
    while not rospy.is_shutdown():
        try:
            timestamp, ls, le, lw, rs, re, rw = get_coords()
            ls_pitch, ls_roll, info = shoulder_pitch_roll('left', rs, ls, le)
            le_yaw, le_roll = elbow_yaw_roll('left', rs, ls, le, lw)
            cd_pub.publish(info)

            rs_pitch, rs_roll, info = shoulder_pitch_roll('right', ls, rs, re)
            re_yaw, re_roll = elbow_yaw_roll('right', ls, rs, re, rw)
            
            header = Header(FRAME_NUM, timestamp, ASSOCIATED_FRAME)
            FRAME_NUM += 1
            joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
                           'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
            joint_angles = [ls_pitch, ls_roll, le_yaw, le_roll, rs_pitch, rs_roll, re_yaw, re_roll]
            #joint_angles = [ls_pitch, ls_roll, le_yaw, le_roll, rs_pitch, rs_roll, re_yaw, re_roll]
            speed = 1
            relative = 0 #absolute angle
            publisher.publish(JointAnglesWithSpeed(header, joint_names, joint_angles, speed, relative))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
