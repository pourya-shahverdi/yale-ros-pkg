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
    
    coords = Coords(elbow_proj_dist, shoulder_proj_dist, 0)
    
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
    #find plane through elbow perpendicular to shoulder-elbow vector
    perp_plane = perp_plane_thru_point([e-s for e,s in zip(elbow_coords, shoulder_coords)], elbow_coords)
    #project wrist onto that plane
    perp_proj_wrist = proj_point_to_plane(wrist_coords, perp_plane)
    #find plane through opp_shoulder, shoulder, and elbow
    base_plane = plane_thru_points(opp_shoulder_coords, shoulder_coords, elbow_coords)
    #project wrist point onto that plane
    base_proj_wrist = proj_point_to_plane(wrist_coords, base_plane)
    #Find the line that is the intersection of the 2 planes
    plane_intersect = plane_plane_intersect(perp_plane, base_plane)
    #then project the point onto that line
    base_perp_proj_wrist = proj_point_to_line(wrist_coords, plane_intersect)
    #atan2
    perp_to_base = math.sqrt(sum([(p-b)**2 for p,b in zip(perp_proj_wrist, base_perp_proj_wrist)]))
    base_to_elbow = math.sqrt(sum([(b-e)**2 for b,e in zip(base_perp_proj_wrist, elbow_coords)]))
    yaw = math.atan2(perp_to_base, base_to_elbow)

    return (yaw, roll)

###############
# TODO figure out how to move this to utility file
###############
import numpy

def perp_plane_thru_point(perp_vector, pnt):
    """
    Given a perpendicular vector and a point,
    Return the [A,B,C] of the Ax + By + Cz + D = 0 plane equation
    
    Eqn from: http://www.jtaylor1142001.net/calcjat/Solutions/VPlanes/VPPtNorm.htm
    """
    vx, vy, vz = perp_vector
    px, py, pz = pnt
    
    A,B,C = vx, vy, vz
    D = -1 * (vx*px + vy*py + vz*pz)
    
    return A,B,C,D

def proj_point_to_plane(pnt, perp_plane):
    """
    Given a point and the [A,B,C,D] of the plane equation,
    orthogonally project the point onto the plane.
    Return the [x,y,z] of the projected point
    
    Eqn from: http://www.9math.com/book/projection-point-plane
    """
    a,b,c,d = perp_plane
    u,v,w = pnt
    
    numerator = (a*u + b*v + c*w + d)
    denominator = numpy.square((a,b,c)).sum()
    t_0 = numerator / denominator
    
    x = u - a*t_0
    y = v - b*t_0
    z = w - c*t_0
    return x,y,z

def proj_point_to_line(pnt, line):
    """
    Given a 3-D point and the equation of a 3-D line,
    Orthogonally project the point onto the line and
    Return the [x,y,z] of the new point
    
    Eqn from: http://sci.tech-archive.net/Archive/sci.math/2006-09/msg04431.html
    """
    origin, dir_vector = line
    #calculate unit vector of the direction vector:
    u = numpy.sqrt(numpy.square(dir_vector).sum())
    #calculate vector from line origin to point
    v = [(p-o) for p,o in zip(pnt, origin)]
    #project v onto u
    proj = numpy.dot(u,v) * u
    
    return [(o + p) for o, p in zip(origin, proj)]

def plane_thru_points(pt1, pt2, pt3):
    """
    Given 3 3-dimensional points, return the
    [A,B,C,D] of the plane through those points
    
    Eqn from: http://www.jtaylor1142001.net/calcjat/Solutions/VPlanes/VP3Pts.htm
    """
    #Get 2 vectors in the plane
    v1 = [a-b for a,b in zip(pt1, pt2)]
    v2 = [a-b for a,b in zip(pt1, pt3)]
    
    perp_vector = numpy.cross(v1,v2)
    
    return perp_plane_thru_point(perp_vector, pt1)

def plane_plane_intersect(p1, p2):
    """
    Given the [A,B,C,D] form of 2 planes,
    Return the line that is their intersection in parametric form:
    x = A + Dt
    y = B + Et
    z = C + Ft
    [(A,B,C), (D,E,F)]
    
    Eqn from: http://www.jtaylor1142001.net/calcjat/Solutions/VPlanes/V2PLOfInt.htm
    """
    A1,B1,C1,D1 = p1
    A2,B2,C2,D2 = p2
    perp_vector = numpy.cross((A1,B1,C1), (A2,B2,C2))
    
    #TODO make sure that this doesn't screw up when parallel to X axis
    #set x to zero to find point on intersection of 2 planes
    y,z = numpy.linalg.solve(
        ((B1,C1), (B2,C2)), #solves the eqns B1x + C1y = -D1 and B2x + C2y = -D2
        (-1 * D1, -1 * D2)
    )
    
    return (0,y,z), perp_vector

    
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
            ls_pitch, ls_roll, info = shoulder_pitch_roll('left', rs, ls, le)
            le_yaw, le_roll = elbow_yaw_roll('left', rs, ls, le, lw)
            cd_pub.publish(info)
            
            rs_pub.publish(Coords(rs[0], rs[1], rs[2]))
            re_pub.publish(Coords(re[0], re[1], re[2]))
            rw_pub.publish(Coords(rw[0], rw[1], rw[2]))
            rs_pitch, rs_roll, info = shoulder_pitch_roll('right', ls, rs, re)
            re_yaw, re_roll = elbow_yaw_roll('right', ls, rs, re, rw)
            
            header = Header(FRAME_NUM, timestamp, ASSOCIATED_FRAME)
            FRAME_NUM += 1
            joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
                           'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
            joint_angles = [ls_pitch,ls_roll,0,le_roll,
                            rs_pitch,rs_roll,0,re_roll]
            #joint_angles = [ls_pitch, ls_roll, le_yaw, le_roll, rs_pitch, rs_roll, re_yaw, re_roll]
            speed = 1
            relative = 0 #absolute angle
            publisher.publish(JointAnglesWithSpeed(header, joint_names, joint_angles, speed, relative))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
