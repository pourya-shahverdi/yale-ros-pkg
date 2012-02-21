#!/usr/bin/env python  
import roslib
roslib.load_manifest('imitation')
import rospy
import math
import tf

if __name__ == '__main__':
    rospy.init_node('skeleton_listener')
    listener = tf.TransformListener()
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            r_elbow_trans, r_elbow_rot = listener.lookupTransform('right_elbow_1', '/right_shoulder_1', rospy.Time(0))
            r_hand_trans, r_hand_rot = listener.lookupTransform('right_hand_1', '/right_shoulder_1', rospy.Time(0))
            print "Right Elbow | transform: %s rotation: %s" % (r_elbow_trans, r_elbow_rot)
            print "Right Hand | transform: %s rotation: %s" % (r_hand_trans, r_hand_rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): #TODO handle Extrapolation better?
            continue
        
        rate.sleep()