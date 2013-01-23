#!/usr/bin/env python
from collections import namedtuple
import roslib; roslib.load_manifest('dragonbot_ik')
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
from geometry_msgs.msg import Transform
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

def ik_publisher(tx,ty,tz,rx,ry,rz,rw):
    pub = rospy.Publisher('ik_topic', Transform)
    rospy.init_node('ik_publisher')
    #pub.publish(Transform((1,1,1),(0,0,0,0)))
    e = euler_from_quaternion([rx,ry,rz,rw])
    print e
    while not rospy.is_shutdown():
	translation = namedtuple("Rotation",["x","y","z"])
	translation.x=tx
	translation.y=ty
	translation.z=tz
	rotation = namedtuple("Translation",["x","y","z","w"])
	rotation.x=rx
	rotation.y=ry
	rotation.z=rz
	rotation.w=rw
        rospy.loginfo(Transform(translation,rotation))
        pub.publish(Transform(translation,rotation))
        rospy.sleep(1.0)
 
if __name__ == '__main__':
    try:
        # hard-coded values for x,y,z,theta
        # -1 values don't matter

        ik_publisher(0,0,-1,0,0,0,3.1415927)
    except rospy.ROSInterruptException: pass



