#!/usr/bin/env python
import roslib; roslib.load_manifest('dragonbot_ik')
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
import math
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState
from std_msgs.msg import Header,Time
from collections import namedtuple

def callback(transform):

    rospy.loginfo(rospy.get_name() + "\nI'm doing ik\n")

    # receive desired end effector positions in mm + theta 
    X = transform.translation.x
    Y = transform.translation.y
    Z = transform.translation.z
    Theta = transform.rotation.w
    print transform.translation
    print transform.rotation
    # kinematic constants in mm
    E1, E2, RB, B2, A, B, D = 29.239, 29.239, 55.48, 34.1675, 111.76, 53.98, 8.255
    J = [] # motor positions in radians
    counter = 0
    for m in range(0,4):
        if m == 0:
            xg = -Y-E1*(math.sin(Theta)-math.cos(Theta))+E2
            yg = X+E1*(math.cos(Theta)+math.sin(Theta))-B2
            zg = Z
        elif m == 1:
            xg = X+E1*(math.cos(Theta)+math.sin(Theta))+E2
            yg = Y+E1*(math.sin(Theta)-math.cos(Theta))+B2
            zg = Z
        elif m == 2:
            xg = Y-E1*(math.sin(Theta)-math.cos(Theta))+E2
            yg = -X+E1*(math.cos(Theta)+math.sin(Theta))-B2
            zg = Z
        elif m == 3:
            xg = -X+E1*(math.cos(Theta)+math.sin(Theta))+E2
            yg = -Y+E1*(math.sin(Theta)-math.cos(Theta))+B2
            zg = Z
            
        print "xg "+repr(xg)+" yg "+repr(yg)+" zg "+repr(zg)

        temp = math.pow((A-2*D)*(A-2*D)-yg*yg,0.5)
        aPrimeSquared = (temp+2*D)*(temp+2*D)
        c = math.pow((xg-RB)*(xg-RB)+zg*zg,0.5)
        #print (-aPrimeSquared+B*B+c*c)/(2*B*c)

        try:
            alpha = math.acos((-aPrimeSquared+B*B+c*c)/(2*B*c))
            print "worked " + repr((-aPrimeSquared+B*B+c*c)/(2*B*c))
            counter = counter + 1
        except ValueError:
            print "acos domain error " + repr((-aPrimeSquared+B*B+c*c)/(2*B*c))
            break
        else:
            beta = math.atan2(zg,xg-RB)
            J.append(beta-alpha)
    print "J:"
    for j in J:
        print " " + repr(j) # publish sensor_msgs/JointState to cmd_pos topic

    if counter == 4:
      # publish the 4 motor positions in radians
      motors = ("1_A","1_B","2_A","2_B")
      velocity = [0,0,0,0]
      effort = [0,0,0,0]
      pub = rospy.Publisher('cmd_pos',JointState)
      header = Header(0,rospy.get_rostime(),"")
      jointState = JointState(header=header,name=motors,position=J,velocity=velocity,effort=effort)
      pub.publish(jointState)


	#for i,j in enumerate(J):
	#	pub = rospy.Publisher('cmd_pos',JointState)
	#	header = Header(0,rospy.get_rostime(),"")
	#	jointState = JointState(header=header,name=[motors[i]],position=[j],velocity=[0.0],effort=[0.0])
	#	pub.publish(jointState)
    # else - what to publish?

#def callback(data):
#    print "works"

def ik_subscriber():
    rospy.init_node('ik_subscriber', anonymous=True)
    rospy.Subscriber('ik_topic', Transform, callback)
    rospy.spin()

if __name__ == '__main__':
    ik_subscriber()



