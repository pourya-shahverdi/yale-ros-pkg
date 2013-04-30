#!/usr/bin/env python

# Elaine Short
# Dragonbot oculesic controller

import roslib; roslib.load_manifest('expeditions_year1')
import rospy
import actionlib
from dragon_msgs.msg import *
import tf

class TrackServer():
    feedback = TrackFeedback()
    result = TrackResult()

    def __init__(self):
        self.lookat_client = actionlib.SimpleActionClient('/Lookat_Server',LookatAction)
        self.tf_listener = tf.TransformListener()
        self.server = actionlib.SimpleActionServer('Track_Server', TrackAction, execute_cb=self.execute_cb)

        rospy.loginfo("Waiting for Dragonbot Action Servers")
        rospy.loginfo(" --- Lookat")
        self.lookat_client.wait_for_server()
        rospy.loginfo("Action servers connected")
        rospy.loginfo("Starting server...")
        self.server.start()

    def lookat(self, x, y, z):
        # range for x: -300,300
        # range for y: -300,300
        # range for z: 20,400
        rospy.loginfo("Lookat: (" + str(x) + "," + str(y) + "," + str(z)+ ")")

        if x > 300:
            rospy.logwarn("Pose x value too large, setting to max value")
            x = 300
        if x < -300:
            rospy.logwarn("Pose x value too small, setting to min value")
            x = -300
        if y > 300:
            rospy.logwarn("Pose y value too large, setting to max value")
            y = 300
        if y < -300:
            rospy.logwarn("Pose y value too small, setting to min value")
            y = -300
        if z > 400:
            rospy.logwarn("Pose z value too large, setting to max value")
            z = 400
        if z < 20:
            rospy.logwarn("Pose z value too small, setting to min value")
            z = 20



        if(float(x)/float(z) > 0.5):
            rospy.logwarn("Invalid lookat, twist value too high, looking at max value")
            x = int(float(z)/2.0)

        goal = dragon_msgs.msg.LookatGoal(x = x, y = y, z = z)
        self.lookat_client.send_goal(goal)

    def lookat_off(self):
        goal = dragon_msgs.msg.LookatGoal(state = "off")
        self.lookat_client.send_goal(goal)


    def execute_cb(self, goal):
        if not goal.on:
            rospy.loginfo("Tracking off.")
            self.lookat_off()
            self.result.result = "OFF"
            self.server.set_succeeded(self.result)
            return
        
        rospy.loginfo("Tracking frame: " + str(goal.target))
        self.feedback.target = goal.target
        warned = False
        success = False

        while not self.server.is_preempt_requested():
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/dragonbot',goal.target, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                if not warned:
                    rospy.logwarn("Error looking up transform.")
                    rospy.sleep(1.0)
                    continue
                warned = True
            else:
                warned = False
                self.lookat(x = trans[0]*100, y = trans[1]*100,z = trans[2]*100)
                self.feedback.x = trans[0]*100
                self.feedback.y = trans[1]*100
                self.feedback.z = trans[2]*100
                self.server.publish_feedback(self.feedback)
                rospy.sleep(0.5)
        if not success:
            rospy.loginfo("Preempted")
            self.server.set_preempted()
        else:
            self.result.result = "SUCCESS"
            self.server.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node('dragonbot_lookat')
    TrackServer()
    while not rospy.is_shutdown():
        rospy.spin
