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
                    continue
                warned = True
            else:
                warned = False
                self.lookat(x = trans[0]*100, y = trans[1]*100,z = trans[2]*100)
                self.feedback.x = trans[0]*100
                self.feedback.y = trans[1]*100
                self.feedback.z = trans[2]*100
                self.server.publish_feedback(self.feedback)
                rospy.sleep(0.2)
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
