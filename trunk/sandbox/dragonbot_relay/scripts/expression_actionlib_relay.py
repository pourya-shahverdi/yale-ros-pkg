#!/usr/bin/env python

import roslib; roslib.load_manifest('dragonbot_relay')
import rospy
import dragon_msgs.msg
from dragon_msgs.msg import *
import actionlib

class ExpressionRelay(object):
  _result = dragon_msgs.msg.ExpressionMotionResult()
  _pub = rospy.Publisher('dragonbot_expression', dragon_msgs.msg.ExpressionMotionGoal )

  def __init__(self,name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, dragon_msgs.msg.ExpressionMotionAction, execute_cb=self.execute_cb)
    self._as.start()


  def execute_cb(self, goal):
    #publish goal to dragonbot topic
    self._pub.publish(goal)
    self._as.set_succeeded()

if __name__ == '__main__':
  rospy.init_node('expression_relay')
  ExpressionRelay(rospy.get_name())
  rospy.spin()

