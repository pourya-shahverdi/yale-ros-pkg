#!/usr/bin/env python

#Elaine Short
#Dragonbot python interface

import roslib; roslib.load_manifest('expeditions_year1')
import rospy
from dragonbot_manager import DragonbotManager
from std_msgs.msg import String

class DragonTeleop():
    def __init__(self):
        self.dm = DragonbotManager()
        self.exp_sub = rospy.Subscriber("/dragon_teleop_GUI/expressions", String, self.exp_callback)
        self.mot_sub = rospy.Subscriber("/dragon_teleop_GUI/motions", String, self.mot_callback)
        self.ph_sub = rospy.Subscriber("/dragon_teleop_GUI/phrases", String, self.ph_callback)
        rospy.loginfo("Ready!")

    def exp_callback(self, data):
        rospy.loginfo("Got expression: " + data.data)
        self.dm.express(data.data, "expression")

    def mot_callback(self, data):
        rospy.loginfo("Got motion: " + data.data)
        self.dm.express(data.data, "motion")

    def ph_callback(self, data):
        rospy.loginfo("Got phrase: " + data.data)
        self.dm.say(data.data)


def main():
    rospy.init_node('dragonbot_teleop')
    DragonTeleop()
    while not rospy.is_shutdown():
        rospy.spin()




if __name__ == '__main__':
    main()
