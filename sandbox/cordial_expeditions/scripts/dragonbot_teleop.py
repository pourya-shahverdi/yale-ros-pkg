#!/usr/bin/env python

#Elaine Short
#Dragonbot python interface

import roslib; roslib.load_manifest('expeditions_year1')
import rospy
from dragonbot_manager import DragonbotManager
from std_msgs.msg import String
from std_msgs.msg import Int32

class DragonTeleop():
    def __init__(self):
        self.dm = DragonbotManager()
        self.dm.pose_off()
        self.exp_sub = rospy.Subscriber("/dragon_teleop_GUI/expressions", String, self.exp_callback)
        self.mot_sub = rospy.Subscriber("/dragon_teleop_GUI/motions", String, self.mot_callback)
        self.ph_sub = rospy.Subscriber("/dragon_teleop_GUI/phrases", String, self.ph_callback)

        self.pose_on_sub = rospy.Subscriber("/dragon_teleop_GUI/pose_on", String, self.pos_on_cb)
        self.x_sub = rospy.Subscriber("/dragon_teleop_GUI/pose_x", Int32, self.x_pos_cb)
        self.y_sub = rospy.Subscriber("/dragon_teleop_GUI/pose_y", Int32, self.y_pos_cb)
        self.z_sub = rospy.Subscriber("/dragon_teleop_GUI/pose_z", Int32, self.z_pos_cb)

        self.look_on_sub = rospy.Subscriber("/dragon_teleop_GUI/lookat_on", String, self.look_on_cb)
        self.xl_sub = rospy.Subscriber("/dragon_teleop_GUI/look_x", Int32, self.x_look_cb)
        self.yl_sub = rospy.Subscriber("/dragon_teleop_GUI/look_y", Int32, self.y_look_cb)
        self.zl_sub = rospy.Subscriber("/dragon_teleop_GUI/look_z", Int32, self.z_look_cb)
        self.blink_sub = rospy.Subscriber("/dragon_teleop_GUI/blink", String, self.blink_cb)

        rospy.loginfo("Ready!")
        self.current_pose = [0,0,0]
        self.pose_on = False

        self.x_min = -2.3
        self.x_max = 2.5
        self.x_range = self.x_max - self.x_min
        self.y_min = -2.49
        self.y_max = 3.4
        self.y_range = self.y_max - self.y_min
        self.z_min = -2.0
        self.z_max = 2.6
        self.z_range = self.z_max - self.z_min

        self.v = 1
        self.a = .05

        self.current_look = [0,0,20]
        self.look_on = False

        self.x_lmin = -300
        self.x_lmax = 300
        self.x_lrange = self.x_lmax - self.x_lmin
        self.y_lmin = -300
        self.y_lmax = 300
        self.y_lrange = self.y_lmax - self.y_lmin
        self.z_lmin = 20
        self.z_lmax = 400
        self.z_lrange = self.z_lmax - self.z_lmin

    def exp_callback(self, data):
        rospy.loginfo("Got expression: " + data.data)
        self.dm.express(data.data)

    def mot_callback(self, data):
        rospy.loginfo("Got motion: " + data.data)
        self.dm.express(data.data)

    def ph_callback(self, data):
        rospy.loginfo("Got phrase: " + data.data)
        self.dm.say(data.data)

    def blink_cb(self, data):
        rospy.loginfo("Got blink command: " + data.data)
        if data.data == "once":
            self.dm.blink()
        elif data.data == "hold":
            self.dm.eye_close()
        elif data.data == "hold_off":
            self.dm.eye_open()

    def pos_on_cb(self, data):
        rospy.loginfo("Setting pose to: " + data.data)
        if data.data == "on":
            self.current_pose = [0,0,0]
            self.dm.pose(0,0,0)
            self.pose_on = True
        elif data.data == " off":
            self.pose_on = False
            self.dm.pose_off()

    def x_pos_cb(self, data):
        rospy.loginfo("Got x pose value: " + str(data.data))
        if self.pose_on:
            self.current_pose[0] = self.x_min + (float(data.data)/100) * self.x_range
            rospy.loginfo("Setting pose to: " + str(self.current_pose))
            self.dm.pose(self.current_pose[0], self.current_pose[1], self.current_pose[2], vel = self.v, acc = self.a)

    def y_pos_cb(self, data):
        rospy.loginfo("Got y pose value: " + str(data.data))
        if self.pose_on:
            self.current_pose[1] = self.y_min + (float(data.data)/100) * self.y_range
            rospy.loginfo("Setting pose to: " + str(self.current_pose))
            self.dm.pose(self.current_pose[0], self.current_pose[1], self.current_pose[2], vel = self.v, acc = self.a)

    def z_pos_cb(self, data):
        rospy.loginfo("Got z pose value: " + str(data.data))
        if self.pose_on:
            self.current_pose[2] = self.z_min + (float(data.data)/100) * self.z_range
            rospy.loginfo("Setting pose to: " + str(self.current_pose))
            self.dm.pose(self.current_pose[0], self.current_pose[1], self.current_pose[2], vel = self.v, acc = self.a)

    def look_on_cb(self, data):
        rospy.loginfo("Setting lookat to: " + data.data)
        if data.data == "on":
            self.current_look = [0,0,20]
            self.dm.lookat(0,0,20)
            self.look_on = True
        elif data.data == " off":
            self.look_on = False
            self.dm.lookat_off()

    def x_look_cb(self, data):
        rospy.loginfo("Got x lookat value: " + str(data.data))
        if self.look_on:
            self.current_look[0] = self.x_lmin + (float(data.data)/100) * self.x_lrange
            rospy.loginfo("Setting lookat to: " + str(self.current_look))
            self.dm.lookat(self.current_look[0], self.current_look[1], self.current_look[2])

    def y_look_cb(self, data):
        rospy.loginfo("Got y lookat value: " + str(data.data))
        if self.look_on:
            self.current_look[1] = self.y_lmin + (float(data.data)/100) * self.y_lrange
            rospy.loginfo("Setting lookat to: " + str(self.current_look))
            self.dm.lookat(self.current_look[0], self.current_look[1], self.current_look[2])

    def z_look_cb(self, data):
        rospy.loginfo("Got z lookat value: " + str(data.data))
        if self.look_on:
            self.current_look[2] = self.z_lmin + (float(data.data)/100) * self.z_lrange
            rospy.loginfo("Setting lookat to: " + str(self.current_look))
            self.dm.lookat(self.current_look[0], self.current_look[1], self.current_look[2])
        




def main():
    rospy.init_node('dragonbot_teleop')
    DragonTeleop()
    while not rospy.is_shutdown():
        rospy.spin()




if __name__ == '__main__':
    main()
