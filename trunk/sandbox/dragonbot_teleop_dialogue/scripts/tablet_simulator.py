#!/usr/bin/env python

#Elaine Short
#Tablet python interface

import roslib; roslib.load_manifest('dragonbot_teleop_dialogue')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Duration
from interface_srv.srv import *
from interface_srv.msg import GUIElement
from Tkinter import *
from tkFont import *


class TabletSimulator():
    def __init__(self, tk_master):
        rospy.wait_for_service('gui_srv')
        try:
            srv_gui = rospy.ServiceProxy('gui_srv', GUIList)
            guis = srv_gui()
        except rospy.ServiceException, e:
            rospy.logerr("Failed to get GUIs: %s"%e)
            sys.exit()
        self.guis = guis.guis
        self.topic_change_sub = rospy.Subscriber('/change_topic', String, self.topic_change) 
        self.gui_changed = False
        rospy.loginfo("Tablet Started")

        self.window = tk_master
        self.frame = Frame(self.window)
        self.frame.grid(padx = 10, pady = 10)

    def get_gui(self, name):
        r = filter(lambda g: g.guiname == name, self.guis)
        return r[0]

    def add_button_group(self, label_string, topic, row):
        button_font = ("courier", 12)
        labels = label_string.split(';')
        i = 0
        for label in labels:
            cb = lambda l=label,t=topic: self.button_group_cb(t, l)
            Button(self.frame, text=label, command = cb, font = button_font, padx=5, pady=5).grid(row=row, column = i)
            i += 1

    def button_group_cb(self, topic, button_name):
        p = rospy.Publisher(topic, String)
        p.publish(button_name)

    def add_button(self, label, topic, row):
        button_font = ("courier", 12)
        Button(self.frame, text=label, command = lambda: self.button_cb(topic), font = button_font, padx=5, pady=5).grid(row=row)

    def button_cb(self, topic):
        p = rospy.Publisher(topic, Int32)
        p.publish(0)

    def topic_change(self, data):
        print "Changing to GUI " + data.data
        for w in self.frame.grid_slaves():
            w.grid_remove()
        i = 0
        for g in self.get_gui(data.data).elements:
            kind = g.type
            if kind == GUIElement.BUTTON_ID:
                self.add_button(g.label, g.topic, i)
            elif kind == GUIElement.BUTTONGROUP_ID:
                self.add_button_group(g.label_string, g.topic, i)
            else:
                rospy.logwarn("Unsupported GUI element type")
        

def main():
    rospy.init_node('dyn_interface_GUI')
    root = Tk()
    root.title("Dynamic Interface GUI")
    #root.minsize(100,50)
    #sw = root.winfo_screenwidth()
    #sh = root.winfo_screenheight()
    #root.geometry("+" + str(sw/4) + "+" + str(sh/4))
    TabletSimulator(root)
    #while not rospy.is_shutdown():
    root.mainloop()


if __name__ == '__main__':
    main()

