#!/usr/bin/env python

# Elaine Short
# Python Simulator for the teleop tablet interface

import roslib; roslib.load_manifest('dragonbot_python')
import rospy
from Tkinter import *



def main():
    rospy.init_node("teleop_dialogue_interface")
    root = Tk()
    w = label(root, text="GO DRAGONBOT")
    w.pack()
    root.mainloop()


if __name__ == '__main__':
    main()







