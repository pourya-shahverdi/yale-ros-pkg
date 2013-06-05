#!/usr/bin/env python

# Elaine Short
# Python Simulator for the teleop tablet interface

import roslib; roslib.load_manifest('dragonbot_python')
import rospy
from Tkinter import *
from tkFont import *

class DynamicInterfaceGUI():
    def __init__(self, master):
        frame = Frame(master)
        frame.pack()

        button_font = ("Courier", 12)

        self.button = Button(frame, text="QUIT",font=button_font, command=frame.quit, height=2)
        self.button.grid(row=0, column=0, pady=5, padx=5)

        self.hi_there = Button(frame, text="Hello", font=button_font, command=self.say_hi, height=2)
        self.hi_there.grid(row=0, column=1, pady=5, padx=5)
        self.toggle = False
        self.window = master


    def say_hi(self):
        if self.toggle:
            print "Goodbye!"
            print self.window.geometry()
        else:
            print "Hello!"

        self.toggle = not self.toggle
        

def main():
    rospy.init_node("teleop_dialogue_interface")
    root = Tk()
    root.title("Dynamic Interface GUI")

    gui = DynamicInterfaceGUI(root)
    root.geometry("+700+400")




if __name__ == '__main__':
    main()







