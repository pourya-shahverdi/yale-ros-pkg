#!/usr/bin/env python

#Elaine Short
#Expeditions Year 1 Experiment

import roslib; roslib.load_manifest('expeditions_year1')
import rospy
import smach
import smach_ros
import sys

# class definitions for states are in the file experiment_states.py
from experiment_states import *

from dragonbot_manager import DragonbotManager
#from dragonbot_simulator import DragonbotManager
from tablet_manager import TabletManager
import yaml
from dialogue_manager import *

def main():
    rospy.init_node('intro_controller')
    dm = DragonbotManager()
    tm = TabletManager()

    rospy.loginfo("loading dialogue")
    with open("intro_dialogue.yaml", 'r') as f:
            s = f.read()

    dialogue_name = "intro"
    session_name = "introduction"
    dialogue = yaml.load(s)[session_name]
    
    dg = DialogueManager(dm, tm, "dragon_GUI/", dialogue_name, dialogue[dialogue_name], session_name)
    
    dm.eye_close()
    tm.change("sleep")
    while not rospy.is_shutdown() and not tm.last_press("/dragon_GUI/sleep") == 1:
        dm.say("intro-05-snore_sleep", wait = False)
        rospy.sleep(6.0)
    dm.express("wakeup")
    dm.eye_open()
    dg.play_dialogue("intro_dialogue")
    dm.eye_close()
    while not rospy.is_shutdown():
        dm.say("intro-05-snore_sleep", wait = False)
        rospy.sleep(6.0)

if __name__ == '__main__':
    main()