#!/usr/bin/env python

#Elaine Short
#Dragonbot python interface

import roslib; roslib.load_manifest('expeditions_year1')
import rospy
from threading import Thread
from threading import Event
import yaml
from dragon_msgs.msg import *
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import actionlib
import time
import tf

class DragonbotManager():

    def __init__(self):
        self.phrases = {}

        self.expressions = ["angry",
                            "disgusted",
                            "frustrated",
                            "mischievous",
                            "shy",
                            "bored",
                            "bored_unimpressed",
                            "ecstatic",
                            "happy",
                            "puppy",
                            "surprised",
                            "confused",
                            "frightened",
                            "lovestruck",
                            "sad"]

        self.motions = ["afraid",
                        "blech",
                        "farted",
                        "idunno",
                        "interest",
                        "mmhmmm",
                        "no",
                        "shy",
                        "think",
                        "whoah",
                        "yes",
                        "anticipation",
                        #"cheer",
                        "heh",
                        "ilikeit",
                        "i_like_it",
                        "laugh",
                        "mph",
                        "question",
                        #"sneeze",
                        "wakeup",
                        "yay",
                        "yummm",
                        "bite",
                        "crazy_laugh",
                        "hungry",
                        "iwantit",
                        "i_want_it",
                        "meh",
                        "nah_nah",
                        "nahnah",
                        "sad",
                        "surprise",
                        #"weee",
                        "yawn"]

    
    def track_frame(self, frame_name):
        print "Tracking frame: " + frame_name

    def track_off(self):
        print "Tracking off."

    def say(self, phrase_name):
        if phrase_name in self.phrases:
            print "Saying phrase: " + phrase_name
            print "====================================="
            print self.phrases[phrase_name]["text"]
            print "-------------------------------------"
        else:
            print "Saying (unloaded) phrase: " + phrase_name

    #loads phrases from a yaml file
    # format:
    # phrases (list)
    #  text (str) #for reference
    #  file (str) #file with audio
    #  visemes (list)
    #   type (string)
    #   start (float)
    #   end (float)
    #  actions (list)
    #   type (string)
    #   time (float)
    def load_phrases(self, filename):
        f = open(filename, 'r')
        s = f.read()
        self.phrases = yaml.load(s)
        

    def express(self, expression_id, expression_type = None):
        recognized = False
        if expression_type == "expression" or expression_type == None:
            if expression_id in self.expressions:
                goal = dragon_msgs.msg.ExpressionMotionGoal(type='expression',constant=expression_id)
                recognized = True
            else: 
                rospy.logwarn("Expression not recognized")
        if expression_type == "motion" or expression_type == None:
            if expression_id in self.motions:
                goal = dragon_msgs.msg.ExpressionMotionGoal(type='motion',constant=expression_id)
                recognizsed = True
            else:
                rospy.logwarn("Motion not recognized")
        if recognized:
            print "Playing expression/motion: " + expression_id
        
        
    def pose(self, vel, acc, x, y, z, theta, neck):
        print "Moving to pose with vel: {vel} acc: {acc} position: ({x},{y},{z}) theta: {theta} neck: {neck}".format(vel = vel, acc = acc, x = x, y = y, z = z, theta = theta, neck = neck)

    def lookat(self, x, y, z):
        # range for x: -300,300
        # range for y: -300,300
        # range for z: 20,400
        print "Looking at: ({x},{y},{z})".format(x = x, y = y, z = z)

    def lookat_off(self):
        print "Lookat off"

    def lookat_frame(self, frame):
        print "Looking at frame: " + frame
        

def main():
    rospy.init_node("dragonbot_manager_test")

    dm = DragonbotManager()

    dm.load_phrases("phrases.yaml")

    dm.say("teaching")

if __name__ == '__main__':
    main()
