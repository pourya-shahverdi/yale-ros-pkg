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

        self.speech_client = actionlib.SimpleActionClient('/SpeechPlay_Server',SpeechPlayAction)
        self.express_client = actionlib.SimpleActionClient('/ExpressionMotion_Server',ExpressionMotionAction)
        self.viseme_client = actionlib.SimpleActionClient('/Viseme_Server',VisemeAction)
        self.ik_client = actionlib.SimpleActionClient('/IK_Server',IKAction)
        self.lookat_client = actionlib.SimpleActionClient('/Lookat_Server',LookatAction)
        self.track_client = actionlib.SimpleActionClient('/Track_Server',TrackAction)

        self.tf_listener = tf.TransformListener()
     
        rospy.loginfo("Waiting for Dragonbot Action Servers")
        rospy.loginfo(" --- Expression Motion")
        self.express_client.wait_for_server()
        rospy.loginfo(" --- Visemes")
        self.viseme_client.wait_for_server()
        rospy.loginfo(" --- IK")
        self.ik_client.wait_for_server()
        rospy.loginfo(" --- Lookat")
        self.lookat_client.wait_for_server()
        rospy.loginfo(" --- TF Tracking")
        self.track_client.wait_for_server()
        rospy.loginfo("Action servers connected")

        rospy.loginfo("Zeroing dragonbot")
        self.viseme_client.cancel_all_goals()
        self.express_client.cancel_all_goals()
        goal = dragon_msgs.msg.LookatGoal(state = "off")
        self.lookat_client.send_goal(goal)
        goal = dragon_msgs.msg.TrackGoal(on = False)
        self.track_client.send_goal(goal)

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
                        "cheer",
                        "heh",
                        "ilikeit",
                        "i_like_it",
                        "laugh",
                        "mph",
                        "question",
                        "sneeze",
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
                        "weee",
                        "yawn"]

    
    def track_frame(self, frame_name):
        goal = dragon_msgs.msg.TrackGoal(target = frame_name, on = True)
        self.track_client.send_goal(goal)
    
    def track_off(self):
        goal = dragon_msgs.msg.TrackGoal(on = False)
        self.track_client.send_goal(goal)

    def say(self, phrase_name):
         goal = dragon_msgs.msg.SpeechPlayGoal(phrase=phrase_name)
         self.speech_client.send_goal(goal)


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
        
    def express(self, expression_id, expression_type):
        if expression_type == "expression":
            if expression_id in self.expressions:
                goal = dragon_msgs.msg.ExpressionMotionGoal(type='expression',constant=expression_id)
            else: 
                rospy.logwarn("Expression not recognized")
                return
        if expression_type == "motion":
            if expression_id in self.motions:
                goal = dragon_msgs.msg.ExpressionMotionGoal(type='motion',constant=expression_id)
            else:
                rospy.logwarn("Motion not recognized")
                return
        self.express_client.send_goal(goal)
        
        
    def pose(self, vel, acc, x, y, z, theta, neck):
        goal = dragon_msgs.msg.IKGoal(vel = vel, acc = acc, x = x, y = y, z = z, theta = theta, neck = neck)
        self.ik_client.send_goal(goal)

    def lookat(self, x, y, z):
        # range for x: -300,300
        # range for y: -300,300
        # range for z: 20,400
        goal = dragon_msgs.msg.LookatGoal(x = x, y = y, z = z)
        self.lookat_client.send_goal(goal)

    def lookat_off(self):
        goal = dragon_msgs.msg.LookatGoal(state = "off")
        self.lookat_client.send_goal(goal)

    def lookat_frame(self, frame):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/dragonbot',frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Error looking up transform.")
        else:
            self.lookat(trans[0]*100,trans[1]*100,trans[2]*100) #m to cm
        

def main():
    rospy.init_node("dragonbot_manager_test")

    dm = DragonbotManager()

    dm.load_phrases("phrases.yaml")

    '''dm.lookat(0, 0 ,400)
    rospy.sleep(5.0)
    dm.lookat_off()
    dm.lookat(100,100,30)
    rospy.sleep(5.0)
    dm.lookat(0,0,400)
    rospy.sleep(5.0)
    #dm.lookat_frame("target")

    rospy.sleep(5.0)

    dm.track_frame("target")
    rospy.sleep(5.0)
    dm.track_off()'''


    dm.say("teaching")
    rospy.sleep(30)


    expressions = ["angry",
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
    
    motions = ["afraid",
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
               "cheer",
               "heh",
               "ilikeit",
               "i_like_it",
               "laugh",
               "mph",
               "question",
               "sneeze",
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
               "weee",
               "yawn"]

    '''for expression in motions:
        dm.express(expression)
        rospy.sleep(7)

    for expression in expressions:
        dm.express(expression)
        rospy.sleep(3.0)

    for motion in motions:
        dm.express(motion)
        rospy.sleep(5.0)'''

if __name__ == '__main__':
    main()
