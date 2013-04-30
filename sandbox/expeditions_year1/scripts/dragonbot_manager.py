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
        self.blink_client = actionlib.SimpleActionClient('/Blink_Server',BlinkAction)
        self.ik_client = actionlib.SimpleActionClient('/IK_Server',IKAction)
        self.lookat_client = actionlib.SimpleActionClient('/Lookat_Server',LookatAction)
        self.track_client = actionlib.SimpleActionClient('/Track_Server',TrackAction)

        self.tf_listener = tf.TransformListener()
     
        rospy.loginfo("Waiting for Dragonbot Action Servers")
        rospy.loginfo(" --- Expression Motion")
        self.express_client.wait_for_server()
        rospy.loginfo(" --- Visemes")
        self.viseme_client.wait_for_server()
        rospy.loginfo(" --- Blinking")
        self.blink_client.wait_for_server()
        rospy.loginfo(" --- IK")
        self.ik_client.wait_for_server()
        rospy.loginfo(" --- Lookat")
        self.lookat_client.wait_for_server()
        rospy.loginfo(" --- TF Tracking")
        self.track_client.wait_for_server()
        rospy.loginfo(" --- Speech")
        self.speech_client.wait_for_server()

        rospy.loginfo("Action servers connected")

        rospy.loginfo("Zeroing dragonbot")
        self.viseme_client.cancel_all_goals()
        self.express_client.cancel_all_goals()
        goal = dragon_msgs.msg.LookatGoal(state = "off")
        self.lookat_client.send_goal(goal)
        goal = dragon_msgs.msg.TrackGoal(on = False)
        self.track_client.send_goal(goal)
        self.pose_off()

        self.expressions = ["angry",
                            "frightened",
                            "puppy",
                            "sipping",
                            "confused",
                            "lovestruck",
                            "sad",
                            "tasting",
                            "disgusted",
                            "mischievous",
                            "surprised"]

        self.motions = ["afraid",
                        "blech",
                        "idunno",
                        "interest",
                        "mmhmmm",
                        "question",
                        "wakeup",
                        "yes",
                        "anticipation",
                        "farted",
                        "i_like_it",
                        "ilikeit",
                        "laugh1",
                        "mph",
                        "surprise",
                        "yay",
                        "yummm",
                        "bite",
                        "hungry",
                        "i_want_it",
                        "iwantit",
                        "meh",
                        "no",
                        "think",
                        "yawn"]

    def stop_speech(self):
        rospy.loginfo("Zeroing dragonbot")
        self.viseme_client.cancel_all_goals()
        self.express_client.cancel_all_goals()
        goal = dragon_msgs.msg.LookatGoal(state = "off")
        self.lookat_client.send_goal(goal)
        goal = dragon_msgs.msg.TrackGoal(on = False)
        self.track_client.send_goal(goal)
        self.speech_client.cancel_all_goals()
        self.pose_off()

    
    def track_frame(self, frame_name):
        goal = dragon_msgs.msg.TrackGoal(target = frame_name, on = True)
        self.track_client.send_goal(goal)
    
    def track_off(self):
        goal = dragon_msgs.msg.TrackGoal(on = False)
        self.track_client.send_goal(goal)

    def say(self, phrase_name,interrupt = True, wait = False):
         goal = dragon_msgs.msg.SpeechPlayGoal(phrase=phrase_name, interrupt=interrupt)
         self.speech_client.send_goal(goal)
         if wait:
             rospy.loginfo("Waiting for speech server result")
             self.speech_client.wait_for_result()


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
        with open(filename, 'r') as f:
            s = f.read()
        self.phrases = yaml.load(s)
        
    def express(self, expression_id, expression_type = None, wait = True):
        recognized = False
        if expression_type == "expression" or expression_type == None:
            if expression_id in self.expressions:
                goal = dragon_msgs.msg.ExpressionMotionGoal(type='expression',constant=expression_id)
                recognized = True
        if expression_type == "motion" or expression_type == None:
            if expression_id in self.motions:
                goal = dragon_msgs.msg.ExpressionMotionGoal(type='motion',constant=expression_id)
                recognized = True
        if recognized:
            self.express_client.send_goal(goal)
            rospy.loginfo("Waiting for expression server result")
            if wait:
                self.express_client.wait_for_result()
        else:
            rospy.logwarn("Expression/Motion not recognized")

    def pose_off(self):
        #rospy.loginfo("Pose disabled")
        #return

        rospy.loginfo("Idling dragonbot")
        goal = dragon_msgs.msg.IKGoal(state = "off", vel = 0.03, acc = 0.0004, x = 0, y = 0, z = 0, theta = 0, neck = 0)
        self.ik_client.send_goal(goal)
        

        
    def pose(self, x, y, z, theta = 0, neck = 0, vel=0.3, acc=0.001):
        #rospy.loginfo("Pose disabled")
        #return

        #range for x (back/forward): -2.3,2.5
        #range for y (right/left): -2.49,3.4
        #range for z (down/up): -2.0,2.6
        # theta --> better not to use
        # neck --> not currently working
        state = "on"

        if not theta == 0:
            rospy.logwarn("For now, don't use theta")
            theta = 0
        if not neck == 0:
            rospy.logwarn("Neck not working.")
            neck = 0
        if x > 2.5:
            rospy.logwarn("Pose x value too large, setting to max value")
            x = 2.5
        if x < -2.3:
            rospy.logwarn("Pose x value too small, setting to min value")
            x = -2.3
        if y > 3.4:
            rospy.logwarn("Pose y value too large, setting to max value")
            y = 3.4
        if y < -2.49:
            rospy.logwarn("Pose y value too small, setting to min value")
            y = -2.49
        if z > 2.6:
            rospy.logwarn("Pose z value too large, setting to max value")
            z = 2.6
        if z < -2.0:
            rospy.logwarn("Pose z value too small, setting to min value")
            z = -2.0

        goal = dragon_msgs.msg.IKGoal(state = state, vel = vel, acc = acc, x = x, y = y, z = z, theta = theta, neck = neck)
        self.ik_client.send_goal(goal)

    def lookat(self, x, y, z):
        # range for x (right/left): -300,300
        # range for y (down/up): -300,300
        # range for z (near/far): 20,400
        # note: state variable can have values: off, random, <anything else>
        rospy.loginfo("Lookat: (" + str(x) + "," + str(y) + "," + str(z)+ ")")

        if x > 300:
            rospy.logwarn("Pose x value too large, setting to max value")
            x = 300
        if x < -300:
            rospy.logwarn("Pose x value too small, setting to min value")
            x = -300
        if y > 300:
            rospy.logwarn("Pose y value too large, setting to max value")
            y = 300
        if y < -300:
            rospy.logwarn("Pose y value too small, setting to min value")
            y = -300
        if z > 400:
            rospy.logwarn("Pose z value too large, setting to max value")
            z = 400
        if z < 20:
            rospy.logwarn("Pose z value too small, setting to min value")
            z = 20



        if(float(x)/float(z) > 0.5):
            rospy.logwarn("Invalid lookat, twist value too high, looking at max value")
            x = int(float(z)/2.0)

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

    def eye_close(self):
        goal = dragon_msgs.msg.BlinkGoal(constant="START")
        self.blink_client.send_goal(goal)

    def eye_open(self):
        goal = dragon_msgs.msg.BlinkGoal(constant="STOP")
        self.blink_client.send_goal(goal)

    def blink(self):
        goal = dragon_msgs.msg.BlinkGoal(constant="ONCE")
        self.blink_client.send_goal(goal)

        

def main():
    rospy.init_node("dragonbot_manager_test")

    dm = DragonbotManager()

    #dm.load_phrases("phrases.yaml")
    rospy.sleep(3)
    print("IMA LOOKIN ATCHOO")
    dm.track_frame("head")
    rospy.sleep(30)
    print "LOOKAT OFF"
    dm.track_off()
    
if __name__ == '__main__':
    main()
