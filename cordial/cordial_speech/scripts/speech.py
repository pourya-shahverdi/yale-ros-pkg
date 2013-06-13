#!/usr/bin/env python

# Elaine Short
# Dragonbot oculesic controller

import roslib; roslib.load_manifest('dragonbot_python')
import rospy
import actionlib
from dragon_msgs.msg import *
from cordial_sound.msg import *
from cordial_sound.msg import SoundRequest
from cordial_sound.libsoundplay import SoundClient

import yaml
import sys
import wave
import contextlib

class SpeechPlayServer():
    feedback = SpeechPlayFeedback()
    result = SpeechPlayResult()

    def __init__(self, phrase_file):
        self.sound_client = SoundClient()
        rospy.sleep(0.5)
        self.sound_client.stopAll()
     
        #self.animation_client = actionlib.SimpleActionClient('/Animation_Server',ExpressionMotionAction)
        self.viseme_client = actionlib.SimpleActionClient('/Viseme_Server',VisemeAction)
        #self.lookat_client = actionlib.SimpleActionClient('/Lookat_Server',LookatAction)

        rospy.loginfo("Waiting for Cordial Action Servers")
        #rospy.loginfo(" --- Animation")
        #self.animation_client.wait_for_server()
        rospy.loginfo(" --- Visemes")
        self.viseme_client.wait_for_server()
        #rospy.loginfo(" --- Lookat")
        #self.lookat_client.wait_for_server()     
        rospy.loginfo("Action servers connected")

        rospy.loginfo("Reading Phrase File")
        with open(phrase_file, 'r') as f:
            s = f.read()
            self.phrases = yaml.load(s)     
        
        self.feedback.viseme = "IDLE"
        self.feedback.action = "none"

        rospy.loginfo("Starting server...")
        self.server = actionlib.SimpleActionServer('SBPlayback_Server', SpeechPlayAction, execute_cb=self.execute_cb)
        self.server.start()
        rospy.loginfo("Server started")


    def execute_cb(self, goal):
        rospy.loginfo("Phrase playing: " + str(goal.phrase))
        preempted = False

        if goal.interrupt == True:
            rospy.loginfo("Speech: cancelling all goals")
            self.animation_client.cancel_all_goals()
            self.viseme_client.cancel_all_goals()
            self.sound_client.stopAll()
            self.lookat_client.cancel_all_goals()
                        
        timing_adjust = rospy.Duration.from_sec(0.2)

        try:
            actions = self.phrases[goal.phrase]["actions"]
        except KeyError:
            rospy.logerr("Phrase id %s not recognized"%goal.phrase)

        time = rospy.Time.now()
        ordered_actions = sorted(actions, 
                                 key=lambda action: action["start"])
        
        #self.sound_client.stopAll()
        wave_file = self.phrases[goal.phrase]["file"]
        wave_duration = 0.0
        with contextlib.closing(wave.open(wave_file,'r')) as f:
            frames=f.getnframes()
            rate=f.getframerate()
            wave_duration=frames/float(rate)

        rospy.loginfo("Speech: playing wave file -- duration: " + str(wave_duration))
        self.sound_client.playWave(wave_file)
        for a in ordered_actions:
            rospy.loginfo("Playing action: " + str(a))
            if a["type"] == "viseme":
                while rospy.Time.now()-time+timing_adjust < rospy.Duration.from_sec(a["start"]) and not self.server.is_preempt_requested():
                    pass
            if self.server.is_preempt_requested():
                preempted = True
                break
                
            if a["type"] == "viseme" and (rospy.Time.now()-time+timing_adjust > rospy.Duration.from_sec(a["end"])):
                continue

            if a["type"] == "viseme":
                #rospy.loginfo("Viseme: " + a["id"])
                vgoal = dragon_msgs.msg.VisemeGoal(constant=a["id"])
                self.viseme_client.send_goal(vgoal)
                self.feedback.viseme = a["id"]
                self.server.publish_feedback(self.feedback)
            else:
                rospy.logwarn("action not recognized")
          
            '''elif a["type"] == "animation":
                rospy.loginfo("Expression: " + a["id"])
                egoal = cordial_msgs.msg.AnimationGoal(name=a["id"],constant=a["id"])
                self.express_client.send_goal(egoal)
                self.feedback.action = a["id"]
                self.server.publish_feedback(self.feedback)
            elif a["type"] == "lookat":
                rospy.loginfo("Lookat: target: " + a["target"])
                lgoal = dragon_msgs.msg.LookatGoal(target=a["target"], target_duration=rospy.Duration.from_sec(float(a["duration"])))
                self.lookat_client.send_goal(lgoal)           
            elif a["type"] == "track":
                rospy.loginfo("Track: " + a["target"])
                tgoal = dragon_msgs.msg.TrackGoal(on = True, target = a["target"])
                self.track_client.send_goal(tgoal)
            else:
                rospy.logwarn("action not recognized")'''

        if preempted:
            rospy.loginfo("Preempted")
            #self.express_client.cancel_all_goals()
            self.viseme_client.cancel_all_goals()
            self.sound_client.stopAll()
            self.server.set_preempted()
            return
        else:
            rospy.loginfo("Waiting for end")
            while (rospy.Time.now()-time+timing_adjust) < rospy.Duration.from_sec(wave_duration):
                pass
            rospy.loginfo("At end -- Success")
            self.result.result = "SUCCESS"
            self.viseme_client.cancel_all_goals()
            self.server.set_succeeded(self.result)
            return
        rospy.logwarn("Speech: the code should probably never reach this point. Setting succeeded anyway.")
        self.result.result="SUCCESS"
        self.server.set_succeeded(self.result)

if __name__ == '__main__':
    rospy.init_node('dragonbot_speech')
    SpeechPlayServer(sys.argv[1])
    while not rospy.is_shutdown():
        rospy.spin
