#!/usr/bin/env python

# Elaine Short
# Dragonbot oculesic controller

import roslib; roslib.load_manifest('dragonbot_python')
import rospy
import actionlib
from dragon_msgs.msg import *
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
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
     
        self.express_client = actionlib.SimpleActionClient('/ExpressionMotion_Server',ExpressionMotionAction)
        self.viseme_client = actionlib.SimpleActionClient('/Viseme_Server',VisemeAction)
        self.ik_client = actionlib.SimpleActionClient('/IK_Server',IKAction)
        self.lookat_client = actionlib.SimpleActionClient('/Lookat_Server',LookatAction)
        self.track_client = actionlib.SimpleActionClient('/Track_Server',TrackAction)
        self.blink_client = actionlib.SimpleActionClient('/Blink_Server',BlinkAction)

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
        rospy.loginfo(" --- Blinking")
        self.blink_client.wait_for_server()
        rospy.loginfo("Action servers connected")

        with open(phrase_file, 'r') as f:
            s = f.read()
            self.phrases = yaml.load(s)      
        
        self.feedback.viseme = "IDLE"
        self.feedback.action = "none"

        rospy.loginfo("Starting server...")
        self.server = actionlib.SimpleActionServer('SBPlayback_Server', SpeechPlayAction, execute_cb=self.execute_cb)
        self.server.start()


    def execute_cb(self, goal):
        rospy.loginfo("Phrase playing: " + str(goal.phrase))
        preempted = False

        if goal.interrupt == True:
            rospy.loginfo("Speech: cancelling all goals")
            #self.express_client.cancel_all_goals()
            self.viseme_client.cancel_all_goals()
            self.sound_client.stopAll()
            lgoal = dragon_msgs.msg.LookatGoal(state = "off")
            self.lookat_client.send_goal(lgoal)
            tgoal = dragon_msgs.msg.TrackGoal(on = False)
            self.track_client.send_goal(tgoal)
            
        timing_adjust = rospy.Duration.from_sec(0.2)

        try:
            actions = self.phrases[goal.phrase]["actions"]
        except KeyError:
            rospy.logerr("Key %s not recognized"%goal.phrase)

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
            elif a["type"] == "expression":
                rospy.loginfo("Expression: " + a["id"])
                egoal = dragon_msgs.msg.ExpressionMotionGoal(type='expression',constant=a["id"])
                self.express_client.send_goal(egoal)
                self.feedback.action = a["id"]
                self.server.publish_feedback(self.feedback)
            elif a["type"] == "motion":
                rospy.loginfo("Motion: " + str(a["id"]))
                egoal = dragon_msgs.msg.ExpressionMotionGoal(type='motion',constant=a["id"])
                self.express_client.send_goal(egoal)
                self.feedback.action = a["id"]
                self.server.publish_feedback(self.feedback)
            elif a["type"] == "lookat":
                rospy.loginfo("Lookat: x: " + str(a["x"]) + " y: " + str(a["y"]) + " z: " + str(a["z"]))
                lgoal = dragon_msgs.msg.LookatGoal(x = a["x"], y = a["y"], z = a["z"])
                self.lookat_client.send_goal(lgoal)
            elif a["type"] == "lookat_frame":
                rospy.loginfo("Lookat: target: " + a["target"])
                try:
                    (trans, rot) = self.tf_listener.lookupTransform('/dragonbot',frame, rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn("Error looking up transform.")
                else:
                    lgoal = dragon_msgs.msg.LookatGoal(x = trans[0]*100, y = trans[1]*100, z = trans[2]*100) #m to cm
                    self.lookat_client.send_goal(lgoal)                
            elif a["type"] == "lookat_off":
                rospy.loginfo("Lookat off")
                lgoal = dragon_msgs.msg.LookatGoal(state = "off")
                self.lookat_client.send_goal(lgoal)
            elif a["type"] == "track":
                rospy.loginfo("Track: " + a["target"])
                tgoal = dragon_msgs.msg.TrackGoal(on = True, target = a["target"])
                self.track_client.send_goal(tgoal)
            elif a["type"] == "track_off":
                rospy.loginfo("Tracking off")
                tgoal = dragon_msgs.msg.TrackGoal(on = False)
                self.track_client.send_goal(tgoal)
            elif a["type"] == "eye_open":
                rospy.loginfo("Eyes open")
                bgoal = dragon_msgs.msg.BlinkGoal(constant = "STOP")
                self.blink_client.send_goal(bgoal)
            elif a["type"] == "eye_close":
                rospy.loginfo("Eyes closed")
                bgoal = dragon_msgs.msg.BlinkGoal(constant = "START")
                self.blink_client.send_goal(bgoal)
            elif a["type"] == "blink":
                rospy.loginfo("Blink")
                bgoal = dragon_msgs.msg.BlinkGoal(constant = "ONCE")
                self.blink_client.send_goal(bgoal)
            elif a["type"] == "pose":
                rospy.loginfo("Pose " + str(a))
                igoal = dragon_msgs.msg.IKGoal(vel = a["vel"],
                                               acc = a["acc"],
                                               x = a["x"],
                                               y = a["y"],
                                               z = a["z"],
                                               theta = a["theta"],
                                               neck = a["neck"],
                                               state = "on")
                self.ik_client.send_goal(igoal)
            elif a["type"] == "pose_off":
                rospy.loginfo("Pose off")
                igoal = dragon_msgs.msg.IKGoal(state = "off")
                self.ik_client.send_goal(igoal)
            else:
                rospy.logwarn("action not recognized")
          
        if preempted:
            rospy.loginfo("Preempted")
            #self.express_client.cancel_all_goals()
            self.viseme_client.cancel_all_goals()
            self.sound_client.stopAll()
            goal = dragon_msgs.msg.LookatGoal(state = "off")
            self.lookat_client.send_goal(goal)
            goal = dragon_msgs.msg.TrackGoal(on = False)
            self.track_client.send_goal(goal)
            self.server.set_preempted()
            return
        else:
            rospy.loginfo("Waiting for end")
            while (rospy.Time.now()-time+timing_adjust) < rospy.Duration.from_sec(wave_duration):
                pass
            rospy.loginfo("At end -- Success")
            self.result.result = "SUCCESS"
            self.viseme_client.cancel_all_goals()
            #self.express_client.cancel_all_goals()
            goal = dragon_msgs.msg.LookatGoal(state = "off")
            self.lookat_client.send_goal(goal)
            #bgoal = dragon_msgs.msg.BlinkGoal(constant = "STOP")
            #self.blink_client.send_goal(bgoal)
            goal = dragon_msgs.msg.TrackGoal(on = False)
            self.track_client.send_goal(goal)
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
