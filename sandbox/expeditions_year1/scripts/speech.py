#!/usr/bin/env python

# Elaine Short
# Dragonbot oculesic controller

import roslib; roslib.load_manifest('expeditions_year1')
import rospy
import actionlib
from dragon_msgs.msg import *
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import yaml
import sys

class SpeechPlayServer():
    feedback = SpeechPlayFeedback()
    result = SpeechPlayResult()

    def __init__(self, phrase_file):
        self.sound_client = SoundClient()
        rospy.sleep(0.5)
        self.sound_client.stopAll()
     
        #self.speech_client = actionlib.SimpleActionClient('/SpeechPlay_Server',SpeechPlayAction)
        self.express_client = actionlib.SimpleActionClient('/ExpressionMotion_Server',ExpressionMotionAction)
        self.viseme_client = actionlib.SimpleActionClient('/Viseme_Server',VisemeAction)
        self.ik_client = actionlib.SimpleActionClient('/IK_Server',IKAction)
        self.lookat_client = actionlib.SimpleActionClient('/Lookat_Server',LookatAction)
        self.track_client = actionlib.SimpleActionClient('/Track_Server',TrackAction)

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

        with open(phrase_file, 'r') as f:
            s = f.read()
            self.phrases = yaml.load(s)      
        
        self.feedback.viseme = "IDLE"
        self.feedback.action = "none"

        rospy.loginfo("Starting server...")
        self.server = actionlib.SimpleActionServer('SpeechPlay_Server', SpeechPlayAction, execute_cb=self.execute_cb)
        self.server.start()


    def execute_cb(self, goal):
        rospy.loginfo("Phrase playing: " + str(goal.phrase))
        preempted = False

        if goal.interrupt == True:
            self.express_client.cancel_all_goals()
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
                                 key=lambda action: actions[action]["start"])
        
        #self.sound_client.stopAll()
        self.sound_client.playWave(self.phrases[goal.phrase]["file"])
        for name in ordered_actions:
            a = actions[name]
            while rospy.Time.now()-time+timing_adjust < rospy.Duration.from_sec(a["start"]) and not self.server.is_preempt_requested():
                pass
            if self.server.is_preempt_requested():
                preempted = True
                break
                
            if (rospy.Time.now()-time+timing_adjust > 
               rospy.Duration.from_sec(a["end"]) and 
               not a["type"] == "track_off" and
               not a["type"] == "lookat_off"):
                continue

            if a["type"] == "viseme":
                #print "Viseme: " + a["id"]
                vgoal = dragon_msgs.msg.VisemeGoal(constant=a["id"])
                self.viseme_client.send_goal(vgoal)
                self.feedback.viseme = a["id"]
                self.server.publish_feedback(self.feedback)
            elif a["type"] == "expression":
                print "Expression: " + a["id"]
                egoal = dragon_msgs.msg.ExpressionMotionGoal(type='expression',constant=a["id"])
                self.express_client.send_goal(egoal)
                self.feedback.action = a["id"]
                self.server.publish_feedback(self.feedback)
            elif a["type"] == "motion":
                print "Motion: " + a["id"]
                egoal = dragon_msgs.msg.ExpressionMotionGoal(type='motion',constant=a["id"])
                self.express_client.send_goal(egoal)
                self.feedback.action = a["id"]
                self.server.publish_feedback(self.feedback)
            elif a["type"] == "lookat":
                print "Lookat: x: " + str(a["x"]) + " y: " + str(a["y"]) + " z: " + str(a["z"])
                lgoal = dragon_msgs.msg.LookatGoal(x = a["x"], y = a["y"], z = a["z"])
                self.lookat_client.send_goal(lgoal)
            elif a["type"] == "lookat_off":
                lgoal = dragon_msgs.msg.LookatGoal(state = "off")
                self.lookat_client.send_goal(lgoal)
            elif a["type"] == "track":
                print "Track: " + a["target"]
                tgoal = dragon_msgs.msg.TrackGoal(on = True, target = a["target"])
                self.track_client.send_goal(tgoal)
            elif a["type"] == "track_off":
                print "Tracking off"
                tgoal = dragon_msgs.msg.TrackGoal(on = False)
                self.track_client.send_goal(tgoal)
            elif a["type"] == "pose":
                print "Pose " + str(a)
                igoal = dragon_msgs.msg.IKGoal(vel = a["vel"],
                                              acc = a["acc"],
                                              x = a["x"],
                                              y = a["y"],
                                              z = a["z"],
                                              theta = a["theta"],
                                              neck = a["neck"])
                self.ik_client.send_goal(igoal)
            else:
                rospy.logwarn("action not recognized")
          
        if preempted:
            rospy.loginfo("Preempted")
            self.express_client.cancel_all_goals()
            self.viseme_client.cancel_all_goals()
            self.sound_client.stopAll()
            goal = dragon_msgs.msg.LookatGoal(state = "off")
            self.lookat_client.send_goal(goal)
            goal = dragon_msgs.msg.TrackGoal(on = False)
            self.track_client.send_goal(goal)
            self.server.set_preempted()
        else:
            while (rospy.Time.now()-time+timing_adjust < 
               rospy.Duration.from_sec(a["end"])):
                pass
            self.result.result = "SUCCESS"
            self.viseme_client.cancel_all_goals()
            self.express_client.cancel_all_goals()
            goal = dragon_msgs.msg.LookatGoal(state = "off")
            self.lookat_client.send_goal(goal)
            goal = dragon_msgs.msg.TrackGoal(on = False)
            self.track_client.send_goal(goal)
            self.server.set_succeeded(self.result)
        


if __name__ == '__main__':
    rospy.init_node('dragonbot_speech')
    SpeechPlayServer(sys.argv[1])
    while not rospy.is_shutdown():
        rospy.spin
