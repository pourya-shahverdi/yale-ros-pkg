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

#TODO: Use stoppable threads here
class Tracker():
    def __init__(self):
        self.stop = Event()
        self.t = Thread(target=self.tracking_thread)
        
    def tracking_thread(self, target):
        while not self.stop.isSet():
            print "tracking frame: " + target
            #TODO: listener = tf.TransformListener()
            #try:
            #    (trans, rot) = listener.lookupTransform('/dragonbot',target, rospy.Time(0))
            #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #    continue
            #dm.lookat(trans[0],trans[1],trans[2]) #assuming trans is x,y,z
            rospy.sleep(1.0)

    def start(self, target):
        if len(target) == 0:
            rospy.logwarn("No target set. Tracking not started.")
        else:
            self.on = True
            self.t = Thread(target=self.tracking_thread, args=(target,))
            self.t.start()

    def stop(self):
        self.stop.set()
        if self.t.is_alive():
            self.t.join()


class StoppableThread(Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self):
        super(StoppableThread, self).__init__()
        self._stop = Event()
 
    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

class VisemeThread(StoppableThread):
    def __init__(self, visemes, client):
        super(VisemeThread, self).__init__()
        self.viseme_client = client
        self.visemes = visemes

    def run(self):
        timing_adjust = rospy.Duration.from_sec(0.2)
        time = rospy.Time.now()
        ordered_visemes = sorted(self.visemes, 
                                 key=lambda viseme: self.visemes[viseme]["start"])
        
        
        for name in ordered_visemes:
            v = self.visemes[name]
            while (rospy.Time.now()-time+timing_adjust < 
                   rospy.Duration.from_sec(v["start"])):
                pass
            if(rospy.Time.now()-time+timing_adjust > 
               rospy.Duration.from_sec(v["end"])):
                continue
            if not self.stopped() and not rospy.is_shutdown():
                print "Viseme: " + v['type']
                goal = dragon_msgs.msg.VisemeGoal(constant=v['type'])
                try:
                    self.viseme_client.send_goal(goal)
                except rospy.exceptions.ROSException:
                    print "ROSException in Viseme Thread"
                    next
            else:
                break
        #rospy.sleep(0.5)
        #goal = dragon_msgs.msg.VisemeGoal(constant='IDLE')
        #self.viseme_client.send_goal(goal)
        #self.viseme_client.cancel_all_goals()
        
class ActionThread(StoppableThread):
    def __init__(self,actions,express_fun):
        super(ActionThread, self).__init__()
        self.actions = actions
        self.express = express_fun

    def run(self):
        timing_adjust = rospy.Duration.from_sec(0.2)
        time = rospy.Time.now()
        ordered_actions = sorted(self.actions, 
                                 key=lambda action: self.actions[action]["start"])
        for name in ordered_actions:

            a = self.actions[name]
            while (rospy.Time.now()-time+timing_adjust 
                   < rospy.Duration.from_sec(a["start"])):
                True

            if(rospy.Time.now()-time+timing_adjust > 
               rospy.Duration.from_sec(a["end"])):
                continue
            if not self.stopped():
                try:
                    self.express(a["type"])
                except rospy.exceptions.ROSException:
                    sys.exit()
            else:
                break


class DragonbotManager():

    def __init__(self):
        #TODO: setup all action clients
        self.tracker = Tracker()
        self.tracking_frame = ""
        self.phrases = {}

        self.speech_client = actionlib.SimpleActionClient('/SpeechPlay_Server',SpeechPlayAction)
        self.express_client = actionlib.SimpleActionClient('/ExpressionMotion_Server',ExpressionMotionAction)
        self.viseme_client = actionlib.SimpleActionClient('/Viseme_Server',VisemeAction)
        '''self.pose_client = actionlib.SimpleActionClient('/',PoseAction)
        self.lookat_client = actionlib.SimpleActionClient('/',LookatAction)'''
        self.sound_client = SoundClient()
        rospy.sleep(0.5)
        self.sound_client.stopAll()
     
        rospy.loginfo("Waiting for Dragonbot Action Servers")
        rospy.loginfo(" --- Expression Motion")
        self.express_client.wait_for_server()
        rospy.loginfo(" --- Visemes")
        self.viseme_client.wait_for_server()
        """rospy.loginfo(" --- Pose")
        self.pose_client.wait_for_server()
        rospy.loginfo(" --- Lookat")
        self.lookat_client.wait_for_server()
        rospy.loginfo("Action servers connected")"""


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
        self.tracking_frame = frame_name
        self.tracker.start(frame_name)
        
    def track_off(self):
        self.tracker.stop

    def track_restart(self):
        if len(self.tracking_frame)>0:
            self.tracker.start(self.tracking_frame)
        else:
            rospy.logwarn("Could not restart tracking; no frame was being tracked")



    def say(self, phrase_name):
         goal = dragon_msgs.msg.SpeechPlayGoal(phrase=phrase_name)
         self.speech_client.send_goal(goal)


        #TODO: look up phrase in loaded phrase library
        #TODO: say phrase with appropriate visemes and actions
        #      note: perhaps use threads to time visemes and actions?
        #phrase = self.phrases[phrase_name]
        #self.th1 = VisemeThread(phrase["visemes"], self.viseme_client)
        #self.th2 = ActionThread(phrase["actions"], self.express)
        #self.has_threads = True
        #self.sound_client.stopAll()
        #self.sound_client.playWave(phrase["file"])
        #try:
        #self.th1.start()
        #self.th2.start()
  
        #print phrase['text'] 
            #while (th1.is_alive() or th2.is_alive) and not rospy.is_shutdown():
            #    th1.join(0.5)
            #    th2.join(0.5)
            #if rospy.is_shutdown():
            #    th1.stop()
            #    th2.stop()
            #th1.join()
            #th2.join()
       # except Exception as e:
            #th1.stop()
            #th2.stop()
            #th1.join()
            #th2.join()
            #rospy.sleep(1.0)
            #raise

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
        
    def express(self, expression_id):
        if expression_id in self.expressions:
            goal = dragon_msgs.msg.ExpressionMotionGoal(type='expression',constant=expression_id)
        else: 
            if expression_id in self.motions:
                goal = dragon_msgs.msg.ExpressionMotionGoal(type='motion',constant=expression_id)
            else:
                rospy.logwarn("expression not recognized")
                return
        self.express_client.send_goal(goal)
        
        
    def pose(self, x, y, z, theta):
        #TODO: move dragonbot to a pose
        True

    def lookat(self, x, y, z):
        #TODO: dragonbot looks at location
        True

    def shutdown(self):
        if self.has_threads():
            if self.th1.is_alive():
                self.th1.stop()
            if self.th2.is_alive():
                self.th2.stop()
            self.th1.join()
            self.th2.join()

        goal = dragon_msgs.msg.VisemeGoal(constant='IDLE')
        self.viseme_client.send_goal(goal)
        self.viseme_client.cancel_all_goals()
        self.sound_client.stopAll()


def kill_speech():
    print "Shutting down dragonbot nicely."
    express_client = actionlib.SimpleActionClient('/ExpressionMotion_Server',ExpressionMotionAction)
    viseme_client = actionlib.SimpleActionClient('/Viseme_Server',VisemeAction)
    '''self.pose_client = actionlib.SimpleActionClient('/',PoseAction)
    self.lookat_client = actionlib.SimpleActionClient('/',LookatAction)'''
    sound_client = SoundClient()

    express_client.wait_for_server()
    viseme_client.wait_for_server()

    #goal = dragon_msgs.msg.VisemeGoal(constant='IDLE')
    #viseme_client.send_goal(goal)
    viseme_client.cancel_all_goals()
    express_client.cancel_all_goals()
    sound_client.stopAll()

def main():
    rospy.init_node("dragonbot_manager_test")

    dm = DragonbotManager()

    rospy.on_shutdown(kill_speech)
    dm.load_phrases("phrases.yaml")
    dm.say("teaching")
    rospy.sleep(10)

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


    '''for expression in expressions:
        dm.express(expression)
        rospy.sleep(3.0)

    for motion in motions:
        dm.express(motion)
        rospy.sleep(5.0)'''

if __name__ == '__main__':
    main()
