#!/usr/bin/env python

#Elaine Short
#Dragonbot python interface

import roslib; roslib.load_manifest('expeditions_year1')
import rospy
from threading import Thread
from threading import Lock
import yaml
from dragon_msgs.msg import *
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import actionlib

class Tracker():
    def __init__(self):
        self.on = False
        self.t = Thread(target=self.tracking_thread)

    def tracking_thread(self, target):
        while self.on:
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
        self.on = False
        if self.t.is_alive():
            self.t.join()

class DragonbotManager():

    def __init__(self):
        #TODO: setup all action clients
        self.tracker = Tracker()
        self.tracking_frame = ""
        self.phrases = {}
        self.express_client = actionlib.SimpleActionClient('/ExpressionMotion_Server',ExpressionMotionAction)
        self.viseme_client = actionlib.SimpleActionClient('/Viseme_Server',VisemeAction)
        '''self.pose_client = actionlib.SimpleActionClient('/',PoseAction)
        self.lookat_client = actionlib.SimpleActionClient('/',LookatAction)'''
        self.sound_client = SoundClient()
        rospy.sleep(0.5)
        self.sound_client.stopAll()
     
        """rospy.loginfo("Waiting for Dragonbot Action Servers")
        rospy.loginfo(" --- Expression Motion")
        self.express_client.wait_for_server()
        rospy.loginfo(" --- Visemes")
        self.viseme_client.wait_for_server()
        rospy.loginfo(" --- Pose")
        self.pose_client.wait_for_server()
        rospy.loginfo(" --- Lookat")
        self.lookat_client.wait_for_server()
        rospy.loginfo("Action servers connected")"""



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


    def viseme_play(self,visemes):
        timing_adjust = rospy.Duration.from_sec(0.2)
        time = rospy.Time.now()
        ordered_visemes = sorted(visemes, 
                                 key=lambda viseme: visemes[viseme]["start"])
        for name in ordered_visemes:
            v = visemes[name]
            while (rospy.Time.now()-time+timing_adjust < rospy.Duration.from_sec(v["start"])):
                True
            goal = dragon_msgs.msg.VisemeGoal(constant=v['type'])
            self.viseme_client.send_goal(goal)
            #print v['type']
        rospy.sleep(0.5)
        goal = dragon_msgs.msg.VisemeGoal(constant='IDLE')
        self.viseme_client.cancel_all_goals()
            

    def action_play(self,actions):
        timing_adjust = rospy.Duration.from_sec(0.0)
        time = rospy.Time.now()
        ordered_actions = sorted(actions, 
                                 key=lambda action: actions[action]["start"])
        for name in ordered_actions:
            a = actions[name]
            while (rospy.Time.now()-time+timing_adjust < rospy.Duration.from_sec(a["start"])):
                True
            print a['type']



    def say(self, phrase_name):
        #TODO: look up phrase in loaded phrase library
        #TODO: say phrase with appropriate visemes and actions
        #      note: perhaps use threads to time visemes and actions?
        phrase = self.phrases[phrase_name]
        th1 = Thread(target=self.viseme_play, args=(phrase["visemes"],))
        th2 = Thread(target=self.action_play, args=(phrase["actions"],))
        self.sound_client.stopAll()
        self.sound_client.playWave(phrase["file"])
        th1.start()
        th2.start()
        print phrase['text'] # TODO: say text
        rospy.sleep(1.0)
        th1.join()
        th2.join()



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
        for phrase in self.phrases:
            print "===" + phrase + "==="
            print self.phrases[phrase]


    def express(self, expression_id):
        goal = dragon_msgs.msg.ExpressionMotionGoal(type='expression',constant=expression_id)
        self.express_client.send_goal(goal)
        
    def pose(self, x, y, z, theta):
        #TODO: move dragonbot to a pose
        True

    def lookat(self, x, y, z):
        #TODO: dragonbot looks at location
        True

def main():
    rospy.init_node("dragonbot_manager_test")
    dm = DragonbotManager()
    dm.load_phrases("phrases.yaml")
    dm.express("happy")
    #dm.say("teaching")
    
    '''
    client = actionlib.SimpleActionClient('Viseme_Server', dragon_msgs.msg.VisemeAction)

    print 'waiting for server... \n' 

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    print 'sending goal... \n'

    # Creates a goal to send to the action server.
    goal = dragon_msgs.msg.VisemeGoal(constant='AA_AH')



    # Sends the goal to the action server.
    client.send_goal(goal)

    rospy.sleep(5.)
    print "Sending new goal... \n"
    goal = dragon_msgs.msg.VisemeGoal(constant='AO_AW')
    client.send_goal(goal)
    rospy.sleep(5.)
    client.cancel_all_goals()'''
    
if __name__ == '__main__':
    main()
