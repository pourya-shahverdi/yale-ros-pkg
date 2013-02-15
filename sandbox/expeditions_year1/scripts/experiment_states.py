import roslib; roslib.load_manifest('expeditions_year1')
import rospy
import smach
import smach_ros
import sys
import random

import actionlib

from actionlib import *
from actionlib.msg import *


class Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wakeup','done'])

    def execute(self, userdata):
        print "Robot sleeping ...zzzzzzz..."
        rospy.sleep(2)
        #check for keystroke
        #if keystroke, robot wakeup if time to do exp, otherwise, end
        if True: 
            return 'wakeup'
        else:
            return 'done'

class Intro(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['panic','end'])
        
    def execute(self, userdata):
        #trigger intro dialogue (managed)
        if True: return 'end'
        #if something goes wrong
        if False: return 'panic'

class FoodIntro(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['panic', 'next_round', 'done'])
        self.ntimes = 0
        self.nstates = 2

    def execute(self, userdata):
        self.ntimes = self.ntimes + 1

        print "Food choice round " + str(self.ntimes)
       #figure out which food task we're doing
        #set shared information accordingly
        #if we're done, go straight to the workout task
        if self.ntimes > self.nstates:
            self.ntimes = 0
            return 'done'
        else:
            #trigger intro dialogue
            #then ask for preference
            #store preference data
            if True: return 'next_round'
        #if something goes wrong
        if False: return 'panic'

class FoodChoice(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['panic', 'end'])

    def execute(self, userdata):
        #trigger prompt
        #get food choice
        #store food choice
        #remind on timeout
        #check for correct food for session
        print "Prompting for food choice..."
        rospy.sleep(2.0)
        if True: return 'end'
        if False: return 'panic'

class FoodFeedback(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['panic', 'good', 'bad'])

    def execute(self, userdata):
        #determine whether food is good or bad
        #if good choice
        r = random.randint(1,10)

        if r > 5:
            #give feedback
            print "Good choice!"
            return 'good'
        else:
            #give feedback
            print "Aw, not such a good choice.  Try again!"
            return 'bad'
        #if disaster occurs
        if False: return 'panic'

class Workout(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue', 'end', 
                                             'timeout', 'panic'])
        self.duration = rospy.Duration(10)

    def execute(self, userdata):
        #play workout routine
        start = rospy.Time.now()
        while rospy.Time.now()-start < self.duration:
            print "Work out!"
            rospy.sleep(1)
        #check for timeout
        if True:
            return 'timeout'
        else:
            #prompt for continue
            if True:
                return 'continue'
            else:
                return 'end'

class Outro(smach.State):
     def __init__(self):
        smach.State.__init__(self, outcomes=['panic','end'])
 
     def execute(self, userdata):
         #play outro dialogue
         #check everything's okay
         if True:
             return 'end'
         else:
             return 'panic'
