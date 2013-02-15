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
        
def main():
    rospy.init_node('experiment_controller')

    sm = smach.StateMachine(outcomes=['end'])
 
    with sm:
        smach.StateMachine.add('SLEEP', Sleep(),
                               transitions={'wakeup':'F_INTRO',
                                            'done':'end'})
        smach.StateMachine.add('INTRO', Intro(),
                               transitions={'panic':'SLEEP',
                                            'end':'F_INTRO'})
        smach.StateMachine.add('F_INTRO', FoodIntro(),
                              transitions={'panic':'SLEEP',
                                           'next_round':'F_CHOICE',
                                           'done':'WORKOUT'})
        smach.StateMachine.add('F_CHOICE', FoodChoice(),
                               transitions={'panic':'SLEEP',
                                            'end':'FEEDBACK'})
        smach.StateMachine.add('FEEDBACK', FoodFeedback(),
                               transitions={'panic':'SLEEP',
                                            'good':'F_INTRO',
                                            'bad':'F_CHOICE'})

        smach.StateMachine.add('WORKOUT', Workout(),
                               transitions={'panic':'SLEEP',
                                            'continue':'WORKOUT',
                                            'end':'OUTRO',
                                            'timeout':'OUTRO'})
        smach.StateMachine.add('OUTRO', Outro(),
                               transitions={'end':'SLEEP',
                                            'panic':'SLEEP'})

    outcome = sm.execute()

if __name__ == '__main__':
    main()
