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

from dragonbot_manager import DragonbotManager
#from dragonbot_simulator import DragonbotManager
from tablet_manager import TabletManager
import yaml

def main():
    if not len(sys.argv) == 2:
        print "Usage: run_exp.py [day_id]"
        sys.exit()

    rospy.init_node('experiment_controller')
    sm = smach.StateMachine(outcomes=['end'])

    lesson_list = {'lunchbox':("whole_grains", "drinks"),
                   'snacks':("snacks1", "snacks2"),
                   'breakfast':("cereal", "breakfast"),
                   'dinner':("sides1","sides2")}



    dm = DragonbotManager()
    tm = TabletManager()

    #info: day, lessons    
    day = sys.argv[1].strip()
    lessons = lesson_list[day]
    info = (day,lessons)
    

    with open("food_phrases.yaml", 'r') as f:
        s = f.read()

    # file format is:
    # lesson:
    #  intro: phrase_id
    #  reminder: phrase_idfg
    #  no_choice: phrase_id
    #  foodname: (food chosen before)
    #    foodname: phrase_id (food chosen after)
    # index as food_info["lesson"]["choice1"]["choice2"]
    # nb: food_info["lesson"]["none"].keys() will give all foods (and "none")
    food_info = yaml.load(s)


    with open("dialogue_phrases.yaml", 'r') as f:
        s = f.read()

    # file format is:

    dialogue_info = yaml.load(s)[day]
    dm.load_phrases("phrases.yaml")
 
    with sm:
        smach.StateMachine.add('SLEEP', Sleep(dm, tm, info),
                               transitions={'wakeup':'INTRO',
                                            'done':'end'})
        smach.StateMachine.add('INTRO', Intro(dm, tm, info, dialogue_info["intro"]),
                               transitions={'panic':'SLEEP',
                                            'end':'F_CHOICE'})
        '''smach.StateMachine.add('F_INTRO', FoodIntro(dm, tm, info),
                              transitions={'panic':'SLEEP',
                                           'next_round':'F_CHOICE',
                                           'done':'WORKOUT'})'''
        smach.StateMachine.add('F_CHOICE', FoodChoice(dm, tm, info, dialogue_info["foods"],food_info),
                               transitions={'panic':'SLEEP',
                                            'next_round':'F_CHOICE',
                                            'end':'WORKOUT'})
        smach.StateMachine.add('WORKOUT', Workout(dm, tm, info, dialogue_info["workout"]),
                               transitions={'panic':'SLEEP',
                                            'continue':'WORKOUT',
                                            'end':'OUTRO',
                                            'timeout':'OUTRO'})
        smach.StateMachine.add('OUTRO', Outro(dm, tm, info, dialogue_info["outro"]),
                               transitions={'end':'SLEEP',
                                            'panic':'SLEEP'})

    outcome = sm.execute()

if __name__ == '__main__':
    main()
