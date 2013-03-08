import roslib; roslib.load_manifest('expeditions_year1')
import rospy
import smach
import smach_ros
import sys
import random

import actionlib

from actionlib import *
from actionlib.msg import *
from dragonbot_manager import DragonbotManager
#from dragonbot_simulator import DragonbotManager
from tablet_manager import TabletManager
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class Sleep(smach.State):
    def __init__(self, dm, tm, exp_info):
        smach.State.__init__(self, outcomes=['wakeup','done'])
        self.dm = dm
        self.tm = tm

    def execute(self, userdata):
        print "==============================================="
        print "+                   SLEEPING                  +"
        print "-----------------------------------------------"
        self.tm.change("sleep")
        self.tm.wait_for_press("/dragon_GUI/sleep")
        return 'wakeup'

class Intro(smach.State):
    def __init__(self, dm, tm, exp_info, dialogue_info):
        smach.State.__init__(self, outcomes=['panic','end'])
        self.dm = dm
        self.tm = tm
        self.day, self.lesson = exp_info
        self.dialogue = dialogue_info
        self.gui_prefix = "dragon_GUI/"
        self.segment = "intro"

    def play_dialogue(self, item_id):
        responses = []

        #print "Item id: " + item_id
        #catch panic/move to next state
        if item_id == 'panic':
            raise PanicException
        if item_id == 'next':
            raise NextStateException

        dialogue_item = self.dialogue[item_id]
        if dialogue_item["type"] == "dialogue":
            for item in dialogue_item["items"]:
                responses = responses + self.play_dialogue(item)
        elif dialogue_item["type"] == "question":
            gui_name = self.day + "_" + self.segment + "_" + item_id
            self.tm.change(gui_name)
            self.dm.say(random.choice(dialogue_item["phrase_ids"]))
            resp = self.tm.wait_for_press(self.gui_prefix + gui_name)
            responses = responses + self.play_dialogue(resp)
            while resp not in dialogue_item["terminal"]:
                self.tm.change(gui_name)
                self.dm.say(random.choice(dialogue_item["phrase_ids"]))
                resp = self.tm.wait_for_press(self.gui_prefix + gui_name)
                responses = responses + self.play_dialogue(resp)
        elif dialogue_item["type"] == "choice":
            gui_name = self.day + "_" + self.segment + "_" + item_id
            self.tm.change(gui_name)
            self.tm.wait_for_press(self.gui_prefix + self.gui_name)
            responses = responses + self.play_dialogue(resp)
        elif dialogue_item["type"] == "statement":
            responses.append(item_id)
            self.tm.change("continue")
            self.dm.say(random.choice(dialogue_item["phrase_ids"]))
            self.tm.wait_for_press(self.gui_prefix + "continue")
        return responses

    def execute(self, userdata):
        print "==============================================="
        print "+              INTRO DIALOGUE                 +"
        print "-----------------------------------------------"

        self.dm.express("wakeup")
        try:
            self.play_dialogue("intro_dialogue")
        except PanicException:
            return 'panic'
        except NextStateException:
            return 'end'
        return 'end'


class FoodChoice(smach.State):
    def __init__(self, dm, tm, exp_info, dialogue_info, food_phrases):
        smach.State.__init__(self, outcomes=['panic', 'next_round', 'end'])
        self.dm = dm
        self.tm = tm
        self.ntimes = 0
        self.day, self.lessons = exp_info
        self.fp = {i:j for i, j in food_phrases.items() if i in self.lessons}
        self.dialogue = dialogue_info
        self.gui_prefix = "dragon_GUI/"
        self.choices = {}
        self.segment = "foods"


    def play_dialogue(self, item_id):
        responses = []

        #print "Item id: " + item_id
        #catch panic/move to next state
        if item_id == 'panic':
            raise PanicException
        if item_id == 'next':
            raise NextStateException

        dialogue_item = self.dialogue[item_id]
        if dialogue_item["type"] == "dialogue":
            for item in dialogue_item["items"]:
                responses = responses + self.play_dialogue(item)
        elif dialogue_item["type"] == "question":
            gui_name = self.day + "_" + self.segment + "_" + item_id
            self.tm.change(gui_name)
            self.dm.say(random.choice(dialogue_item["phrase_ids"]))
            resp = self.tm.wait_for_press(self.gui_prefix + gui_name)
            responses = responses + self.play_dialogue(resp)
            while resp not in dialogue_item["terminal"]:
                self.tm.change(gui_name)
                self.dm.say(random.choice(dialogue_item["phrase_ids"]))
                resp = self.tm.wait_for_press(self.gui_prefix + gui_name)
                responses = responses + self.play_dialogue(resp)
        elif dialogue_item["type"] == "choice":
            gui_name = self.day + "_" + self.segment + "_" + item_id
            self.tm.change(gui_name)
            self.tm.wait_for_press(self.gui_prefix + self.gui_name)
            responses = responses + self.play_dialogue(resp)
        elif dialogue_item["type"] == "statement":
            responses.append(item_id)
            self.tm.change("continue")
            self.dm.say(random.choice(dialogue_item["phrase_ids"]))
            self.tm.wait_for_press(self.gui_prefix + "continue")
        return responses


    def execute(self, userdata):
        self.ntimes = self.ntimes + 1
        if self.ntimes > len(self.lessons):
            self.ntimes = 0
            return 'end'
        
        lesson_name = self.lessons[self.ntimes-1] #lessons 0-indexed
        current_lesson = self.fp[lesson_name] 
        foods = current_lesson["none"].keys()
        good_foods = current_lesson["terminal"]

        print "==============================================="
        print "+                FOOD DIALOGUE                +"
        print "-----------------------------------------------"
 
        self.play_dialogue(current_lesson["intro"])
        self.tm.change(lesson_name)
        resp =  self.tm.wait_for_press(self.gui_prefix + lesson_name)
        done = False
        self.choices[lesson_name]=[]
 
        while done == False:
            if resp == "panic":
                return 'panic'
            elif resp == "next":
                return 'end'
            elif resp == "no_choice":
                self.play_dialogue(current_lesson["no_choice"])
            elif resp == "reminder":
                self.play_dialogue(current_lesson["reminder"])
            else:
                print "-----------------------------------------------"
                prev = ""
                if len(self.choices[lesson_name]) == 0:
                    prev = "none"
                elif resp in self.choices[lesson_name]:
                    prev = resp
                else:
                    prev = self.choices[lesson_name][-1]
                self.choices[lesson_name].append(resp)
                self.play_dialogue(current_lesson[prev][resp])
            if resp in current_lesson["terminal"]:
                done = True
            else:
                self.tm.change(lesson_name)
                resp = self.tm.wait_for_press(self.gui_prefix + lesson_name)
        return 'next_round'

class Workout(smach.State):
    def __init__(self, dm, tm, info, dialogue_info):
        smach.State.__init__(self, outcomes=['continue', 'end', 
                                             'timeout', 'panic'])
        self.sc = SoundClient()
        self.dm = dm
        self.tm = tm
        self.exp_start_time = rospy.Time.now()
        self.duration = rospy.Duration(60)
        self.dialogue = dialogue_info
        #self.workout_phrases = workout_info
        self.day, self.lesson = info
        self.gui_prefix = "dragon_GUI/"
        self.segment = "workout"

        self.music_folder = '/home/eshort/fuerte_workspace/yale-ros-pkg/sandbox/expeditions_year1/music/'
    
        #pose is: vel, acc, x, y, z, theta, neck
        self.poses = {'right': (1, 1, 0, 0, 0, 0, 0),
                      'left': (1, 1, 0, 0, 0, 0, 0),
                      'up': (1, 1, 0, 0, 10, 0, 0),
                      'down': (1, 1, 0, 0, -10, 0, 0)}
        # song and bpm
        self.songs = {'thought_of_you.wav': 108,
                      'try.wav': 83}
        self.current_song = ""

    def play_dialogue(self, item_id, wait_for_continue = True):
        responses = []

        #print "Item id: " + item_id
        #catch panic/move to next state
        if item_id == 'panic':
            raise PanicException
        if item_id == 'next':
            raise NextStateException

        dialogue_item = self.dialogue[item_id]
        if dialogue_item["type"] == "dialogue":
            for item in dialogue_item["items"]:
                responses = responses + self.play_dialogue(item)
        elif dialogue_item["type"] == "question":
            gui_name = self.day + "_" + self.segment + "_" + item_id
            self.tm.change(gui_name)
            self.dm.say(random.choice(dialogue_item["phrase_ids"]))
            resp = self.tm.wait_for_press(self.gui_prefix + gui_name)
            responses = responses + self.play_dialogue(resp)
            while resp not in dialogue_item["terminal"]:
                self.tm.change(gui_name)
                self.dm.say(random.choice(dialogue_item["phrase_ids"]))
                resp = self.tm.wait_for_press(self.gui_prefix + gui_name)
                responses = responses + self.play_dialogue(resp)
        elif dialogue_item["type"] == "choice":
            gui_name = self.day + "_" + self.segment + "_" + item_id
            self.tm.change(gui_name)
            self.tm.wait_for_press(self.gui_prefix + self.gui_name)
            responses = responses + self.play_dialogue(resp)
        elif dialogue_item["type"] == "statement":
            responses.append(item_id)
            self.dm.say(random.choice(dialogue_item["phrase_ids"]))
            if wait_for_continue:
                self.tm.change("continue")
                self.tm.wait_for_press(self.gui_prefix + "continue")
        return responses
  

    def execute(self, userdata):
        print "==============================================="
        print "+                WORKOUT GAME                 +"
        print "-----------------------------------------------"
 
        self.play_dialogue("intro_dialogue")
        routine = ['left','right','left','right','up','down']

        self.current_song, bpm = random.choice(self.songs.items())
        i = 0
        time_adjust = rospy.Duration(-1.0)
        move_sleep = rospy.Duration(60/bpm)

        #play workout routine
        start = rospy.Time.now()
        self.sc.playWave(self.music_folder + self.current_song)
        self.tm.change("stopped_dancing")
        while rospy.Time.now()-start < self.duration:
            p = self.tm.last_press(self.gui_prefix + "stopped_dancing")
            if p == "next":
                return "end"
            elif p == "panic":
                return "panic"
            elif p == "stopped":
                self.sc.stopAll()
                resp = self.play_dialogue("stop_dancing")
                if "dance_more" in resp:
                    self.sc.playWave(self.music_folder + self.current_song)
                    self.tm.change("stopped_dancing")
                    continue
                if "yes_change" in resp:
                    song, bpm = random.choice(self.songs.items())
                    while song == self.current_song:
                        #note that this will break if there's only one song
                        song, bpm = random.choice(self.songs.items())
                    self.current_song = song
                    move_sleep = rospy.Duration(60/bpm)
                    self.sc.playWave(self.music_folder + self.current_song)
                    self.tm.change("stopped_dancing")
                    continue
                if "yes_break" in resp:
                    break
            
            if i % 10 == 0:
                self.play_dialogue("energized_comment", False)
            move = i % len(routine)
            self.dm.pose(*self.poses[routine[move]])
            i = i + 1
            rospy.sleep(move_sleep-time_adjust)
            

        if rospy.Time.now()-start >= self.duration:
            self.play_dialogue("timeout")
            return 'timeout'
        else:
            self.play_dialogue("victory_dance")
            self.dm.express("victory_dance") # do the victory dance - not programmed (yet)
            self.dm.express("what_do_you_think")
            return "end"

class Outro(smach.State):
     def __init__(self, dm, tm, exp_info, dialogue_info):
        smach.State.__init__(self, outcomes=['panic','end'])
        self.dm = dm
        self.tm = tm
        self.day, self.lesson = exp_info
        self.dialogue = dialogue_info
        self.gui_prefix = "dragon_GUI/"
        self.segment = "outro"

     def play_dialogue(self, item_id, wait_for_continue = True):
        responses = []

        #print "Item id: " + item_id
        #catch panic/move to next state
        if item_id == 'panic':
            raise PanicException
        if item_id == 'next':
            raise NextStateException

        dialogue_item = self.dialogue[item_id]
        if dialogue_item["type"] == "dialogue":
            for item in dialogue_item["items"]:
                responses = responses + self.play_dialogue(item)
        elif dialogue_item["type"] == "question":
            gui_name = self.day + "_" + self.segment + "_" + item_id
            self.tm.change(gui_name)
            self.dm.say(random.choice(dialogue_item["phrase_ids"]))
            resp = self.tm.wait_for_press(self.gui_prefix + gui_name)
            responses = responses + self.play_dialogue(resp)
            while resp not in dialogue_item["terminal"]:
                self.tm.change(gui_name)
                self.dm.say(random.choice(dialogue_item["phrase_ids"]))
                resp = self.tm.wait_for_press(self.gui_prefix + gui_name)
                responses = responses + self.play_dialogue(resp)
        elif dialogue_item["type"] == "choice":
            gui_name = self.day + "_" + self.segment + "_" + item_id
            self.tm.change(gui_name)
            self.tm.wait_for_press(self.gui_prefix + self.gui_name)
            responses = responses + self.play_dialogue(resp)
        elif dialogue_item["type"] == "statement":
            responses.append(item_id)
            self.dm.say(random.choice(dialogue_item["phrase_ids"]))
            if wait_for_continue:
                self.tm.change("continue")
                self.tm.wait_for_press(self.gui_prefix + "continue")
        return responses

     def execute(self, userdata):
         print "==============================================="
         print "+                   OUTRO                     +"
         print "-----------------------------------------------"

         self.play_dialogue("outro_dialogue")

         return 'end'

class PanicException(Exception): pass
class NextStateException(Exception): pass
