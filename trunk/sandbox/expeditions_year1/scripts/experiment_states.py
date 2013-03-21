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

class DialogueManager():
    def __init__(self, dm, tm, gui_prefix, segment, dialogue, day):
        self.dm = dm
        self.tm = tm
        self.gui_prefix = gui_prefix
        self.segment = segment
        self.dialogue = dialogue
        self.day = day

        self.seen = []

    def play_dialogue(self, item_id, wait_for_continue = True, interrupt = True):
        responses = []

        #print "Item id: " + item_id
        #catch panic/move to next state
        if item_id == 'panic':
            self.dm.stop_speech()
            raise PanicException
        if item_id == 'next':
            self.dm.stop_speech()
            raise NextStateException

        dialogue_item = self.dialogue[item_id]
        if dialogue_item["type"] == "dialogue":
            for item in dialogue_item["items"]:
                responses = responses + self.play_dialogue(item, wait_for_continue, interrupt)
        elif dialogue_item["type"] == "question":
            gui_name = self.day + "_" + self.segment + "_" + item_id
            self.tm.change(gui_name)
            if not dialogue_item in self.seen:
                self.dm.say(dialogue_item["phrase_ids"][0], interrupt)
            else:
                start = 0
                if len(dialogue_item["phrase_ids"]) > 1:
                    start = 1
                self.dm.say(random.choice(dialogue_item["phrase_ids"][start:len(dialogue_item["phrase_ids"])]), interrupt)
            self.seen.append(dialogue_item)
            resp = self.tm.wait_for_press(self.gui_prefix + gui_name)
            responses = responses + self.play_dialogue(resp, wait_for_continue, interrupt)
            while resp not in dialogue_item["terminal"]:
                self.tm.change(gui_name)
                if not dialogue_item in self.seen:
                    self.dm.say(dialogue_item["phrase_ids"][0], interrupt)
                else:
                    start = 0
                    if len(dialogue_item["phrase_ids"]) > 1:
                        start = 1
                    self.dm.say(random.choice(dialogue_item["phrase_ids"][start:len(dialogue_item["phrase_ids"])]), interrupt)
                self.seen.append(dialogue_item)
                resp = self.tm.wait_for_press(self.gui_prefix + gui_name)
                responses = responses + self.play_dialogue(resp, wait_for_continue, interrupt)
        elif dialogue_item["type"] == "choice":
            gui_name = self.day + "_" + self.segment + "_" + item_id
            self.tm.change(gui_name)
            self.tm.wait_for_press(self.gui_prefix + self.gui_name)
            responses = responses + self.play_dialogue(resp, wait_for_continue, interrupt)
        elif dialogue_item["type"] == "statement":
            responses.append(item_id)
            if not dialogue_item in self.seen:
                self.dm.say(dialogue_item["phrase_ids"][0], interrupt, wait = (not wait_for_continue))
            else:
                start = 0
                if len(dialogue_item["phrase_ids"]) > 1:
                    start = 1
                self.dm.say(random.choice(dialogue_item["phrase_ids"][start:len(dialogue_item["phrase_ids"])]), interrupt, wait = (not wait_for_continue))
            self.seen.append(dialogue_item)
            if wait_for_continue:
                self.tm.change("continue")
                self.tm.wait_for_press(self.gui_prefix + "continue")
        
        
        return responses

                
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
        self.dg = DialogueManager(self.dm, self.tm, self.gui_prefix, self.segment, self.dialogue, self.day)

    def execute(self, userdata):
        print "==============================================="
        print "+              INTRO DIALOGUE                 +"
        print "-----------------------------------------------"

        self.dm.express("wakeup")
        rospy.sleep(3.0)
        try:
            self.dg.play_dialogue("intro_dialogue", wait_for_continue = False)
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
        self.dg = DialogueManager(self.dm, self.tm, self.gui_prefix, self.segment, self.dialogue, self.day)

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
 
        try:
            self.dg.play_dialogue(current_lesson["intro"], wait_for_continue = False)
        except PanicException:
            return 'panic'
        except NextStateException:
            return 'end'
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
                try:
                    self.dg.play_dialogue(current_lesson["no_choice"], wait_for_continue = False)
                except PanicException:
                    return 'panic'
                except NextStateException:
                    return 'end'
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
                try:
                    self.dg.play_dialogue(current_lesson[prev][resp], wait_for_continue = False)
                except PanicException:
                    return 'panic'
                except NextStateException:
                    return 'end'
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
        self.duration = rospy.Duration(300)
        self.dialogue = dialogue_info
        #self.workout_phrases = workout_info
        self.day, self.lesson = info
        self.gui_prefix = "dragon_GUI/"
        self.segment = "workout"

        self.music_folder = '/home/eshort/fuerte_workspace/yale-ros-pkg/sandbox/expeditions_year1/music/'
    
        #pose is: x, y, z, (theta, neck, vel, acc [optional])
        self.poses = {'right': (0, 1.5, 0),
                      'left': (0, -1.5, 0),
                      'up': (0, 0, 1),
                      'down': (0, 0, -1)}
        # song and bpm
        self.songs = {#'thought_of_you.wav': 108,
                      'try.wav': 83,
                      'mario_yoshi.wav':104}
        self.current_song = ""
        self.dg = DialogueManager(self.dm, self.tm, self.gui_prefix, self.segment, self.dialogue, self.day)
        self.vol = 0.1


    def do_victory(self):
        v = 1
        a = .05
        routine = ['up','down','up','down','up']
        for move in routine:
            m = self.poses[move]
            self.dm.pose(x = m[0], y = m[1], z =m[2], vel = v, acc = a)
            rospy.sleep(1.0)
        

    def execute(self, userdata):
        print "==============================================="
        print "+                WORKOUT GAME                 +"
        print "-----------------------------------------------"
 
        try:
            self.dg.play_dialogue("intro_dialogue", wait_for_continue = False)
        except PanicException:
            return 'panic'
        except NextStateException:
            return 'end'
        routine = ['left','right','left','right','up','down']

        self.current_song, bpm = random.choice(self.songs.items())
        i = 0
        bpm = bpm/2
        v = 1
        a = 0.05

        time_adjust = rospy.Duration(0.0)
        delay_adjust = rospy.Duration(5.0)
        move_sleep = rospy.Duration(60/bpm)
        
        self.dm.pose(0,0,0)

        #play workout routine
        start = rospy.Time.now()
        self.sc.playWave(self.music_folder + self.current_song)
        self.sc.waveVol(self.music_folder + self.current_song, self.vol)
        try:
            self.dg.play_dialogue("post_music", wait_for_continue = False, interrupt=False)
        except PanicException:
            self.sc.stopAll()
            return 'panic'
        except NextStateException:
            self.sc.stopAll()
            return 'end'
        self.tm.change("stopped_dancing")
        rospy.sleep(delay_adjust)
        while rospy.Time.now()-start < self.duration and not rospy.is_shutdown():
            p = self.tm.last_press(self.gui_prefix + "stopped_dancing")
            if p == "next":
                return "end"
            elif p == "panic":
                return "panic"
            elif p == "dancing":
                self.vol = self.vol + 0.1
                if self.vol > 1.0:
                    rospy.loginfo("At max volume")
                    self.vol = 1.0
                self.sc.waveVol(self.music_folder + self.current_song, self.vol)
                self.tm.change("stopped_dancing")
                continue
            elif p == "not_dancing":
                self.vol = self.vol - 0.1
                if self.vol < 0.0:
                    rospy.loginfo("At min volume")
                    self.vol = 0.0
                self.sc.waveVol(self.music_folder + self.current_song, self.vol)
                self.tm.change("stopped_dancing")
                continue
            elif p == "music_stopped":
                song, bpm = random.choice(self.songs.items())
                while song == self.current_song:
                        #note that this will break if there's only one song
                    song, bpm = random.choice(self.songs.items())
                self.current_song = song
                bpm = bpm/4
                move_sleep = rospy.Duration(60/bpm)
                self.sc.playWave(self.music_folder + self.current_song)
                self.sc.waveVol(self.music_folder + self.current_song, self.vol)
                self.tm.change("stopped_dancing")
                continue
            elif p == "stopped":
                self.sc.stopAll()
                try:
                    resp = self.dg.play_dialogue("stop_dancing", interrupt = True, wait_for_continue = False)
                except PanicException:
                    self.sc.stopAll()
                    return 'panic'
                except NextStateException:
                    self.sc.stopAll()
                    return 'end'
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
                    bpm = bpm/4
                    move_sleep = rospy.Duration(60/bpm)
                    self.sc.playWave(self.music_folder + self.current_song)
                    self.sc.waveVol(self.music_folder + self.current_song, self.vol)
                    try:
                        self.dg.play_dialogue("here_we_go", interrupt = True, wait_for_continue = False)
                    except PanicException:
                        self.sc.stopAll()
                        return 'panic'
                    except NextStateException:
                        self.sc.stopAll()
                        return 'end'
                    self.tm.change("stopped_dancing")
                    continue
                if "yes_break" in resp:
                    break
            
            
            move = i % len(routine)
            m = self.poses[routine[move]]
            self.dm.pose(x = m[0], y = m[1], z =m[2], vel = v, acc = a)
            i = i + 1
            if i % 10 == 0:
                self.dg.play_dialogue("energized_comment", interrupt = False, wait_for_continue = False)
            rospy.sleep(move_sleep-time_adjust)
            

        if rospy.Time.now()-start >= self.duration:
            rospy.sleep(2.0)
            try:
                self.dg.play_dialogue("timeout")
            except PanicException:
                self.sc.stopAll()
                return 'panic'
            except NextStateException:
                self.sc.stopAll()
                return 'end'
            self.sc.stopAll()
            self.dm.pose_off()
            return 'timeout'
        else:
            try:
                self.dg.play_dialogue("victory_dance", wait_for_continue = False)
            except PanicException:
                self.sc.stopAll()
                return 'panic'
            except NextStateException:
                self.sc.stopAll()
                return 'end'
            self.do_victory() 
            self.dm.pose_off()
            try:
                self.dg.play_dialogue("what_do_you_think", wait_for_continue = False)
            except PanicException:
                self.sc.stopAll()
                return 'panic'
            except NextStateException:
                self.sc.stopAll()
                return 'end'
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
        self.dialogue_seen = []
        self.dg = DialogueManager(self.dm, self.tm, self.gui_prefix, self.segment, self.dialogue, self.day)


     def execute(self, userdata):
         print "==============================================="
         print "+                   OUTRO                     +"
         print "-----------------------------------------------"
         
         try:
             self.dg.play_dialogue("outro_dialogue", wait_for_continue = False)
         except PanicException:
             return 'panic'
         except NextStateException:
             return 'end'
         return 'end'


class PanicException(Exception): pass
class NextStateException(Exception): pass
