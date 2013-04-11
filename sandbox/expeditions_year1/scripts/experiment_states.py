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

    def play_dialogue(self, item_id, interrupt = True, wait_for_finish = True):
        responses = []

        #print "Item id: " + item_id
        #catch panic/move to next state
        if item_id == 'panic':
            self.dm.stop_speech()
            raise PanicException
        if item_id == 'next_segment':
            self.dm.stop_speech()
            raise NextStateException
        if item_id == 'next_phrase':
            self.dm.stop_speech()
            raise NextPhraseException
        
        dialogue_item = self.dialogue[item_id]
        if dialogue_item["type"] == 'redirect':
            responses.append(item_id)
            responses = responses + self.play_dialogue(dialogue_item["goal"])
        elif dialogue_item["type"] == "wait":
            responses.append(item_id)
            self.dm.say(random.choice(dialogue_item["phrase_ids"]), interrupt)
            self.seen.append(dialogue_item)
            self.tm.change("continue")
            self.tm.wait_for_press(self.gui_prefix + "continue")

        elif dialogue_item["type"] == "dialogue":
            for item in dialogue_item["items"]:
                try:
                    responses = responses + self.play_dialogue(item, interrupt, wait_for_finish)
                except NextPhraseException:
                    continue

        elif dialogue_item["type"] == "backstory":
            try:
                if not dialogue_item in self.seen:
                    self.seen.append(dialogue_item)
                    return self.play_dialogue(dialogue_item["items"][0], interrupt, wait_for_finish)
                else:
                    start = 0
                    if len(dialogue_item["items"]) > 1:
                        start = 1
                        self.seen.append(dialogue_item)
                        return self.play_dialogue(random.choice(dialogue_item["items"][start:len(dialogue_item["items"])]), interrupt, wait_for_finish)
            except NextPhraseException:
                return []

        elif dialogue_item["type"] == "question":
            gui_name = self.day + "_" + self.segment + "_" + item_id
            self.tm.change(gui_name)
            self.dm.say(random.choice(dialogue_item["phrase_ids"]), interrupt = True)
            resp = self.tm.wait_for_press(self.gui_prefix + gui_name)
            try:
                responses = responses + self.play_dialogue(resp, interrupt, wait_for_finish)
            except NextPhraseException:
                    return responses
            while resp not in dialogue_item["terminal"]:
                self.tm.change(gui_name)
                self.seen.append(dialogue_item)
                resp = self.tm.wait_for_press(self.gui_prefix + gui_name)
                try:
                    responses = responses + self.play_dialogue(resp, interrupt, wait_for_finish)
                except NextPhraseException:
                    return responses

        elif dialogue_item["type"] == "choice":
            gui_name = self.day + "_" + self.segment + "_" + item_id
            self.tm.change(gui_name)
            resp = self.tm.wait_for_press(self.gui_prefix + self.gui_name)
            try:
                responses = responses + self.play_dialogue(resp, interrupt, wait_for_finish)
            except NextPhraseException:
                return responses

        elif dialogue_item["type"] == "statement":
            responses.append(item_id)
            if len(dialogue_item["phrase_ids"]) > 0:
                self.dm.say(random.choice(dialogue_item["phrase_ids"]), interrupt, wait = wait_for_finish)
            self.seen.append(dialogue_item)
        print str(responses)
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
        self.dm.eye_close()
        self.tm.change("sleep")
        self.tm.wait_for_press("/dragon_GUI/sleep")   
        self.dm.express("wakeup")
        self.dm.eye_open()
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

        try:
            self.dg.play_dialogue("intro_dialogue")
        except PanicException:
            return 'panic'
        except NextStateException:
            return 'end'
        except NextPhraseException:
            pass
        return 'end'


class FoodChoiceDay1(smach.State):
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
 
        self.dm.express("hungry", wait = False)
        try:
            self.dg.play_dialogue(current_lesson["intro"], wait_for_finish = False)
        except PanicException:
            return 'panic'
        except NextStateException:
            return 'end'
        except NextPhraseException:
            pass
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
                    self.dg.play_dialogue(current_lesson["no_choice"])
                except PanicException:
                    return 'panic'
                except NextStateException:
                    return 'end'
                except NextPhraseException:
                    pass
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
                if not prev==resp:
                    self.dg.play_dialogue("why_choose")
                    self.dm.express("tasting", wait = True)
                if resp in current_lesson["terminal"]:
                    self.dm.express("yummm", wait = False)
                else:
                    self.dm.express("blech", wait = False)
                try:
                    self.dg.play_dialogue(current_lesson[prev][resp])
                except PanicException:
                    return 'panic'
                except NextStateException:
                    return 'end'
                except NextPhraseException:
                    pass
            if resp in current_lesson["terminal"]:
                done = True
            else:
                self.tm.change(lesson_name)
                resp = self.tm.wait_for_press(self.gui_prefix + lesson_name)
        if lesson_name == "drinks":
            self.dm.express("sipping", wait = True)
        else:
            self.dm.express("bite", wait = True)
        print str(self.choices)
        return 'next_round'

class FoodChoiceDay2(smach.State):
    def __init__(self, dm, tm, exp_info, dialogue_info, food_phrases):
        smach.State.__init__(self, outcomes=['panic', 'next_round', 'end'])
        self.dm = dm
        self.tm = tm
        self.ntimes = 0
        self.day, self.lessons = exp_info
        self.fp = food_phrases[self.day]
        self.all_foods = self.fp["bad"] + self.fp["good"] + self.fp["sometimes"]
        self.dialogue = dialogue_info
        self.segment = "foods"
        self.gui_prefix = "dragon_GUI/"
        self.dg = DialogueManager(self.dm, self.tm, self.gui_prefix, self.segment, self.dialogue, self.day)
        self.selected_foods = []
        self.feedback_level = {i:0 for i in self.all_foods}
        

    def execute(self, userdata):
        print "==============================================="
        print "+                FOOD DIALOGUE                +"
        print "-----------------------------------------------"
        gui_name = self.day + "_" + "_".join(self.selected_foods)
        self.tm.change(gui_name)
        panicked = False
        while not rospy.is_shutdown():
            resp = self.tm.wait_for_press("dragon_GUI/food_select")
            if resp == "panic":
                return 'panic'
            elif resp == "GO":
                break
            elif resp == "reminder":
                print "I'm reminding you!"
            elif resp == "next":
                print "Go to the next thing"
            elif resp == "--":
                continue
            elif resp in self.selected_foods:
                self.selected_foods.remove(resp)
            else:
                self.selected_foods.append(resp)
                self.selected_foods.sort()
            gui_name = self.day + "_" + "_".join(self.selected_foods)
            self.tm.change(gui_name)
        # have a bunch of choices in self.selected_foods
        bad_foods = []
        good_foods = []
        sometimes_foods = []
        for food in self.selected_foods:
            if food in self.fp["bad"]:
                bad_foods.append(food)
            elif food in self.fp["good"]:
                good_foods.append(food)
            elif food in self.fp["sometimes"]:
                sometimes_foods.append(food)
        rospy.loginfo("Selected foods:")
        rospy.loginfo("  Good foods: " + ",".join(good_foods))
        rospy.loginfo("  Bad foods: " + ",".join(bad_foods))
        rospy.loginfo("  Sometimes foods: " + ",".join(sometimes_foods))
        return 'next_round'


class Workout(smach.State):
    def __init__(self, dm, tm, info, dialogue_info):
        smach.State.__init__(self, outcomes=['continue', 'end', 
                                             'timeout', 'panic'])
        self.sc = SoundClient()
        self.dm = dm
        self.tm = tm
        self.exp_start_time = rospy.Time.now()
        self.duration = rospy.Duration(450)
        self.break_time = rospy.Duration(120)
        self.dialogue = dialogue_info
        #self.workout_phrases = workout_info
        self.day, self.lesson = info
        self.gui_prefix = "dragon_GUI/"
        self.segment = "workout"

        self.music_folder = '/home/eshort/fuerte_workspace/yale-ros-pkg/sandbox/expeditions_year1/music/'
    
        #pose is: x, y, z, (theta, neck, vel, acc [optional])
        self.poses = {'right': (0, 2.4, 0),
                      'left': (0, -2.4, 0),
                      'up': (0, 0, 2),
                      'down': (0, 0, -2)}
        # song and bpm
        self.songs = {'mario_yoshi.wav':104,
                      'Donkey_Kong_Country_Jungle_Stomp_OC_ReMix.wav':84,
                      'Legend_of_Zelda_A_Link_to_the_Past_Kakariko_Rave_Party_OC_ReMix.wav':160,
                      'Legend_of_Zelda_Ocarina_of_Time_This_Valley_Rocks_OC_ReMix.wav':96,
                      'Super_Mario_World_2_Yoshi\'s_Island_Dino_Band_Rehearsal_OC_ReMix.wav':112,
                      'Super_Mario_World_Swanky_Vegas_OC_ReMix.wav':120}

        self.current_song = ""
        self.dg = DialogueManager(self.dm, self.tm, self.gui_prefix, self.segment, self.dialogue, self.day)
        self.vol = 0.1


    def do_victory(self):
        v = 1
        a = .08
        routine = ['up','down','up','down','up']
        for move in routine:
            m = self.poses[move]
            self.dm.pose(x = m[0], y = m[1], z =m[2], vel = v, acc = a)
            rospy.sleep(1.0)
        

    def execute(self, userdata):
        print "==============================================="
        print "+                WORKOUT GAME                 +"
        print "-----------------------------------------------"
 
        self.dm.express("anticipation", wait = False)
        try:
            #TODO: play correct outro statement
            self.dg.play_dialogue("training_backstory")
            self.dg.play_dialogue("intro_dialogue")
        except PanicException:
            return 'panic'
        except NextStateException:
            return 'end'
        except NextPhraseException:
            pass
        routine = ['left','right','left','right','up','down']

        self.current_song, bpm = random.choice(self.songs.items())
        i = 0
        bpm = bpm/2
        v = 1
        a = 0.05

        time_adjust = rospy.Duration(-0.9)
        delay_adjust = rospy.Duration(0.0)
        move_sleep = rospy.Duration(60/bpm)
        
        self.dm.pose(0,0,0)

        #play workout routine
        start = rospy.Time.now()
        self.sc.playWave(self.music_folder + self.current_song)
        self.sc.waveVol(self.music_folder + self.current_song, self.vol)
        try:
            self.dg.play_dialogue("post_music", interrupt=False)
        except PanicException:
            self.dm.pose_off()
            self.sc.stopAll()
            return 'panic'
        except NextStateException:
            self.dm.pose_off()
            self.sc.stopAll()
            return 'end'
        except NextPhraseException:
            pass
        self.tm.change("stopped_dancing")
        rospy.sleep(delay_adjust)
        nbreaks = 1
        while rospy.Time.now()-start < self.duration and not rospy.is_shutdown():
            if rospy.Time.now()-start > self.break_time * nbreaks:
                '''if nbreaks = 1:
                    break_phrase = "break1"
                elif nbreaks = 2:
                    break_phrase = "break2"
                else:
                    break_phrase = "finished_dancing"'''

                break_phrase = "finished_dancing"
                nbreaks = nbreaks + 1

                self.sc.stopAll()
                try:
                    resp = self.dg.play_dialogue(break_phrase, interrupt = True)
                except PanicException:
                    self.dm.pose_off()
                    self.sc.stopAll()
                    return 'panic'
                except NextStateException:
                    self.dm.pose_off()
                    self.sc.stopAll()
                    return 'end'
                except NextPhraseException:
                    pass

                if not "no_finished" in resp:
                    break

                self.sc.playWave(self.music_folder + self.current_song)
                self.tm.change("stopped_dancing")
                continue
                
            p = self.tm.last_press(self.gui_prefix + "stopped_dancing")
            if p == "next":
                self.dm.pose_off()
                self.sc.stopAll()
                return "end"
            elif p == "panic":
                self.dm.pose_off()
                self.sc.stopAll()
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
                    resp = self.dg.play_dialogue("stop_dancing", interrupt = True)
                except PanicException:
                    self.dm.pose_off()
                    self.sc.stopAll()
                    return 'panic'
                except NextStateException:
                    self.dm.pose_off()
                    self.sc.stopAll()
                    return 'end'
                except NextPhraseException:
                    pass
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
                    self.tm.change("stopped_dancing")
                    continue
                if "no_change" in resp:
                    break_phrase = "finished_dancing"
                    self.sc.stopAll()
                    try:
                        resp = self.dg.play_dialogue(break_phrase, interrupt = True)
                    except PanicException:
                        self.dm.pose_off()
                        self.sc.stopAll()
                        return 'panic'
                    except NextStateException:
                        self.dm.pose_off()
                        self.sc.stopAll()
                        return 'end'
                    except NextPhraseException:
                        break

                    print resp

                    if not "no_finished" in resp:
                        break
                    else:
                        self.sc.playWave(self.music_folder + self.current_song)
                        self.tm.change("stopped_dancing")
                        continue
            
            move = i % len(routine)
            m = self.poses[routine[move]]
            self.dm.pose(x = m[0], y = m[1], z =m[2], vel = v, acc = a)
            i = i + 1
            if i % 10 == 0:
                try:
                    self.dg.play_dialogue("energized_comment", interrupt = False)
                except PanicException:
                    self.dm.pose_off()
                    self.sc.stopAll()
                    return 'panic'
                except NextStateException:
                    self.dm.pose_off()
                    self.sc.stopAll()
                    return 'end'
                except NextPhraseException:
                    pass
            rospy.sleep(move_sleep-time_adjust)
            

        if rospy.Time.now()-start >= self.duration:
            rospy.sleep(2.0)
            try:
                self.dg.play_dialogue("timeout")
            except PanicException:
                self.dm.pose_off()
                self.sc.stopAll()
                return 'panic'
            except NextStateException:
                self.dm.pose_off()
                self.sc.stopAll()
                return 'end'
            except NextPhraseException:
                pass
            self.sc.stopAll()
            self.dm.pose_off()
            return 'timeout'
        else:
            try:
                self.dg.play_dialogue("victory_dance")
            except PanicException:
                self.dm.pose_off()
                self.sc.stopAll()
                return 'panic'
            except NextStateException:
                self.dm.pose_off()
                self.sc.stopAll()
                return 'end'
            except NextPhraseException:
                pass
            self.do_victory() 
            self.dm.pose_off()
            try:
                self.dg.play_dialogue("what_do_you_think")
            except PanicException:
                self.sc.stopAll()
                return 'panic'
            except NextStateException:
                self.sc.stopAll()
                return 'end'
            except NextPhraseException:
                pass
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
         
         self.dm.express("yawn")
         try:
             self.dg.play_dialogue("outro_dialogue")
         except PanicException:
             return 'panic'
         except NextStateException:
             return 'end'
         except NextPhraseException:
             pass
         return 'end'


class PanicException(Exception): pass
class NextStateException(Exception): pass
class NextPhraseException(Exception): pass
