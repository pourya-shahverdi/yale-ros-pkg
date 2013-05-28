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
from dialogue_manager import *

class Sleep(smach.State):
    def __init__(self, dm, tm, exp_info):
        smach.State.__init__(self, outcomes=['wakeup','done'])
        self.dm = dm
        self.tm = tm

    def execute(self, userdata):
        print "==============================================="
        print "+                   SLEEPING                  +"
        print "-----------------------------------------------"
        self.dm.stop_speech()
        self.dm.eye_close()
        self.tm.change("sleep")
        self.tm.wait_for_press("/dragon_GUI/sleep")   
        self.dm.express("wakeup")
        rospy.set_param("~start_time", rospy.Time.now().secs)
        self.dm.eye_open()
        return 'wakeup'

class Intro(smach.State):
    def __init__(self, dm, tm, exp_info, dialogue_info):
        smach.State.__init__(self, outcomes=['panic','end'])
        self.dm = dm
        self.tm = tm
        self.day, self.lesson = exp_info
        self.dialogue = dialogue_info
        self.dg = DialogueManager(self.dm, self.tm, self.day + "_intro", self.dialogue)

    def execute(self, userdata):
        print "==============================================="
        print "+              INTRO DIALOGUE                 +"
        print "-----------------------------------------------"

        rospy.set_param("~first_time_foods", True)


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
        smach.State.__init__(self, outcomes=['panic', 'next_round', 'end', 'timeout'])
        self.dm = dm
        self.tm = tm
        self.ntimes = 0
        self.day, self.lessons = exp_info
        self.fp = {i:j for i, j in food_phrases.items() if i in self.lessons}
        self.dialogue = dialogue_info
        self.gui_prefix = "dragon_GUI/"
        self.choices = {}
        self.dg = DialogueManager(self.dm, self.tm, self.day + "_foods", self.dialogue)


    def execute(self, userdata):
        self.ntimes = self.ntimes + 1
        if self.ntimes > len(self.lessons):
            self.ntimes = 0
            return 'end'
        first = True
        last = False
        
        lesson_name = self.lessons[self.ntimes-1] #lessons 0-indexed
        current_lesson = self.fp[lesson_name] 
        foods = current_lesson["none"].keys()
        good_foods = current_lesson["terminal"]

        print "==============================================="
        print "+                FOOD DIALOGUE                +"
        print "-----------------------------------------------"
 
        self.dm.express("puppy", wait = False)
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
                    if resp in current_lesson["terminal"]:
                        last = True
                    if first or last:
                        self.dg.play_dialogue("why_choose")
                        first = False
                    self.dm.express("tasting", wait = True)
                '''if resp in current_lesson["terminal"]:
                    self.dm.express("yummm", wait = False)
                else:
                    self.dm.express("disgusted", wait = False)'''
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
        print str(self.choices)
        return 'next_round'

class FoodChoiceDay2(smach.State):
    def __init__(self, dm, tm, exp_info, dialogue_info, food_phrases):
        smach.State.__init__(self, outcomes=['panic', 'next_round', 'end', 'timeout'])
        self.dm = dm
        self.tm = tm
        self.ntimes = 0
        self.day, self.lessons = exp_info
        self.fp = food_phrases[self.day]
        self.all_foods = self.fp["bad"] + self.fp["good"] + self.fp["sometimes"]
        self.dialogue = dialogue_info
        self.gui_prefix = "dragon_GUI/"
        self.dg = DialogueManager(self.dm, self.tm, self.day + "_foods", self.dialogue)
        self.sc = SoundClient()
        self.music_folder = roslib.packages.get_pkg_dir("expeditions_year1")+ "/music/"

        self.feedback_levels = {i:{"good":0,"bad":0} for i in self.fp["groups"]}
        self.feedback_levels["all"] = {"good":0, "bad":0}
        self.feedback_levels["reminders"] = 0
        self.prev_items = dict(good = [], bad = [], sometimes = [])
        self.target_group = "all"
        self.selected_foods = []
        self.sometimes_feedback = {food:False for food in self.fp["sometimes"]}
        
        self.exp_start_time = rospy.get_param("~start_time")
        self.duration = rospy.get_param("~max_time")
        self.seen_bad = set()

    def execute(self, userdata):
        print "==============================================="
        print "+                FOOD DIALOGUE                +"
        print "-----------------------------------------------"
        
        self.exp_start_time = rospy.get_param("~start_time")
        if rospy.Time.now().secs -self.exp_start_time > self.duration:
            try:
                self.dg.play_dialogue("timeout")
            except PanicException:
                return 'panic'
            except NextStateException:
                return 'end'
            except NextPhraseException:
                pass
            return 'timeout'



        phrases = self.fp["phrases"]
        
        if rospy.get_param("~first_time_foods"):
            if self.day == "meals2_breakfast":
                self.dm.eye_close()
                self.tm.change("sleep")
                self.tm.wait_for_press("/dragon_GUI/sleep")   
                self.dm.express("wakeup")
                self.dm.eye_open()

            try:
                self.dg.play_dialogue(phrases["intro"])
            except PanicException:
                return 'panic'
            except NextStateException:
                rospy.set_param("~first_time_foods",True)
                return 'end'
            except NextPhraseException:
                pass
            # reset everything
            
            self.target_group = "all"
            self.feedback_levels["all"]["bad"] = 0
            for g in self.fp["groups"].keys():
                self.feedback_levels[g]["bad"] = 0
            self.feedback_levels["all"]["good"] = 0
            for g in self.fp["groups"].keys():
                self.feedback_levels[g]["good"] = 0
            self.selected_foods = []


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
                level = self.feedback_levels["reminders"]
                
                try:
                    self.dg.play_dialogue(phrases["reminders"][level])
                except PanicException:
                    return 'panic'
                except NextStateException:
                    rospy.set_param("~first_time_foods",True)
                    return 'end'
                except NextPhraseException:
                    pass

                if level < len(phrases["reminders"]) - 1:
                    self.feedback_levels["reminders"] += 1
            elif resp == "next":
                rospy.set_param("~first_time_foods",True)
                return 'end'
            elif resp == "--":
                continue
            elif resp in self.selected_foods:
                self.selected_foods.remove(resp)
            else:
                self.selected_foods.append(resp)
                self.selected_foods.sort()
            gui_name = self.day + "_" + "_".join(self.selected_foods)
            self.tm.change(gui_name)

        #reset reminder feedback level
        self.feedback_levels["reminders"] = 0
        
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
        
        prev_items = self.prev_items["bad"] + self.prev_items["good"] + self.prev_items["sometimes"]

        added_items = filter(lambda f: f not in prev_items, self.selected_foods)
        removed_items = filter(lambda f: f not in self.selected_foods, prev_items)      

        # escalate level only if failed to improve from last time
        # failure to improve: there were bad items before and we didn't
        # remove one of the ones from the group we were targeting for 
        # improvement or there were missing good foods and we didn't add 
        # one of them from the group we were targeting
        had_bad = len(self.prev_items["bad"]) > 0 
        lacked_good = not set(self.prev_items["good"]) == set(self.fp["good"]) and not rospy.get_param("~first_time_foods")


        rospy.loginfo("Added foods: " + str(added_items) + " Removed foods: " + str(removed_items))
        rospy.loginfo("Target group: " + self.target_group)
        if self.target_group == "all":
            removed_target_bad = len(set(removed_items) & set(self.fp["bad"])) > 0
            added_target_good = len(set(added_items) & set(self.fp["good"])) > 0
        else:
            removed_target_bad = len(set(removed_items) & set(self.fp["bad"]) & set(self.fp["groups"][self.target_group])) > 0
            added_target_good = len(set(added_items) & set(self.fp["good"]) & set(self.fp["groups"][self.target_group])) > 0



        #reset target group and feedback levels if we're switching from 
        #removing bad items to adding good items and vice versa
        if had_bad and len(bad_foods) == 0:
            self.target_group = "all"
            self.feedback_levels["all"]["bad"] = 0
            for g in self.fp["groups"].keys():
                self.feedback_levels[g]["bad"] = 0
        if not had_bad and len(bad_foods) > 0:
            self.target_group = "all"
            # reset good food feedback level
            self.feedback_levels["all"]["good"] = 0
            for g in self.fp["groups"].keys():
                self.feedback_levels[g]["good"] = 0

        print "Seen the following bad foods: " + str(list(self.seen_bad))
        replaced_bad = len(self.seen_bad & set(bad_foods)) > 0

        #escalate if there was a bad item and no bad item was removed,
        #or if the kid added a bad item they've seen before
        if (had_bad and not removed_target_bad) or replaced_bad:
            if self.target_group == "all":
                self.feedback_levels["all"]["bad"] += 1
            else:
                for g in self.fp["groups"].keys():
                    if self.target_group == g:
                        self.feedback_levels[g]["bad"] += 1

        #only adjust good feedback level if no bad foods left
        if lacked_good and not added_target_good and len(bad_foods) == 0:
            if self.target_group == "all":
                self.feedback_levels["all"]["good"] += 1
            else:
                for g in self.fp["groups"].keys():
                    if self.target_group == g:
                        self.feedback_levels[g]["good"] += 1

        self.first_round = False

        try:
            self.dg.play_dialogue(phrases["plate_chant"])
        except PanicException:
            return 'panic'
        except NextStateException:
            rospy.set_param("~first_time_foods",True)
            return 'end'
        except NextPhraseException:
            pass


        rospy.loginfo("Current feedback levels: " + str(self.feedback_levels))

        if not len(bad_foods) == 0:
            rospy.loginfo("BZZZZT")
            self.sc.playWave(self.music_folder + "bzzt.wav")
            rospy.sleep(2.0)

            # if the target group is specific and doesn't have any more bad items, switch targets
            if not self.target_group == "all" and len(set(bad_foods) & set(self.fp["groups"][self.target_group])) == 0:
                rospy.loginfo("Switching target groups...")
                bad_groups = []
                for g in self.fp["groups"].keys():
                    if len(set(bad_foods) & set(self.fp["groups"][g])) > 0:
                        bad_groups.append(g)
                self.target_group = random.choice(bad_groups)
                rospy.loginfo("Switched to group: " + self.target_group)


            #figure out the correct feedback phrase
            feedback_phrase = ""
            if self.target_group == "all" and self.feedback_levels[self.target_group]["bad"] == -1:
                feedback_phrase = ""
            # if we're past the end of our lists
            elif self.feedback_levels[self.target_group]["bad"] > len(self.fp["phrases"]["has_bad"][self.target_group])-1:
                if self.target_group == "all":
                    bad_groups = []
                    for g in self.fp["groups"].keys():
                        if len(set(bad_foods) & set(self.fp["groups"][g])) > 0:
                            bad_groups.append(g)
                    self.target_group = random.choice(bad_groups)
                    feedback_phrase = self.fp["phrases"]["has_bad"][self.target_group][self.feedback_levels[self.target_group]["bad"]]
                else:
                    #give specific feedback
                    food_options = list(set(bad_foods) & set(self.fp["groups"][self.target_group]))
                    food_target = random.choice(food_options)
                    level = min(len(self.fp["phrases"]["specific"][food_target])-1, self.feedback_levels[self.target_group]["bad"]-len(self.fp["phrases"]["has_bad"][self.target_group]))
                    feedback_phrase = self.fp["phrases"]["specific"][food_target][level]
            else:
                feedback_phrase = self.fp["phrases"]["has_bad"][self.target_group][self.feedback_levels[self.target_group]["bad"]]
            
            if len(feedback_phrase) > 0:
                rospy.loginfo("Playing feedback: " + str(feedback_phrase))
                try:
                    self.dg.play_dialogue(feedback_phrase)
                except PanicException:
                    return 'panic'
                except NextStateException:
                    rospy.set_param("~first_time_foods",True)
                    return 'end'
                except NextPhraseException:
                    pass

        elif not set(good_foods) == set(self.fp["good"]):
            rospy.loginfo("ding")
            self.sc.playWave(self.music_folder + "ding.wav")
            rospy.sleep(1.0)

            if not len(filter(lambda f: not self.sometimes_feedback[f], sometimes_foods)) == 0:
                target_food = random.choice(filter(lambda f: not self.sometimes_feedback[f], sometimes_foods))
                feedback_phrase = self.fp["phrases"]["specific"][target_food][0]
                rospy.loginfo("Playing sometimes feedback for food: " + str(target_food))
                try:
                    self.dg.play_dialogue(feedback_phrase)
                    self.sometimes_feedback[target_food] = True
                except PanicException:
                    return 'panic'
                except NextStateException:
                    rospy.set_param("~first_time_foods",True)
                    return 'end'
                except NextPhraseException:
                    pass

            # if there's a specific target group and it's not missing any more good items, switch targets
            if not self.target_group == "all":
                target_good = set(self.fp["groups"][self.target_group]) & set(self.fp["good"])
                # if the target group isn't missing any more good items
                if target_good == (set(good_foods) & set(self.fp["groups"][self.target_group])):
                    rospy.loginfo("Switching target groups...")
                    good_groups = []
                    for g in self.fp["groups"].keys():
                        group_good = set(self.fp["groups"][g]) & set(self.fp["good"])
                        if not group_good == (set(good_foods) & set(self.fp["groups"][g])):
                            good_groups.append(g)
                    self.target_group = random.choice(good_groups)
                    rospy.loginfo("Switched to group: " + self.target_group)

            #figure out the correct feedback phrase
            feedback_phrase = ""
            if self.target_group == "all" and self.feedback_levels[self.target_group]["bad"] == -1:
                feedback_phrase = ""
            # if we're past the end of our lists
            elif self.feedback_levels[self.target_group]["good"] > len(self.fp["phrases"]["missing_good"][self.target_group])-1:
                if self.target_group == "all":
                    good_groups = []
                    for g in self.fp["groups"].keys():
                        group_good = set(self.fp["groups"][g]) & set(self.fp["good"])
                        if not group_good.issubset(set(good_foods)):
                            good_groups.append(g)
                    self.target_group = random.choice(good_groups)
                    feedback_phrase = self.fp["phrases"]["missing_good"][self.target_group][self.feedback_levels[self.target_group]["good"]]
                else:
                    #give specific feedback
                    food_options = list((set(self.fp["groups"][self.target_group]) & set(self.fp["good"]))-set(good_foods))
                    food_target = random.choice(food_options)
                    level = min(len(self.fp["phrases"]["specific"][food_target])-1, self.feedback_levels[self.target_group]["good"]-len(self.fp["phrases"]["missing_good"][self.target_group]))

                    feedback_phrase = self.fp["phrases"]["specific"][food_target][level]
            else:
                feedback_phrase = self.fp["phrases"]["missing_good"][self.target_group][self.feedback_levels[self.target_group]["good"]]
            
            if len(feedback_phrase) > 0:
                rospy.loginfo("Playing feedback: " + str(feedback_phrase))
                try:
                    self.dg.play_dialogue(feedback_phrase)
                except PanicException:
                    return 'panic'
                except NextStateException:
                    rospy.set_param("~first_time_foods",True)
                    return 'end'
                except NextPhraseException:
                    pass
        else:
            rospy.loginfo("FANFARE! YAY!")
            self.sc.playWave(self.music_folder + "fanfare.wav")
            rospy.sleep(2.0)
            try:
                self.dg.play_dialogue(phrases["success"])
            except PanicException:
                return 'panic'
            except NextStateException:
                rospy.set_param("~first_time_foods",True)
                return 'end'
            except NextPhraseException:
                pass
            rospy.set_param("~first_time_foods",True)
            return 'end'

        self.prev_items["good"] = good_foods
        self.prev_items["bad"] = bad_foods
        self.prev_items["sometimes"] = sometimes_foods
        self.seen_bad = self.seen_bad | set(bad_foods)

        
        rospy.set_param("~first_time_foods", False)
        return 'next_round'


class Workout(smach.State):
    def __init__(self, dm, tm, info, dialogue_info):
        smach.State.__init__(self, outcomes=['continue', 'end', 
                                             'timeout', 'panic'])
        self.sc = SoundClient()
        self.dm = dm
        self.tm = tm
        self.exp_start = rospy.get_param("~start_time")
        self.duration = rospy.get_param("~max_time")
        self.break_time = rospy.Duration(60)
        self.encouragement_every = 10
        self.dialogue = dialogue_info
        #self.workout_phrases = workout_info
        self.day, self.lesson = info
        self.gui_prefix = "dragon_GUI/"
        self.seen_victory = False

        self.music_folder = roslib.packages.get_pkg_dir("expeditions_year1")+ "/music/"
    
        #pose is: x, y, z, (theta, neck, vel, acc [optional])
        self.poses = {'right': (0, 2.4, .5),
                      'left': (0, -2.4, .5),
                      'up': (0, 0, 3),
                      'down': (0, 0, 0)}
        # song and bpm
        self.songs = {'mario_yoshi.wav':104,
                      'Donkey_Kong_Country_Jungle_Stomp_OC_ReMix.wav':84,
                      'Legend_of_Zelda_A_Link_to_the_Past_Kakariko_Rave_Party_OC_ReMix.wav':160,
                      'Legend_of_Zelda_Ocarina_of_Time_This_Valley_Rocks_OC_ReMix.wav':96,
                      'Super_Mario_World_2_Yoshi\'s_Island_Dino_Band_Rehearsal_OC_ReMix.wav':112,
                      'Super_Mario_World_Swanky_Vegas_OC_ReMix.wav':120}

        self.current_song = ""
        self.dg = DialogueManager(self.dm, self.tm, self.day + "_workout", self.dialogue)
        self.vol = 0.3


    def do_victory(self):
        print "DOING VICTORY DANCE RAWR"
        v = 1
        a = .08
        routine = ['up','down','up','down','up']

        self.dm.pose_off()
        self.dm.pose(0,0,0,vel=v, acc=a)

        if not self.seen_victory:
            try:
                self.dg.play_dialogue("victory_dance")
            except NextPhraseException:
                pass
            
        for move in routine:
            m = self.poses[move]
            self.dm.pose(x = m[0], y = m[1], z =m[2], vel = v, acc = a)
            rospy.sleep(1.0)
        
        self.dm.pose_off()
        try:
            self.dg.play_dialogue("what_do_you_think")
        except NextPhraseException:
            pass
        self.seen_victory = True

    def bpm_adjust(self, bpm):
        while bpm > 70:
            bpm = bpm/2
        return bpm
        

    def execute(self, userdata):
        self.seen_victory=False
        print "==============================================="
        print "+                WORKOUT GAME                 +"
        print "-----------------------------------------------"
 
        try:
            #TODO: play correct outro statement
            self.dg.play_dialogue("intro_dialogue")
        except PanicException:
            return 'panic'
        except NextStateException:
            return 'end'
        except NextPhraseException:
            pass
        routine = ['up','down']

        self.current_song, bpm = random.choice(self.songs.items())
        i = 0
        v = 1
        a = 0.5
        bpm = self.bpm_adjust(bpm)

        time_adjust = rospy.Duration(0.0)
        delay_adjust = rospy.Duration(0.0)
        move_sleep = rospy.Duration(60/bpm)
        
        self.dm.pose(0,0,0)

        #play workout routine
        #below for timeout from beginning of experiment
        #start = rospy.get_param("~start_time")
        #below for timeout from beginning of workout game
        start = rospy.Time.now().secs
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
        next_break = rospy.Time.now() + self.break_time
        nbreaks = 1
        rospy.loginfo("Next break: " + str(next_break.secs))
        dance_start = rospy.Time.now()
        not_dancing_counter = 0
        print "Time elapsed (game): " + str((rospy.Time.now()-dance_start).secs)
        print "Time elapsed (experiment): " + str(rospy.Time.now().secs-start)

        while rospy.Time.now().secs-start < self.duration and not rospy.is_shutdown():
            print "Time elapsed (game): " + str((rospy.Time.now()-dance_start).secs)
            print "Time elapsed (experiment): " + str(rospy.Time.now().secs-start)
            print "Time until next break: " + str((next_break - rospy.Time.now()).secs)
            if rospy.Time.now() > next_break:
                if nbreaks == 1:
                    break_phrase = "new_song"
                elif nbreaks == 2:
                    break_phrase = "break1"
                elif nbreaks == 3:
                    break_phrase = "break2"
                else:
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


                if "yes_change" in resp:
                    song, bpm = random.choice(self.songs.items())
                    while song == self.current_song:
                        #note that this will break if there's only one song
                        song, bpm = random.choice(self.songs.items())
                    self.current_song = song
                    bpm = self.bpm_adjust(bpm)
                    move_sleep = rospy.Duration(60/bpm)
                    self.sc.playWave(self.music_folder + self.current_song)
                    self.sc.waveVol(self.music_folder + self.current_song, self.vol)
                    next_break = rospy.Time.now() + self.break_time
                    rospy.loginfo("Next break: " + str(next_break.secs))
                    self.tm.change("stopped_dancing")
                    continue

                if "yes_break1" in resp or "yes_break2" in resp and not "hum" in resp:
                    try:
                        self.do_victory()
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

                if break_phrase == "finished_dancing" and not "no_finished" in resp:
                    break

                self.sc.playWave(self.music_folder + self.current_song)
                self.sc.waveVol(self.music_folder + self.current_song, self.vol)
                next_break = rospy.Time.now() + self.break_time
                rospy.loginfo("Next break: " + str(next_break.secs))
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
            elif p == "music_up":
                self.vol = self.vol + 0.1
                if self.vol > 1.0:
                    rospy.loginfo("At max volume")
                    self.vol = 1.0
                self.sc.waveVol(self.music_folder + self.current_song, self.vol)
                self.tm.change("stopped_dancing")
                continue
            elif p == "music_down":
                self.vol = self.vol - 0.1
                if self.vol < 0.0:
                    rospy.loginfo("At min volume")
                    self.vol = 0.0
                self.sc.waveVol(self.music_folder + self.current_song, self.vol)
                self.tm.change("stopped_dancing")
                continue
            elif p == "music_stopped":
                self.sc.stopAll()
                song, bpm = random.choice(self.songs.items())
                while song == self.current_song:
                        #note that this will break if there's only one song
                    song, bpm = random.choice(self.songs.items())
                self.current_song = song
                bpm = self.bpm_adjust(bpm)
                move_sleep = rospy.Duration(60/bpm)
                self.sc.playWave(self.music_folder + self.current_song)
                self.sc.waveVol(self.music_folder + self.current_song, self.vol)
                self.tm.change("stopped_dancing")
                continue
            elif p == "never_dancing":
                not_dancing_counter += 1
                if not_dancing_counter > 4:
                    not_dancing_counter = 4
                try:
                    resp = self.dg.play_dialogue("not_dancing" + str(not_dancing_counter))
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
                
                if "yes_finished" in resp:
                    break
                self.sc.playWave(self.music_folder + self.current_song)
                self.sc.waveVol(self.music_folder + self.current_song, self.vol)
                self.tm.change("stopped_dancing")
                continue
            elif p == "stopped_dancing":
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
                    self.sc.waveVol(self.music_folder + self.current_song, self.vol)
                    self.tm.change("stopped_dancing")
                    continue
                if "yes_change" in resp:
                    song, bpm = random.choice(self.songs.items())
                    while song == self.current_song:
                        #note that this will break if there's only one song
                        song, bpm = random.choice(self.songs.items())
                    self.current_song = song
                    bpm = self.bpm_adjust(bpm)
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
                        self.sc.waveVol(self.music_folder + self.current_song, self.vol)
                        self.tm.change("stopped_dancing")
                        continue
            
            move = i % len(routine)
            m = self.poses[routine[move]]
            self.dm.pose(x = m[0], y = m[1], z =m[2], vel = v, acc = a)
            i = i + 1
            if i % self.encouragement_every == 0:
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
    
        if rospy.Time.now().secs-start >= self.duration:
            print "TIMED OUT"
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
            if not self.seen_victory:
                try:
                    self.do_victory()
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
            self.dm.pose_off()
            self.sc.stopAll()
            return 'timeout'
        else:       
            if not self.seen_victory:
                try:
                    self.do_victory()
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
            self.dm.pose_off()
            self.sc.stopAll()
            return "end"


class Outro(smach.State):
     def __init__(self, dm, tm, exp_info, dialogue_info):
        smach.State.__init__(self, outcomes=['panic','end'])
        self.dm = dm
        self.tm = tm
        self.day, self.lesson = exp_info
        self.dialogue = dialogue_info
        self.gui_prefix = "dragon_GUI/"
        self.dialogue_seen = []
        self.dg = DialogueManager(self.dm, self.tm, self.day + "_outro", self.dialogue)


     def execute(self, userdata):
         print "==============================================="
         print "+                   OUTRO                     +"
         print "-----------------------------------------------"
         #self.dm.express("yawn")
         try:
             self.dg.play_dialogue("outro_dialogue")
         except PanicException:
             return 'panic'
         except NextStateException:
             return 'end'
         except NextPhraseException:
             pass
         return 'end'


