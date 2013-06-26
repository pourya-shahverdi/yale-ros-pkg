import roslib; roslib.load_manifest('expeditions_year1')
import rospy
import sys
import random

import actionlib

from actionlib import *
from actionlib.msg import *
from tablet_manager import TabletManager
from cordial_sound.msg import SoundRequest
from cordial_sound.libsoundplay import SoundClient

class PanicException(Exception): pass
class NextStateException(Exception): pass
class NextPhraseException(Exception): pass


class DialogueManager():
    def __init__(self, tablet_manager, dialogue_name, dialogue):
        self.speech_client = actionlib.SimpleActionClient('/SBPlayback_Server',SpeechPlayAction)

        self.tm = tablet_manager
        self.gui_prefix = "dragon_GUI/" 
        self.dialogue = dialogue
        self.dialogue_name = dialogue_name

        self.seen = []

    def say(self, phrase_name,interrupt = True, wait = False):
        rospy.loginfo("Saying: " + phrase_name)
        goal = dragon_msgs.msg.SpeechPlayGoal(phrase=phrase_name, interrupt=interrupt)
        self.speech_client.send_goal(goal)
        if wait:
            rospy.loginfo("Waiting for speech server result")
            self.speech_client.wait_for_result(rospy.Duration(60.0))
            if not self.speech_client.get_state() == GoalStatus.SUCCEEDED:
                rospy.logwarn("Dragonbot Manager gave up waiting for speech result. This is likely a problem.")

    def stop_speech(self):
        rospy.loginfo("Zeroing dragonbot")
        self.speech_client.cancel_all_goals()

    def play_dialogue(self, item_id, interrupt = True, wait_for_finish = True):
        responses = []

        #print "Item id: " + item_id
        #catch panic/move to next state
        if item_id == 'panic':
            self.stop_speech()
            raise PanicException
        if item_id == 'next_segment':
            self.stop_speech()
            raise NextStateException
        if item_id == 'next_phrase':
            self.stop_speech()
            raise NextPhraseException
        
        dialogue_item = self.dialogue[item_id]
        if dialogue_item["type"] == 'redirect':
            responses.append(item_id)
            responses = responses + self.play_dialogue(dialogue_item["goal"])
        elif dialogue_item["type"] == "wait":
            responses.append(item_id)
            self.say(random.choice(dialogue_item["phrase_ids"]), interrupt)
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
            gui_name = self.dialogue_name + "_" + item_id
            self.tm.change(gui_name)
            self.say(random.choice(dialogue_item["phrase_ids"]), interrupt = True)
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

        elif dialogue_item["type"] == "select":
            gui_name = self.dialogue_name + "_" + item_id
            self.tm.change(gui_name)
            resp = self.tm.wait_for_press(self.gui_prefix + gui_name)
            try:
                responses = responses + self.play_dialogue(resp, interrupt, wait_for_finish)
            except NextPhraseException:
                return responses

        elif dialogue_item["type"] == "statement":
            responses.append(item_id)
            if len(dialogue_item["phrase_ids"]) > 0:
                self.say(random.choice(dialogue_item["phrase_ids"]), interrupt, wait = wait_for_finish)
            self.seen.append(dialogue_item)
        print str(responses)
        return responses
