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

class PanicException(Exception): pass
class NextStateException(Exception): pass
class NextPhraseException(Exception): pass


class DialogueManager():
    def __init__(self, dm, tm, gui_prefix, dialogue_name, dialogue, session_name):
        self.dm = dm
        self.tm = tm
        self.gui_prefix = gui_prefix
        self.segment = dialogue_name
        self.dialogue = dialogue
        self.day = session_name

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
