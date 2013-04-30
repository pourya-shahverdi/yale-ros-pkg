#!/usr/bin/env python

#Elaine Short
#Expeditions Year 1
#Dialogue File Checker
import sys
import yaml


def main():
    if not len(sys.argv) == 3:
        print "Usage: check_dialogue.py [dialogue file] [phrase_file]"
        sys.exit()

    dialogue = {}
    phrases = {}

    print "Loading dialogue"
    with open(sys.argv[1], 'r') as f:
        s = f.read()
        dialogue = yaml.load(s)

    print "Loading phrases"
    with open(sys.argv[2], 'r') as f:
        s = f.read()
        phrase_file = yaml.load(s)

    valid_phrase_ids = phrase_file.keys()

    print "Checking dialogue file"

    for day,sessions in dialogue.items():
        for session_name,phrases in sessions.items():
            for phrase_name,info in phrases.items():
                error_intro = "Error in day: " + day + " session " + session_name + " phrase " + phrase_name + ": "
                if not "type" in info.keys():
                    print error_intro + "no type given"
                    continue
                if info["type"] == "dialogue":
                    if not "items" in info.keys():
                        print error_intro + "missing dialogue items"
                        continue
                    for phrase in items:
                        if phrase not in phrases.items():
                            print error_intro + "item: " + phrase + " is not a phrase in the session"
                            continue
                elif info["type"] == "statement":
                    if not "phrase_ids" in info.keys():
                        print error_intro + "missing phrase_ids"
                        continue
                    for phrase_id in info["phrase_ids"]:
                        if not phrase_id in valid_phrase_ids:
                            print error_intro + "phrase_id " + phrase_id + " is not valid"
                            continue
                elif info["type"] == "question":
                    if not "phrase_ids" in info.keys():
                        print error_intro + "missing phrase_ids"
                        continue
                    for phrase_id in info["phrase_ids"]:
                        if not phrase_id in valid_phrase_ids:
                            print error_intro + "phrase_id " + phrase_id + " is not valid"
                            continue
                    if not "responses" in info.keys():
                        print error_intro + "question missing responses"
                        continue
                    for reponse in info["responses"]:
                        if not response in phrases.keys():
                            print  error_intro + "item: " + phrase + " is not a phrase in the session"
                            continue
                    if not "terminal" in info.keys():
                        print error_intro + "question missing terminal options"
                        continue
                    for item in info["terminal"]:
                        if not item in info["responses"]:
                            
                elif info["type"] == "backstory":
                    pass
                elif info["type"] == "redirect":
                    pass
                elif info["type"] == "wait":
                    pass
                else:
                    print error_intro + "invalid type"
