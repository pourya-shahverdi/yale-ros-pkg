#!/usr/bin/env python
import roslib; roslib.load_manifest('expeditions_year1')
import rospy
from dialogue_manager import *
from dragonbot_manager import DragonbotManager
from tablet_manager import TabletManager
import yaml


def main():
  dm = DragonbotManager()
  tm = TabletManager()
  with open("example_dialogue.yaml") as f:
    s = f.read()
  dialogues = yaml.load(s)
  
  dg = DialogueManager(dm, tm, "my_dialogue", dialogues["my_dialogue"])

  try:
    dg.play_dialogue("introduction")
  except PanicException:
    return 'AHHH PANIC! SOMETHING BAD HAPPENED!'
  except NextStateException:
    return 'I want to end the dialogue, but nothing bad happened'
  except NextPhraseException:
    # Ignore this
    pass
  return "Finished dialogue successfully"

if __name__ == '__main__':
  rospy.init_node("example_dialogue")
  print main()


