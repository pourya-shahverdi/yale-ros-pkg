#!/user/bin/python
import roslib; roslib.load_manifest('expeditions_year1')
import rospy
from dialogue_manager import *
from dragonbot_manager import DragonbotManager
from tablet_manager import TabletManager


def main():
  dm = DragonbotManager()
  tm = TabletManager()
  with open("dialogue_spec.yaml") as f:
    s = f.read()
  dialogues = yaml.load(s)["experiment_dialogues.yaml"]
  
  dg = DialogueManager(dm, tm, "introduction", dialogues["introduction"])

  try:
    self.dg.play_dialogue("my_dialogue")
  except PanicException:
    return 'AHHH PANIC! SOMETHING BAD HAPPENED!'
  except NextStateException:
    return 'I want to end the dialogue, but nothing bad happened'
  except NextPhraseException:
    # Ignore this
    pass
  return "Finished dialogue successfully"

if __name__ == '__main__':
    print main()

