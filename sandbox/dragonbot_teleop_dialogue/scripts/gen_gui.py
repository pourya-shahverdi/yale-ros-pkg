#!/usr/bin/env python

#Elaine Short
#Expeditions Year 1 Experiment
import yaml
import roslib
import sys
from optparse import OptionParser

def main():

    usage = "usage: %prog [options] <dialogue yaml 1> [<dialogue yaml 2> ...]"
    parser = OptionParser(usage)
    parser.add_option("-o", '--outfile', dest="outfile", help="write gui to FILE (if not set, writes to stdout)", metavar="FILE")
    parser.add_option("-i", '--other-items', dest="other_items_file", help="get additional gui items from FILE", metavar="FILE")

    (options,args) = parser.parse_args()

    if options.outfile == None:
        outfile = sys.stdout
    else:
        outfile = open(options.outfile, 'w')

    dialogue_files = args
    
    standard_items = '''- gui: Dragonbot Teleop
  elements:
    - type: button_group
      label: Blink
      label_string: once;hold;hold_off
      topic: dragon_teleop_GUI/blink
    - type: button_group
      label: Pose
      label_string: on; off
      topic: dragon_teleop_GUI/pose_on
    - type: int_slider
      label: Pose X
      min: 0
      max: 100
      topic: dragon_teleop_GUI/pose_x
    - type: int_slider
      label: Pose Y
      min: 0
      max: 100
      topic: dragon_teleop_GUI/pose_y
    - type: int_slider
      label: Pose Z
      min: 0
      max: 100
      topic: dragon_teleop_GUI/pose_z
    - type: button_group
      label: Look At
      label_string: on; off
      topic: dragon_teleop_GUI/lookat_on
    - type: int_slider
      label: Lookat X
      min: 0
      max: 100
      topic: dragon_teleop_GUI/look_x
    - type: int_slider
      label: Lookat Y
      min: 0
      max: 100
      topic: dragon_teleop_GUI/look_y
    - type: int_slider
      label: Lookat Z
      min: 0
      max: 100
      topic: dragon_teleop_GUI/look_z
    - type: button_group
      label: Expressions
      label_string: angry;frightened;puppy;sipping;confused;lovestruck;sad
      topic: dragon_teleop_GUI/expressions
    - type: button_group
      label: Expressions
      label_string: tasting;disgusted;mischievous;surprised;--;--;--
      topic: dragon_teleop_GUI/expressions
    - type: button_group
      label: Motions
      label_string: afraid;blech;idunno;interest;mmhmmm;question;wakeup
      topic: dragon_teleop_GUI/motions
    - type: button_group
      label: Motions
      label_string: yes;anticipation;farted;i_like_it;laugh1;mph;surprise
      topic: dragon_teleop_GUI/motions
    - type: button_group
      label: Motions
      label_string: yay;yummm;bite;hungry;i_want_it;meh;no
      topic: dragon_teleop_GUI/motions
    - type: button_group
      label: Motions
      label_string: think;yawn;--;--;--;--;--
      topic: dragon_teleop_GUI/motions
    - type: button_group
      label: Phrases
      label_string: weird_food_choice;dragon_race;great;not_name;yay;workout_music;none_soda
      topic: dragon_teleop_GUI/phrases
- gui: continue
  elements:
    - type: button_group
      label: Press next_phrase when child done speaking
      label_string: next_phrase;next_segment;panic
      topic: dragon_GUI/continue





'''

    other_items = ""
    if not options.other_items_file == None:
        with open(options.other_items_file, 'r') as f:
            other_items = f.read()
    
    
    outfile.write(standard_items + "\n")
    outfile.write(other_items + "\n")
    
    gui_prefix = "dragon_GUI/"
    
    dialogue_info = []
    for filename in dialogue_files:
        with open(filename, 'r') as f:
            s = f.read()
            dialogue_items = yaml.load(s)
            dialogue_info.append(dialogue_items)

        for dialogue in dialogue_info:
            for dialogue_name,contents in dialogue.items():
                for prompt,phrase in contents.items():
                    if not (phrase["type"] == "question" or phrase["type"] == "select"):
                        continue
                    name =  dialogue_name + "_" + prompt
                    outfile.write("- gui: " + name + "\n")
                    outfile.write("  elements:"+ "\n")
                    outfile.write("  - type: button_group"+ "\n")
                    outfile.write("    label: " + dialogue_name + " " + prompt+ "\n")
                    outfile.write("    label_string: " + \
                                      (";".join(phrase["responses"]) + ";next_phrase;next_segment;panic")+ "\n")
                    outfile.write("    topic: " + gui_prefix + name+ "\n")
    outfile.close()


if __name__ == '__main__':
    main()
