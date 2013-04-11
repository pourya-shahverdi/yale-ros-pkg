#!/usr/bin/env python

#Elaine Short
#Expeditions Year 1 Experiment
import yaml
from itertools import combinations

def main():
    
    print '''- gui: Dragonbot Teleop
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
- gui: sleep
  elements:
    - type: button
      label: wakeup
      topic: dragon_GUI/sleep
- gui: continue
  elements:
    - type: button_group
      label: Press next_phrase when child done speaking
      label_string: next_phrase;next_segment;panic
      topic: dragon_GUI/continue
- gui: stopped_dancing
  elements:
    - type: button_group
      label: Child stopped dancing?
      label_string: dancing;not_dancing;stopped;music_stopped;next;panic
      topic: dragon_GUI/stopped_dancing






'''



    gui_prefix = "dragon_GUI/"

    with open("food_phrases.yaml", 'r') as f:
        s = f.read()
        
        # file format is:
        # lesson:
        #  intro: phrase_id
        #  reminder: phrase_id
        #  no_choice: phrase_id
        #  foodname: (food chosen before)
        #    foodname: phrase_id (food chosen after)
        # index as food_info["lesson"]["choice1"]["choice2"]
        # nb: food_info["lesson"]["none"].keys() will give all foods (and "none")
    food_info = yaml.load(s)
    with open("dialogue_phrases.yaml", 'r') as f:
        s = f.read()

        # file format is:
        # section_name:
        #   prompt_phrase_id:
        #    order: <1, 2, 3, etc.>
        #    responses: [resp1_id, resp2_id, <...>]
        # nb: if phrase with no response, just leave responses blank
    dialogue_info = yaml.load(s)

    with open("day2_food_phrases.yaml", 'r') as f:
        s = f.read()

    food_info2 = yaml.load(s)

    for day in dialogue_info:
        for item in dialogue_info[day]:
            for prompt in dialogue_info[day][item]:
                phrase = dialogue_info[day][item][prompt]
                if not (phrase["type"] == "question" or phrase["type"] == "select"):
                    continue
                name = day + "_" + item + "_" + prompt
                print "- gui: " + name
                print "  elements:"
                print "  - type: button_group"
                print "    label: " + item + " " + prompt
                print "    label_string: " + \
                    (";".join(phrase["responses"]) + ";next_phrase;next_segment;panic") #.strip(";")
                print "    topic: " + gui_prefix + name

    for lesson in food_info:
        foods = food_info[lesson]["none"].keys() 
        print "- gui: " + lesson
        print "  elements:"
        print "  - type: button_group"
        print "    label: " + lesson
        print ("    label_string: " + ";".join(foods) + ";no_choice;next;panic") #.strip(;)
        print "    topic: " + gui_prefix + lesson

    for day in food_info2:
        all_foods = food_info2[day]["bad"]+food_info2[day]["good"]+ food_info2[day]["sometimes"]
        all_foods.sort()

        print "- gui: " + day + "_"
        print "  elements:"
        print "  - type: button_group"
        print "    label: \"Selected foods: Press to deselect\""
        print "    label_string: \'--\'"
        print "    topic: " + gui_prefix + "food_select"
        print "  - type: button_group"
        print "    label: \"Unselected Foods: Press to select\""
        print "    label_string: " + ";".join(all_foods)
        print "    topic: " + gui_prefix + "food_deselect"
        print "    topic: " + gui_prefix + "food_select"
        print "  - type: button_group"
        print "    label: \"GO (press to activate \'magic plate\')\""
        print "    label_string: GO;reminder;next;panic"
        print "    topic: " + gui_prefix +  "food_select"


        print "- gui: " + day + "_" + "_".join(all_foods)
        print "  elements:"
        print "  - type: button_group"
        print "    label: \"Selected foods: Press to deselect\""
        print "    label_string: " + ";".join(all_foods)
        print "    topic: " + gui_prefix + "food_select"
        print "  - type: button_group"
        print "    label: \"Unselected Foods: Press to select\""
        print "    label_string: \'--\'"
        print "    topic: " + gui_prefix + "food_deselect"
        print "    topic: " + gui_prefix + "food_select"
        print "  - type: button_group"
        print "    label: \"GO (press to activate \'magic plate\')\""
        print "    label_string: GO;reminder;next;panic"
        print "    topic: " + gui_prefix +  "food_select"

        for i in range(1, len(all_foods)):
            foods = combinations(all_foods, i)
            for food_combo in foods:
                food_combo = list(food_combo)
                food_combo.sort()
                print "- gui: " + day + "_" + "_".join(food_combo)
                print "  elements:"
                print "  - type: button_group"
                print "    label: \"Selected foods: Press to deselect\""
                print "    label_string: " + ";".join(food_combo)
                print "    topic: " + gui_prefix + "food_select"
                print "  - type: button_group"
                print "    label: \"Unselected Foods: Press to select\""
                print "    label_string: " + ";".join(list(set(all_foods) - set(food_combo)))
                print "    topic: " + gui_prefix + "food_select"
                print "  - type: button_group"
                print "    label: \"GO (press to activate \'magic plate\')\""
                print "    label_string: GO;reminder;next;panic"
                print "    topic: " + gui_prefix +  "food_select"
if __name__ == '__main__':
    main()
