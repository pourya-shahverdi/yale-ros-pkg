#!/usr/bin/env python

#Elaine Short
#Expeditions Year 1 Experiment
import yaml
import roslib
from itertools import combinations

def main():
    gui_prefix = "dragon_GUI/"

    with open(roslib.packages.get_pkg_dir("expeditions_year1")+"/yaml/day1_food_phrases.yaml", 'r') as f:
        s = f.read()
    food_info = yaml.load(s)


    with open(roslib.packages.get_pkg_dir("expeditions_year1")+"/yaml/day2_food_phrases.yaml", 'r') as f:
        s = f.read()

    food_info2 = yaml.load(s)

    print '''- gui: sleep
  elements:
  - type: button
    label: wakeup
    topic: dragon_GUI/sleep


- gui: stopped_dancing
  elements:
  - type: button_group
    label: Child stopped dancing?
    label_string: music_up;music_down;never_dancing;stopped_dancing;music_stopped;next;panic
    topic: dragon_GUI/stopped_dancing

'''


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
        all_foods = sorted(all_foods, key=str.lower)

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
                food_combo = sorted(food_combo, key=str.lower)
                print "- gui: " + day + "_" + "_".join(food_combo)
                print "  elements:"
                print "  - type: button_group"
                print "    label: \"Selected foods: Press to deselect\""
                print "    label_string: " + ";".join(food_combo)
                print "    topic: " + gui_prefix + "food_select"
                print "  - type: button_group"
                print "    label: \"Unselected Foods: Press to select\""
                print "    label_string: " + ";".join(sorted(list(set(all_foods) - set(food_combo)), key=str.lower))
                print "    topic: " + gui_prefix + "food_select"
                print "  - type: button_group"
                print "    label: \"GO (press to activate \'magic plate\')\""
                print "    label_string: GO;reminder;next;panic"
                print "    topic: " + gui_prefix +  "food_select"
if __name__ == '__main__':
    main()
