#!/usr/bin/env python

#Elaine Short
#Expeditions Year 1 Experiment
import yaml


def main():
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
                    (";".join(phrase["responses"]) + ";next;panic") #.strip(";")
                print "    topic: " + gui_prefix + name

    for lesson in food_info:
        foods = food_info[lesson]["none"].keys() 
        print "- gui: " + lesson
        print "  elements:"
        print "  - type: button_group"
        print "    label: " + lesson
        print ("    label_string: " + ";".join(foods) + ";no_choice;next;panic") #.strip(;)
        print "    topic: " + gui_prefix + lesson


if __name__ == '__main__':
    main()
