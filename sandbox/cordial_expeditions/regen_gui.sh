#!/bin/bash
rosrun expeditions_year1 gen_food_guis.py > ./scripts/gui.txt
rosrun dragonbot_teleop_dialogue gen_gui.py -i $PWD/scripts/gui.txt -o $PWD/../../dynamic_interface/interface_srv/guis/dragon_GUI.yaml $PWD/yaml/dialogue_specification.yaml