# Introduction #

Before you can run the experiment scripts, you will need to install the repository and all of the dragonbot code: http://code.google.com/p/yale-ros-pkg/wiki/DragonbotPubSubInstall.

# Additional Setup #

  1. Regenerate the phrase information file (this only needs to be done after a new install or after each time the files in the phrases/data directory change).  If the speech node crashes with a message that it can't find "phrases.yaml" you need to re-run this file.
```
> roscd expeditions_year1
> ./regen_phrases.sh
```
  1. Use ifconfig to find the IP address of your computer; you'll need to set ROS\_IP and ROS\_HOSTNAME for the background nodes and the main experiment node (see below).
```
> ifconfig
```
  1. If you've changed the dialogue or generated the new gui file in the example for developing new experiments, you'll need to regenerate the GUI:
```
> roscd expeditions_year1
> ./regen_gui.sh
```

# Running the Background Nodes #
  1. In a new terminal window, start the dragonbot relay:
```
> roslaunch dragonbot_relay dragonbot_relay.launch
```
  1. In another terminal window, start the dragonbot pubsub:
```
> roscd dragonbot_pubsub
> build/install/dragonbot_pubsub/bin/dragonbot_pubsub edu.yale.dragonbot.DragonbotNode
```
  1. In a third terminal window, start the background nodes:
```
> export ROS_IP=[your IP address here]
> export ROS_HOSTNAME=[your IP address here]
> roslaunch expeditions_year1 background_nodes.launch
```
  1. Start the tablet and connect to your computer.  It should display a long list of GUIs.

# Running the Introduction #
The introduction is a short (~1-2 minute) speech by the robot, starting from being asleep and ending with being asleep. The tablet is used to wake the robot up: the GUI will change to a single button that says "wakeup".  Press it and wait a few seconds and the robot will wakeup and start talking.  To run the introduction:

  1. Run all of the background nodes and setup as above.
  1. Set the IP and run the introduction:
```
> export ROS_IP=[your IP address here]
> export ROS_HOSTNAME=[your IP address here]
> rosrun expeditions_year1 intro.py
```

If all is well, the robot will start snoring and the tablet will display the "wakeup" button.

# Running the Experiment #
The full experiment runs for 10 minutes by default, but this can be adjusted by changing the `_`max`_`time parameter.  In each dialogue act/question, there are several options the experimenter can choose, to reflect the child's speech.  There is also a 'panic' button, which will put the robot back into the sleep state, a "next\_phrase" option, which will skip to the next phrase`*` in the dialogue. and a "next\_segment" button, which will skip to the next segment (the segments are: sleeping, the introduction (and relationship-building), the food game, the workout game, and the outro).

During the food game for day 1, the tablet can be used to select which food the child has chosen.  For day 2 interactions, the tablet can be used to select and deselect the foods that are on the plate, and "GO" is used to trigger the magic plate and the robot's feedback.

As of rev. 251 the following lessons and days are working:
  * unchbox 1
  * unchbox 2

  1. Run all of the background nodes and setup as above.
  1. Set the IP and run the experiment:
```
> export ROS_IP=[your IP address here]
> export ROS_HOSTNAME=[your IP address here]
> rosrun expeditions_year1 run_exp.py _day:=[1 or 2] _lesson:=[lunchbox, snacks, or meals] [_max_time:=[time in seconds]]
```

`*` Note that a phrase might be more than one sentence long; it loosely would represent one question or one block of text (which was recorded as one file).  See the page on [developing new experiments](http://code.google.com/p/yale-ros-pkg/wiki/NewExperiments) for details.