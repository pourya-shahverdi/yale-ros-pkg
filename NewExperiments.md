# Introduction #

Existing support code makes it easy to develop new experiments with teleoperated dialogue.  These instructions explain how to develop a new teleoperated interaction.  Before you can do this, you will need to:
  1. Install the repository and all of the dragonbot code: http://code.google.com/p/yale-ros-pkg/wiki/DragonbotPubSubInstall.
  1. Acquire a copy of the [Annosoft Lipsync Tool](http://www.annosoft.com/)

All the examples in the following text can be found in the [dragonbot\_teleop\_dialogue](http://code.google.com/p/yale-ros-pkg/source/browse/trunk/sandbox/dragonbot_teleop_dialogue/) package, in the folder "example".

# Developing your Dialogue #

To run a teleoperated dialogue with the dragonbot, you will need audio files, optional text files, annosoft files, optional action files, and a dialogue specification.

## Background Information ##

### Dialogue Structure ###

Each dialogue is split up into a number of _dialogue items_, in a recursive structure.  A dialogue item might link to several pre-recorded speech acts, referred to here as _phrases_. Each dialogue item can be one of several types:
  * **Statement**: A statement is the simplest kind of dialogue item.  It involves any amount of speech, after which the system will move to the next item.  Statements are the leaf nodes in the dialogue tree.  A single statement can link to several phrases, and will choose randomly from among them (e.g. "great!", "good job", and "awesome" might be phrases all associated with a single postive\_reinforcement statement).
  * **Dialogue**: A dialogue is any number of dialogue items, to be played in order.
  * **Question**: A question is any amount of speech, after which there will be several options for user response, each of which triggers a response from the robot.  Some of the responses are terminal responses, after which the system moves on to the next phrase, while any non-terminal responses will not change the teleop interface, allowing another response to be selected (note: the question is not repeated).  A response can be any other dialogue item, including another question.  As with phrases, a question can be associated with several phrases, which are chosen from randomly.
  * **Backstory**: A backstory is a collection of dialogue items.  The first time the backstory is encountered by the system, the first dialogue item is played.  Every subsequent time the backstory is encountered, a dialogue item is chosen from among the 2nd-nth items in the list, where n is the length of the list.
  * **Redirect**: A redirect is a dialogue item type used for internal convenience.  It simply plays the target dialogue item.
  * **Wait**: This is similar to a statement, except that a teleop interface is displayed for controlling when to move on to the next dialogue item.
  * **Select**: This dialogue item allows the operator to choose from among several responses (also dialogue items), similarly to a question but without any prompt being played by the robot.

### Dialogue Specification File Format ###
The dialogue specification file uses the [yaml](http://pyyaml.org/) file format.  A single file may contain several dialogues, which group the individual dialogue items.  Dialogue items may not link to items outside of their dialogue.  Names of dialogue items must be unique within each dialogue.  Some of the information within each dialogue item is specific to certain types.  The format is as follows:

```
<dialogue name>:
  <dialogue item name>:
    type: <dialogue, question, statement, wait, redirect, select, or backstory>
    [items: <for dialogue and backstory items, the list of dialogue items to choose from>]
    [phrase_ids: <for all items except dialogue and backstory, the ID of the phrases to choose from to play for the item>]
    [responses: <for question and select items, the dialogue items the operator should choose from>]
    [terminal: <for question items, the options that will cause the system to move on>]
    [goal: <for redirect items, the name of the target dialogue item>]
```

In summary, the required information for each type is as follows:
  * question
    * type
    * phrase`_`ids
    * responses
    * terminal
  * statement & wait
    * type
    * phrase`_`ids
  * redirect
    * type
    * goal
  * select
    * type
    * responses
  * backstory & dialogue
    * type
    * items

The following ([example\_dialogue.yaml](http://code.google.com/p/yale-ros-pkg/source/browse/trunk/sandbox/dragonbot_teleop_dialogue/example/example_dialogue.yaml)) is an example of a dialogue (in which the robot greets the user, then has them guess the robot's favorite color, then asks them to bring it a crayon in that color).

```
my_dialogue:
  introduction:
    type: dialogue
    items: [hello, give_name, how_are_you,how_are_you_resp, guess_fav_color, bring_crayon, bye]
  hello:
    type: statement
    phrase_ids: [hey, hi, hello]
  give_name:
    type: statement
    phrase_ids: [my_name_is]
  how_are_you:
    type: statement
    phrase_ids: [how_are_you]
  how_are_you_resp:
    type: select
    responses: [good_answer, bad_answer]
  good_answer:
    type: statement
    phrase_ids: [great]
  bad_answer:
    type: statement
    phrase_ids: [sorry]
  guess_fav_color:
    type: question
    phrase_ids: [can_you_guess]
    responses: [red, green, blue, grey, brown, cerulean, other]
    terminal: [blue, cerulean]
  yuck_ew:
    type: statement
    phrase_ids: [ew_gross_color]
  brown:
    type: redirect
    goal: yuck_ew
  grey:
    type: redirect
    goal: yuck_ew
  cerulean:
    type: dialogue
    items: [same_as_blue, blue]
  same_as_blue:
    type: statement
    phrase_ids: [same_as_blue]
  blue:
    type: statement
    phrase_ids: [yeah_blue]
  red:
    type: backstory
    items: [red_ugly, remember_ugly, remember_hate]
  red_ugly:
    type: statement
    phrase_ids: [red_ugly]
  remember_ugly:
    type: statement
    phrase_ids: [remember_thats_ugly]
  remember_hate:
    type: statement
    phrase_ids: [remember_i_hate]
  green:
    type: statement
    phrase_ids: [not_green]
  other:
    type: statement
    phrase_ids: [thats_not_it]
  bring_crayon:
    type: wait
    phrase_ids: [bring_me_blue_crayon]
  bye:
    type: statement
    phrase_ids: [thanks_bye]
```

Note that phrase\_ids are not the names of dialogue items (although they may be the same, but rather link into the phrase file.  Instructions on generating the phrase file appear in the next section, "Preparing your Dialogue".

### Action File Format ###

The action file allows you to specify actions that go along with a pre-recorded phrase. Every action requires a start time (in seconds from the beginning of the phrase).  Some require additional information (see example below).  The actions available are:
  * expression: play one of the pre-loaded face-only animations
  * motion: play one of the pre-loaded full-body animations
  * lookat: look at a specified x,y,z point (will hold until lookat\_off or the end of a phrase)
  * lookat\_off: return to idle lookat behavior
  * lookat\_frame: look at a specified tf frame (will hold until lookat\_off or the end of a phrase)
  * eye\_close: close the eyes
  * eye\_open: open the eyes
  * track: track a target tf frame (will hold until track\_off or the end of a phrase)
  * track\_off: return to idle lookat behavior
  * pose: set the robot's body to a specified position (x,y,z,theta,neck) (will hold until pose\_off or the end of a phrase)
  * pose\_off: return the robot's body to idle behavior

One quirk of the yaml file format is that while you usually don't need to put strings in quotes, typing 'yes' and 'no' without quotes will parse as _True_ and _False_, respectively.  **_You will need to put the 'yes' and 'no' motion labels in quotes_**


The motions (full-body animations) available for the dragonbot are:
  * afraid
  * blech
  * idunno
  * interest
  * mmhmmm
  * question
  * wakeup
  * "yes"
  * anticipation
  * farted
  * i\_like\_it
  * ilikeit
  * laugh1
  * mph
  * surprise
  * yay
  * yummm
  * bite
  * hungry
  * i\_want\_it
  * iwantit
  * meh
  * "no"
  * think
  * yawn


The expressions (face-only animations) available are:
  * angry
  * frightened
  * puppy
  * sipping
  * confused
  * lovestruck
  * sad
  * tasting
  * disgusted
  * mischievous
  * surprised


```
- type: expression
  id: puppy
  start: 1.0
- type: motion
  id: farted
  start: 4.0
- type: track
  target: target_tf
  start: 3.0
- type: track_off
  start: 4.0
- type: lookat
  x: 300
  y: 0
  z: 0
  start: 5.0
- type: lookat_off
  start: 6.0
  end: 7.0
- type: pose
  start: 7.2
  vel: 0
  acc: 0
  x: 0
  y: 0
  z: 0
  theta: 0
  neck: 0
- type: pose_off
  start: 9.0
- type: lookat_off:
  start: 0.0
- type: lookat_frame
  start: 0.0
  target: target_tf
- type: eye_open
  start: 0.0
- type: eye_close
  start: 0.0
- type: blink
  start: 0.0
```

This example can be found as [example.act](http://code.google.com/p/yale-ros-pkg/source/browse/trunk/sandbox/dragonbot_teleop_dialogue/example/example.act)

## Preparing your Dialogue ##

### Generating the Phrase File ###

Once you've written a script, you will need to split it up into phrases.  You will likely want these phrases to fit with the dialogue items described above.

For each phrase, you will need:
  1. An audio recording of the phrase (`*`.wav or `*`.ogg)
  1. An annosoft (`*`.anno) file corresponding to the above audio

You can also optionally include:
  1. An action file specifying robot behaviors corresponding to the behaviors associated with each phrase (`*`.act)
  1. A text file with the text of the phrase (`*`.txt)

For a single phrase, the audio, annosoft, and text files must have the same base filename (e.g. "my\_phrase.wav", "my\_phrase.anno", "my\_phrase.txt").  This base filename will be referred to throughout this tutorial as the _phrase`_`id_. The action file can have a different name, as it is not uncommon to have similar actions associated with several phrases (e.g. smiling at the beginning of the phrase).  The action file specification is provided in the section "File Specifications" below.

Once you have all of your phrases prepared, you will need to create a master list file, which has one (optional) line for each phrase (see [dragonbot\_teleop\_dialogue/example/list.txt](http://code.google.com/p/yale-ros-pkg/source/browse/trunk/sandbox/dragonbot_teleop_dialogue/example/list.txt)), with the phrase\_id and the associated action filename.  The audio, annosoft, text, and action files should all be stored in the same folder.

Next, you need to generate the phrase yaml file, which is used by the dragonbot speech node.  To do this, run:

```
> rosrun dragonbot_teleop_dialogue anno_parser.py <list file> <output file> <data dir>
```

Where the filenames are given with their full paths, and data dir is the full path to the directory containing the audio, annosoft, text, and action files.  For an example, see [regen\_phrases.sh](http://code.google.com/p/yale-ros-pkg/source/browse/trunk/sandbox/dragonbot_teleop_dialogue/example/regen_phrases.sh)



### Writing & Checking the Dialogue Specification ###

Armed with your phrase\_ids, you are ready to write the dialogue specification file.  The details on the format for this can be found in the section "File Specifications", above.  Once you've written your dialogue specification, you can check it by running the following script:

```
> rosrun dragonbot_teleop_dialogue check_dialogue.py <dialogue yaml file> <phrase yaml file>
```

Where the dialogue yaml file is the specification file you wrote, and the phrase yaml file is the file generated above.  This script will let you know if any of the tags are wrong, if you're trying to link to dialogue items that don't exist, and if you're trying to link to phrase IDs that don't exist.  For an example, see [check\_dialogue.sh](http://code.google.com/p/yale-ros-pkg/source/browse/trunk/sandbox/dragonbot_teleop_dialogue/example/check_dialogue.sh)

### Generating the GUI ###

Next, you need to generate the teleop GUI to go with the dialogue.  You can add items to the gui with an optional additional file.  The GUI will also generate the standard dragonbot teleop GUI, which can be used to control the robot directly by running:
```
> rosrun dragonbot_python dragonbot_teleop.py
```



Run the following:

```
> rosrun dragonbot_teleop_dialogue gen_gui.py [-i <optional yaml with additional guis in the interface_srv format>] [-o <output GUI file -- prints to stdout if not specified>] <your dialogue specification file>
```

You may need to provide full paths to the files.  For an example, see [regen\_gui.sh](http://code.google.com/p/yale-ros-pkg/source/browse/trunk/sandbox/dragonbot_teleop_dialogue/example/regen_gui.sh).  Note that this script writes directly to the interface\_srv/guis folder and _**WILL OVERWRITE THE GUI**_ that background nodes uses for the expeditions year 1 experiment.  You'll need to run the regen\_gui.sh script in the expeditions\_year1 folder if you want to run the experiment after this.

### Writing the Code ###

Finally, you're ready to run your dialogue using the DialogueManager class, which can be found in the _dragonbot\_teleop\_dialogue_ package as dialogue\_manager.py.  Note that there are a few exceptions that can be thrown by the system:
  * PanicException & NextStateException: These are thrown when "panic" or "next\_segment", respectively, is pressed as a response to a question.  There are two exceptions here so that you can handle them differently if desired.
  * NextPhraseException: This will move to the next phrase in a dialogue.  It is currently possible for the exception to be thrown from the dialogue manager, and can be safely ignored.  This is scheduled to be fixed (so that it will not be thrown at all) in later iterations.

The constructor for DialogueManager takes 4 arguments:
  * robot\_manager: a DragonbotManager object
  * tm: a TabletManager object
  * dialogue\_name: used to determine the name of the auto-generated GUI
  * dialogue: a dialogue from your specification file.  Each dialogue manager only handles one dialogue.


Example:
```
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
```

This file can be found in the examples folder as [example.py](http://code.google.com/p/yale-ros-pkg/source/browse/trunk/sandbox/dragonbot_teleop_dialogue/example/example.py).  If you have run regen\_phrases.py and regen\_gui.py, in that order, you should be able to run the script following the below instructions.


### Running the Teleoperated Dialogue ###

To run the dialogue, first start up the robot:
  1. Run the dragonbot\_relay:
```
> roslaunch dragonbot_relay dragonbot_relay.launch
```
  1. Run the dragonbot pubsub:
```
> roscd dragonbot_pubsub
> build/install/dragonbot_pubsub/bin/dragonbot_pubsub edu.yale.dragonbot.DragonbotNode
```
  1. Set the IP and hostname
```
> export ROS_IP=[your IP address here]
> export ROS_HOSTNAME=[your IP address here]
```
  1. Run the background nodes.  In the example, you can just run:
```
> roscd dragonbot_teleop_dialogue/example
> roslaunch background_nodes.launch
```
> > But if you're running your own dialogue, you will need to
```
> rosrun soundplay_node soundplay_node.py
> rosrun dragonbot_python speech.py <path to your phrases file>
> rosrun dragonbot_python tf_tracker.py
```
  1. Start the tablet and connect to your computer.  It should display a long list of GUIs.
  1. Run your script!

Note that the GUI will only change when the system plays a question, select, or wait dialogue item.  Right now, this means that the "panic", "next\_phrase" and "next\_state" buttons will only work (i.e. interrupt the robot) during a question, select, or wait dialogue item.