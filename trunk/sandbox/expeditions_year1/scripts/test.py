#!/usr/bin/env python

import roslib; roslib.load_manifest('expeditions_year1')
import rospy
from dragonbot_simulator import DragonbotManager
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

def main():
    rospy.init_node('test_node')
    sc = SoundClient()
    rospy.sleep(1.0)
    sc.playWave('/home/eshort/fuerte_workspace/yale-ros-pkg/sandbox/expeditions_year1/music/thought_of_you.wav')
    rospy.sleep(15)
    sc.playWave('/home/eshort/fuerte_workspace/yale-ros-pkg/sandbox/expeditions_year1/phrases/data/okay.wav')


if __name__ == '__main__':
    main()
