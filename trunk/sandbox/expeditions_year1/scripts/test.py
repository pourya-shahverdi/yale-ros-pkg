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
    sc.stopAll()
    sc.playWave('/home/eshort/fuerte_workspace/yale-ros-pkg/sandbox/expeditions_year1/music/thought_of_you.wav')
    rospy.sleep(5.0)
    sc.waveVol('/home/eshort/fuerte_workspace/yale-ros-pkg/sandbox/expeditions_year1/music/thought_of_you.wav', .5)
    rospy.sleep(5.0)
    sc.waveVol('/home/eshort/fuerte_workspace/yale-ros-pkg/sandbox/expeditions_year1/music/thought_of_you.wav', 1)
    rospy.sleep(5.0)
    sc.waveVol('/home/eshort/fuerte_workspace/yale-ros-pkg/sandbox/expeditions_year1/music/thought_of_you.wav', .5)
    rospy.sleep(5.0)
    sc.playWave('/home/eshort/fuerte_workspace/yale-ros-pkg/sandbox/expeditions_year1/phrases/data/okay.wav')
    rospy.sleep(5)
    sc.stopAll()

if __name__ == '__main__':
    main()
