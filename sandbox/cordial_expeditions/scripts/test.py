#!/usr/bin/env python

import roslib; roslib.load_manifest('expeditions_year1')
import rospy
from dragonbot_simulator import DragonbotManager
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from interface_srv.srv import *

def main():
    rospy.init_node('test_node')
    print roslib.packages.get_pkg_dir("expeditions_year1")
    

if __name__ == '__main__':
    main()
