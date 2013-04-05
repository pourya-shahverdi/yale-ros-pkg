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
    rospy.wait_for_service('gui_srv')
    try:
        get_gui = rospy.ServiceProxy('gui_srv', GUIList)
        resp = get_gui()
        print resp.guis[0]
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    main()
