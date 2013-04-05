#!/usr/bin/env python

#Elaine Short
#Tablet python interface

import roslib; roslib.load_manifest('expeditions_year1')
import rospy
from dragonbot_simulator import DragonbotManager
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Duration
from interface_srv.srv import *
from interface_srv.msg import GUIElement
import sys

class TabletSimulator():
    def __init__(self):
        rospy.wait_for_service('gui_srv')
        try:
            srv_gui = rospy.ServiceProxy('gui_srv', GUIList)
            guis = srv_gui()
        except rospy.ServiceException, e:
            rospy.logerr("Failed to get GUIs: %s"%e)
            sys.exit()
        self.guis = guis.guis
        self.topic_change_sub = rospy.Subscriber('/change_topic', String, self.topic_change) 
        self.gui_changed = False
        print "Tablet Started"
        
        while not self.gui_changed and not rospy.is_shutdown():
            rospy.sleep(0.5)

            
        while not rospy.is_shutdown():
            print '================================================='
            print '          ' + self.goal_gui
            print '================================================='
            i = 0
            for g in self.get_gui(self.goal_gui).elements:
                i = i + 1
                kind = g.type
                if kind == GUIElement.BUTTON_ID:
                    print str(i) + ": Button " + g.label
                    print "    1: press button"
                elif kind == GUIElement.BUTTONGROUP_ID:
                    print  str(i) + ": Button Group " + g.label
                    j = 0
                    for l in g.label_string.split(';'):
                        j = j + 1
                        print "    " +  str(j) + ": " + l
                elif kind == GUIElement.INTSLIDER_ID:
                    print str(i) + ": Slider " + g.label
            
            self.gui_changed = False
            while not self.gui_changed:
                print '================================================='
                print "Enter update: <element number>-<value> (e.g. 1-3)"
                print '-------------------------------------------------'
                s = raw_input()
                try:
                    vals = s.split('-')
                    item = int(vals[0])
                    value = int(vals[1])
                except:
                    print "Bad input"

                gui_item = self.get_gui(self.goal_gui).elements[item-1]
                kind = gui_item.type
                if kind == GUIElement.BUTTON_ID:
                    topic_type = Int32
                    value = 0
                elif kind == GUIElement.BUTTONGROUP_ID:
                    topic_type = String
                    value = gui_item.label_string.split(';')[value-1]
                elif kind == GUIElement.INTSLIDER_ID:
                    topic_type = Int32
                    if value > gui_item.max or value < gui_item.min:
                        print "Bad value."
                        continue
                p = rospy.Publisher(gui_item.topic, topic_type)

                if self.gui_changed:
                    break
                else:
                    p.publish(value)


    def get_gui(self, name):
        r = filter(lambda g: g.guiname == name, self.guis)
        return r[0]

    def topic_change(self, data):
        print "Changing to GUI " + data.data
        self.goal_gui = data.data
        self.gui_changed = True

    


def main():
    rospy.init_node('sim_tablet')
    TabletSimulator()
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()

