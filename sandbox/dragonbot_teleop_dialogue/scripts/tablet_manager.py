#!/usr/bin/env python

#Elaine Short
#Tablet python interface

import roslib; roslib.load_manifest('expeditions_year1')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Duration
from interface_srv.srv import *
from interface_srv.msg import GUIElement
import sys

class TabletManager():
    def __init__(self):
        self.pub = rospy.Publisher('/change_topic', String)
        rospy.wait_for_service('gui_srv')
        try:
            get_gui = rospy.ServiceProxy('gui_srv', GUIList)
            guis = get_gui()
        except rospy.ServiceException, e:
            rospy.logerr("Failed to get GUIs: %s"%e)
            sys.exit()
        self.guis = guis.guis
        self.subs = {}
        gui_name = "sleep"
        if not gui_name in map(lambda g: g.guiname, self.guis):
            rospy.logwarn("Invalid GUI name.")
            return
        rospy.loginfo("Changing to GUI: " + gui_name)
        self.current_gui_name = gui_name
        self.pub.publish(self.current_gui_name)
        rospy.sleep(0.5)

    def get_gui(self):
        r = filter(lambda g: g.guiname == self.current_gui_name, self.guis)
        return r[0]

    def update_subs(self, topics):
        for topic in topics:
            # Different elements publish different topic types
            topic_type = String
            cb = self.string_cb

            name,kind = topic

            if kind == GUIElement.BUTTON_ID:
                topic_type = Int32
                cb = self.int_cb
            elif kind == GUIElement.BUTTONGROUP_ID:
                topic_type = String
                cb = self.string_cb
            elif kind == GUIElement.STOPWATCH_ID:
                topic_type = Duration
                cb = self.dur_cb
            elif kind == GUIElement.INTSLIDER_ID:
                topic_type = Int32
                cb = self.int_cb
            else:
                rospy.logwarn("Unsupported element type.")
                continue
            
            if name.strip('/') not in self.subs:
                print "Subscribing to topic: " + name
                rospy.Subscriber(name, topic_type, cb)

            # reset info on the topic, even if already subscribed
            self.subs[name.strip('/')] = {"type_id":kind,
                                   "pressed":False,
                                   "last_press": ""}
 
    def last_press(self, topic):
        topic = topic.strip('/')
        return self.subs[topic]["last_press"]

    def wait_for_press(self,topic, value=None):
        topic = topic.strip('/')
        if value == None:
            self.subs[topic]["pressed"] = False
            while not self.subs[topic]["pressed"]:
                if rospy.is_shutdown():
                    return
                rospy.sleep(0.1)
            ret = self.subs[topic]["last_press"]
        else:
            #don't return right away if the value we are waiting for was the
            #last value pressed before we started waiting
            self.subs[topic]["pressed"] = False
            while not self.subs[topic]["pressed"]:
                if rospy.is_shutdown():
                    return
                rospy.sleep(0.1)
            #now we can just wait until the last_press value is what we want
            while not self.subs[topic]["last_press"] == value:
                if rospy.is_shutdown():
                    return
                rospy.sleep(0.1)
            ret = self.subs[topic]["last_press"]
        return ret

                
    def int_cb(self,data):
        #print "got: " + str(data.data)
        topic = data._connection_header["topic"].strip('/')
        self.subs[topic]["pressed"] = True
        self.subs[topic]["last_press"] = data.data

    def string_cb(self,data):
        #print "got: " + data.data
        topic = data._connection_header["topic"].strip('/')
        self.subs[topic]["pressed"] = True
        self.subs[topic]["last_press"] = data.data

    #this isn't working right now
    def dur_cb(self,data):
        secs = data.data.secs
        nsecs = data.data.nsecs
        print "got: (" + str(secs) + "," + str(nsecs) + ")"
        topic = data._connection_header["topic"].strip('/')
        self.subs[topic]["pressed"] = True
        self.subs[topic]["last_press"] = data.data


    def change(self, gui_name):
        if not gui_name in map(lambda g: g.guiname, self.guis):
            rospy.logwarn("Invalid GUI name.")
            return
        rospy.loginfo("Changing to GUI: " + gui_name)
        self.current_gui_name = gui_name
        self.pub.publish(self.current_gui_name)
        self.current_gui = self.get_gui()
        topics = list(set(map(lambda e: (e.topic,e.type), self.current_gui.elements)))
        self.update_subs(topics)
        
    def get_topics(self):
        return self.subs.keys()
