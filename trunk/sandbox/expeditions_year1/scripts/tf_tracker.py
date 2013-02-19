#!/usr/bin/env python

# Elaine Short
# Dragonbot oculesic controller

import roslib; roslib.load_manifest('expeditions_year1')
import rospy
import actionlib





#TODO: Use stoppable threads here
class Tracker():
    def __init__(self):
        self.stop = Event()
        self.t = Thread(target=self.tracking_thread)
        
    def tracking_thread(self, target):
        while not self.stop.isSet():
            print "tracking frame: " + target
            #TODO: listener = tf.TransformListener()
            #try:
            #    (trans, rot) = listener.lookupTransform('/dragonbot',target, rospy.Time(0))
            #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #    continue
            #dm.lookat(trans[0],trans[1],trans[2]) #assuming trans is x,y,z
            rospy.sleep(1.0)

    def start(self, target):
        if len(target) == 0:
            rospy.logwarn("No target set. Tracking not started.")
        else:
            self.on = True
            self.t = Thread(target=self.tracking_thread, args=(target,))
            self.t.start()

    def stop(self):
        self.stop.set()
        if self.t.is_alive():
            self.t.join()

