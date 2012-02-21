#!/usr/bin/env python

#
# ROS node to set Nao's LEDs through the Aldebaran API.
# This code is currently compatible to NaoQI version 1.12
#
# Copyright 2012 David Feil-Seifer, Yale University
# http://www.ros.org/wiki/nao_led
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import roslib
roslib.load_manifest('nao_led')
import rospy

from dynamic_reconfigure.server import Server
from nao_led.cfg import LEDsConfig

from nao_driver import *

import threading
from threading import Thread

class NaoLEDs(NaoNode,Thread):
    def __init__(self):
        NaoNode.__init__(self)
        Thread.__init__(self)

        # ROS initialization:
        rospy.init_node('nao_leds')
        self.connectNaoQi()
        srv = Server(LEDsConfig, self.callback)
        leftEyeGroup = []
        rightEyeGroup = []

        for index in range(1,9):
          leftEyeGroup.append( 'LeftFaceLed'+str(index) )
          rightEyeGroup.append( 'RightFaceLed'+str(index) )
          rospy.loginfo('adding: %s', 'LeftFaceLed'+str(index))

        self.ledProxy.createGroup('LeftEyeGroup', leftEyeGroup )
        self.ledProxy.createGroup('RightEyeGroup', rightEyeGroup )

        #self.ledProxy.rasta(2)
        result = self.ledProxy.listLEDs()
        self.ledProxy.fadeRGB( 'LeftEyeGroup', 0x00000000, 0.0 )
        self.ledProxy.fadeRGB( 'RightEyeGroup', 0x00000000, 0.0 )        

        self.ledProxy.off( 'ChestLedsGreen')
        self.ledProxy.off( 'ChestLedsRed')
        self.ledProxy.off( 'ChestLedsBlue')
        self.ledProxy.off( 'LeftFootLedsGreen')
        self.ledProxy.off( 'LeftFootLedsRed')
        self.ledProxy.off( 'LeftFootLedsBlue')
        self.ledProxy.off( 'RightFootLedsGreen')
        self.ledProxy.off( 'RightFootLedsRed')
        self.ledProxy.off( 'RightFootLedsBlue')

        for index in result:
          rospy.loginfo("All LEDs: %s", ''.join(index))
        rospy.loginfo("nao_led initialized")

    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.ledProxy = self.getProxy("ALLeds")
        if self.ledProxy is None:
            exit(1)

    def callback( self, config, level ):
        left_eye_color = 0x10000 * config['left_eye_R'] + 0x100 * config['left_eye_G'] + config['left_eye_B'];
        right_eye_color = 0x10000 * config['right_eye_R'] + 0x100 * config['right_eye_G'] + config['right_eye_B'];
        self.ledProxy.fadeRGB( 'RightEyeGroup', right_eye_color, 0.0 )
        self.ledProxy.fadeRGB( 'LeftEyeGroup', left_eye_color, 0.0 )

        if config['chest_led_R']:
          self.ledProxy.on("ChestLedsRed")
        else:
          self.ledProxy.off("ChestLedsRed")

        if config['chest_led_G']:
          self.ledProxy.on("ChestLedsGreen")
        else:
          self.ledProxy.off("ChestLedsGreen")
        if config['chest_led_B']:
          self.ledProxy.on("ChestLedsBlue")
        else:
          self.ledProxy.off("ChestLedsBlue")
 
        if config['right_foot_led_R']:
          self.ledProxy.on("RightFootLedsRed")
        else:
          self.ledProxy.off("RightFootLedsRed")
        if config['right_foot_led_G']:
          self.ledProxy.on("RightFootLedsGreen")
        else:
          self.ledProxy.off("RightFootLedsGreen")
        if config['right_foot_led_B']:
          self.ledProxy.on("RightFootLedsBlue")
        else:
          self.ledProxy.off("RightFootLedsBlue")

        if config['left_foot_led_R']:
          self.ledProxy.on("LeftFootLedsRed")
        else:
          self.ledProxy.off("LeftFootLedsRed")
        if config['left_foot_led_G']:
          self.ledProxy.on("LeftFootLedsGreen")
        else:
          self.ledProxy.off("LeftFootLedsGreen")
        if config['left_foot_led_B']:
          self.ledProxy.on("LeftFootLedsBlue")
        else:
          self.ledProxy.off("LeftFootLedsBlue")



        return config

if __name__ == '__main__':

    leds = NaoLEDs()
    leds.start()

    rospy.spin()

    rospy.loginfo("Stopping nao_leds ...")
    leds.stopThread = True
    leds.join()

    rospy.loginfo("nao_leds stopped.")
    exit(0)

