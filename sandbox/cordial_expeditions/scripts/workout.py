#!/usr/bin/env python

# Kate Swift-Spong
# Workout section state machine

import roslib; roslib.load_manifest('expeditions_year1')
import rospy
import smach
import smach_ros
import sys

# class definitions for states are in the file workout_states.py
from workout_states import *
        
def main():
    rospy.init_node('workout')

    sm = smach.StateMachine(outcomes=['end'])
 
    with sm:
        smach.StateMachine.add('ENERGY_INTRO', EnergyIntro(),
                               transitions={'done':'DANCE'})
        smach.StateMachine.add('TIRED_INTRO', TiredIntro(),
                               transitions={'done':'DANCE'})
        smach.StateMachine.add('DANCE', Dance(),
                               transitions={'child_not_dancing':'CHILD_NOT_DANCING',
                                            'robot_not_dancing':'ROBOT_NOT_DANCING',
                                            'continue':'TIRED_DANCE',
                                            'panic':'end'})
        smach.StateMachine.add('CHILD_NOT_DANCING', ChildNotDancing(),
                               transitions={'dancing_again':'DANCE',
                               				'panic':'end'})
        smach.StateMachine.add('ROBOT_NOT_DANCING', RobotNotDancing(),
                               transitions={'motivated':'DANCE',
                               				'panic':'end'})
        smach.StateMachine.add('OUTRO', Outro(),
                               transitions={'end':'end',
                                            'panic':'end'})
                                            
	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	# Execute the state machine
	outcome = sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
    main()
