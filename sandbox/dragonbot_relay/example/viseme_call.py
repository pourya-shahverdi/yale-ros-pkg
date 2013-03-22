import roslib; roslib.load_manifest('dragonbot_relay')
import rospy
from time import sleep
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import dragon_msgs.msg

def dragon_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('Viseme_Server', dragon_msgs.msg.VisemeAction)

    print 'waiting for server... \n' 

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    print 'sending goal... \n'

    # Creates a goal to send to the action server.
    goal = dragon_msgs.msg.VisemeGoal(constant='N_NG_D_Z')



    # Sends the goal to the action server.
    client.send_goal(goal)

    rospy.sleep(5.)

    #client.cancel_all_goals()
    print "Sending new goal... \n"
    goal = dragon_msgs.msg.VisemeGoal(constant='AO_AW')
    client.send_goal(goal)
    rospy.sleep(10.)
    client.cancel_all_goals()
    print 'cancelling goal... \n'

    # Waits for the server to finish performing the action.
    #client.wait_for_result()

    print 'returning result... \n'

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('dragon_client_py')
        result = dragon_client()
        print result
        # print "Result:", ', '.join([str(n) for n in result.sequence])
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

