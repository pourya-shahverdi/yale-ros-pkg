import roslib; roslib.load_manifest('expeditions_year1')
import rospy
from dragonbot_simulator import DragonbotManager
from std_msgs.msg import String

def main():
    pub = rospy.Publisher('/change_topic', String)
    rospy.init_node('topic_changer')
    pub.publish('BLAH')
    for i in range(0,5):
        print "intro"
        pub.publish('Intro')
        print str(i)
        rospy.sleep(1.0)
        print "teleop"
        pub.publish('Dragonbot Teleop');
        rospy.sleep(1.0)

if __name__ == '__main__':
    main()
