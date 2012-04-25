Get Nao dependencies:
Install rosinstall: http://www.ros.org/wiki/rosinstall#Installation
Install Nao ROS packages: http://www.ros.org/wiki/nao/Installation

Install some of Dave's old code:
$ svn co https://usc-ros-pkg.svn.sourceforge.net/svnroot/usc-ros-pkg/trunk/turn_taking/tf_to_pose
$ rospack profile
$ rosdep install tf_to_pose
$ rosmake tf_to_pose

Run the following after package creation to make sure on path:
$ rospack profile
$ rospack find imitation

Install dependencies:
$ rosdep install imitation

Make package:
$ rosmake imitation

Make nodes executable:
$ chmod +x imitation/nodes/angle_calculator.py

To run roscore:
$ roscore

To play the bag file:
$ rosbag play --clock sandbox/skeleton_data/data/2012-02-21-15-37-28.bag

To run rviz:
$ rosparam set use_sim_time true
$ rosrun rviz rviz
Set Fixed Frame to: /openni_rgb_frame
Set Target Frame to: <Fixed Frame>

If you want to use the fake Nao model in rviz:
$ rosmake fake_nao
$ roslaunch imitation fake_robo.launch

To run angle_calculator:
$ rosrun imitation angle_calculator.py

Before committing:
$ cd sandbox/imitation
$ make clean

Coordinates:
X coordinate is front/back of robot (back is more positive)
Y coordinate is left/right of robot (left is more positive)
Z coordinate is top/bottom of robot (top is more positive)

Joints:
http://developer.aldebaran-robotics.com/doc/1-12/nao/hardware/kinematics/nao-joints-33.html

Offsets:
Shoulder:
	0 pitch is perpendicular to torso.  Positive rotates down.  Negative rotates up.
	0 roll is straight out in front of body
	
TESTING:
	SHOULDER PITCH: Meh
	SHOULDER ROLL: GOOD
	ELBOW YAW: Meh
	ELBOW ROLL: Meh