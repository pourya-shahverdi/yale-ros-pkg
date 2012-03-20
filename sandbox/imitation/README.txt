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

To run angle_calculator:
$ rosrun imitation angle_calculator.py

Before committing:
$ cd sandbox/imitation
$ make clean
