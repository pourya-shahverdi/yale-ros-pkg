/*
 * openni_skeleton
 * Copyright (c) 2012, David Feil-Seifer
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the yale-ros-pkgs nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

std::string suffix = "";

void add_line( tf::TransformListener* tl, visualization_msgs::Marker* ll, std::string to_frame, std::string from_frame )
{
	tf::StampedTransform t1, t2;
	try {
		tl->lookupTransform(ll->header.frame_id, to_frame+suffix, ros::Time(0), t1);
		tl->lookupTransform(ll->header.frame_id, from_frame+suffix, ros::Time(0), t2);
		
		geometry_msgs::Point p1;
		p1.x = t1.getOrigin().x();
		p1.y = t1.getOrigin().y();
		p1.z = t1.getOrigin().z();

		geometry_msgs::Point p2;
		p2.x = t2.getOrigin().x();
		p2.y = t2.getOrigin().y();
		p2.z = t2.getOrigin().z();

		// line list publishes lines by pairs of points

		ll->points.push_back(p1);
		ll->points.push_back(p2);
}
	catch( tf::TransformException ex ) 
	{
		ROS_WARN( "%s (unable to draw full skeleton)", ex.what());
	}

}

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "openn_skeleton" );
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	
	nh_priv.param("suffix", suffix, std::string("_1") );
  std::string pubname = "person_marker" + suffix;
	ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>(pubname, 10 );

	// start up tf listener
	tf::TransformListener tl;	


	// at a set rate
	ros::Rate loop_rate(10);

	visualization_msgs::Marker line_list;
	line_list.header.frame_id = std::string("torso")+suffix;
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.ns = "visualization"+suffix;
	line_list.pose.orientation.w = 1.0;
	line_list.color.g = 1.0;
	line_list.color.a = 1.0;
	line_list.scale.x = 0.02;		
	line_list.scale.y = 0.02;		

	while ( ros::ok() )
	{
		// draw body (as a line list of connected points)
		line_list.header.stamp = ros::Time::now();
		line_list.points.clear();
		add_line( &tl, &line_list, std::string("neck"), std::string("head") );
		add_line( &tl, &line_list, std::string("neck"), std::string("left_shoulder") );
		add_line( &tl, &line_list, std::string("neck"), std::string("right_shoulder") );
		add_line( &tl, &line_list, std::string("right_hip"), std::string("left_shoulder") );
		add_line( &tl, &line_list, std::string("left_hip"), std::string("right_shoulder") );
		add_line( &tl, &line_list, std::string("left_hip"), std::string("right_hip") );
		add_line( &tl, &line_list, std::string("right_elbow"), std::string("right_shoulder") );
		add_line( &tl, &line_list, std::string("right_elbow"), std::string("right_hand") );
		add_line( &tl, &line_list, std::string("left_elbow"), std::string("left_shoulder") );
		add_line( &tl, &line_list, std::string("left_elbow"), std::string("left_hand") );
		
		add_line( &tl, &line_list, std::string("left_hip"), std::string("left_knee") );
		add_line( &tl, &line_list, std::string("left_foot"), std::string("left_knee") );
		add_line( &tl, &line_list, std::string("right_hip"), std::string("right_knee") );
		add_line( &tl, &line_list, std::string("right_foot"), std::string("right_knee") );


		vis_pub.publish(line_list);
		
		ros::spinOnce();
		loop_rate.sleep();
	}



	return 0;
}
