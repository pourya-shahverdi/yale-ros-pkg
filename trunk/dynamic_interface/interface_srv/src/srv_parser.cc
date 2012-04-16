#include <ros/ros.h>

// yaml-cpp includes
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <sstream>
#include <yaml-cpp/yaml.h>

// service and message includes
#include <interface_srv/GUI.h>

interface_srv::GUI::Response response;

bool gui_srv(interface_srv::GUI::Request   &req,
	     interface_srv::GUI::Response  &res )
{
	res = response;
	return true;
}

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "srv_parser" );
	ros::NodeHandle nh;

	// read from YAML file

	if( argc < 2 )
	{
		ROS_WARN( "no file name specified" );
		return false;
	}

	std::string filename=argv[1];
	std::ifstream fin;
	fin.open(filename.c_str());
	if( fin.fail() ) 
	{
		ROS_WARN("couldn't find file: [%s]", filename.c_str());
		return false;
	}

	YAML::Node doc;
	YAML::Parser parser(fin);
	parser.GetNextDocument(doc);

	std::string guiname;
	doc[0]["gui"] >> guiname;
	
	// copy data into GUI response msg

	response.guiname = guiname;
	ROS_INFO( "GUIName: %s", guiname.c_str() );

	const YAML::Node& y_elements = doc[0]["elements"];

	for( int i = 0; i < y_elements.size(); i++ )
	{
		interface_srv::GUIElement element;
		// copy label

		// copy type 0: button 1: slider

		// copy topic

		// if slider
			// copy min/max

		response.elements.push_back(element);
	}

	// wait for requests
	ros::ServiceServer service = nh.advertiseService("gui_srv", gui_srv);	
	ros::spin();
	return 0;
}