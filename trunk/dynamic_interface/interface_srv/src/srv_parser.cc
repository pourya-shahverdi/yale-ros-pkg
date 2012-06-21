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
#include <interface_srv/GUIList.h>

interface_srv::GUIList::Response response;

bool gui_srv(interface_srv::GUIList::Request   &req,
	           interface_srv::GUIList::Response  &res )
{
	res = response;//dereference?
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
  interface_srv::GUI gui;

	gui.guiname = guiname;
	ROS_INFO( "GUIName: %s", guiname.c_str() );

	const YAML::Node& y_elements = doc[0]["elements"];

	for( unsigned int i = 0; i < y_elements.size(); i++ )
	{
		interface_srv::GUIElement element;	
		
		// copy label
		y_elements[i]["label"] >> element.label;

		// copy type 0: button 1: slider
		std::string elem_type;
		y_elements[i]["type"] >> elem_type;
		if(!elem_type.compare("button"))
			element.type = interface_srv::GUIElement::BUTTON_ID;
		else if(!elem_type.compare("slider"))
			element.type = interface_srv::GUIElement::SLIDER_ID;
    else if(!elem_type.compare("stopwatch"))
      element.type = interface_srv::GUIElement::STOPWATCH_ID;
    else if(!elem_type.compare("countdown_timer"))
      element.type = interface_srv::GUIElement::COUNTDOWN_ID;
    else
    {
      ROS_WARN( "element type: %s unrecognized", elem_type.c_str() );
    }
		// copy topic
		y_elements[i]["topic"] >> element.topic;

		if (element.type == interface_srv::GUIElement::SLIDER_ID){
			// copy min/max
			y_elements[i]["min"] >> element.min;
			y_elements[i]["max"] >> element.max;
		}

		gui.elements.push_back(element);
	}
  response.guis.push_back(gui);

	// wait for requests
	ros::ServiceServer service = nh.advertiseService("gui_srv", gui_srv);	
	ros::spin();
	return 0;
}
