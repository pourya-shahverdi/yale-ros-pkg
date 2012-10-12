/*
 * Interface Server
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

bool gui_srv(interface_srv::GUIList::Request &req, interface_srv::GUIList::Response &res){
  res = response;//dereference?
  return true;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "srv_parser");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // read from YAML file to build 2-D array of GUI's and their elements
  std::string filename = std::string("");
  nh_priv.param( "guifilename", filename, std::string("") );


  if( filename == std::string("") ){
    ROS_WARN( "no file name specified" );
    return false;
  }

  std::ifstream fin;
  fin.open(filename.c_str());
  if(fin.fail()){
    ROS_WARN("couldn't find file: [%s]", filename.c_str());
    return false;
  }
  YAML::Parser parser(fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);

  for(std::size_t i = 0;i < doc.size(); i++){
    std::string guiname;
    doc[i]["gui"] >> guiname;
    ROS_INFO( "found GUI: %s", guiname.c_str() );

    // copy data into GUI response msg
    interface_srv::GUI gui;

    gui.guiname = guiname;
    ROS_INFO( "GUIName: %s", guiname.c_str() );

    const YAML::Node& y_elements = doc[i]["elements"];

    for(unsigned int i = 0; i < y_elements.size(); i++){
      interface_srv::GUIElement element;	

      // copy label
      y_elements[i]["label"] >> element.label;

      // copy type
      std::string elem_type;
      y_elements[i]["type"] >> elem_type;
      
      //determine element type
      if(!elem_type.compare("button"))
        element.type = interface_srv::GUIElement::BUTTON_ID;
      else if(!elem_type.compare("slider"))
        element.type = interface_srv::GUIElement::SLIDER_ID;
      else if(!elem_type.compare("stopwatch"))
        element.type = interface_srv::GUIElement::STOPWATCH_ID;
      else if(!elem_type.compare("countdown_timer"))
        element.type = interface_srv::GUIElement::COUNTDOWN_ID;
      else if(!elem_type.compare("image_view"))
        element.type = interface_srv::GUIElement::IMAGE_ID;
      else if(!elem_type.compare("int_slider"))
	element.type = interface_srv::GUIElement::INTSLIDER_ID;
      else if(!elem_type.compare("button_group"))
	element.type = interface_srv::GUIElement::BUTTONGROUP_ID;
      else{
       	ROS_WARN( "element type: %s unrecognized", elem_type.c_str() );
      }
      // copy topic
      y_elements[i]["topic"] >> element.topic;

      if (element.type == interface_srv::GUIElement::INTSLIDER_ID || element.type == interface_srv::GUIElement::SLIDER_ID){
        // copy min/max in case of slider
  	y_elements[i]["min"] >> element.min;
	y_elements[i]["max"] >> element.max;
      }
      //copy label_string from element
      if (element.type == interface_srv::GUIElement::BUTTONGROUP_ID){
        y_elements[i]["label_string"] >> element.label_string;
      }
      gui.elements.push_back(element);
    }
   
    response.guis.push_back(gui);
  }
  
  
  // wait for requests
  ros::ServiceServer service = nh.advertiseService("gui_srv", gui_srv);	
  ros::spin();
  
  return 0;
}
