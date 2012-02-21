/*
 * nao_mover
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


#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nao_msgs/JointAnglesWithSpeed.h"

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Check_Button.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Box.H>
#include <stdio.h>
#include <sys/time.h>

#define DTOR(a) ( (a) * M_PI / 180.0f )


bool seen_joint = false;
sensor_msgs::JointState joint_msg;
Fl_Check_Button* p;
Fl_Value_Slider* q[100];
std::vector<std::string> joint_names;
ros::Publisher joint_publisher;

//callback function for sliders
void slider_cb( Fl_Widget* o, void* )
{
//  bandit_msgs::JointArray jarray;
	const char* label = o->label();
  std::string joint_name = std::string(label);
	Fl_Valuator* oo = (Fl_Valuator*) o;

	nao_msgs::JointAnglesWithSpeed js;

	js.joint_names.push_back(joint_name);
	js.joint_angles.push_back(DTOR(oo->value()));
  js.speed = 0.1;

	printf( "setting %0.2f: %s\n", oo->value(), joint_name.c_str() );
	joint_publisher.publish(js);
}

void joint_cb(const sensor_msgs::JointStateConstPtr &jnt)
{
  seen_joint = true;
  joint_msg = *jnt;
}

int main( int argc, char* argv[] )
{
	ros::init(argc,argv,"nao_mover");
	ros::NodeHandle nh;
	joint_publisher = nh.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles",5);
  ros::Subscriber joints_sub = nh.subscribe("joint_states",1,&joint_cb);

  ros::Rate loop_rate(10);

  while( ros::ok() && !seen_joint )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // build gui
  int slider_w = 400;
  int slider_h = 20;
  int padding = 20;

  int win_w = padding*2+slider_w;
  int num_sliders = joint_msg.name.size();
  int win_h = padding*(4+num_sliders)+(num_sliders+1)*slider_h;

  Fl_Window win( win_w, win_h, "Bandit Mover" );
  win.begin();
  Fl_Value_Slider* sliders[19];

  for( int i = 0; i < joint_msg.name.size(); i++ )
  {
    ROS_INFO( "adding joint: [%s]", joint_msg.name[i].c_str() );
    sliders[i] = new Fl_Value_Slider( padding, (i+1)*padding+i*slider_h,slider_w, slider_h, joint_msg.name[i].c_str() );
    sliders[i]->type(FL_HORIZONTAL);
    sliders[i]->bounds(-180, 180 );
    sliders[i]->callback( slider_cb );
    joint_names.push_back(joint_msg.name[i]);
    q[i] = sliders[i];
  }

  win.end();
  win.show();


  while( ros::ok() && Fl::check() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

	return 0;
}
