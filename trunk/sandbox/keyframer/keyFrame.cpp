/*
 * keyFrame
 * Copyright (c) 2013, Aditi Ramachandran
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

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Hor_Value_Slider.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Text_Display.H>
#include <FL/Fl_Text_Editor.H>
#include <Fl/Fl_Text_Buffer.H>
#include <Fl/Fl_Select_Browser.H>
#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

using namespace std;

Fl_Hor_Value_Slider* sliders[4];
Fl_Select_Browser* output;

ros::Publisher ik_publisher;

void slider_cb(Fl_Widget* o, void*) {
    Fl_Hor_Value_Slider *slider = (Fl_Hor_Value_Slider*)o;

    double xVal = sliders[0]->value();
    double yVal = sliders[1]->value();
    double zVal = sliders[2]->value();
    double tVal = sliders[3]->value();

    printf("%0.2f, %0.2f, %0.2f, %0.2f\n", xVal, yVal, zVal, tVal);
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = xVal / 1000.;
    pose_msg.position.y = yVal / 1000.;
    pose_msg.position.z = zVal / 1000.;
    ik_publisher.publish(pose_msg);
}

void button_cb(Fl_Widget* o, void*) {
    Fl_Button *text = (Fl_Button*) o;

    double xVal = sliders[0]->value();
    double yVal = sliders[1]->value();
    double zVal = sliders[2]->value();
    double tVal = sliders[3]->value();

    stringstream convertX;
    convertX << xVal;
    stringstream convertY;
    convertY << yVal;
    stringstream convertZ;
    convertZ << zVal;
    stringstream convertT;
    convertT << tVal;
    
    //string s = convert.str() + ", \n";
    string s = convertX.str() + ", " + convertY.str() + ", " + convertZ.str() + ", " + convertT.str() + "\n"; 
    //string s; 
    //s = "hello\n";
    //cout << "string s is: " << s << endl;
    //int position = output->insert_position();
    //cout << "insert position is: " << position << endl;
    output->add(s.c_str());
     
    //**CHANGE TO print a line to the console every time one of the four
    //sliders are changed and write to the textbox when the frame button
    //is pressed. items in the textbox should be in a list that you can
    //select (cause eventually we will want to be able to delete them
    printf("%0.2f, %0.2f, %0.2f, %0.2f\n", xVal, yVal, zVal, tVal);

}

void button2_cb(Fl_Widget* o, void*) {
    Fl_Button *button = (Fl_Button*) o;

    //find which line in the select browser is SELECTED and delete it
    int value = output->value();
    if (value!=0){
	output->remove(value);	
    }

}

void button3_cb(Fl_Widget* o, void*) {
    output->clear();
}

int main(int argc, char **argv) {

    ros::init(argc,argv,"keyframer");
    ros::NodeHandle nh;
    ik_publisher = nh.advertise<geometry_msgs::Pose>("ik",5);

    Fl_Window *window = new Fl_Window(340,600);
    //Fl_Box *box = new Fl_Box(20,40,100,40,"Hello, Aditi!");



    Fl_Hor_Value_Slider *xSlider = new Fl_Hor_Value_Slider(20,20,310,20, "x value");
    Fl_Hor_Value_Slider *ySlider = new Fl_Hor_Value_Slider(20,60,310,20, "y value");
    Fl_Hor_Value_Slider *zSlider = new Fl_Hor_Value_Slider(20,100,310,20, "z value");
    Fl_Hor_Value_Slider *tSlider = new Fl_Hor_Value_Slider(20,140,310,20, "theta value");

    xSlider->bounds(-80, 80);
    ySlider->bounds(-80, 80);
    zSlider->bounds(69, 150);
    tSlider->bounds(-60, 60);

    sliders[0] = xSlider;
    sliders[1] = ySlider;
    sliders[2] = zSlider;
    sliders[3] = tSlider;

    Fl_Button *button = new Fl_Button(110, 180, 100, 30, "FRAME");


    output = new Fl_Select_Browser(20, 230, 310, 200, "recorded frames" );

    xSlider->callback(slider_cb);
    ySlider->callback(slider_cb);
    zSlider->callback(slider_cb);
    tSlider->callback(slider_cb);
    button->callback(button_cb);
    //output = new Fl_Text_Editor(20, 230, 310, 200, "recorded values" );

    Fl_Button *delButton = new Fl_Button(50, 480, 100, 30, "delete frame");
    delButton->callback(button2_cb);

    Fl_Button *clearButton = new Fl_Button(50, 530, 100, 30, "clear all frames");
    clearButton->callback(button3_cb);

    Fl_Button *saveButton = new Fl_Button(180, 480, 100, 30, "save frames");

    Fl_Button *playButton = new Fl_Button(180, 530, 100, 30, "play all");

    //box->box(FL_UP_BOX);
    //box->labelfont(FL_BOLD+FL_ITALIC);
    //box->labelsize(36);
    //box->labeltype(FL_SHADOW_LABEL);
    button->type(FL_NORMAL_BUTTON);
    window->end();
    window->show(argc, argv);

    ros::Rate loop_rate(100);

    while( ros::ok() && Fl::check() )
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
}
