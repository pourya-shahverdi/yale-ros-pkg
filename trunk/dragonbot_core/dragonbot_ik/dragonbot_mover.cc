#include "ros/ros.h"
#include ""

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Check_Button.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Box.H>
#include <stdio.h>
#include <sys/time.h>

#define DTOR(a) ( (a) * M_PI / 180.0f )

Fl_Value_Slider* q[6];

//function to record current time
double get_time()
{
  struct timeval tp;
  gettimeofday(&tp,0);
  return ((double)tp.tv_sec + (double)tp.tv_usec/1e6);
}

ros::Publisher joint_state_publisher;

