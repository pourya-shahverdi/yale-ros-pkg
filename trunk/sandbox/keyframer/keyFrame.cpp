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

using namespace std;

Fl_Hor_Value_Slider* sliders[4];
Fl_Select_Browser* output;

void slider_cb(Fl_Widget* o, void*) {
    Fl_Hor_Value_Slider *slider = (Fl_Hor_Value_Slider*)o;

    double xVal = sliders[0]->value();
    double yVal = sliders[1]->value();
    double zVal = sliders[2]->value();
    double tVal = sliders[3]->value();

    printf("%0.2f, %0.2f, %0.2f, %0.2f\n", xVal, yVal, zVal, tVal);
    
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
    return Fl::run();
}
