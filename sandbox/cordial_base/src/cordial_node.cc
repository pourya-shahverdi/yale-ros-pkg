#include <ros/ros.h>

#include <pluginlib/class_loader.h>
#include <cordial_base_/cordial_base.h>

#include <actionlib/server/simple_action_server.h>

int main( int argc, char* argv[] )
{
  ros::NodeHandle nh;
  pluginlib::ClassLoader<cordial_base::CordialBase> plug_loader("cordial_base", "cordial_base::CordialBase");
  cordial_base::CordialBase* robot = NULL;



  // get plugin name
  std::string plugin_name = std::string("");

  
  // load plugin
  try
  {
    robot = plug_loader.createClassInstance(plugin_name);
    robot->initialize();

    robot->viseme_init();
    ROS_INFO("Base Name: %s", base->robot_name().c_str());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  // start actionservers, if available


  return 0;
}
