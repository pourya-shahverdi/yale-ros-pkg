#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <cordial_base_/cordial_base.h>
#include <pluginlib/class_loader.h>

int main( int argc, char* argv[] )
{
  ros::init(argc, argv, "cordial_node");
  ros::NodeHandle nh;
  pluginlib::ClassLoader<cordial_base::CordialBase> plug_loader("cordial_base", "cordial_base::CordialBase");
  cordial_base::CordialBase* robot = NULL;

  // get plugin name
  std::string plugin_name = std::string("");
  nh.param("cordial_robot",plugin_name, std::string(""));
  
  actionlib::SimpleActionServer<dragon_msgs::VisemeAction> * as;

  // load plugin
  try
  {
    robot = plug_loader.createClassInstance(plugin_name);
    robot->initialize();

    robot->viseme_init();
    ROS_INFO("Base Name: %s", robot->robot_name().c_str());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  // initialize actionservers
  robot->init_actionservers(nh);
  /*
   robot->viseme_init(); 
  {
    as = new actionlib::SimpleActionServer<dragon_msgs::VisemeAction>(nh, "Viseme_Server", boost::bind(&cordial_base::CordialBase::viseme_execute, robot, _1), false);
    as->start();
  }
  */
  // start actionservers, if available

  ros::spin();

  return 0;
}
