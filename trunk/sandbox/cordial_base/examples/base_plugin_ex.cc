#include <pluginlib/class_loader.h>
#include <cordial_base_/cordial_base.h>

int main(int argc, char** argv)
{
  pluginlib::ClassLoader<cordial_base::CordialBase> poly_loader("cordial_base", "cordial_base::CordialBase");
  cordial_base::CordialBase* base = NULL;

  try
  {
    base = poly_loader.createClassInstance("cordial_plugins_/base_plugin");
    base->initialize();
    base->viseme_as_init();
    ROS_INFO("Base Name: %s", base->robot_name().c_str());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  return 0;
}

