#ifndef CORDIAL_BASE__CORDIAL_BASE_H_
#define CORDIAL_BASE__CORDIAL_BASE_H_

#include <string>
#include <boost/current_function.hpp>
#include <ros/console.h>

#include <dragon_msgs/VisemeAction.h>
#include <actionlib/server/simple_action_server.h>

namespace cordial_base
{
  class CordialBase
  {
    protected:
      actionlib::SimpleActionServer<dragon_msgs::VisemeAction> * viseme_as;
      ros::NodeHandle nh_;

    public:
      virtual void initialize() = 0;
      virtual std::string robot_name() = 0;

      void init_actionservers(ros::NodeHandle nh)
      {
        nh_ = nh;
        viseme_init();
        viseme_as = new actionlib::SimpleActionServer<dragon_msgs::VisemeAction>(nh, "Viseme_Server", boost::bind(&cordial_base::CordialBase::viseme_execute, this, _1), false);
        viseme_as->start();

      }

      virtual bool viseme_init() { ROS_WARN( "%s: not implemented", BOOST_CURRENT_FUNCTION ); return false; }
      virtual void viseme_execute(const dragon_msgs::VisemeGoalConstPtr &goal) { ROS_WARN( "%s: not implemented", BOOST_CURRENT_FUNCTION ); }


      virtual ~CordialBase(){}

    protected:
      CordialBase(){}
  };
}
#endif

