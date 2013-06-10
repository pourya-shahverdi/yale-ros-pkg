#ifndef CORDIAL_BASE__CORDIAL_BASE_H_
#define CORDIAL_BASE__CORDIAL_BASE_H_

#include <string>
#include <boost/current_function.hpp>
#include <ros/console.h>

#include <dragon_msgs/VisemeAction.h>

namespace cordial_base
{
  class CordialBase
  {
    protected:


    public:
      virtual void initialize() = 0;
      virtual std::string robot_name() = 0;

      virtual bool viseme_init() { ROS_WARN( "%s: not implemented", BOOST_CURRENT_FUNCTION ); return false; }
      virtual void viseme_execute(const dragon_msgs::VisemeGoalConstPtr &goal) { ROS_WARN( "%s: not implemented", BOOST_CURRENT_FUNCTION ); }


      virtual ~CordialBase(){}

    protected:
      CordialBase(){}
  };
}
#endif

