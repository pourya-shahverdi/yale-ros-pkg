//#ifndef CORDIAL_DRAGONBOT_H_
//#define CORDIAL_DRAGONBOT_H_

#include <pluginlib/class_list_macros.h>
#include <cordial_base_/cordial_base.h>

namespace cordial_plugin
{
  class CordialDragonbot : public cordial_base::CordialBase
  {
    protected:
      std::string _robot_name;

      /* viseme stuff */
      dragon_msgs::VisemeResult result_;
      ros::Publisher viseme_pub_;

    public:
      CordialDragonbot() {}


      /* default functions, just used to return a robot name */

      void initialize()
      {
        _robot_name = std::string("Dragonbot");
      }

      std::string robot_name() {return _robot_name;}


      /* code to be run BEFORE the viseme actionserver is started */

      bool viseme_init()
      {
        // declare a publisher to send goal information on to dragonbot_pubsub node topic
        viseme_pub_ = nh_.advertise<dragon_msgs::VisemeGoal>("/dragonbot_viseme", 1000);
        return true;
      }

      /* code to be run when an action goal is sent to Viseme_Server */

      void viseme_execute( const dragon_msgs::VisemeGoalConstPtr &goal )
      {
        ROS_INFO( "setting goal: %s", goal->constant.c_str() );
        // just publish goal on to dragonbot_pubsub topic
        viseme_pub_.publish(goal);
        // set action succeeded topic
        viseme_as->setSucceeded(result_);
      }
  };
}

PLUGINLIB_EXPORT_CLASS(cordial_plugin::CordialDragonbot, cordial_base::CordialBase )

//#endif
