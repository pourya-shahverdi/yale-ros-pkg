#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dragon_msgs/ExpressionMotionAction.h>
#include <dragon_msgs/DragonbotStatus.h>

class ExpressionActionServer
{
  protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<dragon_msgs::ExpressionMotionAction> as_;
    dragon_msgs::ExpressionMotionFeedback feedback_;
    dragon_msgs::ExpressionMotionResult result_;
    std::string action_name_;
    ros::Publisher goal_pub_;
    ros::Subscriber status_sub_;

    std::string expression_;
    std::string des_;

    std::string motion_;

  public:
    ExpressionActionServer(std::string name) :
      as_(nh_, name, boost::bind(&ExpressionActionServer::executeCB, this, _1), false),
      action_name_(name)
    {
      goal_pub_ = nh_.advertise<dragon_msgs::ExpressionMotionGoal>("/dragonbot_expression", 1000);
      status_sub_ = nh_.subscribe(std::string("/dragon_status"), 1000, &ExpressionActionServer::statusCallback, this );
      as_.start();
    }

    ~ExpressionActionServer(void)
    {
    }

    void statusCallback( const dragon_msgs::DragonbotStatusConstPtr& status )
    {
      expression_ = status->expression;
      motion_ = status->motion;
    }

    void executeCB(const dragon_msgs::ExpressionMotionGoalConstPtr &goal)
    {
      goal_pub_.publish(goal);
      ros::spinOnce();


      ROS_INFO( "motion: %s", motion_.c_str() );
      while( (goal->type == "expression" && expression_ == "IDLE") || (goal->type == "motion" && motion_ == "IDLE" ) )
      {
        /*// check for preempts
        if( as_.isPreemptRequested() || !ros::ok() )
        {
          ROS_INFO( "expression preempted" );
          as_.setPreempted();
          return;
	  }*/

        ros::Duration( 0.33 ).sleep();
        ros::spinOnce();
      }
       
      std::string cur = std::string("");
      if( goal->type == "expression" ) cur = expression_;
      else if( goal->type == "motion" ) cur = motion_;

      while( (goal->type == "expression" && expression_ == cur) || (goal->type == "motion" && motion_ == cur ) )
      {
        /*// check for preempts
        if( as_.isPreemptRequested() || !ros::ok() )
        {
          ROS_INFO( "expression preempted" );
          as_.setPreempted();
          return;
	  }*/
        ros::Duration( 0.33 ).sleep();
        ros::spinOnce();
	}

      as_.setSucceeded(result_);
    }
};
    int main( int argc, char** argv)
    {
      ros::init(argc, argv, "Expression_Server");

      ExpressionActionServer expression(ros::this_node::getName());
      ros::spin();
      return 0;
    }

