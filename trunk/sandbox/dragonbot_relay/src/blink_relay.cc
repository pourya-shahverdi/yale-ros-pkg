#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dragon_msgs/BlinkAction.h>

class BlinkActionServer
{
  protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<dragon_msgs::BlinkAction> as_;
    dragon_msgs::BlinkFeedback feedback_;
    dragon_msgs::BlinkResult result_;
    std::string action_name_;
    ros::Publisher goal_pub_;

  public:
    BlinkActionServer(std::string name) :
      as_(nh_, name, boost::bind(&BlinkActionServer::executeCB, this, _1), false),
      action_name_(name)
    {
      goal_pub_ = nh_.advertise<dragon_msgs::BlinkGoal>("/dragonbot_blink", 1000);
      as_.start();
    }

    ~BlinkActionServer(void)
    {
    }

    void executeCB(const dragon_msgs::BlinkGoalConstPtr &goal)
    {
      goal_pub_.publish(goal);
      as_.setSucceeded(result_);
    }
};
    int main( int argc, char** argv)
    {
      ros::init(argc, argv, "Blink_Server");

      BlinkActionServer blink(ros::this_node::getName());
      ros::spin();
      return 0;
    }

