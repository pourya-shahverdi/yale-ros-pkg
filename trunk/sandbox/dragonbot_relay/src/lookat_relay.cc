#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dragon_msgs/LookatAction.h>

class LookatActionServer
{
  protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<dragon_msgs::LookatAction> as_;
    dragon_msgs::LookatFeedback feedback_;
    dragon_msgs::LookatResult result_;
    std::string action_name_;
    ros::Publisher goal_pub_;

  public:
    LookatActionServer(std::string name) :
      as_(nh_, name, boost::bind(&LookatActionServer::executeCB, this, _1), false),
      action_name_(name)
    {
      goal_pub_ = nh_.advertise<dragon_msgs::LookatGoal>("/dragonbot_lookat", 1000);
      as_.start();
    }

    ~LookatActionServer(void)
    {
    }

    void executeCB(const dragon_msgs::LookatGoalConstPtr &goal)
    {
      goal_pub_.publish(goal);
      as_.setSucceeded(result_);
    }
};
    int main( int argc, char** argv)
    {
      ros::init(argc, argv, "Lookat_Server");

      LookatActionServer lookat(ros::this_node::getName());
      ros::spin();
      return 0;
    }

