#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dragon_msgs/IKAction.h>

class IKActionServer
{
  protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<dragon_msgs::IKAction> as_;
    dragon_msgs::IKFeedback feedback_;
    dragon_msgs::IKResult result_;
    std::string action_name_;
    ros::Publisher goal_pub_;

  public:
    IKActionServer(std::string name) :
      as_(nh_, name, boost::bind(&IKActionServer::executeCB, this, _1), false),
      action_name_(name)
    {
      goal_pub_ = nh_.advertise<dragon_msgs::IKGoal>("/dragonbot_ik", 1000);
      as_.start();
    }

    ~IKActionServer(void)
    {
    }

    void executeCB(const dragon_msgs::IKGoalConstPtr &goal)
    {
      goal_pub_.publish(goal);
      as_.setSucceeded(result_);
    }
};
    int main( int argc, char** argv)
    {
      ros::init(argc, argv, "IK_Server");

      IKActionServer ik(ros::this_node::getName());
      ros::spin();
      return 0;
    }

