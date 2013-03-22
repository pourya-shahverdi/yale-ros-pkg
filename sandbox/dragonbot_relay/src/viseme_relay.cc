#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dragon_msgs/VisemeAction.h>

class VisemeActionServer
{
  protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<dragon_msgs::VisemeAction> as_;
    dragon_msgs::VisemeFeedback feedback_;
    dragon_msgs::VisemeResult result_;
    std::string action_name_;
    ros::Publisher goal_pub_;

  public:
    VisemeActionServer(std::string name) :
      as_(nh_, name, boost::bind(&VisemeActionServer::executeCB, this, _1), false),
      action_name_(name)
    {
      goal_pub_ = nh_.advertise<dragon_msgs::VisemeGoal>("/dragonbot_viseme", 1000);
      as_.start();
    }

    ~VisemeActionServer(void)
    {
    }

    void executeCB(const dragon_msgs::VisemeGoalConstPtr &goal)
    {
      goal_pub_.publish(goal);
      as_.setSucceeded(result_);
    }
};
    int main( int argc, char** argv)
    {
      ros::init(argc, argv, "Viseme_Server");

      VisemeActionServer viseme(ros::this_node::getName());
      ros::spin();
      return 0;
    }

