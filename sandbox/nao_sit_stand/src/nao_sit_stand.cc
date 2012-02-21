// (C) David Feil-Seifer 2012

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <nao_msgs/BodyPoseAction.h>
#include <nao_msgs/BodyPoseActionGoal.h>
#include <actionlib/client/simple_action_client.h>

int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "ros_sit_stand" );
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<nao_msgs::BodyPoseAction> bodyPoseClient("body_pose", true);
  ros::ServiceClient m_stiffnessDisableClient;
  ros::ServiceClient m_stiffnessEnableClient;
  m_stiffnessDisableClient = nh.serviceClient<std_srvs::Empty>("body_stiffness/disable");
  m_stiffnessEnableClient = nh.serviceClient<std_srvs::Empty>("body_stiffness/enable");

  std_srvs::Empty e;
  m_stiffnessEnableClient.call(e);


  ros::spinOnce();
  ros::Duration(5.0,0).sleep();
  ros::spinOnce();
  nao_msgs::BodyPoseGoal goal;
  goal.pose_name="init";
  bodyPoseClient.sendGoalAndWait(goal, ros::Duration(5.0) );
  actionlib::SimpleClientGoalState state = bodyPoseClient.getState();
  if( state != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_ERROR( "%s pose action did not succeed (%s): %s", goal.pose_name.c_str(), state.toString().c_str(), state.text_.c_str());
  }
  else
  {
    ROS_INFO("Pose action succeeded" );
  }

  ros::Duration(5.0,0).sleep();
  goal.pose_name="crouch";
  bodyPoseClient.sendGoalAndWait(goal, ros::Duration(5.0) );
  state = bodyPoseClient.getState();
  if( state != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_ERROR( "%s pose action did not succeed (%s): %s", goal.pose_name.c_str(), state.toString().c_str(), state.text_.c_str());
  }
  else
  {
    ROS_INFO("Pose action succeeded" );
  }
  m_stiffnessDisableClient.call(e);

  ros::spin();
  return 0;
}
