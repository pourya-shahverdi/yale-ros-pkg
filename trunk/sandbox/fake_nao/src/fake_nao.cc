#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nao_msgs/JointAnglesWithSpeed.h>


ros::Publisher joint_state_publisher;
std::vector<std::string> joint_names;
std::vector<double> joint_pos;
std::vector<double> zeros;

void add_joint( std::string name, float min_angle, float max_angle, float pos )
{
  joint_names.push_back( name );
  joint_pos.push_back(pos);
  zeros.push_back(0);
}

void js_cb( const nao_msgs::JointAnglesWithSpeedConstPtr& js )
{
  for( unsigned int i = 0; i < js->joint_names.size(); i++ )
  {
    for( unsigned int j = 0; j < joint_names.size(); j++ )
    {
      if( js->joint_names[i] == joint_names[j] )
      {
        joint_pos[j] = js->joint_angles[i];
      }
    }
  }
}

int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "fake_nao" );
  ros::NodeHandle nh;
  ros::Subscriber joints_sub = nh.subscribe( "joint_angles", 1, js_cb );
  joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
  ros::Rate loop_rate(10);
  
  add_joint( "HeadYaw", 0, 0, 0 );
  add_joint( "HeadPitch", 0, 0, 0 );
  add_joint( "LShoulderPitch", 0, 0, 0 );
  add_joint( "LShoulderRoll", 0, 0, 0 );
  add_joint( "LElbowRoll", 0, 0, 0 );
  add_joint( "LElbowYaw", 0, 0, 0 );
  add_joint( "LWristYaw", 0, 0, 0 );
  add_joint( "RShoulderPitch", 0, 0, 0 );
  add_joint( "RShoulderRoll", 0, 0, 0 );
  add_joint( "RElbowRoll", 0, 0, 0 );
  add_joint( "RElbowYaw", 0, 0, 0 );
  add_joint( "RWristYaw", 0, 0, 0 );

  add_joint( "RHipRoll", 0, 0, 0 );
  add_joint( "RHipPitch", 0, 0, 0 );
  add_joint( "RAnklePitch", 0, 0, 0 );
  add_joint( "RAnkleRoll", 0, 0, 0 );
  add_joint( "RKneePitch", 0, 0, 0 );
  add_joint( "LHipRoll", 0, 0, 0 );
  add_joint( "LHipPitch", 0, 0, 0 );
  add_joint( "LAnklePitch", 0, 0, 0 );
  add_joint( "LAnkleRoll", 0, 0, 0 );
  add_joint( "LKneePitch", 0, 0, 0 );


  sensor_msgs::JointState js;
  js.header.frame_id = "/nao_torso_link";
  js.name = joint_names;
  js.velocity = zeros;
  js.effort = zeros;

  while( ros::ok() )
  {
    js.header.stamp = ros::Time::now();
    js.position = joint_pos;
    joint_state_publisher.publish(js);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

