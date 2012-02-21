#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <kdl/frames.hpp>

KDL::Vector left_hand;
KDL::Vector left_elbow;
KDL::Vector left_shoulder;
KDL::Vector right_hand;
KDL::Vector right_elbow;
KDL::Vector right_shoulder;

tf::TransformListener* tl;
std::string global_frame;
std::string suffix;
ros::Time time_;
double rer, rey, rsr, rsp;

void set_vector( tf::StampedTransform t, KDL::Vector& v )
{
  v.x(t.getOrigin().x());
  v.y(t.getOrigin().y());
  v.z(t.getOrigin().z());
}

bool get_poses()
{
  try{
    tf::StampedTransform stf;
    
    tl->lookupTransform( global_frame, "LShoulderPitch_link"+suffix, ros::Time(0), stf );
    set_vector( stf, left_shoulder );
    tl->lookupTransform( global_frame, "LElbowRoll_link"+suffix, ros::Time(0), stf );
    set_vector( stf, left_elbow );
    tl->lookupTransform( global_frame, "LWristYaw_link"+suffix, ros::Time(0), stf );
    set_vector( stf, left_hand );

    tl->lookupTransform( global_frame, "RShoulderPitch_link"+suffix, ros::Time(0), stf );
    set_vector( stf, right_shoulder );
    tl->lookupTransform( global_frame, "RElbowRoll_link"+suffix, ros::Time(0), stf );
    set_vector( stf, right_elbow );
    tl->lookupTransform( global_frame, "RWristYaw_link"+suffix, ros::Time(0), stf );
    time_ = stf.stamp_;
  }
  catch( tf::TransformException &ex )
  {
    ROS_WARN( "could not do transform: [%s]", ex.what() );
    return false;
  }

  return true;
}

void js_cb( const sensor_msgs::JointStateConstPtr& js )
{
  for( unsigned int i = 0; i < js->name.size(); i++ )
  {
    if( js->name[i] == "RElbowRoll" ) rer = js->position[i];
    if( js->name[i] == "RElbowYaw" ) rey = js->position[i];
    if( js->name[i] == "RShoulderRoll" ) rsr = js->position[i];
    if( js->name[i] == "RShoulderPitch" ) rsp = js->position[i];
  }
}


int main( int argc, char* argv [] )
{
  ros::init( argc, argv, "joint_align" );
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  ros::Subscriber joints_sub = nh.subscribe( "joint_states", 1, js_cb );
  tl = new tf::TransformListener();
  global_frame = "base_link";
  suffix = "";

  while( ros::ok() )
  {
    if( !get_poses() )
    {
      loop_rate.sleep();
      ros::spinOnce();
      continue;
    }

    // right elbow roll
    KDL::Vector right_elbow_hand(right_hand - right_elbow);
    KDL::Vector right_elbow_shoulder(right_shoulder - right_elbow);

    printf( "reh: %0.2f %0.2f %0.2f\n", right_elbow_hand.x(), right_elbow_hand.y(), right_elbow_hand.z() );
    printf( "res: %0.2f %0.2f %0.2f\n", right_elbow_shoulder.x(), right_elbow_shoulder.y(), right_elbow_shoulder.z() );

    right_elbow_hand.Normalize();
    right_elbow_shoulder.Normalize();
    double right_elbow_angle_roll = 0;
    //if (joint_position_right_hand.fConfidence >= 0.5 &&
    //    joint_position_right_elbow.fConfidence >= 0.5 &&
    //    joint_position_right_shoulder.fConfidence >= 0.5)
    {
      right_elbow_angle_roll = acos(KDL::dot(right_elbow_hand, right_elbow_shoulder));
      right_elbow_angle_roll = right_elbow_angle_roll;
    }

    // right shoulder roll
    KDL::Vector right_shoulder_elbow(right_elbow - right_shoulder);
    KDL::Vector right_shoulder_neck(left_shoulder - right_shoulder);
    right_shoulder_elbow.Normalize();
    right_shoulder_neck.Normalize();
    double right_shoulder_angle_roll = 0;
    //if (joint_position_neck.fConfidence >= 0.5 &&
    //    joint_position_right_elbow.fConfidence >= 0.5 &&
    //    joint_position_right_shoulder.fConfidence >= 0.5)
    {
      right_shoulder_angle_roll = acos(KDL::dot(right_shoulder_elbow, right_shoulder_neck));
      right_shoulder_angle_roll = right_shoulder_angle_roll;
    }

    // right shoulder pitch
    double right_shoulder_angle_pitch = 0;
    //if (joint_position_right_shoulder.fConfidence >= 0.5)
    {
      right_shoulder_angle_pitch = asin(right_shoulder_elbow.y());
      right_shoulder_angle_pitch = right_shoulder_angle_pitch;
    }

    // right shoulder yaw
    double right_shoulder_angle_yaw = 0;
    //if (joint_position_right_shoulder.fConfidence >= 0.5)
    {
      right_shoulder_angle_yaw = asin(right_elbow_hand.x());  // left_shoulder_elbow.x()
      right_shoulder_angle_yaw = right_shoulder_angle_yaw;
    }

    ROS_INFO( "(true/est) %0.2f/%0.2f %0.2f/%0.2f %0.2f/%0.2f %0.2f/%0.2f", rer, right_shoulder_angle_roll, rey, right_shoulder_angle_pitch, rsr, right_shoulder_angle_yaw, rsp, right_elbow_angle_roll);

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;  
}

