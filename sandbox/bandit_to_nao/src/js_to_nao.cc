/*
 * bandit_to_nao
 * Copyright (c) 2012, David Feil-Seifer
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the yale-ros-pkgs nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nao_msgs/JointAnglesWithSpeed.h>

ros::Publisher nao_pub_;

void joint_cb( const sensor_msgs::JointStateConstPtr& joint )
{

  nao_msgs::JointAnglesWithSpeed output;
  //output.header = joint->header;

  for( unsigned i = 0; i < joint->name.size(); i++ )
  {
    // for each joint

    // convert name to new name
    output.joint_names.push_back(joint->name[i]);
    output.joint_angles.push_back(joint->position[i]);

/*
    if( joint->name[i] == "elbow_right_roll" )
    {
      output.name.push_back("right_bicep_forearm_joint");
      output.position.push_back(joint->position[i]);
      output.velocity.push_back(0.0);
      output.effort.push_back(0.0);
    }
    if( joint->name[i] == "shoulder_right_roll" )
    {
      output.name.push_back("right_shoulder_mounting_shoulder_joint");
      output.position.push_back(-joint->position[i]);
      output.velocity.push_back(0.0);
      output.effort.push_back(0.0);
    }
    if( joint->name[i] == "shoulder_right_pitch" )
    {
      output.name.push_back("right_torso_shoulder_mounting_joint");
      output.position.push_back(-joint->position[i]);
      output.velocity.push_back(0.0);
      output.effort.push_back(0.0);
    }
    if( joint->name[i] == "shoulder_right_yaw" )
    {
      output.name.push_back("right_shoulder_bicep_joint");
      output.position.push_back(-joint->position[i] - M_PI/2.);
      output.velocity.push_back(0.0);
      output.effort.push_back(0.0);
    }
*/
  }

  output.speed = 0.1;
  nao_pub_.publish(output);

}

int main( int argc, char* argv[] )
{
  ros::init(argc,argv,"js_to_nao");
  ros::NodeHandle nh;

  nao_pub_ = nh.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles",10);
  ros::Subscriber joint_sub = nh.subscribe("output_joint_state",10,joint_cb);

  ros::spin();

  return 0;
}
