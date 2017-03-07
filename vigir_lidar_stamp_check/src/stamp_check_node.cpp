//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>

class StampChecker
{
public:

  StampChecker()
  {
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_("~");

    scan_sub_ = nh_.subscribe("/spin_laser/pitch_scan", 10, &StampChecker::scanCallback, this);
    joint_state_sub_ = nh_.subscribe("joint_states", 10, &StampChecker::jointStateCallback, this);
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_out",10,false);
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("spin_joint_state",10,false);




    pnh_.param("spin_joint_name_", p_spin_joint_name_, std::string("spin_lidar_spin_joint"));
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    if (!last_scan_.get()){
      last_scan_ = scan_in;
      return;
    }


    double diff = (scan_in->header.stamp - last_scan_->header.stamp).toSec();

    ROS_INFO("Diff between incoming scans was %f seconds", diff);

    last_scan_ = scan_in;
  }

  void jointStateCallback (const sensor_msgs::JointState::ConstPtr& joint_state_in)
  {
    if (joint_state_in->name[0] != p_spin_joint_name_){
      return;
    }

    if (!last_joint_state_.get()){
      last_joint_state_ = joint_state_in;
      return;
    }

    double diff = (joint_state_in->header.stamp - last_joint_state_->header.stamp).toSec();

    ROS_INFO("Diff between incoming spin joint states was %f seconds", diff);

    joint_state_pub_.publish(joint_state_in);

    last_joint_state_ = joint_state_in;
  }

protected:
  ros::Subscriber scan_sub_;
  ros::Publisher scan_pub_;

  ros::Subscriber joint_state_sub_;
  ros::Publisher joint_state_pub_;


  float desired_intensity_;
  
  float scan_stamp_diff_;
  float joint_state_stamp_diff_;
  
  sensor_msgs::LaserScan scan_out_;

  sensor_msgs::LaserScan::ConstPtr last_scan_;
  sensor_msgs::JointState::ConstPtr last_joint_state_;


  std::string p_spin_joint_name_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stamp_check_node");

  StampChecker ls;

  ros::spin();
}
