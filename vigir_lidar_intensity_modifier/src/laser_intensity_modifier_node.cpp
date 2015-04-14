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


class LaserIntensityModifier
{
public:

  LaserIntensityModifier()
  {
    ros::NodeHandle nh_;

    scan_sub_ = nh_.subscribe("scan", 10, &LaserIntensityModifier::scanCallback, this);
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_out",10,false);

    ros::NodeHandle pnh_("~");

    double desired_intensity;
    pnh_.param("desired_intensity", desired_intensity, 5000.0);

    desired_intensity_ = static_cast<float>(desired_intensity);
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    scan_out_ = *scan_in;
    
    size_t size = scan_in->intensities.size();
    
    for (size_t i = 0; i < size; ++i)
      scan_out_.intensities[i] = desired_intensity_;
    
    scan_pub_.publish(scan_out_);
  }

protected:
  ros::Subscriber scan_sub_;
  ros::Publisher scan_pub_;

  float desired_intensity_;
  
  sensor_msgs::LaserScan scan_out_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vigir_laser_intensity_modifier_node");

  LaserIntensityModifier ls;

  ros::spin();
}
