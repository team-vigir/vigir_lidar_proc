//=================================================================================================
// Copyright (c) 2015, Stefan Kohlbrecher, TU Darmstadt
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

#include <laser_geometry/laser_geometry.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.h>

//#include <filters/filter_chain.h>

class CloudProximityWarner
{
public:

  CloudProximityWarner()
  {
    ros::NodeHandle nh_;

    prior_roll_angle_ = 0.0;

    point_cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_out",10,false);
    prox_warning_pub_ = nh_.advertise<std_msgs::Bool>("proximity_warning",10,false);

    scan_sub_ = nh_.subscribe("cloud", 10, &CloudProximityWarner::cloudCallback, this);


    crop_box_.reset(new pcl::CropBox<pcl::PointXYZ>());

    ros::NodeHandle pnh_("~");

    double x_max, x_min, y_max, y_min, z_max, z_min;
    pnh_.param("x_max", x_max, 1.0);
    pnh_.param("x_min", x_min, 1.0);
    pnh_.param("y_max", y_max, 1.0);
    pnh_.param("y_min", y_min, 1.0);
    pnh_.param("z_max", z_max, 1.0);
    pnh_.param("z_min", z_min, 1.0);

    //filter_chain_.configure("scan_filter_chain", pnh_);

    pnh_.param("target_frame", p_target_frame_, std::string("NO_TARGET_FRAME_SPECIFIED"));



  }

  void cloudCallback (const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
  {

    //ros::Time scan_time = cloud_in->header.stamp;

    if (cloud_in->header.frame_id != p_target_frame_){
        ROS_ERROR_THROTTLE(5.0,"target frame differs from cloud frame, this is not supported at the moment.");
        return;

        /*
        if (!tfl_.get() ){
          tfl_.reset(new tf::TransformListener());
          wait_duration_ = ros::Duration(0.5);
        }
        */

    }

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud_box_filtered;

    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::fromROSMsg(*cloud_in, *cloud);

    crop_box_->setInputCloud(cloud);
    crop_box_->filter(*cloud_box_filtered);

    std_msgs::Bool prox_warning;

    if (cloud_box_filtered->size() > 0){
      prox_warning.data = true;
    }else{
      prox_warning.data = false;
    }

    prox_warning_pub_.publish(prox_warning);

  }

protected:
  ros::Subscriber scan_sub_;
  ros::Publisher point_cloud2_pub_;
  ros::Publisher prox_warning_pub_;

  boost::shared_ptr<tf::TransformListener> tfl_;
  ros::Duration wait_duration_;

  bool p_use_high_fidelity_projection_;
  std::string p_target_frame_;

  double prior_roll_angle_;


  sensor_msgs::PointCloud2 cloud2_;

  std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > > cloud_agg_;

  boost::shared_ptr<pcl::CropBox<pcl::PointXYZ> > crop_box_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vigir_laserscan_to_pointcloud_node");

  CloudProximityWarner ls;

  ros::spin();
}
