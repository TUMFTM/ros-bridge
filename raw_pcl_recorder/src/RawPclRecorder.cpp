/*
* Copyright 2023 Gemb Kaljavesi, Technical University of Munich
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#include <sstream>
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "RawPclRecorder.h"

RawPclRecorder::RawPclRecorder() : Node("raw_pcl_recorder")
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener = new tf2_ros::TransformListener(*tf_buffer_);

  
  this->declare_parameter<std::string>("topic_name", "/sensing/lidar/concatenated/pointcloud");
  std::string topicName;
  this->get_parameter("topic_name", topicName);
  this->declare_parameter<std::string>("data_path", "/tmp/data");
  this->get_parameter("data_path", dataPath);

  if (mkdir(dataPath.c_str(), 0777) == -1) {
    RCLCPP_WARN(this->get_logger(), "Could not create directory %s", dataPath.c_str());
  }

  // Create a ROS subscriber for the input point cloud
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(topicName, rclcpp::QoS{1}.best_effort(), std::bind(&RawPclRecorder::callback, this, std::placeholders::_1), sub_opt);
}

void RawPclRecorder::callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  if ((cloud->width * cloud->height) == 0) {
    return;
  }

  std::stringstream ss;
  ss << dataPath.c_str() << "/" << cloud->header.stamp.sec << cloud->header.stamp.nanosec << ".pcd";


  RCLCPP_INFO (this->get_logger(), "Received %d data points. Storing in %s",
           (int)cloud->width * cloud->height,
           ss.str().c_str());

  Eigen::Affine3d transform;
  try {
    transform = tf2::transformToEigen (tf_buffer_->lookupTransform(fixed_frame_, cloud->header.frame_id,  cloud->header.stamp, rclcpp::Duration::from_seconds(1)));

    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromROSMsg(*cloud, pclCloud);

    pcl::PointCloud<pcl::PointXYZ> transformedCloud;
    pcl::transformPointCloud (pclCloud, transformedCloud, transform);

    pcl::PCDWriter writer;
    writer.writeBinary(ss.str(), transformedCloud);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "Could NOT transform: %s", ex.what());
  }
}
