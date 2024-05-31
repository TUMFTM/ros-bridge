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
#pragma once

#include "rclcpp/rclcpp.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <rclcpp/time_source.hpp>

class RawPclRecorder : public rclcpp::Node
{
public:

  RawPclRecorder();

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
  tf2_ros::TransformListener *tfListener;
  static constexpr const char* fixed_frame_ = "map";
  std::string dataPath;

};
