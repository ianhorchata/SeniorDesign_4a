/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * ROS 2 port of point_cloud_converter/converter.cpp (rusu).
 */

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

using sensor_msgs::msg::PointCloud;
using sensor_msgs::msg::PointCloud2;

class PointCloudConverter : public rclcpp::Node
{
public:
  PointCloudConverter()
  : Node("point_cloud_converter")
  {
    const auto qos = rclcpp::SensorDataQoS();

    sub_points_ = create_subscription<PointCloud>(
      "points_in", qos,
      std::bind(&PointCloudConverter::cloud_cb_points, this, std::placeholders::_1));
    sub_points2_ = create_subscription<PointCloud2>(
      "points2_in", qos,
      std::bind(&PointCloudConverter::cloud_cb_points2, this, std::placeholders::_1));
    pub_points_ = create_publisher<PointCloud>("points_out", qos);
    pub_points2_ = create_publisher<PointCloud2>("points2_out", qos);

    RCLCPP_INFO(
      get_logger(), "PointCloudConverter initialized to transform from PointCloud (%s) to PointCloud2 (%s).",
      sub_points_->get_topic_name(), pub_points2_->get_topic_name());
    RCLCPP_INFO(
      get_logger(), "PointCloudConverter initialized to transform from PointCloud2 (%s) to PointCloud (%s).",
      sub_points2_->get_topic_name(), pub_points_->get_topic_name());
  }

private:
  static std::string getFieldsList(const PointCloud2 & cloud)
  {
    std::string result;
    for (size_t i = 0; i < cloud.fields.size(); ++i) {
      result += (i ? " " : "") + cloud.fields[i].name;
    }
    return result;
  }

  static std::string getFieldsList(const PointCloud & points)
  {
    std::string result = "x y z";
    for (const auto & channel : points.channels) {
      result += " " + channel.name;
    }
    return result;
  }

  void cloud_cb_points2(const PointCloud2::ConstSharedPtr msg)
  {
    if (pub_points_->get_subscription_count() == 0) {
      return;
    }

    RCLCPP_DEBUG(
      get_logger(), "Got a PointCloud2 with %u points (%s) on %s.",
      msg->width * msg->height, getFieldsList(*msg).c_str(), sub_points2_->get_topic_name());

    PointCloud output;
    if (!sensor_msgs::convertPointCloud2ToPointCloud(*msg, output)) {
      RCLCPP_ERROR(get_logger(), "Conversion from PointCloud2 to PointCloud failed!");
      return;
    }
    pub_points_->publish(output);
  }

  void cloud_cb_points(const PointCloud::ConstSharedPtr msg)
  {
    if (pub_points2_->get_subscription_count() == 0) {
      return;
    }

    RCLCPP_DEBUG(
      get_logger(), "Got a PointCloud with %zu points (%s) on %s.",
      msg->points.size(), getFieldsList(*msg).c_str(), sub_points_->get_topic_name());

    PointCloud2 output;
    if (!sensor_msgs::convertPointCloudToPointCloud2(*msg, output)) {
      RCLCPP_ERROR(get_logger(), "Conversion from PointCloud to PointCloud2 failed!");
      return;
    }
    pub_points2_->publish(output);
  }

  rclcpp::Subscription<PointCloud>::SharedPtr sub_points_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_points2_;
  rclcpp::Publisher<PointCloud>::SharedPtr pub_points_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_points2_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudConverter>());
  rclcpp::shutdown();
  return 0;
}
