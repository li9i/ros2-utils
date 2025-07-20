/**
 * @file static_tf_republisher.hpp
 *
 * @author [Alexandros Philotheou] - alefilot@auth.gr
 * @version 0.1
 * @date 2025-05-05
 *
 * @copyright Copyright (c) 2025 - Alexandros Philotheou. All rights reserved.
 *
 * @brief Republishes select static transforms. Why? Because ROS 1 cannot
 *        look up static transforms published in ROS 2
 */
#ifndef STATIC_TF_REPUBLISHER__STATIC_TF_REPUBLISHER_HPP_
#define STATIC_TF_REPUBLISHER__STATIC_TF_REPUBLISHER_HPP_

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace static_tf_republisher {

class StaticTFRepublisher : public rclcpp::Node
{
  public:
    StaticTFRepublisher();

  private:
    void load_frame_pairs(const std::string& yaml_path);
    void publish_transforms();

    std::vector<std::pair<std::string, std::string>> frame_pairs_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace static_tf_republisher

#endif  // STATIC_TF_REPUBLISHER__STATIC_TF_REPUBLISHER_HPP_
