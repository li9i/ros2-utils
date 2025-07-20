/**
 * @file static_tf_republisher.hpp
 *
 * @author [Alexandros Philotheou] - alefilot@auth.gr
 * @version 0.1
 * @date 2025-07
 *
 * @copyright Copyright (c) 2025 - Alexandros Philotheou. All rights reserved.
 *
 * @brief Republishes select static transforms. Why? Because ROS 1 cannot
 *        look up static transforms published in ROS 2
 */
#include "static_tf_republisher/static_tf_republisher.hpp"

namespace static_tf_republisher {

/*****************************************************************************
*/
StaticTFRepublisher::StaticTFRepublisher()
  : Node("tf_republisher_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  static_broadcaster_(this)
{
  // -------------------------------------------------------------------------
  this->declare_parameter<std::string>("frames_yaml", "config/frames.yaml");
  auto yaml_path = this->get_parameter("frames_yaml").as_string();

  RCLCPP_INFO(this->get_logger(), "Loading frames from YAML: %s", yaml_path.c_str());
  load_frame_pairs(yaml_path);
  // -------------------------------------------------------------------------
  double publish_rate = this->declare_parameter("publish_rate", 10.0);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
    std::bind(&StaticTFRepublisher::publish_transforms, this));
  // -------------------------------------------------------------------------
}

/*****************************************************************************
*/
void StaticTFRepublisher::load_frame_pairs(const std::string& yaml_path)
{
  try
  {
    YAML::Node config = YAML::LoadFile(yaml_path);
    if (!config["frames"])
    {
      throw std::runtime_error("YAML file missing 'frames' key");
    }
    for (const auto& pair : config["frames"])
    {
      std::string parent = pair["parent"].as<std::string>();
      std::string child = pair["child"].as<std::string>();
      frame_pairs_.emplace_back(parent, child);
      RCLCPP_INFO(this->get_logger(), "Added frame pair: '%s' -> '%s'", parent.c_str(), child.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "---");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: '%s'", e.what());
  }
}

/*****************************************************************************
*/
void StaticTFRepublisher::publish_transforms()
{
  for (const auto& pair : frame_pairs_)
  {
    const auto& parent = pair.first;
    const auto& child = pair.second;
    try
    {
      auto transform = tf_buffer_.lookupTransform(parent, child, tf2::TimePointZero);
      static_broadcaster_.sendTransform(transform);
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Could not transform '%s' -> '%s': '%s'", parent.c_str(), child.c_str(), ex.what());
    }
  }
}
}  // namespace static_tf_republisher
