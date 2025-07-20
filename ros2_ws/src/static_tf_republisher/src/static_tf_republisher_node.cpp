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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<static_tf_republisher::StaticTFRepublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
