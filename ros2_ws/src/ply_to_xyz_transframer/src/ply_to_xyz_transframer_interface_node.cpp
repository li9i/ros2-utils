/**
 * @file ply_to_xyz_transframer_interface_node.hpp
 *
 * @author [Alexandros Philotheou] - alefilot@auth.gr
 * @version 0.1
 * @date 2025-07
 *
 * @copyright Copyright (c) 2025 - Alexandros Philotheou. All rights reserved.
 *
 * @brief Instantiates the interface class to be used for triggering the
 *        functionality the node provides
 */
#include "ply_to_xyz_transframer/ply_to_xyz_transframer_interface.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<TransframerInterface>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
