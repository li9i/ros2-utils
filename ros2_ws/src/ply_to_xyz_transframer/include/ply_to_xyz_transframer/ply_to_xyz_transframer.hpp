#pragma once

#include <filesystem>
#include <regex>
#include <fstream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <yaml-cpp/yaml.h>

class Transframer
{
  public:
    explicit Transframer(std::shared_ptr<rclcpp::Node> node);
    void run();

  private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string input_directory_;
    std::string output_directory_;
    std::string input_file_prefix_;
    std::string input_file_pattern_;
    std::string output_name_pattern_;
    std::string source_frame_;
    std::string target_frame_;
    std::string input_file_id_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<std::string> find_matching_files() const;
    std::string generate_output_filename(const std::string& input_path) const;
    void load_params(std::shared_ptr<rclcpp::Node> node);
    pcl::PointCloud<pcl::PointXYZ>::Ptr load_ply(const std::string& file);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void save_xyz(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& file);
    std::string transform_filename(const std::string& input_filename) const;
};
