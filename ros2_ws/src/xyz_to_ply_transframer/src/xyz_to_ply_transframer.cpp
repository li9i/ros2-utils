/**
 * @file xyz_to_ply_transframer.hpp
 *
 * @author [Alexandros Philotheou] - alefilot@auth.gr
 * @version 0.1
 * @date 2025-07
 *
 * @copyright Copyright (c) 2025 - Alexandros Philotheou. All rights reserved.
 *
 * @brief Transforms a .xyz file with content expressed in one frame
 *                to a .ply file with content expressed in another
 */
#include "xyz_to_ply_transframer/xyz_to_ply_transframer.hpp"

/*******************************************************************************
*/
Transframer::Transframer(std::shared_ptr<rclcpp::Node> node) : node_(node),
  input_file_id_(std::string(""))
{
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

/*******************************************************************************
 * @brief If `input_file_id_` is set (i.e. not equal to its
 * default---empty---value) then find and load the file specified by
 * `file_name`. Else check files based on regex `input_file_pattern_`.
 * `input_file_id_` is set by parameter `input_file_id`, which may be set from
 * BT node `SetParam`. This is designed in this ways so that we don't have
 * to use a (custom) service (robetarme_ros2_interfaces/SetString), which
 * would require us cloning the repo here and ... etc
*/
std::vector<std::string>
Transframer::find_matching_files() const
{
  std::vector<std::string> matched_files;

  if (input_file_id_.compare("") != 0)
  {
    // The name of the file to be transframed
    std::string file_name = input_file_prefix_ + "_" + input_file_id_ + ".xyz";
    try
    {
      for (const auto& entry : std::filesystem::directory_iterator(input_directory_))
      {
        if (entry.is_regular_file() &&
            entry.path().filename().string().compare(file_name) == 0)
        {
          matched_files.push_back(entry.path().string());
        }
      }
    } catch (const std::filesystem::filesystem_error& e) {
      RCLCPP_ERROR(node_->get_logger(), "File system error: %s", e.what());
    }
  }
  else
  {
    std::regex pattern(input_file_pattern_);

    try {
      for (const auto& entry : std::filesystem::directory_iterator(input_directory_))
      {
        if (entry.is_regular_file() && std::regex_match(entry.path().filename().string(), pattern))
          matched_files.push_back(entry.path().string());
      }
    } catch (const std::filesystem::filesystem_error& e) {
      RCLCPP_ERROR(node_->get_logger(), "File system error: %s", e.what());
    }
  }

  return matched_files;
}

/*******************************************************************************
*/
std::string
Transframer::generate_output_filename(const std::string& input_path) const
{
  std::string transformed_name = transform_filename(input_path);
  std::filesystem::path output_file =
    std::filesystem::path(output_directory_) / transformed_name;
  return output_file.string();
}

/*******************************************************************************
*/
  void
Transframer::load_params(std::shared_ptr<rclcpp::Node> node)
{
  // io directories
  input_directory_ =   node_->get_parameter("input_directory").as_string();
  output_directory_ =  node_->get_parameter("output_directory").as_string();
  input_file_prefix_ = node_->get_parameter("input_file_prefix").as_string();
  input_file_id_ =     node_->get_parameter("input_file_id").as_string();

  /*  ------------------------------------
   * - input_1.xyz → input_1.ply
       input_file_pattern: "(.+)\\.xyz"
       output_name_pattern: ""
       -----------------------------------
     - scan_42.xyz → pointcloud_42.ply
       input_file_pattern: "scan_(\\d+)\\.xyz"
       output_name_pattern: "pointcloud_$1"
       -----------------------------------
     -  object_42_scan.xyz → scan_42_processed.ply
       input_file_pattern: "(\\w+)_(\\d+)_(\\w+)\\.xyz"
       output_name_pattern: "$3_$2_processed"
      -----------------------------------
  */
  // Default: all .xyz files
  input_file_pattern_ =   node_->get_parameter("input_file_pattern").as_string();
  output_name_pattern_ =  node_->get_parameter("output_name_pattern").as_string();

  // io frames
  source_frame_ =         node_->get_parameter("source_frame").as_string();
  target_frame_ =         node_->get_parameter("target_frame").as_string();

  // Validate directories exist
  if (!std::filesystem::exists(input_directory_))
    RCLCPP_ERROR(node_->get_logger(), "Input directory does not exist: '%s'",
      input_directory_.c_str());
  if (!std::filesystem::exists(output_directory_))
  {
    RCLCPP_INFO(node_->get_logger(), "Creating output directory: '%s'",
      output_directory_.c_str());
    std::filesystem::create_directories(output_directory_);
  }
}

/*******************************************************************************
*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr
Transframer::load_xyz(const std::string &file)
{
  RCLCPP_INFO(node_->get_logger(), "Loading xyz file from: '%s'", file.c_str());

  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::ifstream infile(file);

  if (!infile.is_open())
    RCLCPP_ERROR(node_->get_logger(), "Failed to open file: '%s'", file.c_str());

  std::string line;

  while (std::getline(infile, line))
  {
    std::stringstream ss(line);
    float x, y, z;
    ss >> x >> y >> z;
    cloud->points.emplace_back(x, y, z);
  }

  if (cloud->points.empty())
    RCLCPP_WARN(node_->get_logger(), "Loaded empty point cloud");

  cloud->width = cloud->points.size();
  cloud->height = 1;
  return cloud;
}

/*******************************************************************************
*/
void Transframer::run()
{
  RCLCPP_INFO(node_->get_logger(), "Loading params...");
  load_params(node_);

  RCLCPP_INFO(node_->get_logger(), "Finding matching files...");
  auto input_files = find_matching_files();

  if (input_files.empty())
  {
    RCLCPP_WARN(node_->get_logger(), "No matching files found in directory: '%s'",
      input_directory_.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Found %zu files to process", input_files.size());

  for (const auto& input_file : input_files)
  {
    std::string output_file = generate_output_filename(input_file);

    RCLCPP_INFO(node_->get_logger(), "Processing: '%s' -> '%s'",
      input_file.c_str(), output_file.c_str());

    try
    {
      auto input_cloud = load_xyz(input_file);
      if (input_cloud->empty())
      {
        RCLCPP_WARN(node_->get_logger(), "Empty cloud loaded from '%s', skipping",
          input_file.c_str());
        continue;
      }

      auto transformed = transform_cloud(input_cloud);
      save_ply(transformed, output_file);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to process '%s': %s",
        input_file.c_str(), e.what());
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Finished processing %zu files",
    input_files.size());
}

/*******************************************************************************
*/
  void
Transframer::save_ply(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
  const std::string &file)
{
  pcl::io::savePLYFileBinary(file, *cloud);

  if (pcl::io::savePLYFileBinary(file, *cloud) != 0)
    RCLCPP_ERROR(node_->get_logger(), "Failed to save PLY file");
  else
    RCLCPP_INFO(node_->get_logger(), "Saved PLY file to: '%s'", file.c_str());
}


/*******************************************************************************
*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr
Transframer::transform_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  for (const auto &pt : cloud->points)
  {
    geometry_msgs::msg::PointStamped in_pt, out_pt;
    in_pt.header.stamp = node_->now();
    in_pt.header.frame_id = source_frame_;
    in_pt.point.x = pt.x;
    in_pt.point.y = pt.y;
    in_pt.point.z = pt.z;

    try
    {
      out_pt = tf_buffer_->transform(in_pt, target_frame_,
        tf2::Duration(std::chrono::seconds(2)));
      transformed_cloud->points.emplace_back(out_pt.point.x, out_pt.point.y, out_pt.point.z);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(), "TF transform failed: %s", ex.what());
    }
  }

  if (transformed_cloud->points.empty())
    RCLCPP_ERROR(node_->get_logger(), "All transform operations failed");

  transformed_cloud->width = transformed_cloud->points.size();
  transformed_cloud->height = 1;
  return transformed_cloud;
}

/*******************************************************************************
*/
std::string
Transframer::transform_filename(const std::string& input_filename) const
{
  if (output_name_pattern_.empty())
  {
    // Default: just change extension to .ply
    std::filesystem::path p(input_filename);
    return p.stem().string() + ".ply";
  }

  try
  {
    std::regex pattern(input_file_pattern_);
    std::smatch matches;
    std::string filename = std::filesystem::path(input_filename).filename().string();

    if (std::regex_match(filename, matches, pattern))
    {
      // Replace each capture group reference ($1, $2, etc.) in the output pattern
      std::string result = output_name_pattern_;
      for (size_t i = 1; i < matches.size(); ++i)
      {
        std::string token = "$" + std::to_string(i);
        size_t pos = result.find(token);
        if (pos != std::string::npos)
          result.replace(pos, token.length(), matches[i].str());
      }
      return result + ".ply";
    }
  } catch (const std::regex_error& e) {
    RCLCPP_ERROR(node_->get_logger(), "Regex error in output pattern: %s", e.what());
  }

  // Fallback: use default bif pattern matching fails
  std::filesystem::path p(input_filename);
  return p.stem().string() + ".ply";
}
