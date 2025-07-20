/**
 * @file xyz_to_ply_transframer_interface.hpp
 *
 * @author [Alexandros Philotheou] - alefilot@auth.gr
 * @version 0.1
 * @date 2025-07
 *
 * @copyright Copyright (c) 2025 - Alexandros Philotheou. All rights reserved.
 *
 * @brief An interface class to be used for triggering the functionality the
 *        node provides
 */
#include "xyz_to_ply_transframer/xyz_to_ply_transframer_interface.hpp"
#include "xyz_to_ply_transframer/xyz_to_ply_transframer.hpp"

/*******************************************************************************
*/
TransframerInterface::TransframerInterface() : Node("xyz_to_ply_transframer_node"),
  state_t0_(state_t0_init_),
  state_t1_(state_t1_init_),
  is_node_alive_(true),
  has_functionality_been_delivered_successfully_(false)
{
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  // Task planner's requirements
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  // Call this service to start functionality
  enable_service_ =
    this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()) + "/execution/enable",
      std::bind(
        &TransframerInterface::handle_request_enable,
        this,
        std::placeholders::_1,
        std::placeholders::_2)
      );

  // Call this service to pause functionality
  disable_pause_service_ =
    this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()) + "/execution/disable/pause",
      std::bind(
        &TransframerInterface::handle_request_disable_pause,
        this,
        std::placeholders::_1,
        std::placeholders::_2)
      );

  // Call this service to stop functionality
  disable_stop_service_ =
    this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()) + "/execution/disable/stop",
      std::bind(
        &TransframerInterface::handle_request_disable_stop,
        this,
        std::placeholders::_1,
        std::placeholders::_2)
      );

  // Create callback group for shutdown and kill service servers
  cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Call this service to shutdown this node
  shutdown_service_ =
    this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()) + "/execution/halt/shutdown_node",
      std::bind(
        &TransframerInterface::handle_request_shutdown,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default,
      cb_group_
      );

  // Call this service to kill this node
  kill_service_ =
    this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()) + "/execution/halt/kill",
      std::bind(
        &TransframerInterface::handle_request_kill,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default,
      cb_group_
      );

  // This service should be called by code in this node in order to signify
  // the end of execution of this node to the task planner that enabled it
  trigger_join_service_client_ptr_ =
    this->create_client<std_srvs::srv::SetBool>(
      std::string(this->get_name()) + "/execution/join");

  // A timer that checks state variable state_t1_ which determines the state of
  // this node
  clock_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&TransframerInterface::clock_callback, this));

  // Does this node need to notify the task planner that its functionality
  // has been served? Or is this node running solo?
  this->declare_parameter("task_planner_notify_end", false);
  task_planner_notify_end_ =
    this->get_parameter("task_planner_notify_end").as_bool();

  // Declare params to be passed to node `xyz_to_ply_transframer` via `execute()`
  declare_params();

  // ---------------------------------------------------------------------------
  // Transitions between states
  // There are 5 available states in States_
  size_t w = 5;
  size_t h = w;

  for (unsigned int i = 0; i < w; i++)
  {
    std::vector<bool> tmp;
    for (unsigned int j = 0; j < h; j++)
      tmp.push_back(false);

    transition_table_.push_back(tmp);
  }

  // transition_table_[PLAY][PLAY]    = false;
  transition_table_[PLAY][PAUSE]      = true;
  transition_table_[PLAY][STOP]       = true;
  transition_table_[PLAY][SHUTDOWN]   = true;
  transition_table_[PLAY][KILL]       = true;
  //----------------------------------------------------
  transition_table_[PAUSE][PLAY]      = true;
  // transition_table_[PAUSE][PAUSE]  = false;
  transition_table_[PAUSE][STOP]      = true;
  transition_table_[PAUSE][SHUTDOWN]  = true;
  transition_table_[PAUSE][KILL]      = true;
  //-----------------------------------------------------
  transition_table_[STOP][PLAY]       = true;
  // transition_table_[STOP][PAUSE]   = false;
  // transition_table_[STOP][STOP]    = false;
  transition_table_[STOP][SHUTDOWN]   = true;
  transition_table_[STOP][KILL]       = true;
  //-----------------------------------------------------
  // transition_table_[SHUTDOWN][*]   = false;
  //-----------------------------------------------------
  // transition_table_[KILL][*]       = false;
  //-----------------------------------------------------
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
}


/*******************************************************************************
 * @brief Executes periodically.
 */
void TransframerInterface::clock_callback()
{
  // ---------------------------------------------------------------------------
  if (state_t0_.load() != state_t1_.load())
  {
    // Transition to state if transition is allowed
    if (transition_table_[state_t0_.load()][state_t1_.load()] == true)
    {
      RCLCPP_INFO(this->get_logger(), "EXECUTING TRANSITION: %d -> %d",
        state_t0_.load(), state_t1_.load());

      // Store new state to old
      state_t0_.store(state_t1_.load());

      // -------------------------------------
      if (state_t1_.load() == PLAY)
      {
        RCLCPP_INFO(this->get_logger(), "Triggering execution PLAY");
        thread_start("xyz_to_ply_transframer_thread");
      }
      // -------------------------------------
      if (state_t1_.load() == PAUSE)
      {
        RCLCPP_INFO(this->get_logger(), "Triggering execution PAUSE");
        thread_stop("xyz_to_ply_transframer_thread");
      }
      // -------------------------------------
      if (state_t1_.load() == STOP)
      {
        RCLCPP_INFO(this->get_logger(), "Triggering execution STOP");
        thread_stop("xyz_to_ply_transframer_thread");

        // Notify that transframing has ended first (otherwise the task planner
        // that called this package cannot join it after it was stopped, which
        // means that it will block)
        if (task_planner_notify_end_ == true)
          task_planner_notify_end();
      }
      // -------------------------------------
      if (state_t1_.load() == SHUTDOWN)
      {
        // Handled in own service callback. Otherwise functions called in this
        // scope might block SHUTDOWN
      }
      // -------------------------------------
      if (state_t1_.load() == KILL)
      {
        // Handled in own service callback. Otherwise functions called in this
        // scope might block KILL
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "TRANSITION: %d -> %d NOT ALLOWED",
        state_t0_.load(), state_t1_.load());

      // Store old state to new
      state_t1_.store(state_t0_.load());
    }

  }
  // ---------------------------------------------------------------------------
}


/*******************************************************************************
 * @brief The final act: calls rclcpp::shutdown()
 */
void TransframerInterface::commit_node_suicide()
{
  RCLCPP_INFO(this->get_logger(), "Triggering node SHUTDOWN");
  sleep(1);
  rclcpp::shutdown();
}


/*******************************************************************************
 * @brief The final act: calls system("kill self pid")
 */
void TransframerInterface::commit_process_suicide()
{
  RCLCPP_INFO(this->get_logger(), "Triggering node KILL");
  sleep(1);

  pid_t pid = getpid();
  std::string s = "kill -9 " + std::to_string(pid);
  int ret = system(s.c_str());
}

/*******************************************************************************
 * @brief Declare params in this node so that `execute()` may access their
 * values.  If declared downstream (xyz_to_ply_transframer [without the word
 * interface]) then `ros2 param list` does not list them
 */
  void
TransframerInterface::declare_params()
{
  // io directories
  this->declare_parameter("input_directory", "");
  this->declare_parameter("output_directory", "");

  this->declare_parameter("input_file_prefix", "");
  this->declare_parameter("input_file_id", "");

  // Default: all .xyz files
  this->declare_parameter("input_file_pattern", ".*\\.xyz");
  this->declare_parameter("output_name_pattern", "");

  // io frames
  this->declare_parameter("source_frame", "world");
  this->declare_parameter("target_frame", "world");
}


/*******************************************************************************
 * @brief Wraps the functionality that this package provides for use with
 * the task planner
 */
void TransframerInterface::entrypoint_wrapper()
{
  execute();

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  // The functionality has finished successfully, or maybe it hasn't
  has_functionality_been_delivered_successfully_ = true;

  // Execution has ended; return to initial state
  state_t1_.store(state_t1_init_);
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
}


/*******************************************************************************
 * @brief Transframing: the functionality that the package offers
 */
void TransframerInterface::execute()
{
  RCLCPP_INFO(this->get_logger(), "STARTING TO TRANSFRAME ...");

  auto transframer = std::make_shared<Transframer>(shared_from_this());
  transframer->run();

  RCLCPP_INFO(this->get_logger(), "DONE TRANSFRAMING");
}


/*******************************************************************************
 * @brief Service server to pause the functionality that this node offers
 */
void TransframerInterface::handle_request_disable_pause(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "Requesting execution PAUSE");
  state_t1_.store(PAUSE);

  res->message = "SUCCESS";
  res->success = true;
}


/*******************************************************************************
 * @brief Service server to stop the functionality that this node offers
 * (pause + return to task planner)
 */
void TransframerInterface::handle_request_disable_stop(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "Requesting execution STOP");
  state_t1_.store(STOP);

  res->message = "SUCCESS";
  res->success = true;
}


/*******************************************************************************
 * @brief Service server to start the functionality that this node offers
 */
void TransframerInterface::handle_request_enable(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "Requesting execution PLAY");
  state_t1_.store(PLAY);

  res->message = "SUCCESS";
  res->success = true;
}


/*******************************************************************************
 * @brief Service server to shutdown the node altogether
 * https://answers.ros.org/question/294069/shutdown-a-node-from-another-node-in-ros-cpp/
 */
void TransframerInterface::handle_request_kill(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  RCLCPP_WARN(this->get_logger(), "Requesting node KILL");

  state_t1_.store(KILL);
  is_node_alive_.store(false);

  res->message = "SUCCESS";
  res->success = true;

  // Notify that transframing has ended unsuccessfully first (otherwise the task
  // planner that called this package cannot join it after it was stopped,
  // which means that it will block)
  if (task_planner_notify_end_ == true)
    task_planner_notify_end();

  // Spin a new thread so that the service has time to respond back
  std::thread t = std::thread(&TransframerInterface::commit_process_suicide, this);
  t.detach();
}


/*******************************************************************************
 * @brief Service server to shutdown the node altogether
 * https://answers.ros.org/question/294069/shutdown-a-node-from-another-node-in-ros-cpp/
 */
void TransframerInterface::handle_request_shutdown(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  RCLCPP_WARN(this->get_logger(), "Requesting node SHUTDOWN");

  state_t1_.store(SHUTDOWN);
  is_node_alive_.store(false);

  res->message = "SUCCESS";
  res->success = true;

  // Notify that transframing has ended unsuccessfully first (otherwise the task
  // planner that called this package cannot join it after it was stopped,
  // which means that it will block)
  if (task_planner_notify_end_ == true)
    task_planner_notify_end();

  // Spin a new thread so that the service has time to respond back
  std::thread t = std::thread(&TransframerInterface::commit_node_suicide, this);
  t.detach();
}

/*******************************************************************************
 * @brief Notifies the task planner that manages this pkg that this node has
 * finished its work (or that it was stopped/shutdown in the process---and its
 * work was forced to finish).
 */
bool TransframerInterface::task_planner_notify_end()
{
  while (!trigger_join_service_client_ptr_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(),
        "Client of service %s interrupted while waiting for service to appear",
        trigger_join_service_client_ptr_->get_service_name());
      return false;
    }
    RCLCPP_WARN(this->get_logger(),
      "Waiting for service %s to appear...",
      trigger_join_service_client_ptr_->get_service_name());
  }
  // service appeared

  // Craft request
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = has_functionality_been_delivered_successfully_;

  // Notify the task planner that this package ended work
  // and continue with execution
  auto result_future =
    trigger_join_service_client_ptr_->async_send_request(request);

  if (!rclcpp::ok())
  {
    RCLCPP_ERROR(this->get_logger(), "Program canceled");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Will now join main task planner caller ...");
  return true;
}


/*******************************************************************************
 * @brief If the entrypoint of your node is not a periodic callback then you
 * may spin up a thread
 */
void TransframerInterface::thread_start(const std::string &tname)
{
  std::thread t = std::thread(&TransframerInterface::entrypoint_wrapper, this);
  pthread_map_[tname] = t.native_handle();
  t.detach();

  RCLCPP_INFO(this->get_logger(), "Thread %s is alive", tname.c_str());
}


/*******************************************************************************
 * @brief https://github.com/bo-yang/terminate_cpp_thread/blob/master/kill_cpp_thread.cc
 */
void TransframerInterface::thread_stop(const std::string& tname)
{
  std::unordered_map<std::string, pthread_t>::const_iterator it =
    pthread_map_.find(tname);

  if (it != pthread_map_.end())
  {
    pthread_cancel(it->second);
    pthread_map_.erase(tname);

    RCLCPP_INFO(this->get_logger(), "Thread %s is rightfully dead", tname.c_str());
  }
}
