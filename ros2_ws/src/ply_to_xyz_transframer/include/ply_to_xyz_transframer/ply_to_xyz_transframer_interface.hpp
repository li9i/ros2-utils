#include <atomic>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

class TransframerInterface : public rclcpp::Node
{
  public:

    TransframerInterface();

  private:

    // Service servers to power this node on and off
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      enable_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      disable_pause_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      disable_stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      kill_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      shutdown_service_;

    // Service client for task planner to join when this node's execution is
    // over
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
      trigger_join_service_client_ptr_;

    // A timer that checks whether it's time to modify the state of the
    // functionality that this node provides
    rclcpp::TimerBase::SharedPtr clock_;

    // Create callback group for shutdown and kill service servers
    rclcpp::CallbackGroup::CallbackGroup::SharedPtr cb_group_;

    // Available states
    enum States_
    {
      PLAY     = 0,
      PAUSE    = 1,
      STOP     = 2,
      SHUTDOWN = 3,
      KILL     = 4
    };
    std::vector< std::vector<bool> > transition_table_;

    enum States_ state_t0_init_ = STOP;
    enum States_ state_t1_init_ = STOP;
    std::atomic<States_> state_t0_;
    std::atomic<States_> state_t1_;
    std::atomic<bool> is_node_alive_;

    // Set this at the end
    bool has_functionality_been_delivered_successfully_;

    // Does this node need to notify the task planner that its functionality
    // has been served? Or is this node running solo?
    bool task_planner_notify_end_;

    // Maps a thread's given name to its native_handle()
    std::unordered_map<std::string, pthread_t> pthread_map_;


    /***************************************************************************
     * @brief Executes periodically. Reads the execution status flags, which are
     * modified from the service server calls coming from the task planner
     * that manages this pkg.
     */
    void clock_callback();

    /***************************************************************************
     * @brief The final act: calls rclcpp::shutdown()
     */
    void commit_node_suicide();

    /***************************************************************************
     * @brief The final act: calls system("kill self pid")
     */
    void commit_process_suicide();

    /***************************************************************************
     * @brief Declare params in this node so that node `ply_to_xyz_transframer` 
     * (without the word interface), which is called into execution by 
     * `execution()` into execution, may access their values. If declared 
     * downstream (ply_to_xyz_transframer) then `ros2 param list` does not 
     * list them
     */
    void declare_params();

    /***************************************************************************
     * @brief Wraps the functionality that this package provides for use with
     * a task planner
     */
    void entrypoint_wrapper();

    /***************************************************************************
     * @brief Execute the functionality that the package offers
     */
    void execute();

    /***************************************************************************
     * @brief Service servers to start, pause, and stop the functionality that
     * this node offers, and kill the node altogether
     */
    void handle_request_disable_pause(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void handle_request_disable_stop(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void handle_request_enable(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void handle_request_kill(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void handle_request_shutdown(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    /***************************************************************************
     * @brief Notifies the task planner that manages this pkg that this node has
     * finished its work (or that it was stopped/killed in the process---and its
     * work was forced to finish).
     */
    bool task_planner_notify_end();

    /***************************************************************************
     * @brief If the entrypoint of your node is not a periodic callback then you
     * may spin up a thread.
     */
    void thread_start(const std::string &tname);
    void thread_stop(const std::string &tname);
};
