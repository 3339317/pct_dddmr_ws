/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, Indoor Fusion Bridge
 *
 * Fusion bridge node: connects dddmr navigation stack with indoor_nav_ws web UI.
 *
 * Bridges:
 *   1. Pose:  /mcl_pose (PoseWithCovarianceStamped) → /localization (Odometry), optional
 *   2. Goal:  /indoor_route_nav/controller/start → /p2p_move_base (action)
 *   3. State: p2p_move_base feedback → /indoor_route_nav/controller/state (String JSON)
 *   4. Done:  p2p_move_base result → /nav/done (String JSON)
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"

#include "dddmr_sys_core/action/p_to_p_move_base.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <nlohmann/json.hpp>

using namespace std::chrono_literals;
using PToPMoveBase = dddmr_sys_core::action::PToPMoveBase;
using GoalHandlePToP = rclcpp_action::ClientGoalHandle<PToPMoveBase>;

class FusionBridgeNode : public rclcpp::Node
{
public:
  FusionBridgeNode()
  : rclcpp::Node("fusion_bridge_node"),
    navigation_active_(false),
    current_vx_(0.0),
    current_wz_(0.0)
  {
    // =========================================================
    // 1. Pose bridge: /mcl_pose → /localization (Odometry)
    // Disable this when another localization node already publishes /localization.
    // =========================================================
    enable_pose_bridge_ = this->declare_parameter<bool>("enable_pose_bridge", true);
    pose_source_topic_ = this->declare_parameter<std::string>("pose_source_topic", "/mcl_pose");
    localization_topic_ = this->declare_parameter<std::string>("localization_topic", "/localization");

    if (enable_pose_bridge_) {
      sub_mcl_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_source_topic_, rclcpp::QoS(10).reliable(),
        std::bind(&FusionBridgeNode::mclPoseCallback, this, std::placeholders::_1));

      pub_localization_ = this->create_publisher<nav_msgs::msg::Odometry>(
        localization_topic_, rclcpp::QoS(10).reliable());

      RCLCPP_INFO(this->get_logger(), "Pose bridge enabled: %s -> %s",
        pose_source_topic_.c_str(), localization_topic_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(),
        "Pose bridge disabled; expecting localization from an external node.");
    }

    // =========================================================
    // 2. Goal bridge: intercept route start → p2p_move_base action
    // =========================================================
    sub_route_ = this->create_subscription<nav_msgs::msg::Path>(
      "/indoor_route_nav/controller/route", rclcpp::QoS(10).reliable(),
      std::bind(&FusionBridgeNode::routeCallback, this, std::placeholders::_1));

    sub_controller_start_ = this->create_subscription<std_msgs::msg::Empty>(
      "/indoor_route_nav/controller/start", rclcpp::QoS(10).reliable(),
      std::bind(&FusionBridgeNode::controllerStartCallback, this, std::placeholders::_1));

    sub_controller_stop_ = this->create_subscription<std_msgs::msg::Empty>(
      "/indoor_route_nav/controller/stop", rclcpp::QoS(10).reliable(),
      std::bind(&FusionBridgeNode::controllerStopCallback, this, std::placeholders::_1));

    // Action client for /p2p_move_base
    p2p_action_client_ = rclcpp_action::create_client<PToPMoveBase>(
      this, "/p2p_move_base");

    // =========================================================
    // 3. State bridge: publish to /indoor_route_nav/controller/state
    // =========================================================
    pub_controller_state_ = this->create_publisher<std_msgs::msg::String>(
      "/indoor_route_nav/controller/state", rclcpp::QoS(10).reliable());

    pub_controller_status_ = this->create_publisher<std_msgs::msg::String>(
      "/indoor_route_nav/controller/status", rclcpp::QoS(10).reliable());

    pub_nav_done_ = this->create_publisher<std_msgs::msg::String>(
      "/nav/done", rclcpp::QoS(10).reliable());

    // State publish timer (10 Hz)
    state_timer_ = this->create_wall_timer(
      100ms, std::bind(&FusionBridgeNode::publishStateTimer, this));

    // =========================================================
    // 4. Subscribe to dddmr's global_path for progress tracking
    // =========================================================
    sub_global_path_ = this->create_subscription<nav_msgs::msg::Path>(
      "/global_path", rclcpp::QoS(1).reliable(),
      std::bind(&FusionBridgeNode::globalPathCallback, this, std::placeholders::_1));

    // TF buffer for getting current robot pose
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "\033[1;32m---->\033[0m FusionBridgeNode started.");
  }

private:
  // =========================================================
  // Pose bridge
  // =========================================================
  void mclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (!pub_localization_) {
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header = msg->header;
    odom.child_frame_id = "base_link";
    odom.pose = msg->pose;
    pub_localization_->publish(odom);
  }

  // =========================================================
  // Route tracking
  // =========================================================
  void routeCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(route_mutex_);
    latest_route_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Received route with %zu points", msg->poses.size());
  }

  // =========================================================
  // Goal bridge: start navigation via dddmr action
  // =========================================================
  void controllerStartCallback(const std_msgs::msg::Empty::SharedPtr)
  {
    std::lock_guard<std::mutex> lock(route_mutex_);
    if (latest_route_.poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "No route available, cannot start navigation");
      publishStatus("无法导航：没有路线数据");
      return;
    }

    // Get the last point of the route as the goal
    const auto & goal_pose = latest_route_.poses.back().pose;
    sendP2PGoal(goal_pose);
  }

  void controllerStopCallback(const std_msgs::msg::Empty::SharedPtr)
  {
    if (navigation_active_ && p2p_goal_handle_) {
      RCLCPP_INFO(this->get_logger(), "Cancelling current navigation goal");
      p2p_action_client_->async_cancel_goal(p2p_goal_handle_);
      navigation_active_ = false;
      publishZeroVelocity();
      publishStatus("导航已停止");
    }
  }

  void sendP2PGoal(const geometry_msgs::msg::Pose & goal_pose)
  {
    if (!p2p_action_client_->action_server_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "p2p_move_base action server not available!");
      publishStatus("导航失败：p2p_move_base 服务不可用");
      return;
    }

    // Cancel any existing goal
    if (navigation_active_ && p2p_goal_handle_) {
      RCLCPP_INFO(this->get_logger(), "Cancelling previous goal before sending new one");
      p2p_action_client_->async_cancel_goal(p2p_goal_handle_);
      std::this_thread::sleep_for(100ms);
    }

    auto goal_msg = PToPMoveBase::Goal();
    goal_msg.target_pose.header.stamp = this->now();
    goal_msg.target_pose.header.frame_id = "map";
    goal_msg.target_pose.pose = goal_pose;
    goal_msg.target_value = 0.0f;

    RCLCPP_INFO(this->get_logger(),
      "Sending goal to p2p_move_base: (%.2f, %.2f, %.2f)",
      goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);

    publishStatus("正在发送导航目标...");

    auto send_goal_options = rclcpp_action::Client<PToPMoveBase>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FusionBridgeNode::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&FusionBridgeNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&FusionBridgeNode::resultCallback, this, std::placeholders::_1);

    p2p_action_client_->async_send_goal(goal_msg, send_goal_options);
    navigation_active_ = true;
  }

  void goalResponseCallback(GoalHandlePToP::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by p2p_move_base");
      publishStatus("导航目标被拒绝");
      navigation_active_ = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by p2p_move_base");
      p2p_goal_handle_ = goal_handle;
      publishStatus("导航目标已接受，正在执行");
    }
  }

  void feedbackCallback(
    GoalHandlePToP::SharedPtr,
    const std::shared_ptr<const PToPMoveBase::Feedback> feedback)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    last_feedback_ = *feedback;
    has_feedback_ = true;
  }

  void resultCallback(const GoalHandlePToP::WrappedResult & result)
  {
    navigation_active_ = false;
    p2p_goal_handle_.reset();

    switch (result.result->status) {
      case 1:  // SUCCESS
        RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
        publishStatus("导航成功到达目标");
        publishNavDone(true, "导航成功到达目标");
        break;
      case 2:  // FAILURE
        RCLCPP_WARN(this->get_logger(), "Navigation failed");
        publishStatus("导航失败");
        publishNavDone(false, "导航失败");
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Navigation ended with status: %d", result.result->status);
        publishStatus("导航结束");
        publishNavDone(false, "导航结束");
        break;
    }
  }

  // =========================================================
  // Global path tracking (for progress reporting)
  // =========================================================
  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(global_path_mutex_);
    latest_global_path_ = *msg;
  }

  // =========================================================
  // State bridge: publish controller/state JSON
  // =========================================================
  void publishStateTimer()
  {
    if (!navigation_active_ && !publish_idle_state_) {
      return;
    }

    nlohmann::json data;
    data["is_auto_moving"] = navigation_active_;
    data["stage"] = navigation_active_ ? "path_tracking" : "idle";
    data["is_obstacle_paused"] = false;
    data["nav_progress_idx"] = 0;
    data["final_target_yaw"] = 0.0;

    // Get feedback data
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      if (has_feedback_) {
        // Extract velocity from feedback if available
        data["current_vx"] = current_vx_;
        data["current_wz"] = current_wz_;
      }
    }

    data["status_text"] = status_text_;
    data["obstacle_distance"] = 999.0;
    data["front_obstacle_min_distance"] = 999.0;
    data["front_obstacle_detected"] = false;
    data["front_obstacle_point"] = nullptr;
    data["front_obstacle_cluster_center"] = nullptr;
    data["lookahead_target"] = nullptr;

    // Route total points
    {
      std::lock_guard<std::mutex> lock(global_path_mutex_);
      data["route_total_points"] = latest_global_path_.poses.size();
    }

    std_msgs::msg::String msg;
    msg.data = data.dump();
    pub_controller_state_->publish(msg);

    publish_idle_state_ = false;
  }

  void publishStatus(const std::string & text)
  {
    status_text_ = text;
    publish_idle_state_ = true;
    std_msgs::msg::String msg;
    msg.data = text;
    pub_controller_status_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "%s", text.c_str());
  }

  void publishNavDone(bool success, const std::string & message)
  {
    nlohmann::json data;
    data["event"] = "nav_done";
    data["success"] = success;
    data["message"] = message;
    data["time"] = this->now().seconds();

    std_msgs::msg::String msg;
    msg.data = data.dump();
    pub_nav_done_->publish(msg);
  }

  void publishZeroVelocity()
  {
    // Publish zero cmd_vel to stop the robot
    // This is handled by p2p_move_base when goal is cancelled
  }

  // =========================================================
  // Members
  // =========================================================
  // Pose bridge
  bool enable_pose_bridge_;
  std::string pose_source_topic_;
  std::string localization_topic_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_mcl_pose_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_localization_;

  // Goal bridge
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_route_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_controller_start_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_controller_stop_;
  rclcpp_action::Client<PToPMoveBase>::SharedPtr p2p_action_client_;
  GoalHandlePToP::SharedPtr p2p_goal_handle_;

  // State bridge
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_controller_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_controller_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_nav_done_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  // Global path tracking
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_path_;
  nav_msgs::msg::Path latest_global_path_;
  std::mutex global_path_mutex_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // State
  std::mutex route_mutex_;
  nav_msgs::msg::Path latest_route_;
  bool navigation_active_;
  double current_vx_;
  double current_wz_;
  std::string status_text_;
  bool publish_idle_state_{true};

  std::mutex feedback_mutex_;
  PToPMoveBase::Feedback last_feedback_;
  bool has_feedback_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FusionBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
