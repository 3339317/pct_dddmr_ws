#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace indoor_route_nav
{

namespace
{

struct Point
{
  double x{};
  double y{};
  double z{};
};

double normalizeAngleDeg(double deg)
{
  while (deg > 180.0) {
    deg -= 360.0;
  }
  while (deg < -180.0) {
    deg += 360.0;
  }
  return deg;
}

double clampAbs(double value, double max_abs, double min_abs)
{
  const double sign = value >= 0.0 ? 1.0 : -1.0;
  const double mag = std::abs(value);
  if (mag < 1e-9) {
    return 0.0;
  }
  return sign * std::min(max_abs, std::max(min_abs, mag));
}

double dist3d(const Point & a, const Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double dist2d(const Point & a, const Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

Point interpolatePoints(const Point & a, const Point & b, double t)
{
  return Point{
    a.x + t * (b.x - a.x),
    a.y + t * (b.y - a.y),
    a.z + t * (b.z - a.z)};
}

double yawDegFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
}

}  // namespace

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : Node("route_tracker_node")
  {
    localization_topic_ = declare_parameter<std::string>("localization_topic", "/localization");
    route_topic_ = declare_parameter<std::string>("route_topic", "/pct_dddmr_web/controller/route");
    config_topic_ = declare_parameter<std::string>("config_topic", "/pct_dddmr_web/controller/config");
    start_topic_ = declare_parameter<std::string>("start_topic", "/pct_dddmr_web/controller/start");
    stop_topic_ = declare_parameter<std::string>("stop_topic", "/pct_dddmr_web/controller/stop");
    clear_topic_ = declare_parameter<std::string>("clear_topic", "/pct_dddmr_web/controller/clear");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    state_topic_ = declare_parameter<std::string>("state_topic", "/pct_dddmr_web/controller/state");
    status_topic_ = declare_parameter<std::string>("status_topic", "/pct_dddmr_web/controller/status");
    done_topic_ = declare_parameter<std::string>("done_topic", "/nav/done");
    control_frequency_ = declare_parameter("control_frequency", 50.0);

    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      localization_topic_, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        pose_.x = msg->pose.pose.position.x;
        pose_.y = msg->pose.pose.position.y;
        pose_.z = msg->pose.pose.position.z;
        yaw_deg_ = yawDegFromQuat(msg->pose.pose.orientation);
        localized_ = true;
      });

    obstacle_cloud_topic_ = declare_parameter<std::string>(
      "obstacle_cloud_topic", "/segmented_cloud_pure");
    lidar_height_ = declare_parameter("lidar_height", 0.86);
    lidar_forward_offset_ = declare_parameter("lidar_forward_offset", 0.30);
    lidar_left_offset_ = declare_parameter("lidar_left_offset", 0.0);
    ground_z_tolerance_ = declare_parameter("ground_z_tolerance", 0.12);
    max_slope_ = declare_parameter("max_slope", 0.65);
    obstacle_avoidance_enabled_ = declare_parameter("obstacle_avoidance_enabled", false);
    obstacle_confirm_frames_ = declare_parameter("obstacle_confirm_frames", 3);
    obstacle_detour_enabled_ = declare_parameter("obstacle_detour_enabled", true);
    detour_lateral_clearance_ = declare_parameter("detour_lateral_clearance", 0.85);
    detour_rejoin_distance_ = declare_parameter("detour_rejoin_distance", 1.80);
    detour_entry_distance_ = declare_parameter("detour_entry_distance", 1.20);
    detour_safety_margin_ = declare_parameter("detour_safety_margin", 0.25);

    obstacle_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      obstacle_cloud_topic_, 10,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cloud_callback(msg);
      });

    route_sub_ = create_subscription<nav_msgs::msg::Path>(
      route_topic_, 10,
      [this](nav_msgs::msg::Path::SharedPtr msg) {
        setRoute(*msg);
      });

    config_sub_ = create_subscription<std_msgs::msg::String>(
      config_topic_, 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        applyConfig(msg->data);
      });

    start_sub_ = create_subscription<std_msgs::msg::Empty>(
      start_topic_, 10,
      [this](std_msgs::msg::Empty::SharedPtr) {
        startNavigation();
      });

    stop_sub_ = create_subscription<std_msgs::msg::Empty>(
      stop_topic_, 10,
      [this](std_msgs::msg::Empty::SharedPtr) {
        stopNavigation("导航已停止");
      });

    clear_sub_ = create_subscription<std_msgs::msg::Empty>(
      clear_topic_, 10,
      [this](std_msgs::msg::Empty::SharedPtr) {
        route_.clear();
        route_cumlen_.clear();
        stopNavigation("已清空控制器路线");
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    done_pub_ = create_publisher<std_msgs::msg::String>(done_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    const auto control_period = std::chrono::duration<double>(
      1.0 / std::max(1.0, control_frequency_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
      [this]() {controlLoop();});
    publishStatus("Pure Pursuit 路线追踪器已启动");
    RCLCPP_INFO(get_logger(), "route=%s start=%s cmd_vel=%s state=%s obstacle_cloud=%s",
      route_topic_.c_str(), start_topic_.c_str(), cmd_vel_topic_.c_str(), state_topic_.c_str(),
      obstacle_cloud_topic_.c_str());
  }

private:
  void setRoute(const nav_msgs::msg::Path & path)
  {
    std::vector<Point> new_route;
    new_route.reserve(path.poses.size());
    for (const auto & pose : path.poses) {
      new_route.push_back(Point{
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z});
    }

    if (sameRoute(new_route)) {
      return;
    }

    route_ = std::move(new_route);

    route_cumlen_.clear();
    if (!route_.empty()) {
      route_cumlen_.push_back(0.0);
      for (std::size_t i = 1; i < route_.size(); ++i) {
        route_cumlen_.push_back(route_cumlen_.back() + dist3d(route_[i], route_[i - 1]));
      }
    }

    buildRouteSegmentIndex();

    if (route_.size() >= 2 && !final_yaw_configured_) {
      final_target_yaw_ = routeHeadingDeg(route_.size() - 1);
    }
    nav_progress_idx_ = 0;
    lookahead_target_ = Point{};
    has_lookahead_ = false;
    is_obstacle_on_path_ = false;
    clearLocalDetour();
    obstacle_boxes_.clear();
    obstacle_clusters_.clear();
    obstacle_min_distance_ = 999.0;
    publishStatus("控制器已接收路线，点数 " + std::to_string(route_.size()));
  }

  bool sameRoute(const std::vector<Point> & other) const
  {
    if (route_.size() != other.size()) return false;
    for (std::size_t i = 0; i < route_.size(); ++i) {
      if (std::abs(route_[i].x - other[i].x) > 1e-4 ||
          std::abs(route_[i].y - other[i].y) > 1e-4 ||
          std::abs(route_[i].z - other[i].z) > 1e-4) {
        return false;
      }
    }
    return true;
  }

  static std::int64_t gridKey(int ix, int iy)
  {
    return (static_cast<std::int64_t>(ix) << 32) | static_cast<std::uint32_t>(iy);
  }

  void buildRouteSegmentIndex()
  {
    route_segment_index_.clear();
    if (route_.size() < 2) return;

    const double cell_size = 0.8;
    const double inv_cell = 1.0 / cell_size;

    for (std::size_t seg_idx = 0; seg_idx + 1 < route_.size(); ++seg_idx) {
      const auto & a = route_[seg_idx];
      const auto & b = route_[seg_idx + 1];

      int cx0 = static_cast<int>(std::floor(a.x * inv_cell));
      int cy0 = static_cast<int>(std::floor(a.y * inv_cell));
      int cx1 = static_cast<int>(std::floor(b.x * inv_cell));
      int cy1 = static_cast<int>(std::floor(b.y * inv_cell));

      int steps = std::max(std::abs(cx1 - cx0), std::abs(cy1 - cy0));
      if (steps <= 0) steps = 1;

      for (int s = 0; s <= steps; ++s) {
        double t = static_cast<double>(s) / static_cast<double>(steps);
        int cx = static_cast<int>(std::floor((a.x + t * (b.x - a.x)) * inv_cell));
        int cy = static_cast<int>(std::floor((a.y + t * (b.y - a.y)) * inv_cell));
        route_segment_index_[gridKey(cx, cy)].push_back(seg_idx);
      }
    }
  }

  void applyConfig(const std::string & text)
  {
    try {
      const auto cfg = nlohmann::json::parse(text);
      max_vx_ = std::max(0.01, cfg.value("max_linear_x", max_vx_));
      max_wz_ = std::max(0.01, cfg.value("max_angular_z", max_wz_));
      min_vx_ = std::max(0.0, cfg.value("min_linear_x", min_vx_));
      min_wz_ = std::max(0.0, cfg.value("min_angular_z", min_wz_));
      arrival_dist_ = std::max(0.01, cfg.value("arrival_distance", arrival_dist_));
      arrival_angle_ = std::max(0.1, cfg.value("arrival_angle_deg", arrival_angle_));
      alpha_ = std::min(1.0, std::max(0.001, cfg.value("alpha", alpha_)));
      obstacle_threshold_ = std::max(0.01, cfg.value("obstacle_threshold", obstacle_threshold_));
      resume_delay_ = std::max(0.0, cfg.value("resume_delay", resume_delay_));
      deceleration_alpha_ = std::min(1.0, std::max(0.001, cfg.value("deceleration_alpha", deceleration_alpha_)));
      lookahead_distance_ = std::max(0.05, cfg.value("lookahead_distance", lookahead_distance_));
      path_yaw_kp_ = std::max(0.0001, cfg.value("path_yaw_kp", path_yaw_kp_));
      final_yaw_kp_ = std::max(0.0001, cfg.value("final_yaw_kp", final_yaw_kp_));
      end_slowdown_distance_ = std::max(0.05, cfg.value("end_slowdown_distance", end_slowdown_distance_));
      heading_slow_angle_deg_ = std::max(1.0, cfg.value("heading_slow_angle_deg", heading_slow_angle_deg_));
      rotate_in_place_angle_deg_ = std::max(1.0, cfg.value("rotate_in_place_angle_deg", rotate_in_place_angle_deg_));
      rotate_exit_angle_deg_ = std::max(1.0, cfg.value("rotate_exit_angle_deg", rotate_exit_angle_deg_));
      if (cfg.contains("obstacle_avoidance_enabled")) {
        obstacle_avoidance_enabled_ = cfg.at("obstacle_avoidance_enabled").get<bool>();
      }
      if (cfg.contains("obstacle_detour_enabled")) {
        obstacle_detour_enabled_ = cfg.at("obstacle_detour_enabled").get<bool>();
      }
      detour_lateral_clearance_ = std::max(0.20, cfg.value("detour_lateral_clearance", detour_lateral_clearance_));
      detour_rejoin_distance_ = std::max(0.50, cfg.value("detour_rejoin_distance", detour_rejoin_distance_));
      detour_entry_distance_ = std::max(0.10, cfg.value("detour_entry_distance", detour_entry_distance_));
      detour_safety_margin_ = std::max(0.05, cfg.value("detour_safety_margin", detour_safety_margin_));

      if (cfg.contains("final_target_yaw")) {
        final_target_yaw_ = cfg.at("final_target_yaw").get<double>();
        final_yaw_configured_ = true;
      }
      if (min_vx_ > max_vx_) {
        min_vx_ = max_vx_;
      }
      if (min_wz_ > max_wz_) {
        min_wz_ = max_wz_;
      }
    } catch (const std::exception & e) {
      publishStatus(std::string("控制参数解析失败: ") + e.what());
    }
  }

  void startNavigation()
  {
    if (!localized_) {
      publishStatus("开启导航失败: 尚未定位");
      return;
    }
    if (route_.size() < 2 || route_cumlen_.size() != route_.size()) {
      publishStatus("开启导航失败: 尚未收到路线");
      return;
    }

    is_auto_moving_ = true;
    is_obstacle_paused_ = false;
    clearLocalDetour();
    stage_ = "path_tracking";
    current_vx_ = 0.0;
    current_wz_ = 0.0;
    nav_progress_idx_ = findClosestRouteIndex(pose_);
    if (!final_yaw_configured_) {
      final_target_yaw_ = routeHeadingDeg(route_.size() - 1);
    }
    publishStatus("C++ 控制器开始导航");
  }

  void stopNavigation(const std::string & status)
  {
    is_auto_moving_ = false;
    is_obstacle_paused_ = false;
    clearLocalDetour();
    stage_ = "idle";
    has_lookahead_ = false;
    current_vx_ = 0.0;
    current_wz_ = 0.0;
    publishZero();
    publishStatus(status);
  }

  double routeHeadingDeg(std::size_t idx) const
  {
    return routeHeadingDegOn(route_, idx);
  }

  double routeHeadingDegOn(const std::vector<Point> & route, std::size_t idx) const
  {
    if (route.size() < 2) {
      return 0.0;
    }
    idx = std::min(idx, route.size() - 1);
    const Point * a = nullptr;
    const Point * b = nullptr;
    if (idx + 1 < route.size()) {
      a = &route[idx];
      b = &route[idx + 1];
    } else {
      a = &route[idx - 1];
      b = &route[idx];
    }
    return std::atan2(b->y - a->y, b->x - a->x) * 180.0 / M_PI;
  }

  Point interpolateByS(double s) const
  {
    return interpolateBySOn(route_, route_cumlen_, s);
  }

  Point interpolateBySOn(
    const std::vector<Point> & route,
    const std::vector<double> & cumlen,
    double s) const
  {
    if (route.empty()) {
      return Point{};
    }
    if (route.size() == 1 || s <= 0.0) {
      return route.front();
    }
    if (s >= cumlen.back()) {
      return route.back();
    }

    const auto it = std::lower_bound(cumlen.begin(), cumlen.end(), s);
    std::size_t idx = static_cast<std::size_t>(std::distance(cumlen.begin(), it));
    if (idx == 0) {
      return route.front();
    }
    if (idx >= cumlen.size()) {
      return route.back();
    }

    const double s0 = cumlen[idx - 1];
    const double s1 = cumlen[idx];
    const double ds = s1 - s0;
    if (ds < 1e-9) {
      return route[idx - 1];
    }
    const double r = (s - s0) / ds;
    return interpolatePoints(route[idx - 1], route[idx], r);
  }

  std::size_t findClosestRouteIndex(const Point & p) const
  {
    return findClosestRouteIndexOn(route_, p, nav_progress_idx_);
  }

  std::size_t findClosestRouteIndexOn(
    const std::vector<Point> & route,
    const Point & p,
    std::size_t progress_idx) const
  {
    if (route.empty()) {
      return 0;
    }
    std::size_t start = 0;
    std::size_t end = route.size();
    if (is_auto_moving_ && route.size() > 80) {
      start = progress_idx > 20 ? progress_idx - 20 : 0;
      end = std::min(route.size(), progress_idx + 220);
    }

    std::size_t best_idx = start;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = start; i < end; ++i) {
      const double d = dist3d(p, route[i]);
      if (d < best) {
        best = d;
        best_idx = i;
      }
    }
    return best_idx;
  }

  void controlLoop()
  {
    publish_state_tick_++;

    if (!obstacle_avoidance_enabled_) {
      is_obstacle_on_path_ = false;
      is_obstacle_paused_ = false;
      clearLocalDetour();
    }
    if (detour_active_ && isPathBlocked(detour_route_, detour_safety_margin_)) {
      clearLocalDetour();
      is_obstacle_paused_ = true;
      last_obstacle_time_ = now();
      publishStatus("临时绕行路线被障碍占用，控制器暂停");
    }
    if (is_obstacle_on_path_ && is_auto_moving_ && !is_obstacle_paused_ && !detour_active_) {
      if (obstacle_detour_enabled_ && buildLocalDetour()) {
        is_obstacle_paused_ = false;
        publishStatus("检测到前方障碍，已生成临时绕行路线");
      } else {
        is_obstacle_paused_ = true;
        last_obstacle_time_ = now();
        publishStatus("检测到前方障碍，无法安全绕行，控制器暂停");
      }
    }
    if (is_obstacle_paused_ && !is_obstacle_on_path_ &&
        (now() - last_obstacle_time_).seconds() > resume_delay_) {
      is_obstacle_paused_ = false;
      obstacle_min_distance_ = 999.0;
      publishStatus("障碍已移除，C++ 控制器恢复导航");
    }

    if (!is_auto_moving_ || !localized_ || route_.size() < 2) {
      current_vx_ = 0.0;
      current_wz_ = 0.0;
      publishZero();
      throttledPublishState();
      return;
    }

    if (detour_active_) {
      const double detour_end_dist = dist2d(detour_route_.back(), pose_);
      if (detour_end_dist <= std::max(0.25, arrival_dist_)) {
        nav_progress_idx_ = std::max(nav_progress_idx_, detour_rejoin_idx_);
        clearLocalDetour();
        publishStatus("临时绕行完成，已接回原路线");
      }
    }

    if (is_obstacle_paused_) {
      current_vx_ *= (1.0 - deceleration_alpha_);
      current_wz_ *= (1.0 - deceleration_alpha_);
      if (std::abs(current_vx_) < 0.005) {
        current_vx_ = 0.0;
      }
      if (std::abs(current_wz_) < 0.005) {
        current_wz_ = 0.0;
      }
      publishCmd(current_vx_, current_wz_);
      stage_ = "obstacle_pause";
      throttledPublishState();
      return;
    }

    const bool follow_detour = detour_active_ && detour_route_.size() >= 2;
    const auto & follow_route = follow_detour ? detour_route_ : route_;
    const auto & follow_cumlen = follow_detour ? detour_cumlen_ : route_cumlen_;
    std::size_t & follow_progress_idx = follow_detour ? detour_progress_idx_ : nav_progress_idx_;

    const std::size_t nearest_idx = findClosestRouteIndexOn(
      follow_route, pose_, follow_progress_idx);
    follow_progress_idx = std::max(follow_progress_idx, nearest_idx);
    if (!follow_detour) {
      nav_progress_idx_ = follow_progress_idx;
    }

    const auto & end_pt = route_.back();
    const double end_dist = dist2d(end_pt, pose_);
    const double remain_s = route_cumlen_.back() - route_cumlen_[nav_progress_idx_];

    if (stage_ == "final_align") {
      const double yaw_err = normalizeAngleDeg(final_target_yaw_ - yaw_deg_);
      if (std::abs(yaw_err) < arrival_angle_ && std::abs(current_vx_) < 0.005) {
        finishNavigation();
        throttledPublishState();
        return;
      }
      const double target_wz = clampAbs(yaw_err * final_yaw_kp_, max_wz_, min_wz_);
      current_vx_ *= (1.0 - deceleration_alpha_);
      if (std::abs(current_vx_) < 0.005) current_vx_ = 0.0;
      current_wz_ = alpha_ * target_wz + (1.0 - alpha_) * current_wz_;
      if (std::abs(current_wz_) < 0.005) {
        current_wz_ = 0.0;
      }
      publishCmd(current_vx_, current_wz_);
      throttledPublishState();
      return;
    }

    stage_ = follow_detour ? "local_detour" : "path_tracking";
    if (remain_s <= end_slowdown_distance_ && end_dist <= arrival_dist_) {
      stage_ = "final_align";
      current_vx_ *= (1.0 - deceleration_alpha_);
      if (std::abs(current_vx_) < 0.005) current_vx_ = 0.0;
      current_wz_ *= (1.0 - deceleration_alpha_);
      if (std::abs(current_wz_) < 0.005) current_wz_ = 0.0;
      publishCmd(current_vx_, current_wz_);
      throttledPublishState();
      return;
    }

    double desired_v = max_vx_;
    const double target_s = std::min(
      follow_cumlen.back(), follow_cumlen[follow_progress_idx] + lookahead_distance_);
    Point target = interpolateBySOn(follow_route, follow_cumlen, target_s);
    if (remain_s <= end_slowdown_distance_) {
      desired_v = std::min(max_vx_, std::max(0.05, end_dist * 0.9));
    }

    lookahead_target_ = target;
    has_lookahead_ = true;

    const double dx = target.x - pose_.x;
    const double dy = target.y - pose_.y;
    const double dz = target.z - pose_.z;
    const double dist_xy = std::hypot(dx, dy);
    const double dist_3d = std::sqrt(dx * dx + dy * dy + dz * dz);

    double bearing = routeHeadingDegOn(follow_route, follow_progress_idx);
    if (dist_xy >= 1e-6) {
      bearing = std::atan2(dy, dx) * 180.0 / M_PI;
    }
    const double yaw_error = normalizeAngleDeg(bearing - yaw_deg_);
    const double abs_yaw_error = std::abs(yaw_error);

    if (abs_yaw_error >= rotate_in_place_angle_deg_ ||
        (stage_ == "rotate_in_place" && abs_yaw_error > rotate_exit_angle_deg_)) {
      stage_ = "rotate_in_place";
      const double target_wz = clampAbs(yaw_error * path_yaw_kp_, max_wz_, min_wz_);
      current_vx_ = 0.0;
      current_wz_ = alpha_ * target_wz + (1.0 - alpha_) * current_wz_;
      if (std::abs(current_wz_) < 0.005) {
        current_wz_ = 0.0;
      }
      publishCmd(current_vx_, current_wz_);
      throttledPublishState();
      return;
    }

    double turn_scale = 1.0 - std::min(abs_yaw_error, heading_slow_angle_deg_) / heading_slow_angle_deg_;
    turn_scale = std::max(0.18, turn_scale);
    double target_vx = desired_v * turn_scale;
    if (remain_s > end_slowdown_distance_) {
      target_vx = std::max(min_vx_, target_vx);
    }

    const double speed_ref_dist = std::max(dist_xy, std::min(dist_3d, 0.30));
    target_vx = std::min(target_vx, std::max(0.05, speed_ref_dist * 1.2));

    const double target_wz = clampAbs(
      yaw_error * path_yaw_kp_,
      max_wz_,
      abs_yaw_error > 1.0 ? min_wz_ : 0.0);

    current_vx_ = alpha_ * target_vx + (1.0 - alpha_) * current_vx_;
    current_wz_ = alpha_ * target_wz + (1.0 - alpha_) * current_wz_;
    if (std::abs(current_vx_) < 0.005) {
      current_vx_ = 0.0;
    }
    if (std::abs(current_wz_) < 0.005) {
      current_wz_ = 0.0;
    }

    publishCmd(current_vx_, current_wz_);
    throttledPublishState();
  }

  void throttledPublishState()
  {
    if (publish_state_tick_ % 4 == 0) {
      publishState();
    }
  }

  void finishNavigation()
  {
    is_auto_moving_ = false;
    stage_ = "idle";
    current_vx_ = 0.0;
    current_wz_ = 0.0;
    has_lookahead_ = false;
    route_.clear();
    route_cumlen_.clear();
    clearLocalDetour();
    publishZero();
    publishStatus("路线导航完成");

    std_msgs::msg::String msg;
    nlohmann::json data;
    data["event"] = "nav_done";
    data["success"] = true;
    data["message"] = "路线导航完成";
    data["time"] = now().seconds();
    msg.data = data.dump();
    done_pub_->publish(msg);
  }

  void publishCmd(double vx, double wz)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx;
    cmd.angular.z = wz;
    cmd_pub_->publish(cmd);
  }

  void publishZero()
  {
    publishCmd(0.0, 0.0);
  }

  void publishStatus(const std::string & text)
  {
    status_text_ = text;
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  void publishState()
  {
    nlohmann::json data;
    data["current_vx"] = current_vx_;
    data["current_wz"] = current_wz_;
    data["status_text"] = status_text_;
    data["is_auto_moving"] = is_auto_moving_;
    data["stage"] = stage_;
    data["is_obstacle_paused"] = is_obstacle_paused_;
    data["nav_progress_idx"] = nav_progress_idx_;
    data["route_total_points"] = route_.size();
    data["final_target_yaw"] = final_target_yaw_;
    data["obstacle_distance"] = obstacle_min_distance_;
    data["local_detour_active"] = detour_active_;
    nlohmann::json detour_path = nlohmann::json::array();
    for (const auto & p : detour_route_) {
      detour_path.push_back(nlohmann::json{{"x", p.x}, {"y", p.y}, {"z", p.z}});
    }
    data["local_detour_path"] = detour_path;
    if (has_lookahead_) {
      data["lookahead_target"] = {
        {"x", lookahead_target_.x},
        {"y", lookahead_target_.y},
        {"z", lookahead_target_.z}};
    } else {
      data["lookahead_target"] = nullptr;
    }

    nlohmann::json boxes = nlohmann::json::array();
    for (const auto & b : obstacle_boxes_) {
      boxes.push_back({
        {"x_min", b.x_min}, {"y_min", b.y_min},
        {"x_max", b.x_max}, {"y_max", b.y_max},
      });
    }
    data["obstacle_boxes"] = boxes;

    nlohmann::json clusters = nlohmann::json::array();
    for (const auto & c : obstacle_clusters_) {
      clusters.push_back({
        {"center_x", c.center_x},
        {"center_y", c.center_y},
        {"radius", c.radius},
        {"z_min", c.z_min},
        {"z_max", c.z_max},
        {"on_path", c.on_path},
      });
    }
    data["obstacle_clusters"] = clusters;

    static int publish_tick = 0;
    publish_tick++;
    if (publish_tick % 40 == 1) {
      RCLCPP_DEBUG(get_logger(),
        "[publish #%d] clusters=%zu boxes=%zu dist=%.1f moving=%d paused=%d",
        publish_tick, obstacle_clusters_.size(), obstacle_boxes_.size(),
        obstacle_min_distance_, is_auto_moving_ ? 1 : 0,
        is_obstacle_paused_ ? 1 : 0);
    }

    std_msgs::msg::String msg;
    msg.data = data.dump();
    state_pub_->publish(msg);
  }

  static double point_to_segment_dist(double px, double py,
      double ax, double ay, double bx, double by)
  {
    double abx = bx - ax, aby = by - ay;
    double len_sq = abx * abx + aby * aby;
    if (len_sq < 1e-12) return std::hypot(px - ax, py - ay);
    double t = ((px - ax) * abx + (py - ay) * aby) / len_sq;
    if (t < 0.0) return std::hypot(px - ax, py - ay);
    if (t > 1.0) return std::hypot(px - bx, py - by);
    double proj_x = ax + t * abx, proj_y = ay + t * aby;
    return std::hypot(px - proj_x, py - proj_y);
  }

  void rebuildCumlen(const std::vector<Point> & route, std::vector<double> & cumlen) const
  {
    cumlen.clear();
    if (route.empty()) return;
    cumlen.push_back(0.0);
    for (std::size_t i = 1; i < route.size(); ++i) {
      cumlen.push_back(cumlen.back() + dist3d(route[i], route[i - 1]));
    }
  }

  void clearLocalDetour()
  {
    detour_active_ = false;
    detour_route_.clear();
    detour_cumlen_.clear();
    detour_progress_idx_ = 0;
    detour_rejoin_idx_ = 0;
  }

  bool isPathBlocked(const std::vector<Point> & path, double extra_margin) const
  {
    if (path.size() < 2 || obstacle_clusters_.empty()) return false;
    const double margin = 0.22 + extra_margin;
    for (const auto & c : obstacle_clusters_) {
      for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const double d = point_to_segment_dist(
          c.center_x, c.center_y,
          path[i].x, path[i].y,
          path[i + 1].x, path[i + 1].y);
        if (d < margin + c.radius) {
          return true;
        }
      }
    }
    return false;
  }

  bool buildLocalDetour()
  {
    if (route_.size() < 3 || route_cumlen_.size() != route_.size() || obstacle_clusters_.empty()) {
      return false;
    }

    const std::size_t base_idx = findClosestRouteIndex(pose_);
    const double base_s = route_cumlen_[base_idx];
    std::size_t obs_idx = base_idx;
    double best_obs_dist = std::numeric_limits<double>::infinity();
    for (const auto & c : obstacle_clusters_) {
      const Point cp{c.center_x, c.center_y, (c.z_min + c.z_max) * 0.5};
      const std::size_t idx = findClosestRouteIndexOn(route_, cp, base_idx);
      if (idx < base_idx) continue;
      const double d_robot = dist2d(cp, pose_);
      if (d_robot < best_obs_dist) {
        best_obs_dist = d_robot;
        obs_idx = idx;
      }
    }

    if (!std::isfinite(best_obs_dist)) {
      return false;
    }

    const double obs_s = route_cumlen_[obs_idx];
    const double entry_s = std::max(base_s, obs_s - detour_entry_distance_);
    const double exit_s = std::min(route_cumlen_.back(), obs_s + detour_rejoin_distance_);
    if (exit_s <= entry_s + 0.30) {
      return false;
    }

    const Point entry = interpolateByS(entry_s);
    const Point exit = interpolateByS(exit_s);
    const double heading = routeHeadingDeg(obs_idx) * M_PI / 180.0;
    const double nx = -std::sin(heading);
    const double ny = std::cos(heading);

    double max_radius = 0.20;
    for (const auto & c : obstacle_clusters_) {
      max_radius = std::max(max_radius, c.radius);
    }
    const double offset = std::max(detour_lateral_clearance_, max_radius + 0.55);

    auto make_candidate = [&](double side) {
      std::vector<Point> candidate;
      candidate.reserve(6);
      candidate.push_back(pose_);
      candidate.push_back(entry);
      const Point mid = interpolateByS(obs_s);
      candidate.push_back(Point{entry.x + side * nx * offset, entry.y + side * ny * offset, entry.z});
      candidate.push_back(Point{mid.x + side * nx * offset, mid.y + side * ny * offset, mid.z});
      candidate.push_back(Point{exit.x + side * nx * offset * 0.35, exit.y + side * ny * offset * 0.35, exit.z});
      candidate.push_back(exit);
      return candidate;
    };

    const auto left = make_candidate(1.0);
    const auto right = make_candidate(-1.0);
    const bool left_blocked = isPathBlocked(left, detour_safety_margin_);
    const bool right_blocked = isPathBlocked(right, detour_safety_margin_);
    if (left_blocked && right_blocked) {
      return false;
    }

    if (!left_blocked && !right_blocked) {
      detour_route_ = pathLength(left) <= pathLength(right) ? left : right;
    } else {
      detour_route_ = left_blocked ? right : left;
    }

    rebuildCumlen(detour_route_, detour_cumlen_);
    detour_progress_idx_ = 0;
    detour_rejoin_idx_ = obs_idx;
    for (std::size_t i = obs_idx; i < route_cumlen_.size(); ++i) {
      if (route_cumlen_[i] >= exit_s) {
        detour_rejoin_idx_ = i;
        break;
      }
    }
    detour_active_ = detour_route_.size() >= 2;
    return detour_active_;
  }

  double pathLength(const std::vector<Point> & path) const
  {
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      length += dist3d(path[i], path[i - 1]);
    }
    return length;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
  {
    if (!localized_) return;
    if (route_.size() < 2) {
      is_obstacle_on_path_ = false;
      obstacle_boxes_.clear();
      obstacle_clusters_.clear();
      return;
    }

    obstacle_boxes_.clear();
    obstacle_clusters_.clear();
    is_obstacle_on_path_ = false;
    obstacle_min_distance_ = 999.0;

    double cos_yaw = std::cos(yaw_deg_ * M_PI / 180.0);
    double sin_yaw = std::sin(yaw_deg_ * M_PI / 180.0);
    double corridor_hw = 0.50;
    double floor_z = pose_.z - lidar_height_;
    double close_range = lidar_height_ * 2.5;
    double max_range = obstacle_threshold_ * 4.0;

    struct ObstPt { double x, y, z; double horiz_dist; bool on_path; };
    std::vector<ObstPt> obstacle_points;

    const double cell_size = 0.8;
    const double inv_cell = 1.0 / cell_size;

    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    bool found = false;

    std::size_t raw_pts = 0;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      raw_pts++;
      const double px = static_cast<double>(*iter_x);
      const double py = static_cast<double>(*iter_y);
      const double pz = static_cast<double>(*iter_z);
      Point map_pt = obstaclePointToMap(px, py, pz, msg->header.frame_id);
      double dx = map_pt.x - pose_.x;
      double dy = map_pt.y - pose_.y;
      double horiz_dist = std::hypot(dx, dy);
      if (horiz_dist < 0.08 || horiz_dist > max_range) continue;

      double front_check = (dx * cos_yaw + dy * sin_yaw) / horiz_dist;
      if (front_check < 0.5) continue;

      double dz = map_pt.z - floor_z;
      bool is_ground = false;

      if (horiz_dist < close_range) {
        if (dz < ground_z_tolerance_) is_ground = true;
      } else {
        double slope = dz / horiz_dist;
        if (slope < max_slope_) is_ground = true;
      }

      if (is_ground) continue;

      int cx = static_cast<int>(std::floor(map_pt.x * inv_cell));
      int cy = static_cast<int>(std::floor(map_pt.y * inv_cell));

      bool hit = false;
      for (int dc = -1; dc <= 1 && !hit; ++dc) {
        for (int dr = -1; dr <= 1 && !hit; ++dr) {
          auto it = route_segment_index_.find(gridKey(cx + dc, cy + dr));
          if (it == route_segment_index_.end()) continue;
          for (std::size_t seg_idx : it->second) {
            double d = point_to_segment_dist(
              map_pt.x, map_pt.y,
              route_[seg_idx].x, route_[seg_idx].y,
              route_[seg_idx + 1].x, route_[seg_idx + 1].y);
            if (d < corridor_hw) {
              min_x = std::min(min_x, map_pt.x);
              max_x = std::max(max_x, map_pt.x);
              min_y = std::min(min_y, map_pt.y);
              max_y = std::max(max_y, map_pt.y);
              if (horiz_dist < obstacle_min_distance_) obstacle_min_distance_ = horiz_dist;
              found = true;
              hit = true;
              break;
            }
          }
        }
      }
      obstacle_points.push_back({map_pt.x, map_pt.y, map_pt.z, horiz_dist, hit});
    }

    if (found) {
      obstacle_boxes_.push_back({min_x, min_y, max_x, max_y});
    }

    const bool close_obstacle_on_path =
      found && obstacle_min_distance_ <= obstacle_threshold_;
    if (close_obstacle_on_path) {
      obstacle_hit_count_++;
    } else {
      obstacle_hit_count_ = 0;
    }
    is_obstacle_on_path_ =
      obstacle_avoidance_enabled_ && obstacle_hit_count_ >= obstacle_confirm_frames_;

    if (!obstacle_points.empty()) {
      double cluster_dist = 0.50;
      std::vector<bool> assigned(obstacle_points.size(), false);

      for (size_t i = 0; i < obstacle_points.size(); ++i) {
        if (assigned[i]) continue;

        ObstacleCluster cl;
        cl.center_x = obstacle_points[i].x;
        cl.center_y = obstacle_points[i].y;
        cl.z_min = obstacle_points[i].z;
        cl.z_max = obstacle_points[i].z;
        cl.radius = 0.15;
        bool on_path = obstacle_points[i].on_path;
        assigned[i] = true;
        int count = 1;

        bool changed = true;
        while (changed) {
          changed = false;
          for (size_t j = 0; j < obstacle_points.size(); ++j) {
            if (assigned[j]) continue;
            double dx2 = obstacle_points[j].x - cl.center_x;
            double dy2 = obstacle_points[j].y - cl.center_y;
            double d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
            if (d2 < cluster_dist) {
              cl.center_x = (cl.center_x * count + obstacle_points[j].x) / (count + 1);
              cl.center_y = (cl.center_y * count + obstacle_points[j].y) / (count + 1);
              cl.z_min = std::min(cl.z_min, obstacle_points[j].z);
              cl.z_max = std::max(cl.z_max, obstacle_points[j].z);
              on_path = on_path || obstacle_points[j].on_path;
              assigned[j] = true;
              count++;
              changed = true;
            }
          }
        }

        if (count > 1) {
          cl.radius = 0.0;
          std::vector<size_t> cluster_indices = {i};
          for (size_t j = i + 1; j < obstacle_points.size(); ++j) {
            if (!assigned[j]) continue;
            double dx2 = obstacle_points[j].x - cl.center_x;
            double dy2 = obstacle_points[j].y - cl.center_y;
            double d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
            if (d2 < cluster_dist * 2.0) {
              cluster_indices.push_back(j);
            }
          }
          if (cluster_indices.empty()) cluster_indices.push_back(i);
          for (auto idx : cluster_indices) {
            double dx2 = obstacle_points[idx].x - cl.center_x;
            double dy2 = obstacle_points[idx].y - cl.center_y;
            double d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
            if (d2 > cl.radius) cl.radius = d2;
          }
        }
        cl.radius = std::max(0.10, cl.radius + 0.08);
        cl.on_path = on_path;

        obstacle_clusters_.push_back(cl);
      }
    }

    static int cloud_tick = 0;
    cloud_tick++;
    if (cloud_tick % 5 == 1) {
      RCLCPP_DEBUG(get_logger(),
        "[cloud #%d] raw_pts=%zu obstacle_pts=%zu clusters=%zu dist=%.1f hit=%d/%d avoid=%d",
        cloud_tick, raw_pts, obstacle_points.size(),
        obstacle_clusters_.size(), obstacle_min_distance_,
        obstacle_hit_count_, obstacle_confirm_frames_,
        obstacle_avoidance_enabled_ ? 1 : 0);
    }
  }

  Point obstaclePointToMap(double x, double y, double z, const std::string & frame_id) const
  {
    if (frame_id == "map" || frame_id == "camera_init") {
      return Point{x, y, z};
    }

    if (frame_id == "livox_frame" || frame_id == "base_link" || frame_id == "body") {
      double body_x = x;
      double body_y = y;
      double body_z = z;

      if (frame_id == "livox_frame") {
        body_x += lidar_forward_offset_;
        body_y += lidar_left_offset_;
        body_z += lidar_height_;
      }

      const double yaw = yaw_deg_ * M_PI / 180.0;
      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      return Point{
        pose_.x + cos_yaw * body_x - sin_yaw * body_y,
        pose_.y + sin_yaw * body_x + cos_yaw * body_y,
        pose_.z + body_z};
    }

    return Point{x, y, z};
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr route_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr done_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string localization_topic_;
  std::string route_topic_;
  std::string config_topic_;
  std::string start_topic_;
  std::string stop_topic_;
  std::string clear_topic_;
  std::string cmd_vel_topic_;
  std::string state_topic_;
  std::string status_topic_;
  std::string done_topic_;
  double control_frequency_{50.0};

  std::vector<Point> route_;
  std::vector<double> route_cumlen_;
  std::unordered_map<std::int64_t, std::vector<std::size_t>> route_segment_index_;
  Point pose_;
  Point lookahead_target_;
  bool has_lookahead_{false};
  bool localized_{false};
  double yaw_deg_{0.0};

  bool is_auto_moving_{false};
  bool is_obstacle_paused_{false};
  std::string stage_{"idle"};
  std::string status_text_{"C++ 控制器待机"};
  std::size_t nav_progress_idx_{0};
  rclcpp::Time last_obstacle_time_{0, 0, RCL_ROS_TIME};

  double current_vx_{0.0};
  double current_wz_{0.0};
  double final_target_yaw_{0.0};
  bool final_yaw_configured_{false};

  double max_vx_{0.70};
  double max_wz_{0.50};
  double min_vx_{0.12};
  double min_wz_{0.10};
  double arrival_dist_{0.30};
  double arrival_angle_{5.0};
  double alpha_{0.18};
  double obstacle_threshold_{1.5};
  double resume_delay_{0.5};
  double lidar_height_{0.40};
  double lidar_forward_offset_{0.30};
  double lidar_left_offset_{0.0};
  double ground_z_tolerance_{0.12};
  double max_slope_{0.65};
  std::string obstacle_cloud_topic_{"/segmented_cloud_pure"};
  double obstacle_min_distance_{999.0};
  bool is_obstacle_on_path_{false};
  bool obstacle_avoidance_enabled_{false};
  bool obstacle_detour_enabled_{true};
  int obstacle_hit_count_{0};
  int obstacle_confirm_frames_{3};
  double detour_lateral_clearance_{0.85};
  double detour_rejoin_distance_{1.80};
  double detour_entry_distance_{1.20};
  double detour_safety_margin_{0.25};
  bool detour_active_{false};
  std::vector<Point> detour_route_;
  std::vector<double> detour_cumlen_;
  std::size_t detour_progress_idx_{0};
  std::size_t detour_rejoin_idx_{0};

  struct ObstacleBox { double x_min, y_min, x_max, y_max; };
  std::vector<ObstacleBox> obstacle_boxes_;
  struct ObstacleCluster { double center_x, center_y, radius, z_min, z_max; bool on_path; };
  std::vector<ObstacleCluster> obstacle_clusters_;
  double deceleration_alpha_{0.10};
  double lookahead_distance_{1.00};
  int publish_state_tick_{0};
  double path_yaw_kp_{0.020};
  double final_yaw_kp_{0.020};
  double end_slowdown_distance_{1.50};
  double heading_slow_angle_deg_{35.0};
  double rotate_in_place_angle_deg_{45.0};
  double rotate_exit_angle_deg_{15.0};
};

}  // namespace indoor_route_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<indoor_route_nav::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
