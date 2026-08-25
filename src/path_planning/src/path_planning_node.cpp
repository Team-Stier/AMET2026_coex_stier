// Copyright 2026 Physicar contributors

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interfaces/msg/objects.hpp>
#include <interfaces/msg/search_tree.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "path_planning/planner_core.hpp"

namespace path_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr char kMapFrameId[] = "map";
struct StampedPose
{
  Pose2D pose;
  rclcpp::Time stamp{std::int64_t{0}, RCL_ROS_TIME};
  std::string frame_id;
};

void require_finite(const char * name, const double value)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string{name} + " must be finite");
  }
}

void require_positive(const char * name, const double value)
{
  require_finite(name, value);
  if (value <= 0.0) {
    throw std::invalid_argument(std::string{name} + " must be greater than zero");
  }
}

void require_nonnegative(const char * name, const double value)
{
  require_finite(name, value);
  if (value < 0.0) {
    throw std::invalid_argument(std::string{name} + " must not be negative");
  }
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(const double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

std::optional<double> yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  if (!std::isfinite(quaternion.x) || !std::isfinite(quaternion.y) ||
    !std::isfinite(quaternion.z) || !std::isfinite(quaternion.w))
  {
    return std::nullopt;
  }

  const double norm_squared =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  if (!std::isfinite(norm_squared) || norm_squared <= std::numeric_limits<double>::epsilon()) {
    return std::nullopt;
  }

  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  const double x = quaternion.x * inverse_norm;
  const double y = quaternion.y * inverse_norm;
  const double z = quaternion.z * inverse_norm;
  const double w = quaternion.w * inverse_norm;
  return normalize_yaw(std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)));
}

class PlanningRegistry
{
public:
  void update_pose(StampedPose pose)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_pose_ = std::move(pose);
  }

  std::optional<StampedPose> latest_pose() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_pose_;
  }

  void replace_obstacles(std::vector<Circle> obstacles)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacles_ = std::move(obstacles);
  }

  PlanningSnapshot planning_snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    PlanningSnapshot snapshot;
    snapshot.obstacles = obstacles_;
    if (latest_pose_) {
      snapshot.pose = latest_pose_->pose;
      snapshot.frame_id = latest_pose_->frame_id;
      snapshot.stamp_nanoseconds = latest_pose_->stamp.nanoseconds();
    }
    return snapshot;
  }

  std::shared_ptr<const PlannedPath> current_path() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_path_;
  }

  void commit_path(PlannedPath path)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_path_ = std::make_shared<const PlannedPath>(std::move(path));
  }

private:
  mutable std::mutex mutex_;
  std::optional<StampedPose> latest_pose_;
  std::vector<Circle> obstacles_;
  std::shared_ptr<const PlannedPath> latest_path_;
};

class LocalPathPublisher
{
public:
  LocalPathPublisher(const PlanningRegistry & registry, const double local_path_length_m)
  : registry_(registry), local_path_length_(local_path_length_m)
  {
  }

  nav_msgs::msg::Path slice(const StampedPose & current_pose) const
  {
    nav_msgs::msg::Path message;
    message.header.stamp = current_pose.stamp;
    message.header.frame_id = current_pose.frame_id;

    const std::shared_ptr<const PlannedPath> path = registry_.current_path();
    if (!path || path->points.empty() || path->frame_id != current_pose.frame_id) {
      return message;
    }

    std::size_t nearest_index = 0;
    double nearest_squared_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < path->points.size(); ++index) {
      const double dx = path->points[index].x - current_pose.pose.x;
      const double dy = path->points[index].y - current_pose.pose.y;
      const double squared_distance = dx * dx + dy * dy;
      if (squared_distance < nearest_squared_distance) {
        nearest_squared_distance = squared_distance;
        nearest_index = index;
      }
    }

    message.poses.reserve(path->points.size() - nearest_index);
    double accumulated_length = 0.0;
    for (std::size_t index = nearest_index; index < path->points.size(); ++index) {
      if (index > nearest_index) {
        const PathPoint & previous = path->points[index - 1];
        const PathPoint & current = path->points[index];
        accumulated_length += std::hypot(current.x - previous.x, current.y - previous.y);
      }

      const PathPoint & point = path->points[index];
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation = quaternion_from_yaw(point.yaw);
      message.poses.push_back(std::move(pose));

      if (accumulated_length >= local_path_length_) {
        break;
      }
    }
    return message;
  }

private:
  const PlanningRegistry & registry_;
  const double local_path_length_;
};

class PlanningWorker
{
public:
  using AttemptCallback =
    std::function<void(const PlanningSnapshot &, const PlanAttemptResult &)>;

  PlanningWorker(
    const HybridAStarPlanner & planner,
    PlanningRegistry & registry,
    AttemptCallback attempt_callback,
    rclcpp::Logger logger)
  : planner_(planner),
    registry_(registry),
    attempt_callback_(std::move(attempt_callback)),
    logger_(std::move(logger)),
    thread_(&PlanningWorker::run, this)
  {
  }

  ~PlanningWorker()
  {
    stop();
  }

  PlanningWorker(const PlanningWorker &) = delete;
  PlanningWorker & operator=(const PlanningWorker &) = delete;

  void enable()
  {
    {
      std::lock_guard<std::mutex> lock(gate_mutex_);
      enabled_ = true;
    }
    gate_condition_.notify_one();
  }

private:
  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(gate_mutex_);
      stopping_ = true;
    }
    gate_condition_.notify_one();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void run()
  {
    {
      std::unique_lock<std::mutex> lock(gate_mutex_);
      gate_condition_.wait(lock, [this]() {return stopping_ || enabled_;});
    }

    while (!stopping_) {
      PlanningSnapshot snapshot = registry_.planning_snapshot();
      if (!snapshot.pose) {
        std::this_thread::yield();
        continue;
      }

      const auto planning_started = std::chrono::steady_clock::now();
      PlanAttemptResult result;
      try {
        result = planner_.plan(snapshot);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(logger_, "planning attempt failed with exception: %s", error.what());
        continue;
      } catch (...) {
        RCLCPP_ERROR(logger_, "planning attempt failed with an unknown exception");
        continue;
      }
      const double planning_elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - planning_started).count();
      RCLCPP_INFO(logger_, "planning completed in %.3f ms", planning_elapsed_ms);

      if (result.status == PlanStatus::kSuccess && result.path) {
        try {
          PlannedPath path = std::move(*result.path);
          path.frame_id = snapshot.frame_id;
          registry_.commit_path(std::move(path));
        } catch (const std::exception & error) {
          RCLCPP_ERROR(logger_, "could not commit planned path: %s", error.what());
          continue;
        } catch (...) {
          RCLCPP_ERROR(logger_, "could not commit planned path: unknown exception");
          continue;
        }

        if (attempt_callback_) {
          try {
            attempt_callback_(snapshot, result);
          } catch (const std::exception & error) {
            RCLCPP_ERROR(logger_, "planning success callback failed: %s", error.what());
          } catch (...) {
            RCLCPP_ERROR(logger_, "planning success callback failed with an unknown exception");
          }
        }
      }
    }
  }

  const HybridAStarPlanner & planner_;
  PlanningRegistry & registry_;
  AttemptCallback attempt_callback_;
  rclcpp::Logger logger_;
  std::mutex gate_mutex_;
  std::condition_variable gate_condition_;
  bool enabled_{false};
  std::atomic_bool stopping_{false};
  std::thread thread_;
};

class PathPlanningNode : public rclcpp::Node
{
public:
  PathPlanningNode()
  : Node("path_planning_node")
  {
    load_parameters();
    validate_parameters();

    std::filesystem::path rddf_path{rddf_file_};
    if (rddf_path.is_relative()) {
      rddf_path =
        std::filesystem::path{ament_index_cpp::get_package_share_directory("path_planning")} /
      rddf_path;
    }
    if (!std::filesystem::is_regular_file(rddf_path)) {
      throw std::invalid_argument("rddf_file is not a regular file: " + rddf_path.string());
    }

    track_ = std::make_unique<RddfTrack>(RddfTrack::from_csv(rddf_path.string()));
    collision_checker_ = std::make_unique<CollisionChecker>(
      *track_,
      VehicleFootprint(vehicle_length_m_, vehicle_width_m_, wheelbase_m_, wheel_track_m_),
      track_lookup_resolution_m_);
    cost_model_ = std::make_unique<CostModel>(select_cost_function(select_function_));
    std::vector<double> steering_candidates_rad;
    steering_candidates_rad.reserve(steering_candidates_deg_.size());
    for (const double steering_deg : steering_candidates_deg_) {
      steering_candidates_rad.push_back(steering_deg * kPi / 180.0);
    }
    planner_ = std::make_unique<HybridAStarPlanner>(
      *track_, *collision_checker_, *cost_model_, wheelbase_m_, planning_horizon_m_,
      xy_resolution_m_, yaw_resolution_deg_ * kPi / 180.0, progress_resolution_m_,
      motion_primitive_length_m_, collision_check_step_m_,
      vehicle_max_steering_deg_ * kPi / 180.0, std::move(steering_candidates_rad),
      goal_longitudinal_tolerance_m_,
      goal_yaw_tolerance_deg_ * kPi / 180.0, progress_regression_tolerance_m_,
      max_progress_advance_ratio_, static_cast<std::size_t>(max_search_nodes_),
      publish_search_tree_debug_);
    registry_ = std::make_unique<PlanningRegistry>();
    local_path_publisher_ =
      std::make_unique<LocalPathPublisher>(*registry_, local_path_length_m_);

    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/path", rclcpp::QoS(10));
    if (publish_search_tree_debug_) {
      rclcpp::QoS debug_qos{rclcpp::KeepLast(1)};
      debug_qos.best_effort().durability_volatile();
      search_tree_publisher_ =
        create_publisher<interfaces::msg::SearchTree>(
        "/path_planning/debug/search_tree", debug_qos);
    }

    object_subscription_ = create_subscription<interfaces::msg::Objects>(
      "/object_info", rclcpp::QoS(10),
      std::bind(&PathPlanningNode::on_object_info, this, std::placeholders::_1));

    const std::string selected_pose_topic =
      use_calibride_odom_ ? "/pose/calibration" : "/pose";
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      selected_pose_topic, rclcpp::SensorDataQoS(),
      std::bind(&PathPlanningNode::on_selected_odometry, this, std::placeholders::_1));

    PlanningWorker::AttemptCallback attempt_callback;
    if (publish_search_tree_debug_) {
      attempt_callback =
        [this](const PlanningSnapshot & snapshot, const PlanAttemptResult & result) {
          publish_search_tree(snapshot, result);
        };
    }
    planning_worker_ = std::make_unique<PlanningWorker>(
      *planner_, *registry_, std::move(attempt_callback), get_logger());
    gosign_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/gosign", rclcpp::QoS(10),
      std::bind(&PathPlanningNode::on_gosign, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "waiting for /gosign=true; using %s with RDDF %s",
      selected_pose_topic.c_str(),
      rddf_path.string().c_str());
  }

  ~PathPlanningNode() override
  {
    gosign_subscription_.reset();
    odometry_subscription_.reset();
    object_subscription_.reset();
    planning_worker_.reset();
  }

private:
  template<typename T>
  T required_parameter(const char * name)
  {
    return declare_parameter<T>(name);
  }

public:
  void load_parameters()
  {
    rddf_file_ = required_parameter<std::string>("rddf_file");
    use_calibride_odom_ = required_parameter<bool>("use_calibride_odom");
    vehicle_width_m_ = required_parameter<double>("vehicle_width_m");
    vehicle_length_m_ = required_parameter<double>("vehicle_length_m");
    wheelbase_m_ = required_parameter<double>("wheelbase_m");
    vehicle_max_steering_deg_ = required_parameter<double>("vehicle_max_steering_deg");
    wheel_track_m_ = required_parameter<double>("wheel_track_m");
    obstacle_inflation_radius_m_ =
      required_parameter<double>("obstacle_inflation_radius_m");
    track_lookup_resolution_m_ = required_parameter<double>("track_lookup_resolution_m");
    publish_search_tree_debug_ = required_parameter<bool>("publish_search_tree_debug");
    planning_horizon_m_ = required_parameter<double>("planning_horizon_m");
    local_path_length_m_ = required_parameter<double>("local_path_length_m");
    xy_resolution_m_ = required_parameter<double>("xy_resolution_m");
    yaw_resolution_deg_ = required_parameter<double>("yaw_resolution_deg");
    collision_check_step_m_ = required_parameter<double>("collision_check_step_m");
    motion_primitive_length_m_ =
      required_parameter<double>("motion_primitive_length_m");
    steering_candidates_deg_ = required_parameter<std::vector<double>>("steering_candidates_deg");
    progress_resolution_m_ = required_parameter<double>("progress_resolution_m");
    goal_longitudinal_tolerance_m_ =
      required_parameter<double>("goal_longitudinal_tolerance_m");
    goal_yaw_tolerance_deg_ = required_parameter<double>("goal_yaw_tolerance_deg");
    progress_regression_tolerance_m_ =
      required_parameter<double>("progress_regression_tolerance_m");
    max_progress_advance_ratio_ =
      required_parameter<double>("max_progress_advance_ratio");
    max_search_nodes_ = required_parameter<std::int64_t>("max_search_nodes");
    select_function_ = required_parameter<std::string>("select_function");
  }

private:
  void validate_parameters()
  {
    if (rddf_file_.empty()) {
      throw std::invalid_argument("rddf_file must not be empty");
    }
    require_positive("vehicle_width_m", vehicle_width_m_);
    require_positive("vehicle_length_m", vehicle_length_m_);
    require_positive("wheelbase_m", wheelbase_m_);
    require_positive("vehicle_max_steering_deg", vehicle_max_steering_deg_);
    require_positive("wheel_track_m", wheel_track_m_);
    require_positive("obstacle_inflation_radius_m", obstacle_inflation_radius_m_);
    require_positive("track_lookup_resolution_m", track_lookup_resolution_m_);
    require_positive("planning_horizon_m", planning_horizon_m_);
    require_positive("local_path_length_m", local_path_length_m_);
    require_positive("xy_resolution_m", xy_resolution_m_);
    require_positive("yaw_resolution_deg", yaw_resolution_deg_);
    require_positive("collision_check_step_m", collision_check_step_m_);
    require_positive("motion_primitive_length_m", motion_primitive_length_m_);
    require_positive("progress_resolution_m", progress_resolution_m_);
    require_positive("goal_longitudinal_tolerance_m", goal_longitudinal_tolerance_m_);
    require_positive("goal_yaw_tolerance_deg", goal_yaw_tolerance_deg_);
    require_nonnegative(
      "progress_regression_tolerance_m", progress_regression_tolerance_m_);
    require_positive("max_progress_advance_ratio", max_progress_advance_ratio_);
    if (select_function_.empty()) {
      throw std::invalid_argument("select_function must not be empty");
    }

    if (steering_candidates_deg_.empty()) {
      throw std::invalid_argument("steering_candidates_deg must not be empty");
    }
    if (vehicle_max_steering_deg_ >= 90.0) {
      throw std::invalid_argument("vehicle_max_steering_deg must be less than 90 degrees");
    }
    for (const double steering_deg : steering_candidates_deg_) {
      if (!std::isfinite(steering_deg) || std::abs(steering_deg) >= 90.0) {
        throw std::invalid_argument(
                "steering_candidates_deg values must be finite and between -90 and 90");
      }
      if (std::abs(steering_deg) > vehicle_max_steering_deg_) {
        throw std::invalid_argument(
                "steering_candidates_deg values must not exceed vehicle_max_steering_deg");
      }
    }
    if (max_search_nodes_ <= 0) {
      throw std::invalid_argument("max_search_nodes must be greater than zero");
    }
    if (max_progress_advance_ratio_ < 1.0) {
      throw std::invalid_argument("max_progress_advance_ratio must be at least 1.0");
    }
    if (wheelbase_m_ > vehicle_length_m_) {
      throw std::invalid_argument("wheelbase_m must not exceed vehicle_length_m");
    }
    if (wheel_track_m_ > vehicle_width_m_) {
      throw std::invalid_argument("wheel_track_m must not exceed vehicle_width_m");
    }
    if (collision_check_step_m_ > motion_primitive_length_m_) {
      throw std::invalid_argument(
              "collision_check_step_m must not exceed motion_primitive_length_m");
    }
    if (yaw_resolution_deg_ > 180.0 || goal_yaw_tolerance_deg_ > 180.0) {
      throw std::invalid_argument("yaw angles must not exceed 180 degrees");
    }
    if (static_cast<std::uint64_t>(max_search_nodes_) >
      std::numeric_limits<std::size_t>::max())
    {
      throw std::invalid_argument("max_search_nodes exceeds size_t");
    }
  }

public:
  void on_gosign(const std_msgs::msg::Bool::ConstSharedPtr message)
  {
    if (!message->data) {
      return;
    }
    planning_worker_->enable();
  }

  void on_selected_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    const auto & position = message->pose.pose.position;
    const std::optional<double> yaw = yaw_from_quaternion(message->pose.pose.orientation);
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !yaw ||
      message->header.frame_id != kMapFrameId)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "discarding invalid selected odometry");
      return;
    }

    rclcpp::Time stamp{std::int64_t{0}, RCL_ROS_TIME};
    try {
      stamp = rclcpp::Time(message->header.stamp, RCL_ROS_TIME);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "discarding odometry with invalid stamp: %s", error.what());
      return;
    }

    StampedPose pose;
    pose.pose = {position.x, position.y, *yaw};
    pose.stamp = stamp;
    pose.frame_id = kMapFrameId;
    registry_->update_pose(pose);
    path_publisher_->publish(local_path_publisher_->slice(pose));
  }

  void on_object_info(const interfaces::msg::Objects::ConstSharedPtr message)
  {
    if (message->length < 0 ||
      message->length > static_cast<std::int32_t>(message->x.size()) ||
      message->length > static_cast<std::int32_t>(message->y.size()))
    {
      RCLCPP_WARN(get_logger(), "discarding Objects with invalid length %d", message->length);
      return;
    }

    const std::optional<StampedPose> latest_pose = registry_->latest_pose();
    if (!latest_pose) {
      RCLCPP_WARN(get_logger(), "discarding Objects before selected odometry is available");
      return;
    }

    std::vector<Circle> obstacles;
    obstacles.reserve(static_cast<std::size_t>(message->length));
    const double cosine = std::cos(latest_pose->pose.yaw);
    const double sine = std::sin(latest_pose->pose.yaw);
    for (std::int32_t index = 0; index < message->length; ++index) {
      const double local_x = message->x[static_cast<std::size_t>(index)];
      const double local_y = message->y[static_cast<std::size_t>(index)];
      if (!std::isfinite(local_x) || !std::isfinite(local_y)) {
        RCLCPP_WARN(get_logger(), "discarding Objects with a non-finite coordinate");
        return;
      }
      const double global_x =
        latest_pose->pose.x + cosine * local_x - sine * local_y;
      const double global_y =
        latest_pose->pose.y + sine * local_x + cosine * local_y;
      if (!std::isfinite(global_x) || !std::isfinite(global_y)) {
        RCLCPP_WARN(get_logger(), "discarding Objects after coordinate transform overflow");
        return;
      }
      obstacles.push_back(Circle{global_x, global_y, obstacle_inflation_radius_m_});
    }

    registry_->replace_obstacles(std::move(obstacles));
  }

  void publish_search_tree(
    const PlanningSnapshot & snapshot,
    const PlanAttemptResult & result)
  {
    if (!publish_search_tree_debug_ || !search_tree_publisher_) {
      return;
    }

    interfaces::msg::SearchTree message;
    message.header.stamp = rclcpp::Time(snapshot.stamp_nanoseconds, RCL_ROS_TIME);
    message.header.frame_id = snapshot.frame_id;
    message.x.reserve(result.tree.size());
    message.y.reserve(result.tree.size());
    message.yaw.reserve(result.tree.size());
    message.parent_index.reserve(result.tree.size());
    message.final_node_index = result.final_node_index;
    for (const TreeNode & node : result.tree) {
      message.x.push_back(static_cast<float>(node.x));
      message.y.push_back(static_cast<float>(node.y));
      message.yaw.push_back(static_cast<float>(normalize_yaw(node.yaw)));
      message.parent_index.push_back(node.parent_index);
    }
    search_tree_publisher_->publish(std::move(message));
  }

private:
  std::string rddf_file_;
  bool use_calibride_odom_{false};
  double vehicle_width_m_{};
  double vehicle_length_m_{};
  double wheelbase_m_{};
  double vehicle_max_steering_deg_{};
  double wheel_track_m_{};
  double obstacle_inflation_radius_m_{};
  double track_lookup_resolution_m_{};
  bool publish_search_tree_debug_{false};
  double planning_horizon_m_{};
  double local_path_length_m_{};
  double xy_resolution_m_{};
  double yaw_resolution_deg_{};
  double collision_check_step_m_{};
  double motion_primitive_length_m_{};
  std::vector<double> steering_candidates_deg_;
  double progress_resolution_m_{};
  double goal_longitudinal_tolerance_m_{};
  double goal_yaw_tolerance_deg_{};
  double progress_regression_tolerance_m_{};
  double max_progress_advance_ratio_{};
  std::int64_t max_search_nodes_{};
  std::string select_function_;
  std::unique_ptr<RddfTrack> track_;
  std::unique_ptr<CollisionChecker> collision_checker_;
  std::unique_ptr<CostModel> cost_model_;
  std::unique_ptr<HybridAStarPlanner> planner_;
  std::unique_ptr<PlanningRegistry> registry_;
  std::unique_ptr<LocalPathPublisher> local_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<interfaces::msg::SearchTree>::SharedPtr search_tree_publisher_;
  rclcpp::Subscription<interfaces::msg::Objects>::SharedPtr object_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gosign_subscription_;
  std::unique_ptr<PlanningWorker> planning_worker_;
};

}  // namespace
}  // namespace path_planning

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<path_planning::PathPlanningNode>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("path_planning_node"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
