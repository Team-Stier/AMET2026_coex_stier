// Copyright 2026 Physicar contributors

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace path_planning
{

struct Point2D
{
  double x{};
  double y{};
};

struct Pose2D
{
  double x{};
  double y{};
  double yaw{};
};

struct Circle
{
  double x{};
  double y{};
  double radius{};
};

struct PathPoint
{
  double x{};
  double y{};
  double yaw{};
  double curvature{};
  double progress{};
};

struct PlannedPath
{
  std::vector<PathPoint> points;
  std::string frame_id;
};

struct PlanningSnapshot
{
  std::optional<Pose2D> pose;
  std::vector<Circle> obstacles;
  std::string frame_id;
  std::int64_t stamp_nanoseconds{};
};

struct TreeNode
{
  double x{};
  double y{};
  double yaw{};
  std::int32_t parent_index{-1};
};

enum class PlanStatus
{
  kSuccess,
  kFailure,
  kInvalidStart,
};

struct PlanAttemptResult
{
  PlanStatus status{PlanStatus::kFailure};
  std::optional<PlannedPath> path;
  std::vector<TreeNode> tree;
  std::int32_t final_node_index{-1};
  std::size_t expanded_nodes{};
};

struct GoalGate
{
  Point2D center;
  Point2D inner;
  Point2D outer;
  Point2D tangent;
  double progress{};
};

double normalize_yaw(double yaw) noexcept;
Pose2D transform_pose_between_frames(
  const Pose2D & pose,
  const Pose2D & source_origin,
  const Pose2D & target_origin) noexcept;

class RddfTrack
{
public:
  RddfTrack(
    std::vector<Point2D> centerline,
    std::vector<Point2D> inner_boundary,
    std::vector<Point2D> outer_boundary);

  static RddfTrack from_csv(const std::string & csv_path);

  bool contains(const Point2D & point) const noexcept;
  bool contains(const Point2D & point, double boundary_margin_m) const noexcept;
  double boundary_distance(const Point2D & point) const noexcept;
  double signed_clearance(const Point2D & point) const noexcept;
  double progress(const Point2D & point) const noexcept;
  double progress(const Point2D & point, double reference_progress) const noexcept;
  std::optional<double> progress_within(
    const Point2D & point,
    double minimum_progress,
    double maximum_progress) const noexcept;
  GoalGate goal_gate_from(double start_progress, double horizon_m) const;
  double lap_length() const noexcept;

  const std::vector<Point2D> & centerline() const noexcept;
  const std::vector<Point2D> & inner_boundary() const noexcept;
  const std::vector<Point2D> & outer_boundary() const noexcept;

private:
  struct TrackSample
  {
    Point2D center;
    Point2D inner;
    Point2D outer;
    Point2D tangent;
  };

  struct RingIndex
  {
    double minimum_y{};
    double maximum_y{};
    double bin_height{};
    std::vector<std::vector<std::size_t>> bins;
  };

  static RingIndex build_ring_index(const std::vector<Point2D> & ring);
  static bool point_inside_ring(
    const Point2D & point,
    const std::vector<Point2D> & ring,
    const RingIndex & index) noexcept;
  static bool edge_within_distance(
    const Point2D & point,
    double distance_m,
    const std::vector<Point2D> & ring,
    const RingIndex & index) noexcept;
  TrackSample sample(double wrapped_progress) const;
  double wrapped_progress(const Point2D & point) const noexcept;

  std::vector<Point2D> centerline_;
  std::vector<Point2D> inner_boundary_;
  std::vector<Point2D> outer_boundary_;
  std::vector<double> cumulative_length_;
  double lap_length_{};
  RingIndex inner_index_;
  RingIndex outer_index_;
};

class VehicleFootprint
{
public:
  VehicleFootprint(
    double vehicle_length_m,
    double vehicle_width_m,
    double wheelbase_m,
    double wheel_track_m);

  std::array<Point2D, 4> wheel_points(const Pose2D & pose) const noexcept;
  bool body_intersects(const Circle & circle, const Pose2D & pose) const noexcept;
  double body_clearance(const Circle & circle, const Pose2D & pose) const noexcept;

private:
  double longitudinal_min_{};
  double longitudinal_max_{};
  double half_width_{};
  double wheelbase_{};
  double half_wheel_track_{};
};

class CollisionChecker
{
public:
  CollisionChecker(
    const RddfTrack & track,
    VehicleFootprint footprint,
    double track_margin_m,
    double track_lookup_resolution_m);

  bool is_pose_valid(
    const Pose2D & pose,
    const std::vector<Circle> & obstacles) const noexcept;
  bool is_primitive_valid(
    const std::vector<Pose2D> & primitive,
    const std::vector<Circle> & obstacles) const noexcept;

private:
  bool wheel_is_inside_track(const Point2D & wheel) const noexcept;

  const RddfTrack & track_;
  VehicleFootprint footprint_;
  double track_margin_{};
  double lookup_resolution_{};
  double lookup_minimum_x_{};
  double lookup_minimum_y_{};
  double lookup_maximum_x_{};
  double lookup_maximum_y_{};
  std::size_t lookup_columns_{};
  std::size_t lookup_rows_{};
  std::vector<double> track_clearance_lookup_;
};

class CostModel
{
public:
  CostModel(
    double max_speed_mps,
    double max_lateral_accel_mps2,
    double w_curvature,
    double w_curvature_change);

  double expected_speed(double curvature) const noexcept;
  double transition_cost(
    double distance_m,
    double curvature,
    double previous_curvature) const noexcept;
  double heuristic(double minimum_travel_distance_m) const noexcept;

private:
  double max_speed_{};
  double max_lateral_accel_{};
  double w_curvature_{};
  double w_curvature_change_{};
};

class HybridAStarPlanner
{
public:
  HybridAStarPlanner(
    const RddfTrack & track,
    const CollisionChecker & collision_checker,
    const CostModel & cost_model,
    double wheelbase_m,
    double planning_horizon_m,
    double xy_resolution_m,
    double yaw_resolution_rad,
    double progress_resolution_m,
    double primitive_length_m,
    double collision_check_step_m,
    std::vector<double> steering_candidates_rad,
    double goal_longitudinal_tolerance_m,
    double goal_yaw_tolerance_rad,
    double progress_regression_tolerance_m,
    double max_progress_advance_ratio,
    std::size_t max_search_nodes,
    bool collect_search_tree);

  PlanAttemptResult plan(const PlanningSnapshot & snapshot) const;

private:
  const RddfTrack & track_;
  const CollisionChecker & collision_checker_;
  const CostModel & cost_model_;
  double wheelbase_{};
  double planning_horizon_{};
  double xy_resolution_{};
  double yaw_resolution_{};
  double progress_resolution_{};
  double primitive_length_{};
  double collision_check_step_{};
  std::vector<double> curvatures_;
  double goal_longitudinal_tolerance_{};
  double goal_yaw_tolerance_{};
  double progress_regression_tolerance_{};
  double max_progress_advance_ratio_{};
  std::size_t max_search_nodes_{};
  bool collect_search_tree_{};
};

}  // namespace path_planning
