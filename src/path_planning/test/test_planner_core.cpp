// Copyright 2026 Physicar contributors

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <vector>

#include "path_planning/planner_core.hpp"

namespace path_planning
{
namespace
{

RddfTrack square_track()
{
  return RddfTrack(
    {{-2.0, -2.0}, {2.0, -2.0}, {2.0, 2.0}, {-2.0, 2.0}},
    {{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}},
    {{-3.0, -3.0}, {3.0, -3.0}, {3.0, 3.0}, {-3.0, 3.0}});
}

std::vector<double> steering_candidates()
{
  const double radians_per_degree = std::acos(-1.0) / 180.0;
  return {
    -30.0 * radians_per_degree, -15.0 * radians_per_degree, 0.0,
    15.0 * radians_per_degree, 30.0 * radians_per_degree,
  };
}

TEST(CoordinateTransformTest, AlignsLocalOdometryOriginAndHeadingToMap)
{
  const Pose2D odom_origin{2.0, -1.0, 0.25};
  const Pose2D map_origin{1.4, 3.4, -std::acos(-1.0) / 2.0};

  const Pose2D mapped_origin =
    transform_pose_between_frames(odom_origin, odom_origin, map_origin);
  EXPECT_NEAR(mapped_origin.x, map_origin.x, 1.0e-12);
  EXPECT_NEAR(mapped_origin.y, map_origin.y, 1.0e-12);
  EXPECT_NEAR(mapped_origin.yaw, map_origin.yaw, 1.0e-12);

  const Pose2D one_meter_forward{
    odom_origin.x + std::cos(odom_origin.yaw),
    odom_origin.y + std::sin(odom_origin.yaw),
    odom_origin.yaw,
  };
  const Pose2D mapped_forward =
    transform_pose_between_frames(one_meter_forward, odom_origin, map_origin);
  EXPECT_NEAR(mapped_forward.x, map_origin.x, 1.0e-12);
  EXPECT_NEAR(mapped_forward.y, map_origin.y - 1.0, 1.0e-12);
  EXPECT_NEAR(mapped_forward.yaw, map_origin.yaw, 1.0e-12);
}

TEST(RddfTrackTest, RepresentsClosedTrackWithoutStartLineSeam)
{
  const RddfTrack track = square_track();

  EXPECT_TRUE(track.contains({0.0, -2.0}));
  EXPECT_TRUE(track.contains({-2.0, -2.0}));
  EXPECT_FALSE(track.contains({0.0, 0.0}));
  EXPECT_FALSE(track.contains({4.0, 0.0}));
  EXPECT_NEAR(track.boundary_distance({0.0, -2.0}), 1.0, 1.0e-9);
  EXPECT_NEAR(track.progress({0.0, -2.0}), 2.0, 1.0e-9);

  // 같은 거리의 평행 segment가 있어도 허용 progress 구간이 위쪽 segment를 고정한다.
  const auto continuous_progress = track.progress_within({0.0, 0.0}, 9.0, 11.0);
  ASSERT_TRUE(continuous_progress.has_value());
  EXPECT_NEAR(*continuous_progress, 10.0, 1.0e-9);

  const GoalGate gate = track.goal_gate_from(0.0, 1.0);
  EXPECT_NEAR(gate.center.x, -1.0, 1.0e-9);
  EXPECT_NEAR(gate.center.y, -2.0, 1.0e-9);
  EXPECT_NEAR(gate.tangent.x, 1.0, 1.0e-9);
  EXPECT_NEAR(gate.tangent.y, 0.0, 1.0e-9);
}

TEST(RddfTrackTest, LoadsRepositoryRddfWithRepeatedBoundarySamples)
{
  const std::filesystem::path repository = std::filesystem::path(__FILE__)
    .parent_path().parent_path().parent_path().parent_path();
  const RddfTrack track = RddfTrack::from_csv((repository / "rddf/rddf.csv").string());

  EXPECT_EQ(track.centerline().size(), 495U);
  EXPECT_NEAR(track.lap_length(), 35.5582, 1.0e-3);
}

TEST(CollisionCheckerTest, EnforcesFourWheelsAndRectangleCircleCollision)
{
  const RddfTrack track = square_track();
  const VehicleFootprint footprint(0.28, 0.20, 0.18, 0.20);
  const CollisionChecker checker(track, footprint, 0.20, 0.05);
  const Pose2D valid_pose{-2.0, -2.0, 0.0};

  EXPECT_TRUE(checker.is_pose_valid(valid_pose, {}));
  EXPECT_FALSE(checker.is_pose_valid(valid_pose, {{-1.75, -2.0, 0.10}}));
  EXPECT_FALSE(checker.is_pose_valid({-2.0, -2.75, 0.0}, {}));

  std::size_t accepted = 0U;
  for (double x = -2.0; x <= 1.5; x += 0.10) {
    for (double y = -2.70; y <= -1.30; y += 0.05) {
      const Pose2D pose{x, y, 0.0};
      if (!checker.is_pose_valid(pose, {})) {
        continue;
      }
      ++accepted;
      for (const Point2D & wheel : footprint.wheel_points(pose)) {
        EXPECT_GE(track.signed_clearance(wheel) + 1.0e-9, 0.20);
      }
    }
  }
  EXPECT_GT(accepted, 0U);
  EXPECT_THROW(
    CollisionChecker(track, footprint, 0.20, 1.0e-12),
    std::length_error);
}

TEST(CostModelTest, UsesTravelDistanceOnly)
{
  const CostModel cost(1.5, 1.5, 1.0, 0.2);
  const double straight = cost.transition_cost(0.10, 0.0, 0.0);
  const double curved = cost.transition_cost(0.10, 1.0, 0.0);
  const double smooth_curve = cost.transition_cost(0.10, 1.0, 1.0);

  EXPECT_NEAR(straight, 0.10, 1.0e-9);
  EXPECT_NEAR(curved, straight, 1.0e-9);
  EXPECT_NEAR(smooth_curve, straight, 1.0e-9);
  EXPECT_NEAR(cost.heuristic(3.0), 3.0, 1.0e-9);
}

TEST(HybridAStarPlannerTest, ValidatesSteeringCandidates)
{
  const RddfTrack track = square_track();
  const CollisionChecker checker(
    track, VehicleFootprint(0.28, 0.20, 0.18, 0.20), 0.20, 0.05);
  const CostModel cost(1.5, 1.5, 1.0, 0.2);
  const auto make_planner = [&](std::vector<double> candidates) {
      return HybridAStarPlanner(
        track, checker, cost,
        0.18, 1.0,
        0.05, 5.0 * std::acos(-1.0) / 180.0, 0.05,
        0.10, 0.025, candidates,
        0.11, 15.0 * std::acos(-1.0) / 180.0, 0.05, 3.0, 10000, false);
    };

  EXPECT_NO_THROW(make_planner({0.0}));
  EXPECT_THROW(make_planner({}), std::invalid_argument);
  EXPECT_THROW(
    make_planner({std::numeric_limits<double>::quiet_NaN()}), std::invalid_argument);
  EXPECT_THROW(make_planner({0.5 * std::acos(-1.0)}), std::invalid_argument);
}

TEST(HybridAStarPlannerTest, PlansForwardAndPreservesFrameAndTreeYaw)
{
  const RddfTrack track = square_track();
  const CollisionChecker checker(
    track, VehicleFootprint(0.28, 0.20, 0.18, 0.20), 0.20, 0.05);
  const CostModel cost(1.5, 1.5, 1.0, 0.2);
  const HybridAStarPlanner planner(
    track, checker, cost,
    0.18, 1.0,
    0.05, 5.0 * std::acos(-1.0) / 180.0, 0.05,
    0.10, 0.025, steering_candidates(),
    0.11, 15.0 * std::acos(-1.0) / 180.0, 0.05, 3.0, 10000, true);
  PlanningSnapshot snapshot;
  snapshot.pose = {-2.0, -2.0, 0.0};
  snapshot.frame_id = "odom";
  snapshot.stamp_nanoseconds = 123456;

  const PlanAttemptResult result = planner.plan(snapshot);

  ASSERT_EQ(result.status, PlanStatus::kSuccess);
  ASSERT_TRUE(result.path.has_value());
  EXPECT_EQ(result.path->frame_id, "odom");
  ASSERT_GT(result.path->points.size(), 1U);
  EXPECT_GE(result.path->points.back().progress, 0.89);
  ASSERT_FALSE(result.tree.empty());
  EXPECT_LE(result.expanded_nodes, result.tree.size());
  ASSERT_GE(result.final_node_index, 0);
  ASSERT_LT(result.final_node_index, static_cast<std::int32_t>(result.tree.size()));
  EXPECT_EQ(result.tree.front().parent_index, -1);
  for (std::size_t i = 1; i < result.tree.size(); ++i) {
    EXPECT_GE(result.tree[i].parent_index, 0);
    EXPECT_LT(result.tree[i].parent_index, static_cast<std::int32_t>(i));
    EXPECT_GE(result.tree[i].yaw, -std::acos(-1.0));
    EXPECT_LT(result.tree[i].yaw, std::acos(-1.0));
  }
}

TEST(HybridAStarPlannerTest, AvoidsObstacleAndSkipsTreeWhenDebugIsDisabled)
{
  const RddfTrack track = square_track();
  const CollisionChecker checker(
    track, VehicleFootprint(0.28, 0.20, 0.18, 0.20), 0.20, 0.05);
  const CostModel cost(1.5, 1.5, 1.0, 0.2);
  const HybridAStarPlanner planner(
    track, checker, cost,
    0.18, 2.0,
    0.05, 5.0 * std::acos(-1.0) / 180.0, 0.05,
    0.10, 0.025, steering_candidates(),
    0.11, 20.0 * std::acos(-1.0) / 180.0, 0.05, 3.0, 50000, false);
  const std::vector<Circle> obstacles{{-1.20, -2.0, 0.15}};

  PlanningSnapshot snapshot;
  snapshot.pose = Pose2D{-2.0, -2.0, 0.0};
  snapshot.obstacles = obstacles;
  snapshot.frame_id = "odom";
  const PlanAttemptResult result = planner.plan(snapshot);

  ASSERT_EQ(result.status, PlanStatus::kSuccess);
  ASSERT_TRUE(result.path.has_value());
  EXPECT_TRUE(result.tree.empty());
  EXPECT_TRUE(std::any_of(
      result.path->points.begin(), result.path->points.end(),
      [](const PathPoint & point) {
        return std::abs(point.curvature) > 1.0e-9;
      }));
  for (const PathPoint & point : result.path->points) {
    EXPECT_TRUE(checker.is_pose_valid({point.x, point.y, point.yaw}, obstacles));
  }
}

TEST(HybridAStarPlannerTest, FailedPlanDoesNotProduceDebugTree)
{
  const RddfTrack track = square_track();
  const CollisionChecker checker(
    track, VehicleFootprint(0.28, 0.20, 0.18, 0.20), 0.20, 0.05);
  const CostModel cost(1.5, 1.5, 1.0, 0.2);
  const HybridAStarPlanner planner(
    track, checker, cost,
    0.18, 2.0,
    0.05, 5.0 * std::acos(-1.0) / 180.0, 0.05,
    0.10, 0.025, steering_candidates(),
    0.11, 15.0 * std::acos(-1.0) / 180.0, 0.05, 3.0, 10000, true);

  const PlanAttemptResult missing_pose_result = planner.plan(PlanningSnapshot{});
  EXPECT_EQ(missing_pose_result.status, PlanStatus::kInvalidStart);
  EXPECT_FALSE(missing_pose_result.path.has_value());
  EXPECT_TRUE(missing_pose_result.tree.empty());

  PlanningSnapshot invalid_start_snapshot;
  invalid_start_snapshot.pose = Pose2D{0.0, 0.0, 0.0};
  invalid_start_snapshot.frame_id = "map";
  const PlanAttemptResult result = planner.plan(invalid_start_snapshot);

  EXPECT_EQ(result.status, PlanStatus::kInvalidStart);
  EXPECT_FALSE(result.path.has_value());
  EXPECT_TRUE(result.tree.empty());

  const std::vector<Circle> blocking_obstacle{{-1.65, -2.0, 0.10}};
  ASSERT_TRUE(checker.is_pose_valid({-2.0, -2.0, 0.0}, blocking_obstacle));
  PlanningSnapshot blocked_snapshot;
  blocked_snapshot.pose = Pose2D{-2.0, -2.0, 0.0};
  blocked_snapshot.obstacles = blocking_obstacle;
  blocked_snapshot.frame_id = "map";
  const PlanAttemptResult blocked_result = planner.plan(blocked_snapshot);

  EXPECT_EQ(blocked_result.status, PlanStatus::kFailure);
  EXPECT_FALSE(blocked_result.path.has_value());
  EXPECT_TRUE(blocked_result.tree.empty());
  EXPECT_EQ(blocked_result.expanded_nodes, 1U);
}

}  // namespace
}  // namespace path_planning
