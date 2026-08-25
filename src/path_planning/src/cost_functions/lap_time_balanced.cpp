// Copyright 2026 Physicar contributors

#include <algorithm>
#include <cmath>

namespace path_planning::cost_functions
{

namespace
{

constexpr double kTargetSpeedMps = 0.85;
constexpr double kMaximumLateralAccelerationMps2 = 0.60;
constexpr double kReferenceMaximumCurvatureInvM = 1.805109423516146;
constexpr double kPeakCurvatureWeight = 0.18;
constexpr double kCurvatureEpsilonInvM = 1.0e-9;

}  // namespace

double lap_time_balanced_transition(
  double distance_m, double curvature_inv_m, double) noexcept
{
  const double absolute_curvature = std::abs(curvature_inv_m);
  const double curve_speed = absolute_curvature <= kCurvatureEpsilonInvM ?
    kTargetSpeedMps :
    std::min(
    kTargetSpeedMps,
    std::sqrt(kMaximumLateralAccelerationMps2 / absolute_curvature));
  const double travel_time = distance_m / curve_speed;
  const double normalized_curvature =
    absolute_curvature / kReferenceMaximumCurvatureInvM;
  const double normalized_squared = normalized_curvature * normalized_curvature;
  return travel_time *
         (1.0 + kPeakCurvatureWeight * normalized_squared * normalized_squared);
}

double lap_time_balanced_heuristic(double minimum_distance_m) noexcept
{
  return std::max(0.0, minimum_distance_m) / kTargetSpeedMps;
}

}  // namespace path_planning::cost_functions
