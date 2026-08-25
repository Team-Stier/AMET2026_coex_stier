// Copyright 2026 Physicar contributors

#include <algorithm>

namespace path_planning::cost_functions
{

double min_curvature_transition(
  double distance,
  double curvature,
  double previous_curvature) noexcept
{
  constexpr double bending_weight = 0.10;
  constexpr double peak_curvature_weight = 0.0025;
  constexpr double curvature_rate_weight = 0.01;
  const double curvature_squared = curvature * curvature;
  const double curvature_change = curvature - previous_curvature;
  return distance +
         bending_weight * curvature_squared * distance +
         peak_curvature_weight * curvature_squared * curvature_squared * distance +
         curvature_rate_weight * curvature_change * curvature_change / distance;
}

double min_curvature_heuristic(double minimum_distance) noexcept
{
  return std::max(0.0, minimum_distance);
}

}  // namespace path_planning::cost_functions
