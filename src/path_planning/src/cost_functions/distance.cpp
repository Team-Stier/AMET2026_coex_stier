// Copyright 2026 Physicar contributors

#include <algorithm>

namespace path_planning::cost_functions
{

double distance_transition(double distance, double, double) noexcept
{
  return distance;
}

double distance_heuristic(double minimum_distance) noexcept
{
  return std::max(0.0, minimum_distance);
}

}  // namespace path_planning::cost_functions
