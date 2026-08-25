// Copyright 2026 Physicar contributors

#include "path_planning/cost_function.hpp"

#include <stdexcept>

namespace path_planning
{
namespace cost_functions
{

double distance_transition(double, double, double) noexcept;
double distance_heuristic(double) noexcept;
double min_curvature_transition(double, double, double) noexcept;
double min_curvature_heuristic(double) noexcept;

}  // namespace cost_functions

const CostFunction & select_cost_function(const std::string & filename)
{
  static const CostFunction distance{
    cost_functions::distance_transition,
    cost_functions::distance_heuristic,
  };
  static const CostFunction min_curvature{
    cost_functions::min_curvature_transition,
    cost_functions::min_curvature_heuristic,
  };

  if (filename == "distance.cpp") {
    return distance;
  }
  if (filename == "min_curvature.cpp") {
    return min_curvature;
  }
  throw std::invalid_argument(
          "unknown select_function '" + filename +
          "' (expected distance.cpp or min_curvature.cpp)");
}

}  // namespace path_planning
