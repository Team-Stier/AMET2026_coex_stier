// Copyright 2026 Physicar contributors

#include "path_planning/cost_function.hpp"

#include <stdexcept>

namespace path_planning
{
namespace cost_functions
{

double distance_transition(double, double, double) noexcept;
double distance_heuristic(double) noexcept;
double lap_time_balanced_transition(double, double, double) noexcept;
double lap_time_balanced_heuristic(double) noexcept;
double lap_time_fast_transition(double, double, double) noexcept;
double lap_time_fast_heuristic(double) noexcept;
double lap_time_qualifying_transition(double, double, double) noexcept;
double lap_time_qualifying_heuristic(double) noexcept;
double lap_time_safe_transition(double, double, double) noexcept;
double lap_time_safe_heuristic(double) noexcept;
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
  static const CostFunction lap_time_balanced{
    cost_functions::lap_time_balanced_transition,
    cost_functions::lap_time_balanced_heuristic,
  };
  static const CostFunction lap_time_fast{
    cost_functions::lap_time_fast_transition,
    cost_functions::lap_time_fast_heuristic,
  };
  static const CostFunction lap_time_qualifying{
    cost_functions::lap_time_qualifying_transition,
    cost_functions::lap_time_qualifying_heuristic,
  };
  static const CostFunction lap_time_safe{
    cost_functions::lap_time_safe_transition,
    cost_functions::lap_time_safe_heuristic,
  };

  if (filename == "distance.cpp") {
    return distance;
  }
  if (filename == "min_curvature.cpp") {
    return min_curvature;
  }
  if (filename == "lap_time_balanced.cpp") {
    return lap_time_balanced;
  }
  if (filename == "lap_time_fast.cpp") {
    return lap_time_fast;
  }
  if (filename == "lap_time_qualifying.cpp") {
    return lap_time_qualifying;
  }
  if (filename == "lap_time_safe.cpp") {
    return lap_time_safe;
  }
  throw std::invalid_argument(
          "unknown select_function '" + filename +
          "' (expected a compiled file from src/cost_functions)");
}

}  // namespace path_planning
