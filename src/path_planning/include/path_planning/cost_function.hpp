// Copyright 2026 Physicar contributors

#pragma once

#include <string>

namespace path_planning
{

using TransitionCostFunction = double (*)(double, double, double) noexcept;
using HeuristicCostFunction = double (*)(double) noexcept;

struct CostFunction
{
  TransitionCostFunction transition_cost{};
  HeuristicCostFunction heuristic{};
};

const CostFunction & select_cost_function(const std::string & filename);

}  // namespace path_planning
