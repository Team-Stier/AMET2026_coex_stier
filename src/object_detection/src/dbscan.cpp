#include "object_detection/dbscan.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

namespace object_detection
{
namespace
{

constexpr int kNoise = -1;
constexpr int kUnvisited = -2;
constexpr double kContainmentToleranceM = 1.0e-9;

void validate_scan_config(const ScanFilterConfig & config)
{
  if (!std::isfinite(config.minimum_range_m) || config.minimum_range_m < 0.0) {
    throw std::invalid_argument("minimum_range_m must be finite and non-negative");
  }
  if (!std::isfinite(config.maximum_range_m) ||
    config.maximum_range_m <= config.minimum_range_m)
  {
    throw std::invalid_argument("maximum_range_m must exceed minimum_range_m");
  }
  if (!std::isfinite(config.minimum_forward_x_m)) {
    throw std::invalid_argument("minimum_forward_x_m must be finite");
  }
  if (!std::isfinite(config.maximum_absolute_y_m) || config.maximum_absolute_y_m <= 0.0) {
    throw std::invalid_argument("maximum_absolute_y_m must be finite and positive");
  }
}

void validate_dbscan_config(double epsilon_m, std::size_t minimum_samples)
{
  if (!std::isfinite(epsilon_m) || epsilon_m <= 0.0) {
    throw std::invalid_argument("epsilon_m must be finite and positive");
  }
  if (minimum_samples == 0U) {
    throw std::invalid_argument("minimum_samples must be at least one");
  }
}

std::vector<std::size_t> region_query(
  const std::vector<Point2D> & points,
  std::size_t query_index,
  double epsilon_squared)
{
  std::vector<std::size_t> neighbors;
  neighbors.reserve(points.size());
  const auto & query = points[query_index];
  for (std::size_t index = 0; index < points.size(); ++index) {
    const double delta_x = points[index].x - query.x;
    const double delta_y = points[index].y - query.y;
    if (delta_x * delta_x + delta_y * delta_y <= epsilon_squared) {
      neighbors.push_back(index);
    }
  }
  return neighbors;
}

double squared_distance_from_origin(const Cluster & cluster)
{
  return cluster.center.x * cluster.center.x + cluster.center.y * cluster.center.y;
}

double distance(const Point2D & first, const Point2D & second)
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

bool contains(const EnclosingCircle & circle, const Point2D & point)
{
  return circle.radius_m >= 0.0 &&
         distance(circle.center, point) <= circle.radius_m + kContainmentToleranceM;
}

EnclosingCircle circle_from_two_points(const Point2D & first, const Point2D & second)
{
  const Point2D center{
    (first.x + second.x) * 0.5,
    (first.y + second.y) * 0.5,
  };
  return EnclosingCircle{center, distance(first, second) * 0.5};
}

EnclosingCircle circle_from_three_points(
  const Point2D & first,
  const Point2D & second,
  const Point2D & third)
{
  EnclosingCircle best{{0.0, 0.0}, std::numeric_limits<double>::infinity()};
  const auto consider_pair = [&best, &first, &second, &third](
    const Point2D & pair_first, const Point2D & pair_second)
    {
      const auto candidate = circle_from_two_points(pair_first, pair_second);
      if (candidate.radius_m < best.radius_m && contains(candidate, first) &&
        contains(candidate, second) && contains(candidate, third))
      {
        best = candidate;
      }
    };
  consider_pair(first, second);
  consider_pair(first, third);
  consider_pair(second, third);
  if (std::isfinite(best.radius_m)) {
    return best;
  }

  const double denominator = 2.0 * (
    first.x * (second.y - third.y) +
    second.x * (third.y - first.y) +
    third.x * (first.y - second.y));
  const double coordinate_scale = std::max({
        1.0, std::abs(first.x), std::abs(first.y), std::abs(second.x),
        std::abs(second.y), std::abs(third.x), std::abs(third.y)});
  const double numerical_tolerance = 64.0 * std::numeric_limits<double>::epsilon() *
    coordinate_scale * coordinate_scale;
  if (std::abs(denominator) <= numerical_tolerance) {
    EnclosingCircle fallback = circle_from_two_points(first, second);
    const auto first_third = circle_from_two_points(first, third);
    const auto second_third = circle_from_two_points(second, third);
    if (first_third.radius_m > fallback.radius_m) {
      fallback = first_third;
    }
    if (second_third.radius_m > fallback.radius_m) {
      fallback = second_third;
    }
    fallback.radius_m = std::max({
          fallback.radius_m,
          distance(fallback.center, first),
          distance(fallback.center, second),
          distance(fallback.center, third)});
    return fallback;
  }

  const double first_squared = first.x * first.x + first.y * first.y;
  const double second_squared = second.x * second.x + second.y * second.y;
  const double third_squared = third.x * third.x + third.y * third.y;
  const Point2D center{
    (first_squared * (second.y - third.y) +
    second_squared * (third.y - first.y) +
    third_squared * (first.y - second.y)) / denominator,
    (first_squared * (third.x - second.x) +
    second_squared * (first.x - third.x) +
    third_squared * (second.x - first.x)) / denominator,
  };
  return EnclosingCircle{center, distance(center, first)};
}

EnclosingCircle circle_from_boundary(const std::vector<Point2D> & boundary)
{
  if (boundary.empty()) {
    return EnclosingCircle{{0.0, 0.0}, -1.0};
  }
  if (boundary.size() == 1U) {
    return EnclosingCircle{boundary.front(), 0.0};
  }
  if (boundary.size() == 2U) {
    return circle_from_two_points(boundary[0], boundary[1]);
  }
  return circle_from_three_points(boundary[0], boundary[1], boundary[2]);
}

EnclosingCircle welzl(
  const std::vector<Point2D> & points,
  std::size_t point_count,
  std::vector<Point2D> & boundary)
{
  if (point_count == 0U || boundary.size() == 3U) {
    return circle_from_boundary(boundary);
  }

  const Point2D point = points[point_count - 1U];
  auto circle = welzl(points, point_count - 1U, boundary);
  if (contains(circle, point)) {
    return circle;
  }

  boundary.push_back(point);
  circle = welzl(points, point_count - 1U, boundary);
  boundary.pop_back();
  return circle;
}

}  // namespace

EnclosingCircle minimum_enclosing_circle(const std::vector<Point2D> & points)
{
  if (points.empty()) {
    throw std::invalid_argument("minimum enclosing circle requires at least one point");
  }
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("minimum enclosing circle input points must be finite");
    }
  }

  auto shuffled_points = points;
  std::mt19937 generator(0x4D454343U);
  std::shuffle(shuffled_points.begin(), shuffled_points.end(), generator);
  std::vector<Point2D> boundary;
  boundary.reserve(3U);
  auto circle = welzl(shuffled_points, shuffled_points.size(), boundary);

  for (const auto & point : points) {
    circle.radius_m = std::max(circle.radius_m, distance(circle.center, point));
  }
  return circle;
}

std::vector<Point2D> scan_to_points(
  const std::vector<float> & ranges,
  double angle_min,
  double angle_increment,
  double sensor_range_min,
  double sensor_range_max,
  const ScanFilterConfig & config)
{
  validate_scan_config(config);
  if (!std::isfinite(angle_min) || !std::isfinite(angle_increment) || angle_increment <= 0.0) {
    throw std::invalid_argument("scan angles must be finite with a positive increment");
  }

  double effective_minimum = config.minimum_range_m;
  double effective_maximum = config.maximum_range_m;
  if (std::isfinite(sensor_range_min)) {
    effective_minimum = std::max(effective_minimum, sensor_range_min);
  }
  if (std::isfinite(sensor_range_max)) {
    effective_maximum = std::min(effective_maximum, sensor_range_max);
  }
  if (effective_maximum <= effective_minimum) {
    return {};
  }

  std::vector<Point2D> points;
  points.reserve(ranges.size());
  for (std::size_t index = 0; index < ranges.size(); ++index) {
    const double range_m = static_cast<double>(ranges[index]);
    if (!std::isfinite(range_m) || range_m < effective_minimum ||
      range_m > effective_maximum)
    {
      continue;
    }

    const double angle = angle_min + static_cast<double>(index) * angle_increment;
    const Point2D point{range_m * std::cos(angle), range_m * std::sin(angle)};
    if (point.x < config.minimum_forward_x_m ||
      std::abs(point.y) > config.maximum_absolute_y_m)
    {
      continue;
    }
    points.push_back(point);
  }
  return points;
}

std::vector<int> dbscan_labels(
  const std::vector<Point2D> & points,
  double epsilon_m,
  std::size_t minimum_samples)
{
  validate_dbscan_config(epsilon_m, minimum_samples);
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("DBSCAN input points must be finite");
    }
  }

  std::vector<int> labels(points.size(), kUnvisited);
  std::vector<bool> queued(points.size(), false);
  const double epsilon_squared = epsilon_m * epsilon_m;
  int cluster_label = 0;

  for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
    if (labels[point_index] != kUnvisited) {
      continue;
    }

    const auto neighbors = region_query(points, point_index, epsilon_squared);
    if (neighbors.size() < minimum_samples) {
      labels[point_index] = kNoise;
      continue;
    }

    labels[point_index] = cluster_label;
    std::deque<std::size_t> seeds;
    for (const auto neighbor : neighbors) {
      if (neighbor != point_index && !queued[neighbor]) {
        seeds.push_back(neighbor);
        queued[neighbor] = true;
      }
    }

    while (!seeds.empty()) {
      const std::size_t current = seeds.front();
      seeds.pop_front();

      if (labels[current] == kNoise) {
        labels[current] = cluster_label;
      }
      if (labels[current] != kUnvisited) {
        continue;
      }

      labels[current] = cluster_label;
      const auto current_neighbors = region_query(points, current, epsilon_squared);
      if (current_neighbors.size() < minimum_samples) {
        continue;
      }
      for (const auto neighbor : current_neighbors) {
        if (!queued[neighbor] && neighbor != point_index) {
          seeds.push_back(neighbor);
          queued[neighbor] = true;
        }
      }
    }
    ++cluster_label;
  }
  return labels;
}

std::vector<Cluster> detect_clusters(
  const std::vector<Point2D> & points,
  double epsilon_m,
  std::size_t minimum_samples,
  std::size_t minimum_cluster_points,
  double maximum_cluster_extent_m,
  std::size_t maximum_clusters)
{
  if (minimum_cluster_points == 0U) {
    throw std::invalid_argument("minimum_cluster_points must be at least one");
  }
  if (!std::isfinite(maximum_cluster_extent_m) || maximum_cluster_extent_m <= 0.0) {
    throw std::invalid_argument("maximum_cluster_extent_m must be finite and positive");
  }
  if (maximum_clusters == 0U) {
    return {};
  }

  const auto labels = dbscan_labels(points, epsilon_m, minimum_samples);
  int largest_label = kNoise;
  for (const int label : labels) {
    largest_label = std::max(largest_label, label);
  }

  std::vector<std::vector<Point2D>> grouped_points(
    largest_label >= 0 ? static_cast<std::size_t>(largest_label + 1) : 0U);
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (labels[index] >= 0) {
      grouped_points[static_cast<std::size_t>(labels[index])].push_back(points[index]);
    }
  }

  std::vector<Cluster> clusters;
  clusters.reserve(grouped_points.size());
  for (std::size_t label = 0; label < grouped_points.size(); ++label) {
    auto & cluster_points = grouped_points[label];
    if (cluster_points.size() < minimum_cluster_points) {
      continue;
    }

    double minimum_x = std::numeric_limits<double>::infinity();
    double maximum_x = -std::numeric_limits<double>::infinity();
    double minimum_y = std::numeric_limits<double>::infinity();
    double maximum_y = -std::numeric_limits<double>::infinity();
    for (const auto & point : cluster_points) {
      minimum_x = std::min(minimum_x, point.x);
      maximum_x = std::max(maximum_x, point.x);
      minimum_y = std::min(minimum_y, point.y);
      maximum_y = std::max(maximum_y, point.y);
    }
    const double extent_m = std::hypot(maximum_x - minimum_x, maximum_y - minimum_y);
    if (extent_m > maximum_cluster_extent_m) {
      continue;
    }

    const auto circle = minimum_enclosing_circle(cluster_points);
    clusters.push_back(
      Cluster{
        std::move(cluster_points), circle.center, circle.radius_m, extent_m,
        static_cast<int>(label)});
  }

  std::sort(
    clusters.begin(), clusters.end(),
    [](const Cluster & first, const Cluster & second) {
      const double first_distance = squared_distance_from_origin(first);
      const double second_distance = squared_distance_from_origin(second);
      if (first_distance != second_distance) {
        return first_distance < second_distance;
      }
      if (first.center.x != second.center.x) {
        return first.center.x < second.center.x;
      }
      return first.center.y < second.center.y;
    });
  if (clusters.size() > maximum_clusters) {
    clusters.resize(maximum_clusters);
  }
  return clusters;
}

}  // namespace object_detection
