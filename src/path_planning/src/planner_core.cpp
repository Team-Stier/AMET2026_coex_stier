// Copyright 2026 Physicar contributors

#include "path_planning/planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace path_planning
{
namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kGeometryEpsilon = 1.0e-9;
constexpr double kClosureTolerance = 1.0e-5;
constexpr std::size_t kMaximumTrackLookupCells = 10'000'000U;

bool finite(double value) noexcept
{
  return std::isfinite(value);
}

bool finite(const Point2D & point) noexcept
{
  return finite(point.x) && finite(point.y);
}

bool finite(const Pose2D & pose) noexcept
{
  return finite(pose.x) && finite(pose.y) && finite(pose.yaw);
}

double squared_distance(const Point2D & lhs, const Point2D & rhs) noexcept
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  return dx * dx + dy * dy;
}

double distance(const Point2D & lhs, const Point2D & rhs) noexcept
{
  return std::sqrt(squared_distance(lhs, rhs));
}

Point2D subtract(const Point2D & lhs, const Point2D & rhs) noexcept
{
  return {lhs.x - rhs.x, lhs.y - rhs.y};
}

double dot(const Point2D & lhs, const Point2D & rhs) noexcept
{
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

double cross(const Point2D & lhs, const Point2D & rhs) noexcept
{
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

double point_segment_squared_distance(
  const Point2D & point, const Point2D & start, const Point2D & finish,
  double * projection_ratio = nullptr) noexcept
{
  const Point2D segment = subtract(finish, start);
  const double length_squared = dot(segment, segment);
  double ratio = 0.0;
  if (length_squared > kGeometryEpsilon) {
    ratio = std::clamp(dot(subtract(point, start), segment) / length_squared, 0.0, 1.0);
  }
  if (projection_ratio != nullptr) {
    *projection_ratio = ratio;
  }
  const Point2D projection{start.x + ratio * segment.x, start.y + ratio * segment.y};
  return squared_distance(point, projection);
}

bool point_on_segment(
  const Point2D & point, const Point2D & start, const Point2D & finish) noexcept
{
  return point_segment_squared_distance(point, start, finish) <=
         kGeometryEpsilon * kGeometryEpsilon;
}

enum class RingLocation
{
  kOutside,
  kInside,
  kBoundary,
};

struct RingQuery
{
  RingLocation location{RingLocation::kOutside};
  double minimum_squared_distance{std::numeric_limits<double>::infinity()};
};

RingQuery query_ring(const Point2D & point, const std::vector<Point2D> & ring) noexcept
{
  bool inside = false;
  bool on_boundary = false;
  double minimum_squared = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < ring.size(); ++i) {
    const Point2D & start = ring[i];
    const Point2D & finish = ring[(i + 1U) % ring.size()];
    const double edge_distance = point_segment_squared_distance(point, start, finish);
    minimum_squared = std::min(minimum_squared, edge_distance);
    if (edge_distance <= kGeometryEpsilon * kGeometryEpsilon) {
      on_boundary = true;
    }
    const bool crosses_y = (start.y > point.y) != (finish.y > point.y);
    if (crosses_y) {
      const double crossing_x =
        start.x + (point.y - start.y) * (finish.x - start.x) / (finish.y - start.y);
      if (point.x < crossing_x) {
        inside = !inside;
      }
    }
  }
  return {
    on_boundary ? RingLocation::kBoundary :
    (inside ? RingLocation::kInside : RingLocation::kOutside),
    minimum_squared,
  };
}

RingLocation locate_in_ring(const Point2D & point, const std::vector<Point2D> & ring) noexcept
{
  return query_ring(point, ring).location;
}

double signed_area(const std::vector<Point2D> & ring) noexcept
{
  double twice_area = 0.0;
  for (std::size_t i = 0; i < ring.size(); ++i) {
    const Point2D & current = ring[i];
    const Point2D & next = ring[(i + 1U) % ring.size()];
    twice_area += current.x * next.y - next.x * current.y;
  }
  return 0.5 * twice_area;
}

int orientation(const Point2D & a, const Point2D & b, const Point2D & c) noexcept
{
  const double value = cross(subtract(b, a), subtract(c, a));
  if (value > kGeometryEpsilon) {
    return 1;
  }
  if (value < -kGeometryEpsilon) {
    return -1;
  }
  return 0;
}

bool segments_intersect(
  const Point2D & a, const Point2D & b, const Point2D & c, const Point2D & d) noexcept
{
  const int abc = orientation(a, b, c);
  const int abd = orientation(a, b, d);
  const int cda = orientation(c, d, a);
  const int cdb = orientation(c, d, b);
  if (abc * abd < 0 && cda * cdb < 0) {
    return true;
  }
  return (abc == 0 && point_on_segment(c, a, b)) ||
         (abd == 0 && point_on_segment(d, a, b)) ||
         (cda == 0 && point_on_segment(a, c, d)) ||
         (cdb == 0 && point_on_segment(b, c, d));
}

void validate_simple_ring(const std::vector<Point2D> & ring, const char * name)
{
  std::vector<Point2D> compact;
  compact.reserve(ring.size());
  for (const Point2D & point : ring) {
    if (compact.empty() ||
      squared_distance(point, compact.back()) > kGeometryEpsilon * kGeometryEpsilon)
    {
      compact.push_back(point);
    }
  }
  if (compact.size() > 1U &&
    squared_distance(compact.front(), compact.back()) <= kGeometryEpsilon * kGeometryEpsilon)
  {
    compact.pop_back();
  }
  if (compact.size() < 3U) {
    throw std::invalid_argument(std::string(name) + " has fewer than three unique points");
  }

  const std::size_t count = compact.size();
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t i_next = (i + 1U) % count;
    for (std::size_t j = i + 1U; j < count; ++j) {
      const std::size_t j_next = (j + 1U) % count;
      if (i == j || i_next == j || j_next == i) {
        continue;
      }
      if (segments_intersect(compact[i], compact[i_next], compact[j], compact[j_next])) {
        throw std::invalid_argument(std::string(name) + " self-intersects");
      }
    }
  }
}

void validate_disjoint_rings(
  const std::vector<Point2D> & lhs, const std::vector<Point2D> & rhs)
{
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (squared_distance(lhs[i], lhs[(i + 1U) % lhs.size()]) <=
      kGeometryEpsilon * kGeometryEpsilon)
    {
      continue;
    }
    for (std::size_t j = 0; j < rhs.size(); ++j) {
      if (squared_distance(rhs[j], rhs[(j + 1U) % rhs.size()]) <=
        kGeometryEpsilon * kGeometryEpsilon)
      {
        continue;
      }
      if (segments_intersect(
          lhs[i], lhs[(i + 1U) % lhs.size()], rhs[j], rhs[(j + 1U) % rhs.size()]))
      {
        throw std::invalid_argument("inner and outer boundaries intersect");
      }
    }
  }
}

std::string trim(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1U);
}

std::vector<std::string> split_csv_row(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(trim(field));
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

double parse_number(const std::string & text, std::size_t line_number)
{
  std::size_t parsed = 0U;
  double value = 0.0;
  try {
    value = std::stod(text, &parsed);
  } catch (const std::exception &) {
    throw std::runtime_error("non-numeric RDDF value at line " + std::to_string(line_number));
  }
  if (parsed != text.size() || !finite(value)) {
    throw std::runtime_error("invalid RDDF value at line " + std::to_string(line_number));
  }
  return value;
}

bool same_point(const Point2D & lhs, const Point2D & rhs, double tolerance) noexcept
{
  return squared_distance(lhs, rhs) <= tolerance * tolerance;
}

Point2D transform_local_point(const Pose2D & pose, double local_x, double local_y) noexcept
{
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return {
    pose.x + cosine * local_x - sine * local_y,
    pose.y + sine * local_x + cosine * local_y,
  };
}

Pose2D propagate(const Pose2D & start, double curvature, double distance_m) noexcept
{
  if (std::abs(curvature) <= kGeometryEpsilon) {
    return {
      start.x + distance_m * std::cos(start.yaw),
      start.y + distance_m * std::sin(start.yaw),
      normalize_yaw(start.yaw),
    };
  }
  const double finish_yaw = start.yaw + curvature * distance_m;
  return {
    start.x + (std::sin(finish_yaw) - std::sin(start.yaw)) / curvature,
    start.y + (std::cos(start.yaw) - std::cos(finish_yaw)) / curvature,
    normalize_yaw(finish_yaw),
  };
}

double angular_distance(double lhs, double rhs) noexcept
{
  return std::abs(normalize_yaw(lhs - rhs));
}

void require_positive(double value, const char * name)
{
  if (!finite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
  }
}

void require_nonnegative(double value, const char * name)
{
  if (!finite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and non-negative");
  }
}

struct SearchKey
{
  std::int64_t x{};
  std::int64_t y{};
  std::int64_t yaw{};
  std::int64_t progress{};

  bool operator==(const SearchKey & other) const noexcept
  {
    return x == other.x && y == other.y && yaw == other.yaw && progress == other.progress;
  }
};

struct SearchKeyHash
{
  std::size_t operator()(const SearchKey & key) const noexcept
  {
    std::size_t seed = 0U;
    const auto combine = [&seed](std::int64_t value) {
        const std::size_t hashed = std::hash<std::int64_t>{}(value);
        seed ^= hashed + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
      };
    combine(key.x);
    combine(key.y);
    combine(key.yaw);
    combine(key.progress);
    return seed;
  }
};

struct SearchRecord
{
  SearchKey key;
  Pose2D pose;
  double progress{};
  double curvature{};
  double cost{};
  std::int32_t parent{-1};
  bool closed{false};
};

struct QueueEntry
{
  double priority{};
  double cost{};
  std::size_t index{};
};

struct HigherPriority
{
  bool operator()(const QueueEntry & lhs, const QueueEntry & rhs) const noexcept
  {
    if (lhs.priority == rhs.priority) {
      return lhs.cost > rhs.cost;
    }
    return lhs.priority > rhs.priority;
  }
};

SearchKey make_key(
  const Pose2D & pose, double progress, double xy_resolution, double yaw_resolution,
  double progress_resolution) noexcept
{
  return {
    static_cast<std::int64_t>(std::llround(pose.x / xy_resolution)),
    static_cast<std::int64_t>(std::llround(pose.y / xy_resolution)),
    static_cast<std::int64_t>(std::llround(normalize_yaw(pose.yaw) / yaw_resolution)),
    static_cast<std::int64_t>(std::llround(progress / progress_resolution)),
  };
}

bool reached_goal(
  const SearchRecord & record, const GoalGate & gate, double longitudinal_tolerance,
  double yaw_tolerance) noexcept
{
  if (record.progress + longitudinal_tolerance < gate.progress) {
    return false;
  }
  const Point2D relative{record.pose.x - gate.center.x, record.pose.y - gate.center.y};
  if (std::abs(dot(relative, gate.tangent)) > longitudinal_tolerance) {
    return false;
  }
  const Point2D gate_span = subtract(gate.outer, gate.inner);
  const double span_squared = dot(gate_span, gate_span);
  if (span_squared <= kGeometryEpsilon) {
    return false;
  }
  const Point2D position{record.pose.x, record.pose.y};
  const double lateral_ratio = dot(subtract(position, gate.inner), gate_span) / span_squared;
  if (lateral_ratio < 0.0 || lateral_ratio > 1.0) {
    return false;
  }
  const double gate_yaw = std::atan2(gate.tangent.y, gate.tangent.x);
  return angular_distance(record.pose.yaw, gate_yaw) <= yaw_tolerance;
}

}  // namespace

double normalize_yaw(double yaw) noexcept
{
  if (!finite(yaw)) {
    return yaw;
  }
  yaw = std::fmod(yaw + kPi, 2.0 * kPi);
  if (yaw < 0.0) {
    yaw += 2.0 * kPi;
  }
  return yaw - kPi;
}

Pose2D transform_pose_between_frames(
  const Pose2D & pose,
  const Pose2D & source_origin,
  const Pose2D & target_origin) noexcept
{
  const double yaw_offset = target_origin.yaw - source_origin.yaw;
  const double cosine = std::cos(yaw_offset);
  const double sine = std::sin(yaw_offset);
  const double dx = pose.x - source_origin.x;
  const double dy = pose.y - source_origin.y;
  return {
    target_origin.x + cosine * dx - sine * dy,
    target_origin.y + sine * dx + cosine * dy,
    normalize_yaw(pose.yaw + yaw_offset),
  };
}

RddfTrack::RddfTrack(
  std::vector<Point2D> centerline,
  std::vector<Point2D> inner_boundary,
  std::vector<Point2D> outer_boundary)
: centerline_(std::move(centerline)),
  inner_boundary_(std::move(inner_boundary)),
  outer_boundary_(std::move(outer_boundary))
{
  if (centerline_.size() == inner_boundary_.size() &&
    centerline_.size() == outer_boundary_.size() && centerline_.size() >= 2U &&
    same_point(centerline_.front(), centerline_.back(), kClosureTolerance) &&
    same_point(inner_boundary_.front(), inner_boundary_.back(), kClosureTolerance) &&
    same_point(outer_boundary_.front(), outer_boundary_.back(), kClosureTolerance))
  {
    centerline_.pop_back();
    inner_boundary_.pop_back();
    outer_boundary_.pop_back();
  }

  if (centerline_.size() < 3U || centerline_.size() != inner_boundary_.size() ||
    centerline_.size() != outer_boundary_.size())
  {
    throw std::invalid_argument("RDDF center, inner, and outer rings must have equal size >= 3");
  }
  for (std::size_t i = 0; i < centerline_.size(); ++i) {
    if (!finite(centerline_[i]) || !finite(inner_boundary_[i]) || !finite(outer_boundary_[i])) {
      throw std::invalid_argument("RDDF contains a non-finite point");
    }
    if (squared_distance(centerline_[i], centerline_[(i + 1U) % centerline_.size()]) <=
      kGeometryEpsilon * kGeometryEpsilon)
    {
      throw std::invalid_argument("centerline contains duplicate adjacent points");
    }
  }

  validate_simple_ring(centerline_, "centerline");
  validate_simple_ring(inner_boundary_, "inner boundary");
  validate_simple_ring(outer_boundary_, "outer boundary");
  validate_disjoint_rings(inner_boundary_, outer_boundary_);

  const double inner_area = signed_area(inner_boundary_);
  const double outer_area = signed_area(outer_boundary_);
  if (std::abs(inner_area) <= kGeometryEpsilon || std::abs(outer_area) <= kGeometryEpsilon ||
    inner_area * outer_area <= 0.0 || std::abs(outer_area) <= std::abs(inner_area) ||
    locate_in_ring(inner_boundary_.front(), outer_boundary_) != RingLocation::kInside)
  {
    throw std::invalid_argument("RDDF inner/outer boundary orientation or ordering is invalid");
  }
  for (const Point2D & point : centerline_) {
    if (locate_in_ring(point, outer_boundary_) != RingLocation::kInside ||
      locate_in_ring(point, inner_boundary_) != RingLocation::kOutside)
    {
      throw std::invalid_argument("RDDF centerline is not strictly between its boundaries");
    }
  }

  cumulative_length_.reserve(centerline_.size() + 1U);
  cumulative_length_.push_back(0.0);
  for (std::size_t i = 0; i < centerline_.size(); ++i) {
    lap_length_ += distance(centerline_[i], centerline_[(i + 1U) % centerline_.size()]);
    cumulative_length_.push_back(lap_length_);
  }
  require_positive(lap_length_, "RDDF lap length");
  inner_index_ = build_ring_index(inner_boundary_);
  outer_index_ = build_ring_index(outer_boundary_);
}

RddfTrack::RingIndex RddfTrack::build_ring_index(const std::vector<Point2D> & ring)
{
  RingIndex index;
  index.minimum_y = ring.front().y;
  index.maximum_y = ring.front().y;
  double perimeter = 0.0;
  std::size_t nonzero_edges = 0U;
  for (std::size_t i = 0; i < ring.size(); ++i) {
    const Point2D & start = ring[i];
    const Point2D & finish = ring[(i + 1U) % ring.size()];
    index.minimum_y = std::min({index.minimum_y, start.y, finish.y});
    index.maximum_y = std::max({index.maximum_y, start.y, finish.y});
    const double edge_length = distance(start, finish);
    if (edge_length > kGeometryEpsilon) {
      perimeter += edge_length;
      ++nonzero_edges;
    }
  }
  const double vertical_span = index.maximum_y - index.minimum_y;
  const double average_edge_length = perimeter / static_cast<double>(nonzero_edges);
  std::size_t bin_count = 1U;
  if (vertical_span > kGeometryEpsilon) {
    bin_count = static_cast<std::size_t>(std::ceil(vertical_span / average_edge_length));
    bin_count = std::clamp<std::size_t>(bin_count, 1U, ring.size());
  }
  index.bin_height = vertical_span > kGeometryEpsilon ?
    vertical_span / static_cast<double>(bin_count) : 1.0;
  index.bins.resize(bin_count);
  const auto bin_for_y = [&index, bin_count](double y) {
      if (y <= index.minimum_y) {
        return std::size_t{0};
      }
      if (y >= index.maximum_y) {
        return bin_count - 1U;
      }
      return std::min(
        static_cast<std::size_t>((y - index.minimum_y) / index.bin_height),
        bin_count - 1U);
    };
  for (std::size_t i = 0; i < ring.size(); ++i) {
    const Point2D & start = ring[i];
    const Point2D & finish = ring[(i + 1U) % ring.size()];
    const std::size_t first = bin_for_y(std::min(start.y, finish.y));
    const std::size_t last = bin_for_y(std::max(start.y, finish.y));
    for (std::size_t bin = first; bin <= last; ++bin) {
      index.bins[bin].push_back(i);
    }
  }
  return index;
}

bool RddfTrack::point_inside_ring(
  const Point2D & point,
  const std::vector<Point2D> & ring,
  const RingIndex & index) noexcept
{
  if (point.y < index.minimum_y || point.y > index.maximum_y) {
    return false;
  }
  const std::size_t bin = point.y >= index.maximum_y ? index.bins.size() - 1U :
    std::min(
    static_cast<std::size_t>((point.y - index.minimum_y) / index.bin_height),
    index.bins.size() - 1U);
  bool inside = false;
  for (const std::size_t edge : index.bins[bin]) {
    const Point2D & start = ring[edge];
    const Point2D & finish = ring[(edge + 1U) % ring.size()];
    if ((start.y > point.y) != (finish.y > point.y)) {
      const double crossing_x =
        start.x + (point.y - start.y) * (finish.x - start.x) / (finish.y - start.y);
      if (point.x < crossing_x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

bool RddfTrack::edge_within_distance(
  const Point2D & point,
  double distance_m,
  const std::vector<Point2D> & ring,
  const RingIndex & index) noexcept
{
  if (point.y + distance_m < index.minimum_y ||
    point.y - distance_m > index.maximum_y)
  {
    return false;
  }
  const auto bin_for_y = [&index](double y) {
      if (y <= index.minimum_y) {
        return std::size_t{0};
      }
      if (y >= index.maximum_y) {
        return index.bins.size() - 1U;
      }
      return std::min(
        static_cast<std::size_t>((y - index.minimum_y) / index.bin_height),
        index.bins.size() - 1U);
    };
  const std::size_t first = bin_for_y(point.y - distance_m);
  const std::size_t last = bin_for_y(point.y + distance_m);
  const double threshold_squared = distance_m * distance_m;
  for (std::size_t bin = first; bin <= last; ++bin) {
    for (const std::size_t edge : index.bins[bin]) {
      if (point_segment_squared_distance(
          point, ring[edge], ring[(edge + 1U) % ring.size()]) <= threshold_squared)
      {
        return true;
      }
    }
  }
  return false;
}

RddfTrack RddfTrack::from_csv(const std::string & csv_path)
{
  std::ifstream stream(csv_path);
  if (!stream) {
    throw std::runtime_error("failed to open RDDF CSV: " + csv_path);
  }

  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("RDDF CSV is empty: " + csv_path);
  }
  std::vector<std::string> header = split_csv_row(line);
  if (!header.empty() && header.front().size() >= 3U &&
    static_cast<unsigned char>(header.front()[0]) == 0xEFU &&
    static_cast<unsigned char>(header.front()[1]) == 0xBBU &&
    static_cast<unsigned char>(header.front()[2]) == 0xBFU)
  {
    header.front().erase(0U, 3U);
  }
  const std::vector<std::string> expected{
    "center_x_m", "center_y_m", "inner_x_m", "inner_y_m", "outer_x_m", "outer_y_m"};
  if (header != expected) {
    throw std::runtime_error("RDDF CSV header does not match the required six columns");
  }

  std::vector<Point2D> centerline;
  std::vector<Point2D> inner;
  std::vector<Point2D> outer;
  std::size_t line_number = 1U;
  while (std::getline(stream, line)) {
    ++line_number;
    if (trim(line).empty()) {
      continue;
    }
    const std::vector<std::string> fields = split_csv_row(line);
    if (fields.size() != expected.size()) {
      throw std::runtime_error("RDDF CSV row width mismatch at line " +
          std::to_string(line_number));
    }
    centerline.push_back({parse_number(fields[0], line_number),
        parse_number(fields[1], line_number)});
    inner.push_back({parse_number(fields[2], line_number), parse_number(fields[3], line_number)});
    outer.push_back({parse_number(fields[4], line_number), parse_number(fields[5], line_number)});
  }
  if (centerline.size() < 4U ||
    !same_point(centerline.front(), centerline.back(), kClosureTolerance) ||
    !same_point(inner.front(), inner.back(), kClosureTolerance) ||
    !same_point(outer.front(), outer.back(), kClosureTolerance))
  {
    throw std::runtime_error("RDDF CSV rings must explicitly repeat their first point at the end");
  }
  return RddfTrack(std::move(centerline), std::move(inner), std::move(outer));
}

bool RddfTrack::contains(const Point2D & point) const noexcept
{
  return contains(point, 0.0);
}

bool RddfTrack::contains(const Point2D & point, double boundary_margin_m) const noexcept
{
  if (!finite(point) || !finite(boundary_margin_m) || boundary_margin_m < 0.0) {
    return false;
  }
  const bool on_outer = edge_within_distance(
    point, kGeometryEpsilon, outer_boundary_, outer_index_);
  const bool on_inner = edge_within_distance(
    point, kGeometryEpsilon, inner_boundary_, inner_index_);
  if ((!on_outer && !point_inside_ring(point, outer_boundary_, outer_index_)) ||
    (!on_inner && point_inside_ring(point, inner_boundary_, inner_index_)))
  {
    return false;
  }
  if (boundary_margin_m <= kGeometryEpsilon) {
    return true;
  }
  const double strict_margin = boundary_margin_m - kGeometryEpsilon;
  return !edge_within_distance(point, strict_margin, outer_boundary_, outer_index_) &&
         !edge_within_distance(point, strict_margin, inner_boundary_, inner_index_);
}

double RddfTrack::boundary_distance(const Point2D & point) const noexcept
{
  if (!finite(point)) {
    return std::numeric_limits<double>::infinity();
  }
  const RingQuery outer = query_ring(point, outer_boundary_);
  const RingQuery inner = query_ring(point, inner_boundary_);
  return std::sqrt(std::min(
      outer.minimum_squared_distance, inner.minimum_squared_distance));
}

double RddfTrack::signed_clearance(const Point2D & point) const noexcept
{
  if (!finite(point)) {
    return -std::numeric_limits<double>::infinity();
  }
  const RingQuery outer = query_ring(point, outer_boundary_);
  const RingQuery inner = query_ring(point, inner_boundary_);
  const double clearance = std::sqrt(std::min(
      outer.minimum_squared_distance, inner.minimum_squared_distance));
  if (outer.location == RingLocation::kBoundary || inner.location == RingLocation::kBoundary) {
    return 0.0;
  }
  const bool inside_track = outer.location == RingLocation::kInside &&
    inner.location == RingLocation::kOutside;
  return inside_track ? clearance : -clearance;
}

double RddfTrack::wrapped_progress(const Point2D & point) const noexcept
{
  double best_squared = std::numeric_limits<double>::infinity();
  double best_progress = 0.0;
  for (std::size_t i = 0; i < centerline_.size(); ++i) {
    const Point2D & start = centerline_[i];
    const Point2D & finish = centerline_[(i + 1U) % centerline_.size()];
    double ratio = 0.0;
    const double candidate_squared =
      point_segment_squared_distance(point, start, finish, &ratio);
    if (candidate_squared < best_squared) {
      best_squared = candidate_squared;
      best_progress = cumulative_length_[i] + ratio * distance(start, finish);
    }
  }
  return best_progress >= lap_length_ ? 0.0 : best_progress;
}

double RddfTrack::progress(const Point2D & point) const noexcept
{
  return wrapped_progress(point);
}

double RddfTrack::progress(const Point2D & point, double reference_progress) const noexcept
{
  const double base = wrapped_progress(point);
  if (!finite(reference_progress)) {
    return base;
  }
  return base + std::round((reference_progress - base) / lap_length_) * lap_length_;
}

std::optional<double> RddfTrack::progress_within(
  const Point2D & point,
  double minimum_progress,
  double maximum_progress) const noexcept
{
  if (!finite(point) || !finite(minimum_progress) || !finite(maximum_progress) ||
    minimum_progress > maximum_progress)
  {
    return std::nullopt;
  }
  const double reference = 0.5 * (minimum_progress + maximum_progress);
  double best_squared = std::numeric_limits<double>::infinity();
  std::optional<double> best;
  for (std::size_t i = 0; i < centerline_.size(); ++i) {
    const Point2D & start = centerline_[i];
    const Point2D & finish = centerline_[(i + 1U) % centerline_.size()];
    double ratio = 0.0;
    const double candidate_squared =
      point_segment_squared_distance(point, start, finish, &ratio);
    const double base = cumulative_length_[i] + ratio * distance(start, finish);
    const double nearest_lap = std::round((reference - base) / lap_length_);
    for (int offset = -1; offset <= 1; ++offset) {
      const double candidate = base + (nearest_lap + static_cast<double>(offset)) * lap_length_;
      if (candidate + kGeometryEpsilon < minimum_progress ||
        candidate - kGeometryEpsilon > maximum_progress)
      {
        continue;
      }
      if (candidate_squared < best_squared) {
        best_squared = candidate_squared;
        best = candidate;
      }
    }
  }
  return best;
}

RddfTrack::TrackSample RddfTrack::sample(double wrapped_progress_value) const
{
  double wrapped = std::fmod(wrapped_progress_value, lap_length_);
  if (wrapped < 0.0) {
    wrapped += lap_length_;
  }
  auto upper = std::upper_bound(cumulative_length_.begin(), cumulative_length_.end(), wrapped);
  std::size_t index = static_cast<std::size_t>(std::distance(cumulative_length_.begin(),
      upper) - 1);
  index = std::min(index, centerline_.size() - 1U);
  const std::size_t next = (index + 1U) % centerline_.size();
  const double segment_length = cumulative_length_[index + 1U] - cumulative_length_[index];
  const double ratio = (wrapped - cumulative_length_[index]) / segment_length;
  const auto interpolate = [ratio](const Point2D & start, const Point2D & finish) {
      return Point2D{
      start.x + ratio * (finish.x - start.x),
      start.y + ratio * (finish.y - start.y),
      };
    };
  const Point2D tangent_delta = subtract(centerline_[next], centerline_[index]);
  const double tangent_length = std::hypot(tangent_delta.x, tangent_delta.y);
  return {
    interpolate(centerline_[index], centerline_[next]),
    interpolate(inner_boundary_[index], inner_boundary_[next]),
    interpolate(outer_boundary_[index], outer_boundary_[next]),
    {tangent_delta.x / tangent_length, tangent_delta.y / tangent_length},
  };
}

GoalGate RddfTrack::goal_gate_from(double start_progress, double horizon_m) const
{
  if (!finite(start_progress)) {
    throw std::invalid_argument("start progress must be finite");
  }
  require_positive(horizon_m, "planning horizon");
  const double target_progress = start_progress + horizon_m;
  const TrackSample target = sample(target_progress);
  return {target.center, target.inner, target.outer, target.tangent, target_progress};
}

double RddfTrack::lap_length() const noexcept
{
  return lap_length_;
}

const std::vector<Point2D> & RddfTrack::centerline() const noexcept
{
  return centerline_;
}

const std::vector<Point2D> & RddfTrack::inner_boundary() const noexcept
{
  return inner_boundary_;
}

const std::vector<Point2D> & RddfTrack::outer_boundary() const noexcept
{
  return outer_boundary_;
}

VehicleFootprint::VehicleFootprint(
  double vehicle_length_m,
  double vehicle_width_m,
  double wheelbase_m,
  double wheel_track_m)
{
  require_positive(vehicle_length_m, "vehicle length");
  require_positive(vehicle_width_m, "vehicle width");
  require_positive(wheelbase_m, "wheelbase");
  require_positive(wheel_track_m, "wheel track");
  if (wheelbase_m > vehicle_length_m || wheel_track_m > vehicle_width_m) {
    throw std::invalid_argument("wheelbase/wheel track cannot exceed vehicle dimensions");
  }
  const double overhang = 0.5 * (vehicle_length_m - wheelbase_m);
  longitudinal_min_ = -overhang;
  longitudinal_max_ = wheelbase_m + overhang;
  half_width_ = 0.5 * vehicle_width_m;
  wheelbase_ = wheelbase_m;
  half_wheel_track_ = 0.5 * wheel_track_m;
}

std::array<Point2D, 4> VehicleFootprint::wheel_points(const Pose2D & pose) const noexcept
{
  return {
    transform_local_point(pose, 0.0, half_wheel_track_),
    transform_local_point(pose, 0.0, -half_wheel_track_),
    transform_local_point(pose, wheelbase_, half_wheel_track_),
    transform_local_point(pose, wheelbase_, -half_wheel_track_),
  };
}

double VehicleFootprint::body_clearance(const Circle & circle, const Pose2D & pose) const noexcept
{
  if (!finite(pose) || !finite(circle.x) || !finite(circle.y) || !finite(circle.radius) ||
    circle.radius < 0.0)
  {
    return -std::numeric_limits<double>::infinity();
  }
  const double dx = circle.x - pose.x;
  const double dy = circle.y - pose.y;
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  const double local_x = cosine * dx + sine * dy;
  const double local_y = -sine * dx + cosine * dy;
  const double nearest_x = std::clamp(local_x, longitudinal_min_, longitudinal_max_);
  const double nearest_y = std::clamp(local_y, -half_width_, half_width_);
  return std::hypot(local_x - nearest_x, local_y - nearest_y) - circle.radius;
}

bool VehicleFootprint::body_intersects(const Circle & circle, const Pose2D & pose) const noexcept
{
  return body_clearance(circle, pose) <= 0.0;
}

CollisionChecker::CollisionChecker(
  const RddfTrack & track,
  VehicleFootprint footprint,
  double track_lookup_resolution_m)
: track_(track),
  footprint_(std::move(footprint)),
  lookup_resolution_(track_lookup_resolution_m)
{
  require_positive(track_lookup_resolution_m, "track lookup resolution");
  const std::vector<Point2D> & outer = track_.outer_boundary();
  lookup_minimum_x_ = outer.front().x;
  lookup_maximum_x_ = outer.front().x;
  lookup_minimum_y_ = outer.front().y;
  lookup_maximum_y_ = outer.front().y;
  for (const Point2D & point : outer) {
    lookup_minimum_x_ = std::min(lookup_minimum_x_, point.x);
    lookup_maximum_x_ = std::max(lookup_maximum_x_, point.x);
    lookup_minimum_y_ = std::min(lookup_minimum_y_, point.y);
    lookup_maximum_y_ = std::max(lookup_maximum_y_, point.y);
  }
  const double column_intervals =
    (lookup_maximum_x_ - lookup_minimum_x_) / lookup_resolution_;
  const double row_intervals =
    (lookup_maximum_y_ - lookup_minimum_y_) / lookup_resolution_;
  const double maximum_intervals =
    static_cast<double>(kMaximumTrackLookupCells - 1U);
  if (!finite(column_intervals) || !finite(row_intervals) ||
    column_intervals > maximum_intervals || row_intervals > maximum_intervals)
  {
    throw std::length_error("track lookup grid dimensions overflow size_t");
  }
  lookup_columns_ = static_cast<std::size_t>(std::ceil(column_intervals)) + 1U;
  lookup_rows_ = static_cast<std::size_t>(std::ceil(row_intervals)) + 1U;
  if (lookup_columns_ == 0U || lookup_rows_ == 0U ||
    lookup_rows_ > std::numeric_limits<std::size_t>::max() / lookup_columns_ ||
    lookup_columns_ * lookup_rows_ > kMaximumTrackLookupCells)
  {
    throw std::length_error("track lookup grid exceeds the safe cell limit");
  }
  track_clearance_lookup_.resize(lookup_columns_ * lookup_rows_);
  for (std::size_t row = 0; row < lookup_rows_; ++row) {
    const double y = lookup_minimum_y_ + static_cast<double>(row) * lookup_resolution_;
    for (std::size_t column = 0; column < lookup_columns_; ++column) {
      const double x = lookup_minimum_x_ + static_cast<double>(column) * lookup_resolution_;
      track_clearance_lookup_[row * lookup_columns_ + column] =
        track_.signed_clearance({x, y});
    }
  }
}

bool CollisionChecker::wheel_is_inside_track(const Point2D & wheel) const noexcept
{
  if (!finite(wheel)) {
    return false;
  }
  if (wheel.x < lookup_minimum_x_ - kGeometryEpsilon ||
    wheel.x > lookup_maximum_x_ + kGeometryEpsilon ||
    wheel.y < lookup_minimum_y_ - kGeometryEpsilon ||
    wheel.y > lookup_maximum_y_ + kGeometryEpsilon)
  {
    return false;
  }
  if (wheel.x < lookup_minimum_x_ || wheel.x > lookup_maximum_x_ ||
    wheel.y < lookup_minimum_y_ || wheel.y > lookup_maximum_y_)
  {
    return track_.contains(wheel);
  }
  const std::size_t column = std::min(
    static_cast<std::size_t>(std::llround((wheel.x - lookup_minimum_x_) / lookup_resolution_)),
    lookup_columns_ - 1U);
  const std::size_t row = std::min(
    static_cast<std::size_t>(std::llround((wheel.y - lookup_minimum_y_) / lookup_resolution_)),
    lookup_rows_ - 1U);
  const Point2D sample{
    lookup_minimum_x_ + static_cast<double>(column) * lookup_resolution_,
    lookup_minimum_y_ + static_cast<double>(row) * lookup_resolution_,
  };
  const double sample_distance = distance(wheel, sample);
  const double sampled_clearance =
    track_clearance_lookup_[row * lookup_columns_ + column];
  if (sampled_clearance >= sample_distance + kGeometryEpsilon) {
    return true;
  }
  if (sampled_clearance < -sample_distance - kGeometryEpsilon) {
    return false;
  }
  return track_.contains(wheel);
}

bool CollisionChecker::is_pose_valid(
  const Pose2D & pose,
  const std::vector<Circle> & obstacles) const noexcept
{
  if (!finite(pose)) {
    return false;
  }
  const auto wheels = footprint_.wheel_points(pose);
  if (std::none_of(
      wheels.begin(), wheels.end(),
      [this](const Point2D & wheel) { return wheel_is_inside_track(wheel); }))
  {
    return false;
  }
  for (const Circle & obstacle : obstacles) {
    if (!finite(obstacle.x) || !finite(obstacle.y) || !finite(obstacle.radius) ||
      obstacle.radius <= 0.0 || footprint_.body_intersects(obstacle, pose))
    {
      return false;
    }
  }
  return true;
}

bool CollisionChecker::is_primitive_valid(
  const std::vector<Pose2D> & primitive,
  const std::vector<Circle> & obstacles) const noexcept
{
  if (primitive.empty()) {
    return false;
  }
  for (const Pose2D & pose : primitive) {
    if (!is_pose_valid(pose, obstacles)) {
      return false;
    }
  }
  return true;
}

CostModel::CostModel(const CostFunction & function)
: transition_cost_function_(function.transition_cost),
  heuristic_function_(function.heuristic)
{
  if (transition_cost_function_ == nullptr || heuristic_function_ == nullptr) {
    throw std::invalid_argument("cost function callbacks must not be null");
  }
}

double CostModel::transition_cost(
  double distance_m,
  double curvature,
  double previous_curvature) const noexcept
{
  if (!finite(distance_m) || distance_m <= 0.0 || !finite(curvature) ||
    !finite(previous_curvature))
  {
    return std::numeric_limits<double>::infinity();
  }
  return transition_cost_function_(distance_m, curvature, previous_curvature);
}

double CostModel::heuristic(double minimum_travel_distance_m) const noexcept
{
  return heuristic_function_(minimum_travel_distance_m);
}

HybridAStarPlanner::HybridAStarPlanner(
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
  double vehicle_max_steering_rad,
  std::vector<double> steering_candidates_rad,
  double goal_longitudinal_tolerance_m,
  double goal_yaw_tolerance_rad,
  double progress_regression_tolerance_m,
  double max_progress_advance_ratio,
  std::size_t max_search_nodes,
  bool collect_search_tree)
: track_(track),
  collision_checker_(collision_checker),
  cost_model_(cost_model),
  wheelbase_(wheelbase_m),
  planning_horizon_(planning_horizon_m),
  xy_resolution_(xy_resolution_m),
  yaw_resolution_(yaw_resolution_rad),
  progress_resolution_(progress_resolution_m),
  primitive_length_(primitive_length_m),
  collision_check_step_(collision_check_step_m),
  goal_longitudinal_tolerance_(goal_longitudinal_tolerance_m),
  goal_yaw_tolerance_(goal_yaw_tolerance_rad),
  progress_regression_tolerance_(progress_regression_tolerance_m),
  max_progress_advance_ratio_(max_progress_advance_ratio),
  max_search_nodes_(max_search_nodes),
  collect_search_tree_(collect_search_tree)
{
  require_positive(wheelbase_m, "wheelbase");
  require_positive(planning_horizon_m, "planning horizon");
  require_positive(xy_resolution_m, "xy resolution");
  require_positive(yaw_resolution_rad, "yaw resolution");
  require_positive(progress_resolution_m, "progress resolution");
  require_positive(primitive_length_m, "primitive length");
  require_positive(collision_check_step_m, "collision-check step");
  require_positive(vehicle_max_steering_rad, "vehicle maximum steering");
  require_positive(goal_longitudinal_tolerance_m, "goal longitudinal tolerance");
  require_positive(goal_yaw_tolerance_rad, "goal yaw tolerance");
  require_nonnegative(progress_regression_tolerance_m, "progress regression tolerance");
  if (!finite(max_progress_advance_ratio) || max_progress_advance_ratio < 1.0) {
    throw std::invalid_argument("maximum progress advance ratio must be finite and at least one");
  }
  if (collision_check_step_m > primitive_length_m) {
    throw std::invalid_argument("collision-check step cannot exceed primitive length");
  }
  if (yaw_resolution_rad > 2.0 * kPi || goal_yaw_tolerance_rad > kPi) {
    throw std::invalid_argument("yaw resolution/tolerance is outside its valid range");
  }
  if (vehicle_max_steering_rad >= 0.5 * kPi) {
    throw std::invalid_argument("vehicle maximum steering must be less than pi/2");
  }
  if (steering_candidates_rad.empty()) {
    throw std::invalid_argument("steering candidate list must not be empty");
  }
  curvatures_.reserve(steering_candidates_rad.size());
  for (const double steering : steering_candidates_rad) {
    if (!finite(steering) || std::abs(steering) >= 0.5 * kPi) {
      throw std::invalid_argument(
              "steering candidates must be finite and strictly between -pi/2 and pi/2");
    }
    if (std::abs(steering) > vehicle_max_steering_rad) {
      throw std::invalid_argument(
              "steering candidates must not exceed the vehicle maximum steering angle");
    }
    curvatures_.push_back(std::tan(steering) / wheelbase_);
  }
  if (max_search_nodes == 0U ||
    max_search_nodes > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()))
  {
    throw std::invalid_argument("maximum search node count must fit a positive int32 index");
  }
}

PlanAttemptResult HybridAStarPlanner::plan(const PlanningSnapshot & snapshot) const
{
  if (!snapshot.pose) {
    PlanAttemptResult result;
    result.status = PlanStatus::kInvalidStart;
    return result;
  }

  const Pose2D & requested_pose = *snapshot.pose;
  std::vector<SearchRecord> records;
  records.reserve(std::min<std::size_t>(max_search_nodes_, 4096U));
  const double start_progress = track_.progress({requested_pose.x, requested_pose.y});
  const Pose2D start_pose{
    requested_pose.x, requested_pose.y, normalize_yaw(requested_pose.yaw)};
  const SearchKey start_key = make_key(
    start_pose, start_progress, xy_resolution_, yaw_resolution_, progress_resolution_);
  records.push_back({
      start_key,
      start_pose,
      start_progress,
      0.0,
      0.0,
      -1,
      false,
  });

  const auto finish = [&](PlanStatus status, std::optional<PlannedPath> path = std::nullopt,
    std::size_t expanded = 0U, std::int32_t final_node_index = -1) {
      PlanAttemptResult result;
      result.status = status;
      result.path = std::move(path);
      result.expanded_nodes = expanded;
      result.final_node_index = final_node_index;
      if (collect_search_tree_ && status == PlanStatus::kSuccess) {
        result.tree.reserve(records.size());
        for (const SearchRecord & record : records) {
          result.tree.push_back(
            {record.pose.x, record.pose.y, normalize_yaw(record.pose.yaw), record.parent});
        }
      }
      return result;
    };

  if (!finite(requested_pose) ||
    !collision_checker_.is_pose_valid(records.front().pose, snapshot.obstacles))
  {
    return finish(PlanStatus::kInvalidStart);
  }
  for (const Circle & obstacle : snapshot.obstacles) {
    if (!finite(obstacle.x) || !finite(obstacle.y) || !finite(obstacle.radius) ||
      obstacle.radius <= 0.0)
    {
      return finish(PlanStatus::kFailure);
    }
  }

  const GoalGate goal = track_.goal_gate_from(start_progress, planning_horizon_);

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, HigherPriority> open;
  std::unordered_map<SearchKey, std::size_t, SearchKeyHash> discovered;
  discovered.emplace(start_key, 0U);
  const auto heuristic = [this, &goal](const Pose2D & pose, double progress) {
      const Point2D position{pose.x, pose.y};
      const double gate_distance = std::max(
        0.0,
        std::sqrt(point_segment_squared_distance(position, goal.inner, goal.outer)) -
        goal_longitudinal_tolerance_);
      const double progress_distance = std::max(
        0.0, goal.progress - progress - goal_longitudinal_tolerance_) /
        max_progress_advance_ratio_;
      return cost_model_.heuristic(std::max(gate_distance, progress_distance));
    };
  open.push({heuristic(records.front().pose, records.front().progress), 0.0, 0U});

  const std::size_t interpolation_count = static_cast<std::size_t>(
    std::ceil(primitive_length_ / collision_check_step_));
  std::vector<Pose2D> primitive;
  primitive.reserve(interpolation_count);
  std::size_t expanded = 0U;

  while (!open.empty()) {
    const QueueEntry entry = open.top();
    open.pop();
    SearchRecord & current = records[entry.index];
    const auto latest = discovered.find(current.key);
    if (current.closed || latest == discovered.end() || latest->second != entry.index ||
      entry.cost > current.cost + kGeometryEpsilon)
    {
      continue;
    }
    current.closed = true;
    ++expanded;
    if (reached_goal(current, goal, goal_longitudinal_tolerance_, goal_yaw_tolerance_)) {
      std::vector<PathPoint> reverse_path;
      std::int32_t index = static_cast<std::int32_t>(entry.index);
      while (index >= 0) {
        const SearchRecord & record = records[static_cast<std::size_t>(index)];
        reverse_path.push_back(
          {record.pose.x, record.pose.y, normalize_yaw(record.pose.yaw), record.curvature,
            record.progress});
        index = record.parent;
      }
      std::reverse(reverse_path.begin(), reverse_path.end());
      PlannedPath path;
      path.points = std::move(reverse_path);
      path.frame_id = snapshot.frame_id;
      return finish(
        PlanStatus::kSuccess, std::move(path), expanded,
        static_cast<std::int32_t>(entry.index));
    }

    if (records.size() >= max_search_nodes_) {
      continue;
    }

    const Pose2D parent_pose = current.pose;
    const double parent_progress = current.progress;
    const double parent_curvature = current.curvature;
    const double parent_cost = current.cost;
    const double maximum_child_progress =
      parent_progress + primitive_length_ * max_progress_advance_ratio_;
    for (const double curvature : curvatures_) {
      primitive.clear();
      for (std::size_t step = 1U; step <= interpolation_count; ++step) {
        const double travelled = primitive_length_ * static_cast<double>(step) /
          static_cast<double>(interpolation_count);
        primitive.push_back(propagate(parent_pose, curvature, travelled));
      }
      if (!collision_checker_.is_primitive_valid(primitive, snapshot.obstacles)) {
        continue;
      }

      const Pose2D child_pose = primitive.back();
      const std::optional<double> child_progress_candidate = track_.progress_within(
        {child_pose.x, child_pose.y},
        parent_progress - progress_regression_tolerance_,
        maximum_child_progress);
      if (!child_progress_candidate.has_value()) {
        continue;
      }
      const double child_progress = *child_progress_candidate;
      const double transition = cost_model_.transition_cost(
        primitive_length_, curvature, parent_curvature);
      const double child_cost = parent_cost + transition;
      if (!finite(child_cost)) {
        continue;
      }
      const SearchKey key = make_key(
        child_pose, child_progress, xy_resolution_, yaw_resolution_, progress_resolution_);
      const auto found = discovered.find(key);
      std::size_t child_index = 0U;
      if (found != discovered.end() &&
        child_cost + kGeometryEpsilon >= records[found->second].cost)
      {
        continue;
      }
      if (records.size() >= max_search_nodes_) {
        continue;
      }
      child_index = records.size();
      records.push_back({
          key,
          child_pose,
          child_progress,
          curvature,
          child_cost,
          static_cast<std::int32_t>(entry.index),
          false,
      });
      if (found == discovered.end()) {
        discovered.emplace(key, child_index);
      } else {
        found->second = child_index;
      }
      const double child_priority = child_cost + heuristic(child_pose, child_progress);
      open.push({child_priority, child_cost, child_index});
    }
  }

  return finish(PlanStatus::kFailure, std::nullopt, expanded);
}

}  // namespace path_planning
