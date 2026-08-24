#ifndef OBJECT_DETECTION__DBSCAN_HPP_
#define OBJECT_DETECTION__DBSCAN_HPP_

#include <cstddef>
#include <vector>

namespace object_detection
{

struct Point2D
{
  double x;
  double y;
};

struct EnclosingCircle
{
  Point2D center;
  double radius_m;
};

struct ScanFilterConfig
{
  bool apply_roi{true};
  double minimum_range_m{0.15};
  double maximum_range_m{4.0};
  double minimum_forward_x_m{0.0};
  double maximum_absolute_y_m{1.5};
};

struct Cluster
{
  std::vector<Point2D> points;
  Point2D center;
  double radius_m;
  double extent_m;
  int source_label;
};

EnclosingCircle minimum_enclosing_circle(const std::vector<Point2D> & points);

std::vector<Point2D> scan_to_points(
  const std::vector<float> & ranges,
  double angle_min,
  double angle_increment,
  double sensor_range_min,
  double sensor_range_max,
  const ScanFilterConfig & config);

std::vector<int> dbscan_labels(
  const std::vector<Point2D> & points,
  double epsilon_m,
  std::size_t minimum_samples);

std::vector<Cluster> detect_clusters(
  const std::vector<Point2D> & points,
  double epsilon_m,
  std::size_t minimum_samples,
  std::size_t minimum_cluster_points,
  double maximum_cluster_extent_m,
  std::size_t maximum_clusters);

}  // namespace object_detection

#endif  // OBJECT_DETECTION__DBSCAN_HPP_
