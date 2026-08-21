#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "interfaces/msg/objects.hpp"
#include "object_detection/dbscan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace object_detection
{
namespace
{

constexpr std::size_t kMessageCapacity = 20U;
constexpr char kMarkerNamespace[] = "object_detection";
constexpr char kRoiMarkerNamespace[] = "object_detection_roi";
constexpr char kFittedCircleMarkerNamespace[] = "object_detection_fitted_circle";
constexpr std::size_t kArcSegments = 48U;
constexpr std::size_t kCircleSegments = 64U;
constexpr double kPi = 3.14159265358979323846;

void append_line_segment(
  visualization_msgs::msg::Marker & marker,
  double first_x,
  double first_y,
  double second_x,
  double second_y)
{
  geometry_msgs::msg::Point first;
  first.x = first_x;
  first.y = first_y;
  marker.points.push_back(first);

  geometry_msgs::msg::Point second;
  second.x = second_x;
  second.y = second_y;
  marker.points.push_back(second);
}

std_msgs::msg::ColorRGBA color_for_cluster(std::size_t index)
{
  constexpr std::array<std::array<float, 3>, 12> palette{{
    {{0.95F, 0.20F, 0.20F}},
    {{0.20F, 0.75F, 0.25F}},
    {{0.20F, 0.45F, 1.00F}},
    {{1.00F, 0.65F, 0.10F}},
    {{0.70F, 0.25F, 0.90F}},
    {{0.10F, 0.80F, 0.80F}},
    {{1.00F, 0.35F, 0.70F}},
    {{0.60F, 0.75F, 0.10F}},
    {{0.35F, 0.30F, 1.00F}},
    {{0.85F, 0.45F, 0.15F}},
    {{0.15F, 0.65F, 0.55F}},
    {{0.75F, 0.15F, 0.45F}},
  }};
  const auto & rgb = palette[index % palette.size()];
  std_msgs::msg::ColorRGBA color;
  color.r = rgb[0];
  color.g = rgb[1];
  color.b = rgb[2];
  color.a = 0.95F;
  return color;
}

}  // namespace

class ObjectDetectionNode : public rclcpp::Node
{
public:
  ObjectDetectionNode()
  : Node("object_detection_node")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    object_topic_ = declare_parameter<std::string>("object_topic", "/object_info");
    marker_topic_ =
      declare_parameter<std::string>("marker_topic", "/object_detection/markers");
    fitted_circle_marker_topic_ = declare_parameter<std::string>(
      "fitted_circle_marker_topic", "/object_detection/fitted_circles");

    scan_filter_.apply_roi = declare_parameter<bool>("apply_roi", false);
    scan_filter_.minimum_range_m = declare_parameter<double>("minimum_range_m", 0.15);
    scan_filter_.maximum_range_m = declare_parameter<double>("maximum_range_m", 4.0);
    scan_filter_.minimum_forward_x_m =
      declare_parameter<double>("minimum_forward_x_m", 0.0);
    scan_filter_.maximum_absolute_y_m =
      declare_parameter<double>("maximum_absolute_y_m", 1.5);
    epsilon_m_ = declare_parameter<double>("dbscan_epsilon_m", 0.18);

    const auto minimum_samples = declare_parameter<int>("dbscan_minimum_samples", 3);
    const auto minimum_cluster_points = declare_parameter<int>("minimum_cluster_points", 3);
    const auto maximum_objects = declare_parameter<int>("maximum_objects", 20);
    if (minimum_samples < 1) {
      throw std::invalid_argument("dbscan_minimum_samples must be at least one");
    }
    if (minimum_cluster_points < 1) {
      throw std::invalid_argument("minimum_cluster_points must be at least one");
    }
    if (maximum_objects < 1 || maximum_objects > static_cast<int>(kMessageCapacity)) {
      throw std::invalid_argument("maximum_objects must be between 1 and 20");
    }
    minimum_samples_ = static_cast<std::size_t>(minimum_samples);
    minimum_cluster_points_ = static_cast<std::size_t>(minimum_cluster_points);
    maximum_objects_ = static_cast<std::size_t>(maximum_objects);

    maximum_cluster_extent_m_ =
      declare_parameter<double>("maximum_cluster_extent_m", 0.8);
    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    marker_point_size_m_ = declare_parameter<double>("marker_point_size_m", 0.04);
    marker_center_size_m_ = declare_parameter<double>("marker_center_size_m", 0.12);
    marker_roi_line_width_m_ = declare_parameter<double>("marker_roi_line_width_m", 0.01);
    marker_circle_line_width_m_ =
      declare_parameter<double>("marker_circle_line_width_m", 0.02);
    if (marker_point_size_m_ <= 0.0 || marker_center_size_m_ <= 0.0 ||
      marker_roi_line_width_m_ <= 0.0 || marker_circle_line_width_m_ <= 0.0)
    {
      throw std::invalid_argument("marker sizes must be positive");
    }

    object_publisher_ = create_publisher<interfaces::msg::Objects>(object_topic_, 10);
    if (publish_markers_) {
      marker_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
      fitted_circle_marker_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
        fitted_circle_marker_topic_, 10);
    }
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObjectDetectionNode::on_scan, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "DBSCAN minimum enclosing circle detector ready: "
      "scan=%s eps=%.3f min_samples=%zu min_cluster_points=%zu roi=%s markers=%s",
      scan_topic_.c_str(), epsilon_m_, minimum_samples_, minimum_cluster_points_,
      scan_filter_.apply_roi ? "on" : "off",
      publish_markers_ ? "on" : "off");
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    const auto started = std::chrono::steady_clock::now();
    const auto points = scan_to_points(
      scan->ranges,
      static_cast<double>(scan->angle_min),
      static_cast<double>(scan->angle_increment),
      static_cast<double>(scan->range_min),
      static_cast<double>(scan->range_max),
      scan_filter_);
    const auto clusters = detect_clusters(
      points,
      epsilon_m_,
      minimum_samples_,
      minimum_cluster_points_,
      maximum_cluster_extent_m_,
      maximum_objects_);

    publish_objects(*scan, clusters);
    if (publish_markers_) {
      publish_cluster_markers(*scan, clusters);
      publish_fitted_circle_markers(*scan, clusters);
    }

    const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started);
    RCLCPP_DEBUG(
      get_logger(), "processed %zu scan points into %zu objects in %.3f ms",
      points.size(), clusters.size(), elapsed.count());
  }

  void publish_objects(
    const sensor_msgs::msg::LaserScan & scan,
    const std::vector<Cluster> & clusters)
  {
    interfaces::msg::Objects message;
    message.header = scan.header;
    message.length = static_cast<std::int32_t>(clusters.size());
    message.x.fill(0.0F);
    message.y.fill(0.0F);
    for (std::size_t index = 0; index < clusters.size(); ++index) {
      message.x[index] = static_cast<float>(clusters[index].center.x);
      message.y[index] = static_cast<float>(clusters[index].center.y);
    }
    object_publisher_->publish(std::move(message));
  }

  void publish_cluster_markers(
    const sensor_msgs::msg::LaserScan & scan,
    const std::vector<Cluster> & clusters)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    const std::size_t current_marker_count = clusters.size() * 2U;
    marker_array.markers.reserve(
      1U + current_marker_count +
      (previous_marker_count_ > current_marker_count ?
      previous_marker_count_ - current_marker_count : 0U));
    marker_array.markers.push_back(
      scan_filter_.apply_roi ? make_roi_marker(scan) : make_roi_delete_marker(scan));

    for (std::size_t index = 0; index < clusters.size(); ++index) {
      const auto color = color_for_cluster(index);
      visualization_msgs::msg::Marker points_marker;
      points_marker.header = scan.header;
      points_marker.ns = kMarkerNamespace;
      points_marker.id = static_cast<std::int32_t>(index * 2U);
      points_marker.type = visualization_msgs::msg::Marker::POINTS;
      points_marker.action = visualization_msgs::msg::Marker::ADD;
      points_marker.pose.orientation.w = 1.0;
      points_marker.scale.x = marker_point_size_m_;
      points_marker.scale.y = marker_point_size_m_;
      points_marker.color = color;
      points_marker.points.reserve(clusters[index].points.size());
      for (const auto & point : clusters[index].points) {
        geometry_msgs::msg::Point marker_point;
        marker_point.x = point.x;
        marker_point.y = point.y;
        marker_point.z = 0.0;
        points_marker.points.push_back(marker_point);
      }
      marker_array.markers.push_back(std::move(points_marker));

      visualization_msgs::msg::Marker center_marker;
      center_marker.header = scan.header;
      center_marker.ns = kMarkerNamespace;
      center_marker.id = static_cast<std::int32_t>(index * 2U + 1U);
      center_marker.type = visualization_msgs::msg::Marker::SPHERE;
      center_marker.action = visualization_msgs::msg::Marker::ADD;
      center_marker.pose.position.x = clusters[index].center.x;
      center_marker.pose.position.y = clusters[index].center.y;
      center_marker.pose.orientation.w = 1.0;
      center_marker.scale.x = marker_center_size_m_;
      center_marker.scale.y = marker_center_size_m_;
      center_marker.scale.z = marker_center_size_m_;
      center_marker.color = color;
      marker_array.markers.push_back(std::move(center_marker));
    }

    for (std::size_t marker_id = current_marker_count;
      marker_id < previous_marker_count_; ++marker_id)
    {
      visualization_msgs::msg::Marker delete_marker;
      delete_marker.header = scan.header;
      delete_marker.ns = kMarkerNamespace;
      delete_marker.id = static_cast<std::int32_t>(marker_id);
      delete_marker.action = visualization_msgs::msg::Marker::DELETE;
      marker_array.markers.push_back(std::move(delete_marker));
    }
    previous_marker_count_ = current_marker_count;
    marker_publisher_->publish(std::move(marker_array));
  }

  void publish_fitted_circle_markers(
    const sensor_msgs::msg::LaserScan & scan,
    const std::vector<Cluster> & clusters)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.reserve(
      clusters.size() +
      (previous_circle_marker_count_ > clusters.size() ?
      previous_circle_marker_count_ - clusters.size() : 0U));

    for (std::size_t index = 0; index < clusters.size(); ++index) {
      visualization_msgs::msg::Marker circle_marker;
      circle_marker.header = scan.header;
      circle_marker.ns = kFittedCircleMarkerNamespace;
      circle_marker.id = static_cast<std::int32_t>(index);
      circle_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      circle_marker.action = visualization_msgs::msg::Marker::ADD;
      circle_marker.pose.orientation.w = 1.0;
      circle_marker.scale.x = marker_circle_line_width_m_;
      circle_marker.color = color_for_cluster(index);
      circle_marker.points.reserve(kCircleSegments + 1U);
      for (std::size_t segment = 0; segment <= kCircleSegments; ++segment) {
        const double angle = 2.0 * kPi * static_cast<double>(segment) /
          static_cast<double>(kCircleSegments);
        geometry_msgs::msg::Point point;
        point.x = clusters[index].center.x + clusters[index].radius_m * std::cos(angle);
        point.y = clusters[index].center.y + clusters[index].radius_m * std::sin(angle);
        point.z = 0.0;
        circle_marker.points.push_back(point);
      }
      marker_array.markers.push_back(std::move(circle_marker));
    }

    for (std::size_t marker_id = clusters.size();
      marker_id < previous_circle_marker_count_; ++marker_id)
    {
      visualization_msgs::msg::Marker delete_marker;
      delete_marker.header = scan.header;
      delete_marker.ns = kFittedCircleMarkerNamespace;
      delete_marker.id = static_cast<std::int32_t>(marker_id);
      delete_marker.action = visualization_msgs::msg::Marker::DELETE;
      marker_array.markers.push_back(std::move(delete_marker));
    }
    previous_circle_marker_count_ = clusters.size();
    fitted_circle_marker_publisher_->publish(std::move(marker_array));
  }

  visualization_msgs::msg::Marker make_roi_marker(
    const sensor_msgs::msg::LaserScan & scan) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = scan.header;
    marker.ns = kRoiMarkerNamespace;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_roi_line_width_m_;
    marker.color.r = 0.10F;
    marker.color.g = 0.85F;
    marker.color.b = 1.00F;
    marker.color.a = 0.80F;

    const double minimum_x = std::max(0.0, scan_filter_.minimum_forward_x_m);
    const double outer_radius = scan_filter_.maximum_range_m;
    if (minimum_x >= outer_radius) {
      return marker;
    }

    const double maximum_y_at_minimum_x =
      std::sqrt(outer_radius * outer_radius - minimum_x * minimum_x);
    const double boundary_y =
      std::min(scan_filter_.maximum_absolute_y_m, maximum_y_at_minimum_x);
    const double outer_angle = std::asin(boundary_y / outer_radius);
    const double outer_x_at_boundary = outer_radius * std::cos(outer_angle);

    double horizontal_start_x = minimum_x;
    const double inner_radius = scan_filter_.minimum_range_m;
    if (boundary_y == scan_filter_.maximum_absolute_y_m && inner_radius > boundary_y) {
      horizontal_start_x = std::max(
        horizontal_start_x,
        std::sqrt(inner_radius * inner_radius - boundary_y * boundary_y));
    }
    if (outer_x_at_boundary > horizontal_start_x) {
      append_line_segment(
        marker, horizontal_start_x, -boundary_y, outer_x_at_boundary, -boundary_y);
      append_line_segment(
        marker, outer_x_at_boundary, boundary_y, horizontal_start_x, boundary_y);
    }

    for (std::size_t index = 0; index < kArcSegments; ++index) {
      const double first_fraction = static_cast<double>(index) /
        static_cast<double>(kArcSegments);
      const double second_fraction = static_cast<double>(index + 1U) /
        static_cast<double>(kArcSegments);
      const double first_angle = -outer_angle + 2.0 * outer_angle * first_fraction;
      const double second_angle = -outer_angle + 2.0 * outer_angle * second_fraction;
      append_line_segment(
        marker,
        outer_radius * std::cos(first_angle),
        outer_radius * std::sin(first_angle),
        outer_radius * std::cos(second_angle),
        outer_radius * std::sin(second_angle));
    }

    double inner_y_at_minimum_x = 0.0;
    if (inner_radius > minimum_x) {
      inner_y_at_minimum_x = std::sqrt(
        inner_radius * inner_radius - minimum_x * minimum_x);
    }
    if (inner_y_at_minimum_x < boundary_y) {
      append_line_segment(
        marker, minimum_x, -boundary_y, minimum_x, -inner_y_at_minimum_x);
      append_line_segment(
        marker, minimum_x, inner_y_at_minimum_x, minimum_x, boundary_y);
    }

    if (inner_radius > minimum_x) {
      const double angle_limited_by_x = std::acos(minimum_x / inner_radius);
      const double angle_limited_by_y = scan_filter_.maximum_absolute_y_m >= inner_radius ?
        kPi / 2.0 : std::asin(scan_filter_.maximum_absolute_y_m / inner_radius);
      const double inner_angle = std::min(angle_limited_by_x, angle_limited_by_y);
      for (std::size_t index = 0; index < kArcSegments; ++index) {
        const double first_fraction = static_cast<double>(index) /
          static_cast<double>(kArcSegments);
        const double second_fraction = static_cast<double>(index + 1U) /
          static_cast<double>(kArcSegments);
        const double first_angle = -inner_angle + 2.0 * inner_angle * first_fraction;
        const double second_angle = -inner_angle + 2.0 * inner_angle * second_fraction;
        append_line_segment(
          marker,
          inner_radius * std::cos(first_angle),
          inner_radius * std::sin(first_angle),
          inner_radius * std::cos(second_angle),
          inner_radius * std::sin(second_angle));
      }
    }
    return marker;
  }

  visualization_msgs::msg::Marker make_roi_delete_marker(
    const sensor_msgs::msg::LaserScan & scan) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = scan.header;
    marker.ns = kRoiMarkerNamespace;
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    return marker;
  }

  std::string scan_topic_;
  std::string object_topic_;
  std::string marker_topic_;
  std::string fitted_circle_marker_topic_;
  ScanFilterConfig scan_filter_;
  double epsilon_m_;
  std::size_t minimum_samples_;
  std::size_t minimum_cluster_points_;
  double maximum_cluster_extent_m_;
  std::size_t maximum_objects_;
  bool publish_markers_;
  double marker_point_size_m_;
  double marker_center_size_m_;
  double marker_roi_line_width_m_;
  double marker_circle_line_width_m_;
  std::size_t previous_marker_count_{0U};
  std::size_t previous_circle_marker_count_{0U};

  rclcpp::Publisher<interfaces::msg::Objects>::SharedPtr object_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    fitted_circle_marker_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
};

}  // namespace object_detection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<object_detection::ObjectDetectionNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("object_detection_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
