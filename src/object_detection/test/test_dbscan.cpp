#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "object_detection/dbscan.hpp"

namespace object_detection
{
namespace
{

TEST(ScanToPoints, FiltersRangeAndFrontRegion)
{
  constexpr double pi = 3.14159265358979323846;
  const std::vector<float> ranges{1.0F, 1.0F, 1.0F, 1.0F};
  ScanFilterConfig config;
  config.minimum_forward_x_m = 0.1;
  config.maximum_absolute_y_m = 0.5;

  const auto points = scan_to_points(
    ranges, -pi / 2.0, pi / 2.0, 0.1, 16.0, config);

  ASSERT_EQ(points.size(), 1U);
  EXPECT_NEAR(points[0].x, 1.0, 1.0e-9);
  EXPECT_NEAR(points[0].y, 0.0, 1.0e-9);
}

TEST(ScanToPoints, RejectsInvalidAndOutOfRangeValues)
{
  const std::vector<float> ranges{
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity(),
    0.05F,
    5.0F,
  };

  const auto points = scan_to_points(
    ranges, 0.0, 0.01, 0.1, 16.0, ScanFilterConfig{});

  EXPECT_TRUE(points.empty());
}

TEST(DbscanLabels, FindsTwoClustersAndNoise)
{
  const std::vector<Point2D> points{
    {1.00, 0.00}, {1.03, 0.00}, {1.00, 0.03}, {1.03, 0.03},
    {2.00, 0.50}, {2.04, 0.50}, {2.00, 0.54}, {2.04, 0.54},
    {3.50, 1.20},
  };

  const auto labels = dbscan_labels(points, 0.08, 3U);

  ASSERT_EQ(labels.size(), points.size());
  EXPECT_GE(labels[0], 0);
  EXPECT_EQ(labels[0], labels[1]);
  EXPECT_EQ(labels[0], labels[2]);
  EXPECT_GE(labels[4], 0);
  EXPECT_EQ(labels[4], labels[7]);
  EXPECT_NE(labels[0], labels[4]);
  EXPECT_EQ(labels[8], -1);
}

TEST(MinimumEnclosingCircle, RecoversCenterAndRadius)
{
  constexpr double center_x = 1.2;
  constexpr double center_y = -0.4;
  constexpr double radius = 0.3;
  const std::vector<Point2D> points{
    {center_x + radius, center_y},
    {center_x, center_y + radius},
    {center_x - radius, center_y},
    {center_x, center_y - radius},
  };

  const auto circle = minimum_enclosing_circle(points);

  EXPECT_NEAR(circle.center.x, center_x, 1.0e-12);
  EXPECT_NEAR(circle.center.y, center_y, 1.0e-12);
  EXPECT_NEAR(circle.radius_m, radius, 1.0e-12);
}

TEST(MinimumEnclosingCircle, UsesEndpointDiameterForNearlyCollinearPoints)
{
  const std::vector<Point2D> points{
    {2.444336, 0.289711},
    {2.523225, 0.321445},
    {2.629801, 0.358403},
  };

  const auto circle = minimum_enclosing_circle(points);
  const double expected_center_x = (points.front().x + points.back().x) * 0.5;
  const double expected_center_y = (points.front().y + points.back().y) * 0.5;
  const double expected_radius = std::hypot(
    points.front().x - points.back().x,
    points.front().y - points.back().y) * 0.5;

  EXPECT_NEAR(circle.center.x, expected_center_x, 1.0e-12);
  EXPECT_NEAR(circle.center.y, expected_center_y, 1.0e-12);
  EXPECT_NEAR(circle.radius_m, expected_radius, 1.0e-12);
}

TEST(MinimumEnclosingCircle, UsesThreeBoundaryPointsForAcuteTriangle)
{
  const double square_root_three = std::sqrt(3.0);
  const std::vector<Point2D> points{
    {0.0, 0.0}, {2.0, 0.0}, {1.0, square_root_three}, {1.0, 0.5},
  };

  const auto circle = minimum_enclosing_circle(points);

  EXPECT_NEAR(circle.center.x, 1.0, 1.0e-12);
  EXPECT_NEAR(circle.center.y, square_root_three / 3.0, 1.0e-12);
  EXPECT_NEAR(circle.radius_m, 2.0 / square_root_three, 1.0e-12);
}

TEST(DetectClusters, RejectsLongWallAndKeepsCompactObject)
{
  std::vector<Point2D> points{
    {1.00, 0.00}, {1.04, 0.00}, {1.00, 0.04}, {1.04, 0.04},
  };
  for (std::size_t index = 0; index < 10U; ++index) {
    points.push_back(Point2D{2.0, static_cast<double>(index) * 0.1});
  }

  const auto clusters = detect_clusters(points, 0.12, 3U, 3U, 0.30, 20U);

  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_EQ(clusters[0].points.size(), 4U);
  EXPECT_NEAR(clusters[0].center.x, 1.02, 1.0e-9);
  EXPECT_NEAR(clusters[0].center.y, 0.02, 1.0e-9);
}

TEST(DetectClusters, SortsByDistanceAndLimitsOutputCount)
{
  const std::vector<Point2D> points{
    {2.00, 0.00}, {2.03, 0.00}, {2.00, 0.03},
    {0.50, 0.00}, {0.53, 0.00}, {0.50, 0.03},
  };

  const auto clusters = detect_clusters(points, 0.08, 3U, 3U, 0.30, 1U);

  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_LT(clusters[0].center.x, 1.0);
}

TEST(DetectClusters, StoresMinimumEnclosingCircle)
{
  const std::vector<Point2D> points{
    {1.10, 0.00}, {1.00, 0.10}, {0.90, 0.00}, {1.00, -0.10},
  };

  const auto clusters = detect_clusters(points, 0.15, 3U, 3U, 0.50, 20U);

  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_NEAR(clusters[0].center.x, 1.0, 1.0e-12);
  EXPECT_NEAR(clusters[0].center.y, 0.0, 1.0e-12);
  EXPECT_NEAR(clusters[0].radius_m, 0.1, 1.0e-12);
}

TEST(DetectClusters, RejectsClusterBelowMinimumPointCount)
{
  const std::vector<Point2D> points{
    {1.00, 0.00}, {1.03, 0.00}, {1.00, 0.03}, {1.03, 0.03},
  };

  const auto clusters = detect_clusters(points, 0.08, 3U, 5U, 0.30, 20U);

  EXPECT_TRUE(clusters.empty());
}

}  // namespace
}  // namespace object_detection
