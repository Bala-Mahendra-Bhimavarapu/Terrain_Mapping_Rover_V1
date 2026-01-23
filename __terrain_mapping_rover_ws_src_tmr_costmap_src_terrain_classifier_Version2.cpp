/**
 * @file terrain_classifier.cpp
 * @brief Implementation of terrain classification
 */

#include "tmr_costmap/terrain_classifier.hpp"

#include <algorithm>
#include <numeric>
#include <cmath>

#include <pcl/common/common.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

namespace tmr_costmap
{

TerrainClassifier::TerrainClassifier()
  : estimated_ground_height_(0.0),
    ground_estimated_(false)
{
}

TerrainClassifier::TerrainClassifier(const TerrainClassifierConfig& config)
  : config_(config),
    estimated_ground_height_(0.0),
    ground_estimated_(false)
{
}

void TerrainClassifier::setConfig(const TerrainClassifierConfig& config)
{
  config_ = config;
}

std::vector<TerrainCell> TerrainClassifier::classifyTerrain(
  const PointCloudConstPtr& cloud,
  double origin_x, double origin_y,
  unsigned int width, unsigned int height)
{
  std::vector<TerrainCell> cells;
  
  if (!cloud || cloud->empty()) {
    return cells;
  }

  // Estimate ground plane if not done
  if (!ground_estimated_) {
    estimateGroundPlane(cloud, estimated_ground_height_);
    ground_estimated_ = true;
  }

  // Create grid of points
  double resolution = config_.cell_resolution;
  std::vector<std::vector<std::vector<PointT>>> grid(
    width, std::vector<std::vector<PointT>>(height));

  // Bin points into cells
  for (const auto& point : cloud->points) {
    // Skip invalid points
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    // Height filtering
    if (point.z < config_.min_height || point.z > config_.max_height) {
      continue;
    }

    // Calculate cell indices
    int cell_x = static_cast<int>((point.x - origin_x) / resolution);
    int cell_y = static_cast<int>((point.y - origin_y) / resolution);

    // Bounds check
    if (cell_x >= 0 && cell_x < static_cast<int>(width) &&
        cell_y >= 0 && cell_y < static_cast<int>(height)) {
      grid[cell_x][cell_y].push_back(point);
    }
  }

  // Classify each cell
  cells.reserve(width * height);
  
  for (unsigned int i = 0; i < width; ++i) {
    for (unsigned int j = 0; j < height; ++j) {
      double cell_x = origin_x + (i + 0.5) * resolution;
      double cell_y = origin_y + (j + 0.5) * resolution;
      
      TerrainCell cell = classifyCell(grid[i][j], cell_x, cell_y);
      
      if (cell.classification != TerrainClass::UNKNOWN) {
        cells.push_back(cell);
      }
    }
  }

  return cells;
}

TerrainCell TerrainClassifier::classifyCell(
  const std::vector<PointT>& points,
  double cell_x, double cell_y)
{
  TerrainCell cell;
  cell.x = cell_x;
  cell.y = cell_y;
  cell.point_count = static_cast<int>(points.size());

  // Not enough points for valid classification
  if (cell.point_count < config_.min_points_per_cell) {
    cell.classification = TerrainClass::UNKNOWN;
    cell.cost = 255;  // Unknown cost
    return cell;
  }

  // Calculate height statistics
  double sum_z = 0;
  cell.min_height = std::numeric_limits<double>::max();
  cell.max_height = std::numeric_limits<double>::lowest();

  for (const auto& point : points) {
    sum_z += point.z;
    cell.min_height = std::min(cell.min_height, static_cast<double>(point.z));
    cell.max_height = std::max(cell.max_height, static_cast<double>(point.z));
  }

  cell.mean_height = sum_z / cell.point_count;

  // Calculate roughness
  cell.roughness = calculateRoughness(points, cell.mean_height);

  // Calculate slope from height range
  double height_range = cell.max_height - cell.min_height;
  cell.slope = calculateSlope(height_range, config_.cell_resolution);

  // Calculate height above estimated ground
  double height_above_ground = cell.mean_height - estimated_ground_height_;

  // Determine terrain class
  cell.classification = determineClass(height_above_ground, cell.slope, cell.roughness);
  cell.cost = terrainClassToCost(cell.classification);

  return cell;
}

unsigned char TerrainClassifier::terrainClassToCost(TerrainClass terrain_class)
{
  switch (terrain_class) {
    case TerrainClass::FREE:
      return nav2_costmap_2d::FREE_SPACE;  // 0
    case TerrainClass::LOW_COST:
      return 50;
    case TerrainClass::MEDIUM_COST:
      return 128;
    case TerrainClass::HIGH_COST:
      return 200;
    case TerrainClass::LETHAL:
      return nav2_costmap_2d::LETHAL_OBSTACLE;  // 254
    case TerrainClass::UNKNOWN:
    default:
      return nav2_costmap_2d::NO_INFORMATION;  // 255
  }
}

bool TerrainClassifier::estimateGroundPlane(
  const PointCloudConstPtr& cloud,
  double& ground_height)
{
  if (!cloud || cloud->size() < 100) {
    ground_height = 0.0;
    return false;
  }

  // Use RANSAC to find the dominant horizontal plane
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.05);
  
  // Looking for horizontal plane (perpendicular to Z axis)
  Eigen::Vector3f axis(0, 0, 1);
  seg.setAxis(axis);
  seg.setEpsAngle(0.2);  // ~11 degrees tolerance

  PointCloudPtr cloud_copy(new PointCloud(*cloud));
  seg.setInputCloud(cloud_copy);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    // Fallback: use median Z value of lowest points
    std::vector<float> z_values;
    for (const auto& pt : cloud->points) {
      if (std::isfinite(pt.z)) {
        z_values.push_back(pt.z);
      }
    }
    
    if (z_values.empty()) {
      ground_height = 0.0;
      return false;
    }

    std::sort(z_values.begin(), z_values.end());
    // Use 10th percentile as ground estimate
    size_t idx = z_values.size() / 10;
    ground_height = z_values[idx];
    return true;
  }

  // Ground height from plane equation: ax + by + cz + d = 0
  // At origin: cz + d = 0 -> z = -d/c
  if (std::abs(coefficients->values[2]) > 0.01) {
    ground_height = -coefficients->values[3] / coefficients->values[2];
  } else {
    ground_height = 0.0;
  }

  return true;
}

double TerrainClassifier::calculateSlope(double height_diff, double distance)
{
  if (distance <= 0) return 0;
  // Return slope in degrees
  return std::atan2(height_diff, distance) * 180.0 / M_PI;
}

double TerrainClassifier::calculateRoughness(
  const std::vector<PointT>& points,
  double mean_height)
{
  if (points.size() < 2) return 0;

  double sum_sq_diff = 0;
  for (const auto& point : points) {
    double diff = point.z - mean_height;
    sum_sq_diff += diff * diff;
  }

  // Standard deviation of heights
  return std::sqrt(sum_sq_diff / points.size());
}

TerrainClass TerrainClassifier::determineClass(
  double height_above_ground,
  double slope,
  double roughness)
{
  // Check for lethal obstacles first (high obstacles)
  if (height_above_ground > config_.lethal_obstacle_height) {
    return TerrainClass::LETHAL;
  }

  // Check slope
  if (slope > config_.max_slope) {
    return TerrainClass::LETHAL;
  }

  // High obstacles
  if (height_above_ground > config_.high_obstacle_height) {
    return TerrainClass::HIGH_COST;
  }

  // Steep slope
  if (slope > config_.caution_slope) {
    return TerrainClass::HIGH_COST;
  }

  // Medium obstacles
  if (height_above_ground > config_.medium_obstacle_height) {
    return TerrainClass::MEDIUM_COST;
  }

  // Moderate slope
  if (slope > config_.max_traversable_slope) {
    return TerrainClass::MEDIUM_COST;
  }

  // Very rough terrain
  if (roughness > config_.very_rough_threshold) {
    return TerrainClass::MEDIUM_COST;
  }

  // Low obstacles
  if (height_above_ground > config_.low_obstacle_height) {
    return TerrainClass::LOW_COST;
  }

  // Rough terrain
  if (roughness > config_.rough_threshold) {
    return TerrainClass::LOW_COST;
  }

  // Check if below ground (potential pit/hole)
  if (height_above_ground < -config_.ground_height_tolerance) {
    // Negative height could be a hole - mark as obstacle
    if (height_above_ground < -config_.low_obstacle_height) {
      return TerrainClass::HIGH_COST;
    }
    return TerrainClass::MEDIUM_COST;
  }

  // Ground level, smooth terrain
  return TerrainClass::FREE;
}

}  // namespace tmr_costmap