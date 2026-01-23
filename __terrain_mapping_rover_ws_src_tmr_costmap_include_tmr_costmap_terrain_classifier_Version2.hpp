/**
 * @file terrain_classifier.hpp
 * @brief Terrain classification based on point cloud data
 * 
 * Classifies terrain into categories based on height and slope analysis:
 * - FREE: Traversable terrain
 * - LOW_COST: Slightly rough but traversable
 * - MEDIUM_COST: Rough terrain, proceed with caution
 * - HIGH_COST: Very rough, avoid if possible
 * - LETHAL: Non-traversable obstacles
 */

#ifndef TMR_COSTMAP__TERRAIN_CLASSIFIER_HPP_
#define TMR_COSTMAP__TERRAIN_CLASSIFIER_HPP_

#include <vector>
#include <memory>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace tmr_costmap
{

/**
 * @brief Terrain classification categories
 */
enum class TerrainClass
{
  UNKNOWN = 0,
  FREE = 1,
  LOW_COST = 2,
  MEDIUM_COST = 3,
  HIGH_COST = 4,
  LETHAL = 5
};

/**
 * @brief Result of terrain classification for a cell
 */
struct TerrainCell
{
  double x;                    // World X coordinate
  double y;                    // World Y coordinate
  double min_height;           // Minimum height in cell
  double max_height;           // Maximum height in cell
  double mean_height;          // Mean height in cell
  double slope;                // Estimated slope (radians)
  double roughness;            // Surface roughness metric
  int point_count;             // Number of points in cell
  TerrainClass classification; // Final classification
  unsigned char cost;          // Nav2 cost value (0-254)
  
  TerrainCell()
    : x(0), y(0), min_height(0), max_height(0), mean_height(0),
      slope(0), roughness(0), point_count(0),
      classification(TerrainClass::UNKNOWN), cost(255) {}
};

/**
 * @brief Configuration for terrain classifier
 */
struct TerrainClassifierConfig
{
  // Height thresholds (meters, relative to ground plane)
  double ground_height_tolerance = 0.05;   // Height tolerance for ground
  double low_obstacle_height = 0.10;       // Low obstacles
  double medium_obstacle_height = 0.20;    // Medium obstacles
  double high_obstacle_height = 0.30;      // High obstacles
  double lethal_obstacle_height = 0.50;    // Lethal obstacles
  
  // Slope thresholds (degrees)
  double max_traversable_slope = 15.0;     // Max slope for free traversal
  double caution_slope = 25.0;             // Slope requiring caution
  double max_slope = 35.0;                 // Maximum traversable slope
  
  // Roughness thresholds
  double smooth_threshold = 0.02;          // Smooth terrain
  double rough_threshold = 0.05;           // Rough terrain
  double very_rough_threshold = 0.10;      // Very rough terrain
  
  // Cell parameters
  double cell_resolution = 0.05;           // Grid cell size (meters)
  int min_points_per_cell = 3;             // Minimum points for valid cell
  
  // Height filtering
  double min_height = -0.5;                // Minimum valid height
  double max_height = 2.0;                 // Maximum valid height
  
  // Robot parameters
  double robot_height = 0.15;              // Robot ground clearance
  double robot_radius = 0.20;              // Robot radius for inflation
};

/**
 * @brief Classifies terrain from point cloud data
 */
class TerrainClassifier
{
public:
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;

  /**
   * @brief Constructor
   */
  TerrainClassifier();

  /**
   * @brief Constructor with configuration
   * @param config Configuration parameters
   */
  explicit TerrainClassifier(const TerrainClassifierConfig& config);

  /**
   * @brief Destructor
   */
  ~TerrainClassifier() = default;

  /**
   * @brief Set configuration
   * @param config New configuration
   */
  void setConfig(const TerrainClassifierConfig& config);

  /**
   * @brief Get current configuration
   * @return Current configuration
   */
  const TerrainClassifierConfig& getConfig() const { return config_; }

  /**
   * @brief Process point cloud and classify terrain
   * @param cloud Input point cloud (in robot base frame)
   * @param origin_x X origin of the classification grid
   * @param origin_y Y origin of the classification grid
   * @param width Grid width in cells
   * @param height Grid height in cells
   * @return Vector of classified terrain cells
   */
  std::vector<TerrainCell> classifyTerrain(
    const PointCloudConstPtr& cloud,
    double origin_x, double origin_y,
    unsigned int width, unsigned int height);

  /**
   * @brief Classify a single cell
   * @param points Points within the cell
   * @param cell_x Cell X coordinate
   * @param cell_y Cell Y coordinate
   * @return Classified terrain cell
   */
  TerrainCell classifyCell(
    const std::vector<PointT>& points,
    double cell_x, double cell_y);

  /**
   * @brief Convert terrain class to Nav2 cost value
   * @param terrain_class Terrain classification
   * @return Cost value (0-254)
   */
  static unsigned char terrainClassToCost(TerrainClass terrain_class);

  /**
   * @brief Estimate ground plane from point cloud
   * @param cloud Input point cloud
   * @param ground_height Output estimated ground height
   * @return True if ground plane found
   */
  bool estimateGroundPlane(
    const PointCloudConstPtr& cloud,
    double& ground_height);

private:
  TerrainClassifierConfig config_;
  double estimated_ground_height_;
  bool ground_estimated_;

  /**
   * @brief Calculate slope from height differences
   */
  double calculateSlope(double height_diff, double distance);

  /**
   * @brief Calculate surface roughness
   */
  double calculateRoughness(const std::vector<PointT>& points, double mean_height);

  /**
   * @brief Determine terrain class from metrics
   */
  TerrainClass determineClass(
    double height_above_ground,
    double slope,
    double roughness);
};

}  // namespace tmr_costmap

#endif  // TMR_COSTMAP__TERRAIN_CLASSIFIER_HPP_