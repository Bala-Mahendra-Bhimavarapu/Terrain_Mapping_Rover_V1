/**
 * @file tof_terrain_layer.cpp
 * @brief Implementation of ToF Terrain Layer for Nav2
 */

#include "tmr_costmap/tof_terrain_layer.hpp"

#include <algorithm>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <nav2_costmap_2d/costmap_math.hpp>

// Register plugin
PLUGINLIB_EXPORT_CLASS(tmr_costmap::ToFTerrainLayer, nav2_costmap_2d::Layer)

namespace tmr_costmap
{

ToFTerrainLayer::ToFTerrainLayer()
  : last_min_x_(-std::numeric_limits<double>::max()),
    last_min_y_(-std::numeric_limits<double>::max()),
    last_max_x_(std::numeric_limits<double>::max()),
    last_max_y_(std::numeric_limits<double>::max()),
    need_recalculation_(false)
{
}

ToFTerrainLayer::~ToFTerrainLayer()
{
}

void ToFTerrainLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  // Declare and get parameters
  declareParameters();
  getParameters();

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize terrain classifier
  terrain_classifier_ = std::make_unique<TerrainClassifier>(classifier_config_);

  // Initialize point cloud processor
  cloud_processor_ = std::make_unique<PointCloudProcessor>(processor_config_);
  cloud_processor_->setTFBuffer(tf_buffer_);

  // Subscribe to point cloud
  point_cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ToFTerrainLayer::pointCloudCallback, this, std::placeholders::_1));

  // Publisher for visualization
  if (publish_visualization_) {
    viz_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "terrain_layer/visualization", 10);
  }

  // Initialize last observation time
  last_observation_time_ = node->get_clock()->now();

  RCLCPP_INFO(logger_, "ToF Terrain Layer initialized");
  RCLCPP_INFO(logger_, "  Point cloud topic: %s", point_cloud_topic_.c_str());
  RCLCPP_INFO(logger_, "  Global frame: %s", global_frame_.c_str());
  RCLCPP_INFO(logger_, "  Robot frame: %s", robot_base_frame_.c_str());

  current_ = true;
  default_value_ = nav2_costmap_2d::NO_INFORMATION;
  matchSize();
}

void ToFTerrainLayer::declareParameters()
{
  auto node = node_.lock();
  
  // Topic parameters
  declareParameter("point_cloud_topic", rclcpp::ParameterValue("/tof/points"));
  declareParameter("global_frame", rclcpp::ParameterValue("odom"));
  declareParameter("robot_base_frame", rclcpp::ParameterValue("base_link"));
  
  // Layer behavior
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("observation_persistence", rclcpp::ParameterValue(1.0));
  declareParameter("update_frequency", rclcpp::ParameterValue(10.0));
  declareParameter("publish_visualization", rclcpp::ParameterValue(true));
  
  // Terrain classifier parameters
  declareParameter("ground_height_tolerance", rclcpp::ParameterValue(0.05));
  declareParameter("low_obstacle_height", rclcpp::ParameterValue(0.10));
  declareParameter("medium_obstacle_height", rclcpp::ParameterValue(0.20));
  declareParameter("high_obstacle_height", rclcpp::ParameterValue(0.30));
  declareParameter("lethal_obstacle_height", rclcpp::ParameterValue(0.50));
  declareParameter("max_traversable_slope", rclcpp::ParameterValue(15.0));
  declareParameter("caution_slope", rclcpp::ParameterValue(25.0));
  declareParameter("max_slope", rclcpp::ParameterValue(35.0));
  declareParameter("cell_resolution", rclcpp::ParameterValue(0.05));
  declareParameter("min_points_per_cell", rclcpp::ParameterValue(3));
  
  // Point cloud processor parameters
  declareParameter("enable_voxel_filter", rclcpp::ParameterValue(true));
  declareParameter("voxel_size", rclcpp::ParameterValue(0.02));
  declareParameter("min_range", rclcpp::ParameterValue(0.15));
  declareParameter("max_range", rclcpp::ParameterValue(4.0));
  declareParameter("min_height", rclcpp::ParameterValue(-0.5));
  declareParameter("max_height", rclcpp::ParameterValue(1.5));
  declareParameter("enable_outlier_removal", rclcpp::ParameterValue(true));
}

void ToFTerrainLayer::getParameters()
{
  auto node = node_.lock();
  
  // Topic parameters
  node->get_parameter(name_ + ".point_cloud_topic", point_cloud_topic_);
  node->get_parameter(name_ + ".global_frame", global_frame_);
  node->get_parameter(name_ + ".robot_base_frame", robot_base_frame_);
  
  // Layer behavior
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".observation_persistence", observation_persistence_);
  node->get_parameter(name_ + ".update_frequency", update_frequency_);
  node->get_parameter(name_ + ".publish_visualization", publish_visualization_);
  
  // Terrain classifier config
  node->get_parameter(name_ + ".ground_height_tolerance", 
    classifier_config_.ground_height_tolerance);
  node->get_parameter(name_ + ".low_obstacle_height", 
    classifier_config_.low_obstacle_height);
  node->get_parameter(name_ + ".medium_obstacle_height", 
    classifier_config_.medium_obstacle_height);
  node->get_parameter(name_ + ".high_obstacle_height", 
    classifier_config_.high_obstacle_height);
  node->get_parameter(name_ + ".lethal_obstacle_height", 
    classifier_config_.lethal_obstacle_height);
  node->get_parameter(name_ + ".max_traversable_slope", 
    classifier_config_.max_traversable_slope);
  node->get_parameter(name_ + ".caution_slope", 
    classifier_config_.caution_slope);
  node->get_parameter(name_ + ".max_slope", 
    classifier_config_.max_slope);
  node->get_parameter(name_ + ".cell_resolution", 
    classifier_config_.cell_resolution);
  node->get_parameter(name_ + ".min_points_per_cell", 
    classifier_config_.min_points_per_cell);
  
  // Point cloud processor config
  node->get_parameter(name_ + ".enable_voxel_filter", 
    processor_config_.enable_voxel_filter);
  node->get_parameter(name_ + ".voxel_size", 
    processor_config_.voxel_size);
  node->get_parameter(name_ + ".min_range", 
    processor_config_.min_range);
  node->get_parameter(name_ + ".max_range", 
    processor_config_.max_range);
  node->get_parameter(name_ + ".min_height", 
    processor_config_.min_height);
  node->get_parameter(name_ + ".max_height", 
    processor_config_.max_height);
  node->get_parameter(name_ + ".enable_outlier_removal", 
    processor_config_.enable_outlier_removal);
  
  processor_config_.target_frame = robot_base_frame_;
}

void ToFTerrainLayer::activate()
{
  RCLCPP_INFO(logger_, "ToF Terrain Layer activated");
}

void ToFTerrainLayer::deactivate()
{
  RCLCPP_INFO(logger_, "ToF Terrain Layer deactivated");
}

void ToFTerrainLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_terrain_cells_.clear();
  has_new_data_ = false;
  resetMaps();
}

void ToFTerrainLayer::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!enabled_) {
    return;
  }

  processPointCloud(msg);
}

void ToFTerrainLayer::processPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) return;

  // Process point cloud
  PointCloudProcessor::PointCloudPtr processed_cloud;
  if (!cloud_processor_->processPointCloud(msg, processed_cloud, logger_)) {
    RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 5000,
      "Failed to process point cloud");
    return;
  }

  if (processed_cloud->empty()) {
    return;
  }

  // Get costmap bounds
  double ox = layered_costmap_->getCostmap()->getOriginX();
  double oy = layered_costmap_->getCostmap()->getOriginY();
  unsigned int size_x = layered_costmap_->getCostmap()->getSizeInCellsX();
  unsigned int size_y = layered_costmap_->getCostmap()->getSizeInCellsY();

  // Classify terrain
  std::vector<TerrainCell> cells = terrain_classifier_->classifyTerrain(
    processed_cloud, ox, oy, size_x, size_y);

  // Update data
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_terrain_cells_ = std::move(cells);
    last_observation_time_ = node->get_clock()->now();
    has_new_data_ = true;
  }

  // Publish visualization
  if (publish_visualization_ && viz_pub_) {
    publishVisualization(latest_terrain_cells_);
  }
}

void ToFTerrainLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double* min_x, double* min_y,
  double* max_x, double* max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  // Clear old observations
  auto node = node_.lock();
  if (node) {
    rclcpp::Time now = node->get_clock()->now();
    double age = (now - last_observation_time_).seconds();
    if (age > observation_persistence_) {
      latest_terrain_cells_.clear();
      has_new_data_ = false;
    }
  }

  if (latest_terrain_cells_.empty()) {
    return;
  }

  // Update bounds based on terrain cells
  double new_min_x = std::numeric_limits<double>::max();
  double new_min_y = std::numeric_limits<double>::max();
  double new_max_x = std::numeric_limits<double>::lowest();
  double new_max_y = std::numeric_limits<double>::lowest();

  for (const auto& cell : latest_terrain_cells_) {
    new_min_x = std::min(new_min_x, cell.x);
    new_min_y = std::min(new_min_y, cell.y);
    new_max_x = std::max(new_max_x, cell.x);
    new_max_y = std::max(new_max_y, cell.y);
  }

  // Expand bounds with some margin
  double margin = classifier_config_.cell_resolution * 2;
  *min_x = std::min(*min_x, new_min_x - margin);
  *min_y = std::min(*min_y, new_min_y - margin);
  *max_x = std::max(*max_x, new_max_x + margin);
  *max_y = std::max(*max_y, new_max_y + margin);

  // Store for cost update
  last_min_x_ = *min_x;
  last_min_y_ = *min_y;
  last_max_x_ = *max_x;
  last_max_y_ = *max_y;
}

void ToFTerrainLayer::updateCosts(
  nav2_costmap_2d::Costmap2D& master_grid,
  int min_i, int min_j,
  int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  if (latest_terrain_cells_.empty()) {
    return;
  }

  // Get costmap parameters
  double resolution = master_grid.getResolution();
  double origin_x = master_grid.getOriginX();
  double origin_y = master_grid.getOriginY();

  // Update costs from terrain cells
  for (const auto& cell : latest_terrain_cells_) {
    // Convert world coordinates to map coordinates
    unsigned int mx, my;
    if (!master_grid.worldToMap(cell.x, cell.y, mx, my)) {
      continue;  // Outside costmap
    }

    // Bounds check
    int imx = static_cast<int>(mx);
    int imy = static_cast<int>(my);
    if (imx < min_i || imx >= max_i || imy < min_j || imy >= max_j) {
      continue;
    }

    // Get current cost
    unsigned char current_cost = master_grid.getCost(mx, my);

    // Only update if new cost is higher (obstacles take priority)
    // Or if current is unknown
    if (cell.cost > current_cost || current_cost == nav2_costmap_2d::NO_INFORMATION) {
      master_grid.setCost(mx, my, cell.cost);
    }
  }
}

void ToFTerrainLayer::publishVisualization(const std::vector<TerrainCell>& cells)
{
  if (!viz_pub_ || cells.empty()) {
    return;
  }

  auto node = node_.lock();
  if (!node) return;

  visualization_msgs::msg::MarkerArray marker_array;

  // Create marker for each terrain class
  std::map<TerrainClass, visualization_msgs::msg::Marker> class_markers;

  // Initialize markers for each class
  auto init_marker = [&](TerrainClass tc, float r, float g, float b) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = robot_base_frame_;
    marker.header.stamp = node->get_clock()->now();
    marker.ns = "terrain_" + std::to_string(static_cast<int>(tc));
    marker.id = static_cast<int>(tc);
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = classifier_config_.cell_resolution;
    marker.scale.y = classifier_config_.cell_resolution;
    marker.scale.z = 0.02;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.7;
    marker.pose.orientation.w = 1.0;
    return marker;
  };

  class_markers[TerrainClass::FREE] = init_marker(TerrainClass::FREE, 0.0, 1.0, 0.0);
  class_markers[TerrainClass::LOW_COST] = init_marker(TerrainClass::LOW_COST, 0.5, 1.0, 0.0);
  class_markers[TerrainClass::MEDIUM_COST] = init_marker(TerrainClass::MEDIUM_COST, 1.0, 1.0, 0.0);
  class_markers[TerrainClass::HIGH_COST] = init_marker(TerrainClass::HIGH_COST, 1.0, 0.5, 0.0);
  class_markers[TerrainClass::LETHAL] = init_marker(TerrainClass::LETHAL, 1.0, 0.0, 0.0);

  // Add points to appropriate markers
  for (const auto& cell : cells) {
    if (cell.classification == TerrainClass::UNKNOWN) {
      continue;
    }

    geometry_msgs::msg::Point p;
    p.x = cell.x;
    p.y = cell.y;
    p.z = cell.mean_height;

    class_markers[cell.classification].points.push_back(p);
  }

  // Add non-empty markers to array
  for (auto& [tc, marker] : class_markers) {
    if (!marker.points.empty()) {
      marker_array.markers.push_back(marker);
    }
  }

  viz_pub_->publish(marker_array);
}

}  // namespace tmr_costmap