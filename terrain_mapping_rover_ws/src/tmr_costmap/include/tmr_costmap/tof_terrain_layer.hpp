/**
 * @file tof_terrain_layer.hpp
 * @brief Nav2 Costmap Layer plugin for ToF terrain classification
 * 
 * This layer processes ToF point cloud data to classify terrain and
 * update the costmap with traversability information.
 */

#ifndef TMR_COSTMAP__TOF_TERRAIN_LAYER_HPP_
#define TMR_COSTMAP__TOF_TERRAIN_LAYER_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pluginlib/class_list_macros.hpp>

#include "tmr_costmap/terrain_classifier.hpp"
#include "tmr_costmap/point_cloud_processor.hpp"

namespace tmr_costmap
{

/**
 * @brief Nav2 Costmap Layer for ToF terrain classification
 * 
 * Subscribes to ToF point cloud data, classifies terrain based on
 * height and slope analysis, and updates the costmap accordingly.
 */
class ToFTerrainLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  /**
   * @brief Constructor
   */
  ToFTerrainLayer();

  /**
   * @brief Destructor
   */
  virtual ~ToFTerrainLayer();

  /**
   * @brief Initialize the layer
   * Called by the LayeredCostmap after the layer is created.
   */
  virtual void onInitialize() override;

  /**
   * @brief Update the bounds of the layer
   * @param robot_x Robot X position
   * @param robot_y Robot Y position
   * @param robot_yaw Robot orientation
   * @param min_x Output minimum X bound
   * @param min_y Output minimum Y bound
   * @param max_x Output maximum X bound
   * @param max_y Output maximum Y bound
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double* min_x, double* min_y,
    double* max_x, double* max_y) override;

  /**
   * @brief Update the costs in the master costmap
   * @param master_grid The master costmap to update
   * @param min_i Minimum X index
   * @param min_j Minimum Y index
   * @param max_i Maximum X index
   * @param max_j Maximum Y index
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid,
    int min_i, int min_j,
    int max_i, int max_j) override;

  /**
   * @brief Reset the layer
   */
  virtual void reset() override;

  /**
   * @brief Check if the layer is clearable
   */
  virtual bool isClearable() override { return true; }

  /**
   * @brief Activate the layer
   */
  virtual void activate() override;

  /**
   * @brief Deactivate the layer
   */
  virtual void deactivate() override;

private:
  /**
   * @brief Callback for point cloud messages
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief Process point cloud and update internal costmap
   */
  void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief Update the internal costmap with terrain cells
   */
  void updateTerrainCostmap(const std::vector<TerrainCell>& cells);

  /**
   * @brief Clear old observations
   */
  void clearOldObservations();

  /**
   * @brief Publish terrain visualization markers
   */
  void publishVisualization(const std::vector<TerrainCell>& cells);

  /**
   * @brief Declare and get parameters
   */
  void declareParameters();
  void getParameters();

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Processing components
  std::unique_ptr<TerrainClassifier> terrain_classifier_;
  std::unique_ptr<PointCloudProcessor> cloud_processor_;

  // State
  std::mutex data_mutex_;
  std::vector<TerrainCell> latest_terrain_cells_;
  rclcpp::Time last_observation_time_;
  std::atomic<bool> has_new_data_{false};

  // Parameters
  std::string point_cloud_topic_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double observation_persistence_;  // How long to keep observations
  double update_frequency_;
  bool publish_visualization_;
  bool enabled_;

  // Bounds tracking
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  bool need_recalculation_;

  // Configuration
  TerrainClassifierConfig classifier_config_;
  PointCloudProcessorConfig processor_config_;
};

}  // namespace tmr_costmap

#endif  // TMR_COSTMAP__TOF_TERRAIN_LAYER_HPP_
