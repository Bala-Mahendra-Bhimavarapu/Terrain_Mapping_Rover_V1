/**
 * @file point_cloud_processor.hpp
 * @brief Point cloud processing utilities for terrain analysis
 */

#ifndef TMR_COSTMAP__POINT_CLOUD_PROCESSOR_HPP_
#define TMR_COSTMAP__POINT_CLOUD_PROCESSOR_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace tmr_costmap
{

/**
 * @brief Configuration for point cloud processing
 */
struct PointCloudProcessorConfig
{
  // Voxel grid downsampling
  bool enable_voxel_filter = true;
  double voxel_size = 0.02;  // meters
  
  // Range filtering
  double min_range = 0.15;   // Minimum distance from sensor
  double max_range = 4.0;    // Maximum distance from sensor
  
  // Height filtering
  double min_height = -0.5;  // Relative to base_link
  double max_height = 1.5;
  
  // Outlier removal
  bool enable_outlier_removal = true;
  int outlier_mean_k = 10;
  double outlier_std_thresh = 1.0;
  
  // Transform
  std::string target_frame = "base_link";
  double transform_timeout = 0.1;  // seconds
};

/**
 * @brief Processes point clouds for terrain analysis
 */
class PointCloudProcessor
{
public:
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloud::Ptr;

  /**
   * @brief Constructor
   */
  PointCloudProcessor();

  /**
   * @brief Constructor with configuration
   */
  explicit PointCloudProcessor(const PointCloudProcessorConfig& config);

  /**
   * @brief Destructor
   */
  ~PointCloudProcessor() = default;

  /**
   * @brief Set configuration
   */
  void setConfig(const PointCloudProcessorConfig& config);

  /**
   * @brief Set TF buffer for transforms
   */
  void setTFBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  /**
   * @brief Process a ROS point cloud message
   * @param cloud_msg Input point cloud message
   * @param processed_cloud Output processed point cloud
   * @param logger Optional logger for warnings
   * @return True if processing succeeded
   */
  bool processPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
    PointCloudPtr& processed_cloud,
    rclcpp::Logger logger = rclcpp::get_logger("point_cloud_processor"));

  /**
   * @brief Apply voxel grid filter
   */
  void applyVoxelFilter(PointCloudPtr& cloud);

  /**
   * @brief Apply range filter
   */
  void applyRangeFilter(PointCloudPtr& cloud);

  /**
   * @brief Apply height filter
   */
  void applyHeightFilter(PointCloudPtr& cloud);

  /**
   * @brief Apply statistical outlier removal
   */
  void applyOutlierRemoval(PointCloudPtr& cloud);

  /**
   * @brief Transform point cloud to target frame
   */
  bool transformCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
    PointCloudPtr& cloud_out,
    rclcpp::Logger logger);

  /**
   * @brief Get statistics about last processing
   */
  struct ProcessingStats
  {
    size_t input_points = 0;
    size_t output_points = 0;
    double processing_time_ms = 0;
  };
  ProcessingStats getLastStats() const { return last_stats_; }

private:
  PointCloudProcessorConfig config_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  ProcessingStats last_stats_;

  // PCL filters
  pcl::VoxelGrid<PointT> voxel_filter_;
  pcl::PassThrough<PointT> pass_filter_;
  pcl::StatisticalOutlierRemoval<PointT> outlier_filter_;
};

}  // namespace tmr_costmap

#endif  // TMR_COSTMAP__POINT_CLOUD_PROCESSOR_HPP_
