/**
 * @file point_cloud_processor.cpp
 * @brief Implementation of point cloud processing
 */

#include "tmr_costmap/point_cloud_processor.hpp"

#include <chrono>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace tmr_costmap
{

PointCloudProcessor::PointCloudProcessor()
{
}

PointCloudProcessor::PointCloudProcessor(const PointCloudProcessorConfig& config)
  : config_(config)
{
}

void PointCloudProcessor::setConfig(const PointCloudProcessorConfig& config)
{
  config_ = config;
}

void PointCloudProcessor::setTFBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  tf_buffer_ = tf_buffer;
}

bool PointCloudProcessor::processPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
  PointCloudPtr& processed_cloud,
  rclcpp::Logger logger)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  
  // Transform to target frame
  if (!transformCloud(cloud_msg, processed_cloud, logger)) {
    return false;
  }

  last_stats_.input_points = processed_cloud->size();

  // Apply filters
  if (config_.enable_voxel_filter && processed_cloud->size() > 100) {
    applyVoxelFilter(processed_cloud);
  }

  applyRangeFilter(processed_cloud);
  applyHeightFilter(processed_cloud);

  if (config_.enable_outlier_removal && processed_cloud->size() > 50) {
    applyOutlierRemoval(processed_cloud);
  }

  last_stats_.output_points = processed_cloud->size();

  auto end_time = std::chrono::high_resolution_clock::now();
  last_stats_.processing_time_ms = 
    std::chrono::duration<double, std::milli>(end_time - start_time).count();

  return !processed_cloud->empty();
}

bool PointCloudProcessor::transformCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
  PointCloudPtr& cloud_out,
  rclcpp::Logger logger)
{
  if (!tf_buffer_) {
    // No transform needed, just convert
    cloud_out.reset(new PointCloud);
    pcl::fromROSMsg(*cloud_msg, *cloud_out);
    return true;
  }

  try {
    // Check if transform is available
    if (!tf_buffer_->canTransform(
        config_.target_frame,
        cloud_msg->header.frame_id,
        cloud_msg->header.stamp,
        rclcpp::Duration::from_seconds(config_.transform_timeout)))
    {
      RCLCPP_WARN(logger, "Transform not available: %s -> %s",
        cloud_msg->header.frame_id.c_str(), config_.target_frame.c_str());
      return false;
    }

    // Get transform
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      config_.target_frame,
      cloud_msg->header.frame_id,
      cloud_msg->header.stamp,
      rclcpp::Duration::from_seconds(config_.transform_timeout));

    // Transform cloud
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    tf2::doTransform(*cloud_msg, transformed_cloud, transform);

    // Convert to PCL
    cloud_out.reset(new PointCloud);
    pcl::fromROSMsg(transformed_cloud, *cloud_out);

    return true;

  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(logger, "Transform error: %s", ex.what());
    return false;
  }
}

void PointCloudProcessor::applyVoxelFilter(PointCloudPtr& cloud)
{
  PointCloudPtr filtered(new PointCloud);
  
  voxel_filter_.setInputCloud(cloud);
  voxel_filter_.setLeafSize(
    config_.voxel_size,
    config_.voxel_size,
    config_.voxel_size);
  voxel_filter_.filter(*filtered);
  
  cloud.swap(filtered);
}

void PointCloudProcessor::applyRangeFilter(PointCloudPtr& cloud)
{
  PointCloudPtr filtered(new PointCloud);
  filtered->reserve(cloud->size());

  double min_sq = config_.min_range * config_.min_range;
  double max_sq = config_.max_range * config_.max_range;

  for (const auto& point : cloud->points) {
    double dist_sq = point.x * point.x + point.y * point.y;
    if (dist_sq >= min_sq && dist_sq <= max_sq) {
      filtered->push_back(point);
    }
  }

  cloud.swap(filtered);
}

void PointCloudProcessor::applyHeightFilter(PointCloudPtr& cloud)
{
  pass_filter_.setInputCloud(cloud);
  pass_filter_.setFilterFieldName("z");
  pass_filter_.setFilterLimits(config_.min_height, config_.max_height);
  
  PointCloudPtr filtered(new PointCloud);
  pass_filter_.filter(*filtered);
  cloud.swap(filtered);
}

void PointCloudProcessor::applyOutlierRemoval(PointCloudPtr& cloud)
{
  PointCloudPtr filtered(new PointCloud);
  
  outlier_filter_.setInputCloud(cloud);
  outlier_filter_.setMeanK(config_.outlier_mean_k);
  outlier_filter_.setStddevMulThresh(config_.outlier_std_thresh);
  outlier_filter_.filter(*filtered);
  
  cloud.swap(filtered);
}

}  // namespace tmr_costmap