/**
 * @file odometry.cpp
 * @brief Odometry calculation implementation
 */

#include "odometry. hpp"
#include <cmath>

namespace odom {

void OdometryTracker::reset() {
    pose_ = Pose();
    velocity_ = Velocity();
    prev_left_ticks_ = 0;
    prev_right_ticks_ = 0;
    prev_timestamp_ms_ = 0;
    initialized_ = false;
}

void OdometryTracker::reset(double x, double y, double theta) {
    reset();
    pose_.x = x;
    pose_.y = y;
    pose_.theta = theta;
}

void OdometryTracker::update(int64_t left_ticks, int64_t right_ticks, uint32_t timestamp_ms) {
    // Initialize on first call
    if (!initialized_) {
        prev_left_ticks_ = left_ticks;
        prev_right_ticks_ = right_ticks;
        prev_timestamp_ms_ = timestamp_ms;
        initialized_ = true;
        return;
    }
    
    // Calculate tick deltas
    int64_t delta_left = left_ticks - prev_left_ticks_;
    int64_t delta_right = right_ticks - prev_right_ticks_;
    
    // Calculate time delta
    uint32_t delta_time_ms = timestamp_ms - prev_timestamp_ms_;
    if (delta_time_ms == 0) {
        delta_time_ms = 1;  // Prevent division by zero
    }
    double delta_time_s = delta_time_ms / 1000.0;
    
    // Convert ticks to meters
    double delta_left_m = delta_left * config:: METERS_PER_TICK;
    double delta_right_m = delta_right * config::METERS_PER_TICK;
    
    // Differential drive kinematics
    // Distance traveled by robot center
    double delta_distance = (delta_left_m + delta_right_m) / 2.0;
    
    // Change in heading
    double delta_theta = (delta_right_m - delta_left_m) / config::TRACK_WIDTH_M;
    
    // Update pose using midpoint integration (more accurate)
    double mid_theta = pose_.theta + delta_theta / 2.0;
    
    pose_.x += delta_distance * std::cos(mid_theta);
    pose_.y += delta_distance * std::sin(mid_theta);
    pose_.theta += delta_theta;
    
    // Normalize theta to [-pi, pi]
    pose_.theta = normalize_angle(pose_.theta);
    
    // Calculate velocities
    velocity_.left = delta_left_m / delta_time_s;
    velocity_.right = delta_right_m / delta_time_s;
    velocity_.linear = delta_distance / delta_time_s;
    velocity_.angular = delta_theta / delta_time_s;
    
    // Store for next iteration
    prev_left_ticks_ = left_ticks;
    prev_right_ticks_ = right_ticks;
    prev_timestamp_ms_ = timestamp_ms;
}

double OdometryTracker::normalize_angle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

}  // namespace odom
