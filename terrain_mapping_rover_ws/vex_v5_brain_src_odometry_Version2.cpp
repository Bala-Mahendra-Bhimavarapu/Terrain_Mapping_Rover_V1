/**
 * @file odometry. cpp
 * @brief Odometry calculation implementation
 */

#include "odometry.hpp"

namespace tmr {

Odometry::Odometry() {
    reset();
}

void Odometry::reset() {
    state_ = OdometryState();
    prev_left_ticks_ = 0;
    prev_right_ticks_ = 0;
    prev_timestamp_ms_ = 0;
    first_update_ = true;
}

float Odometry::ticksToMeters(int32_t ticks) {
    // meters = ticks * (2 * PI * wheel_radius) / ticks_per_revolution
    return static_cast<float>(ticks) * 
           (2.0f * M_PI * config::WHEEL_RADIUS_M) / 
           static_cast<float>(config::TICKS_PER_REVOLUTION);
}

float Odometry::normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void Odometry::update(int32_t left_ticks, int32_t right_ticks, uint32_t timestamp_ms) {
    if (first_update_) {
        prev_left_ticks_ = left_ticks;
        prev_right_ticks_ = right_ticks;
        prev_timestamp_ms_ = timestamp_ms;
        first_update_ = false;
        return;
    }
    
    // Calculate deltas
    int32_t delta_left = left_ticks - prev_left_ticks_;
    int32_t delta_right = right_ticks - prev_right_ticks_;
    uint32_t delta_time_ms = timestamp_ms - prev_timestamp_ms_;
    
    // Update previous values
    prev_left_ticks_ = left_ticks;
    prev_right_ticks_ = right_ticks;
    prev_timestamp_ms_ = timestamp_ms;
    
    // Guard against invalid time delta
    if (delta_time_ms == 0 || delta_time_ms > 1000) {
        return;
    }
    
    float dt = static_cast<float>(delta_time_ms) / 1000.0f;  // Convert to seconds
    
    // Convert ticks to distance
    float left_distance = ticksToMeters(delta_left);
    float right_distance = ticksToMeters(delta_right);
    
    // Calculate robot motion
    float linear_distance = (left_distance + right_distance) / 2.0f;
    float angular_distance = (right_distance - left_distance) / config::TRACK_WIDTH_M;
    
    // Calculate velocities
    state_.linear_vel_ms = linear_distance / dt;
    state_.angular_vel_rads = angular_distance / dt;
    
    // Update pose using midpoint integration
    float delta_theta = angular_distance;
    float delta_x, delta_y;
    
    if (std::abs(delta_theta) < 1e-6f) {
        // Straight line motion
        delta_x = linear_distance * std::cos(state_.theta_rad);
        delta_y = linear_distance * std::sin(state_.theta_rad);
    } else {
        // Arc motion
        float radius = linear_distance / delta_theta;
        delta_x = radius * (std::sin(state_.theta_rad + delta_theta) - std::sin(state_.theta_rad));
        delta_y = radius * (std::cos(state_.theta_rad) - std::cos(state_.theta_rad + delta_theta));
    }
    
    state_.x_m += delta_x;
    state_. y_m += delta_y;
    state_.theta_rad = normalizeAngle(state_.theta_rad + delta_theta);
    state_.timestamp_ms = timestamp_ms;
}

}  // namespace tmr