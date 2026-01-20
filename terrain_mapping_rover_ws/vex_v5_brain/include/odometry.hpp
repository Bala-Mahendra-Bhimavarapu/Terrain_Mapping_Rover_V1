/**
 * @file odometry. hpp
 * @brief Odometry calculation (optional - can be done on Pi side)
 */

#ifndef TMR_ODOMETRY_HPP_
#define TMR_ODOMETRY_HPP_

#include "config.hpp"
#include <cstdint>
#include <cmath>

namespace tmr {

/**
 * @brief Odometry state
 */
struct OdometryState {
    float x_m = 0.0f;           // X position in meters
    float y_m = 0.0f;           // Y position in meters
    float theta_rad = 0.0f;     // Heading in radians
    float linear_vel_ms = 0.0f; // Linear velocity in m/s
    float angular_vel_rads = 0.0f; // Angular velocity in rad/s
    uint32_t timestamp_ms = 0;
};

/**
 * @brief Odometry calculator
 * 
 * Note: This is optional - the Pi side can calculate odometry
 * from raw encoder ticks. This is provided for local tracking
 * or if the VEX needs position information.
 */
class Odometry {
public:
    Odometry();
    
    /**
     * @brief Update odometry from encoder ticks
     * @param left_ticks Left wheel encoder ticks (averaged)
     * @param right_ticks Right wheel encoder ticks (averaged)
     * @param timestamp_ms Current timestamp in milliseconds
     */
    void update(int32_t left_ticks, int32_t right_ticks, uint32_t timestamp_ms);
    
    /**
     * @brief Reset odometry to origin
     */
    void reset();
    
    /**
     * @brief Get current odometry state
     */
    const OdometryState& getState() const { return state_; }
    
    /**
     * @brief Get X position in meters
     */
    float getX() const { return state_.x_m; }
    
    /**
     * @brief Get Y position in meters
     */
    float getY() const { return state_.y_m; }
    
    /**
     * @brief Get heading in radians
     */
    float getTheta() const { return state_.theta_rad; }
    
    /**
     * @brief Get linear velocity in m/s
     */
    float getLinearVelocity() const { return state_.linear_vel_ms; }
    
    /**
     * @brief Get angular velocity in rad/s
     */
    float getAngularVelocity() const { return state_.angular_vel_rads; }

private:
    /**
     * @brief Convert ticks to meters
     */
    float ticksToMeters(int32_t ticks);
    
    /**
     * @brief Normalize angle to [-PI, PI]
     */
    float normalizeAngle(float angle);
    
    OdometryState state_;
    
    int32_t prev_left_ticks_ = 0;
    int32_t prev_right_ticks_ = 0;
    uint32_t prev_timestamp_ms_ = 0;
    bool first_update_ = true;
};

}  // namespace tmr

#endif  // TMR_ODOMETRY_HPP_
