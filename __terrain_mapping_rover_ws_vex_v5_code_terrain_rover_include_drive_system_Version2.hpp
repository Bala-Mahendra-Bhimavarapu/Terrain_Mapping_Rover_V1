/**
 * @file drive_system.hpp
 * @brief Tank drive motor control and odometry
 */

#ifndef DRIVE_SYSTEM_HPP
#define DRIVE_SYSTEM_HPP

#include "pros/motors.hpp"
#include "robot_config.hpp"
#include <memory>

class DriveSystem {
public: 
    DriveSystem();
    
    /**
     * Initialize motors and encoders
     */
    void initialize();
    
    /**
     * Set motor velocities in m/s
     * Positive = forward for both sides
     */
    void set_velocity(double left_ms, double right_ms);
    
    /**
     * Emergency stop - immediately halt all motors
     */
    void stop();
    
    /**
     * Get current encoder ticks (cumulative)
     * Note: Right encoder ticks are already corrected for motor reversal
     */
    int32_t get_left_ticks() const;
    int32_t get_right_ticks() const;
    
    /**
     * Get current wheel velocities in m/s
     */
    double get_left_velocity() const;
    double get_right_velocity() const;
    
    /**
     * Reset encoder positions to zero
     */
    void reset_encoders();
    
    /**
     * Check if motors are functioning
     */
    bool is_healthy() const;

private:
    // Motor objects
    std::unique_ptr<pros::Motor> front_left_;
    std::unique_ptr<pros::Motor> front_right_;
    std::unique_ptr<pros::Motor> back_left_;
    std::unique_ptr<pros::Motor> back_right_;
    
    // Motor groups for synchronized control
    std::unique_ptr<pros::Motor_Group> left_motors_;
    std::unique_ptr<pros::Motor_Group> right_motors_;
    
    // Encoder tracking (separate from motor direction)
    int32_t left_ticks_offset_ = 0;
    int32_t right_ticks_offset_ = 0;
    
    /**
     * Convert m/s to motor velocity (RPM)
     */
    double velocity_to_rpm(double velocity_ms) const;
    
    /**
     * Get raw encoder ticks from motors
     */
    int32_t get_raw_left_ticks() const;
    int32_t get_raw_right_ticks() const;
};

#endif // DRIVE_SYSTEM_HPP