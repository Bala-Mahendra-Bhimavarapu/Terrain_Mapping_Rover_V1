/**
 * @file motor_controller.hpp
 * @brief Motor control for differential drive robot
 */

#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include "api.h"
#include "config.hpp"

namespace motors {

/**
 * @brief Motor controller for tank drive robot
 * 
 * Handles: 
 * - Motor initialization with correct direction
 * - Velocity control
 * - Encoder reading with direction compensation
 */
class MotorController {
public:
    /**
     * @brief Initialize motor controller
     */
    void initialize();
    
    /**
     * @brief Set wheel velocities
     * @param left_velocity_ms Left wheel velocity in m/s
     * @param right_velocity_ms Right wheel velocity in m/s
     */
    void set_velocities(double left_velocity_ms, double right_velocity_ms);
    
    /**
     * @brief Stop all motors immediately
     */
    void stop();
    
    /**
     * @brief Get cumulative left encoder ticks
     * @return Encoder ticks (direction-corrected)
     */
    int64_t get_left_ticks() const;
    
    /**
     * @brief Get cumulative right encoder ticks
     * @return Encoder ticks (direction-corrected)
     */
    int64_t get_right_ticks() const;
    
    /**
     * @brief Get left wheel velocity
     * @return Velocity in ticks/second
     */
    double get_left_velocity() const;
    
    /**
     * @brief Get right wheel velocity
     * @return Velocity in ticks/second
     */
    double get_right_velocity() const;
    
    /**
     * @brief Get motor temperatures
     * @param temps Output array of 4 temperatures (FL, FR, BL, BR)
     */
    void get_temperatures(double temps[4]) const;
    
    /**
     * @brief Check if motors are initialized
     */
    bool is_initialized() const { return initialized_; }

private:
    // Motor objects
    pros::Motor* front_left_ = nullptr;
    pros::Motor* front_right_ = nullptr;
    pros::Motor* back_left_ = nullptr;
    pros::Motor* back_right_ = nullptr;
    
    // Motor groups for easier control
    pros::MotorGroup* left_motors_ = nullptr;
    pros:: MotorGroup* right_motors_ = nullptr;
    
    bool initialized_ = false;
    
    /**
     * @brief Convert m/s to motor RPM
     */
    double velocity_to_rpm(double velocity_ms) const;
    
    /**
     * @brief Convert m/s to motor voltage (mV)
     */
    double velocity_to_voltage(double velocity_ms) const;
};

}  // namespace motors

#endif  // MOTOR_CONTROLLER_HPP
