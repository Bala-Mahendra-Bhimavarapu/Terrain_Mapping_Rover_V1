/**
 * @file motor_controller.hpp
 * @brief Motor control with direction handling and velocity conversion
 */

#ifndef TMR_MOTOR_CONTROLLER_HPP_
#define TMR_MOTOR_CONTROLLER_HPP_

#include "api.h"
#include "config.hpp"
#include <cstdint>
#include <cmath>

namespace tmr {

/**
 * @brief Motor controller for differential drive robot
 */
class MotorController {
public:
    MotorController();
    ~MotorController();
    
    /**
     * @brief Initialize motors
     */
    void initialize();
    
    /**
     * @brief Set robot velocity (differential drive kinematics)
     * @param linear_ms Linear velocity in m/s (positive = forward)
     * @param angular_rads Angular velocity in rad/s (positive = CCW)
     */
    void setVelocity(float linear_ms, float angular_rads);
    
    /**
     * @brief Set individual wheel velocities
     * @param left_ms Left wheel velocity in m/s
     * @param right_ms Right wheel velocity in m/s
     */
    void setWheelVelocities(float left_ms, float right_ms);
    
    /**
     * @brief Stop all motors immediately
     */
    void stop();
    
    /**
     * @brief Enable/disable motors
     */
    void setEnabled(bool enabled);
    
    /**
     * @brief Check if motors are enabled
     */
    bool isEnabled() const { return enabled_; }
    
    /**
     * @brief Get encoder ticks for all wheels
     * 
     * Note:  PROS automatically handles motor reversal, so encoder
     * values are already corrected.  Positive = forward motion.
     */
    void getEncoderTicks(int32_t& lf, int32_t& rf, int32_t& lr, int32_t& rr);
    
    /**
     * @brief Get encoder ticks averaged by side
     */
    void getEncoderTicksAveraged(int32_t& left, int32_t& right);
    
    /**
     * @brief Reset encoder positions
     */
    void resetEncoders();
    
    /**
     * @brief Get motor temperatures
     */
    void getTemperatures(double& lf, double& rf, double& lr, double& rr);
    
    /**
     * @brief Get motor currents in mA
     */
    void getCurrents(int32_t& lf, int32_t& rf, int32_t& lr, int32_t& rr);
    
    /**
     * @brief Check for motor faults
     * @return true if any motor has a fault
     */
    bool hasMotorFault();
    
    /**
     * @brief Get current commanded velocities
     */
    float getCommandedLinear() const { return cmd_linear_ms_; }
    float getCommandedAngular() const { return cmd_angular_rads_; }
    
    /**
     * @brief Get actual wheel velocities in m/s
     */
    void getActualWheelVelocities(float& left_ms, float& right_ms);

private:
    /**
     * @brief Convert wheel velocity (m/s) to motor RPM
     */
    float velocityToRpm(float velocity_ms);
    
    /**
     * @brief Convert encoder ticks to meters
     */
    float ticksToMeters(int32_t ticks);
    
    // Motor objects
    pros::Motor* left_front_;
    pros::Motor* right_front_;
    pros::Motor* left_rear_;
    pros:: Motor* right_rear_;
    
    // State
    bool enabled_ = true;
    float cmd_linear_ms_ = 0.0f;
    float cmd_angular_rads_ = 0.0f;
    
    // Encoder offsets for reset
    int32_t offset_lf_ = 0;
    int32_t offset_rf_ = 0;
    int32_t offset_lr_ = 0;
    int32_t offset_rr_ = 0;
};

}  // namespace tmr

#endif  // TMR_MOTOR_CONTROLLER_HPP_
