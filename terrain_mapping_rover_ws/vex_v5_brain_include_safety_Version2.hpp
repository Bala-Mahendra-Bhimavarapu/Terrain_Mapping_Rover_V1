/**
 * @file safety.hpp
 * @brief Safety monitoring and emergency stop handling
 */

#ifndef TMR_SAFETY_HPP_
#define TMR_SAFETY_HPP_

#include "api.h"
#include "config.hpp"
#include "motor_controller.hpp"
#include <cstdint>

namespace tmr {

/**
 * @brief Safety status flags
 */
struct SafetyStatus {
    bool emergency_stop_active = false;
    bool heartbeat_timeout = false;
    bool command_timeout = false;
    bool battery_low = false;
    bool battery_critical = false;
    bool motor_fault = false;
    bool motor_overtemp = false;
    bool motor_overcurrent = false;
};

/**
 * @brief Safety monitor class
 */
class SafetyMonitor {
public:
    SafetyMonitor(MotorController& motors);
    
    /**
     * @brief Update safety checks
     * @param current_time_ms Current timestamp
     */
    void update(uint32_t current_time_ms);
    
    /**
     * @brief Record heartbeat received
     */
    void recordHeartbeat(uint32_t current_time_ms);
    
    /**
     * @brief Record velocity command received
     */
    void recordCommand(uint32_t current_time_ms);
    
    /**
     * @brief Trigger emergency stop
     */
    void triggerEmergencyStop();
    
    /**
     * @brief Clear emergency stop (if safe)
     * @return true if cleared successfully
     */
    bool clearEmergencyStop();
    
    /**
     * @brief Check if motors should be allowed to run
     */
    bool isOperationAllowed() const;
    
    /**
     * @brief Get current safety status
     */
    const SafetyStatus& getStatus() const { return status_; }
    
    /**
     * @brief Get battery voltage
     */
    float getBatteryVoltage() const { return battery_voltage_; }
    
    /**
     * @brief Get battery percentage
     */
    float getBatteryPercent() const { return battery_percent_; }
    
    /**
     * @brief Get error flags as bitmask
     */
    uint8_t getErrorFlags() const;

private:
    /**
     * @brief Check battery status
     */
    void checkBattery();
    
    /**
     * @brief Check motor health
     */
    void checkMotors();
    
    /**
     * @brief Check communication timeouts
     */
    void checkTimeouts(uint32_t current_time_ms);
    
    MotorController& motors_;
    SafetyStatus status_;
    
    uint32_t last_heartbeat_time_ = 0;
    uint32_t last_command_time_ = 0;
    
    float battery_voltage_ = 12.0f;
    float battery_percent_ = 100.0f;
};

}  // namespace tmr

#endif  // TMR_SAFETY_HPP_