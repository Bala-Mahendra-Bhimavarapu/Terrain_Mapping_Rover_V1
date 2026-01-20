/**
 * @file safety.cpp
 * @brief Safety monitoring implementation
 */

#include "safety.hpp"

namespace tmr {

SafetyMonitor::SafetyMonitor(MotorController& motors)
    : motors_(motors)
{
}

void SafetyMonitor:: update(uint32_t current_time_ms) {
    checkBattery();
    checkMotors();
    checkTimeouts(current_time_ms);
    
    // If any critical condition, disable motors
    if (! isOperationAllowed()) {
        motors_.setEnabled(false);
        motors_.stop();
    }
}

void SafetyMonitor::recordHeartbeat(uint32_t current_time_ms) {
    last_heartbeat_time_ = current_time_ms;
    status_. heartbeat_timeout = false;
}

void SafetyMonitor::recordCommand(uint32_t current_time_ms) {
    last_command_time_ = current_time_ms;
    status_.command_timeout = false;
}

void SafetyMonitor::triggerEmergencyStop() {
    status_.emergency_stop_active = true;
    motors_.setEnabled(false);
    motors_.stop();
}

bool SafetyMonitor::clearEmergencyStop() {
    // Only clear if no other critical conditions
    if (status_.battery_critical || status_.motor_fault) {
        return false;
    }
    
    status_.emergency_stop_active = false;
    motors_.setEnabled(true);
    return true;
}

bool SafetyMonitor::isOperationAllowed() const {
    return ! status_.emergency_stop_active &&
           !status_.heartbeat_timeout &&
           !status_.battery_critical &&
           !status_. motor_fault;
}

void SafetyMonitor::checkBattery() {
    // Get battery voltage (V5 brain reports in millivolts)
    battery_voltage_ = static_cast<float>(pros::c::battery_get_voltage()) / 1000.0f;
    battery_percent_ = static_cast<float>(pros::c::battery_get_capacity());
    
    // Check thresholds
    status_.battery_critical = (battery_voltage_ < config:: BATTERY_CRITICAL_VOLTAGE);
    status_.battery_low = (battery_voltage_ < config::BATTERY_WARNING_VOLTAGE);
}

void SafetyMonitor:: checkMotors() {
    // Check for motor faults
    status_.motor_fault = motors_.hasMotorFault();
    
    // Check temperatures
    double temp_lf, temp_rf, temp_lr, temp_rr;
    motors_.getTemperatures(temp_lf, temp_rf, temp_lr, temp_rr);
    
    status_.motor_overtemp = (
        temp_lf > config:: MOTOR_TEMP_LIMIT_C ||
        temp_rf > config::MOTOR_TEMP_LIMIT_C ||
        temp_lr > config::MOTOR_TEMP_LIMIT_C ||
        temp_rr > config:: MOTOR_TEMP_LIMIT_C
    );
    
    // Check currents
    int32_t curr_lf, curr_rf, curr_lr, curr_rr;
    motors_.getCurrents(curr_lf, curr_rf, curr_lr, curr_rr);
    
    status_.motor_overcurrent = (
        curr_lf > config::MOTOR_CURRENT_LIMIT_MA ||
        curr_rf > config:: MOTOR_CURRENT_LIMIT_MA ||
        curr_lr > config::MOTOR_CURRENT_LIMIT_MA ||
        curr_rr > config::MOTOR_CURRENT_LIMIT_MA
    );
}

void SafetyMonitor::checkTimeouts(uint32_t current_time_ms) {
    // Check heartbeat timeout
    if (last_heartbeat_time_ > 0) {
        uint32_t heartbeat_age = current_time_ms - last_heartbeat_time_;
        status_.heartbeat_timeout = (heartbeat_age > config:: HEARTBEAT_TIMEOUT_MS);
    }
    
    // Check command timeout
    if (last_command_time_ > 0) {
        uint32_t command_age = current_time_ms - last_command_time_;
        status_.command_timeout = (command_age > config:: COMMAND_TIMEOUT_MS);
        
        // Stop motors on command timeout (but don't trigger full e-stop)
        if (status_.command_timeout) {
            motors_.stop();
        }
    }
}

uint8_t SafetyMonitor:: getErrorFlags() const {
    uint8_t flags = 0;
    
    if (status_.emergency_stop_active) flags |= 0x01;
    if (status_.heartbeat_timeout)     flags |= 0x02;
    if (status_.command_timeout)       flags |= 0x04;
    if (status_.battery_low)           flags |= 0x08;
    if (status_.battery_critical)      flags |= 0x10;
    if (status_.motor_fault)           flags |= 0x20;
    if (status_.motor_overtemp)        flags |= 0x40;
    if (status_.motor_overcurrent)     flags |= 0x80;
    
    return flags;
}

}  // namespace tmr
