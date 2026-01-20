/**
 * @file config.hpp
 * @brief Configuration constants for TMR VEX V5 Brain
 * 
 * ============================================================================
 * IMPORTANT:  Adjust these values to match your physical robot! 
 * ============================================================================
 */

#ifndef TMR_CONFIG_HPP_
#define TMR_CONFIG_HPP_

#include <cstdint>

namespace tmr {
namespace config {

// =============================================================================
// MOTOR PORT CONFIGURATION
// =============================================================================
// Adjust these to match your motor port assignments

constexpr int LEFT_FRONT_MOTOR_PORT = 1;
constexpr int RIGHT_FRONT_MOTOR_PORT = 2;
constexpr int LEFT_REAR_MOTOR_PORT = 3;
constexpr int RIGHT_REAR_MOTOR_PORT = 4;

// =============================================================================
// MOTOR DIRECTION CONFIGURATION
// =============================================================================
// Set to TRUE if motor needs to be reversed so that: 
// - Positive velocity = robot moves FORWARD
// - Positive encoder ticks = robot moved FORWARD
//
// Typical differential drive has right side motors reversed

constexpr bool REVERSE_LEFT_FRONT = false;
constexpr bool REVERSE_RIGHT_FRONT = true;
constexpr bool REVERSE_LEFT_REAR = false;
constexpr bool REVERSE_RIGHT_REAR = true;

// =============================================================================
// MOTOR GEARSET CONFIGURATION
// =============================================================================
// Options: E_MOTOR_GEARSET_36 (100 RPM), E_MOTOR_GEARSET_18 (200 RPM), 
//          E_MOTOR_GEARSET_06 (600 RPM)

constexpr pros::motor_gearset_e_t MOTOR_GEARSET = pros::E_MOTOR_GEARSET_36;

// Max RPM for the selected gearset
constexpr float MAX_MOTOR_RPM = 100.0f;  // For 36:1 (red) cartridge

// =============================================================================
// ROBOT PHYSICAL PARAMETERS
// =============================================================================
// These MUST match the values on the Raspberry Pi side!

// Wheel radius in meters
// 4" diameter wheels = 2" radius = 0.0508 m
constexpr float WHEEL_RADIUS_M = 0.0508f;

// Track width (distance between left and right wheel centers) in meters
// Measure from center of left wheel to center of right wheel
constexpr float TRACK_WIDTH_M = 0.295f;

// Encoder ticks per motor output shaft revolution
// V5 motor: 1800 ticks per revolution at output shaft (after internal gearbox)
constexpr int32_t TICKS_PER_REVOLUTION = 1800;

// =============================================================================
// VELOCITY LIMITS
// =============================================================================

// Maximum linear velocity in m/s
constexpr float MAX_LINEAR_VELOCITY_MS = 0.3f;

// Maximum angular velocity in rad/s
constexpr float MAX_ANGULAR_VELOCITY_RADS = 0.25f;

// Calculate max wheel velocity from motor RPM
// v = (RPM / 60) * 2 * PI * wheel_radius
constexpr float MAX_WHEEL_VELOCITY_MS = (MAX_MOTOR_RPM / 60.0f) * 2.0f * 3.14159f * WHEEL_RADIUS_M;

// =============================================================================
// SERIAL COMMUNICATION
// =============================================================================

// Baud rate for USB serial communication with Pi
constexpr int SERIAL_BAUD_RATE = 115200;

// Smart port used for USB serial (use E_LINK_SMART_PORT for USB)
// If using a smart port cable, change to the port number (1-21)
constexpr int SERIAL_PORT = 21;  // USB port

// =============================================================================
// TIMING CONFIGURATION
// =============================================================================

// Main loop delay (ms) - controls overall update rate
constexpr uint32_t MAIN_LOOP_DELAY_MS = 5;

// Encoder data send rate (ms)
constexpr uint32_t ENCODER_SEND_INTERVAL_MS = 20;  // 50 Hz

// Status send rate (ms)
constexpr uint32_t STATUS_SEND_INTERVAL_MS = 500;  // 2 Hz

// Heartbeat timeout - stop motors if no heartbeat received (ms)
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 1000;

// Command timeout - stop motors if no velocity command received (ms)
constexpr uint32_t COMMAND_TIMEOUT_MS = 500;

// =============================================================================
// SAFETY CONFIGURATION
// =============================================================================

// Minimum battery voltage before warning (volts)
constexpr float BATTERY_WARNING_VOLTAGE = 7.0f;

// Minimum battery voltage before emergency stop (volts)
constexpr float BATTERY_CRITICAL_VOLTAGE = 6.5f;

// Motor current limit for overcurrent protection (mA)
constexpr int32_t MOTOR_CURRENT_LIMIT_MA = 2500;

// Motor temperature limit (Celsius)
constexpr float MOTOR_TEMP_LIMIT_C = 55.0f;

}  // namespace config
}  // namespace tmr

#endif  // TMR_CONFIG_HPP_