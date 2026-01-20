/**
 * @file robot_config.hpp
 * @brief Robot hardware configuration and constants
 * 
 * Terrain Mapping Rover - VEX V5 Configuration
 */

#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include "pros/motors.hpp"
#include "pros/adi.hpp"

// =============================================================================
// MOTOR PORTS
// =============================================================================
constexpr int FRONT_LEFT_MOTOR_PORT = 1;
constexpr int FRONT_RIGHT_MOTOR_PORT = 2;
constexpr int BACK_LEFT_MOTOR_PORT = 11;
constexpr int BACK_RIGHT_MOTOR_PORT = 12;

// =============================================================================
// MOTOR CONFIGURATION
// =============================================================================
// Right motors are physically mirrored, so we reverse them in software
constexpr bool LEFT_MOTORS_REVERSED = false;
constexpr bool RIGHT_MOTORS_REVERSED = true;  // FIX: Reverse right motors

// Motor cartridge:  18: 1 (green cartridge, 200 RPM)
constexpr pros::motor_gearset_e_t MOTOR_GEARSET = pros:: E_MOTOR_GEARSET_18;

// =============================================================================
// ROBOT PHYSICAL PARAMETERS
// =============================================================================
constexpr double WHEEL_DIAMETER_M = 0.1016;      // 4 inches in meters
constexpr double WHEEL_RADIUS_M = 0.0508;        // 2 inches in meters
constexpr double TRACK_WIDTH_M = 0.295;          // Distance between wheel centers
constexpr double TICKS_PER_REVOLUTION = 900. 0;   // Encoder ticks per wheel revolution

// Derived constants
constexpr double WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * 3.14159265359;
constexpr double METERS_PER_TICK = WHEEL_CIRCUMFERENCE_M / TICKS_PER_REVOLUTION;

// =============================================================================
// VELOCITY LIMITS
// =============================================================================
constexpr double MAX_LINEAR_VELOCITY_MS = 0.3;   // m/s
constexpr double MAX_ANGULAR_VELOCITY_RADS = 0.25; // rad/s

// Convert m/s to motor RPM
// RPM = (velocity_ms / wheel_circumference_m) * 60
constexpr double MS_TO_RPM = 60.0 / WHEEL_CIRCUMFERENCE_M;

// =============================================================================
// SERIAL COMMUNICATION
// =============================================================================
constexpr int SERIAL_BAUD_RATE = 115200;
constexpr int ODOM_PUBLISH_RATE_MS = 20;    // 50 Hz
constexpr int STATUS_PUBLISH_RATE_MS = 1000; // 1 Hz

// =============================================================================
// SAFETY
// =============================================================================
constexpr double COMMAND_TIMEOUT_MS = 500. 0;  // Stop if no command for this long
constexpr double MIN_BATTERY_VOLTAGE = 6.5;   // Emergency stop threshold

#endif // ROBOT_CONFIG_HPP