/**
 * @file config.hpp
 * @brief Robot configuration constants
 * 
 * All physical robot parameters in one place. 
 * MUST match the ROS 2 side configuration! 
 */

#ifndef CONFIG_HPP
#define CONFIG_HPP

namespace config {

// ==================== MOTOR PORTS ====================
constexpr int FRONT_LEFT_MOTOR_PORT = 1;
constexpr int FRONT_RIGHT_MOTOR_PORT = 2;
constexpr int BACK_LEFT_MOTOR_PORT = 11;
constexpr int BACK_RIGHT_MOTOR_PORT = 12;

// ==================== MOTOR DIRECTION ====================
// Right motors are physically mirrored, so we reverse them
constexpr bool REVERSE_LEFT_MOTORS = false;
constexpr bool REVERSE_RIGHT_MOTORS = true;  // Mirrored mounting

// ==================== ROBOT GEOMETRY ====================
constexpr double WHEEL_RADIUS_M = 0.0508;      // 5.08 cm
constexpr double TRACK_WIDTH_M = 0.29;         // 29 cm
constexpr double WHEEL_CIRCUMFERENCE_M = 2.0 * 3.14159265359 * WHEEL_RADIUS_M;

// ==================== ENCODER CONFIG ====================
// VEX V5 motor with 18:1 gear cartridge = 900 ticks/rev
constexpr int TICKS_PER_REVOLUTION = 900;
constexpr double METERS_PER_TICK = WHEEL_CIRCUMFERENCE_M / TICKS_PER_REVOLUTION;

// ==================== SERIAL CONFIG ====================
constexpr int SERIAL_BAUD_RATE = 115200;
constexpr int ODOM_UPDATE_RATE_MS = 20;  // 50 Hz
constexpr int STATUS_UPDATE_RATE_MS = 1000;  // 1 Hz

// ==================== VELOCITY LIMITS ====================
constexpr double MAX_WHEEL_VELOCITY_MS = 0.5;  // m/s
constexpr double MAX_MOTOR_VOLTAGE = 12000;    // mV (max for V5 motor)

// ==================== CHECKSUM ====================
constexpr int CHECKSUM_MODULO = 256;

}  // namespace config

#endif  // CONFIG_HPP
