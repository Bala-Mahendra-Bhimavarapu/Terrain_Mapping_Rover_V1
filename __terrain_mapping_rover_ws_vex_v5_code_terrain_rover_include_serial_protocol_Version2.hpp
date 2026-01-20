/**
 * @file serial_protocol.hpp
 * @brief Serial communication protocol handlers
 */

#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <string>
#include <functional>

// Message types
enum class MessageType {
    UNKNOWN,
    CMD,        // Motor command
    STOP,       // Emergency stop
    PING        // Heartbeat check
};

// Parsed command structure
struct MotorCommand {
    int command_id;
    double left_velocity;   // m/s
    double right_velocity;  // m/s
    bool valid;
};

/**
 * Parse incoming message from Pi
 */
MessageType parse_message_type(const std::string& line);

/**
 * Parse motor command
 * Format: CMD,<id>,<left_vel>,<right_vel>
 */
MotorCommand parse_motor_command(const std::string& line);

/**
 * Format odometry message
 * Format:  ODOM,<timestamp_ms>,<left_ticks>,<right_ticks>,<left_vel>,<right_vel>
 */
std::string format_odom_message(
    uint32_t timestamp_ms,
    int32_t left_ticks,
    int32_t right_ticks,
    double left_vel,
    double right_vel
);

/**
 * Format status message
 * Format: STATUS,<battery_voltage>,<battery_percent>
 */
std:: string format_status_message(double voltage, double percent);

/**
 * Format acknowledgment
 */
std::string format_ack(int command_id);

/**
 * Format error message
 */
std::string format_error(int code, const std::string& message);

#endif // SERIAL_PROTOCOL_HPP