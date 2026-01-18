/**
 * @file serial_protocol.hpp
 * @brief Serial communication protocol for Pi <-> V5 communication
 */

#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <string>
#include <cstdint>

namespace serial {

/**
 * @brief Message types in the protocol
 */
enum class MessageType {
    CMD,      // Command from Pi
    ODOM,     // Odometry to Pi
    STATUS,   // Status to Pi
    ERROR,    // Error to Pi
    ACK       // Acknowledgment
};

/**
 * @brief Parsed command from Pi
 */
struct CommandMessage {
    double left_velocity_ms;   // m/s
    double right_velocity_ms;  // m/s
    bool valid;
    
    CommandMessage() : left_velocity_ms(0), right_velocity_ms(0), valid(false) {}
};

/**
 * @brief Calculate checksum for a message body
 * @param data Message body (without checksum)
 * @return Checksum value (0-255)
 */
int calculate_checksum(const std::string& data);

/**
 * @brief Parse a command message from the Pi
 * @param message Raw message string
 * @return Parsed command
 */
CommandMessage parse_command(const std::string& message);

/**
 * @brief Format an odometry message
 * @param left_ticks Cumulative left encoder ticks
 * @param right_ticks Cumulative right encoder ticks
 * @param left_vel Left velocity (ticks/sec)
 * @param right_vel Right velocity (ticks/sec)
 * @param timestamp_ms Timestamp in milliseconds
 * @return Formatted message with checksum
 */
std:: string format_odom_message(
    int64_t left_ticks,
    int64_t right_ticks,
    double left_vel,
    double right_vel,
    uint32_t timestamp_ms
);

/**
 * @brief Format a status message
 * @param battery_voltage Battery voltage
 * @param temps Array of 4 motor temperatures
 * @return Formatted message with checksum
 */
std:: string format_status_message(
    double battery_voltage,
    double temps[4]
);

/**
 * @brief Format an error message
 * @param code Error code
 * @param message Error description
 * @return Formatted message with checksum
 */
std::string format_error_message(
    const std::string& code,
    const std::string& message
);

}  // namespace serial

#endif  // SERIAL_PROTOCOL_HPP
