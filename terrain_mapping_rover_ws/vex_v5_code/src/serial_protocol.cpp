/**
 * @file serial_protocol.cpp
 * @brief Serial protocol implementation
 */

#include "serial_protocol.hpp"
#include "config.hpp"
#include <sstream>
#include <iomanip>
#include <cstring>

namespace serial {

int calculate_checksum(const std::string& data) {
    int sum = 0;
    for (char c : data) {
        sum += static_cast<unsigned char>(c);
    }
    return sum % config::CHECKSUM_MODULO;
}

CommandMessage parse_command(const std::string& message) {
    CommandMessage cmd;
    cmd.valid = false;
    
    // Expected format: CMD,<left_vel>,<right_vel>,<checksum>
    if (message.length() < 10 || message.substr(0, 4) != "CMD,") {
        return cmd;
    }
    
    // Find commas
    size_t pos1 = 4;  // After "CMD,"
    size_t pos2 = message.find(',', pos1);
    size_t pos3 = message. find(',', pos2 + 1);
    
    if (pos2 == std::string:: npos || pos3 == std::string::npos) {
        return cmd;
    }
    
    try {
        // Parse values
        std::string left_str = message.substr(pos1, pos2 - pos1);
        std::string right_str = message.substr(pos2 + 1, pos3 - pos2 - 1);
        std::string checksum_str = message.substr(pos3 + 1);
        
        cmd.left_velocity_ms = std::stod(left_str);
        cmd.right_velocity_ms = std::stod(right_str);
        int received_checksum = std::stoi(checksum_str);
        
        // Validate checksum
        std::string body = message.substr(0, pos3);
        int expected_checksum = calculate_checksum(body);
        
        if (received_checksum == expected_checksum) {
            cmd.valid = true;
        }
    } catch (...) {
        cmd.valid = false;
    }
    
    return cmd;
}

std::string format_odom_message(
    int64_t left_ticks,
    int64_t right_ticks,
    double left_vel,
    double right_vel,
    uint32_t timestamp_ms
) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "ODOM," << left_ticks << "," << right_ticks << ","
       << left_vel << "," << right_vel << "," << timestamp_ms;
    
    std::string body = ss.str();
    int checksum = calculate_checksum(body);
    
    return body + "," + std::to_string(checksum) + "\n";
}

std::string format_status_message(
    double battery_voltage,
    double temps[4]
) {
    std::ostringstream ss;
    ss << std:: fixed << std::setprecision(1);
    ss << "STATUS," << battery_voltage << ","
       << temps[0] << "," << temps[1] << ","
       << temps[2] << "," << temps[3];
    
    std::string body = ss.str();
    int checksum = calculate_checksum(body);
    
    return body + "," + std::to_string(checksum) + "\n";
}

std::string format_error_message(
    const std::string& code,
    const std::string& message
) {
    std::string body = "ERROR," + code + "," + message;
    int checksum = calculate_checksum(body);
    return body + "," + std::to_string(checksum) + "\n";
}

}  // namespace serial
