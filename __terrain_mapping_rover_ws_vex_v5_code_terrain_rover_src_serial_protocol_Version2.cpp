/**
 * @file serial_protocol.cpp
 * @brief Serial protocol implementation
 */

#include "serial_protocol.hpp"
#include <sstream>
#include <cstdio>

MessageType parse_message_type(const std::string& line) {
    if (line.empty()) return MessageType::UNKNOWN;
    
    if (line.rfind("CMD,", 0) == 0) return MessageType::CMD;
    if (line == "STOP") return MessageType:: STOP;
    if (line == "PING") return MessageType::PING;
    
    return MessageType::UNKNOWN;
}

MotorCommand parse_motor_command(const std::string& line) {
    MotorCommand cmd = {0, 0.0, 0.0, false};
    
    // Format: CMD,<id>,<left_vel>,<right_vel>
    int id;
    double left, right;
    
    if (sscanf(line.c_str(), "CMD,%d,%lf,%lf", &id, &left, &right) == 3) {
        cmd.command_id = id;
        cmd.left_velocity = left;
        cmd.right_velocity = right;
        cmd.valid = true;
    }
    
    return cmd;
}

std::string format_odom_message(
    uint32_t timestamp_ms,
    int32_t left_ticks,
    int32_t right_ticks,
    double left_vel,
    double right_vel
) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), 
             "ODOM,%u,%d,%d,%.4f,%.4f\n",
             timestamp_ms, left_ticks, right_ticks, left_vel, right_vel);
    return std::string(buffer);
}

std::string format_status_message(double voltage, double percent) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "STATUS,%.2f,%.1f\n", voltage, percent);
    return std::string(buffer);
}

std::string format_ack(int command_id) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "ACK,%d\n", command_id);
    return std::string(buffer);
}

std::string format_error(int code, const std::string& message) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "ERROR,%d,%s\n", code, message.c_str());
    return std::string(buffer);
}