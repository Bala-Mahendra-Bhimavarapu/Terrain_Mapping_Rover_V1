/**
 * @file serial_protocol.hpp
 * @brief Serial communication protocol with Raspberry Pi
 */

#ifndef TMR_SERIAL_PROTOCOL_HPP_
#define TMR_SERIAL_PROTOCOL_HPP_

#include "api.h"
#include <cstdint>
#include <cstring>

namespace tmr {

// =============================================================================
// PROTOCOL CONSTANTS
// =============================================================================

constexpr uint8_t START_BYTE = 0xAA;
constexpr uint8_t END_BYTE = 0x55;
constexpr size_t MAX_PAYLOAD_SIZE = 32;
constexpr size_t RX_BUFFER_SIZE = 128;
constexpr size_t TX_BUFFER_SIZE = 64;

// =============================================================================
// COMMAND TYPES (Pi -> VEX)
// =============================================================================

enum class CommandType :  uint8_t {
    SET_VELOCITY      = 0x01,  // Set linear/angular velocity
    SET_MOTOR_PWM     = 0x02,  // Set raw motor PWM (not used)
    REQUEST_STATUS    = 0x03,  // Request status packet
    REQUEST_ENCODERS  = 0x04,  // Request encoder data
    EMERGENCY_STOP    = 0x05,  // Emergency stop all motors
    RESET_ENCODERS    = 0x06,  // Reset encoder counts to zero
    SET_PID_GAINS     = 0x07,  // Set PID parameters (future)
    HEARTBEAT         = 0x08,  // Keep-alive heartbeat
    CLEAR_ESTOP       = 0x09,  // Clear emergency stop
};

// =============================================================================
// MESSAGE TYPES (VEX -> Pi)
// =============================================================================

enum class MessageType : uint8_t {
    STATUS         = 0x10,  // Status (battery, flags)
    ENCODER_DATA   = 0x11,  // Encoder tick counts
    ODOMETRY       = 0x12,  // Computed odometry (optional)
    ACK            = 0x13,  // Command acknowledgment
    ERROR          = 0x14,  // Error report
    HEARTBEAT_ACK  = 0x15,  // Heartbeat response
};

// =============================================================================
// ERROR CODES
// =============================================================================

enum class ErrorCode : uint8_t {
    NONE              = 0x00,
    INVALID_COMMAND   = 0x01,
    CHECKSUM_MISMATCH = 0x02,
    MOTOR_FAULT       = 0x03,
    ENCODER_FAULT     = 0x04,
    TIMEOUT           = 0x05,
    BUFFER_OVERFLOW   = 0x06,
    BATTERY_LOW       = 0x07,
    ESTOP_ACTIVE      = 0x08,
};

// =============================================================================
// DATA STRUCTURES
// =============================================================================

/**
 * @brief Velocity command from Pi
 */
struct VelocityCommand {
    float linear_ms;      // Linear velocity in m/s
    float angular_rads;   // Angular velocity in rad/s
};

/**
 * @brief Encoder data to send to Pi
 */
struct EncoderPacket {
    int32_t left_front_ticks;
    int32_t right_front_ticks;
    int32_t left_rear_ticks;
    int32_t right_rear_ticks;
    uint32_t timestamp_ms;
};

/**
 * @brief Status data to send to Pi
 */
struct StatusPacket {
    float battery_voltage;
    float battery_percent;
    uint8_t motors_enabled;
    uint8_t emergency_stop;
    uint8_t error_flags;
    uint32_t uptime_ms;
};

// =============================================================================
// SERIAL PROTOCOL CLASS
// =============================================================================

class SerialProtocol {
public: 
    SerialProtocol();
    
    /**
     * @brief Initialize serial communication
     */
    void initialize();
    
    /**
     * @brief Process incoming serial data
     * @return Number of complete packets processed
     */
    int processIncoming();
    
    /**
     * @brief Send encoder data to Pi
     */
    void sendEncoderData(const EncoderPacket& data);
    
    /**
     * @brief Send status data to Pi
     */
    void sendStatus(const StatusPacket& status);
    
    /**
     * @brief Send acknowledgment
     */
    void sendAck();
    
    /**
     * @brief Send heartbeat acknowledgment
     */
    void sendHeartbeatAck();
    
    /**
     * @brief Send error message
     */
    void sendError(ErrorCode code);
    
    /**
     * @brief Check if new velocity command is available
     */
    bool hasVelocityCommand() const { return velocity_command_new_; }
    
    /**
     * @brief Get the latest velocity command
     */
    VelocityCommand getVelocityCommand();
    
    /**
     * @brief Check if emergency stop was requested
     */
    bool isEmergencyStopRequested() const { return emergency_stop_requested_; }
    
    /**
     * @brief Clear emergency stop request flag
     */
    void clearEmergencyStopRequest() { emergency_stop_requested_ = false; }
    
    /**
     * @brief Check if encoder reset was requested
     */
    bool isEncoderResetRequested() const { return encoder_reset_requested_; }
    
    /**
     * @brief Clear encoder reset request flag
     */
    void clearEncoderResetRequest() { encoder_reset_requested_ = false; }
    
    /**
     * @brief Check if status was requested
     */
    bool isStatusRequested() const { return status_requested_; }
    
    /**
     * @brief Clear status request flag
     */
    void clearStatusRequest() { status_requested_ = false; }
    
    /**
     * @brief Check if encoders were requested
     */
    bool isEncodersRequested() const { return encoders_requested_; }
    
    /**
     * @brief Clear encoders request flag
     */
    void clearEncodersRequest() { encoders_requested_ = false; }
    
    /**
     * @brief Check if heartbeat was received
     */
    bool isHeartbeatReceived() const { return heartbeat_received_; }
    
    /**
     * @brief Clear heartbeat received flag
     */
    void clearHeartbeatReceived() { heartbeat_received_ = false; }
    
    /**
     * @brief Check if clear e-stop was requested
     */
    bool isClearEstopRequested() const { return clear_estop_requested_; }
    
    /**
     * @brief Clear the clear e-stop request flag
     */
    void clearClearEstopRequest() { clear_estop_requested_ = false; }
    
    /**
     * @brief Get statistics
     */
    uint32_t getPacketsReceived() const { return packets_received_; }
    uint32_t getPacketErrors() const { return packet_errors_; }
    uint32_t getBytesSent() const { return bytes_sent_; }
    uint32_t getBytesReceived() const { return bytes_received_; }

private:
    /**
     * @brief Calculate XOR checksum
     */
    uint8_t calculateChecksum(const uint8_t* data, size_t len);
    
    /**
     * @brief Send a packet
     */
    void sendPacket(MessageType type, const uint8_t* payload, size_t payload_len);
    
    /**
     * @brief Process a complete packet
     */
    void processPacket(uint8_t cmd_type, const uint8_t* payload, size_t payload_len);
    
    // Buffers
    uint8_t rx_buffer_[RX_BUFFER_SIZE];
    uint8_t tx_buffer_[TX_BUFFER_SIZE];
    size_t rx_index_ = 0;
    
    // Parser state
    enum class ParseState {
        WAIT_START,
        READ_TYPE,
        READ_LENGTH,
        READ_PAYLOAD,
        READ_CHECKSUM,
        WAIT_END
    };
    ParseState parse_state_ = ParseState::WAIT_START;
    uint8_t current_type_ = 0;
    uint8_t current_length_ = 0;
    uint8_t payload_index_ = 0;
    uint8_t current_payload_[MAX_PAYLOAD_SIZE];
    
    // Latest command data
    VelocityCommand latest_velocity_cmd_ = {0.0f, 0.0f};
    bool velocity_command_new_ = false;
    
    // Request flags
    bool emergency_stop_requested_ = false;
    bool encoder_reset_requested_ = false;
    bool status_requested_ = false;
    bool encoders_requested_ = false;
    bool heartbeat_received_ = false;
    bool clear_estop_requested_ = false;
    
    // Statistics
    uint32_t packets_received_ = 0;
    uint32_t packet_errors_ = 0;
    uint32_t bytes_sent_ = 0;
    uint32_t bytes_received_ = 0;
};

}  // namespace tmr

#endif  // TMR_SERIAL_PROTOCOL_HPP_
