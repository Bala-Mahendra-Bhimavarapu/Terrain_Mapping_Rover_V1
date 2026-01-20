/**
 * @file serial_protocol.cpp
 * @brief Serial protocol implementation
 */

#include "serial_protocol.hpp"
#include "config.hpp"

namespace tmr {

SerialProtocol::SerialProtocol() {
    // Initialize buffers
    memset(rx_buffer_, 0, sizeof(rx_buffer_));
    memset(tx_buffer_, 0, sizeof(tx_buffer_));
    memset(current_payload_, 0, sizeof(current_payload_));
}

void SerialProtocol::initialize() {
    // Initialize USB serial
    // Note: pros::c::fdctl is used for USB serial control
    // The V5 brain's USB port is used for communication
    
    // USB serial is automatically available in PROS
    // No explicit initialization needed for USB
    
    // If using a smart port with serial adapter: 
    // pros::c::serial_enable(config:: SERIAL_PORT);
    // pros::c::serial_set_baudrate(config::SERIAL_PORT, config::SERIAL_BAUD_RATE);
}

uint8_t SerialProtocol::calculateChecksum(const uint8_t* data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

void SerialProtocol:: sendPacket(MessageType type, const uint8_t* payload, size_t payload_len) {
    size_t idx = 0;
    
    tx_buffer_[idx++] = START_BYTE;
    tx_buffer_[idx++] = static_cast<uint8_t>(type);
    tx_buffer_[idx++] = static_cast<uint8_t>(payload_len);
    
    // Copy payload
    if (payload != nullptr && payload_len > 0) {
        memcpy(tx_buffer_ + idx, payload, payload_len);
        idx += payload_len;
    }
    
    // Calculate checksum (type + length + payload)
    uint8_t checksum = calculateChecksum(tx_buffer_ + 1, 2 + payload_len);
    tx_buffer_[idx++] = checksum;
    tx_buffer_[idx++] = END_BYTE;
    
    // Send via USB
    // PROS uses printf/fwrite to stdout which goes to USB
    fwrite(tx_buffer_, 1, idx, stdout);
    fflush(stdout);
    
    bytes_sent_ += idx;
}

void SerialProtocol::sendEncoderData(const EncoderPacket& data) {
    uint8_t payload[20];
    
    memcpy(payload, &data. left_front_ticks, 4);
    memcpy(payload + 4, &data.right_front_ticks, 4);
    memcpy(payload + 8, &data.left_rear_ticks, 4);
    memcpy(payload + 12, &data.right_rear_ticks, 4);
    memcpy(payload + 16, &data.timestamp_ms, 4);
    
    sendPacket(MessageType::ENCODER_DATA, payload, 20);
}

void SerialProtocol::sendStatus(const StatusPacket& status) {
    uint8_t payload[15];
    
    memcpy(payload, &status.battery_voltage, 4);
    memcpy(payload + 4, &status.battery_percent, 4);
    payload[8] = status.motors_enabled;
    payload[9] = status.emergency_stop;
    payload[10] = status.error_flags;
    memcpy(payload + 11, &status.uptime_ms, 4);
    
    sendPacket(MessageType::STATUS, payload, 15);
}

void SerialProtocol:: sendAck() {
    sendPacket(MessageType::ACK, nullptr, 0);
}

void SerialProtocol:: sendHeartbeatAck() {
    sendPacket(MessageType::HEARTBEAT_ACK, nullptr, 0);
}

void SerialProtocol::sendError(ErrorCode code) {
    uint8_t payload[1] = {static_cast<uint8_t>(code)};
    sendPacket(MessageType::ERROR, payload, 1);
}

VelocityCommand SerialProtocol::getVelocityCommand() {
    velocity_command_new_ = false;
    return latest_velocity_cmd_;
}

int SerialProtocol::processIncoming() {
    int packets_processed = 0;
    
    // Read available bytes from USB stdin
    // PROS provides non-blocking read via getchar/fread when data available
    
    // Check if data is available
    // Note: In PROS, we can use pros::c::controller_get_digital for controller
    // For USB serial, we use standard I/O which may need polling
    
    // Read bytes into buffer
    int c;
    while ((c = getchar()) != EOF && rx_index_ < RX_BUFFER_SIZE) {
        rx_buffer_[rx_index_++] = static_cast<uint8_t>(c);
        bytes_received_++;
    }
    
    // Parse packets using state machine
    size_t i = 0;
    while (i < rx_index_) {
        uint8_t byte = rx_buffer_[i];
        
        switch (parse_state_) {
            case ParseState::WAIT_START: 
                if (byte == START_BYTE) {
                    parse_state_ = ParseState::READ_TYPE;
                }
                i++;
                break;
                
            case ParseState::READ_TYPE:
                current_type_ = byte;
                parse_state_ = ParseState::READ_LENGTH;
                i++;
                break;
                
            case ParseState::READ_LENGTH: 
                current_length_ = byte;
                payload_index_ = 0;
                if (current_length_ > 0 && current_length_ <= MAX_PAYLOAD_SIZE) {
                    parse_state_ = ParseState::READ_PAYLOAD;
                } else if (current_length_ == 0) {
                    parse_state_ = ParseState::READ_CHECKSUM;
                } else {
                    // Invalid length
                    parse_state_ = ParseState::WAIT_START;
                    packet_errors_++;
                }
                i++;
                break;
                
            case ParseState:: READ_PAYLOAD:
                current_payload_[payload_index_++] = byte;
                if (payload_index_ >= current_length_) {
                    parse_state_ = ParseState::READ_CHECKSUM;
                }
                i++;
                break;
                
            case ParseState:: READ_CHECKSUM:  {
                // Verify checksum
                uint8_t calc_checksum = current_type_ ^ current_length_;
                for (size_t j = 0; j < current_length_; j++) {
                    calc_checksum ^= current_payload_[j];
                }
                
                if (byte == calc_checksum) {
                    parse_state_ = ParseState::WAIT_END;
                } else {
                    // Checksum mismatch
                    parse_state_ = ParseState::WAIT_START;
                    packet_errors_++;
                }
                i++;
                break;
            }
                
            case ParseState::WAIT_END:
                if (byte == END_BYTE) {
                    // Valid packet - process it
                    processPacket(current_type_, current_payload_, current_length_);
                    packets_received_++;
                    packets_processed++;
                } else {
                    packet_errors_++;
                }
                parse_state_ = ParseState::WAIT_START;
                i++;
                break;
        }
    }
    
    // Clear processed bytes from buffer
    rx_index_ = 0;
    
    return packets_processed;
}

void SerialProtocol::processPacket(uint8_t cmd_type, const uint8_t* payload, size_t payload_len) {
    CommandType cmd = static_cast<CommandType>(cmd_type);
    
    switch (cmd) {
        case CommandType::SET_VELOCITY: 
            if (payload_len >= 8) {
                memcpy(&latest_velocity_cmd_. linear_ms, payload, 4);
                memcpy(&latest_velocity_cmd_.angular_rads, payload + 4, 4);
                velocity_command_new_ = true;
            }
            sendAck();
            break;
            
        case CommandType::REQUEST_STATUS:
            status_requested_ = true;
            break;
            
        case CommandType::REQUEST_ENCODERS:
            encoders_requested_ = true;
            break;
            
        case CommandType::EMERGENCY_STOP:
            emergency_stop_requested_ = true;
            sendAck();
            break;
            
        case CommandType:: RESET_ENCODERS:
            encoder_reset_requested_ = true;
            sendAck();
            break;
            
        case CommandType::HEARTBEAT:
            heartbeat_received_ = true;
            sendHeartbeatAck();
            break;
            
        case CommandType::CLEAR_ESTOP:
            clear_estop_requested_ = true;
            sendAck();
            break;
            
        case CommandType::SET_PID_GAINS:
            // Future implementation
            sendAck();
            break;
            
        default:
            sendError(ErrorCode::INVALID_COMMAND);
            break;
    }
}

}  // namespace tmr
