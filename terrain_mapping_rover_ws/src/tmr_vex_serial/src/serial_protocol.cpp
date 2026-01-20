/**
 * @file serial_protocol.cpp
 * @brief Implementation of serial protocol for VEX V5
 */

#include "tmr_vex_serial/serial_protocol.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <iostream>

namespace tmr_vex_serial
{

SerialProtocol:: SerialProtocol()
{
    rx_buffer_.reserve(RX_BUFFER_SIZE);
}

SerialProtocol::~SerialProtocol()
{
    close();
}

bool SerialProtocol::open(const std::string& port, int baud_rate)
{
    // Close if already open
    if (fd_ >= 0) {
        close();
    }

    // Open port
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "Failed to open serial port: " << port 
                  << " - " << strerror(errno) << std::endl;
        return false;
    }

    // Configure port
    if (! configurePort(baud_rate)) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Clear buffers
    tcflush(fd_, TCIOFLUSH);
    rx_buffer_.clear();
    parse_state_ = ParseState:: WAIT_START;
    current_packet_.clear();

    return true;
}

void SerialProtocol::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialProtocol::configurePort(int baud_rate)
{
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "Error getting terminal attributes:  " << strerror(errno) << std::endl;
        return false;
    }

    // Set baud rate
    speed_t speed = baudRateToSpeed(baud_rate);
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1 mode
    tty. c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 bits
    tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable read, ignore modem control

    // Raw input mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Raw output mode
    tty.c_oflag &= ~OPOST;

    // Disable software flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Read settings
    tty.c_cc[VMIN] = 0;   // Non-blocking read
    tty. c_cc[VTIME] = 0;  // No timeout

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting terminal attributes: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

speed_t SerialProtocol::baudRateToSpeed(int baud_rate)
{
    switch (baud_rate) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B115200;
    }
}

uint8_t SerialProtocol:: calculateChecksum(const std::vector<uint8_t>& data)
{
    uint8_t checksum = 0;
    for (uint8_t byte : data) {
        checksum ^= byte;
    }
    return checksum;
}

bool SerialProtocol::sendPacket(CommandType cmd, const std::vector<uint8_t>& payload)
{
    if (fd_ < 0) {
        return false;
    }

    std::vector<uint8_t> packet;
    packet.push_back(START_BYTE);
    packet.push_back(static_cast<uint8_t>(cmd));
    packet.push_back(static_cast<uint8_t>(payload.size()));
    
    for (uint8_t byte : payload) {
        packet.push_back(byte);
    }

    // Calculate checksum (excluding START_BYTE)
    std::vector<uint8_t> checksum_data(packet.begin() + 1, packet.end());
    uint8_t checksum = calculateChecksum(checksum_data);
    packet.push_back(checksum);
    packet.push_back(END_BYTE);

    // Write to serial
    ssize_t written = ::write(fd_, packet.data(), packet.size());
    if (written < 0) {
        std::cerr << "Serial write error: " << strerror(errno) << std::endl;
        return false;
    }

    bytes_sent_ += written;
    return written == static_cast<ssize_t>(packet.size());
}

bool SerialProtocol::sendVelocityCommand(const VelocityCommand& cmd)
{
    std::vector<uint8_t> payload(8);
    
    // Pack linear velocity as float (4 bytes)
    memcpy(payload.data(), &cmd.linear_ms, 4);
    
    // Pack angular velocity as float (4 bytes)
    memcpy(payload.data() + 4, &cmd.angular_rads, 4);
    
    return sendPacket(CommandType:: SET_VELOCITY, payload);
}

bool SerialProtocol:: sendEmergencyStop()
{
    return sendPacket(CommandType::EMERGENCY_STOP, {});
}

bool SerialProtocol::sendHeartbeat()
{
    return sendPacket(CommandType:: HEARTBEAT, {});
}

bool SerialProtocol::requestStatus()
{
    return sendPacket(CommandType::REQUEST_STATUS, {});
}

bool SerialProtocol::requestEncoders()
{
    return sendPacket(CommandType::REQUEST_ENCODERS, {});
}

bool SerialProtocol::resetEncoders()
{
    return sendPacket(CommandType:: RESET_ENCODERS, {});
}

int SerialProtocol::processIncoming()
{
    if (fd_ < 0) {
        return 0;
    }

    // Read available bytes
    uint8_t buffer[128];
    ssize_t bytes_read = ::read(fd_, buffer, sizeof(buffer));
    
    if (bytes_read <= 0) {
        return 0;
    }

    bytes_received_ += bytes_read;

    // Add to receive buffer
    for (ssize_t i = 0; i < bytes_read; ++i) {
        rx_buffer_. push_back(buffer[i]);
    }

    // Parse packets
    int packets_parsed = 0;
    size_t i = 0;

    while (i < rx_buffer_.size()) {
        uint8_t byte = rx_buffer_[i];

        switch (parse_state_) {
            case ParseState::WAIT_START:
                if (byte == START_BYTE) {
                    current_packet_.clear();
                    current_packet_.push_back(byte);
                    parse_state_ = ParseState::READ_TYPE;
                }
                break;

            case ParseState::READ_TYPE:
                current_packet_.push_back(byte);
                parse_state_ = ParseState::READ_DATA;
                break;

            case ParseState::READ_DATA: 
                current_packet_.push_back(byte);
                expected_length_ = byte;  // Length byte
                if (expected_length_ == 0) {
                    parse_state_ = ParseState::READ_CHECKSUM;
                } else {
                    // Read expected_length_ more bytes
                    size_t remaining = rx_buffer_. size() - i - 1;
                    size_t to_read = std::min(remaining, static_cast<size_t>(expected_length_));
                    for (size_t j = 0; j < to_read; ++j) {
                        current_packet_.push_back(rx_buffer_[i + 1 + j]);
                    }
                    i += to_read;
                    if (current_packet_.size() >= 3 + expected_length_) {
                        parse_state_ = ParseState::READ_CHECKSUM;
                    }
                }
                break;

            case ParseState:: READ_CHECKSUM:
                current_packet_.push_back(byte);
                parse_state_ = ParseState:: WAIT_END;
                break;

            case ParseState::WAIT_END:
                if (byte == END_BYTE) {
                    current_packet_.push_back(byte);
                    if (parsePacket(current_packet_)) {
                        packets_parsed++;
                    } else {
                        packet_errors_++;
                    }
                } else {
                    packet_errors_++;
                }
                parse_state_ = ParseState:: WAIT_START;
                current_packet_.clear();
                break;
        }
        i++;
    }

    // Clear processed data from buffer
    rx_buffer_.clear();

    return packets_parsed;
}

bool SerialProtocol::parsePacket(const std::vector<uint8_t>& packet)
{
    if (packet.size() < 5) {  // Minimum:  START + TYPE + LEN + CHECKSUM + END
        return false;
    }

    // Verify start/end bytes
    if (packet. front() != START_BYTE || packet.back() != END_BYTE) {
        return false;
    }

    // Extract and verify checksum
    size_t checksum_idx = packet.size() - 2;
    uint8_t received_checksum = packet[checksum_idx];
    
    std::vector<uint8_t> checksum_data(packet.begin() + 1, packet.begin() + checksum_idx);
    uint8_t calculated_checksum = calculateChecksum(checksum_data);

    if (received_checksum != calculated_checksum) {
        last_error_ = ErrorCode::CHECKSUM_MISMATCH;
        return false;
    }

    // Parse message type
    MessageType msg_type = static_cast<MessageType>(packet[1]);
    uint8_t data_len = packet[2];
    const uint8_t* data = packet.data() + 3;

    switch (msg_type) {
        case MessageType::STATUS:
            if (data_len >= 14) {
                StatusData status;
                memcpy(&status. battery_voltage, data, 4);
                memcpy(&status.battery_percent, data + 4, 4);
                status.motors_enabled = data[8] != 0;
                status.emergency_stop = data[9] != 0;
                status.error_flags = data[10];
                memcpy(&status.uptime_ms, data + 11, 4);
                
                latest_status_data_ = status;
                status_data_new_ = true;
            }
            break;

        case MessageType::ENCODER_DATA:
            if (data_len >= 20) {
                EncoderData enc;
                memcpy(&enc.left_front_ticks, data, 4);
                memcpy(&enc.right_front_ticks, data + 4, 4);
                memcpy(&enc. left_rear_ticks, data + 8, 4);
                memcpy(&enc.right_rear_ticks, data + 12, 4);
                memcpy(&enc.timestamp_ms, data + 16, 4);
                
                latest_encoder_data_ = enc;
                encoder_data_new_ = true;
            }
            break;

        case MessageType::ODOMETRY:
            if (data_len >= 24) {
                OdometryData odom;
                memcpy(&odom.x_m, data, 4);
                memcpy(&odom.y_m, data + 4, 4);
                memcpy(&odom.theta_rad, data + 8, 4);
                memcpy(&odom.linear_vel_ms, data + 12, 4);
                memcpy(&odom.angular_vel_rads, data + 16, 4);
                memcpy(&odom.timestamp_ms, data + 20, 4);
                
                latest_odometry_data_ = odom;
                odometry_data_new_ = true;
            }
            break;

        case MessageType::ACK:
            // Command acknowledged
            break;

        case MessageType::ERROR:
            if (data_len >= 1) {
                last_error_ = static_cast<ErrorCode>(data[0]);
            }
            break;

        case MessageType::HEARTBEAT_ACK:
            // Heartbeat acknowledged - connection is alive
            break;

        default:
            return false;
    }

    return true;
}

std::optional<EncoderData> SerialProtocol::getEncoderData()
{
    if (encoder_data_new_) {
        encoder_data_new_ = false;
        return latest_encoder_data_;
    }
    return std::nullopt;
}

std::optional<OdometryData> SerialProtocol::getOdometryData()
{
    if (odometry_data_new_) {
        odometry_data_new_ = false;
        return latest_odometry_data_;
    }
    return std::nullopt;
}

std::optional<StatusData> SerialProtocol::getStatusData()
{
    if (status_data_new_) {
        status_data_new_ = false;
        return latest_status_data_;
    }
    return std::nullopt;
}

}  // namespace tmr_vex_serial
