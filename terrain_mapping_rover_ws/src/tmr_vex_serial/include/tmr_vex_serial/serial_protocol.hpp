/**
 * @file serial_protocol.hpp
 * @brief Serial communication protocol for VEX V5 Brain
 * 
 * Protocol format:
 * 
 * Pi -> VEX (Command):
 *   START_BYTE | CMD_TYPE | DATA...  | CHECKSUM | END_BYTE
 *   0xAA       | 1 byte   | varies  | 1 byte   | 0x55
 * 
 * VEX -> Pi (Response):
 *   START_BYTE | MSG_TYPE | DATA... | CHECKSUM | END_BYTE
 *   0xAA       | 1 byte   | varies  | 1 byte   | 0x55
 */

#ifndef TMR_VEX_SERIAL__SERIAL_PROTOCOL_HPP_
#define TMR_VEX_SERIAL__SERIAL_PROTOCOL_HPP_

#include <cstdint>
#include <vector>
#include <string>
#include <optional>
#include <termios.h>

namespace tmr_vex_serial
{

// Protocol constants
constexpr uint8_t START_BYTE = 0xAA;
constexpr uint8_t END_BYTE = 0x55;
constexpr size_t MAX_PACKET_SIZE = 64;
constexpr size_t HEADER_SIZE = 2;  // START + CMD/MSG type
constexpr size_t FOOTER_SIZE = 2;  // CHECKSUM + END

// Command types (Pi -> VEX)
enum class CommandType : uint8_t
{
    SET_VELOCITY = 0x01,      // Set wheel velocities
    SET_MOTOR_PWM = 0x02,     // Set raw motor PWM
    REQUEST_STATUS = 0x03,    // Request status update
    REQUEST_ENCODERS = 0x04,  // Request encoder values
    EMERGENCY_STOP = 0x05,    // Emergency stop all motors
    RESET_ENCODERS = 0x06,    // Reset encoder counts
    SET_PID_GAINS = 0x07,     // Set PID parameters
    HEARTBEAT = 0x08,         // Keep-alive heartbeat
};

// Message types (VEX -> Pi)
enum class MessageType : uint8_t
{
    STATUS = 0x10,            // Status update (battery, connection)
    ENCODER_DATA = 0x11,      // Encoder tick counts
    ODOMETRY = 0x12,          // Computed odometry
    ACK = 0x13,               // Command acknowledgment
    ERROR = 0x14,             // Error message
    HEARTBEAT_ACK = 0x15,     // Heartbeat acknowledgment
};

// Error codes
enum class ErrorCode : uint8_t
{
    NONE = 0x00,
    INVALID_COMMAND = 0x01,
    CHECKSUM_MISMATCH = 0x02,
    MOTOR_FAULT = 0x03,
    ENCODER_FAULT = 0x04,
    TIMEOUT = 0x05,
    BUFFER_OVERFLOW = 0x06,
};

/**
 * @brief Encoder data from VEX
 */
struct EncoderData
{
    int32_t left_front_ticks;
    int32_t right_front_ticks;
    int32_t left_rear_ticks;
    int32_t right_rear_ticks;
    uint32_t timestamp_ms;
};

/**
 * @brief Odometry data from VEX
 */
struct OdometryData
{
    float x_m;                 // X position in meters
    float y_m;                 // Y position in meters
    float theta_rad;           // Heading in radians
    float linear_vel_ms;       // Linear velocity m/s
    float angular_vel_rads;    // Angular velocity rad/s
    uint32_t timestamp_ms;     // Timestamp in milliseconds
};

/**
 * @brief Status data from VEX
 */
struct StatusData
{
    float battery_voltage;     // Battery voltage
    float battery_percent;     // Battery percentage (0-100)
    bool motors_enabled;       // Motors enabled flag
    bool emergency_stop;       // E-stop active
    uint8_t error_flags;       // Error bit flags
    uint32_t uptime_ms;        // VEX uptime in ms
};

/**
 * @brief Velocity command to VEX
 */
struct VelocityCommand
{
    float linear_ms;           // Linear velocity m/s
    float angular_rads;        // Angular velocity rad/s
};

/**
 * @brief Serial port handler for VEX communication
 */
class SerialProtocol
{
public:
    SerialProtocol();
    ~SerialProtocol();

    /**
     * @brief Open serial port
     * @param port Serial port path (e.g., "/dev/ttyACM0")
     * @param baud_rate Baud rate (default 115200)
     * @return true if successful
     */
    bool open(const std::string& port, int baud_rate = 115200);

    /**
     * @brief Close serial port
     */
    void close();

    /**
     * @brief Check if port is open
     */
    bool isOpen() const { return fd_ >= 0; }

    /**
     * @brief Send velocity command
     * @param cmd Velocity command
     * @return true if sent successfully
     */
    bool sendVelocityCommand(const VelocityCommand& cmd);

    /**
     * @brief Send emergency stop command
     * @return true if sent successfully
     */
    bool sendEmergencyStop();

    /**
     * @brief Send heartbeat
     * @return true if sent successfully
     */
    bool sendHeartbeat();

    /**
     * @brief Request status update
     * @return true if sent successfully
     */
    bool requestStatus();

    /**
     * @brief Request encoder data
     * @return true if sent successfully
     */
    bool requestEncoders();

    /**
     * @brief Reset encoder counts
     * @return true if sent successfully
     */
    bool resetEncoders();

    /**
     * @brief Read and parse incoming data
     * @return Number of complete messages parsed
     */
    int processIncoming();

    /**
     * @brief Get latest encoder data
     * @return Encoder data if available
     */
    std::optional<EncoderData> getEncoderData();

    /**
     * @brief Get latest odometry data
     * @return Odometry data if available
     */
    std::optional<OdometryData> getOdometryData();

    /**
     * @brief Get latest status data
     * @return Status data if available
     */
    std::optional<StatusData> getStatusData();

    /**
     * @brief Get last error code
     */
    ErrorCode getLastError() const { return last_error_; }

    /**
     * @brief Get bytes received count
     */
    uint64_t getBytesReceived() const { return bytes_received_; }

    /**
     * @brief Get bytes sent count
     */
    uint64_t getBytesSent() const { return bytes_sent_; }

    /**
     * @brief Get packet error count
     */
    uint32_t getPacketErrors() const { return packet_errors_; }

private:
    /**
     * @brief Calculate checksum for data
     */
    uint8_t calculateChecksum(const std::vector<uint8_t>& data);

    /**
     * @brief Send raw packet
     */
    bool sendPacket(CommandType cmd, const std::vector<uint8_t>& payload);

    /**
     * @brief Parse a complete packet
     */
    bool parsePacket(const std::vector<uint8_t>& packet);

    /**
     * @brief Configure serial port settings
     */
    bool configurePort(int baud_rate);

    /**
     * @brief Convert baud rate to termios constant
     */
    speed_t baudRateToSpeed(int baud_rate);

    // File descriptor
    int fd_ = -1;

    // Receive buffer
    std::vector<uint8_t> rx_buffer_;
    static constexpr size_t RX_BUFFER_SIZE = 256;

    // Latest data
    std::optional<EncoderData> latest_encoder_data_;
    std:: optional<OdometryData> latest_odometry_data_;
    std::optional<StatusData> latest_status_data_;
    bool encoder_data_new_ = false;
    bool odometry_data_new_ = false;
    bool status_data_new_ = false;

    // Statistics
    uint64_t bytes_received_ = 0;
    uint64_t bytes_sent_ = 0;
    uint32_t packet_errors_ = 0;
    ErrorCode last_error_ = ErrorCode::NONE;

    // Parser state
    enum class ParseState
    {
        WAIT_START,
        READ_TYPE,
        READ_DATA,
        READ_CHECKSUM,
        WAIT_END
    };
    ParseState parse_state_ = ParseState:: WAIT_START;
    std::vector<uint8_t> current_packet_;
    uint8_t expected_length_ = 0;
};

}  // namespace tmr_vex_serial

#endif  // TMR_VEX_SERIAL__SERIAL_PROTOCOL_HPP_
