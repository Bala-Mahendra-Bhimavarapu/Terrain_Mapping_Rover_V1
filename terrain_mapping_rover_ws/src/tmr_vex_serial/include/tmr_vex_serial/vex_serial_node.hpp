/**
 * @file vex_serial_node.hpp
 * @brief ROS 2 node for VEX V5 serial communication
 */

#ifndef TMR_VEX_SERIAL__VEX_SERIAL_NODE_HPP_
#define TMR_VEX_SERIAL__VEX_SERIAL_NODE_HPP_

#include <memory>
#include <string>
#include <chrono>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_srvs/srv/trigger.hpp"

#include "tmr_vex_serial/serial_protocol.hpp"
#include "tmr_msgs/msg/vex_status.hpp"
#include "tmr_msgs/msg/encoder_ticks.hpp"
#include "tmr_msgs/msg/wheel_velocities.hpp"

namespace tmr_vex_serial
{

/**
 * @brief ROS 2 node for VEX V5 communication
 * 
 * This node: 
 * - Receives cmd_vel and sends motor commands to VEX
 * - Publishes odometry from wheel encoders
 * - Publishes VEX status (battery, connection)
 * - Publishes odom->base_link transform (optional)
 */
class VexSerialNode :  public rclcpp::Node
{
public:
    explicit VexSerialNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~VexSerialNode();

private:
    // =========================================================================
    // Initialization
    // =========================================================================
    
    /**
     * @brief Declare and get parameters
     */
    void declareParameters();

    /**
     * @brief Initialize serial connection
     */
    bool initializeSerial();

    /**
     * @brief Create publishers, subscribers, services
     */
    void createInterfaces();

    // =========================================================================
    // Callbacks
    // =========================================================================

    /**
     * @brief cmd_vel callback - convert to wheel velocities
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Timer callback for serial communication
     */
    void serialTimerCallback();

    /**
     * @brief Timer callback for publishing odometry
     */
    void odometryTimerCallback();

    /**
     * @brief Timer callback for heartbeat
     */
    void heartbeatTimerCallback();

    /**
     * @brief Timer callback for status publishing
     */
    void statusTimerCallback();

    /**
     * @brief Timer callback for watchdog (cmd_vel timeout)
     */
    void watchdogTimerCallback();

    // =========================================================================
    // Service callbacks
    // =========================================================================

    /**
     * @brief Emergency stop service
     */
    void emergencyStopCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger:: Response> response);

    /**
     * @brief Reset encoders service
     */
    void resetEncodersCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // =========================================================================
    // Odometry calculation
    // =========================================================================

    /**
     * @brief Calculate odometry from encoder data
     */
    void calculateOdometry(const EncoderData& encoder_data);

    /**
     * @brief Publish odometry message and TF
     */
    void publishOdometry();

    /**
     * @brief Convert linear/angular velocity to wheel velocities
     */
    void velocityToWheels(float linear, float angular, 
                          float& left_vel, float& right_vel);

    // =========================================================================
    // Members - Serial
    // =========================================================================
    
    std::unique_ptr<SerialProtocol> serial_;
    std::mutex serial_mutex_;
    bool connected_ = false;

    // =========================================================================
    // Members - Parameters
    // =========================================================================

    // Serial parameters
    std::string serial_port_;
    int baud_rate_;

    // Robot physical parameters
    double wheel_radius_m_;        // Wheel radius in meters
    double track_width_m_;         // Distance between wheels
    int ticks_per_revolution_;     // Encoder ticks per wheel revolution
    
    // Velocity limits
    double max_linear_velocity_;   // Max linear velocity m/s
    double max_angular_velocity_;  // Max angular velocity rad/s

    // Frame IDs
    std::string odom_frame_;
    std::string base_frame_;

    // Control parameters
    bool publish_tf_;              // Whether to publish odom->base_link TF
    double cmd_vel_timeout_;       // Timeout for cmd_vel watchdog

    // =========================================================================
    // Members - State
    // =========================================================================

    // Odometry state
    double odom_x_ = 0.0;
    double odom_y_ = 0.0;
    double odom_theta_ = 0.0;
    double odom_linear_vel_ = 0.0;
    double odom_angular_vel_ = 0.0;

    // Previous encoder values for delta calculation
    int32_t prev_left_ticks_ = 0;
    int32_t prev_right_ticks_ = 0;
    bool first_encoder_reading_ = true;
    rclcpp::Time last_encoder_time_;

    // Last command time for watchdog
    rclcpp:: Time last_cmd_vel_time_;
    bool cmd_vel_active_ = false;

    // Current velocity command
    float cmd_linear_ = 0.0;
    float cmd_angular_ = 0.0;

    // Status
    float battery_voltage_ = 0.0;
    float battery_percent_ = 100.0;
    bool emergency_stop_active_ = false;

    // =========================================================================
    // Members - ROS interfaces
    // =========================================================================

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<tmr_msgs::msg::VexStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<tmr_msgs::msg::EncoderTicks>::SharedPtr encoder_pub_;
    rclcpp:: Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_encoders_srv_;

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Timers
    rclcpp:: TimerBase::SharedPtr serial_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace tmr_vex_serial

#endif  // TMR_VEX_SERIAL__VEX_SERIAL_NODE_HPP_
