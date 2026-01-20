/**
 * @file vex_serial_node.cpp
 * @brief Implementation of VEX Serial ROS 2 node
 */

#include "tmr_vex_serial/vex_serial_node. hpp"

#include <cmath>
#include <chrono>

using namespace std:: chrono_literals;

namespace tmr_vex_serial
{

VexSerialNode::VexSerialNode(const rclcpp::NodeOptions& options)
    : Node("vex_serial_node", options)
{
    RCLCPP_INFO(get_logger(), "Initializing VEX Serial Node.. .");

    // Initialize
    declareParameters();
    createInterfaces();

    // Create serial protocol handler
    serial_ = std::make_unique<SerialProtocol>();

    // Try to connect
    if (!initializeSerial()) {
        RCLCPP_WARN(get_logger(), "Failed to connect to VEX.  Will retry...");
    }

    // Initialize timestamps
    last_cmd_vel_time_ = this->now();
    last_encoder_time_ = this->now();

    RCLCPP_INFO(get_logger(), "VEX Serial Node initialized");
}

VexSerialNode::~VexSerialNode()
{
    // Send stop command before closing
    if (serial_ && serial_->isOpen()) {
        VelocityCommand stop_cmd{0.0f, 0.0f};
        serial_->sendVelocityCommand(stop_cmd);
        serial_->close();
    }
}

void VexSerialNode::declareParameters()
{
    // Serial parameters
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter("baud_rate", 115200);

    // Robot parameters
    this->declare_parameter("wheel_radius_m", 0.0508);  // 2 inches = 0.0508m
    this->declare_parameter("track_width_m", 0.295);    // ~11. 6 inches
    this->declare_parameter("ticks_per_revolution", 900);

    // Velocity limits
    this->declare_parameter("max_linear_velocity_ms", 0.3);
    this->declare_parameter("max_angular_velocity_rads", 0.25);

    // Frame IDs
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_link");

    // Control parameters
    this->declare_parameter("publish_tf", false);  // EKF publishes TF
    this->declare_parameter("cmd_vel_timeout", 0.5);

    // Get parameters
    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    wheel_radius_m_ = this->get_parameter("wheel_radius_m").as_double();
    track_width_m_ = this->get_parameter("track_width_m").as_double();
    ticks_per_revolution_ = this->get_parameter("ticks_per_revolution").as_int();
    max_linear_velocity_ = this->get_parameter("max_linear_velocity_ms").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity_rads").as_double();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();

    RCLCPP_INFO(get_logger(), "Parameters:");
    RCLCPP_INFO(get_logger(), "  Serial port: %s", serial_port_.c_str());
    RCLCPP_INFO(get_logger(), "  Baud rate: %d", baud_rate_);
    RCLCPP_INFO(get_logger(), "  Wheel radius: %. 4f m", wheel_radius_m_);
    RCLCPP_INFO(get_logger(), "  Track width: %.4f m", track_width_m_);
    RCLCPP_INFO(get_logger(), "  Max linear vel: %.2f m/s", max_linear_velocity_);
    RCLCPP_INFO(get_logger(), "  Max angular vel:  %.2f rad/s", max_angular_velocity_);
}

void VexSerialNode:: createInterfaces()
{
    // Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "odom_raw", 10);
    
    status_pub_ = this->create_publisher<tmr_msgs::msg::VexStatus>(
        "status", 10);
    
    encoder_pub_ = this->create_publisher<tmr_msgs::msg::EncoderTicks>(
        "encoders", 10);
    
    connected_pub_ = this->create_publisher<std_msgs::msg:: Bool>(
        "connected", 10);
    
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);

    // Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&VexSerialNode::cmdVelCallback, this, std::placeholders::_1));

    // Services
    emergency_stop_srv_ = this->create_service<std_srvs::srv:: Trigger>(
        "emergency_stop",
        std::bind(&VexSerialNode::emergencyStopCallback, this,
                  std::placeholders:: _1, std::placeholders:: _2));
    
    reset_encoders_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "reset_encoders",
        std::bind(&VexSerialNode::resetEncodersCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // TF broadcaster
    if (publish_tf_) {
        tf_broadcaster_ = std:: make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // Timers
    serial_timer_ = this->create_wall_timer(
        10ms, std::bind(&VexSerialNode::serialTimerCallback, this));
    
    odom_timer_ = this->create_wall_timer(
        20ms, std::bind(&VexSerialNode::odometryTimerCallback, this));
    
    heartbeat_timer_ = this->create_wall_timer(
        200ms, std::bind(&VexSerialNode::heartbeatTimerCallback, this));
    
    status_timer_ = this->create_wall_timer(
        500ms, std::bind(&VexSerialNode::statusTimerCallback, this));
    
    watchdog_timer_ = this->create_wall_timer(
        100ms, std::bind(&VexSerialNode::watchdogTimerCallback, this));
}

bool VexSerialNode::initializeSerial()
{
    std::lock_guard<std::mutex> lock(serial_mutex_);

    if (serial_->open(serial_port_, baud_rate_)) {
        connected_ = true;
        RCLCPP_INFO(get_logger(), "Connected to VEX on %s", serial_port_.c_str());
        
        // Reset encoders on connect
        serial_->resetEncoders();
        first_encoder_reading_ = true;
        
        return true;
    }

    connected_ = false;
    return false;
}

void VexSerialNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Clamp velocities to limits
    cmd_linear_ = std::clamp(static_cast<float>(msg->linear.x),
                             static_cast<float>(-max_linear_velocity_),
                             static_cast<float>(max_linear_velocity_));
    
    cmd_angular_ = std:: clamp(static_cast<float>(msg->angular.z),
                              static_cast<float>(-max_angular_velocity_),
                              static_cast<float>(max_angular_velocity_));

    last_cmd_vel_time_ = this->now();
    cmd_vel_active_ = true;

    // Send command immediately
    if (connected_ && ! emergency_stop_active_) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        VelocityCommand cmd{cmd_linear_, cmd_angular_};
        serial_->sendVelocityCommand(cmd);
    }
}

void VexSerialNode::serialTimerCallback()
{
    if (!connected_) {
        // Try to reconnect
        initializeSerial();
        return;
    }

    std::lock_guard<std::mutex> lock(serial_mutex_);

    // Process incoming data
    int packets = serial_->processIncoming();
    
    if (packets > 0) {
        // Check for encoder data
        auto encoder_data = serial_->getEncoderData();
        if (encoder_data.has_value()) {
            calculateOdometry(encoder_data.value());
            
            // Publish encoder ticks
            auto enc_msg = tmr_msgs::msg::EncoderTicks();
            enc_msg.header.stamp = this->now();
            enc_msg.left_front = encoder_data->left_front_ticks;
            enc_msg.right_front = encoder_data->right_front_ticks;
            enc_msg.left_rear = encoder_data->left_rear_ticks;
            enc_msg.right_rear = encoder_data->right_rear_ticks;
            encoder_pub_->publish(enc_msg);
        }

        // Check for status data
        auto status_data = serial_->getStatusData();
        if (status_data.has_value()) {
            battery_voltage_ = status_data->battery_voltage;
            battery_percent_ = status_data->battery_percent;
            emergency_stop_active_ = status_data->emergency_stop;
        }
    }

    // Request encoder data periodically
    serial_->requestEncoders();
}

void VexSerialNode::odometryTimerCallback()
{
    publishOdometry();
}

void VexSerialNode::heartbeatTimerCallback()
{
    if (connected_) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_->sendHeartbeat();
    }
}

void VexSerialNode::statusTimerCallback()
{
    // Publish connection status
    auto connected_msg = std_msgs::msg::Bool();
    connected_msg.data = connected_;
    connected_pub_->publish(connected_msg);

    // Publish VEX status
    auto status_msg = tmr_msgs::msg::VexStatus();
    status_msg.header.stamp = this->now();
    status_msg.connected = connected_;
    status_msg.battery_voltage = battery_voltage_;
    status_msg.battery_percent = battery_percent_;
    status_msg.emergency_stop = emergency_stop_active_;
    status_msg.motors_enabled = ! emergency_stop_active_ && cmd_vel_active_;
    status_pub_->publish(status_msg);

    // Request status from VEX
    if (connected_) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_->requestStatus();
    }
}

void VexSerialNode::watchdogTimerCallback()
{
    // Check for cmd_vel timeout
    if (cmd_vel_active_) {
        double elapsed = (this->now() - last_cmd_vel_time_).seconds();
        if (elapsed > cmd_vel_timeout_) {
            // Timeout - stop motors
            cmd_vel_active_ = false;
            cmd_linear_ = 0.0f;
            cmd_angular_ = 0.0f;

            if (connected_) {
                std::lock_guard<std::mutex> lock(serial_mutex_);
                VelocityCommand stop_cmd{0.0f, 0.0f};
                serial_->sendVelocityCommand(stop_cmd);
            }

            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                 "cmd_vel timeout - stopping motors");
        }
    }
}

void VexSerialNode::emergencyStopCallback(
    const std::shared_ptr<std_srvs::srv:: Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger:: Response> response)
{
    RCLCPP_WARN(get_logger(), "EMERGENCY STOP triggered!");

    emergency_stop_active_ = true;
    cmd_linear_ = 0.0f;
    cmd_angular_ = 0.0f;

    if (connected_) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_->sendEmergencyStop();
    }

    response->success = true;
    response->message = "Emergency stop activated";
}

void VexSerialNode::resetEncodersCallback(
    const std::shared_ptr<std_srvs:: srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(get_logger(), "Resetting encoders and odometry");

    // Reset odometry
    odom_x_ = 0.0;
    odom_y_ = 0.0;
    odom_theta_ = 0.0;
    first_encoder_reading_ = true;

    if (connected_) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_->resetEncoders();
    }

    response->success = true;
    response->message = "Encoders and odometry reset";
}

void VexSerialNode::calculateOdometry(const EncoderData& encoder_data)
{
    // Use average of front and rear for each side (4-wheel drive)
    int32_t left_ticks = (encoder_data.left_front_ticks + encoder_data.left_rear_ticks) / 2;
    int32_t right_ticks = (encoder_data.right_front_ticks + encoder_data.right_rear_ticks) / 2;

    if (first_encoder_reading_) {
        prev_left_ticks_ = left_ticks;
        prev_right_ticks_ = right_ticks;
        last_encoder_time_ = this->now();
        first_encoder_reading_ = false;
        return;
    }

    // Calculate delta ticks
    int32_t delta_left = left_ticks - prev_left_ticks_;
    int32_t delta_right = right_ticks - prev_right_ticks_;

    prev_left_ticks_ = left_ticks;
    prev_right_ticks_ = right_ticks;

    // Calculate time delta
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_encoder_time_).seconds();
    last_encoder_time_ = current_time;

    if (dt <= 0.0 || dt > 1.0) {
        return;  // Invalid time delta
    }

    // Convert ticks to distance
    double meters_per_tick = (2. 0 * M_PI * wheel_radius_m_) / ticks_per_revolution_;
    double left_distance = delta_left * meters_per_tick;
    double right_distance = delta_right * meters_per_tick;

    // Calculate robot motion
    double linear_distance = (left_distance + right_distance) / 2.0;
    double angular_distance = (right_distance - left_distance) / track_width_m_;

    // Calculate velocities
    odom_linear_vel_ = linear_distance / dt;
    odom_angular_vel_ = angular_distance / dt;

    // Update pose using midpoint integration
    double delta_theta = angular_distance;
    double delta_x, delta_y;

    if (std::abs(delta_theta) < 1e-6) {
        // Straight line motion
        delta_x = linear_distance * std::cos(odom_theta_);
        delta_y = linear_distance * std::sin(odom_theta_);
    } else {
        // Arc motion
        double radius = linear_distance / delta_theta;
        delta_x = radius * (std::sin(odom_theta_ + delta_theta) - std::sin(odom_theta_));
        delta_y = radius * (std::cos(odom_theta_) - std::cos(odom_theta_ + delta_theta));
    }

    odom_x_ += delta_x;
    odom_y_ += delta_y;
    odom_theta_ += delta_theta;

    // Normalize theta to [-pi, pi]
    while (odom_theta_ > M_PI) odom_theta_ -= 2.0 * M_PI;
    while (odom_theta_ < -M_PI) odom_theta_ += 2.0 * M_PI;
}

void VexSerialNode::publishOdometry()
{
    auto current_time = this->now();

    // Create odometry message
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    // Position
    odom_msg.pose.pose.position.x = odom_x_;
    odom_msg.pose.pose.position.y = odom_y_;
    odom_msg.pose.pose.position.z = 0.0;

    // Orientation (quaternion from yaw)
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = std::sin(odom_theta_ / 2.0);
    odom_msg.pose.pose.orientation.w = std::cos(odom_theta_ / 2.0);

    // Velocity
    odom_msg. twist.twist.linear.x = odom_linear_vel_;
    odom_msg.twist. twist.linear.y = 0.0;
    odom_msg.twist.twist.linear. z = 0.0;
    odom_msg.twist. twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = odom_angular_vel_;

    // Covariance (diagonal)
    // Pose covariance
    odom_msg.pose.covariance[0] = 0.01;   // x
    odom_msg.pose.covariance[7] = 0.01;   // y
    odom_msg.pose.covariance[14] = 1e6;   // z (unused)
    odom_msg.pose. covariance[21] = 1e6;   // roll (unused)
    odom_msg.pose.covariance[28] = 1e6;   // pitch (unused)
    odom_msg.pose.covariance[35] = 0.03;  // yaw

    // Twist covariance
    odom_msg.twist. covariance[0] = 0.01;  // vx
    odom_msg. twist.covariance[7] = 1e6;   // vy (unused)
    odom_msg.twist.covariance[14] = 1e6;  // vz (unused)
    odom_msg.twist.covariance[21] = 1e6;  // vroll (unused)
    odom_msg.twist.covariance[28] = 1e6;  // vpitch (unused)
    odom_msg.twist.covariance[35] = 0.03; // vyaw

    odom_pub_->publish(odom_msg);

    // Publish TF if enabled
    if (publish_tf_ && tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped transform;
        transform. header.stamp = current_time;
        transform.header.frame_id = odom_frame_;
        transform.child_frame_id = base_frame_;
        transform.transform.translation.x = odom_x_;
        transform. transform.translation.y = odom_y_;
        transform.transform.translation.z = 0.0;
        transform.transform. rotation = odom_msg.pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform);
    }

    // Publish joint states for wheel visualization
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = current_time;
    joint_state_msg.name = {
        "left_front_wheel_joint",
        "right_front_wheel_joint",
        "left_rear_wheel_joint",
        "right_rear_wheel_joint"
    };
    
    // Calculate wheel positions from odometry (simplified)
    double wheel_rotation = odom_x_ / wheel_radius_m_;
    joint_state_msg.position = {wheel_rotation, wheel_rotation, wheel_rotation, wheel_rotation};
    joint_state_msg.velocity = {
        odom_linear_vel_ / wheel_radius_m_,
        odom_linear_vel_ / wheel_radius_m_,
        odom_linear_vel_ / wheel_radius_m_,
        odom_linear_vel_ / wheel_radius_m_
    };

    joint_state_pub_->publish(joint_state_msg);
}

void VexSerialNode::velocityToWheels(float linear, float angular,
                                      float& left_vel, float& right_vel)
{
    // Differential drive kinematics
    left_vel = linear - (angular * track_width_m_ / 2.0f);
    right_vel = linear + (angular * track_width_m_ / 2.0f);
}

}  // namespace tmr_vex_serial
