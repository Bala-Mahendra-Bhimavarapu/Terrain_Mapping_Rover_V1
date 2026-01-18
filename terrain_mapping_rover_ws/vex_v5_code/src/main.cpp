/**
 * @file main.cpp
 * @brief Main entry point for VEX V5 robot program
 * 
 * This program: 
 * - Reads velocity commands from Pi over USB serial
 * - Controls motors with proper direction compensation
 * - Streams odometry back to Pi at 50 Hz
 * - Handles connection errors gracefully
 * 
 * Compatible with PROS 3.x
 */

#include "main.h"
#include "config.hpp"
#include "serial_protocol.hpp"
#include "motor_controller.hpp"
#include "odometry.hpp"

#include <string>
#include <cstring>

// Global instances
motors::MotorController motor_ctrl;
odom:: OdometryTracker odom_tracker;

// Serial communication state
bool serial_connected = false;
uint32_t last_cmd_time_ms = 0;
constexpr uint32_t CMD_TIMEOUT_MS = 500;  // Stop if no commands for 500ms

// Read buffer for serial
std::string serial_buffer;
constexpr size_t MAX_BUFFER_SIZE = 256;

/**
 * @brief Process incoming serial data
 */
void process_serial() {
    // Read available data from stdin (USB serial)
    while (true) {
        int c = fgetc(stdin);
        if (c == EOF) {
            break;
        }
        
        char ch = static_cast<char>(c);
        
        if (ch == '\n') {
            // Complete message received
            if (! serial_buffer.empty()) {
                serial:: CommandMessage cmd = serial::parse_command(serial_buffer);
                
                if (cmd. valid) {
                    // Apply velocity command
                    motor_ctrl.set_velocities(
                        cmd.left_velocity_ms,
                        cmd.right_velocity_ms
                    );
                    last_cmd_time_ms = pros::millis();
                    serial_connected = true;
                }
                
                serial_buffer.clear();
            }
        } else if (serial_buffer.size() < MAX_BUFFER_SIZE) {
            serial_buffer += ch;
        } else {
            // Buffer overflow, clear it
            serial_buffer. clear();
        }
    }
}

/**
 * @brief Send odometry to Pi
 */
void send_odometry() {
    uint32_t timestamp_ms = pros::millis();
    
    // Get encoder values (already direction-corrected by motor controller)
    int64_t left_ticks = motor_ctrl.get_left_ticks();
    int64_t right_ticks = motor_ctrl. get_right_ticks();
    double left_vel = motor_ctrl.get_left_velocity();
    double right_vel = motor_ctrl.get_right_velocity();
    
    // Update odometry tracker
    odom_tracker.update(left_ticks, right_ticks, timestamp_ms);
    
    // Format and send message
    std::string msg = serial::format_odom_message(
        left_ticks, right_ticks,
        left_vel, right_vel,
        timestamp_ms
    );
    
    printf("%s", msg.c_str());
    fflush(stdout);
}

/**
 * @brief Send status to Pi
 */
void send_status() {
    double temps[4];
    motor_ctrl.get_temperatures(temps);
    
    // Get battery voltage (V5 battery is nominally 12. 8V)
    double battery_voltage = pros::battery:: get_voltage() / 1000.0;
    
    std::string msg = serial::format_status_message(battery_voltage, temps);
    printf("%s", msg.c_str());
    fflush(stdout);
}

/**
 * @brief Check for command timeout and stop motors if needed
 */
void check_timeout() {
    uint32_t current_time = pros::millis();
    
    if (serial_connected && (current_time - last_cmd_time_ms) > CMD_TIMEOUT_MS) {
        // Timeout - stop motors for safety
        motor_ctrl.stop();
        serial_connected = false;
    }
}

/**
 * @brief PROS initialization function
 * Called once when the program starts
 */
void initialize() {
    // Initialize motor controller
    motor_ctrl.initialize();
    
    // Reset odometry
    odom_tracker.reset();
    
    // Clear serial buffer
    serial_buffer. clear();
    serial_buffer.reserve(MAX_BUFFER_SIZE);
    
    // Display startup message on V5 brain screen
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Lunar Rover V5");
    pros::lcd::set_text(2, "Waiting for Pi...");
}

/**
 * @brief PROS disabled function
 * Called when the robot is disabled
 */
void disabled() {
    motor_ctrl.stop();
}

/**
 * @brief PROS competition initialize function
 */
void competition_initialize() {
    // Nothing special needed
}

/**
 * @brief PROS autonomous function
 * Not used - Pi handles all control
 */
void autonomous() {
    // Pi controls everything via serial
    // This function runs the same loop as opcontrol
    while (true) {
        process_serial();
        check_timeout();
        send_odometry();
        pros::delay(config::ODOM_UPDATE_RATE_MS);
    }
}

/**
 * @brief PROS operator control function
 * Main control loop
 */
void opcontrol() {
    uint32_t last_odom_time = 0;
    uint32_t last_status_time = 0;
    
    while (true) {
        uint32_t current_time = pros::millis();
        
        // Process incoming serial commands
        process_serial();
        
        // Check for command timeout
        check_timeout();
        
        // Send odometry at configured rate (50 Hz)
        if ((current_time - last_odom_time) >= config::ODOM_UPDATE_RATE_MS) {
            send_odometry();
            last_odom_time = current_time;
        }
        
        // Send status at configured rate (1 Hz)
        if ((current_time - last_status_time) >= config::STATUS_UPDATE_RATE_MS) {
            send_status();
            last_status_time = current_time;
            
            // Update LCD display
            const odom::Pose& pose = odom_tracker.get_pose();
            char buf[32];
            snprintf(buf, sizeof(buf), "X: %.2f Y:%.2f", pose.x, pose.y);
            pros::lcd::set_text(3, buf);
            snprintf(buf, sizeof(buf), "Theta:%.1f deg", pose.theta * 180.0 / M_PI);
            pros::lcd:: set_text(4, buf);
            
            if (serial_connected) {
                pros::lcd::set_text(2, "Pi Connected");
            } else {
                pros::lcd::set_text(2, "Waiting for Pi...");
            }
        }
        
        // Small delay to prevent CPU hogging
        pros::delay(5);
    }
}
