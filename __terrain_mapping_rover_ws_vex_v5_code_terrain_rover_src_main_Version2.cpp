/**
 * @file main.cpp
 * @brief Main entry point for VEX V5 Terrain Mapping Rover
 * 
 * Handles: 
 * - Serial communication with Raspberry Pi
 * - Motor control based on received commands
 * - Odometry publishing
 * - Status reporting
 * - Safety monitoring
 */

#include "main.h"
#include "robot_config.hpp"
#include "serial_protocol.hpp"
#include "drive_system.hpp"

#include <string>
#include <cstring>

// Global objects
DriveSystem drive;

// State variables
volatile bool emergency_stop = false;
volatile uint32_t last_command_time = 0;
volatile int last_command_id = 0;

// Serial buffer
char serial_buffer[256];
int buffer_index = 0;

/**
 * Read and process serial input
 */
void process_serial_input() {
    // Read available characters
    while (true) {
        int c = fgetc(stdin);
        if (c == EOF) break;
        
        if (c == '\n' || c == '\r') {
            if (buffer_index > 0) {
                serial_buffer[buffer_index] = '\0';
                std::string line(serial_buffer);
                buffer_index = 0;
                
                // Process the complete line
                MessageType type = parse_message_type(line);
                
                switch (type) {
                    case MessageType::CMD:  {
                        if (! emergency_stop) {
                            MotorCommand cmd = parse_motor_command(line);
                            if (cmd.valid) {
                                drive.set_velocity(cmd.left_velocity, cmd. right_velocity);
                                last_command_time = pros::millis();
                                last_command_id = cmd.command_id;
                                
                                // Send acknowledgment
                                printf("%s", format_ack(cmd.command_id).c_str());
                            }
                        }
                        break;
                    }
                    
                    case MessageType::STOP: {
                        emergency_stop = true;
                        drive.stop();
                        printf("ACK,STOP\n");
                        break;
                    }
                    
                    case MessageType::PING: {
                        printf("PONG\n");
                        break;
                    }
                    
                    default: 
                        break;
                }
            }
        } else if (buffer_index < (int)sizeof(serial_buffer) - 1) {
            serial_buffer[buffer_index++] = (char)c;
        }
    }
}

/**
 * Publish odometry at regular intervals
 */
void odom_task_fn(void* param) {
    (void)param;
    
    while (true) {
        uint32_t timestamp = pros::millis();
        
        int32_t left_ticks = drive.get_left_ticks();
        int32_t right_ticks = drive.get_right_ticks();
        double left_vel = drive.get_left_velocity();
        double right_vel = drive.get_right_velocity();
        
        std::string msg = format_odom_message(
            timestamp, left_ticks, right_ticks, left_vel, right_vel
        );
        printf("%s", msg.c_str());
        
        pros::delay(ODOM_PUBLISH_RATE_MS);
    }
}

/**
 * Publish status at regular intervals
 */
void status_task_fn(void* param) {
    (void)param;
    
    while (true) {
        double voltage = pros::battery:: get_voltage() / 1000.0;  // mV to V
        double percent = pros::battery::get_capacity();
        
        std::string msg = format_status_message(voltage, percent);
        printf("%s", msg.c_str());
        
        // Check battery threshold
        if (voltage < MIN_BATTERY_VOLTAGE && voltage > 0) {
            emergency_stop = true;
            drive.stop();
            printf("%s", format_error(1, "Battery critical").c_str());
        }
        
        pros::delay(STATUS_PUBLISH_RATE_MS);
    }
}

/**
 * Safety watchdog - stop motors if no command received
 */
void watchdog_task_fn(void* param) {
    (void)param;
    
    while (true) {
        if (!emergency_stop) {
            uint32_t now = pros::millis();
            if (now - last_command_time > COMMAND_TIMEOUT_MS) {
                // No recent command - stop motors
                drive.set_velocity(0.0, 0.0);
            }
        }
        
        pros::delay(50);
    }
}

/**
 * PROS initialization - runs once at startup
 */
void initialize() {
    // Initialize serial communication
    // PROS uses stdin/stdout for USB serial
    
    // Initialize drive system
    drive.initialize();
    
    // Start background tasks
    pros::Task odom_task(odom_task_fn, nullptr, "odom");
    pros::Task status_task(status_task_fn, nullptr, "status");
    pros::Task watchdog_task(watchdog_task_fn, nullptr, "watchdog");
    
    printf("TMR V5 Ready\n");
}

/**
 * Runs during disabled mode
 */
void disabled() {
    drive.stop();
}

/**
 * Runs during competition connected mode
 */
void competition_initialize() {
    // Not used for this project
}

/**
 * Runs during autonomous mode
 */
void autonomous() {
    // Autonomous is handled by ROS on Pi
    // V5 just responds to serial commands
    while (true) {
        process_serial_input();
        pros::delay(1);
    }
}

/**
 * Runs during operator control mode
 */
void opcontrol() {
    // Main loop - process serial commands
    while (true) {
        process_serial_input();
        
        // Small delay to prevent CPU hogging
        pros::delay(1);
    }
}