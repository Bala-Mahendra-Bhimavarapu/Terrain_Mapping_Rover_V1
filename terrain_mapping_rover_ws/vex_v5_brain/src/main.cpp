/**
 * @file main.cpp
 * @brief Main entry point for TMR VEX V5 Brain
 * 
 * This program runs on the VEX V5 Brain and handles:  
 * - Serial communication with Raspberry Pi
 * - Motor control with proper direction handling
 * - Encoder reading with automatic sign correction
 * - Safety monitoring (battery, temperature, timeouts)
 * - Emergency stop functionality
 */

#include "main.h"

using namespace tmr;

// =============================================================================
// GLOBAL OBJECTS
// =============================================================================

static MotorController g_motors;
static SerialProtocol g_serial;
static Odometry g_odometry;
static SafetyMonitor* g_safety = nullptr;

// Timing
static uint32_t g_last_encoder_send_time = 0;
static uint32_t g_last_status_send_time = 0;
static uint32_t g_last_lcd_update_time = 0;

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

/**
 * @brief Update LCD display with status information
 */
void updateLCD(uint32_t current_time) {
    if (current_time - g_last_lcd_update_time < 250) {
        return;  // Update at 4 Hz
    }
    g_last_lcd_update_time = current_time;
    
    const auto& safety_status = g_safety->getStatus();
    
    // Line 0: Title and connection status
    pros::lcd::print(0, "TMR VEX v1.0 %s", 
                     safety_status.heartbeat_timeout ?  "[NO CONN]" : "[OK]");
    
    // Line 1: Battery
    pros::lcd::print(1, "Batt: %. 1fV (%.0f%%) %s",
                     g_safety->getBatteryVoltage(),
                     g_safety->getBatteryPercent(),
                     safety_status.battery_low ? "LOW!" : "");
    
    // Line 2:  Commanded velocity
    pros::lcd::print(2, "Cmd: L=%.2f A=%.2f",
                     g_motors.getCommandedLinear(),
                     g_motors. getCommandedAngular());
    
    // Line 3: Actual velocity
    float left_vel, right_vel;
    g_motors.getActualWheelVelocities(left_vel, right_vel);
    pros::lcd::print(3, "Act: L=%.2f R=%.2f m/s",
                     left_vel, right_vel);
    
    // Line 4: Encoder ticks
    int32_t left_ticks, right_ticks;
    g_motors.getEncoderTicksAveraged(left_ticks, right_ticks);
    pros::lcd::print(4, "Enc: L=%d R=%d",
                     left_ticks, right_ticks);
    
    // Line 5: Odometry
    pros::lcd::print(5, "Pos: X=%.2f Y=%.2f T=%.1f",
                     g_odometry.getX(),
                     g_odometry. getY(),
                     g_odometry.getTheta() * 180.0f / M_PI);
    
    // Line 6: Safety status
    if (safety_status.emergency_stop_active) {
        pros::lcd::print(6, "*** EMERGENCY STOP ***");
    } else if (safety_status. motor_fault) {
        pros::lcd::print(6, "MOTOR FAULT!");
    } else if (safety_status.motor_overtemp) {
        pros::lcd::print(6, "MOTOR OVERTEMP!");
    } else if (safety_status.command_timeout) {
        pros::lcd::print(6, "Command timeout");
    } else {
        pros::lcd:: print(6, "Status: OK");
    }
    
    // Line 7: Serial stats
    pros::lcd:: print(7, "Rx:%lu Tx:%lu Err:%lu",
                     g_serial.getBytesReceived(),
                     g_serial.getBytesSent(),
                     g_serial.getPacketErrors());
}

/**
 * @brief Send encoder data to Pi
 */
void sendEncoderData(uint32_t current_time) {
    if (current_time - g_last_encoder_send_time < config::ENCODER_SEND_INTERVAL_MS) {
        return;
    }
    g_last_encoder_send_time = current_time;
    
    EncoderPacket packet;
    g_motors.getEncoderTicks(
        packet.left_front_ticks,
        packet.right_front_ticks,
        packet.left_rear_ticks,
        packet.right_rear_ticks
    );
    packet.timestamp_ms = current_time;
    
    g_serial.sendEncoderData(packet);
}

/**
 * @brief Send status data to Pi
 */
void sendStatusData(uint32_t current_time) {
    if (current_time - g_last_status_send_time < config:: STATUS_SEND_INTERVAL_MS) {
        return;
    }
    g_last_status_send_time = current_time;
    
    const auto& safety_status = g_safety->getStatus();
    
    StatusPacket packet;
    packet. battery_voltage = g_safety->getBatteryVoltage();
    packet.battery_percent = g_safety->getBatteryPercent();
    packet.motors_enabled = g_motors.isEnabled() ? 1 : 0;
    packet.emergency_stop = safety_status.emergency_stop_active ? 1 : 0;
    packet.error_flags = g_safety->getErrorFlags();
    packet.uptime_ms = current_time;
    
    g_serial. sendStatus(packet);
}

/**
 * @brief Process serial commands from Pi
 */
void processSerialCommands(uint32_t current_time) {
    // Process incoming serial data
    g_serial.processIncoming();
    
    // Handle velocity commands
    if (g_serial.hasVelocityCommand()) {
        VelocityCommand cmd = g_serial.getVelocityCommand();
        
        // Only apply if operation is allowed
        if (g_safety->isOperationAllowed()) {
            g_motors.setVelocity(cmd.linear_ms, cmd.angular_rads);
        }
        
        g_safety->recordCommand(current_time);
    }
    
    // Handle heartbeat
    if (g_serial.isHeartbeatReceived()) {
        g_serial.clearHeartbeatReceived();
        g_safety->recordHeartbeat(current_time);
    }
    
    // Handle emergency stop request
    if (g_serial. isEmergencyStopRequested()) {
        g_serial.clearEmergencyStopRequest();
        g_safety->triggerEmergencyStop();
    }
    
    // Handle clear e-stop request
    if (g_serial. isClearEstopRequested()) {
        g_serial.clearClearEstopRequest();
        g_safety->clearEmergencyStop();
    }
    
    // Handle encoder reset request
    if (g_serial. isEncoderResetRequested()) {
        g_serial. clearEncoderResetRequest();
        g_motors.resetEncoders();
        g_odometry.reset();
    }
    
    // Handle status request
    if (g_serial. isStatusRequested()) {
        g_serial.clearStatusRequest();
        sendStatusData(current_time);
    }
    
    // Handle encoder data request
    if (g_serial. isEncodersRequested()) {
        g_serial.clearEncodersRequest();
        sendEncoderData(current_time);
    }
}

/**
 * @brief Update local odometry (optional - can be done on Pi side)
 */
void updateOdometry(uint32_t current_time) {
    int32_t left_ticks, right_ticks;
    g_motors.getEncoderTicksAveraged(left_ticks, right_ticks);
    g_odometry.update(left_ticks, right_ticks, current_time);
}

// =============================================================================
// PROS ENTRY POINTS
// =============================================================================

/**
 * @brief Initialize the robot
 * 
 * Called once when the program starts. 
 */
void initialize() {
    // Initialize LCD
    pros::lcd::initialize();
    pros::lcd::print(0, "TMR VEX Initializing...");
    
    // Initialize motors
    pros::lcd::print(1, "Init motors...");
    g_motors.initialize();
    
    // Initialize serial communication
    pros::lcd::print(2, "Init serial...");
    g_serial.initialize();
    
    // Initialize safety monitor
    pros::lcd::print(3, "Init safety...");
    g_safety = new SafetyMonitor(g_motors);
    
    // Initialize odometry
    g_odometry.reset();
    
    // Clear timers
    g_last_encoder_send_time = 0;
    g_last_status_send_time = 0;
    g_last_lcd_update_time = 0;
    
    pros::lcd::print(4, "Initialization complete!");
    pros::delay(500);
    
    // Clear LCD for runtime display
    pros::lcd::clear();
}

/**
 * @brief Called when the robot is disabled
 */
void disabled() {
    // Stop all motors when disabled
    g_motors.stop();
    g_motors.setEnabled(false);
    
    pros::lcd::print(6, "*** DISABLED ***");
}

/**
 * @brief Competition initialization
 */
void competition_initialize() {
    // Nothing special needed for competition
}

/**
 * @brief Autonomous mode
 * 
 * Not used for this application - all control comes from Pi.
 */
void autonomous() {
    // This robot is controlled by the Pi, not autonomous routines
    // Just keep processing serial commands
    
    while (true) {
        uint32_t current_time = pros::millis();
        
        processSerialCommands(current_time);
        g_safety->update(current_time);
        updateOdometry(current_time);
        sendEncoderData(current_time);
        sendStatusData(current_time);
        updateLCD(current_time);
        
        pros::delay(config::MAIN_LOOP_DELAY_MS);
    }
}

/**
 * @brief Operator control mode
 * 
 * Main control loop - processes commands from Pi.
 */
void opcontrol() {
    // Enable motors
    g_motors.setEnabled(true);
    
    // Record initial heartbeat time
    g_safety->recordHeartbeat(pros::millis());
    g_safety->recordCommand(pros:: millis());
    
    // Main loop
    while (true) {
        uint32_t current_time = pros::millis();
        
        // Process serial commands from Pi
        processSerialCommands(current_time);
        
        // Update safety monitoring
        g_safety->update(current_time);
        
        // Update local odometry
        updateOdometry(current_time);
        
        // Send periodic encoder data
        sendEncoderData(current_time);
        
        // Send periodic status data
        sendStatusData(current_time);
        
        // Update LCD display
        updateLCD(current_time);
        
        // Small delay to prevent CPU hogging
        pros::delay(config::MAIN_LOOP_DELAY_MS);
    }
}
