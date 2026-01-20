/**
 * @file drive_system.cpp
 * @brief Tank drive implementation
 */

#include "drive_system.hpp"
#include <cmath>

DriveSystem::DriveSystem() {}

void DriveSystem::initialize() {
    // Create individual motor objects
    front_left_ = std::make_unique<pros::Motor>(
        FRONT_LEFT_MOTOR_PORT, 
        MOTOR_GEARSET, 
        LEFT_MOTORS_REVERSED
    );
    
    front_right_ = std::make_unique<pros::Motor>(
        FRONT_RIGHT_MOTOR_PORT, 
        MOTOR_GEARSET, 
        RIGHT_MOTORS_REVERSED  // REVERSED to fix direction
    );
    
    back_left_ = std::make_unique<pros::Motor>(
        BACK_LEFT_MOTOR_PORT, 
        MOTOR_GEARSET, 
        LEFT_MOTORS_REVERSED
    );
    
    back_right_ = std::make_unique<pros::Motor>(
        BACK_RIGHT_MOTOR_PORT, 
        MOTOR_GEARSET, 
        RIGHT_MOTORS_REVERSED  // REVERSED to fix direction
    );
    
    // Set brake mode to COAST for safety
    front_left_->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    front_right_->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_left_->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_right_->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    
    // Create motor groups
    left_motors_ = std::make_unique<pros:: Motor_Group>(
        std::initializer_list<pros::Motor>{*front_left_, *back_left_}
    );
    
    right_motors_ = std:: make_unique<pros::Motor_Group>(
        std::initializer_list<pros::Motor>{*front_right_, *back_right_}
    );
    
    // Reset encoders
    reset_encoders();
}

void DriveSystem::set_velocity(double left_ms, double right_ms) {
    // Clamp velocities
    left_ms = std::max(-MAX_LINEAR_VELOCITY_MS, std::min(MAX_LINEAR_VELOCITY_MS, left_ms));
    right_ms = std::max(-MAX_LINEAR_VELOCITY_MS, std::min(MAX_LINEAR_VELOCITY_MS, right_ms));
    
    // Convert to RPM
    double left_rpm = velocity_to_rpm(left_ms);
    double right_rpm = velocity_to_rpm(right_ms);
    
    // Set motor velocities
    // Note: Motor reversal is already handled in motor initialization
    // So we send the same sign for both sides
    left_motors_->move_velocity(static_cast<int32_t>(left_rpm));
    right_motors_->move_velocity(static_cast<int32_t>(right_rpm));
}

void DriveSystem:: stop() {
    // Immediate stop with brake
    front_left_->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    front_right_->set_brake_mode(pros:: E_MOTOR_BRAKE_BRAKE);
    back_left_->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    back_right_->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    
    left_motors_->move_velocity(0);
    right_motors_->move_velocity(0);
    
    // Reset to coast after stopping
    pros::delay(100);
    front_left_->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    front_right_->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_left_->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_right_->set_brake_mode(pros:: E_MOTOR_BRAKE_COAST);
}

int32_t DriveSystem::get_left_ticks() const {
    return get_raw_left_ticks() - left_ticks_offset_;
}

int32_t DriveSystem::get_right_ticks() const {
    // Right motors are reversed, so encoder reading is already correct
    // (PROS handles the sign when motor is set as reversed)
    return get_raw_right_ticks() - right_ticks_offset_;
}

double DriveSystem::get_left_velocity() const {
    // Get average velocity of left motors in RPM, convert to m/s
    double avg_rpm = (front_left_->get_actual_velocity() + 
                      back_left_->get_actual_velocity()) / 2.0;
    return (avg_rpm / 60.0) * WHEEL_CIRCUMFERENCE_M;
}

double DriveSystem::get_right_velocity() const {
    // Right motors are reversed, but PROS returns correct sign
    double avg_rpm = (front_right_->get_actual_velocity() + 
                      back_right_->get_actual_velocity()) / 2.0;
    return (avg_rpm / 60.0) * WHEEL_CIRCUMFERENCE_M;
}

void DriveSystem::reset_encoders() {
    front_left_->tare_position();
    front_right_->tare_position();
    back_left_->tare_position();
    back_right_->tare_position();
    
    left_ticks_offset_ = 0;
    right_ticks_offset_ = 0;
}

bool DriveSystem::is_healthy() const {
    // Check if all motors are responding
    return ! front_left_->is_over_temp() &&
           !front_right_->is_over_temp() &&
           !back_left_->is_over_temp() &&
           !back_right_->is_over_temp();
}

double DriveSystem::velocity_to_rpm(double velocity_ms) const {
    // RPM = (velocity / circumference) * 60
    return (velocity_ms / WHEEL_CIRCUMFERENCE_M) * 60.0;
}

int32_t DriveSystem:: get_raw_left_ticks() const {
    // Average of front and back left encoder positions
    return static_cast<int32_t>(
        (front_left_->get_position() + back_left_->get_position()) / 2.0
    );
}

int32_t DriveSystem::get_raw_right_ticks() const {
    // Average of front and back right encoder positions
    // Note: Position is already sign-corrected because motor is reversed
    return static_cast<int32_t>(
        (front_right_->get_position() + back_right_->get_position()) / 2.0
    );
}