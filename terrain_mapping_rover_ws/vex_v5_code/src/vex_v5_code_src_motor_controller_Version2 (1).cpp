/**
 * @file motor_controller. cpp
 * @brief Motor controller implementation
 */

#include "motor_controller.hpp"
#include <cmath>

namespace motors {

void MotorController::initialize() {
    // Create motor objects with correct gearing
    // 18:1 cartridge = pros::E_MOTOR_GEARSET_18
    
    front_left_ = new pros::Motor(
        config::FRONT_LEFT_MOTOR_PORT,
        pros::E_MOTOR_GEARSET_18,
        config::REVERSE_LEFT_MOTORS,
        pros::E_MOTOR_ENCODER_COUNTS
    );
    
    front_right_ = new pros::Motor(
        config::FRONT_RIGHT_MOTOR_PORT,
        pros::E_MOTOR_GEARSET_18,
        config::REVERSE_RIGHT_MOTORS,  // Reversed for mirrored mounting
        pros::E_MOTOR_ENCODER_COUNTS
    );
    
    back_left_ = new pros:: Motor(
        config::BACK_LEFT_MOTOR_PORT,
        pros::E_MOTOR_GEARSET_18,
        config::REVERSE_LEFT_MOTORS,
        pros:: E_MOTOR_ENCODER_COUNTS
    );
    
    back_right_ = new pros::Motor(
        config:: BACK_RIGHT_MOTOR_PORT,
        pros::E_MOTOR_GEARSET_18,
        config::REVERSE_RIGHT_MOTORS,  // Reversed for mirrored mounting
        pros::E_MOTOR_ENCODER_COUNTS
    );
    
    // Set brake mode to coast for smoother control
    front_left_->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    front_right_->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_left_->set_brake_mode(pros:: E_MOTOR_BRAKE_COAST);
    back_right_->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    
    // Reset encoder positions
    front_left_->tare_position();
    front_right_->tare_position();
    back_left_->tare_position();
    back_right_->tare_position();
    
    initialized_ = true;
}

void MotorController::set_velocities(double left_velocity_ms, double right_velocity_ms) {
    if (!initialized_) return;
    
    // Clamp velocities
    left_velocity_ms = std::max(-config:: MAX_WHEEL_VELOCITY_MS,
                               std::min(config::MAX_WHEEL_VELOCITY_MS, left_velocity_ms));
    right_velocity_ms = std::max(-config::MAX_WHEEL_VELOCITY_MS,
                                std::min(config::MAX_WHEEL_VELOCITY_MS, right_velocity_ms));
    
    // Convert to voltage (simple proportional control)
    double left_voltage = velocity_to_voltage(left_velocity_ms);
    double right_voltage = velocity_to_voltage(right_velocity_ms);
    
    // Apply to motors
    // Note: Motor direction is already handled by the reversed flag in initialization
    front_left_->move_voltage(static_cast<int>(left_voltage));
    back_left_->move_voltage(static_cast<int>(left_voltage));
    front_right_->move_voltage(static_cast<int>(right_voltage));
    back_right_->move_voltage(static_cast<int>(right_voltage));
}

void MotorController::stop() {
    if (!initialized_) return;
    
    front_left_->move_voltage(0);
    front_right_->move_voltage(0);
    back_left_->move_voltage(0);
    back_right_->move_voltage(0);
}

int64_t MotorController:: get_left_ticks() const {
    if (!initialized_) return 0;
    
    // Average front and back encoders for better accuracy
    double front = front_left_->get_position();
    double back = back_left_->get_position();
    return static_cast<int64_t>((front + back) / 2. 0);
}

int64_t MotorController::get_right_ticks() const {
    if (!initialized_) return 0;
    
    // Average front and back encoders
    double front = front_right_->get_position();
    double back = back_right_->get_position();
    return static_cast<int64_t>((front + back) / 2.0);
}

double MotorController::get_left_velocity() const {
    if (!initialized_) return 0.0;
    
    // Get actual velocity in RPM, convert to ticks/sec
    double rpm = (front_left_->get_actual_velocity() + 
                  back_left_->get_actual_velocity()) / 2.0;
    // RPM to ticks/sec:  rpm * ticks_per_rev / 60
    return rpm * config::TICKS_PER_REVOLUTION / 60.0;
}

double MotorController::get_right_velocity() const {
    if (!initialized_) return 0.0;
    
    double rpm = (front_right_->get_actual_velocity() + 
                  back_right_->get_actual_velocity()) / 2.0;
    return rpm * config::TICKS_PER_REVOLUTION / 60.0;
}

void MotorController::get_temperatures(double temps[4]) const {
    if (!initialized_) {
        temps[0] = temps[1] = temps[2] = temps[3] = 0.0;
        return;
    }
    
    temps[0] = front_left_->get_temperature();
    temps[1] = front_right_->get_temperature();
    temps[2] = back_left_->get_temperature();
    temps[3] = back_right_->get_temperature();
}

double MotorController::velocity_to_rpm(double velocity_ms) const {
    // velocity (m/s) -> wheel RPM
    // v = omega * r = (rpm * 2 * pi / 60) * r
    // rpm = v * 60 / (2 * pi * r) = v * 60 / circumference
    return velocity_ms * 60.0 / config::WHEEL_CIRCUMFERENCE_M;
}

double MotorController::velocity_to_voltage(double velocity_ms) const {
    // Simple linear mapping from velocity to voltage
    // At max velocity, use max voltage
    double ratio = velocity_ms / config::MAX_WHEEL_VELOCITY_MS;
    return ratio * config::MAX_MOTOR_VOLTAGE;
}

}  // namespace motors