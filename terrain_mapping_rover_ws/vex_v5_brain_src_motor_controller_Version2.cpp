/**
 * @file motor_controller.cpp
 * @brief Motor controller implementation
 */

#include "motor_controller.hpp"

namespace tmr {

MotorController::MotorController()
    : left_front_(nullptr)
    , right_front_(nullptr)
    , left_rear_(nullptr)
    , right_rear_(nullptr)
{
}

MotorController::~MotorController() {
    stop();
    
    delete left_front_;
    delete right_front_;
    delete left_rear_;
    delete right_rear_;
}

void MotorController::initialize() {
    using namespace config;
    
    // Create motor objects with direction configuration
    // When reverse=true, PROS automatically: 
    // 1. Reverses move_velocity() commands
    // 2. Reverses get_position() readings
    // 3. Reverses get_actual_velocity() readings
    
    left_front_ = new pros::Motor(
        LEFT_FRONT_MOTOR_PORT,
        MOTOR_GEARSET,
        REVERSE_LEFT_FRONT,
        pros::E_MOTOR_ENCODER_DEGREES
    );
    
    right_front_ = new pros::Motor(
        RIGHT_FRONT_MOTOR_PORT,
        MOTOR_GEARSET,
        REVERSE_RIGHT_FRONT,
        pros::E_MOTOR_ENCODER_DEGREES
    );
    
    left_rear_ = new pros::Motor(
        LEFT_REAR_MOTOR_PORT,
        MOTOR_GEARSET,
        REVERSE_LEFT_REAR,
        pros::E_MOTOR_ENCODER_DEGREES
    );
    
    right_rear_ = new pros::Motor(
        RIGHT_REAR_MOTOR_PORT,
        MOTOR_GEARSET,
        REVERSE_RIGHT_REAR,
        pros::E_MOTOR_ENCODER_DEGREES
    );
    
    // Set brake mode to brake (not coast)
    left_front_->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_front_->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    left_rear_->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_rear_->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    
    // Set current limits
    left_front_->set_current_limit(MOTOR_CURRENT_LIMIT_MA);
    right_front_->set_current_limit(MOTOR_CURRENT_LIMIT_MA);
    left_rear_->set_current_limit(MOTOR_CURRENT_LIMIT_MA);
    right_rear_->set_current_limit(MOTOR_CURRENT_LIMIT_MA);
    
    // Reset encoder positions
    left_front_->tare_position();
    right_front_->tare_position();
    left_rear_->tare_position();
    right_rear_->tare_position();
    
    // Clear offsets
    offset_lf_ = 0;
    offset_rf_ = 0;
    offset_lr_ = 0;
    offset_rr_ = 0;
}

float MotorController::velocityToRpm(float velocity_ms) {
    // Convert m/s to RPM
    // v = (RPM / 60) * 2 * PI * wheel_radius
    // RPM = v * 60 / (2 * PI * wheel_radius)
    return (velocity_ms * 60.0f) / (2.0f * M_PI * config::WHEEL_RADIUS_M);
}

void MotorController::setVelocity(float linear_ms, float angular_rads) {
    cmd_linear_ms_ = linear_ms;
    cmd_angular_rads_ = angular_rads;
    
    // Differential drive kinematics: 
    // v_left  = v_linear - (v_angular * track_width / 2)
    // v_right = v_linear + (v_angular * track_width / 2)
    
    float left_vel = linear_ms - (angular_rads * config:: TRACK_WIDTH_M / 2.0f);
    float right_vel = linear_ms + (angular_rads * config:: TRACK_WIDTH_M / 2.0f);
    
    setWheelVelocities(left_vel, right_vel);
}

void MotorController::setWheelVelocities(float left_ms, float right_ms) {
    if (! enabled_) {
        stop();
        return;
    }
    
    // Clamp to max wheel velocity
    left_ms = std::max(-config::MAX_WHEEL_VELOCITY_MS, 
                       std::min(config::MAX_WHEEL_VELOCITY_MS, left_ms));
    right_ms = std::max(-config::MAX_WHEEL_VELOCITY_MS,
                        std::min(config::MAX_WHEEL_VELOCITY_MS, right_ms));
    
    // Convert to RPM
    float left_rpm = velocityToRpm(left_ms);
    float right_rpm = velocityToRpm(right_ms);
    
    // Clamp to motor RPM limits
    left_rpm = std::max(-config::MAX_MOTOR_RPM, 
                        std::min(config::MAX_MOTOR_RPM, left_rpm));
    right_rpm = std::max(-config::MAX_MOTOR_RPM,
                         std::min(config::MAX_MOTOR_RPM, right_rpm));
    
    // Send to motors
    // Note: Because of REVERSE_ flags in motor initialization,
    // positive velocity = forward for both sides
    left_front_->move_velocity(static_cast<int32_t>(left_rpm));
    left_rear_->move_velocity(static_cast<int32_t>(left_rpm));
    right_front_->move_velocity(static_cast<int32_t>(right_rpm));
    right_rear_->move_velocity(static_cast<int32_t>(right_rpm));
}

void MotorController::stop() {
    left_front_->move_velocity(0);
    right_front_->move_velocity(0);
    left_rear_->move_velocity(0);
    right_rear_->move_velocity(0);
    
    cmd_linear_ms_ = 0. 0f;
    cmd_angular_rads_ = 0.0f;
}

void MotorController::setEnabled(bool enabled) {
    enabled_ = enabled;
    if (! enabled) {
        stop();
    }
}

void MotorController::getEncoderTicks(int32_t& lf, int32_t& rf, int32_t& lr, int32_t& rr) {
    // PROS reports position in degrees
    // Convert to ticks:  ticks = degrees * (TICKS_PER_REV / 360)
    constexpr float DEG_TO_TICKS = static_cast<float>(config:: TICKS_PER_REVOLUTION) / 360.0f;
    
    // Note: Because of REVERSE_ flags, encoder values are already
    // corrected by PROS.  Positive = forward motion.
    lf = static_cast<int32_t>(left_front_->get_position() * DEG_TO_TICKS) - offset_lf_;
    rf = static_cast<int32_t>(right_front_->get_position() * DEG_TO_TICKS) - offset_rf_;
    lr = static_cast<int32_t>(left_rear_->get_position() * DEG_TO_TICKS) - offset_lr_;
    rr = static_cast<int32_t>(right_rear_->get_position() * DEG_TO_TICKS) - offset_rr_;
}

void MotorController:: getEncoderTicksAveraged(int32_t& left, int32_t& right) {
    int32_t lf, rf, lr, rr;
    getEncoderTicks(lf, rf, lr, rr);
    
    // Average front and rear for each side
    left = (lf + lr) / 2;
    right = (rf + rr) / 2;
}

void MotorController::resetEncoders() {
    constexpr float DEG_TO_TICKS = static_cast<float>(config::TICKS_PER_REVOLUTION) / 360.0f;
    
    // Store current positions as offsets
    offset_lf_ = static_cast<int32_t>(left_front_->get_position() * DEG_TO_TICKS);
    offset_rf_ = static_cast<int32_t>(right_front_->get_position() * DEG_TO_TICKS);
    offset_lr_ = static_cast<int32_t>(left_rear_->get_position() * DEG_TO_TICKS);
    offset_rr_ = static_cast<int32_t>(right_rear_->get_position() * DEG_TO_TICKS);
}

void MotorController::getTemperatures(double& lf, double& rf, double& lr, double& rr) {
    lf = left_front_->get_temperature();
    rf = right_front_->get_temperature();
    lr = left_rear_->get_temperature();
    rr = right_rear_->get_temperature();
}

void MotorController::getCurrents(int32_t& lf, int32_t& rf, int32_t& lr, int32_t& rr) {
    lf = left_front_->get_current_draw();
    rf = right_front_->get_current_draw();
    lr = left_rear_->get_current_draw();
    rr = right_rear_->get_current_draw();
}

bool MotorController::hasMotorFault() {
    // Check for motor faults using get_faults()
    uint32_t faults = 0;
    faults |= left_front_->get_faults();
    faults |= right_front_->get_faults();
    faults |= left_rear_->get_faults();
    faults |= right_rear_->get_faults();
    
    return faults != 0;
}

void MotorController::getActualWheelVelocities(float& left_ms, float& right_ms) {
    // Get actual RPM from motors
    float lf_rpm = left_front_->get_actual_velocity();
    float rf_rpm = right_front_->get_actual_velocity();
    float lr_rpm = left_rear_->get_actual_velocity();
    float rr_rpm = right_rear_->get_actual_velocity();
    
    // Average front and rear
    float left_rpm = (lf_rpm + lr_rpm) / 2.0f;
    float right_rpm = (rf_rpm + rr_rpm) / 2.0f;
    
    // Convert RPM to m/s
    // v = (RPM / 60) * 2 * PI * wheel_radius
    left_ms = (left_rpm / 60.0f) * 2.0f * M_PI * config::WHEEL_RADIUS_M;
    right_ms = (right_rpm / 60.0f) * 2.0f * M_PI * config::WHEEL_RADIUS_M;
}

}  // namespace tmr