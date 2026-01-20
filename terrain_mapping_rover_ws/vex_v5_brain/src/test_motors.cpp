/**
 * @file test_motors.cpp
 * @brief Motor direction and encoder test utilities
 * 
 * Include this file and call test functions from opcontrol() to verify
 * motor configuration before running the full system. 
 */

#include "main.h"

using namespace tmr;

/**
 * @brief Test that all motors drive the robot FORWARD with positive velocity
 * 
 * Usage: Call from opcontrol() before main loop
 * Expected: Robot moves forward for 2 seconds, all encoder values positive
 */
void testMotorDirections(MotorController& motors) {
    pros::lcd::clear();
    pros::lcd::print(0, "=== MOTOR DIRECTION TEST ===");
    pros::lcd::print(1, "Robot should move FORWARD");
    pros::lcd::print(2, "Starting in 3 seconds...");
    pros::delay(3000);
    
    // Reset encoders
    motors.resetEncoders();
    
    // Drive forward at 0.1 m/s for 2 seconds
    pros::lcd::print(2, "DRIVING FORWARD...");
    motors.setVelocity(0.1f, 0.0f);
    pros::delay(2000);
    
    // Stop
    motors.stop();
    
    // Read encoders
    int32_t lf, rf, lr, rr;
    motors. getEncoderTicks(lf, rf, lr, rr);
    
    pros::lcd::print(3, "LF:  %d  RF: %d", lf, rf);
    pros::lcd::print(4, "LR: %d  RR: %d", lr, rr);
    
    // Check results
    bool all_positive = (lf > 0) && (rf > 0) && (lr > 0) && (rr > 0);
    
    if (all_positive) {
        pros::lcd::print(5, "RESULT:  PASS");
        pros::lcd::print(6, "All encoders positive!");
    } else {
        pros::lcd::print(5, "RESULT: FAIL");
        pros::lcd::print(6, "Check REVERSE_ config!");
        
        // Print which motors need adjustment
        if (lf <= 0) pros::lcd::print(7, "FIX:  LEFT_FRONT");
        if (rf <= 0) pros::lcd::print(7, "FIX: RIGHT_FRONT");
        if (lr <= 0) pros::lcd::print(7, "FIX: LEFT_REAR");
        if (rr <= 0) pros::lcd::print(7, "FIX:  RIGHT_REAR");
    }
    
    pros::delay(5000);
}

/**
 * @brief Test that robot turns correctly
 * 
 * Usage: Call from opcontrol() before main loop
 * Expected: 
 * - Positive angular velocity = robot turns LEFT (CCW when viewed from above)
 * - Left encoders negative, right encoders positive
 */
void testTurning(MotorController& motors) {
    pros::lcd::clear();
    pros::lcd::print(0, "=== TURNING TEST ===");
    pros::lcd::print(1, "Robot should turn LEFT (CCW)");
    pros::lcd::print(2, "Starting in 3 seconds...");
    pros::delay(3000);
    
    // Reset encoders
    motors.resetEncoders();
    
    // Turn left (positive angular velocity)
    pros::lcd::print(2, "TURNING LEFT...");
    motors.setVelocity(0.0f, 0.2f);  // 0.2 rad/s CCW
    pros::delay(2000);
    
    // Stop
    motors.stop();
    
    // Read encoders
    int32_t lf, rf, lr, rr;
    motors.getEncoderTicks(lf, rf, lr, rr);
    
    pros::lcd::print(3, "LF: %d  RF:  %d", lf, rf);
    pros::lcd::print(4, "LR: %d  RR: %d", lr, rr);
    
    // For CCW turn:  left wheels go backward (negative), right wheels go forward (positive)
    bool left_negative = (lf < 0) && (lr < 0);
    bool right_positive = (rf > 0) && (rr > 0);
    
    if (left_negative && right_positive) {
        pros::lcd::print(5, "RESULT: PASS");
        pros::lcd::print(6, "Turning direction correct!");
    } else {
        pros::lcd::print(5, "RESULT: FAIL");
        pros::lcd::print(6, "Check motor wiring/config!");
    }
    
    pros::delay(5000);
}

/**
 * @brief Test individual motor by port
 * 
 * Useful for identifying which physical motor is on which port.
 */
void testIndividualMotor(int port, const char* name) {
    pros::lcd::clear();
    pros::lcd::print(0, "=== INDIVIDUAL MOTOR TEST ===");
    pros::lcd::print(1, "Testing:  %s (Port %d)", name, port);
    pros::lcd::print(2, "Motor should spin FORWARD");
    pros::lcd::print(3, "Starting in 2 seconds...");
    pros::delay(2000);
    
    // Create temporary motor (no reversal to see raw direction)
    pros::Motor test_motor(port, pros::E_MOTOR_GEARSET_36, false);
    
    // Spin forward
    pros::lcd::print(3, "Spinning at 50 RPM...");
    test_motor.move_velocity(50);
    pros::delay(3000);
    
    // Stop
    test_motor.move_velocity(0);
    
    pros::lcd::print(4, "Position: %. 1f deg", test_motor.get_position());
    pros::lcd::print(5, "Test complete!");
    pros::lcd::print(6, "Note which wheel moved.");
    
    pros::delay(3000);
}

/**
 * @brief Run all motor tests in sequence
 */
void runAllMotorTests(MotorController& motors) {
    pros::lcd::clear();
    pros::lcd::print(0, "=== MOTOR TEST SUITE ===");
    pros::lcd::print(1, "Press A to start tests");
    pros::lcd::print(2, "Press B to skip");
    
    // Wait for button press
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    while (true) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            break;
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            return;
        }
        pros::delay(50);
    }
    
    // Run tests
    testMotorDirections(motors);
    testTurning(motors);
    
    pros::lcd::clear();
    pros::lcd::print(0, "=== TESTS COMPLETE ===");
    pros::lcd::print(1, "Press any button to continue");
    
    while (! master.get_digital(pros:: E_CONTROLLER_DIGITAL_A) &&
           !master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        pros::delay(50);
    }
}
