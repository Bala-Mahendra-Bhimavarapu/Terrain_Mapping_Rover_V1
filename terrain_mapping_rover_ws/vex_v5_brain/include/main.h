/**
 * @file main.h
 * @brief PROS main header for TMR VEX V5 Brain
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"

#ifdef __cplusplus
extern "C" {
#endif

void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include "config.hpp"
#include "serial_protocol.hpp"
#include "motor_controller.hpp"
#include "odometry.hpp"
#include "safety.hpp"
#endif

#endif  // _PROS_MAIN_H_
