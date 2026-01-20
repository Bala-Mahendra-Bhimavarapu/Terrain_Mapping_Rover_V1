/**
 * @file main.h
 * @brief PROS main header
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

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
#include <iostream>
#endif

#endif  // _PROS_MAIN_H_