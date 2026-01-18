/**
 * @file main.h
 * @brief PROS main header file
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#include "api.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

// Function prototypes
#ifdef __cplusplus
extern "C" {
#endif

void initialize();
void disabled();
void competition_initialize();
void autonomous();
void opcontrol();

#ifdef __cplusplus
}
#endif

#endif  // _PROS_MAIN_H_
