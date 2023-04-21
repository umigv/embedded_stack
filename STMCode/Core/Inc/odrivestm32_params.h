/*
 * odrivestm32_params.h
 *
 *  Created on: Mar 24, 2023
 *      Author: xuzhekai
 */

#ifndef INC_ODRIVESTM32_PARAMS_H_
#define INC_ODRIVESTM32_PARAMS_H_

int eStopMultiplier = 1;

bool is_autonomous = false;
bool mode_change = true;
float left_vel = 20;
float right_vel = 20;
uint8_t wireless_stop = 0;
unsigned long lastData = 0;
const float WHEEL_BASE = 0.62;
const float WHEEL_DIAMETER = 0.3;
const long CONTROL_TIMEOUT = 1000;
const int8_t LEFT_POLARITY = -1;
const int8_t RIGHT_POLARITY = 1;
//float VEL_TO_RPS = 1.0 / (WHEEL_DIAMETER * PI) * 98.0/3.0;
const float RPS_LIMIT = 20;
//const float  VEL_LIMIT = RPS_LIMIT / VEL_TO_RPS; // 1.2 mph (~0.57 m/s) limit
const double PI = 3.14;
const double VEL_TO_RPS = 1.0 / (WHEEL_DIAMETER * PI) * 98.0 / 3.0;


#endif /* INC_ODRIVESTM32_PARAMS_H_ */
