#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "main.h"
#include <stdlib.h>
#include <math.h>
#include "odrivestm32_enums.h"
#include "odrivestm32_params.h"
 extern UART_HandleTypeDef huart4;


 /*------------working odrive libraries for stm------------------*/
void SetPosition(UART_HandleTypeDef *uart_handler, int motor_number, float position);
//note: setposition will call set position with current velocity, the set position with velocity is not working
//but it is not used in the original code so that should not be a problem
void SetPositionWithCurrentVelocity(UART_HandleTypeDef *uart_handler, int motor_number, float position, float velocity_feedforward, float current_feedforward);
void SetVelocity(UART_HandleTypeDef *uart_handler, int motor_number, float velocity);
float GetVelocity(UART_HandleTypeDef *uart_handler,int motor_number);
double GetPosition(UART_HandleTypeDef *uart_handler,int motor_number);
void run_state(UART_HandleTypeDef *uart_handler,int axis, int requested_state, bool wait_for_idle, float timeout);
void ClearErrors(UART_HandleTypeDef *uart_handler);
void Setup_Cur_Lim(UART_HandleTypeDef *uart_handler, double cur_lim);
void Setup_cpr(UART_HandleTypeDef *uart_handler, double cpr);
void Setup_vel_limit(UART_HandleTypeDef *uart_handler, double RPS_limit);
/*--------end working odrive libraries for stm--------------*/


int position_state = 0;
double GetVelocity_custom(UART_HandleTypeDef *uart_handler, int motor_number, double wheel_diameter);
void SetPositionWithVelocity(UART_HandleTypeDef *uart_handler, int motor_number, float position, float velocity_feedforward);
void SetPositionWithCurrentVelocity(UART_HandleTypeDef *uart_handler, int motor_number, float position, float velocity_feedforward, float current_feedforward);
void SetVelocityWithCurrent(UART_HandleTypeDef *uart_handler, int motor_number, float velocity, float current_feedforward);
//void SetCurrent(UART_HandleTypeDef *uart_handler, int motor_number, float current);
//not implemented in the original code
//void TrapezoidalMove(int motor_number, float position) ;
//not implemented in the original code
double readFloat(UART_HandleTypeDef *uart_handler);
int32_t readInt(UART_HandleTypeDef *uart_handler);
void readString(UART_HandleTypeDef *uart_handler,char* buf, uint16_t len, int timeout);
//double valuefilter(double input, int position_state);
void set_tuning_parameters(UART_HandleTypeDef *uart_handler, int motor_number, float pos_gain_value, float vel_gain_value, float vel_integrator_gain_value);
//void RunCalibrationSequence(UART_HandleTypeDef *uart_handler, int motor_number);


