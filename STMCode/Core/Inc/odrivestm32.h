#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "main.h"
#include <stdlib.h>
// extern UART_HandleTypeDef huart4;
void SetDefaultPosition(int motor_number, float position);
void SetPositionWithVelocity(int motor_number, float position, float velocity_feedforward);
void SetPositionWithCurrentVelocity(int motor_number, float position, float velocity_feedforward, float current_feedforward);
void SetVelocity(int motor_number, float velocity);
void SetVelocityWithCurrent(int motor_number, float velocity, float current_feedforward);
void SetCurrent(int motor_number, float current);
void TrapezoidalMove(int motor_number, float position) ;
double readFloat();
float GetVelocity(int motor_number);
double GetPosition(int motor_number);
int32_t readInt();
bool run_state(int axis, int requested_state, bool wait_for_idle, float timeout);
void readString(char* buf, uint16_t len, int timeout);

