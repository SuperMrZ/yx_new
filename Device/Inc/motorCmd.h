#include "main.h"
#include "bsp_can.h"
#include "PID.h"
#ifndef _MOTOR_CMD_H
#define _MOTOR_CMD_H


extern PID pid_M3508Friction[4];
extern PID pid_M3508Friction_angle[4];
extern motorReceiveInfo M3508Friction[4];

void cmd_M3508Friction_speed(int16_t target[4]);
void cmd_M3508Friction_angle(int16_t target[4]);




#endif
