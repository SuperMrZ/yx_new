#include "main.h"
#include "bsp_can.h"
#include "PID.h"
#include "bsp_uart.h"
#ifndef _MOTOR_CMD_H
#define _MOTOR_CMD_H


extern int16_t M3508Friction_currnt[4];
extern int16_t M3508Load_currnt;
extern int16_t M2006Pushrop_currnt;
extern int16_t M6020Yaw_currrnt;


extern PID pid_M3508Friction[4];
extern PID pid_M3508Friction_angle[4];
extern motorReceiveInfo M3508Friction[3];
extern motorReceiveInfo M3508Load;
extern motorReceiveInfo M2006Pushrop;
extern motorReceiveInfo M6020Yaw;

void cmd_M3508Friction_speed(int16_t target[3]);
void cmd_M3508Friction_angle(int16_t target[3]);
void cmd_M3508Laod_speed(int16_t target);
void cmd_M3508Load_angle(int16_t target);
void cmd_M2006pushrop_speed(int16_t target);
void cmd_M2006pushrop_angle(int16_t target);
void cmd_M6020Yaw_speed(int16_t target);
void cmd_M6020Yaw_angle(int16_t target);


void Cmd_gamble3508_currnt(void);
void Cmd_gamble2006_currnt(void);
void Cmd_gamble6020_currnt(void);





#endif
