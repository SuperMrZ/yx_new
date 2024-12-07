#ifndef _PID_H
#define _PID_H
#include "main.h"



typedef struct
{
//p,i,d参数值,maxI积分限幅，maxO输出限幅
float kp;
float ki;
float kd;
int16_t maxI;  //maxI积分限幅
int16_t maxO;  //maxO输出限幅
int16_t error_now;
int16_t error_last;
int32_t pout ;
int16_t iout;
int32_t dout;
int32_t output;

} PID;




extern PID pid_M3508Friction[4];
extern PID pid_M3508Friction_angle[4];
extern PID pid_M3508Load_speed;
extern PID pid_M3508Laod_angle;
extern PID pid_M2006Pushrop_speed;
extern PID pid_M2006Pushrop_angle;

#endif
