#ifndef _PID_H
#define _PID_H
#include "main.h"



typedef struct
{
//p,i,d参数值,maxI积分限幅，maxO输出限幅
float kp;
float ki;
float kd;
float maxI;  //maxI积分限幅
float maxO;  //maxO输出限幅
float error_now_max; //积分分离点
float error_now;
float error_last;
float pout ;
float iout;
float dout;
float output;

} PID;

float pid_output(PID *pid, float feedback, float target) ;


extern PID pid_M3508Friction[4];
extern PID pid_M3508Friction_angle[4];
extern PID pid_M3508Load_speed;
extern PID pid_M3508Laod_angle;
extern PID pid_M2006Pushrop_speed;
extern PID pid_M2006Pushrop_angle;
extern PID pid_D4310Pitch_speed;
extern PID pid_D4310Pitch_angle;
extern PID pid_M6020Yaw_speed;
extern PID pid_M6020Yaw_angle;



#endif


