#include "PID.h"

PID pid_M3508Friction[4] = {
    {10, 0.01, 0, 0x3000, 0x3000, 0, 0,0,0,0,0},
    {10, 0.01, 0, 0x3000, 0x3000, 0, 0,0,0,0,0},
    {10, 0.01, 0, 0x3000, 0x3000, 0, 0,0,0,0,0},
    {10, 0.01, 0, 0x3000, 0x3000, 0, 0,0,0,0,0}
};

PID pid_M3508Friction_angle[4] = {
    {0.3, 0.01, 0.3, 0x3000, 0x3000, 0, 0,0,0,0,0},
    {10, 0.01, 0, 0x3000, 0x3000, 0, 0,0,0,0,0},
    {10, 0.01, 0, 0x3000, 0x3000, 0, 0,0,0,0,0},
    {10, 0.01, 0, 0x3000, 0x3000, 0, 0,0,0,0,0}
};

int32_t pid_output(PID *pid, int16_t feedback, int16_t target) 
{
    // 更新误差
    pid->error_last = pid->error_now;
    pid->error_now = target - feedback;

    int16_t a = pid->error_now;
    int16_t b = pid->error_last;

    // 计算P部分
    int16_t pout = pid->kp * pid->error_now;
	pid->pout = pout;

    // 计算并限制I部分
    pid->iout += (pid->ki * pid->error_now);
    int16_t c = pid->iout;        
    if (pid->iout > pid->maxI) {
        pid->iout = pid->maxI;
    }
	else if (pid->iout < - pid->maxI) {
        pid->iout =  - pid->maxI;
    }


    // 计算D部分
    int16_t dout = pid->kd * (pid->error_now - pid->error_last);
	pid->dout = dout;

    // 计算输出并限制
 pid->output = pout - dout + pid->iout;
   if (pid->output > pid->maxO) {
       pid->output = pid->maxO;
   }
   else if(pid->output < -pid->maxO)
	pid->output = pid->maxO;

    return pid->output;
}
