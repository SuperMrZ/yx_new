#include "PID.h"

PID pid_M3508Friction[4] = {
    {1, 0.01, 0.5, 0x1000, 0x3000, 0x3000, 0,0,0,0,0,0},
    {1, 0.01, 0.5, 0x1000, 0x3000, 0x3000, 0,0,0,0,0,0},
    {1, 0.01, 0.5, 0x1000, 0x3000, 0x3000, 0,0,0,0,0,0},
    {5, 0.01, 0, 0x3000, 0x3000, 0x3000, 0,0,0,0,0,0}
};

PID pid_M3508Friction_angle[4] = {
    {0.5, 0, 0, 0x3000, 0x3000, 0x3000, 0,0,0,0,0,0},
    {10, 0.01, 0, 0x3000, 0x3000, 0x3000, 0,0,0,0,0,0},
    {10, 0.01, 0, 0x3000, 0x3000, 0x3000, 0,0,0,0,0,0},
    {10, 0.01, 0, 0x3000, 0x3000, 0x3000, 0,0,0,0,0},0
};

PID pid_M3508Load_speed=
{6,0,8,0x1000,0x5000,0x3000,0,0,0,0,0,0};

PID pid_M3508Laod_angle=
{1,0,5,700,400,0x3000,0,0,0,0,0,0};

PID pid_M2006Pushrop_speed=
{2,0,0,0x1000,0x5000,0x3000,0,0,0,0,0,0};

PID pid_M2006Pushrop_angle=
{1,0,2,0x1000,0x5000,0x3000,0,0,0,0,0,0};

PID pid_M6020Yaw_speed=
{8,0.2,5,1500,0x3000,0x3000,0,0,0,0,0,0};

PID pid_M6020Yaw_angle=
{3,0,10,0x1000,800,0x3000,0,0,0,0,0,0};

PID pid_D4310Pitch_speed=
{1,0,1,0x1000,0x3000,0x3000,0,0,0,0,0,0};

PID pid_D4310Pitch_angle=
{10,0.2,20,0x1000,0x3000,0x3000,0,0,0,0,0,0};



float pid_output(PID *pid, float feedback, float target) 
{
    int8_t index;//积分分离系数

    // 更新误差
    pid->error_last = pid->error_now;
    pid->error_now = target - feedback;



    // 计算P部分
    float pout = pid->kp * pid->error_now;
	pid->pout = pout;

    // 计算并限制I部分 
    if (pid->iout > pid->maxI) //积分超过限幅时
    {
        
        if(pid->error_now > pid->error_now_max)//积分分离
        {
            index = 0;//误差过大时，舍弃积分
        }
        else
        {
            index = 1;
            if(pid->error_now <0) //误差小于pid->error_now_max时，且是反向积分则积分累加否则不加
            {
                pid ->iout += pid->ki * pid->error_now;
            }
           
        }

        

    }
	else if (pid->iout < - pid->maxI) //积分超过限幅时
    {
        pid->iout =  - pid->maxI;
        if(pid->error_now < -pid->error_now_max)//积分分离
        {
            index = 0;//误差过大时，舍弃积分
        }
        else
        {
            index = 1;
            if(pid->error_now > 0)//误差小于pid->error_now_max时，且是反向积分则积分累加否则不加
            {
                pid->iout += pid->ki * pid->error_now;

            }
            
        }
    }
    else//积分没超限幅时
    {
        if     (pid->error_now >pid->error_now_max)  {index = 0;}//积分分离
        else if(pid->error_now < -pid->error_now_max){index = 0;}//积分分离
        else
        {
            index = 1;
            pid->iout += pid->ki * pid->error_now;
        }

    }

    //     // 计算并限制I部分
    //  pid->iout += (pid->ki * pid->error_now);  
    // if (pid->iout > pid->maxI) {
    //     pid->iout = pid->maxI;
        

    // }
	// else if (pid->iout < - pid->maxI) {
    //     pid->iout =  - pid->maxI;

    // }



    // 计算D部分
    float dout = pid->kd * (pid->error_now - pid->error_last);
	pid->dout = dout;

    // 计算输出并限制
    pid->output = pout + dout + (index * pid->iout);
   if (pid->output > pid->maxO) {
       pid->output = pid->maxO;
   }
   else if(pid->output < -pid->maxO)
	{
        pid->output = - pid->maxO;
    }



    return pid->output;
}

