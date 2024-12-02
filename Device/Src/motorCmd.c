#include "motorCmd.h"




void cmd_M3508Friction_speed(int16_t target[4])
{
    int16_t motor_currnt[4];
    for (uint16_t i = 0; i < 4; i++) 
    {	
        motor_currnt[i] = pid_output(&pid_M3508Friction[i],M3508Friction[i].speed_rpm,target[i]); 
    }
    CAN_SendData(1,0x200,motor_currnt);

}

void cmd_M3508Friction_angle(int16_t target[4])
{
    int16_t speed[4];
    int16_t cur;

    for (uint16_t i = 0; i < 4; i++) 
    {	
        cur=M3508Friction[i].ecd;
        if(target[i]-cur>4096)
        {
            cur +=8192;
        }
        else if(target[i]-cur<=-4096)
        {
            cur =cur-8192;
        }
        speed[i] = pid_output(&pid_M3508Friction_angle[i],cur,target[i]); 
    }
    cmd_M3508Friction_speed(speed);

}



