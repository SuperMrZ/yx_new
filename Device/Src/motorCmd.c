#include "motorCmd.h"

int16_t M3508Friction_currnt[4];
int16_t M3508Load_currnt;
int16_t M2006Pushrop_currnt;
int16_t M6020Yaw_currrnt;

extern SBUS_Buffer SBUS;

void cmd_M3508Friction_speed(int16_t target[3])
{
    int32_t motor_currnt[3];
    for (uint16_t i = 0; i < 3; i++) 
    {	
        motor_currnt[i] = pid_output(&pid_M3508Friction[i],M3508Friction[i].speed_rpm,target[i]); 
        if (motor_currnt[i]>10000)
        {
            motor_currnt[i] =10000;
        }
        if(motor_currnt[i]< -10000)
        {
            motor_currnt[i] =-10000;
        }
        
        M3508Friction_currnt[i] = motor_currnt[i];
    }
    //CAN_SendData(1,0x200,motor_currnt);


}

void cmd_M3508Friction_angle(int16_t target[3])
{
    int16_t speed[3];
    int16_t cur;

    for (uint16_t i = 0; i < 3; i++) 
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


void cmd_M3508Laod_speed(int16_t target)
{
    int16_t motor_currnt;
   	
        motor_currnt = pid_output(&pid_M3508Load_speed,M3508Load.speed_rpm,target); 
        M3508Friction_currnt[3] = motor_currnt;
        // M3508Friction[3] = motor_currnt;
  
    //CAN_SendData(1,0x200,motor_currnt);


}

void cmd_M3508Load_angle(int16_t target)
{
    int16_t speed;
    int16_t cur;

 	
        cur=M3508Load.ecd;
        if(target-cur>4096)
        {
            cur +=8192;
        }
        else if(target-cur<=-4096)
        {
            cur =cur-8192;
        }
        speed = pid_output(&pid_M3508Laod_angle,cur,target); 
    
    cmd_M3508Laod_speed(speed);

}



void cmd_M2006pushrop_speed(int16_t target)
{
    int16_t motor_currnt;
    	
        motor_currnt = pid_output(&pid_M2006Pushrop_speed,M2006Pushrop.speed_rpm,target);

        

        M2006Pushrop_currnt = motor_currnt;
    
    

}

void cmd_M2006pushrop_angle(int16_t target)
{
    int16_t speed;
    int16_t cur;

    
        cur=M2006Pushrop.ecd;
        if(target-cur>4096)
        {
            cur +=8192;
        }
        else if(target-cur<=-4096)
        {
            cur =cur-8192;
        }
        speed = pid_output(&pid_M2006Pushrop_angle,cur,target); 
    
    cmd_M2006pushrop_speed(speed);

}


void cmd_M6020Yaw_speed(int16_t target)
{
    int32_t motor_currnt;
	
        // motor_currnt =  pid_output(&pid_M6020Yaw_speed,M6020Yaw.speed_rpm,target); 
        motor_currnt = target*0.2f + pid_output(&pid_M6020Yaw_speed,M6020Yaw.speed_rpm,target); 

        // if (motor_currnt>20000)
        // {
        //     motor_currnt =20000;
        // }
        // if(motor_currnt< -20000)
        // {
        //     motor_currnt =-20000;
        // }
        
        M6020Yaw_currrnt = motor_currnt;
    
    //CAN_SendData(1,0x200,motor_currnt);


}
float yaw_positiontarget_last;
float test2;

void cmd_M6020Yaw_angle(int16_t target)
{
    int16_t speed;
    int16_t cur;
    int16_t target2;
    target2 = target; 

    
        cur=INS.Yaw;
        if(target-cur>180)
        {
            cur +=360;
        }
        else if(target-cur<=-180)
        {
            cur =cur-360;
        }

         if(target2-yaw_positiontarget_last>180)
        {
            target2 =target2-360;
            
        }
        else if(target2-yaw_positiontarget_last<=-180)
        {
            target2 +=360;
        }

        test2 = (float)(target2 - yaw_positiontarget_last)*1.0f;

        speed =  (target2 - yaw_positiontarget_last)*1.0f+ pid_output(&pid_M6020Yaw_angle,cur,target); 
        // speed = pidOutputDiffFirst(&pid_DiffFirst_M6020Yaw_angle,cur,target);
        yaw_positiontarget_last =target;
    
    cmd_M6020Yaw_speed(speed);

}







void Cmd_gamble3508_currnt(void)
{
    int16_t currnt_target[4];
    currnt_target[0] = M3508Friction_currnt[0];
    currnt_target[1] = M3508Friction_currnt[1];
    currnt_target[2] = M3508Friction_currnt[2];
    currnt_target[3] = M3508Friction_currnt[3];
    CAN_SendData(1,0x200,currnt_target);
    CAN_SendData(2,0x200,currnt_target); 
} 

void Cmd_gamble2006_currnt(void)
{
    int16_t currnt_target[4];
    currnt_target[0] = M2006Pushrop_currnt;
    currnt_target[1] = 0;
    currnt_target[2] = 0;
    currnt_target[3] = 0;    

    CAN_SendData(1,0x1FF,currnt_target);

}


void Cmd_gamble6020_currnt(void)
{
    int16_t currnt_target[4];
    currnt_target[0] = M6020Yaw_currrnt;
    currnt_target[1] = M6020Yaw_currrnt;
    // currnt_target[1] = -1000;
    currnt_target[2] = M6020Yaw_currrnt;
    currnt_target[3] = M6020Yaw_currrnt;

    CAN_SendData(2,0x2FE,currnt_target);  
}





