#include "tasks.h"

 int16_t down_MEG[4];
uint16_t down_MEG2[4];

void M3508Load_Move();

extern SBUS_Buffer SBUS;
extern INS_t INS;

/*第一个任务，用于计算数据*/
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  INS_Init();

  for(;;)
  {

    /*imu解算开始*/
    INS_Task();

    /*imu解算结束*/

    /*电机电流控制计算开始*/

    if(SBUS.SF == 1695)
    {
      if(SBUS.SG ==353 && back_flag ==1)
      {
        cmd_M2006pushrop_speed(pushrot_M2006_speedTarget);

      }
      if(SBUS.SG ==353 && back_flag ==0)
      {
        M2006Pushrop_currnt =0;

      }
      if(SBUS.SG !=353)
      {
        M2006PushRop_Move();
      }
      
      M3508Load_Move();
      //  cmd_M3508Laod_speed(5*(SBUS.Ch1 - 1024));



      cmd_M3508Friction_speed(M3508Friction_speedTarget);

        cmd_M6020Yaw_angle(Yaw6020_positiontarget);
      // cmd_M6020Yaw_speed(SBUS.Ch1-1024);


    }

   /*电机电流控制计算结束*/


    osDelay(1);   
  }
  /* USER CODE END StartDefaultTask */
}


/*第二个任务，用于发送数据*/
void Sendmessage(void *argument)
{
  while(1)
  {
   Cmd_gamble2006_currnt();
   Cmd_gamble3508_currnt();
   Down_SendMEG();
   Down_SendMEG2();
    
   ctrl_position_damiao_motor(0x01,YAW_D4310_positiontarget);
    
    
    Cmd_gamble6020_currnt();
    osDelay(1);
  }
}



void M3508Load_Move()
{
  if(Load_M3508_positionTarget>8192)
  {
    cmd_M3508Laod_speed(1000);
    if(M3508Load.last_ecd-M3508Load.ecd > 4096)
    {
      Load_M3508_positionTarget=Load_M3508_positionTarget-8192;
    }
    M3508Load.last_ecd = M3508Load.ecd;
  }

  if(Load_M3508_positionTarget<8192)
  {
    cmd_M3508Load_angle(Load_M3508_positionTarget);
  }
}


void M2006PushRop_Move()
{
  if(pushrot_M2006_positionTarget > 8192)
  {
    cmd_M2006pushrop_speed(4000);
    if(M2006Pushrop.last_ecd-M2006Pushrop.ecd>4096)
    {
      pushrot_M2006_positionTarget -= 8192;
    }
    M2006Pushrop.last_ecd = M2006Pushrop.ecd;
  }
  if(pushrot_M2006_positionTarget < 0)
  {
    cmd_M2006pushrop_speed(-4000);
    if(M2006Pushrop.ecd - M2006Pushrop.last_ecd>4096)
    {
      pushrot_M2006_positionTarget += 8192;
    }
    M2006Pushrop.last_ecd = M2006Pushrop.ecd;
  } 

  if(0 < pushrot_M2006_positionTarget && pushrot_M2006_positionTarget <8192)
  {
    cmd_M2006pushrop_angle(pushrot_M2006_positionTarget);
  }
}


void Down_SendMEG()
{
  down_MEG[0]=SBUS.Ch3;
  down_MEG[1]=SBUS.Ch4;
  down_MEG[2]=SBUS.Ch1;
  down_MEG[3]=SBUS.SA;
  CAN_SendData(2,0x123,down_MEG);
} 


uint32_t temp;


void Down_SendMEG2()
{
   temp = *(uint32_t*)&INS.Yaw;

  //  temp = &INS.Yaw;
  down_MEG2[0] = (uint16_t)(temp >> 16);  // 获取高16位
  down_MEG2[1] = (uint16_t)(temp & 0xFFFF);  // 获取低16位
  down_MEG2[2] = 0;
  down_MEG2[3] = 0;
  
  CAN_SendData(2,0x124,down_MEG2);

}