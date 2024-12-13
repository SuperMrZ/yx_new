#include "tasks.h"

 


void M3508Load_Move();

extern SBUS_Buffer SBUS;

/*第一个任务，用于计算数据*/
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

  for(;;)
  {

    /*imu解算开始*/

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

      cmd_M3508Friction_speed(M3508Friction_speedTarget);

      cmd_M6020Yaw_angle(Yaw6020_positiontarget);
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
    
   // osDelay(1);
    //ctrl_position_damiao_motor(0x01,YAW_D4310_positiontarget);
    
    osDelay(1);
    Cmd_gamble6020_currnt();
    osDelay(1);
  }
}


void StartINSTask(void *argument)
{
  /* USER CODE BEGIN StartINSTask */
   INS_Init();
  /* Infinite loop */
  for(;;)
  {
    INS_Task();
    osDelay(1);
  }
  /* USER CODE END StartINSTask */
}

void M3508Load_Move()
{
  if(Load_M3508_positionTarget>8192)
  {
    cmd_M3508Laod_speed(500);
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
    cmd_M2006pushrop_speed(3000);
    if(M2006Pushrop.last_ecd-M2006Pushrop.ecd>4096)
    {
      pushrot_M2006_positionTarget -= 8192;
    }
    M2006Pushrop.last_ecd = M2006Pushrop.ecd;
  }
  if(pushrot_M2006_positionTarget < 0)
  {
    cmd_M2006pushrop_speed(-3000);
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