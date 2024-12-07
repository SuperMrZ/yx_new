#include "tasks.h"

 
void pushrop_Move();
void M3508Load_Move();

extern SBUS_Buffer SBUS;


void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    // pushrop_Move(pushrot_M2006_positionTarget);
     osDelay(1);

     M3508Load_Move(Load_M3508_positionTarget);
    
    cmd_M3508Friction_speed(M3508Friction_speedTarget);
    cmd_M2006pushrop_speed(Load_M3508_speedTarget);

    

    
    
  

    osDelay(1);   
  }
  /* USER CODE END StartDefaultTask */
}

void Sendmessage(void *argument)
{
  while(1)
  {
    Cmd_gamble2006_currnt();
    Cmd_gamble3508_currnt();

    osDelay(1);
  }
}







void pushrop_Move()
{
  if(pushrot_M2006_positionTarget>8192)
  {
    cmd_M2006pushrop_speed(2000);
    if(M2006Pushrop.last_ecd-M2006Pushrop.ecd > 4096)
    {
      pushrot_M2006_positionTarget = pushrot_M2006_positionTarget - 8192;
    }
    M2006Pushrop.last_ecd = M2006Pushrop.ecd;

  }
    if(pushrot_M2006_positionTarget<0)
  {
    cmd_M2006pushrop_speed(-2000);
    if(M2006Pushrop.ecd-M2006Pushrop.last_ecd > 4096)
    {
      pushrot_M2006_positionTarget = pushrot_M2006_positionTarget + 8192;
    }
    M2006Pushrop.last_ecd = M2006Pushrop.ecd;

  }


  if(pushrot_M2006_positionTarget<8192 && pushrot_M2006_positionTarget>0)
  {
    cmd_M2006pushrop_angle(pushrot_M2006_positionTarget);
  }
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