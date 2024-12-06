#include "tasks.h"

 
void pushrop_Move(int32_t* pushrot_M2006_positionTarget);

extern SBUS_Buffer SBUS;


void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
   // pushrop_Move(pushrot_M2006_positionTarget);
    cmd_M3508Friction_speed(M3508Friction_speedTarget);
    osDelay(1);
    Cmd_gamble3508_currnt();
  

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}






void pushrop_Move(int32_t* pushrot_M2006_positionTarget)
{
  if(*pushrot_M2006_positionTarget>8192)
  {
    cmd_M2006pushrop_speed(300);
    if(M2006Pushrop.last_ecd-M2006Pushrop.ecd > 4096)
    {
      *pushrot_M2006_positionTarget-8192;
    }
    M2006Pushrop.last_ecd = M2006Pushrop.ecd;

  }
  if(*pushrot_M2006_positionTarget<8192)
  {
    cmd_M2006pushrop_angle(*pushrot_M2006_positionTarget);
  }
}